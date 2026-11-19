"""The live coverage session: telemetry and tags in, labeled motions and plant coverage out."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, NamedTuple

import numpy as np

from auto_battlebot.calibration.sysid import model
from auto_battlebot.calibration.sysid.maneuvers import (
    AMPS,
    BINS,
    EDGE_ROWS,
    Event,
    ManeuverSegmenter,
    bin_of,
)
from auto_battlebot.calibration.sysid.pose_stream import PoseStream, TagPose, state_at
from auto_battlebot.calibration.sysid.profile import SysidProfile
from auto_battlebot.calibration.sysid.telemetry import FLIP_SWITCH_DOWN, LiveClock, sticks
from auto_battlebot.calibration.sysid.worker import RefitEngine, RefitJob, RefitReply
from auto_battlebot.control.firmware_mixer import motor_to_plant
from auto_battlebot.control.plant import MODEL_LADDER
from auto_battlebot.recording.sysid_io import RobotTagsFrame

_NS = 1_000_000_000
DT_NS = int(round(model.DT * _NS))
LAG_S = 0.5
GAP_S = 0.05
TAG_COVERAGE_MIN = 0.7
PITCH_LIMIT_DEG = 4.0
PITCH_MEDIAN_N = 7
ROBOT_LENGTH_M = 0.15
IDLE_CMD = 0.02
CONTACT_SIGMA = 4.0
HOLDOUT_EVERY = 5
REFIT_S = 4.0
GHOST_EVERY_S = 0.24
GHOST_KEEP_S = 6.0
KEEP_S = 12.0
TELEMETRY_PERIOD_MS = 10


class WindowStatus(str, Enum):
    USED = "used"
    HOLDOUT = "holdout"
    IDLE = "idle"
    JETSON = "jetson"
    LINK = "link"
    TAGS = "tags"
    FLIP = "flip"
    WALL = "wall"
    DISARMED = "disarmed"


FIT_STATUSES = (WindowStatus.USED, WindowStatus.HOLDOUT)


class SeqGap(NamedTuple):
    robot_ms_before: int
    robot_ms_after: int


class RateMark(NamedTuple):
    host_ns: int
    telemetry_count: int
    frame_count: int


@dataclass
class GridRow:
    t_ns: int
    a: float
    b: float
    auto: bool
    armed: bool
    jetson: bool
    upside_down: bool
    left: float
    right: float
    gyro_cw_dps: float
    heading_deg: float
    pid_setpoint: float
    pid_output: float
    yaw_rate_cmd: float
    gap: bool
    vbat: float


@dataclass
class WindowRecord:
    id: int
    t0_session_s: float
    t1_session_s: float
    status: WindowStatus
    reason: str = ""
    subs: list[model.SubWindow] = field(default_factory=list)


@dataclass
class _Telemetry:
    robot_ms: int
    host_ns: int
    row: dict[str, Any]


def _f(value: Any) -> float:
    return math.nan if value is None else float(value)


def _pitched(poses: list[TagPose]) -> bool:
    """Whether the running median of pitch leaves ``PITCH_LIMIT_DEG`` anywhere in ``poses``."""
    pitch = np.abs([p.pitch for p in poses])
    if len(pitch) == 0:
        return False
    if len(pitch) < PITCH_MEDIAN_N:
        return bool(np.median(pitch) > math.radians(PITCH_LIMIT_DEG))
    runs = np.lib.stride_tricks.sliding_window_view(pitch, PITCH_MEDIAN_N)
    return bool(np.median(runs, axis=1).max() > math.radians(PITCH_LIMIT_DEG))


def straight_run_room_m(diag: float) -> float:
    """Room for a straight run: the diagonal less a robot length and a stop at each end."""
    return max(diag - 0.6, 0.0)


class CoverageSession:
    def __init__(self, profile: SysidProfile, engine: RefitEngine) -> None:
        self.profile = profile
        self.engine = engine
        self.clock = LiveClock()
        self.poses = PoseStream(profile.mounts, profile.rest_pitch_rad)
        self.segmenter = ManeuverSegmenter(model.DT)
        self.half_extent = profile.half_extent_m
        diag = math.hypot(profile.field_size_x_m, profile.field_size_y_m)
        self.floor_run_m = straight_run_room_m(diag)

        self._tele: deque[_Telemetry] = deque()
        self._tele_ptr = 0
        self._last_seq: int | None = None
        self._seq_gaps: deque[SeqGap] = deque()
        self.seq_drops = 0
        self.grid: deque[GridRow] = deque()
        self.t_origin_ns: int | None = None
        self._next_grid_ns: int | None = None
        self._window_start_ns: int | None = None
        self.window_rows = int(round(model.WINDOW_S / model.DT))

        self.windows: list[WindowRecord] = []
        self.usable_count = 0
        self.theta = model.PRIOR_VECTOR.copy()
        self.ladder = {s.name: model.PRIOR_VECTOR.copy() for s in MODEL_LADDER}
        self.fisher_at_last_refit = np.zeros((model.NP, model.NP))
        self.fisher_since_refit = np.zeros((model.NP, model.NP))
        self.scale = 1.0
        self.refits = 0
        self._last_refit_window_id = -1
        self._sent_upto = -1
        self._last_refit_t = 0.0
        self._pending_refit = False
        self.candidates: list[dict[str, Any]] = []
        self.floor_limited = np.zeros(model.NP, dtype=bool)
        self.history: list[dict[str, Any]] = []
        self.ladder_holdout: dict[str, float] = {}
        self.residual = math.nan

        self.events: list[dict[str, Any]] = []
        self._pending_events: list[Event] = []
        self.coverage_count = np.zeros((len(BINS), len(BINS)))
        self.coverage_held = np.zeros((len(BINS), len(BINS)))
        self.edges = np.zeros((len(EDGE_ROWS), 2 * len(AMPS)), dtype=int)
        self.ghosts: deque[dict[str, Any]] = deque()
        self._ghost_next_ns: int | None = None
        self.driving_s = 0.0
        self.jetson_seen = False
        self._frames_seen = 0
        self._tele_seen = 0
        self._rate_marks: deque[RateMark] = deque()
        self._push_history(0.0, math.nan, {})

    def feed_telemetry(self, row: dict[str, Any], log_time_ns: int) -> None:
        robot_ms = int(row["timestamp_ms"])
        host_ns = int(row.get("host_receive_ns") or log_time_ns)
        if self._tele and robot_ms <= self._tele[-1].robot_ms:
            if robot_ms < self._tele[-1].robot_ms - 1000:
                self._tele.clear()
                self._tele_ptr = 0
                self._last_seq = None
            else:
                return
        seq = row.get("telemetry_seq")
        if seq is not None and self._last_seq is not None and int(seq) != self._last_seq + 1:
            lost = max(int(seq) - self._last_seq - 1, 0)
            self.seq_drops += lost
            if lost * TELEMETRY_PERIOD_MS > GAP_S * 1000:
                prev = self._tele[-1].robot_ms if self._tele else robot_ms
                self._seq_gaps.append(SeqGap(prev, robot_ms))
        if seq is not None:
            self._last_seq = int(seq)
        if not row.get("backfill") or self.clock.empty or self.clock.rebooted(robot_ms):
            self.clock.push(robot_ms, host_ns)
        self._tele.append(_Telemetry(robot_ms, host_ns, row))
        self._tele_seen += 1

    def feed_field(self, t_field_camera: np.ndarray) -> None:
        self.poses.set_field(t_field_camera)

    def feed_tags(self, frame: RobotTagsFrame) -> None:
        self.poses.push_frame(frame)
        self._frames_seen += 1

    def _session_s(self, t_ns: int) -> float:
        return 0.0 if self.t_origin_ns is None else (t_ns - self.t_origin_ns) / _NS

    def _horizon_ns(self) -> int | None:
        if not self._tele:
            return None
        newest = int(self.clock.to_host_ns(self._tele[-1].robot_ms))
        if self.poses.has_field and self.poses.newest_frame_ns:
            newest = min(newest, self.poses.newest_frame_ns)
        return newest - int(LAG_S * _NS)

    def advance(self) -> None:
        """Process every grid sample older than ``LAG_S``, close windows, start refits."""
        self.poses.finalize()
        horizon = self._horizon_ns()
        if horizon is None:
            return
        if self._next_grid_ns is None:
            first = int(self.clock.to_host_ns(self._tele[0].robot_ms))
            self._next_grid_ns = first - first % DT_NS + DT_NS
            self.t_origin_ns = self._next_grid_ns
            self._window_start_ns = self._next_grid_ns
            self._ghost_next_ns = self._next_grid_ns + HIST_NS
        while self._next_grid_ns <= horizon:
            row = self._sample(self._next_grid_ns)
            if row is None:
                break
            self._push_row(row)
            self._next_grid_ns += DT_NS
            if (
                self._window_start_ns is not None
                and self._next_grid_ns - self._window_start_ns >= self.window_rows * DT_NS
            ):
                self._close_window(self._window_start_ns, self._next_grid_ns)
                self._window_start_ns = self._next_grid_ns
        self._make_ghosts()
        self._trim()
        self._poll_refit()

    def _sample(self, t_ns: int) -> GridRow | None:
        """Telemetry held to ``t_ns``: the newest frame at or before it."""
        tele = self._tele
        while self._tele_ptr + 1 < len(tele) and (
            int(self.clock.to_host_ns(tele[self._tele_ptr + 1].robot_ms)) <= t_ns
        ):
            self._tele_ptr += 1
        if not tele:
            return None
        cur = tele[self._tele_ptr]
        cur_ns = int(self.clock.to_host_ns(cur.robot_ms))
        gap = cur_ns > t_ns
        if self._tele_ptr + 1 < len(tele):
            nxt = tele[self._tele_ptr + 1]
            gap = gap or (nxt.robot_ms - cur.robot_ms) / 1000.0 > GAP_S
            gap = gap or any(a <= cur.robot_ms < b for a, b in self._seq_gaps)
        else:
            gap = True
        r = cur.row
        gyro = _f(r.get("yaw_rate"))
        if not gap and cur_ns <= t_ns:
            nxt = tele[self._tele_ptr + 1]
            nxt_ns = int(self.clock.to_host_ns(nxt.robot_ms))
            frac = (t_ns - cur_ns) / max(nxt_ns - cur_ns, 1)
            gyro += frac * (_f(nxt.row.get("yaw_rate")) - gyro)
        return GridRow(
            t_ns=t_ns,
            a=_f(r.get("a_percent")),
            b=_f(r.get("b_percent")),
            auto=int(r.get("flip_switch") or 0) == FLIP_SWITCH_DOWN,
            armed=bool(r.get("armed")),
            jetson=bool(r.get("espnow_active")) or bool(r.get("autonomy_enabled")),
            upside_down=bool(r.get("is_upside_down")),
            left=_f(r.get("left_cmd")),
            right=_f(r.get("right_cmd")),
            gyro_cw_dps=gyro,
            heading_deg=_f(r.get("orientation_x")),
            pid_setpoint=_f(r.get("pid_setpoint")),
            pid_output=_f(r.get("pid_output")),
            yaw_rate_cmd=_f(r.get("yaw_rate_cmd")),
            gap=gap,
            vbat=_f(r.get("vbat")),
        )

    def _push_row(self, row: GridRow) -> None:
        self.grid.append(row)
        thr, st = sticks(np.nan_to_num(row.a), np.nan_to_num(row.b))
        moving = row.armed and (abs(float(thr)) > IDLE_CMD or abs(float(st)) > IDLE_CMD)
        if moving:
            self.driving_s += model.DT
        if row.jetson:
            self.jetson_seen = True
        for ev in self.segmenter.push(float(thr), float(st), row.auto):
            self._pending_events.append(ev)

    def _rows_between(self, t0_ns: int, t1_ns: int) -> list[GridRow]:
        return [r for r in self.grid if t0_ns <= r.t_ns < t1_ns]

    def _poses_between(self, t0_ns: int, t1_ns: int) -> list[TagPose]:
        return [p for p in self.poses.poses if t0_ns <= p.stamp_ns <= t1_ns]

    def _gate(
        self, rows: list[GridRow], poses: list[TagPose], t0_ns: int, t1_ns: int
    ) -> WindowStatus | None:
        if any(r.jetson for r in rows):
            return WindowStatus.JETSON
        if any(r.gap for r in rows):
            return WindowStatus.LINK
        frames = sum(1 for s in self.poses.frame_stamps if t0_ns <= s < t1_ns)
        if frames == 0 or len(poses) / frames < TAG_COVERAGE_MIN:
            return WindowStatus.TAGS
        if any(r.upside_down for r in rows) or any(p.upside_down for p in poses) or _pitched(poses):
            return WindowStatus.FLIP
        limit = self.half_extent - ROBOT_LENGTH_M
        if any(max(abs(p.x), abs(p.y)) > limit for p in poses):
            return WindowStatus.WALL
        if not all(r.armed for r in rows):
            return WindowStatus.DISARMED
        return None

    def _subwindow(self, start_ns: int) -> model.SubWindow | None:
        hist0 = start_ns - model.HIST_N * DT_NS
        rows = self._rows_between(hist0, start_ns + model.SUB_N * DT_NS)
        if len(rows) != model.HIST_N + model.SUB_N:
            return None
        poses = list(self.poses.poses)
        est = state_at(poses, start_ns)
        if est is None:
            return None
        cur = rows[model.HIST_N]
        if not math.isfinite(cur.gyro_cw_dps):
            return None
        w0 = -math.radians(cur.gyro_cw_dps)
        left = np.nan_to_num(np.array([r.left for r in rows]))
        right = np.nan_to_num(np.array([r.right for r in rows]))
        lin, ang = motor_to_plant(left, right)
        end_ns = start_ns + model.SUB_N * DT_NS
        inside = [p for p in poses if start_ns < p.stamp_ns <= end_ns]
        if len(inside) < 4:
            return None
        if len(inside) > model.MAX_TAG_FIXES_PER_SUB:
            pick = np.linspace(0, len(inside) - 1, model.MAX_TAG_FIXES_PER_SUB).round().astype(int)
            inside = [inside[i] for i in pick]
        tau = np.array([(p.stamp_ns - start_ns) / _NS for p in inside])
        yaw_meas = np.array([p.yaw for p in inside])
        yaw_meas = est.theta + np.unwrap(np.concatenate([[0.0], yaw_meas - est.theta]))[1:]
        body = rows[model.HIST_N :]
        gyro = np.array([r.gyro_cw_dps for r in body])[
            model.GYRO_STRIDE_SAMPLES : model.GYRO_STRIDE_SAMPLES
            * (model.GYRO_N + 1) : model.GYRO_STRIDE_SAMPLES
        ]
        end = state_at(poses, end_ns)
        return model.SubWindow(
            state0=np.array([est.x, est.y, est.theta, est.v, w0]),
            cmd_lin=lin,
            cmd_ang=ang,
            tag_tau_s=tau,
            tag_x=np.array([p.x for p in inside]),
            tag_y=np.array([p.y for p in inside]),
            tag_yaw=yaw_meas,
            tag_sigma_along_m=np.array([p.sigma_along for p in inside]),
            tag_sigma_across_m=np.array([p.sigma_across for p in inside]),
            tag_sigma_yaw_rad=np.array([p.sigma_yaw for p in inside]),
            gyro=-np.radians(gyro),
            gyro_ok=np.isfinite(gyro),
            a_percent=np.nan_to_num(np.array([r.a for r in body])),
            b_percent=np.nan_to_num(np.array([r.b for r in body])),
            auto_steer=np.array([r.auto for r in body]),
            loop_seed={
                "orientation_x": cur.heading_deg,
                "pid_setpoint": cur.pid_setpoint,
                "pid_output": cur.pid_output,
                "yaw_rate_cmd": cur.yaw_rate_cmd,
                "yaw_rate": cur.gyro_cw_dps,
            },
            end_xy=None if end is None else np.array([end.x, end.y]),
        )

    def _close_window(self, t0_ns: int, t1_ns: int) -> None:
        rows = self._rows_between(t0_ns, t1_ns)
        poses = self._poses_between(t0_ns, t1_ns)
        rec = WindowRecord(
            id=len(self.windows),
            t0_session_s=self._session_s(t0_ns),
            t1_session_s=self._session_s(t1_ns),
            status=WindowStatus.IDLE,
        )
        dropped = self._gate(rows, poses, t0_ns, t1_ns)
        if dropped is None:
            subs = [self._subwindow(t0_ns + j * model.SUB_N * DT_NS) for j in range(model.SUBS)]
            rec.subs = [s for s in subs if s is not None]
            cmd = max(
                (
                    max(abs(float(np.max(np.abs(s.cmd_lin)))), float(np.max(np.abs(s.cmd_ang))))
                    for s in rec.subs
                ),
                default=0.0,
            )
            speed = max((abs(s.state0[3]) for s in rec.subs), default=0.0)
            if cmd < IDLE_CMD and speed < 0.05:
                dropped = WindowStatus.IDLE
            elif len(rec.subs) < 2:
                dropped = WindowStatus.TAGS
        if dropped is not None:
            rec.status = dropped
            rec.subs = []
        else:
            self.usable_count += 1
            rec.status = (
                WindowStatus.HOLDOUT
                if self.usable_count % HOLDOUT_EVERY == 0
                else WindowStatus.USED
            )
            if rec.status == WindowStatus.USED:
                batch = model.stack(
                    rec.subs, [rec.id] * len(rec.subs), self.poses.noise.view_azimuth_rad
                )
                _r, jac = model.jacobian(self.theta, batch)
                self.fisher_since_refit += model.information(jac)
            self._add_coverage(rows, poses)
        self.windows.append(rec)
        self._resolve_events()

    def _add_coverage(self, rows: list[GridRow], poses: list[TagPose]) -> None:
        stamps = np.array([p.stamp_ns for p in poses], dtype=np.int64)
        for r in rows:
            k = int(np.searchsorted(stamps, r.t_ns))
            near = any(
                0 <= j < len(stamps) and abs(int(stamps[j]) - r.t_ns) <= 20_000_000
                for j in (k - 1, k)
            )
            if not near:
                continue
            thr, st = sticks(np.nan_to_num(r.a), np.nan_to_num(r.b))
            self.coverage_count[bin_of(float(thr)), bin_of(float(st))] += 1

    def _window_at(self, t: float) -> WindowRecord | None:
        k = int(t // model.WINDOW_S)
        if 0 <= k < len(self.windows):
            return self.windows[k]
        return None

    def _resolve_events(self) -> None:
        """Give each finished event the status of the window it ends in, once that closed."""
        still: list[Event] = []
        for ev in self._pending_events:
            w = self._window_at(ev.t1 - 1e-6)
            if w is None:
                still.append(ev)
                continue
            first = self._window_at(ev.t0)
            statuses = {w.status} | ({first.status} if first is not None else set())
            clean = statuses <= set(FIT_STATUSES)
            if ev.kind == "plateau":
                if clean and ev.hold_cell is not None:
                    self.coverage_held[bin_of(ev.hold_cell[0]), bin_of(ev.hold_cell[1])] += (
                        ev.hold_s
                    )
                continue
            if ev.kind == "edge" and clean and ev.edge_cell is not None:
                self.edges[ev.edge_cell] += 1
            if w.status in (WindowStatus.IDLE, WindowStatus.DISARMED):
                continue
            entry = ev.as_dict()
            if ev.kind == "slew":
                entry["status"] = "warn"
            elif clean:
                entry["status"] = "ok"
            else:
                bad = next(s for s in statuses if s not in FIT_STATUSES)
                entry["status"] = "bad"
                entry["text"] = f"{ev.text}, dropped: {DROP_TEXT[bad]}"
                entry["informs"] = []
            self.events.append(entry)
        self._pending_events = still
        if len(self.events) > 80:
            del self.events[: len(self.events) - 80]

    def _poll_refit(self) -> None:
        reply = self.engine.poll()
        if reply is not None:
            self._apply_refit(reply)
        if self.engine.busy:
            return
        t = self.driving_s
        new = [w for w in self.windows if w.id > self._sent_upto and w.status in FIT_STATUSES]
        if not new:
            return
        if self.refits and t - self._last_refit_t < REFIT_S:
            return
        self._last_refit_t = t
        upto = self.windows[-1].id
        self.engine.submit(
            RefitJob(
                theta=self.theta.copy(),
                ladder={k: v.copy() for k, v in self.ladder.items()},
                refit_index=self.refits,
                add=[(w.id, w.status, w.subs) for w in new],
                upto=upto,
                loop_on=bool(self.grid and self.grid[-1].auto),
                floor_run_m=self.floor_run_m,
                view_azimuth=self.poses.noise.view_azimuth_rad,
                t=self._session_s(self.grid[-1].t_ns) if self.grid else 0.0,
                driving_s=self.driving_s,
            )
        )
        self._sent_upto = upto

    def _apply_refit(self, reply: RefitReply) -> None:
        self.theta = reply.theta
        self.ladder = reply.ladder
        self.scale = reply.scale
        self.residual = math.sqrt(reply.scale) if reply.rows else math.nan
        self.refits += 1
        for wid in reply.contact:
            w = self.windows[wid]
            if w.status == WindowStatus.USED:
                w.status = WindowStatus.WALL
                w.reason = "contact"
        self._last_refit_window_id = reply.upto
        self.fisher_at_last_refit = reply.fisher
        self.fisher_since_refit = np.zeros((model.NP, model.NP))
        late = [w for w in self.windows if w.id > reply.upto and w.status == WindowStatus.USED]
        if late:
            subs, owners = [], []
            for w in late:
                subs += w.subs
                owners += [w.id] * len(w.subs)
            batch = model.stack(subs, owners, self.poses.noise.view_azimuth_rad)
            _r, jac = model.jacobian(self.theta, batch)
            self.fisher_since_refit = model.information(jac)
        self.candidates = reply.candidates
        self.floor_limited = np.asarray(reply.floor_limited, dtype=bool)
        self.ladder_holdout = reply.ladder_holdout
        self._push_history(reply.driving_s, reply.holdout, reply.ladder_holdout)

    def _covariance(self) -> np.ndarray:
        return model.covariance(self.fisher_at_last_refit + self.fisher_since_refit, self.scale)

    def _push_history(self, t: float, hold: float, ladder: dict[str, float]) -> None:
        cov = self._covariance()
        std = np.sqrt(np.diag(cov))
        fills = model.fills_of(std)
        self.history.append(
            {
                "t": round(t, 2),
                "theta": self.theta.tolist(),
                "std": std.tolist(),
                "overall": float(np.mean(fills)),
                "hold": None if not math.isfinite(hold) else hold,
                "ladder": ladder,
                "residual": None if not math.isfinite(self.residual) else self.residual,
                "used": sum(1 for w in self.windows if w.status == WindowStatus.USED),
            }
        )

    def _make_ghosts(self) -> None:
        if self._ghost_next_ns is None or not self.grid:
            return
        last = self.grid[-1].t_ns
        starts = []
        while self._ghost_next_ns + model.SUB_N * DT_NS <= last:
            starts.append(self._ghost_next_ns)
            self._ghost_next_ns += int(GHOST_EVERY_S * _NS)
        subs = [(t, self._subwindow(t)) for t in starts]
        ready = [(t, s) for t, s in subs if s is not None]
        if ready:
            out = model.predict_closed_loop(self.theta, [s for _t, s in ready])
            for i, (t, _s) in enumerate(ready):
                self.ghosts.append(
                    {
                        "t": self._session_s(t),
                        "x": np.round(out["x"][i, ::5], 4).tolist(),
                        "y": np.round(out["y"][i, ::5], 4).tolist(),
                    }
                )
        cutoff = self._session_s(last) - GHOST_KEEP_S
        while self.ghosts and self.ghosts[0]["t"] < cutoff:
            self.ghosts.popleft()

    def _trim(self) -> None:
        if not self.grid:
            return
        keep_ns = self.grid[-1].t_ns - int(KEEP_S * _NS)
        while self.grid and self.grid[0].t_ns < keep_ns:
            self.grid.popleft()
        self.poses.trim(keep_ns)
        horizon_ms = None
        while len(self._tele) > 2 and self._tele_ptr > 1:
            if int(self.clock.to_host_ns(self._tele[1].robot_ms)) >= keep_ns:
                break
            horizon_ms = self._tele.popleft().robot_ms
            self._tele_ptr -= 1
        if horizon_ms is not None:
            while self._seq_gaps and self._seq_gaps[0][1] < horizon_ms:
                self._seq_gaps.popleft()

    def snapshot(self) -> dict[str, Any]:
        cov = self._covariance()
        std = np.sqrt(np.diag(cov))
        fills = model.fills_of(std)
        counts = {s.value: 0 for s in WindowStatus}
        for w in self.windows:
            counts[w.status.value] += 1
        now = self.grid[-1].t_ns if self.grid else 0
        tape_rows = [r for r in self.grid if r.t_ns >= now - 8 * _NS][::4]
        tthr, tst = sticks(
            np.nan_to_num([r.a for r in tape_rows]), np.nan_to_num([r.b for r in tape_rows])
        )
        recent = [p for p in self.poses.poses if p.stamp_ns >= now - int(GHOST_KEEP_S * _NS)]
        cur = self.grid[-1] if self.grid else None
        cthr, cst = sticks(
            np.nan_to_num(cur.a if cur else 0.0), np.nan_to_num(cur.b if cur else 0.0)
        )
        pose = recent[-1] if recent else None
        return {
            "t": round(self._session_s(now), 2),
            "driving_s": round(self.driving_s, 1),
            "theta": self.theta.tolist(),
            "std": std.tolist(),
            "fills": fills.tolist(),
            "overall": float(np.mean(fills)),
            "corr": np.round(model.correlation(cov), 3).ravel().tolist(),
            "floor_limited": self.floor_limited.tolist(),
            "windows": counts,
            "window_spans": [
                {
                    "t0": w.t0_session_s,
                    "t1": w.t1_session_s,
                    "status": w.status.value,
                    "reason": w.reason,
                }
                for w in self.windows
                if w.t1_session_s >= self._session_s(now) - 8.0
            ],
            "coverage": {
                "bins": list(BINS),
                "count": self.coverage_count.ravel().tolist(),
                "held": np.round(self.coverage_held, 2).ravel().tolist(),
            },
            "edges": self.edges.tolist(),
            "edge_rows": list(EDGE_ROWS),
            "events": self.events[-12:],
            "tape": {
                "t": [round(self._session_s(r.t_ns), 2) for r in tape_rows],
                "thr": np.round(tthr, 3).tolist(),
                "steer": np.round(tst, 3).tolist(),
            },
            "stick": {
                "thr": float(cthr),
                "steer": float(cst),
                "armed": bool(cur.armed) if cur else False,
                "loop": bool(cur.auto) if cur else False,
            },
            "candidates": self.candidates,
            "history": self.history,
            "ladder": self.ladder_holdout,
            "residual": None if not math.isfinite(self.residual) else self.residual,
            "refits": self.refits,
            "arena": {
                "size": [self.profile.field_size_x_m, self.profile.field_size_y_m],
                "limit": self.half_extent - ROBOT_LENGTH_M,
                "dots": [[round(p.x, 4), round(p.y, 4)] for p in recent[::2]],
                "ghosts": list(self.ghosts),
                "pose": None if pose is None else [pose.x, pose.y, pose.yaw],
            },
            "floor_run_m": self.floor_run_m,
            "link": self._link(),
        }

    def _link(self) -> dict[str, Any]:
        now = self.grid[-1].t_ns if self.grid else 0
        self._rate_marks.append(RateMark(now, self._tele_seen, self._frames_seen))
        while self._rate_marks and self._rate_marks[0][0] < now - 3 * _NS:
            self._rate_marks.popleft()
        t0, tele0, frames0 = self._rate_marks[0]
        span = max((now - t0) / _NS, 1e-9)
        status = self.clock.status
        poses_2s = sum(1 for p in self.poses.poses if p.stamp_ns >= now - 2 * _NS)
        frames_2s = sum(1 for s in self.poses.frame_stamps if s >= now - 2 * _NS)
        return {
            "telemetry_hz": (self._tele_seen - tele0) / span if span > 0.5 else None,
            "frames_hz": (self._frames_seen - frames0) / span if span > 0.5 else None,
            "tag_coverage": poses_2s / frames_2s if frames_2s else None,
            "clock_p50_ms": None if status is None else status.residual_p50_ms,
            "clock_p95_ms": None if status is None else status.residual_p95_ms,
            "clock_drift_ppm": None if status is None else status.drift_ppm,
            "seq_drops": self.seq_drops,
            "field": self.poses.has_field,
            "jetson_seen": self.jetson_seen,
            "vbat": None
            if not self.grid or not math.isfinite(self.grid[-1].vbat)
            else self.grid[-1].vbat,
        }


HIST_NS = model.HIST_N * DT_NS
DROP_TEXT = {
    WindowStatus.WALL: "wall contact",
    WindowStatus.FLIP: "flipped or nose lifted",
    WindowStatus.TAGS: "tag coverage",
    WindowStatus.LINK: "telemetry gap",
    WindowStatus.JETSON: "Jetson could command",
    WindowStatus.DISARMED: "disarmed",
    WindowStatus.IDLE: "idle",
}
