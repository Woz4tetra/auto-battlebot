"""Match-driving window loader for the plant fit.

Joins regenerated replay poses to the original transmitted commands for one NHRL
recording, resamples both onto the SVO frame grid, and cuts open-loop scoring
windows with `auto_battlebot.plant.make_windows`. The sibling of the jig loader
in `jig_fit.py`: same window machinery, different excitation source (match
driving instead of jig protocols).

The join (docs/experiments/control_improvement/match_plant_fit_plan.md step 2):

1. Replay mcap `/camera/frame_meta` carries `svo_frame_index` and the raw
   original-clock `image_stamp_ns` per tick, so the SVO frame index is the join
   key and the grid at once. No timestamp matching against the SVO file is
   needed; `image_stamp_ns` never went through the replay's stamp rebase.
2. `/robot_markers` (ns `robot_bounds`) in the same replay mcap carries the
   field-frame pose per tick. Ticks are associated to frame_meta by log order
   (markers follow their frame_meta within 0.4 ms) and the association is
   verified through the constant rebase offset between the marker header stamp
   and `image_stamp_ns`.
3. The original mcap `/diagnostics` `opentx_transmitter` `channels` entries are
   the transmitted command, already on the original clock.

Command normalization matches the jig fit so the stage A seed stays on one
scale: channels divided by 1024 full scale, tank inverse mix with channel B
inverted, angular negated (`drive_protocol.MixConfig` defaults, verified
against this data: positive angular tracks positive yaw rate, positive linear
tracks forward motion; join validation 2026-09-01).
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from auto_battlebot.mcap_io import (
    decode_diagnostic_array,
    decode_string,
    iter_messages,
)
from auto_battlebot.plant import WindowSet, concat_windows, make_windows

OUR_ROBOT_MARKER_ID = 4  # FrameId enum index of OUR_ROBOT_1
THEIR_ROBOT_MARKER_IDS = (6, 7, 8)

# The auto switch reads back 1024 when autonomy is enabled. Only then does the
# radio mix the logged channels into what the robot receives; manual segments
# are excluded from windows entirely (plan step 3).
AUTO_SWITCH_VALUE = 1024.0

# Full scale of the radio channel readback stream, same as
# auto_battlebot.calibration.drive_protocol.MixConfig.channel_scale.
CHANNEL_SCALE = 2048.0  # (ch0 - ch1) spans 2 * 1024 at full stick

# Contact gates. A window overlapping any gated sample is dropped; contact
# windows are not fit material (impact model later, not this fit).
WALL_MARGIN_M = 0.10  # robot center this close to the occupancy hull counts as wall contact
OPPONENT_DIST_M = 0.35  # center-to-center; below this the robots are plausibly touching
# Finite-difference acceleration beyond what k_fwd/tau can produce means an
# impact or a perception glitch. Stage A gives k_fwd/tau_lin_a ~ 33 m/s^2 and
# k_ang/tau_ang_a ~ 182 rad/s^2; thresholds sit above those plus the
# pose-noise floor of the 30 Hz double difference.
LIN_ACCEL_LIMIT = 50.0  # m/s^2
ANG_ACCEL_LIMIT = 400.0  # rad/s^2

MANEUVER_CLASSES = ("straight", "arc", "spin", "reversal", "stop", "mixed", "idle")


# ---------------------------------------------------------------------------
# Replay side: pose track keyed by SVO frame index
# ---------------------------------------------------------------------------


@dataclass
class ReplayTrack:
    """Per-SVO-frame pose track from one replay mcap, on the original clock."""

    replay_path: Path
    svo_index: np.ndarray  # (N,) int, strictly increasing, gaps where frames dropped
    stamp_ns: np.ndarray  # (N,) original-clock image stamp
    x: np.ndarray
    y: np.ndarray
    theta: np.ndarray
    live: np.ndarray  # bool, our robot detected fresh this frame
    opp_x: np.ndarray  # nan when no opponent track this frame
    opp_y: np.ndarray
    field_size: tuple[float, float] | None
    rebase_offset_std_ns: float  # join self-check, see load_replay_track


def _quat_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _load_frame_meta(replay_path: Path) -> list[tuple[int, int, int]]:
    """(log_time, image_stamp_ns, svo_frame_index) per tick, in log order."""
    metas: list[tuple[int, int, int]] = []
    for _topic, ts, data in iter_messages(replay_path, ["/camera/frame_meta"]):
        payload = json.loads(decode_string(data))
        metas.append((ts, int(payload["image_stamp_ns"]), int(payload["svo_frame_index"])))
    if not metas:
        raise ValueError(f"{replay_path} has no /camera/frame_meta; re-record the replay")
    return metas


def _load_live_by_tick(replay_path: Path, meta_log: np.ndarray) -> dict[int, bool]:
    """Whether our robot was detected live (not coasted) on each tick."""
    live: dict[int, bool] = {}
    for _topic, ts, data in iter_messages(replay_path, ["/diagnostics"]):
        for status in decode_diagnostic_array(data):
            if status["hardware_id"] == "runner" and status["name"] == "perception":
                tick = int(np.searchsorted(meta_log, ts)) - 1
                if tick >= 0:
                    live[tick] = status["values"].get("our_present_live") == "1"
    return live


def _log_time_ns(log_time: Any) -> int:
    if isinstance(log_time, (int, np.integer)):
        return int(log_time)
    return int(round(log_time.timestamp() * 1e9))


def _load_marker_poses(
    replay_path: Path, meta_log: np.ndarray
) -> tuple[
    dict[int, tuple[float, float, float, int]],
    dict[int, tuple[float, float]],
    tuple[float, float] | None,
]:
    """Per-tick our-robot pose (x, y, yaw, header stamp), opponent xy, and the field size."""
    # mcap_ros1 handles the visualization_msgs decoding; imported here so the module can be
    # imported without it when only command loading is needed.
    from mcap_ros1.reader import read_ros1_messages

    ours: dict[int, tuple[float, float, float, int]] = {}
    opps: dict[int, tuple[float, float]] = {}
    field_size: tuple[float, float] | None = None

    for message in read_ros1_messages(
        str(replay_path), topics=["/robot_markers", "/field_markers"]
    ):
        log_ns = _log_time_ns(message.log_time)
        for marker in message.ros_msg.markers:
            if marker.ns == "field" and len(marker.points) >= 4:
                field_size = _wider_field(field_size, marker.points[:4])
                continue
            if marker.ns != "robot_bounds":
                continue
            tick = int(np.searchsorted(meta_log, log_ns)) - 1
            if tick < 0:
                continue
            pose = marker.pose
            if marker.id == OUR_ROBOT_MARKER_ID:
                stamp = marker.header.stamp
                stamp_ns = int(stamp.secs) * 1_000_000_000 + int(stamp.nsecs)
                yaw = _quat_yaw(
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
                )
                ours[tick] = (float(pose.position.x), float(pose.position.y), yaw, stamp_ns)
            elif marker.id in THEIR_ROBOT_MARKER_IDS:
                opps[tick] = (float(pose.position.x), float(pose.position.y))

    return ours, opps, field_size


def _wider_field(current: tuple[float, float] | None, points: Any) -> tuple[float, float]:
    """Keep the largest field border seen: early inits can fit an undersized square."""
    corners = [np.array([p.x, p.y]) for p in points]
    width = float(np.linalg.norm(corners[1] - corners[0]))
    height = float(np.linalg.norm(corners[2] - corners[1]))
    if current is None or width * height > current[0] * current[1]:
        return (width, height)
    return current


def load_replay_track(replay_path: Path | str) -> ReplayTrack:
    """Extract the our-robot pose per SVO frame from a replay mcap.

    Ticks are associated to `/camera/frame_meta` by log order: within one tick
    the publisher writes frame_meta first, then markers and diagnostics, and
    the next frame_meta is a frame interval away. The association is then
    verified via the stamp rebase: marker header stamp minus the frame's
    `image_stamp_ns` must be one constant across the run (the replay clock
    offset). A drifting offset means the association slipped a tick.
    """
    replay_path = Path(replay_path)
    metas = _load_frame_meta(replay_path)
    meta_log = np.array([m[0] for m in metas], dtype=np.int64)
    live_by_tick = _load_live_by_tick(replay_path, meta_log)
    ours, opps, field_size = _load_marker_poses(replay_path, meta_log)

    rows = []
    offsets = []
    for tick, (_log, image_stamp, svo_index) in enumerate(metas):
        if svo_index < 0:
            continue
        our = ours.get(tick)
        opp = opps.get(tick)
        if our is not None:
            offsets.append(our[3] - image_stamp)
        rows.append(
            (
                svo_index,
                image_stamp,
                our[0] if our else np.nan,
                our[1] if our else np.nan,
                our[2] if our else np.nan,
                bool(live_by_tick.get(tick, False)) and our is not None,
                opp[0] if opp else np.nan,
                opp[1] if opp else np.nan,
            )
        )
    if not rows:
        raise ValueError(f"{replay_path}: no frames with a valid svo_frame_index")

    svo_indices = np.array([r[0] for r in rows], dtype=np.int64)
    order = np.argsort(svo_indices)
    offset_arr = np.array(offsets, dtype=np.int64)
    offset_std = float(offset_arr.std()) if len(offset_arr) else float("nan")
    if len(offset_arr) and offset_std > 1e6:  # 1 ms; measured 100 ns on clean runs
        raise ValueError(
            f"{replay_path}: marker-to-frame association drifted "
            f"(rebase offset std {offset_std / 1e6:.2f} ms); the join is not trustworthy"
        )

    def _col(i: int, dtype: Any = float) -> np.ndarray:
        return np.array([r[i] for r in rows], dtype=dtype)[order]

    return ReplayTrack(
        replay_path=replay_path,
        svo_index=svo_indices[order],
        stamp_ns=_col(1, np.int64),
        x=_col(2),
        y=_col(3),
        theta=_col(4),
        live=_col(5, bool),
        opp_x=_col(6),
        opp_y=_col(7),
        field_size=field_size,
        rebase_offset_std_ns=offset_std,
    )


# ---------------------------------------------------------------------------
# Original side: transmitted commands
# ---------------------------------------------------------------------------


@dataclass
class CommandLog:
    """Transmitted commands from the original mcap, on the original clock."""

    t_ns: np.ndarray  # (M,) log time of each channels entry
    lin: np.ndarray  # normalized, jig convention
    ang: np.ndarray
    auto: np.ndarray  # bool, auto switch up (forward-filled; latched physical state)


def load_commands(original_path: Path | str) -> CommandLog:
    """Decode the `opentx_transmitter` channel readback into body commands.

    lin = (ch0 - ch1) / 2048, ang = -(ch0 + ch1) / 2048: the tank inverse mix
    with channel B inverted and angular negated, exactly
    `drive_protocol.MixConfig.from_channels` at readback scale 1024. Entries
    are logged on change; consumers zero-order hold between entries.
    """
    t_ns: list[int] = []
    lin: list[float] = []
    ang: list[float] = []
    auto: list[bool] = []
    auto_state = False
    for _topic, ts, data in iter_messages(original_path, ["/diagnostics"]):
        for status in decode_diagnostic_array(data):
            if status["hardware_id"] != "opentx_transmitter" or status["name"] != "channels":
                continue
            values = status["values"]
            if "values/15" in values:
                auto_state = float(values["values/15"]) == AUTO_SWITCH_VALUE
            if "values/0" not in values or "values/1" not in values:
                continue
            ch0 = float(values["values/0"])
            ch1 = float(values["values/1"])
            t_ns.append(ts)
            lin.append((ch0 - ch1) / CHANNEL_SCALE)
            ang.append(-(ch0 + ch1) / CHANNEL_SCALE)
            auto.append(auto_state)
    if not t_ns:
        raise ValueError(f"{original_path} has no opentx_transmitter channel entries")
    return CommandLog(
        t_ns=np.array(t_ns, dtype=np.int64),
        lin=np.array(lin),
        ang=np.array(ang),
        auto=np.array(auto, dtype=bool),
    )


# ---------------------------------------------------------------------------
# Grid assembly
# ---------------------------------------------------------------------------


@dataclass
class MatchRun:
    """One recording resampled onto its SVO frame grid, jig-Run-shaped.

    The grid index is the SVO frame index shifted to zero, so the grid is exact
    by construction: no accumulated drift, and a dropped exporter frame is a
    hole in `pose_valid` rather than a timing error.
    """

    name: str
    recording: str
    role: str  # "fit" or "holdout" or "validation"
    t: np.ndarray  # seconds from grid start, uniform
    dt: float
    t0_ns: int  # original-clock stamp of grid index 0
    x: np.ndarray
    y: np.ndarray
    theta: np.ndarray
    v: np.ndarray  # body forward speed, signed, finite-differenced from pose
    w: np.ndarray  # yaw rate
    cmd_lin: np.ndarray
    cmd_ang: np.ndarray
    valid: np.ndarray  # bool, usable for windows (pose + auto + command + no contact)
    contact: np.ndarray  # bool, gated by wall/opponent/accel (subset of ~valid)
    auto: np.ndarray  # bool

    def stamp_of(self, index: int | np.ndarray) -> np.ndarray:
        """Original-clock nanosecond stamp of a grid index."""
        return (self.t0_ns + np.asarray(index) * self.dt * 1e9).astype(np.int64)


def _fill_grid(track: ReplayTrack) -> tuple[np.ndarray, dict[str, np.ndarray], float, int]:
    """Place per-frame samples onto a uniform time grid.

    The SVO frame index is NOT the grid: the original camera occasionally
    missed a grab, so consecutive indices can sit two frame intervals apart
    (the 17-42-20 recording has ~1% of intervals at 67 ms). Slots are assigned
    by cumulative rounding of the stamp differences, so a missed grab leaves an
    empty slot instead of shearing everything after it; per-frame jitter is
    about a millisecond against a 16 ms rounding margin.
    """
    rel = (track.stamp_ns - track.stamp_ns[0]).astype(float)
    steps = np.diff(rel)
    dt0 = float(np.median(steps))
    slots = np.concatenate([[0], np.cumsum(np.maximum(1, np.round(steps / dt0).astype(int)))])
    # Refine dt by least squares of stamp against slot, centered so the fit is
    # well conditioned, then re-assign. One pass converges: the refinement is
    # well under the rounding margin.
    slope, intercept = np.polyfit(slots.astype(float), rel, 1)
    slots = np.concatenate([[0], np.cumsum(np.maximum(1, np.round(steps / slope).astype(int)))])
    slope, intercept = np.polyfit(slots.astype(float), rel, 1)
    dt = float(slope / 1e9)
    t0_ns = int(track.stamp_ns[0] + intercept)
    size = int(slots[-1]) + 1

    grid: dict[str, np.ndarray] = {
        "x": np.full(size, np.nan),
        "y": np.full(size, np.nan),
        "theta": np.full(size, np.nan),
        "opp_x": np.full(size, np.nan),
        "opp_y": np.full(size, np.nan),
    }
    present = np.zeros(size, dtype=bool)
    live = np.zeros(size, dtype=bool)
    for key, source in (
        ("x", track.x),
        ("y", track.y),
        ("theta", track.theta),
        ("opp_x", track.opp_x),
        ("opp_y", track.opp_y),
    ):
        grid[key][slots] = source
    present[slots] = True
    live[slots] = track.live
    grid["present"] = present
    grid["live"] = live
    t = np.arange(size) * dt
    return t, grid, dt, t0_ns


def build_match_run(
    track: ReplayTrack,
    commands: CommandLog,
    *,
    name: str,
    role: str,
) -> MatchRun:
    """Resample one joined recording onto its frame grid and gate it."""
    t, grid, dt, t0_ns = _fill_grid(track)
    size = len(t)
    stamps = (t0_ns + (t * 1e9)).astype(np.int64)

    # Single-slot holes between two live frames are camera grab drops, not
    # perception dropouts: the original pipeline missed ~25% of grabs on this
    # data, so a strict continuity rule keeps no window at all. The pose is
    # interpolated across exactly one missing slot (chord error under 6 mm at
    # 1 m/s and 10 rad/s, below the per-frame pose noise); longer holes and
    # true perception dropouts still split windows.
    live = grid["live"]
    hole = ~grid["present"]
    hole[1:-1] &= grid["live"][:-2] & grid["live"][2:]
    hole[0] = hole[-1] = False
    for key in ("x", "y", "opp_x", "opp_y"):
        grid[key][hole] = 0.5 * (np.roll(grid[key], 1)[hole] + np.roll(grid[key], -1)[hole])
    theta_prev = np.roll(grid["theta"], 1)[hole]
    theta_next = np.roll(grid["theta"], -1)[hole]
    half_turn = (theta_next - theta_prev + np.pi) % (2.0 * np.pi) - np.pi
    grid["theta"][hole] = theta_prev + 0.5 * half_turn
    live = live.copy()
    live[hole] = True
    grid["live"] = live
    grid["present"] = grid["present"] | hole

    # Commands: zero-order hold of the change log at each grid stamp. Samples
    # before the first entry or after the last have no known command.
    j = np.searchsorted(commands.t_ns, stamps, side="right") - 1
    in_span = (j >= 0) & (stamps <= commands.t_ns[-1] + int(1e9))
    j = np.clip(j, 0, len(commands.t_ns) - 1)
    cmd_lin = commands.lin[j]
    cmd_ang = commands.ang[j]
    auto = commands.auto[j] & in_span

    usable = grid["live"]

    # Velocities by central difference over live neighbors. A sample needs both
    # neighbors live for the central form; edges of a live stretch fall back to
    # one-sided, and isolated live samples get no velocity (and no window ever
    # starts there, because the window span check needs a live run anyway).
    x, y, theta = grid["x"], grid["y"], grid["theta"]
    v = np.full(size, np.nan)
    w = np.full(size, np.nan)

    def _wrapped(a: np.ndarray) -> np.ndarray:
        return (a + np.pi) % (2.0 * np.pi) - np.pi

    prev_ok = np.zeros(size, dtype=bool)
    next_ok = np.zeros(size, dtype=bool)
    prev_ok[1:] = usable[:-1]
    next_ok[:-1] = usable[1:]
    both = usable & prev_ok & next_ok
    fwd = usable & ~prev_ok & next_ok
    bwd = usable & prev_ok & ~next_ok

    idx = np.flatnonzero(both)
    if len(idx):
        dx = x[idx + 1] - x[idx - 1]
        dy = y[idx + 1] - y[idx - 1]
        v[idx] = (dx * np.cos(theta[idx]) + dy * np.sin(theta[idx])) / (2.0 * dt)
        w[idx] = _wrapped(theta[idx + 1] - theta[idx - 1]) / (2.0 * dt)
    for sel, a, b in ((np.flatnonzero(fwd), 1, 0), (np.flatnonzero(bwd), 0, -1)):
        if len(sel):
            dx = x[sel + a] - x[sel + b]
            dy = y[sel + a] - y[sel + b]
            v[sel] = (dx * np.cos(theta[sel]) + dy * np.sin(theta[sel])) / dt
            w[sel] = _wrapped(theta[sel + a] - theta[sel + b]) / dt

    # Contact gates. Walls come from the pose occupancy hull, not the field
    # marker: the field frame is not guaranteed centered, and the 17-42-24
    # replay's single field detection reports a nonsense 2.6 x 0.8 m box. The
    # robot bumps every wall over a match, so the occupancy extremes ARE the
    # wall positions for the robot center.
    contact = np.zeros(size, dtype=bool)
    if np.count_nonzero(usable) > 100:
        x_lo, x_hi = np.nanpercentile(x[usable], [0.2, 99.8])
        y_lo, y_hi = np.nanpercentile(y[usable], [0.2, 99.8])
        contact |= usable & (
            (x < x_lo + WALL_MARGIN_M)
            | (x > x_hi - WALL_MARGIN_M)
            | (y < y_lo + WALL_MARGIN_M)
            | (y > y_hi - WALL_MARGIN_M)
        )
    opp_dist = np.hypot(grid["opp_x"] - x, grid["opp_y"] - y)
    contact |= usable & (opp_dist < OPPONENT_DIST_M)

    # Acceleration spikes from consecutive live velocity samples. |dv/dt| past
    # what the actuators can produce is an impact (or a perception glitch, e.g.
    # a front/back keypoint flip); either way the samples around it are not
    # open-loop plant response. The spike gates its neighbors too, because the
    # difference stencil smears one bad pose across adjacent velocity samples.
    dv_ok = both.copy()
    lin_spike = np.zeros(size, dtype=bool)
    ang_spike = np.zeros(size, dtype=bool)
    idx = np.flatnonzero(dv_ok[:-1] & dv_ok[1:])
    if len(idx):
        lin_acc = np.abs(v[idx + 1] - v[idx]) / dt
        ang_acc = np.abs(w[idx + 1] - w[idx]) / dt
        lin_spike[idx] |= lin_acc > LIN_ACCEL_LIMIT
        ang_spike[idx] |= ang_acc > ANG_ACCEL_LIMIT
    spike = lin_spike | ang_spike
    for shift in (-1, 1):
        spike |= np.roll(lin_spike | ang_spike, shift)
    contact |= spike

    valid = usable & auto & ~contact & np.isfinite(v) & np.isfinite(w)

    return MatchRun(
        name=name,
        recording=name.split("/")[0],
        role=role,
        t=t,
        dt=dt,
        t0_ns=t0_ns,
        x=x,
        y=y,
        theta=theta,
        v=v,
        w=w,
        cmd_lin=cmd_lin,
        cmd_ang=cmd_ang,
        valid=valid,
        contact=contact,
        auto=auto,
    )


def split_run(run: MatchRun, fraction: float) -> tuple[MatchRun, MatchRun]:
    """Split a run at a time fraction into contiguous (head, tail) runs.

    Contiguous, not random: the holdout must not interleave the training data
    or the two sets share every slow-varying condition (battery, floor spot).
    """
    cut = int(len(run.t) * fraction)

    def _piece(sl: slice, role: str, tag: str) -> MatchRun:
        start = sl.start or 0
        return MatchRun(
            name=f"{run.name}[{tag}]",
            recording=run.recording,
            role=role,
            t=run.t[sl] - run.t[sl][0],
            dt=run.dt,
            t0_ns=int(run.t0_ns + start * run.dt * 1e9),
            x=run.x[sl],
            y=run.y[sl],
            theta=run.theta[sl],
            v=run.v[sl],
            w=run.w[sl],
            cmd_lin=run.cmd_lin[sl],
            cmd_ang=run.cmd_ang[sl],
            valid=run.valid[sl],
            contact=run.contact[sl],
            auto=run.auto[sl],
        )

    return _piece(slice(0, cut), "fit", "train"), _piece(slice(cut, None), "holdout", "holdout")


# ---------------------------------------------------------------------------
# Windows
# ---------------------------------------------------------------------------


@dataclass
class MatchWindows:
    """A WindowSet plus the per-window metadata the match analysis needs."""

    windows: WindowSet
    run_index: np.ndarray  # (W,) index into the runs list passed to build
    runs: list[MatchRun] = field(repr=False, default_factory=list)

    def start_stamps_ns(self) -> np.ndarray:
        """Original-clock stamp of each window start."""
        out = np.empty(len(self.windows.starts), dtype=np.int64)
        for i, (run_i, start) in enumerate(zip(self.run_index, self.windows.starts)):
            out[i] = self.runs[run_i].stamp_of(int(start))
        return out

    def maneuver_classes(self) -> np.ndarray:
        """Per-window maneuver tag from the delayed command matrices."""
        u_lin = self.windows.u_lin
        u_ang = self.windows.u_ang
        n = u_lin.shape[0]
        labels = np.empty(n, dtype=object)
        third = max(1, u_lin.shape[1] // 3)
        for i in range(n):
            ul = u_lin[i]
            ua = u_ang[i]
            mean_l = float(np.mean(np.abs(ul)))
            mean_a = float(np.mean(np.abs(ua)))
            starts_hot = max(np.abs(ul[:third]).max(), np.abs(ua[:third]).max())
            ends_cold = max(np.abs(ul[-third:]).max(), np.abs(ua[-third:]).max())
            if np.any(ul > 0.12) and np.any(ul < -0.12):
                labels[i] = "reversal"
            elif starts_hot > 0.2 and ends_cold < 0.05:
                labels[i] = "stop"
            elif mean_a > 0.15 and mean_l < 0.08:
                labels[i] = "spin"
            elif mean_l > 0.15 and mean_a < 0.06:
                labels[i] = "straight"
            elif mean_l > 0.10 and mean_a > 0.06:
                labels[i] = "arc"
            elif mean_l < 0.03 and mean_a < 0.03:
                labels[i] = "idle"
            else:
                labels[i] = "mixed"
        return labels


def build_match_windows(
    runs: Sequence[MatchRun],
    delay_s: float,
    horizons: Sequence[float],
    stride_s: float,
    max_windows: int,
    drop_stamps_ns: Sequence[tuple[int, int]] = (),
) -> MatchWindows | None:
    """Cut windows from every run and stack them, mirroring jig build_windows.

    `drop_stamps_ns` carries (start, end) original-clock spans to exclude, which
    is how the radio-dropout gate removes flagged windows before the final fit.
    """
    sets = []
    run_indices = []
    for index, run in enumerate(runs):
        valid = run.valid.copy()
        for start_ns, end_ns in drop_stamps_ns:
            lo = int(np.floor((start_ns - run.t0_ns) / (run.dt * 1e9)))
            hi = int(np.ceil((end_ns - run.t0_ns) / (run.dt * 1e9)))
            if hi < 0 or lo >= len(valid):
                continue
            valid[max(lo, 0) : min(hi + 1, len(valid))] = False
        if np.count_nonzero(valid) < 10:
            continue
        ws = make_windows(
            run.t,
            run.v,
            run.w,
            run.theta,
            run.x,
            run.y,
            run.cmd_lin,
            run.cmd_ang,
            dt=run.dt,
            delay_s=delay_s,
            horizons=horizons,
            stride_s=stride_s,
            valid=valid,
            origin=index,
        )
        if ws is not None:
            sets.append(ws)
            run_indices.append(index)
    if not sets:
        return None
    combined = concat_windows(sets)
    if max_windows > 0 and combined.count() > max_windows:
        combined = combined.subsample(max_windows)
    assert combined.origin is not None
    return MatchWindows(windows=combined, run_index=combined.origin, runs=list(runs))
