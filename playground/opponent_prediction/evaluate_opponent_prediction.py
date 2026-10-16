"""Score the opponent filter against AprilTag ground truth on the Mr Stabs Mk2 sysid sessions.

The filter is `auto_battlebot.perception.opponent_filter`, the parity-tested mirror of the C++
opponent arm. Each session replays the way the app runs: a measurement arrives `LATENCY_S` after
its shutter, coast() runs every `TICK_S` on the control clock, and the estimate is rendered at
now + `LEAD_S`. Every rendered estimate is scored against the tag truth at its render time. See
`sysid_data.py` for where the measurements and the truth come from.

Arms:
- `hold`: the estimate is the last measurement (today's production `opponent_mode = HOLD`).
- `kalman4`: the previous 4-state KALMAN arm: position filter only, past `max_coast_s` the
  covariance freezes with the state, and heading is the last measured rotation, or the velocity
  direction while stale and faster than 0.3 m/s.
- `phase1`: the 6-state filter this plan adds.

Dropout scenarios withhold every measurement inside periodic gaps, on top of the natural gaps
YOLO already has. Errors are split by time since the last measurement arrived (0 means it arrived
this tick) and by motion class.

Usage:
    source scripts/activate_python.sh
    python playground/opponent_prediction/evaluate_opponent_prediction.py grid    # tune on train
    python playground/opponent_prediction/evaluate_opponent_prediction.py report  # score both
Output goes under playground/opponent_prediction/out/.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
from dataclasses import dataclass, field

import numpy as np
import torch

from auto_battlebot.compat import tomllib
from auto_battlebot.perception.opponent_filter import (
    MAX_HEADING_VARIANCE,
    MAX_INTEGRATION_STEP_S,
    OMEGA,
    PX,
    PY,
    THETA,
    VX,
    VY,
    N,
    OpponentFilter,
    OpponentFilterConfig,
    RenderedEstimate,
)
from playground.opponent_prediction.sysid_data import OUT, REPO_ROOT, SessionData, load_session

LATENCY_S = 0.063  # measured perception latency (kalman_motion_estimator.hpp)
TICK_S = 0.01
LEGACY_MIN_HEADING_SPEED = 0.3
T0_S = 100.0

GAP_BINS = [(0.0, 0.05), (0.05, 0.15), (0.15, 0.275), (0.275, 0.425), (0.425, 1.2), (1.2, 3.5)]
GAP_LABELS = ["0", "0.1", "0.2", "0.35", "0.43-1.2", "1.2-3.5"]
MOTION_CLASSES = ["still", "straight", "arcing", "spinning"]


def common_config() -> dict:
    with open(REPO_ROOT / "config/_common.toml", "rb") as f:
        return tomllib.load(f)


def lead_s() -> float:
    return float(common_config()["plant"]["delay_s"])


def max_coast_s() -> float:
    """The shared horizon the previous 4-state arm coasted opponents to."""
    return float(common_config()["robot_filter"]["motion_estimator"]["max_coast_s"])


def opponent_max_coast_s() -> float:
    est = common_config()["robot_filter"]["motion_estimator"]
    return float(est.get("opponent_max_coast_s", OpponentFilterConfig.opponent_max_coast_s))


# ------------------------------------------------------------------------------------- arms


class LegacyOpponentFilter(OpponentFilter):
    """The 4-state arm the plan replaces, built on the mirror so gating and rewind are shared.

    Its translation block is identical to the 6-state filter's. Two things differ: past the coast
    horizon it froze the covariance with the state (identity transition), and it rendered the
    last measured rotation, or the velocity direction while stale and fast.
    """

    def __init__(self, batch: int, config: OpponentFilterConfig) -> None:
        super().__init__(batch, config)
        # It coasted opponents to the horizon it shared with our robot.
        self.opponent_max_coast_s = torch.full((batch,), max_coast_s(), dtype=torch.float64)

    def _advance(self, target: torch.Tensor, mask: torch.Tensor) -> None:
        dt_total = target - self.stamp
        move = mask & (dt_total > 0.0)
        if not bool(move.any()):
            return
        dt = torch.clamp(dt_total, max=MAX_INTEGRATION_STEP_S)
        age = self.stamp - self.last_measured
        hold = (age >= self.opponent_max_coast_s) | (dt_total > MAX_INTEGRATION_STEP_S)
        f = torch.eye(N, dtype=self.dtype).repeat(self.batch, 1, 1)
        step = torch.where(hold, torch.zeros_like(dt), dt)
        f[:, PX, VX] = step
        f[:, PY, VY] = step
        f[:, THETA, OMEGA] = step
        q = torch.zeros(self.batch, N, N, dtype=self.dtype)
        qa = self.opponent_accel_psd
        q[:, PX, PX] = q[:, PY, PY] = qa * dt**3 / 3.0
        q[:, PX, VX] = q[:, VX, PX] = qa * dt**2 / 2.0
        q[:, PY, VY] = q[:, VY, PY] = qa * dt**2 / 2.0
        q[:, VX, VX] = q[:, VY, VY] = qa * dt
        q[:, THETA, THETA] = MAX_HEADING_VARIANCE  # heading unused; keep it inert
        x = torch.einsum("bij,bj->bi", f, self.x)
        p = f @ self.p @ f.transpose(1, 2) + q
        p = (p + p.transpose(1, 2)) * 0.5
        self.x = torch.where(move[:, None], x, self.x)
        self.p = torch.where(move[:, None, None], p, self.p)
        self.stamp = torch.where(move, target, self.stamp)
        self._push_snapshot(move)

    def render(self, stale: torch.Tensor) -> RenderedEstimate:
        out = super().render(stale)
        speed = torch.hypot(out.vx, out.vy)
        velocity_heading = stale & (speed >= LEGACY_MIN_HEADING_SPEED)
        out.yaw = torch.where(velocity_heading, torch.atan2(out.vy, out.vx), self.measured_yaw)
        out.omega = torch.zeros_like(out.omega)
        inf = torch.full_like(out.x, math.inf)
        out.heading_sigma_rad = inf
        out.yaw_rate_sigma_rad_s = inf
        return out


@dataclass
class Trace:
    """Rendered estimates at every recorded tick, (T, B) each, plus per-update stats."""

    target_ns: np.ndarray  # (T,) render time on the session clock
    since_arrival_s: np.ndarray  # (T, B) time since the last measurement this row accepted arrived
    values: dict[str, np.ndarray]
    update_stats: dict[str, np.ndarray] = field(default_factory=dict)  # (M, B)


def _rel_s(stamp_ns: np.ndarray, t0_ns: int) -> np.ndarray:
    return T0_S + (np.asarray(stamp_ns, np.int64) - t0_ns) / 1e9


def simulate_filter(
    data: SessionData,
    filt: OpponentFilter,
    keep: np.ndarray,
    position_sigma_m: float | torch.Tensor,
    record_stride: int = 1,
    window_ns: tuple[int, int] | None = None,
) -> Trace:
    """Replays the session through `filt`; `keep` (B, M) withholds measurements per row."""
    t0_ns = int(data.meas_stamp_ns[0])
    sel = np.ones(len(data.meas_stamp_ns), dtype=bool)
    if window_ns is not None:
        sel = (data.meas_stamp_ns >= window_ns[0]) & (data.meas_stamp_ns <= window_ns[1])
    idx = np.nonzero(sel)[0]
    t_meas = _rel_s(data.meas_stamp_ns[idx], t0_ns)
    deliver = t_meas + LATENCY_S
    keep_t = torch.as_tensor(keep[:, idx])
    b = filt.batch
    xy = torch.as_tensor(np.stack([data.meas_x[idx], data.meas_y[idx]], axis=1))
    yaw = torch.as_tensor(data.meas_yaw[idx])
    is_kp = torch.ones(b, dtype=torch.bool)
    var = torch.as_tensor(position_sigma_m, dtype=torch.float64).expand(b) ** 2

    ticks = np.arange(t_meas[0], deliver[-1] + 0.5, TICK_S)
    names = [
        "x",
        "y",
        "yaw",
        "vx",
        "vy",
        "omega",
        "position_sigma_m",
        "heading_sigma_rad",
        "yaw_rate_sigma_rad_s",
        "is_stale",
        "active",
    ]
    rec: dict[str, list[np.ndarray]] = {k: [] for k in names}
    since: list[np.ndarray] = []
    targets: list[float] = []
    stats: dict[str, list[np.ndarray]] = {
        k: []
        for k in ("accepted", "position_nis", "heading_nis", "heading_updated", "heading_flip")
    }
    stats["measured"] = []
    last_arrival = np.full(b, -np.inf)
    j = 0
    for i, now in enumerate(ticks):
        while j < len(t_meas) and deliver[j] <= now + 1e-9:
            measured = keep_t[:, j]
            _, st = filt.update(
                float(t_meas[j]),
                measured,
                xy[j].expand(b, 2),
                yaw[j].expand(b),
                is_kp,
                var,
            )
            acc = st.accepted.numpy()
            last_arrival = np.where(acc, deliver[j], last_arrival)
            stats["measured"].append(measured.numpy().copy())
            stats["accepted"].append(acc)
            stats["position_nis"].append(st.position_nis.numpy())
            stats["heading_nis"].append(st.heading_nis.numpy())
            stats["heading_updated"].append(st.heading_updated.numpy())
            stats["heading_flip"].append(st.heading_flip.numpy())
            j += 1
        out = filt.coast(float(now))
        if i % record_stride:
            continue
        targets.append(now + filt.render_lead_s)
        since.append(now - last_arrival)
        for k in names:
            rec[k].append(getattr(out, k).numpy().astype(float))
    target_ns = (t0_ns + np.round((np.asarray(targets) - T0_S) * 1e9)).astype(np.int64)
    return Trace(
        target_ns=target_ns,
        since_arrival_s=np.asarray(since),
        values={k: np.asarray(v) for k, v in rec.items()},
        update_stats={k: np.asarray(v) for k, v in stats.items()},
    )


def simulate_hold(
    data: SessionData,
    keep: np.ndarray,
    lead: float,
    record_stride: int = 1,
    window_ns: tuple[int, int] | None = None,
) -> Trace:
    """The estimate is the last delivered measurement, with no prediction."""
    t0_ns = int(data.meas_stamp_ns[0])
    sel = np.ones(len(data.meas_stamp_ns), dtype=bool)
    if window_ns is not None:
        sel = (data.meas_stamp_ns >= window_ns[0]) & (data.meas_stamp_ns <= window_ns[1])
    idx = np.nonzero(sel)[0]
    t_meas = _rel_s(data.meas_stamp_ns[idx], t0_ns)
    deliver = t_meas + LATENCY_S
    b = keep.shape[0]
    ticks = np.arange(t_meas[0], deliver[-1] + 0.5, TICK_S)[::record_stride]
    # Index of the newest kept measurement delivered by each tick, per row.
    last = np.full((len(ticks), b), -1)
    for r in range(b):
        kept = np.nonzero(keep[r, idx])[0]
        pos = np.searchsorted(deliver[kept], ticks + 1e-9, side="right") - 1
        last[:, r] = np.where(pos >= 0, kept[np.clip(pos, 0, None)], -1)
    ok = last >= 0
    li = np.clip(last, 0, None)
    mx, my, myaw = data.meas_x[idx][li], data.meas_y[idx][li], data.meas_yaw[idx][li]
    nan = np.full(last.shape, np.nan)
    inf = np.full(last.shape, np.inf)
    since = np.where(ok, ticks[:, None] - deliver[li], np.inf)
    target_ns = (t0_ns + np.round((ticks + lead - T0_S) * 1e9)).astype(np.int64)
    return Trace(
        target_ns=target_ns,
        since_arrival_s=since,
        values={
            "x": np.where(ok, mx, nan),
            "y": np.where(ok, my, nan),
            "yaw": np.where(ok, myaw, nan),
            "vx": np.zeros(last.shape),
            "vy": np.zeros(last.shape),
            "omega": np.zeros(last.shape),
            "position_sigma_m": inf,
            "heading_sigma_rad": inf,
            "yaw_rate_sigma_rad_s": inf,
            "is_stale": (since > 0.1).astype(float),
            "active": ok.astype(float),
        },
    )


# ---------------------------------------------------------------------------------- dropouts


def periodic_gaps(
    stamps_ns: np.ndarray, gap_s: float, period_s: float, phase_s: float = 1.0
) -> np.ndarray:
    """True for measurements kept; drops every measurement inside [k*period + phase, + gap)."""
    if gap_s <= 0.0:
        return np.ones(len(stamps_ns), dtype=bool)
    t = (stamps_ns - stamps_ns[0]) / 1e9 - phase_s
    return ~((t >= 0.0) & (np.mod(t, period_s) < gap_s))


# ----------------------------------------------------------------------------------- scoring


def motion_class(speed: np.ndarray, yaw_rate: np.ndarray) -> np.ndarray:
    w = np.abs(yaw_rate)
    cls = np.full(speed.shape, "straight", dtype=object)
    cls[(w >= 0.5)] = "arcing"
    cls[(w >= 6.0)] = "spinning"
    cls[(speed < 0.05) & (w < 0.5)] = "still"
    return cls


def _pct(x: np.ndarray, q: float) -> float:
    return float(np.percentile(x, q)) if x.size else float("nan")


def score_trace(data: SessionData, trace: Trace, row: int) -> dict:
    v = trace.values
    tr = data.truth_at(trace.target_ns)
    ok = tr["valid"] & (v["active"][:, row] > 0) & np.isfinite(v["x"][:, row])
    pos_err = np.hypot(v["x"][:, row] - tr["x"], v["y"][:, row] - tr["y"])
    yaw_err = np.abs(wrap_np(v["yaw"][:, row] - tr["yaw"]))
    omega_err = np.abs(v["omega"][:, row] - tr["yaw_rate"])
    since = trace.since_arrival_s[:, row]
    cls = motion_class(np.hypot(tr["vx"], tr["vy"]), tr["yaw_rate"])

    def summarize(mask: np.ndarray) -> dict:
        m = ok & mask
        out: dict = {"n": int(m.sum())}
        if not m.any():
            return out
        out["pos_p50_mm"] = 1e3 * _pct(pos_err[m], 50)
        out["pos_p90_mm"] = 1e3 * _pct(pos_err[m], 90)
        out["yaw_p50_deg"] = math.degrees(_pct(yaw_err[m], 50))
        out["yaw_p90_deg"] = math.degrees(_pct(yaw_err[m], 90))
        out["omega_p50"] = _pct(omega_err[m], 50)
        out["omega_p90"] = _pct(omega_err[m], 90)
        for name, err, sig in (
            ("pos", pos_err, v["position_sigma_m"][:, row]),
            ("yaw", yaw_err, v["heading_sigma_rad"][:, row]),
            ("omega", omega_err, v["yaw_rate_sigma_rad_s"][:, row]),
        ):
            finite = m & np.isfinite(sig)
            if finite.any():
                out[f"cover2_{name}"] = float(np.mean(err[finite] <= 2.0 * sig[finite]))
        return out

    result: dict = {"all": summarize(np.ones_like(ok))}
    result["by_gap"] = {
        label: summarize((since >= lo) & (since < hi))
        for label, (lo, hi) in zip(GAP_LABELS, GAP_BINS, strict=True)
    }
    result["by_motion"] = {c: summarize(cls == c) for c in MOTION_CLASSES}
    result["by_gap_motion"] = {
        label: {c: summarize((since >= lo) & (since < hi) & (cls == c)) for c in MOTION_CLASSES}
        for label, (lo, hi) in zip(GAP_LABELS, GAP_BINS, strict=True)
    }
    st = trace.update_stats
    if st:
        acc = st["accepted"][:, row]
        result["nis"] = {
            "position_mean": float(np.mean(st["position_nis"][acc, row])),
            "joint_mean": float(
                np.mean(
                    st["position_nis"][acc, row]
                    + np.where(st["heading_updated"][acc, row], st["heading_nis"][acc, row], 0.0)
                )
            ),
            "joint_dof": float(np.mean(2.0 + st["heading_updated"][acc, row])),
            "accepted_frac": float(acc[st["measured"][:, row]].mean()),
            "flips_per_100": 100.0 * float(st["heading_flip"][:, row].mean()),
        }
    return result


def wrap_np(a: np.ndarray) -> np.ndarray:
    return np.asarray(np.angle(np.exp(1j * a)))


# ----------------------------------------------------------------------------------- tuning

GRID = {
    "opponent_accel_psd": [3.0, 10.0, 30.0, 100.0, 300.0, 1000.0],
    "opponent_yaw_accel_psd": [10.0, 30.0, 100.0, 300.0, 1000.0],
    "opponent_heading_sigma_rad": [0.05, 0.1, 0.15, 0.25],
    "opponent_position_sigma_m": [0.008, 0.012, 0.02, 0.03],
}
TUNE_GAP_S = 0.4
TUNE_PERIOD_S = 3.0


def objective(s: dict) -> float:
    """Lower is better.

    Median and p90 position error fresh and at 0.35 s, normalized by 20 / 80 mm and
    100 / 400 mm; median heading error normalized by 0.1 and 0.3 rad; the 2-sigma coverage miss on
    position and heading, 0.05 of miss per unit; and the share of measurements the gate rejected,
    1% per unit. A rejected measurement on a clean sysid stream is lost track, not an outlier.
    """
    fresh = s["by_gap"]["0"]
    late = s["by_gap"]["0.35"]
    every = s["all"]
    j = fresh["pos_p50_mm"] / 20.0 + fresh["pos_p90_mm"] / 80.0
    j += late["pos_p50_mm"] / 100.0 + late["pos_p90_mm"] / 400.0
    j += math.radians(fresh["yaw_p50_deg"]) / 0.1 + math.radians(late["yaw_p50_deg"]) / 0.3
    j += abs(every.get("cover2_pos", 0.0) - 0.95) / 0.05
    j += abs(every.get("cover2_yaw", 0.0) - 0.95) / 0.05
    j += (1.0 - s["nis"]["accepted_frac"]) / 0.01
    return j


def run_grid(train: SessionData) -> list[dict]:
    combos = list(itertools.product(*GRID.values()))
    keys = list(GRID)
    cfg = OpponentFilterConfig(opponent_max_coast_s=opponent_max_coast_s(), render_lead_s=lead_s())
    columns = {
        key: torch.tensor([c[k_i] for c in combos], dtype=torch.float64)
        for k_i, key in enumerate(keys)
    }
    position_sigma = columns.pop("opponent_position_sigma_m")
    for key, column in columns.items():
        setattr(cfg, key, column)
    keep_row = periodic_gaps(train.meas_stamp_ns, TUNE_GAP_S, TUNE_PERIOD_S)
    keep = np.tile(keep_row, (len(combos), 1))
    filt = OpponentFilter(len(combos), cfg)
    trace = simulate_filter(train, filt, keep, position_sigma, record_stride=2)
    rows = []
    for r, combo in enumerate(combos):
        s = score_trace(train, trace, r)
        rows.append({**dict(zip(keys, combo, strict=True)), "objective": objective(s), "score": s})
    rows.sort(key=lambda row: row["objective"])
    return rows


# ----------------------------------------------------------------------------------- report

SCENARIOS = {"natural": 0.0, "gap_0.2": 0.2, "gap_1": 1.0, "gap_3": 3.0}
SCENARIO_PERIOD_S = 8.0


def tuned_config() -> tuple[OpponentFilterConfig, float]:
    """The opponent fields from config/_common.toml as the C++ loads them, and the opponent
    keypoint position sigma (the mirror takes the measurement variance per call)."""
    est = common_config()["robot_filter"]["motion_estimator"]
    cfg = OpponentFilterConfig(opponent_max_coast_s=opponent_max_coast_s(), render_lead_s=lead_s())
    for key in ("opponent_accel_psd", "opponent_yaw_accel_psd", "opponent_heading_sigma_rad"):
        if key in est:
            setattr(cfg, key, float(est[key]))
    return cfg, float(est.get("opponent_position_sigma_m", 0.03))


def scenario_keep(data: SessionData) -> np.ndarray:
    return np.stack(
        [periodic_gaps(data.meas_stamp_ns, gap, SCENARIO_PERIOD_S) for gap in SCENARIOS.values()]
    )


def run_report(sessions: list[SessionData], cfg: OpponentFilterConfig, sigma: float) -> dict:
    report: dict = {
        "config": {**{k: _scalar(v) for k, v in vars(cfg).items()}, "position_sigma_m": sigma},
        "sessions": {},
    }
    for data in sessions:
        keep = scenario_keep(data)
        b = keep.shape[0]
        traces = {
            "hold": simulate_hold(data, keep, cfg.render_lead_s),
            "kalman4": simulate_filter(data, LegacyOpponentFilter(b, cfg), keep, 0.03),
            "phase1": simulate_filter(data, OpponentFilter(b, cfg), keep, sigma),
        }
        per = {}
        for arm, trace in traces.items():
            per[arm] = {s: score_trace(data, trace, r) for r, s in enumerate(SCENARIOS)}
        report["sessions"][data.name] = {"role": data.role, "arms": per}
    return report


def _scalar(v: object) -> object:
    if isinstance(v, torch.Tensor):
        return v.tolist()
    return v


def markdown_tables(report: dict) -> str:
    lines = []
    for name, sess in report["sessions"].items():
        lines.append(f"### {name} ({sess['role']})\n")
        lines.append("Position error p50 / p90 (mm) by time since the last measurement arrived:\n")
        lines.append("| arm | scenario | " + " | ".join(GAP_LABELS) + " |")
        lines.append("| --- | --- | " + " | ".join("---" for _ in GAP_LABELS) + " |")
        for arm, scen in sess["arms"].items():
            for sname, s in scen.items():
                cells = []
                for label in GAP_LABELS:
                    g = s["by_gap"][label]
                    cells.append(
                        f"{g['pos_p50_mm']:.0f} / {g['pos_p90_mm']:.0f}" if g.get("n") else "-"
                    )
                lines.append(f"| {arm} | {sname} | " + " | ".join(cells) + " |")
        lines.append("")
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=["grid", "report"])
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    if args.command == "grid":
        train = load_session("2026-10-15_00-29-00")
        rows = run_grid(train)
        (OUT / "grid.json").write_text(json.dumps(rows, indent=1))
        for row in rows[:10]:
            s = row["score"]
            print(
                f"q={row['opponent_accel_psd']:g} qy={row['opponent_yaw_accel_psd']:g} "
                f"hs={row['opponent_heading_sigma_rad']:g} ps={row['opponent_position_sigma_m']:g} "
                f"J={row['objective']:.2f} "
                f"fresh {s['by_gap']['0']['pos_p50_mm']:.1f}mm "
                f"{s['by_gap']['0']['yaw_p50_deg']:.1f}deg "
                f"0.35s {s['by_gap']['0.35']['pos_p50_mm']:.1f}mm "
                f"{s['by_gap']['0.35']['yaw_p50_deg']:.1f}deg "
                f"cov pos {s['all'].get('cover2_pos', 0):.2f} "
                f"yaw {s['all'].get('cover2_yaw', 0):.2f} "
                f"NIS {s['nis']['joint_mean']:.2f}/{s['nis']['joint_dof']:.2f}"
            )
    else:
        sessions = [load_session(n) for n in ("2026-10-15_00-29-00", "2026-10-15_00-20-58")]
        report = run_report(sessions, *tuned_config())
        (OUT / "report.json").write_text(json.dumps(report, indent=1))
        (OUT / "report.md").write_text(markdown_tables(report))
        print(markdown_tables(report))


if __name__ == "__main__":
    main()
