"""Plant fitting for velocity jig sessions: loading, per-phase fits, joint fit, scoring."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

import numpy as np
from scipy.optimize import least_squares

from auto_battlebot.calibration.jig.velocity_jig import (
    JigCalibration,
    Run,
    Session,
    load_runs,
    load_session_dir,
)
from auto_battlebot.control.plant import (
    FULL_MODEL,
    PARAM_BOUNDS,
    ModelStructure,
    PlantParams,
    WindowErrors,
    WindowSet,
    concat_windows,
    effective_command,
    make_windows,
    predict_windows,
)

# Per-sample motion floors. Below these the measurement is noise, whatever the command says.
MOTION_EPS_LIN = 0.02  # m/s
MOTION_EPS_ANG = 0.05  # rad/s
ONSET_SIGMA = 5.0  # "moving" is 5 sigma above the stationary noise, per the plan
MIN_HOLD_S = 0.4  # shortest command hold worth fitting a steady state to

DEADZONE_KINDS = ("staircase",)
STEP_KINDS = ("step",)
COAST_KINDS = ("step", "coast")
GRID_KINDS = ("grid",)
DRIFT_KINDS = ("step", "staircase", "coast", "trim")

PARAM_SOURCES: dict[str, tuple[tuple[str, str], ...]] = {
    "dz_lin_fwd": (("staircase", "linear"),),
    "dz_lin_rev": (("staircase", "linear"),),
    "dz_ang_l": (("staircase", "angular"),),
    "dz_ang_r": (("staircase", "angular"),),
    "k_fwd": (("step", "linear"),),
    "k_rev": (("step", "linear"),),
    "k_ang": (("step", "angular"),),
    "tau_lin_a": (("step", "linear"),),
    "tau_ang_a": (("step", "angular"),),
    "tau_lin_d": (("step", "linear"), ("coast", "linear")),
    "tau_ang_d": (("step", "angular"), ("coast", "angular")),
    "c_sb": (("grid", "combined"),),
    "c_ad": (("grid", "combined"),),
    "c_drift": (("step", "linear"), ("grid", "combined")),
    "c_drift_bias": (("step", "linear"), ("grid", "combined")),
    "delay_s": (("step", "*"), ("staircase", "*"), ("grid", "combined"), ("prbs", "*")),
}


@dataclass
class Loaded:
    """Every usable run across every session, plus why the rest were dropped."""

    sessions: list[Session] = field(default_factory=list)
    runs: list[Run] = field(default_factory=list)
    session_ids: list[str] = field(default_factory=list)
    excluded: list[tuple[str, str]] = field(default_factory=list)

    def select(
        self,
        *,
        kinds: Sequence[str] | None = None,
        channels: Sequence[str] | None = None,
        roles: Sequence[str] | None = None,
    ) -> list[Run]:
        """Runs matching a routing selector. None means any."""
        return [r for r in self.runs if r.spec.matches(kinds=kinds, channels=channels, roles=roles)]

    def by_waveform(self) -> dict[str, list[Run]]:
        out: dict[str, list[Run]] = {}
        for run in self.runs:
            out.setdefault(run.waveform, []).append(run)
        return out

    def by_kind_channel(self) -> dict[tuple[str, str], list[Run]]:
        out: dict[tuple[str, str], list[Run]] = {}
        for run in self.runs:
            out.setdefault((run.kind, run.channel), []).append(run)
        return out

    def split_run(self, index: int) -> tuple[list[Run], list[Run]]:
        """(train, test) for a leave-one-run-out jackknife."""
        held = self.runs[index]
        return [r for i, r in enumerate(self.runs) if i != index], [held]

    def split_session(self, held: str) -> tuple[list[Run], list[Run]]:
        """(train, test) for leave-one-session-out."""
        train = [r for r, s in zip(self.runs, self.session_ids) if s != held]
        test = [r for r, s in zip(self.runs, self.session_ids) if s == held]
        return train, test


def load_all(
    session_dirs: Sequence[Path],
    calib: JigCalibration,
    fit_hz: float,
    keep_bad: bool,
    smooth_s: float = 0.02,
    commands: str = "measured",
    max_saturation: float = 0.0,
) -> Loaded:
    """Load every capture directory. A session is a directory, so its logs are inside it."""
    out = Loaded()
    for session_dir in session_dirs:
        session = load_session_dir(session_dir, commands=commands)
        out.sessions.append(session)
        runs, skipped = load_runs(session, calib, fit_hz=fit_hz, smooth_s=smooth_s)
        for record, why in skipped:
            out.excluded.append((record.log_file or record.run_id, why))
        for name in session.orphans:
            out.excluded.append((name, "log file with no sidecar TOML"))
        for run in runs:
            problems = run.quality.problems(max_saturation=max_saturation)
            if problems and not keep_bad:
                out.excluded.append((run.name, "; ".join(problems)))
                continue
            if problems:
                print(f"  keeping {run.name} despite: {'; '.join(problems)}")
            out.runs.append(run)
            out.session_ids.append(session.session_id)
    return out


def _runs_of(mask: np.ndarray) -> list[tuple[int, int]]:
    idx = np.flatnonzero(mask)
    if len(idx) == 0:
        return []
    breaks = np.flatnonzero(np.diff(idx) > 1)
    starts = np.concatenate([[idx[0]], idx[breaks + 1]])
    stops = np.concatenate([idx[breaks], [idx[-1]]])
    return list(zip(starts.tolist(), (stops + 1).tolist()))


def _eps(run: Run, channel: str) -> float:
    if channel == "lin":
        return max(MOTION_EPS_LIN, ONSET_SIGMA * run.v_noise)
    return max(MOTION_EPS_ANG, ONSET_SIGMA * run.w_noise)


def _channel(run: Run, channel: str) -> tuple[np.ndarray, np.ndarray]:
    if channel == "lin":
        return run.cmd_lin, run.v
    return run.cmd_ang, run.w


@dataclass
class HeldSegment:
    """One held command level with its measured steady state and rise."""

    run: str
    channel: str
    level: float  # commanded amplitude, signed
    u_eff: float  # after deadzone removal (filled once the deadzone is known)
    v_ss: float  # steady-state |velocity|
    tau: float  # first-order rise constant, nan when not fittable
    other: float  # |command| on the other channel, the gate for "is this a pure run"
    # Signed version of the same. The magnitude alone cannot tell a left turn from a right
    # one, and turn direction is exactly what separates angular droop from straight-line
    # drift, so the coupling fit needs the sign kept.
    other_signed: float
    n: int
    t0: float


def _rise_tau(mv: np.ndarray, v_ss: float, dt: float, eps: float) -> float:
    """First-order rise constant from the leading contiguous 15-90% band, log-linear."""
    inband = (mv > 0.15 * v_ss) & (mv < 0.9 * v_ss) & ((v_ss - mv) > 0.5 * eps)
    runs = _runs_of(inband)
    if not runs:
        return float("nan")
    a0, b0 = runs[0]
    sel = np.arange(a0, b0)
    if len(sel) < 4:
        return float("nan")
    slope = float(np.polyfit(sel * dt, np.log(v_ss - mv[sel]), 1)[0])
    return -1.0 / slope if slope < 0 else float("nan")


def held_segments(run: Run, channel: str, min_level: float = 0.0) -> list[HeldSegment]:
    """Every contiguous stretch where this channel held a constant nonzero command."""
    cmd, vel = _channel(run, channel)
    other = run.cmd_ang if channel == "lin" else run.cmd_lin
    eps = _eps(run, channel)
    min_hold = max(5, int(round(MIN_HOLD_S / run.dt)))

    active = np.abs(cmd) > max(1e-3, min_level)
    changed = np.concatenate([[False], np.abs(np.diff(cmd)) > 1e-4])
    seg_id = np.cumsum(changed)
    out: list[HeldSegment] = []
    for a, b in _runs_of(active):
        start = a
        while start < b:
            same = seg_id[start:b] == seg_id[start]
            stop = start + int(np.count_nonzero(same))
            n = stop - start
            level = float(np.median(cmd[start:stop]))
            if n >= min_hold and abs(level) >= min_level:
                mv = np.abs(vel[start:stop])
                tail = slice(int(0.6 * n), n)
                v_ss = float(np.median(mv[tail]))
                out.append(
                    HeldSegment(
                        run=run.name,
                        channel=channel,
                        level=level,
                        u_eff=float("nan"),
                        v_ss=v_ss,
                        tau=(
                            _rise_tau(mv, v_ss, run.dt, eps)
                            if v_ss > max(3 * eps, 1e-3)
                            else float("nan")
                        ),
                        other=float(np.median(np.abs(other[start:stop]))),
                        other_signed=float(np.median(other[start:stop])),
                        n=n,
                        t0=float(run.t[start]),
                    )
                )
            start = stop
    return out


def coast_taus(run: Run, channel: str) -> list[float]:
    """Decel constants from every zero-command coast tail in the run."""
    cmd, vel = _channel(run, channel)
    eps = _eps(run, channel)
    out: list[float] = []
    for a, b in _runs_of(np.abs(cmd) < 0.02):
        n = b - a
        if n < max(4, int(round(0.2 / run.dt))):
            continue
        mv = np.abs(vel[a:b])
        if mv[0] < 5 * eps:
            continue
        k = 0
        while k < n and mv[k] > max(3 * eps, 0.1 * mv[0]):
            k += 1
        if k < 4:
            continue
        tt = np.arange(k) * run.dt
        slope = float(np.polyfit(tt, np.log(mv[:k]), 1)[0])
        if slope < 0:
            out.append(-1.0 / slope)
    return out


@dataclass
class Estimate:
    """A parameter with the spread it was measured over."""

    value: float
    spread: float = float("nan")
    n: int = 0
    note: str = ""

    def __str__(self) -> str:
        if math.isnan(self.value):
            return f"n/a ({self.note})" if self.note else "n/a"
        text = f"{self.value:.4g}"
        if self.n:
            text += f" +/- {self.spread:.3g} (n={self.n})"
        if self.note:
            text += f"  [{self.note}]"
        return text


def _pool(values: Sequence[float]) -> Estimate:
    """Median of the usable segment values, with a spread that survives one bad segment."""
    seen = len(values)
    vals = np.array([v for v in values if np.isfinite(v)], dtype=float)
    if len(vals) == 0:
        return Estimate(float("nan"), note=f"no usable segments out of {seen}")
    kept, cut = vals, 0
    # Quartiles need a real sample behind them; below this, one value moves the fence.
    if len(vals) >= 8:
        q1, q3 = np.percentile(vals, [25, 75])
        span = q3 - q1
        keep = (vals >= q1 - 1.5 * span) & (vals <= q3 + 1.5 * span)
        if keep.any():
            cut = int(np.count_nonzero(~keep))
            kept = vals[keep]
    notes = []
    if cut:
        notes.append(f"{cut} outlier{'s' if cut > 1 else ''} cut")
    if seen - len(vals):
        notes.append(f"{seen - len(vals)}/{seen} segments gave no value")
    return Estimate(float(np.median(kept)), float(np.std(kept)), len(kept), "; ".join(notes))


def fit_deadzone(runs: Sequence[Run], channel: str, sign: float) -> Estimate:
    """Deadzone from the staircase: the command level where motion first clears the noise."""
    levels: dict[float, tuple[float, int]] = {}
    noise: list[float] = []
    for run in runs:
        noise.append(run.v_noise if channel == "lin" else run.w_noise)
        for seg in held_segments(run, channel):
            if np.sign(seg.level) != sign:
                continue
            key = round(abs(seg.level), 4)
            prev = levels.get(key)
            if prev is None or seg.n > prev[1]:
                levels[key] = (seg.v_ss, seg.n)
    if len(levels) < 3:
        return Estimate(float("nan"), note=f"{len(levels)} staircase levels, need 3")

    floor = MOTION_EPS_LIN if channel == "lin" else MOTION_EPS_ANG
    sigma = float(np.median(noise)) if noise else 0.0
    ordered = sorted(levels.items())
    prev_level, prev_speed = 0.0, 0.0
    for level, (speed, n) in ordered:
        thresh = max(floor, ONSET_SIGMA * sigma / math.sqrt(max(n, 1)))
        if speed > thresh:
            if speed <= prev_speed:
                return Estimate(level, note="non-monotone staircase")
            frac = (thresh - prev_speed) / (speed - prev_speed)
            return Estimate(
                prev_level + frac * (level - prev_level),
                spread=level - prev_level,
                n=len(ordered),
                note=f"crossing between {prev_level:.3f} and {level:.3f}",
            )
        prev_level, prev_speed = level, speed
    return Estimate(float("nan"), note="never cleared the motion threshold")


def fit_gain(
    segs: Sequence[HeldSegment], dz_pos: float, dz_neg: float, sign: float
) -> tuple[Estimate, list[HeldSegment]]:
    """Max speed from steady state against effective command, through the origin."""
    used: list[HeldSegment] = []
    for seg in segs:
        if np.sign(seg.level) != sign or seg.other > 0.02:
            continue
        u_eff = float(effective_command(np.array(seg.level), dz_pos, dz_neg))
        if abs(u_eff) < 1e-3 or seg.v_ss <= 1e-3:
            continue
        seg.u_eff = u_eff
        used.append(seg)
    if len(used) < 2:
        return Estimate(float("nan"), note=f"{len(used)} held segments"), used
    ratios = [s.v_ss / abs(s.u_eff) for s in used]
    x = np.array([abs(s.u_eff) for s in used])
    y = np.array([s.v_ss for s in used])
    slope = float(np.sum(x * y) / np.sum(x * x))
    return Estimate(slope, float(np.std(ratios)), len(used)), used


def _coupling_cells(
    runs: Sequence[Run], p: PlantParams
) -> list[tuple[float, float, float, float, float]]:
    """One cell per held linear segment of the E13 grid.

    Each is (u_lin_eff, u_ang_eff, v_ss, w_ss, angular command as issued). The last one is
    what mirror matching keys on: the effective command has the per-side deadzone removed,
    and those two deadzones are not equal, so +0.08 and -0.08 come out at +0.0650 and
    -0.0573 and never match. The grid is symmetric in what was commanded, never in what is
    left after deadzone removal.
    """
    cells: list[tuple[float, float, float, float, float]] = []
    for run in runs:
        for seg in held_segments(run, "lin"):
            lin_eff = float(effective_command(np.array(seg.level), p.dz_lin_fwd, p.dz_lin_rev))
            if seg.other <= 0.02:
                cells.append((lin_eff, 0.0, seg.v_ss, 0.0, 0.0))
                continue
            ang_eff = float(effective_command(np.array(seg.other_signed), p.dz_ang_l, p.dz_ang_r))
            mask = (run.t >= seg.t0) & (run.t < seg.t0 + seg.n * run.dt)
            w_ss = float(np.median(run.w[mask])) if np.count_nonzero(mask) else 0.0
            cells.append((lin_eff, ang_eff, seg.v_ss, w_ss, float(seg.other_signed)))
    return cells


def _steer_brake_points(
    cells: Sequence[tuple[float, float, float, float, float]], p: PlantParams
) -> tuple[list[float], list[float]]:
    """(|u_ang_eff|, fractional speed loss) for every cell that turns and drives at once."""
    xs: list[float] = []
    ys: list[float] = []
    for lin_eff, ang_eff, v_ss, _w_ss, _cmd in cells:
        if abs(lin_eff) <= 1e-3 or abs(ang_eff) <= 1e-3:
            continue
        k = p.k_fwd if lin_eff > 0 else p.k_rev
        v0 = k * abs(lin_eff)
        if v0 > 1e-3:
            xs.append(abs(ang_eff))
            ys.append(1.0 - v_ss / v0)
    return xs, ys


def fit_coupling(
    runs: Sequence[Run], p: PlantParams
) -> tuple[Estimate, Estimate, list[tuple[float, float, float, float, float]]]:
    """Steer-brake and angular droop from the E13 grid."""
    cells = _coupling_cells(runs, p)
    sb_x, sb_y = _steer_brake_points(cells, p)
    ad_x: list[float] = []
    ad_y: list[float] = []

    pairs = _mirror_pairs(cells)
    for lin_eff, ang_mag, w_pos, w_neg in pairs:
        w0 = p.k_ang * ang_mag
        if w0 <= 1e-3:
            continue
        commanded = 0.5 * (w_pos - w_neg)  # drift cancels
        ad_x.append(abs(lin_eff))
        ad_y.append(1.0 - commanded / w0)

    def _slope(xs: list[float], ys: list[float]) -> Estimate:
        if len(xs) < 3:
            return Estimate(float("nan"), note=f"{len(xs)} grid cells, need 3")
        x = np.array(xs)
        y = np.array(ys)
        slope = float(np.sum(x * y) / np.sum(x * x))
        resid = y - slope * x
        se = float(np.sqrt(np.sum(resid**2) / max(len(x) - 1, 1) / np.sum(x**2)))
        note = "consistent with zero" if abs(slope) < 2 * se else ""
        return Estimate(slope, se, len(x), note)

    ad = _slope(ad_x, ad_y)
    if not ad_x:
        ad = Estimate(
            float("nan"),
            note=(
                "grid is not symmetric in angular sign, so angular droop and "
                "straight-line drift cannot be separated from it"
            ),
        )
    return _slope(sb_x, sb_y), ad, cells


def _mirror_pairs(
    cells: Sequence[tuple[float, float, float, float, float]],
) -> list[tuple[float, float, float, float]]:
    """Match each grid cell to its mirror across the angular axis."""
    index: dict[tuple[int, int], tuple[float, float]] = {}
    for lin_eff, ang_eff, _v, w_ss, ang_cmd in cells:
        if abs(ang_cmd) <= 1e-3:
            continue
        index[(int(round(lin_eff * 1000)), int(round(ang_cmd * 1000)))] = (w_ss, ang_eff)

    out: list[tuple[float, float, float, float]] = []
    for (lin_key, ang_key), (w_pos, eff_pos) in index.items():
        if ang_key <= 0:
            continue
        mirror = index.get((lin_key, -ang_key))
        if mirror is None:
            continue
        w_neg, eff_neg = mirror
        mag = 0.5 * (abs(eff_pos) + abs(eff_neg))
        if mag <= 1e-3:
            continue
        out.append((lin_key / 1000.0, mag, w_pos, w_neg))
    return out


def fit_drift(runs: Sequence[Run], p: PlantParams) -> tuple[Estimate, Estimate, list]:
    """Straight-line drift: the yaw a pure forward command produces on its own."""
    points: list[tuple[float, float]] = []  # (u_lin_eff, w_ss)
    for run in runs:
        for seg in held_segments(run, "lin"):
            if seg.other > 0.02:
                continue
            lin_eff = float(effective_command(np.array(seg.level), p.dz_lin_fwd, p.dz_lin_rev))
            if abs(lin_eff) < 1e-3:
                continue
            mask = (run.t >= seg.t0) & (run.t < seg.t0 + seg.n * run.dt)
            if not np.count_nonzero(mask):
                continue
            points.append((lin_eff, float(np.median(run.w[mask]))))

    if len(points) < 4:
        note = f"{len(points)} straight held segments, need 4"
        return Estimate(float("nan"), note=note), Estimate(float("nan"), note=note), points

    x = np.array([u for u, _ in points])
    y = np.array([w for _, w in points])
    if not (np.any(x > 0) and np.any(x < 0)):
        note = "only one direction driven, so drift and drift bias are not separable"
        return Estimate(float("nan"), note=note), Estimate(float("nan"), note=note), points

    design = np.stack([x, np.sign(x)], axis=1)
    coeff, *_ = np.linalg.lstsq(design, y, rcond=None)
    resid = y - design @ coeff
    dof = max(len(x) - 2, 1)
    try:
        cov = np.linalg.inv(design.T @ design) * float(resid @ resid) / dof
        se = np.sqrt(np.diag(cov))
    except np.linalg.LinAlgError:
        se = np.array([float("nan"), float("nan")])

    def _est(value: float, err: float) -> Estimate:
        note = "consistent with zero" if np.isfinite(err) and abs(value) < 2 * err else ""
        return Estimate(float(value), float(err), len(x), note)

    return _est(coeff[0], se[0]), _est(coeff[1], se[1]), points


@dataclass
class DelayStack:
    """Pooled command-edge onset stack, the stage A transport delay estimate."""

    lags_ms: np.ndarray
    stack: np.ndarray
    smooth: np.ndarray
    count: int
    snr: float
    lag_ms: float
    clock_measured: bool


def fit_delay(runs: Sequence[Run]) -> DelayStack:
    """Transport delay from every clean command edge in the session, pooled."""
    dt = runs[0].dt if runs else 0.005
    pre = 4
    max_lag = max(5, int(round(0.24 / dt)))
    win = pre + max_lag + 1
    lags_ms = (np.arange(win) - pre) * dt * 1000.0
    stack = np.zeros(win)
    count = 0
    clock_measured = all(r.truth.clock.measured for r in runs) if runs else False

    for run in runs:
        for cmd, vel in ((run.cmd_lin, run.v_raw), (run.cmd_ang, run.w)):
            accel = np.concatenate([[0.0], np.diff(vel)]) / run.dt
            scale = float(np.std(accel))
            if scale < 1e-9:
                continue
            dcmd = np.diff(cmd)
            for i in np.flatnonzero(np.abs(dcmd) > 0.15):
                e = i + 1
                lo, hi = e - pre, e + max_lag + 1
                if lo < 0 or hi > len(accel):
                    continue
                if max(abs(cmd[e]), abs(cmd[e - 1])) < 0.1:
                    continue
                stack += np.sign(dcmd[i]) * accel[lo:hi] / scale
                count += 1

    if count < 6:
        return DelayStack(lags_ms, stack, stack, count, 0.0, float("nan"), clock_measured)

    stack = stack / count
    stack = stack - float(np.mean(stack[:pre]))
    smooth = np.convolve(stack, np.array([0.25, 0.5, 0.25]), mode="same")
    causal = smooth[pre:]
    peak_i = int(np.argmax(causal))
    delta = 0.0
    if 0 < peak_i < len(causal) - 1:
        a, b, c = causal[peak_i - 1], causal[peak_i], causal[peak_i + 1]
        denom = a - 2.0 * b + c
        if abs(denom) > 1e-9:
            delta = float(np.clip(0.5 * (a - c) / denom, -1.0, 1.0))
    lag_ms = (peak_i + delta - 0.5) * dt * 1000.0
    noise = float(np.std(stack[:pre])) or 1e-9
    return DelayStack(
        lags_ms, stack, smooth, count, float(causal[peak_i] / noise), lag_ms, clock_measured
    )


@dataclass
class StageA:
    params: PlantParams
    estimates: dict[str, Estimate]
    lin_segments: list[HeldSegment]
    ang_segments: list[HeldSegment]
    delay: DelayStack
    coupling_cells: list[tuple[float, float, float, float, float]]
    # (u_lin_eff, w_ss) over straight held segments, the evidence behind the drift terms.
    drift_points: list[tuple[float, float]] = field(default_factory=list)


def stage_a(loaded: Loaded, prior: PlantParams) -> StageA:
    """Per-phase fits, in the order each depends on the last."""
    est: dict[str, Estimate] = {}

    dz_runs_lin = loaded.select(kinds=DEADZONE_KINDS, channels=("linear",), roles=("fit",))
    dz_runs_ang = loaded.select(kinds=DEADZONE_KINDS, channels=("angular",), roles=("fit",))
    est["dz_lin_fwd"] = fit_deadzone(dz_runs_lin, "lin", 1.0)
    est["dz_lin_rev"] = fit_deadzone(dz_runs_lin, "lin", -1.0)
    est["dz_ang_l"] = fit_deadzone(dz_runs_ang, "ang", 1.0)
    est["dz_ang_r"] = fit_deadzone(dz_runs_ang, "ang", -1.0)

    def _dz(name: str, fallback: float) -> float:
        value = est[name].value
        return fallback if math.isnan(value) else value

    p = prior.replace(
        dz_lin_fwd=_dz("dz_lin_fwd", prior.dz_lin_fwd),
        dz_lin_rev=_dz("dz_lin_rev", prior.dz_lin_rev),
        dz_ang_l=_dz("dz_ang_l", prior.dz_ang_l),
        dz_ang_r=_dz("dz_ang_r", prior.dz_ang_r),
    )

    # Above the radio's readback quantization, below the smallest commanded step in any
    # ladder (0.1 linear after the reverse cap, 0.244 angular), so this drops rest cells and
    # nothing that was actually driven.
    step_floor = 0.01
    lin_segs: list[HeldSegment] = []
    for run in loaded.select(kinds=STEP_KINDS, channels=("linear",), roles=("fit",)):
        if run.encoder_valid:
            lin_segs += held_segments(run, "lin", min_level=step_floor)
    ang_segs: list[HeldSegment] = []
    for run in loaded.select(kinds=STEP_KINDS, channels=("angular",), roles=("fit",)):
        ang_segs += held_segments(run, "ang", min_level=step_floor)

    est["k_fwd"], used_f = fit_gain(lin_segs, p.dz_lin_fwd, p.dz_lin_rev, 1.0)
    est["k_rev"], used_r = fit_gain(lin_segs, p.dz_lin_fwd, p.dz_lin_rev, -1.0)
    k_ang_l, used_l = fit_gain(ang_segs, p.dz_ang_l, p.dz_ang_r, 1.0)
    k_ang_r, used_rr = fit_gain(ang_segs, p.dz_ang_l, p.dz_ang_r, -1.0)
    both = [v for v in (k_ang_l.value, k_ang_r.value) if np.isfinite(v)]
    est["k_ang"] = (
        Estimate(float(np.mean(both)), float(np.std(both)), len(used_l) + len(used_rr))
        if both
        else Estimate(float("nan"), note="no angular held segments")
    )
    est["k_ang_left"] = k_ang_l
    est["k_ang_right"] = k_ang_r

    est["tau_lin_a"] = _pool([s.tau for s in used_f + used_r])
    est["tau_ang_a"] = _pool([s.tau for s in used_l + used_rr])

    lin_coast: list[float] = []
    for run in loaded.select(kinds=COAST_KINDS, channels=("linear",), roles=("fit",)):
        if run.encoder_valid:
            lin_coast += coast_taus(run, "lin")
    ang_coast: list[float] = []
    for run in loaded.select(kinds=COAST_KINDS, channels=("angular",), roles=("fit",)):
        ang_coast += coast_taus(run, "ang")
    est["tau_lin_d"] = _pool(lin_coast)
    est["tau_ang_d"] = _pool(ang_coast)

    p = p.replace(
        k_fwd=_value(est["k_fwd"], p.k_fwd),
        k_rev=_value(est["k_rev"], p.k_rev),
        k_ang=_value(est["k_ang"], p.k_ang),
        tau_lin_a=_value(est["tau_lin_a"], p.tau_lin_a),
        tau_lin_d=_value(est["tau_lin_d"], p.tau_lin_d),
        tau_ang_a=_value(est["tau_ang_a"], p.tau_ang_a),
        tau_ang_d=_value(est["tau_ang_d"], p.tau_ang_d),
    )

    c_sb, c_ad, cells = fit_coupling(loaded.select(kinds=GRID_KINDS, roles=("fit",)), p)
    est["c_sb"] = c_sb
    est["c_ad"] = c_ad
    p = p.replace(
        c_sb=0.0 if c_sb.note else _value(c_sb, 0.0),
        c_ad=0.0 if c_ad.note else _value(c_ad, 0.0),
    )

    drift_runs = loaded.select(kinds=DRIFT_KINDS, channels=("linear",), roles=("fit",))
    c_drift, c_drift_bias, drift_points = fit_drift(drift_runs, p)
    est["c_drift"] = c_drift
    est["c_drift_bias"] = c_drift_bias
    p = p.replace(
        c_drift=0.0 if c_drift.note else _value(c_drift, 0.0),
        c_drift_bias=0.0 if c_drift_bias.note else _value(c_drift_bias, 0.0),
    )

    delay = fit_delay(loaded.runs)
    est["delay_s"] = Estimate(
        delay.lag_ms / 1000.0 if np.isfinite(delay.lag_ms) else float("nan"),
        n=delay.count,
        note=(
            "onset stack"
            if delay.clock_measured
            else "onset stack on an unaligned clock: this is offset + delay, not delay"
        ),
    )
    if np.isfinite(delay.lag_ms) and delay.clock_measured:
        p = p.replace(delay_s=delay.lag_ms / 1000.0)

    return StageA(p, est, lin_segs, ang_segs, delay, cells, drift_points)


def _value(est: Estimate, fallback: float) -> float:
    return fallback if math.isnan(est.value) else est.value


@dataclass
class FitWeights:
    """Residual scaling. Position in meters and heading in radians are not commensurate."""

    position: float = 0.05  # m
    heading: float = 0.05  # rad


def build_windows(
    runs: Sequence[Run],
    delay_s: float,
    horizons: Sequence[float],
    stride_s: float,
    max_windows: int,
    require_encoder: bool = True,
) -> WindowSet | None:
    """Cut windows out of every run and stack them into one batch."""
    sets = []
    for index, run in enumerate(runs):
        if require_encoder and not run.encoder_valid:
            continue
        if not require_encoder and run.encoder_valid:
            continue
        valid = run.commanded
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
            # Index into `runs` as passed in, so the report can ask which waveform a
            # residual came from. Without it, concatenating runs makes every per-run
            # question unanswerable.
            origin=index,
        )
        if ws is not None:
            sets.append(ws)
    if not sets:
        return None
    return concat_windows(sets).subsample(max_windows)


def window_delay(params: PlantParams, structure: ModelStructure) -> float:
    """The delay the window command matrices must be built with."""
    return structure.apply(params).delay_s


def residual_vector(err: WindowErrors, weights: FitWeights) -> np.ndarray:
    return np.concatenate(
        [
            (err.along / weights.position).ravel(),
            (err.cross / weights.position).ravel(),
            (err.heading / weights.heading).ravel(),
        ]
    )


def joint_fit(
    windows: WindowSet,
    start: PlantParams,
    structure: ModelStructure,
    weights: FitWeights,
    max_nfev: int | None = None,
    bounds: dict[str, tuple[float, float]] | None = None,
) -> tuple[PlantParams, float]:
    """Minimize windowed open-loop error over the structure's free parameters."""
    names = structure.free_names()
    x0 = start.to_vector(names)
    table = {**PARAM_BOUNDS, **(bounds or {})}
    lo = np.array([table[n][0] for n in names])
    hi = np.array([table[n][1] for n in names])
    x0 = np.clip(x0, lo + 1e-9, hi - 1e-9)

    def objective(x: np.ndarray) -> np.ndarray:
        p = start.with_vector(names, x)
        err = predict_windows(windows, p, structure)
        return residual_vector(err, weights)

    result = least_squares(
        objective,
        x0,
        bounds=(lo, hi),
        loss="soft_l1",
        f_scale=1.0,
        x_scale="jac",
        max_nfev=max_nfev,
    )
    return start.with_vector(names, result.x), float(result.cost)


@dataclass
class DelayProfile:
    delays: np.ndarray
    costs: np.ndarray
    best_delay: float
    best_params: PlantParams


def profile_delay(
    runs: Sequence[Run],
    start: PlantParams,
    structure: ModelStructure,
    weights: FitWeights,
    horizons: Sequence[float],
    *,
    stride_s: float,
    max_windows: int,
    delay_grid: np.ndarray,
    max_nfev: int,
) -> DelayProfile:
    """Grid search the transport delay, refitting the continuous parameters at each point."""
    costs = []
    fits = []
    for delay in delay_grid:
        windows = build_windows(runs, float(delay), horizons, stride_s, max_windows)
        if windows is None:
            costs.append(float("inf"))
            fits.append(start)
            continue
        params, cost = joint_fit(
            windows, start.replace(delay_s=float(delay)), structure, weights, max_nfev=max_nfev
        )
        costs.append(cost)
        fits.append(params.replace(delay_s=float(delay)))
        print(f"    delay {delay * 1000:6.1f} ms -> cost {cost:.4f}")
    best = int(np.argmin(costs))
    return DelayProfile(
        np.asarray(delay_grid), np.asarray(costs), float(delay_grid[best]), fits[best]
    )


@dataclass
class HorizonReport:
    horizons: np.ndarray
    rows: dict[str, np.ndarray]  # metric name -> per-horizon values


def score(err: WindowErrors) -> HorizonReport:
    """Bias, RMSE, and percentiles per horizon, decomposed along-track and cross-track."""
    rows: dict[str, np.ndarray] = {}
    for name, data in (
        ("along_bias_mm", err.along * 1e3),
        ("along_rmse_mm", err.along * 1e3),
        ("cross_bias_mm", err.cross * 1e3),
        ("cross_rmse_mm", err.cross * 1e3),
        ("pos_rmse_mm", err.position * 1e3),
        ("pos_p95_mm", err.position * 1e3),
        ("pos_p99_mm", err.position * 1e3),
        ("head_rmse_deg", np.degrees(err.heading)),
        ("head_p95_deg", np.degrees(np.abs(err.heading))),
    ):
        if name.endswith("bias_mm"):
            rows[name] = np.mean(data, axis=0)
        elif name.endswith("p95_mm") or name.endswith("p95_deg"):
            rows[name] = np.percentile(np.abs(data), 95, axis=0)
        elif name.endswith("p99_mm"):
            rows[name] = np.percentile(np.abs(data), 99, axis=0)
        else:
            rows[name] = np.sqrt(np.mean(data**2, axis=0))
    return HorizonReport(err.horizons, rows)


def print_horizon_table(title: str, report: HorizonReport) -> None:
    print(f"\n  {title}")
    header = "    horizon ms |" + "".join(f"{h * 1000:9.0f}" for h in report.horizons)
    print(header)
    print("    " + "-" * (len(header) - 4))
    for name, values in report.rows.items():
        print(f"    {name:>11} |" + "".join(f"{v:9.2f}" for v in values))


def residual_autocorrelation(err: WindowErrors, lag: int = 1) -> float:
    """Lag-1 autocorrelation of the longest-horizon position error across window starts."""
    series = err.position[:, -1]
    if len(series) <= lag + 2:
        return float("nan")
    a = series[:-lag] - series.mean()
    b = series[lag:] - series.mean()
    denom = float(np.sqrt(np.sum(a**2) * np.sum(b**2)))
    return float(np.sum(a * b) / denom) if denom > 0 else float("nan")


def print_stage_a(stage: StageA) -> None:
    print("\n=== Stage A: per-phase fits ===")
    order = [
        "dz_lin_fwd",
        "dz_lin_rev",
        "dz_ang_l",
        "dz_ang_r",
        "k_fwd",
        "k_rev",
        "k_ang",
        "k_ang_left",
        "k_ang_right",
        "tau_lin_a",
        "tau_lin_d",
        "tau_ang_a",
        "tau_ang_d",
        "c_sb",
        "c_ad",
        "delay_s",
    ]
    for name in order:
        est = stage.estimates.get(name)
        if est is not None:
            print(f"  {name:12} {est}")
    d = stage.delay
    print(
        f"  delay stack: {d.count} onsets, peak SNR {d.snr:.1f},"
        f" lag {d.lag_ms:.1f} ms, clock {'measured' if d.clock_measured else 'NOT measured'}"
    )


def cross_checks(p: PlantParams, track_width_m: float) -> list[str]:
    """The three sanity checks the plan asks for before believing an angular fit."""
    out: list[str] = []
    bound = 2.0 * p.k_fwd / track_width_m if track_width_m > 0 else float("inf")
    if p.k_ang > bound:
        out.append(
            f"k_ang {p.k_ang:.1f} rad/s exceeds the geometric bound 2*k_fwd/track_width"
            f" = {bound:.1f} rad/s. Either the fit is wrong or the track width is."
        )
    else:
        out.append(f"k_ang {p.k_ang:.1f} rad/s is under the geometric bound {bound:.1f} rad/s.")
    stage2_k_ang, stage2_delay = 61.5, 0.059
    ratio = p.k_ang / stage2_k_ang if stage2_k_ang else float("nan")
    if ratio < 0.8:
        out.append(
            f"k_ang is {100 * (1 - ratio):.0f}% below the stage 2 camera value of 61.5 rad/s."
            " That points at yaw keypoint flips inflating the camera number, which is a finding"
            " about perception, not only about the plant."
        )
    delta_ms = abs(p.delay_s - stage2_delay) * 1e3
    out.append(
        f"delay {p.delay_s * 1e3:.1f} ms vs stage 2's 59 ms, {delta_ms:.1f} ms apart."
        " Two independent methods on independent hardware agreeing is the strongest evidence"
        " available for this number."
    )
    return out


def write_params(
    path: Path, p: PlantParams, stage: StageA, note: str, structure: ModelStructure
) -> None:
    """Write the selected model parameters, with the terms it disables set to zero."""
    p = structure.apply(p)
    disabled = sorted(set(FULL_MODEL.free_names()) - set(structure.free_names()))
    lines = [
        "# Drivetrain plant parameters fit from velocity jig data.",
        f"# {note}",
        "# Consumed by the C++ filter and simulation/kinematic_sim_server.py. One source of",
        "# truth for the plant numbers: edit the fit, not this file.",
        f"# Model {structure.name}. Terms it disables are written as zero"
        + (f": {', '.join(disabled)}." if disabled else "; there are none."),
        "",
        p.to_toml("plant").rstrip(),
        "",
        "[plant.provenance]",
    ]
    for name, est in stage.estimates.items():
        lines.append(f'{name} = "{est}"')
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"\nwrote {path}")
