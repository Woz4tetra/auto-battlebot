"""Fit the drivetrain plant model from velocity jig sessions.

Input is one or more session JSON files exported by the jig web tool plus the directory of
`LOG-N.TXT` files they name, and the bench calibration from runbook block 0. Output is a plant
parameter TOML, a printed report, and an optional diagnostic plot.

The fit runs in two stages, because a cold-start joint fit over twelve parameters will find a
local minimum and look convincing while doing it:

- **Stage A, per-phase.** Each experiment was designed to make one group of parameters dominant,
  so each fits in near-closed form: deadzone from the staircases (E7, E10), gains and rise
  constants from the steps (E8, E11), decel from the coast tails (E9), coupling from the grid
  (E13), transport delay from a pooled onset stack over every command edge. This is what
  `fit_plant_calib.py` did against AprilTag truth, ported to on-robot truth. It also reports
  per-parameter spread, so a parameter that is not actually measured says so.

- **Stage B, joint by simulation error.** Starting from stage A, minimize multi-step open-loop
  prediction error over sliding windows at the horizons the filter coasts over. This optimizes
  exactly the quantity the Kalman filter depends on, which one-step-ahead fitting does not: any
  model with a delay term can fake one-step accuracy.

Transport delay is not smooth in the objective, so it is profiled on a grid rather than handed
to the optimizer, which also produces its profile likelihood curve.

Usage:
    source scripts/activate_python.sh
    python playground/calibration/fit_jig_plant.py \\
        playground/calibration/out/session_A.json \\
        --logs playground/calibration/out/logs_A \\
        --calibration playground/calibration/out/jig_calibration.toml \\
        --out playground/calibration/out/plant_params.toml \\
        --plot playground/calibration/out/jig_fit.png
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from auto_battlebot.plant import (  # noqa: E402
    MODEL_LADDER,
    PARAM_BOUNDS,
    ModelStructure,
    PlantParams,
    WindowErrors,
    concat_windows,
    effective_command,
    make_windows,
    predict_windows,
)
from auto_battlebot.velocity_jig import (  # noqa: E402
    CALIBRATION_TEMPLATE,
    FIT_EXPERIMENTS,
    HOLDOUT_EXPERIMENTS,
    JigCalibration,
    Run,
    Session,
    load_runs,
    load_session,
)

# Per-sample motion floors. Below these the measurement is noise, whatever the command says.
MOTION_EPS_LIN = 0.02  # m/s
MOTION_EPS_ANG = 0.05  # rad/s
ONSET_SIGMA = 5.0  # "moving" is 5 sigma above the stationary noise, per the plan
MIN_HOLD_S = 0.4  # shortest command hold worth fitting a steady state to

# Experiment routing. Each fit reads the phases designed to excite it.
EXP_DEADZONE_LIN = ("E7",)
EXP_DEADZONE_ANG = ("E10",)
EXP_STEP_LIN = ("E8", "E20")
EXP_STEP_ANG = ("E11", "E12")
EXP_COAST = ("E8", "E9", "E20")
EXP_GRID = ("E13",)


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------


@dataclass
class Loaded:
    """Every usable run across every session, plus why the rest were dropped."""

    sessions: list[Session] = field(default_factory=list)
    runs: list[Run] = field(default_factory=list)
    # Session id per run, positional. Not a dict keyed by run name: two sessions recorded on
    # different days both start their log files at LOG-0.
    session_ids: list[str] = field(default_factory=list)
    excluded: list[tuple[str, str]] = field(default_factory=list)

    def by_experiment(self, ids: Sequence[str]) -> list[Run]:
        keep = set(ids)
        return [r for r in self.runs if r.experiment_id in keep]

    def split_session(self, held: str) -> tuple[list[Run], list[Run]]:
        """(train, test) for leave-one-session-out."""
        train = [r for r, s in zip(self.runs, self.session_ids) if s != held]
        test = [r for r, s in zip(self.runs, self.session_ids) if s == held]
        return train, test


def load_all(
    session_paths: Sequence[Path],
    log_dirs: Sequence[Path],
    calib: JigCalibration,
    fit_hz: float,
    keep_bad: bool,
    smooth_s: float = 0.02,
) -> Loaded:
    out = Loaded()
    for session_path, log_dir in zip(session_paths, log_dirs):
        session = load_session(session_path)
        out.sessions.append(session)
        runs, skipped = load_runs(session, log_dir, calib, fit_hz=fit_hz, smooth_s=smooth_s)
        for record, why in skipped:
            out.excluded.append((record.log_file or record.run_id, why))
        for run in runs:
            problems = run.quality.problems()
            if problems and not keep_bad:
                out.excluded.append((run.name, "; ".join(problems)))
                continue
            if problems:
                print(f"  keeping {run.name} despite: {'; '.join(problems)}")
            out.runs.append(run)
            out.session_ids.append(session.session_id)
    return out


# ---------------------------------------------------------------------------
# Stage A: segment extraction
# ---------------------------------------------------------------------------


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
    other: float  # |command| on the other channel, for the coupling fit
    n: int
    t0: float


def _rise_tau(mv: np.ndarray, v_ss: float, dt: float, eps: float, bad: np.ndarray) -> float:
    """First-order rise constant from the leading contiguous 15-90% band, log-linear.

    Restricting to the first contiguous in-band run is the stage 2 fix that mattered most: one
    late steady-state sample that dips back into the band flattens the slope and inflates tau by
    10x, turning a 55 ms rise into a reported 1.1 s.
    """
    inband = (mv > 0.15 * v_ss) & (mv < 0.9 * v_ss) & ((v_ss - mv) > 0.5 * eps) & ~bad
    runs = _runs_of(inband)
    if not runs:
        return float("nan")
    a0, b0 = runs[0]
    sel = np.arange(a0, b0)
    if len(sel) < 4:
        return float("nan")
    slope = float(np.polyfit(sel * dt, np.log(v_ss - mv[sel]), 1)[0])
    return -1.0 / slope if slope < 0 else float("nan")


def held_segments(run: Run, channel: str) -> list[HeldSegment]:
    """Every contiguous stretch where this channel held a constant nonzero command."""
    cmd, vel = _channel(run, channel)
    other = run.cmd_ang if channel == "lin" else run.cmd_lin
    eps = _eps(run, channel)
    min_hold = max(5, int(round(MIN_HOLD_S / run.dt)))

    active = np.abs(cmd) > 1e-3
    # Split on any command change, so a staircase is many segments rather than one.
    changed = np.concatenate([[False], np.abs(np.diff(cmd)) > 1e-4])
    seg_id = np.cumsum(changed)
    out: list[HeldSegment] = []
    for a, b in _runs_of(active):
        start = a
        while start < b:
            same = seg_id[start:b] == seg_id[start]
            stop = start + int(np.count_nonzero(same))
            n = stop - start
            if n >= min_hold:
                mv = np.abs(vel[start:stop])
                bad = run.slip[start:stop]
                tail = slice(int(0.6 * n), n)
                clean = mv[tail][~bad[tail]]
                v_ss = float(np.median(clean)) if len(clean) >= 2 else float(np.median(mv[tail]))
                out.append(
                    HeldSegment(
                        run=run.name,
                        channel=channel,
                        level=float(np.median(cmd[start:stop])),
                        u_eff=float("nan"),
                        v_ss=v_ss,
                        tau=(
                            _rise_tau(mv, v_ss, run.dt, eps, bad)
                            if v_ss > max(3 * eps, 1e-3)
                            else float("nan")
                        ),
                        other=float(np.median(np.abs(other[start:stop]))),
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
        if mv[0] < 5 * eps:  # already stopped, nothing to decay
            continue
        k = 0
        while k < n and mv[k] > max(3 * eps, 0.1 * mv[0]):
            k += 1
        if k < 4 or run.slip[a : a + k].any():
            continue
        tt = np.arange(k) * run.dt
        slope = float(np.polyfit(tt, np.log(mv[:k]), 1)[0])
        if slope < 0:
            out.append(-1.0 / slope)
    return out


# ---------------------------------------------------------------------------
# Stage A: parameter fits
# ---------------------------------------------------------------------------


@dataclass
class Estimate:
    """A parameter with the spread it was measured over.

    Spread comparable to the value means the parameter is not measured, and the report says so
    rather than printing a confident number, the way stage 2 printed `nan`.
    """

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
    vals = np.array([v for v in values if np.isfinite(v)], dtype=float)
    if len(vals) == 0:
        return Estimate(float("nan"), note="no usable segments")
    return Estimate(float(np.median(vals)), float(np.std(vals)), len(vals))


def fit_deadzone(runs: Sequence[Run], channel: str, sign: float) -> Estimate:
    """Deadzone from the staircase: the command level where motion first clears the noise.

    The test is on the median velocity over a 1.5 s hold, so the threshold is 5 sigma of the
    stationary noise divided by sqrt(N), not 5 sigma of a single sample. Stage 2 reported 0.04
    because that was the smallest level tested; the staircase starts at 0.01 so the answer is no
    longer floored by the protocol.
    """
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
            # Linear interpolation between the last still level and the first moving one puts
            # the estimate between staircase steps instead of on one.
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
    """Max speed from steady state against effective command, through the origin.

    With the jig this is interpolation rather than extrapolation: the robot can hold full command
    without driving out of the camera's field of view, which is what forced stage 2 to
    extrapolate its max speeds from partial steps.
    """
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


def fit_coupling(
    runs: Sequence[Run], p: PlantParams
) -> tuple[Estimate, Estimate, list[tuple[float, float, float, float]]]:
    """Steer-brake and angular droop from the E13 grid.

        v = k_lin * u_lin_eff * (1 - c_sb * |u_ang_eff|)
        w = k_ang * u_ang_eff * (1 - c_ad * |u_lin_eff|)

    Both are regressed through the origin on fractional loss. A coefficient whose spread covers
    zero does not go in the model: three terms that are all real beat six where two are noise.
    """
    cells: list[tuple[float, float, float, float]] = []  # u_lin_eff, u_ang_eff, v_ss, w_ss
    for run in runs:
        for seg in held_segments(run, "lin"):
            if seg.other <= 0.02:
                lin_eff = float(effective_command(np.array(seg.level), p.dz_lin_fwd, p.dz_lin_rev))
                cells.append((lin_eff, 0.0, seg.v_ss, 0.0))
                continue
            lin_eff = float(effective_command(np.array(seg.level), p.dz_lin_fwd, p.dz_lin_rev))
            ang_eff = float(effective_command(np.array(seg.other), p.dz_ang_l, p.dz_ang_r))
            # The angular steady state over the same window comes from the run's yaw rate.
            mask = (run.t >= seg.t0) & (run.t < seg.t0 + seg.n * run.dt)
            w_ss = float(np.median(np.abs(run.w[mask]))) if np.count_nonzero(mask) else 0.0
            cells.append((lin_eff, ang_eff, seg.v_ss, w_ss))

    sb_x, sb_y, ad_x, ad_y = [], [], [], []
    for lin_eff, ang_eff, v_ss, w_ss in cells:
        if abs(lin_eff) > 1e-3 and ang_eff > 1e-3:
            k = p.k_fwd if lin_eff > 0 else p.k_rev
            v0 = k * abs(lin_eff)
            if v0 > 1e-3:
                sb_x.append(ang_eff)
                sb_y.append(1.0 - v_ss / v0)
            w0 = p.k_ang * ang_eff
            if w0 > 1e-3 and w_ss > 0.0:
                ad_x.append(abs(lin_eff))
                ad_y.append(1.0 - w_ss / w0)

    def _slope(xs: list[float], ys: list[float]) -> Estimate:
        if len(xs) < 3:
            return Estimate(float("nan"), note=f"{len(xs)} grid cells, need 3")
        x = np.array(xs)
        y = np.array(ys)
        slope = float(np.sum(x * y) / np.sum(x * x))
        resid = y - slope * x
        # Standard error of a through-origin slope.
        se = float(np.sqrt(np.sum(resid**2) / max(len(x) - 1, 1) / np.sum(x**2)))
        note = "consistent with zero" if abs(slope) < 2 * se else ""
        return Estimate(slope, se, len(x), note)

    return _slope(sb_x, sb_y), _slope(ad_x, ad_y), cells


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
    """Transport delay from every clean command edge in the session, pooled.

    Linear edges are timed against forward acceleration, angular edges against yaw acceleration.
    The physical path is shared (Crossfire link, ESC, mechanical), which is what lets both stack
    together. The peak of the averaged acceleration pulse sits at the transport delay: with a
    pure delay in front of a first-order lag, dv/dt is zero until motion starts and jumps to its
    maximum right at the delay.

    This estimate is only a transport delay if the clocks were aligned. On an identity clock it
    is `clock_offset + delay` and the report has to say so.
    """
    dt = runs[0].dt if runs else 0.005
    pre = 4
    max_lag = max(5, int(round(0.24 / dt)))
    win = pre + max_lag + 1
    lags_ms = (np.arange(win) - pre) * dt * 1000.0
    stack = np.zeros(win)
    count = 0
    clock_measured = all(r.truth.clock.measured for r in runs) if runs else False

    for run in runs:
        # Onset timing uses the lightly smoothed velocity. The 20 ms smoothing the rest of the
        # fit runs on moves the peak of dv/dt about half a window later, which would add 10 ms
        # to every delay estimate.
        for cmd, vel in ((run.cmd_lin, run.v_raw), (run.cmd_ang, run.w)):
            # Causal backward difference. A centered difference leaks the response one sample
            # before the edge, which would bias the delay earlier than physically possible.
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
                if run.slip[lo:hi].any():
                    continue
                if max(abs(cmd[e]), abs(cmd[e - 1])) < 0.1:
                    continue
                # Sign-flip by the step direction so accels and decels add rather than cancel.
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
    # The causal backward difference estimates acceleration half a grid step before the sample it
    # is stored at, so the picked peak is half a step late by construction.
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
    coupling_cells: list[tuple[float, float, float, float]]


def stage_a(loaded: Loaded, prior: PlantParams) -> StageA:
    """Per-phase fits, in the order each depends on the last."""
    est: dict[str, Estimate] = {}

    dz_runs_lin = loaded.by_experiment(EXP_DEADZONE_LIN)
    dz_runs_ang = loaded.by_experiment(EXP_DEADZONE_ANG)
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

    lin_segs: list[HeldSegment] = []
    for run in loaded.by_experiment(EXP_STEP_LIN):
        if run.encoder_valid:
            lin_segs += held_segments(run, "lin")
    ang_segs: list[HeldSegment] = []
    for run in loaded.by_experiment(EXP_STEP_ANG):
        ang_segs += held_segments(run, "ang")

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
    for run in loaded.by_experiment(EXP_COAST):
        if run.encoder_valid:
            lin_coast += coast_taus(run, "lin")
    ang_coast: list[float] = []
    for run in loaded.by_experiment(EXP_STEP_ANG):
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

    c_sb, c_ad, cells = fit_coupling(loaded.by_experiment(EXP_GRID), p)
    est["c_sb"] = c_sb
    est["c_ad"] = c_ad
    p = p.replace(
        c_sb=0.0 if c_sb.note else _value(c_sb, 0.0),
        c_ad=0.0 if c_ad.note else _value(c_ad, 0.0),
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

    return StageA(p, est, lin_segs, ang_segs, delay, cells)


def _value(est: Estimate, fallback: float) -> float:
    return fallback if math.isnan(est.value) else est.value


# ---------------------------------------------------------------------------
# Stage B: joint fit by simulation error
# ---------------------------------------------------------------------------


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
):  # -> WindowSet | None
    """Cut windows out of every run and stack them into one batch.

    Runs with a detached encoder carry no linear truth, so they are excluded from the position
    objective. Their heading is still measured, which is what makes the angular parameters
    fittable at rates the encoder cannot survive; `require_encoder=False` builds that set.

    A window needs a known command for every sample it spans. That rules out the still holds at
    both ends of every run and rules out an operator-driven run whose sticks were never logged:
    with no command, the model predicts a robot that never moves and the residual is pure truth.
    """
    sets = []
    for run in runs:
        if require_encoder and not run.encoder_valid:
            continue
        if not require_encoder and run.encoder_valid:
            continue
        valid = ~run.slip & run.commanded
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
        )
        if ws is not None:
            sets.append(ws)
    if not sets:
        return None
    return concat_windows(sets).subsample(max_windows)


def window_delay(params: PlantParams, structure: ModelStructure) -> float:
    """The delay the window command matrices must be built with.

    The delay is applied when the windows are cut, not inside the model, so a rung of the ladder
    that has no delay term needs windows built without one. Reading it back through the structure
    is what keeps M0 from quietly inheriting M1's delay and scoring identically.
    """
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
    windows,
    start: PlantParams,
    structure: ModelStructure,
    weights: FitWeights,
    max_nfev: int | None = None,
) -> tuple[PlantParams, float]:
    """Minimize windowed open-loop error over the structure's free parameters.

    Soft-L1 loss so a slipped wheel or a wall bump that survived the slip flag cannot steer the
    parameters. Delay is fixed here and profiled outside: the objective is not smooth in it at
    the millisecond resolution that matters.
    """
    names = structure.free_names()
    x0 = start.to_vector(names)
    lo = np.array([PARAM_BOUNDS[n][0] for n in names])
    hi = np.array([PARAM_BOUNDS[n][1] for n in names])
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
    """Grid search the transport delay, refitting the continuous parameters at each point.

    The profile curve is the deliverable, not just its minimum: a flat profile means the delay is
    not identifiable from this excitation, and that belongs in the report rather than being
    hidden behind a single number.

    Resolution is bounded by the analysis grid, not by `delay_grid`. On synthetic data with a
    known 45 ms delay, a 200 Hz grid puts the minimum at 50 ms and a 500 Hz grid puts it at
    44 ms, so quote the delay no finer than half a grid step.
    """
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


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------


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
    """Lag-1 autocorrelation of the longest-horizon position error across window starts.

    Strong autocorrelation says the model is missing a structural term. No amount of process
    noise tuning fixes that honestly; it only widens the covariance to cover a bias.
    """
    series = err.position[:, -1]
    if len(series) <= lag + 2:
        return float("nan")
    a = series[:-lag] - series.mean()
    b = series[lag:] - series.mean()
    denom = float(np.sqrt(np.sum(a**2) * np.sum(b**2)))
    return float(np.sum(a * b) / denom) if denom > 0 else float("nan")


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------


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


def write_params(path: Path, p: PlantParams, stage: StageA, note: str) -> None:
    lines = [
        "# Drivetrain plant parameters fit from velocity jig data.",
        f"# {note}",
        "# Consumed by the C++ filter and simulation/kinematic_sim_server.py. One source of",
        "# truth for the plant numbers: edit the fit, not this file.",
        "",
        p.to_toml("plant").rstrip(),
        "",
        "[plant.provenance]",
    ]
    for name, est in stage.estimates.items():
        lines.append(f'{name} = "{est}"')
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"\nwrote {path}")


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------


def make_plot(path: Path, stage: StageA, profile: DelayProfile | None, reports: dict) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(12, 9))
    gs = fig.add_gridspec(3, 2, hspace=0.45, wspace=0.28)

    ax = fig.add_subplot(gs[0, 0])
    for segs, color, label in (
        ([s for s in stage.lin_segments if s.level > 0], "C0", "forward"),
        ([s for s in stage.lin_segments if s.level < 0], "C1", "reverse"),
    ):
        xs = [abs(s.u_eff) for s in segs if np.isfinite(s.u_eff)]
        ys = [s.v_ss for s in segs if np.isfinite(s.u_eff)]
        ax.plot(xs, ys, "o", color=color, ms=4, label=label)
    xx = np.linspace(0, 1, 10)
    ax.plot(xx, stage.params.k_fwd * xx, "C0-", lw=1)
    ax.plot(xx, stage.params.k_rev * xx, "C1-", lw=1)
    ax.set_xlabel("effective command", fontsize=8)
    ax.set_ylabel("steady speed (m/s)", fontsize=8)
    ax.legend(fontsize=7)

    ax = fig.add_subplot(gs[0, 1])
    xs = [abs(s.u_eff) for s in stage.ang_segments if np.isfinite(s.u_eff)]
    ys = [s.v_ss for s in stage.ang_segments if np.isfinite(s.u_eff)]
    ax.plot(xs, ys, "o", color="C2", ms=4)
    ax.plot(xx, stage.params.k_ang * xx, "C2-", lw=1)
    ax.set_xlabel("effective command", fontsize=8)
    ax.set_ylabel("steady yaw rate (rad/s)", fontsize=8)

    ax = fig.add_subplot(gs[1, 0])
    d = stage.delay
    ax.axvline(0.0, color="k", lw=0.9)
    ax.plot(d.lags_ms, d.stack, "C0", lw=2, label=f"{d.count} onsets")
    ax.plot(d.lags_ms, d.smooth, "C3", lw=1)
    if np.isfinite(d.lag_ms):
        ax.axvline(d.lag_ms, color="m", ls="--", lw=1, label=f"{d.lag_ms:.0f} ms")
    ax.set_xlabel("lag from command edge (ms)", fontsize=8)
    ax.set_ylabel("normalized accel", fontsize=8)
    ax.legend(fontsize=7)

    ax = fig.add_subplot(gs[1, 1])
    if profile is not None:
        ax.plot(profile.delays * 1e3, profile.costs, "C0-o", ms=3)
        ax.axvline(profile.best_delay * 1e3, color="m", ls="--", lw=1)
        ax.set_xlabel("transport delay (ms)", fontsize=8)
        ax.set_ylabel("window fit cost", fontsize=8)
    else:
        ax.text(0.5, 0.5, "delay not profiled", ha="center", va="center", transform=ax.transAxes)

    ax = fig.add_subplot(gs[2, :])
    for label, report in reports.items():
        ax.plot(report.horizons * 1e3, report.rows["pos_rmse_mm"], "-o", ms=3, label=label)
    ax.axhline(15.0, color="0.5", ls=":", lw=1)
    ax.axhline(80.0, color="0.5", ls=":", lw=1)
    ax.set_xlabel("horizon (ms)", fontsize=8)
    ax.set_ylabel("position RMSE (mm)", fontsize=8)
    ax.legend(fontsize=7)

    fig.suptitle("Velocity jig plant fit", fontsize=12)
    fig.savefig(path, dpi=110, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("sessions", type=Path, nargs="*", help="session JSON exports")
    parser.add_argument(
        "--logs",
        type=Path,
        nargs="*",
        default=None,
        help="log directory per session (default: each session file's directory)",
    )
    parser.add_argument("--calibration", type=Path, required=False, help="jig calibration TOML")
    parser.add_argument("--out", type=Path, default=None, help="write fitted parameters here")
    parser.add_argument("--plot", type=Path, default=None)
    parser.add_argument(
        "--fit-hz",
        type=float,
        default=200.0,
        help=(
            "uniform analysis grid rate. This sets how finely the delay can be resolved: the pose"
            " integration lands the response within half a grid step, so a 200 Hz grid reports"
            " the delay to about 5 ms and a 500 Hz grid to about 1 ms, at 2.5x the fit time."
        ),
    )
    parser.add_argument(
        "--smooth-ms",
        type=float,
        default=20.0,
        help=(
            "encoder velocity smoothing. Wider is quieter against count quantization but rounds"
            " the corner of a step, which the fit pays back as a few ms of extra transport delay."
            " Worth sweeping once on real data."
        ),
    )
    parser.add_argument("--stride-ms", type=float, default=50.0, help="window start stride")
    parser.add_argument("--max-windows", type=int, default=4000)
    parser.add_argument(
        "--delay-windows", type=int, default=1200, help="windows for the delay profile"
    )
    parser.add_argument("--delay-step-ms", type=float, default=2.0)
    parser.add_argument("--delay-max-ms", type=float, default=120.0)
    parser.add_argument(
        "--track-width", type=float, default=0.10, help="meters, for the k_ang bound"
    )
    parser.add_argument("--model", default="M4", choices=[m.name for m in MODEL_LADDER])
    parser.add_argument("--ladder", action="store_true", help="fit and score every model rung")
    parser.add_argument("--no-joint", action="store_true", help="stop after stage A")
    parser.add_argument("--no-delay-profile", action="store_true", help="keep the stage A delay")
    parser.add_argument("--keep-bad", action="store_true", help="do not exclude runs that fail QC")
    parser.add_argument(
        "--write-calibration-template",
        type=Path,
        default=None,
        help="write a blank jig calibration TOML and exit",
    )
    args = parser.parse_args()

    if args.write_calibration_template:
        args.write_calibration_template.write_text(CALIBRATION_TEMPLATE, encoding="utf-8")
        print(f"wrote {args.write_calibration_template}")
        return
    if not args.sessions:
        parser.error("at least one session JSON is required")
    if not args.calibration:
        parser.error("--calibration is required (runbook block 0)")

    log_dirs = args.logs if args.logs else [p.parent for p in args.sessions]
    if len(log_dirs) != len(args.sessions):
        parser.error("--logs must name one directory per session")

    calib = JigCalibration.from_toml(args.calibration)
    print(f"calibration: {calib.meters_per_count:.8g} m/count, gyro scale {calib.gyro_scale:.4f}")

    loaded = load_all(
        args.sessions, log_dirs, calib, args.fit_hz, args.keep_bad, args.smooth_ms / 1000.0
    )
    print(f"\nloaded {len(loaded.runs)} runs from {len(loaded.sessions)} session(s)")
    for name, why in loaded.excluded:
        print(f"  excluded {name}: {why}")
    if not loaded.runs:
        print("nothing to fit")
        return

    stage = stage_a(loaded, PlantParams())
    print_stage_a(stage)

    structure = next(m for m in MODEL_LADDER if m.name == args.model)
    params = stage.params
    profile: DelayProfile | None = None
    reports: dict[str, HorizonReport] = {}

    fit_runs = [r for r in loaded.runs if r.experiment_id in FIT_EXPERIMENTS]
    test_runs = [r for r in loaded.runs if r.experiment_id in HOLDOUT_EXPERIMENTS]
    print(f"\ntrain runs: {len(fit_runs)}, holdout runs: {len(test_runs)}")

    # The plan's objective horizons, plus 400 ms because that is where acceptance criteria C2 and
    # C3 are stated and a table should measure the horizon it reports.
    horizons = (0.033, 0.066, 0.100, 0.200, 0.300, 0.400, 0.500)
    stride_s = args.stride_ms / 1000.0
    weights = FitWeights()

    if not args.no_joint and fit_runs:
        print("\n=== Stage B: joint fit by simulation error ===")
        if not args.no_delay_profile and structure.use_delay:
            grid = np.arange(0.0, args.delay_max_ms + 1e-9, args.delay_step_ms) / 1000.0
            print(f"  profiling delay over {len(grid)} points")
            profile = profile_delay(
                fit_runs,
                params,
                structure,
                weights,
                horizons,
                stride_s=stride_s,
                max_windows=args.delay_windows,
                delay_grid=grid,
                max_nfev=30,
            )
            params = profile.best_params
            print(f"  profile minimum at {profile.best_delay * 1000:.1f} ms")

        windows = build_windows(
            fit_runs, window_delay(params, structure), horizons, stride_s, args.max_windows
        )
        if windows is None:
            print("  no usable windows; keeping stage A parameters")
        else:
            print(f"  fitting {windows.count()} windows on {len(structure.free_names())} params")
            params, cost = joint_fit(windows, params, structure, weights)
            print(f"  final cost {cost:.4f}")
            reports["train"] = score(predict_windows(windows, params, structure))

            # Weight sensitivity: the fit should not depend much on how position and heading
            # residuals are traded off. A parameter that moves a lot here is being set by the
            # weighting rather than by the data.
            alt, _ = joint_fit(windows, params, structure, FitWeights(0.05, 0.025))
            moved = {
                n: abs(getattr(alt, n) - getattr(params, n)) / max(abs(getattr(params, n)), 1e-9)
                for n in structure.free_names()
            }
            worst = max(moved.items(), key=lambda kv: kv[1])
            print(f"  weight check (heading weight 2x): largest move {worst[0]} {worst[1]:.1%}")

    if test_runs:
        test_windows = build_windows(
            test_runs, window_delay(params, structure), horizons, stride_s, args.max_windows
        )
        if test_windows is not None:
            err = predict_windows(test_windows, params, structure)
            reports["holdout"] = score(err)
            print(f"\nholdout windows: {test_windows.count()}")
            print(
                f"  lag-1 residual autocorrelation at 500 ms: {residual_autocorrelation(err):.2f}"
            )

    print("\n=== Fitted parameters ===")
    for name, value in params.to_dict().items():
        print(f"  {name:12} {value:.6g}")

    for title, report in reports.items():
        print_horizon_table(f"{title} error vs horizon", report)

    if reports.get("holdout") is not None:
        rep = reports["holdout"]
        idx_100 = int(np.argmin(np.abs(rep.horizons - 0.100)))
        idx_400 = int(np.argmin(np.abs(rep.horizons - 0.400)))
        print("\n=== Acceptance criteria (holdout) ===")
        print(
            f"  C1 position RMSE at {rep.horizons[idx_100] * 1e3:.0f} ms:"
            f" {rep.rows['pos_rmse_mm'][idx_100]:.1f} mm (target < 15)"
        )
        print(
            f"  C2 position RMSE at {rep.horizons[idx_400] * 1e3:.0f} ms:"
            f" {rep.rows['pos_rmse_mm'][idx_400]:.1f} mm (target < 80)"
        )
        print(
            f"  C3 heading RMSE at {rep.horizons[idx_400] * 1e3:.0f} ms:"
            f" {rep.rows['head_rmse_deg'][idx_400]:.2f} deg (target < 8)"
        )

    if args.ladder and fit_runs and test_runs:
        print("\n=== Model ladder ===")
        print("    model | train 500 ms RMSE mm | holdout 500 ms RMSE mm | params")
        prev_holdout = None
        for rung in MODEL_LADDER:
            rung_params = stage.params
            train_ws = build_windows(
                fit_runs, window_delay(rung_params, rung), horizons, stride_s, args.delay_windows
            )
            if train_ws is None:
                continue
            rung_params, _ = joint_fit(train_ws, rung_params, rung, weights, max_nfev=60)
            test_ws = build_windows(
                test_runs, window_delay(rung_params, rung), horizons, stride_s, args.delay_windows
            )
            train_rmse = score(predict_windows(train_ws, rung_params, rung)).rows["pos_rmse_mm"][-1]
            hold_rmse = (
                score(predict_windows(test_ws, rung_params, rung)).rows["pos_rmse_mm"][-1]
                if test_ws is not None
                else float("nan")
            )
            gain = (
                ""
                if prev_holdout is None or not np.isfinite(hold_rmse)
                else f"  ({100 * (prev_holdout - hold_rmse) / prev_holdout:+.1f}% vs previous)"
            )
            print(
                f"    {rung.name:5} | {train_rmse:20.1f} | {hold_rmse:22.1f} |"
                f" {len(rung.free_names())}{gain}"
            )
            if np.isfinite(hold_rmse):
                prev_holdout = hold_rmse
        print("    A term that does not buy 5% at the long horizon does not ship.")

    if len(loaded.sessions) > 1:
        print("\n=== Leave-one-session-out ===")
        for held in loaded.sessions:
            train, test = loaded.split_session(held.session_id)
            delay = window_delay(params, structure)
            train_ws = build_windows(train, delay, horizons, stride_s, args.delay_windows)
            test_ws = build_windows(test, delay, horizons, stride_s, args.delay_windows)
            if train_ws is None or test_ws is None:
                print(f"  {held.session_id}: not enough windows")
                continue
            fitted, _ = joint_fit(train_ws, stage.params, structure, weights, max_nfev=80)
            rmse = score(predict_windows(test_ws, fitted, structure)).rows["pos_rmse_mm"][-1]
            print(f"  holding out {held.session_id}: 500 ms position RMSE {rmse:.1f} mm")

    print("\n=== Cross-checks ===")
    for line in cross_checks(params, args.track_width):
        print(f"  {line}")

    if args.out:
        note = f"fit from {', '.join(str(s) for s in args.sessions)} with model {structure.name}"
        write_params(args.out, params, stage, note)
    if args.plot:
        make_plot(args.plot, stage, profile, reports)


if __name__ == "__main__":
    main()
