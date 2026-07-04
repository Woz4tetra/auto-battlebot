"""Fit the full tank drivetrain plant from a calibration session.

Input is one CSV from analyze_apriltag_mcap.py run on an apriltag_track.py --drive recording. That single
file carries both halves the fit needs, recorded in one process on one absolute CLOCK_MONOTONIC clock (no
time alignment is done here):
  - the solved ground-truth pose per frame   (t, x, y, yaw, visible)
  - the scripted command active at that frame (cmd_lin, cmd_ang, trainer_lin, trainer_ang, label)

It extends playground/control_stage0/fit_plant.py: the same first-order AR(1) identification
(v[k+1] = a*v[k] + b*cmd[k], tau = -dt/ln(a), gain = b/(1-a)) and lag-by-cross-correlation idea, but driven
by clean AprilTag truth and the labelled excitation protocol instead of noisy MCAP perception poses. That
lets us pull out what the MCAP fit could not: per-direction deadzone, forward/reverse gain asymmetry,
separate accel vs coast (decel) time constants, the steer-brake coupling, and the actuation lag.

Outputs: a printed report, the `[our_robot]` block to paste into simulation/kinematic_sim.toml, a
recommended `lifted_deadzone_percent` for config/main.toml, and a validation plot (--plot).

Usage:
    source scripts/activate_python.sh
    python playground/calibration/fit_plant_calib.py \
        playground/calibration/out/truth_log.csv \
        --plot playground/calibration/out/fit.png
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

FIT_HZ = 50.0  # uniform analysis grid; makes fits independent of camera frame rate
SMOOTH_TIME = 0.08  # s, position low-pass window before differentiation (cutoff well below 1/tau)
MOTION_EPS_LIN = 0.05  # m/s floor for "moving" (raised to the measured noise floor if larger)
MOTION_EPS_ANG = 0.15  # rad/s floor
ONSET_SIGMA = 5.0  # deadzone motion threshold = this many idle-noise sigmas above zero


# ---------------------------------------------------------------------------
# Load + align
# ---------------------------------------------------------------------------


@dataclass
class Session:
    t: np.ndarray  # common time grid (s)
    cmd_lin: np.ndarray  # normalized [-1, 1], zero-order-hold onto truth times
    cmd_ang: np.ndarray
    label: np.ndarray  # protocol phase per sample
    v: np.ndarray  # forward body speed (m/s), smoothed, on a uniform FIT_HZ grid
    w: np.ndarray  # yaw rate (rad/s), smoothed
    v_raw: np.ndarray  # forward speed without smoothing, for onset-timing (lag) estimates
    w_raw: np.ndarray  # yaw rate without smoothing, for onset-timing (lag) estimates
    gap: np.ndarray  # True where a grid sample sits in a truth dropout (interpolated, not measured)
    dt: float  # uniform grid step (1 / FIT_HZ)
    v_noise: float  # forward-speed noise floor (std over idle)
    w_noise: float  # yaw-rate noise floor
    truth_dt: float  # median truth frame interval (sets the actuation-lag resolution)


def _read_csv(path: Path) -> dict[str, list[str]]:
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        cols: dict[str, list[str]] = {name: [] for name in reader.fieldnames or []}
        for row in reader:
            for k, v in row.items():
                cols[k].append(v)
    return cols


def _smooth(a: np.ndarray, window: int) -> np.ndarray:
    if window < 2 or len(a) < window:
        return a
    kernel = np.ones(window) / window
    return np.convolve(a, kernel, mode="same")


def _zoh(src_t: np.ndarray, src_v: np.ndarray, dst_t: np.ndarray) -> np.ndarray:
    """Zero-order-hold resample of a piecewise-constant signal onto dst_t."""
    idx = np.searchsorted(src_t, dst_t, side="right") - 1
    idx = np.clip(idx, 0, len(src_v) - 1)
    return src_v[idx]


def load_session(session_path: Path) -> Session:
    """Load one CSV from analyze_apriltag_mcap.py (a --drive recording) into a fitting Session.

    The CSV carries both halves the fit needs: the solved pose per frame (x, y, yaw, visible) and the
    scripted command active at that frame (cmd_lin, cmd_ang, label, zero-order-held onto the frame time).
    Commands are read from every row (the full timeline, including frames where the tag was not visible);
    pose is read only from visible frames.
    """
    data = _read_csv(session_path)
    if "cmd_lin" not in data:
        raise SystemExit(
            f"{session_path} has no command columns. fit_plant_calib needs an apriltag_track.py --drive "
            "recording analyzed by analyze_apriltag_mcap.py (read-only stick captures are not enough)."
        )

    # Commands span the full timeline. Frames before the first issued command have empty cells; treat the
    # command there as zero (no command) and the label as unset (matches no protocol phase).
    cmd_t = np.array(data["t"], dtype=float)
    cmd_lin = np.array([v or "0" for v in data["cmd_lin"]], dtype=float)
    cmd_ang = np.array([v or "0" for v in data["cmd_ang"]], dtype=float)
    cmd_label = np.array([v or "" for v in data["label"]])

    visible = np.array(data["visible"], dtype=float) > 0.5
    tru_t = np.array(data["t"], dtype=float)[visible]
    x = np.array([v or "nan" for v in data["x"]], dtype=float)[visible]
    y = np.array([v or "nan" for v in data["y"]], dtype=float)[visible]
    yaw = np.array([v or "nan" for v in data["yaw"]], dtype=float)[visible]

    # Commands and poses come from one recording on one absolute CLOCK_MONOTONIC clock, so no time alignment
    # is needed. The sync twitches at the protocol's start/end are a visual cross-check in --plot.
    # Resample the (gappy, variable-rate) truth onto a uniform grid at FIT_HZ so the fits do not depend on
    # the camera frame rate, then differentiate. Differentiating raw pose amplifies perception noise by
    # 1/dt, so a fixed-time smoother (not fixed-sample) is applied before differentiation; the unsmoothed
    # version is kept only for motion-onset (lag) timing.
    dt = 1.0 / FIT_HZ
    grid = np.arange(tru_t[0], tru_t[-1], dt)
    xg = np.interp(grid, tru_t, x)
    yg = np.interp(grid, tru_t, y)
    yawg = np.interp(grid, tru_t, np.unwrap(yaw))
    win = max(2, round(SMOOTH_TIME * FIT_HZ))

    def body_forward(xx: np.ndarray, yy: np.ndarray) -> np.ndarray:
        vx = np.gradient(xx, dt)
        vy = np.gradient(yy, dt)
        return vx * np.cos(yawg) + vy * np.sin(yawg)

    v = body_forward(_smooth(xg, win), _smooth(yg, win))
    v_raw = body_forward(xg, yg)
    w = np.gradient(_smooth(yawg, win), dt)
    w_raw = np.gradient(yawg, dt)

    # Mark grid samples that fall inside a truth dropout. `grid` was interpolated across the gappy truth
    # times, so a sample far from any real truth frame is a straight-line guess, not measured motion. This
    # is how a flip or a drive out of frame shows up: seconds of interpolated velocity. The lag fit rejects
    # any onset whose response window overlaps a gap, so those segments cannot corrupt the estimate.
    truth_dt = float(np.median(np.diff(tru_t)))
    pos = np.clip(np.searchsorted(tru_t, grid), 1, len(tru_t) - 1)
    nearest = np.minimum(grid - tru_t[pos - 1], tru_t[pos] - grid)
    gap = nearest > max(0.06, 2.0 * truth_dt)  # a genuine dropout of >~2 truth frames

    cmd_lin_g = _zoh(cmd_t, cmd_lin, grid)
    cmd_ang_g = _zoh(cmd_t, cmd_ang, grid)
    label_g = _zoh(cmd_t, cmd_label, grid)

    idle = np.array([str(lbl) == "idle" for lbl in label_g])
    v_noise = float(np.std(v[idle])) if idle.sum() > 4 else 0.0
    w_noise = float(np.std(w[idle])) if idle.sum() > 4 else 0.0

    return Session(
        t=grid,
        cmd_lin=cmd_lin_g,
        cmd_ang=cmd_ang_g,
        label=label_g,
        v=v,
        w=w,
        v_raw=v_raw,
        w_raw=w_raw,
        gap=gap,
        dt=dt,
        v_noise=v_noise,
        w_noise=w_noise,
        truth_dt=truth_dt,
    )


# ---------------------------------------------------------------------------
# Per-parameter fits
# ---------------------------------------------------------------------------


def _mask(s: Session, prefix: str) -> np.ndarray:
    return np.array([str(lbl).startswith(prefix) for lbl in s.label])


def _deadzone_levels(cmd: np.ndarray, vel: np.ndarray, sign: float) -> dict[float, float]:
    """Median |velocity| held at each distinct staircase command level of the given sign."""
    buckets: dict[float, list[float]] = {}
    for c, vlist in zip(np.round(cmd, 3), vel):
        if np.sign(c) == sign and abs(c) > 1e-6:
            buckets.setdefault(abs(float(c)), []).append(abs(vlist))
    return {level: float(np.median(vs)) for level, vs in buckets.items()}


def fit_deadzone(cmd: np.ndarray, vel: np.ndarray, sign: float, eps: float) -> float:
    """Smallest staircase command level (same sign) whose sustained motion exceeds eps.

    Works per command level, not per sample: the staircase holds each command for ~0.8 s, so the median
    |velocity| over that hold is compared to the threshold. This rejects single-sample noise spikes that a
    per-sample test would mistake for motion onset.
    """
    moving = [level for level, med in _deadzone_levels(cmd, vel, sign).items() if med > eps]
    return float(min(moving)) if moving else float("nan")


def _runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """Contiguous [start, stop) index ranges where mask is True."""
    idx = np.flatnonzero(mask)
    if len(idx) == 0:
        return []
    breaks = np.flatnonzero(np.diff(idx) > 1)
    starts = np.concatenate([[idx[0]], idx[breaks + 1]])
    stops = np.concatenate([idx[breaks], [idx[-1]]])
    return list(zip(starts.tolist(), (stops + 1).tolist()))


def _rise_tau(mv: np.ndarray, vss: float, dt: float, eps: float) -> float:
    """First-order rise time constant via log-linear fit of ln(Vss - |v|) over the 15-85% band.

    Parametric (fits the whole curve), so it is far less biased than an AR(1) coefficient under both
    measurement noise and the pre-differentiation smoothing.
    """
    tt = np.arange(len(mv)) * dt
    sel = (mv > 0.15 * vss) & (mv < 0.85 * vss) & ((vss - mv) > 0.5 * eps)
    if sel.sum() < 4:
        return float("nan")
    slope = float(np.polyfit(tt[sel], np.log(vss - mv[sel]), 1)[0])
    return -1.0 / slope if slope < 0 else float("nan")


@dataclass
class DriveSeg:
    """One held-drive segment, with the intermediates behind the gain and rise-tau fits."""

    sign: float  # command sign (+1 fwd/left, -1 rev/right)
    amp: float  # raw |command| held over the segment
    ceff: float  # effective command after the deadzone is removed
    vss: float  # steady-state |velocity| (median over the last 40%)
    tau: float  # first-order rise time constant (nan if it could not be fit)
    t: np.ndarray  # segment-relative time (s)
    mv: np.ndarray  # |velocity| over the segment


def _drive_segments(
    s: Session,
    cmd: np.ndarray,
    vel: np.ndarray,
    mask: np.ndarray,
    dz_fwd: float,
    dz_rev: float,
    eps: float,
) -> list[DriveSeg]:
    """Extract every usable held-drive segment (the shared source for the gain and rise-tau fits)."""
    active = mask & (np.abs(cmd) > 1e-3)
    min_hold = max(5, round(0.5 / s.dt))
    segs: list[DriveSeg] = []
    for a, b in _runs(active):
        n = b - a
        if n < min_hold:
            continue
        sign = float(np.sign(np.median(cmd[a:b])))
        dz = dz_fwd if sign > 0 else dz_rev
        dz = dz if not math.isnan(dz) else 0.0
        amp = float(np.median(np.abs(cmd[a:b])))
        ceff = (amp - dz) / (1.0 - dz)
        if ceff <= 1e-3:
            continue
        mv = np.abs(vel[a:b])
        vss = float(np.median(mv[int(0.6 * n) :]))
        if vss <= max(3 * eps, 1e-3):
            continue
        tau = _rise_tau(mv, vss, s.dt, eps)
        segs.append(DriveSeg(sign, amp, ceff, vss, tau, np.arange(n) * s.dt, mv))
    return segs


def fit_drive_segments(
    s: Session,
    cmd: np.ndarray,
    vel: np.ndarray,
    mask: np.ndarray,
    dz_fwd: float,
    dz_rev: float,
    eps: float,
) -> tuple[list[float], list[float], list[float], list[float]]:
    """Per held-drive segment: steady-state gain and rise tau, bucketed by command sign.

    Returns (gains_fwd, gains_rev, taus_fwd, taus_rev). Gain is Vss / effective-command (deadzone removed),
    which is a steady-state quantity and so immune to the smoothing/noise tau trade-off.
    """
    gf: list[float] = []
    gr: list[float] = []
    tf: list[float] = []
    tr: list[float] = []
    for seg in _drive_segments(s, cmd, vel, mask, dz_fwd, dz_rev, eps):
        (gf if seg.sign > 0 else gr).append(seg.vss / seg.ceff)
        if not math.isnan(seg.tau):
            (tf if seg.sign > 0 else tr).append(seg.tau)
    return gf, gr, tf, tr


@dataclass
class DecaySeg:
    """One zero-command coast tail and its exponential-decay fit."""

    t: np.ndarray  # segment-relative time over the coasting portion (s)
    mv: np.ndarray  # |velocity| over the coasting portion
    tau: float  # coast time constant (nan if the fit was not a decay)


def _decay_segments(
    s: Session, cmd: np.ndarray, vel: np.ndarray, mask: np.ndarray, eps: float
) -> list[DecaySeg]:
    """Extract every zero-command coast tail: |v| = V0 exp(-t/tau). Shared by the fit and the plot."""
    quiet = mask & (np.abs(cmd) < 0.02)
    out: list[DecaySeg] = []
    for a, b in _runs(quiet):
        n = b - a
        if n < max(4, round(0.3 / s.dt)):
            continue
        mv = np.abs(vel[a:b])
        if mv[0] < 5 * eps:  # must actually be coasting down from motion
            continue
        k = 0
        while k < n and mv[k] > max(3 * eps, 0.1 * mv[0]):
            k += 1
        if k < 4:
            continue
        tt = np.arange(k) * s.dt
        slope = float(np.polyfit(tt, np.log(mv[:k]), 1)[0])
        out.append(DecaySeg(tt, mv[:k], -1.0 / slope if slope < 0 else float("nan")))
    return out


def fit_decay_tau(
    s: Session, cmd: np.ndarray, vel: np.ndarray, mask: np.ndarray, eps: float
) -> float:
    """Coast (decel) time constant from the zero-command tails after each step: |v| = V0 exp(-t/tau)."""
    taus = [d.tau for d in _decay_segments(s, cmd, vel, mask, eps) if not math.isnan(d.tau)]
    return float(np.median(taus)) if taus else float("nan")


def _steer_brake_levels(s: Session) -> dict[float, float]:
    """Steady-state forward speed at each |angular command| cell of the steer-brake grid."""
    drive = _mask(s, "steer_brake") & (s.cmd_lin > 0.5)
    levels: dict[float, float] = {}
    for a, b in _runs(drive):
        if b - a < max(5, round(0.5 / s.dt)):
            continue
        ang = round(float(np.median(np.abs(s.cmd_ang[a:b]))), 2)
        spd = float(np.median(s.v[int(0.5 * (a + b)) : b]))
        levels.setdefault(ang, spd)
    return levels


def fit_steer_brake(s: Session) -> float:
    """Fractional forward-speed loss per unit |angular command| during the steer-brake grid.

    One steady-state forward-speed sample per grid cell (last half of each drive segment), so the rising
    transient does not bias the slope. speed/speed0 = 1 - coeff*|ang|.
    """
    levels = _steer_brake_levels(s)
    if 0.0 not in levels or len(levels) < 3:
        return float("nan")
    v0 = levels[0.0]
    if v0 <= 1e-3:
        return float("nan")
    angs = np.array([a for a in levels if a > 0.0])
    loss = np.array([1.0 - levels[a] / v0 for a in angs])
    coeff, *_ = np.linalg.lstsq(angs[:, None], loss, rcond=None)
    return float(coeff[0])


@dataclass
class LatencyDiag:
    """The stacked-onset intermediate behind the actuation-lag fit (also drives the latency plot)."""

    pre: int  # samples retained before each edge (the pre-onset baseline)
    dt: float
    lags_ms: np.ndarray  # ms offset from the edge for each window sample (negative = before the edge)
    stack: np.ndarray  # baseline-subtracted averaged acceleration pulse over the window
    smooth: np.ndarray  # 3-tap smoothed stack the peak is picked from
    windows: list[np.ndarray]  # per-onset normalized curves that were averaged (for a faint overlay)
    count: int  # number of in-frame onsets pooled
    snr: float  # peak over pre-edge baseline spread: a confidence gauge
    peak_i: int  # peak index into the causal region (0 = at the edge)
    lag_ms: float  # the fitted transport lag


def _latency_stack(s: Session) -> LatencyDiag:
    """Pool every clean command edge and stack the acceleration that follows it to time the transport lag.

    The dedicated latency battery is not trusted on its own: its aggressive bipolar steps flip the robot or
    drive it out of frame, leaving the truth pose (and so the velocity) mostly interpolated across dropouts.
    Instead every clean command edge in the whole session is pooled. Linear edges are timed against forward
    acceleration (dv/dt), angular edges against yaw acceleration (dw/dt); the transport delay is the same
    physical path (Crossfire + ESC + mechanical) for both, so they stack together. Any edge whose response
    window overlaps a truth gap is dropped, which is what removes the flipped latency segments and leans the
    estimate on the many in-frame angular-step onsets.

    Each edge contributes its post-edge acceleration, sign-flipped to the step direction so accels and
    decels add rather than cancel, and scaled by the channel's own accel spread so m/s^2 and rad/s^2 are
    comparable. Stacking many onsets averages down the per-frame perception noise that made the single
    latency-battery correlation swing wildly between runs. Lag is the peak of the averaged acceleration
    pulse, which for a delay + first-order plant sits exactly at the transport delay: dv/dt is zero until
    motion starts then jumps to its maximum right at the delay. The peak is far more robust to noise here
    than a first-threshold crossing at this frame rate and onset count.
    """
    pre = 4  # samples kept before each edge to measure the pre-onset baseline
    max_lag = max(5, round(0.24 / s.dt))  # search transport delays out to 240 ms
    win = pre + max_lag + 1
    lags_ms = (np.arange(win) - pre) * s.dt * 1000.0
    stack = np.zeros(win)
    windows: list[np.ndarray] = []
    for cmd, vel in ((s.cmd_lin, s.v_raw), (s.cmd_ang, s.w_raw)):
        # Causal backward difference, not np.gradient: a centered difference leaks the response one sample
        # before the edge, which would bias a transport-delay estimate earlier than physically possible.
        accel = np.concatenate([[0.0], np.diff(vel)]) / s.dt
        scale = float(np.std(accel))
        if scale < 1e-9:
            continue
        # The AprilTag pose frame can be inverted relative to the command frame (a mirrored tag or a
        # 180-deg mount), so a positive command reads as negative measured velocity. Recover the sign from
        # the data so both channels stack in the same direction instead of cancelling.
        moving = (~s.gap) & (np.abs(cmd) > 0.1)
        pol = 1.0
        if moving.sum() > 5:
            corr = float(np.corrcoef(cmd[moving], vel[moving])[0, 1])
            pol = -1.0 if corr < 0 else 1.0
        dcmd = np.diff(cmd)
        for i in np.flatnonzero(np.abs(dcmd) > 0.15):
            e = i + 1  # first accel sample at or after the edge
            lo, hi = e - pre, e + max_lag + 1
            if lo < 0 or hi > len(accel):
                continue
            if s.gap[lo:hi].any():  # response window is interpolated, not measured
                continue
            if max(abs(cmd[e]), abs(cmd[e - 1])) < 0.1:  # both sides idle: no motion to time
                continue
            curve = pol * np.sign(dcmd[i]) * accel[lo:hi] / scale
            stack += curve
            windows.append(curve)
    count = len(windows)
    if count < 6:
        return LatencyDiag(pre, s.dt, lags_ms, stack, stack, windows, count, 0.0, 0, float("nan"))
    stack = stack / count - float(np.mean(stack[:pre] / count))  # remove the pre-edge baseline offset
    smooth = np.convolve(stack, np.array([0.25, 0.5, 0.25]), mode="same")
    causal = smooth[pre:]
    peak_i = int(np.argmax(causal))
    # Parabolic sub-sample refinement around the peak, to average down the 1-frame quantization across runs.
    delta = 0.0
    if 0 < peak_i < len(causal) - 1:
        a, b, c = causal[peak_i - 1], causal[peak_i], causal[peak_i + 1]
        denom = a - 2.0 * b + c
        if abs(denom) > 1e-9:
            delta = float(np.clip(0.5 * (a - c) / denom, -0.5, 0.5))
    snr = float(causal[peak_i] / (np.std(smooth[:pre]) + 1e-9))
    lag_ms = (peak_i + delta) * s.dt * 1000.0
    return LatencyDiag(pre, s.dt, lags_ms, stack, smooth, windows, count, snr, peak_i, lag_ms)


def fit_actuation_lag(s: Session) -> tuple[float, int, float]:
    """Transport lag (ms), the number of in-frame onsets it used, and an SNR confidence gauge."""
    d = _latency_stack(s)
    return d.lag_ms, d.count, d.snr


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------


@dataclass
class FitContext:
    """Masks and thresholds shared by every fit, so the report and the plots derive from one source."""

    lin: np.ndarray  # linear drive mask (lin_step | lin_max)
    ang: np.ndarray  # angular drive mask (ang_step | ang_max)
    eps_lin: float  # per-sample "moving" floor, forward speed
    eps_ang: float  # per-sample "moving" floor, yaw rate
    dz_eps_lin: float  # gentler deadzone-onset threshold, forward speed
    dz_eps_ang: float  # gentler deadzone-onset threshold, yaw rate
    dz_lin_fwd: float
    dz_lin_rev: float
    dz_ang_l: float
    dz_ang_r: float


def _context(s: Session) -> FitContext:
    eps_lin = max(MOTION_EPS_LIN, ONSET_SIGMA * s.v_noise)
    eps_ang = max(MOTION_EPS_ANG, ONSET_SIGMA * s.w_noise)
    # The deadzone test compares a per-level MEDIAN over a ~0.8 s hold, which averages down the noise by
    # ~sqrt(N); use a gentler threshold than the per-sample motion floor so the onset is not pushed high.
    dz_eps_lin = max(MOTION_EPS_LIN, 2.0 * s.v_noise)
    dz_eps_ang = max(MOTION_EPS_ANG, 2.0 * s.w_noise)
    return FitContext(
        lin=_mask(s, "lin_step") | _mask(s, "lin_max"),
        ang=_mask(s, "ang_step") | _mask(s, "ang_max"),
        eps_lin=eps_lin,
        eps_ang=eps_ang,
        dz_eps_lin=dz_eps_lin,
        dz_eps_ang=dz_eps_ang,
        dz_lin_fwd=fit_deadzone(
            s.cmd_lin[_mask(s, "lin_deadzone_fwd")], s.v[_mask(s, "lin_deadzone_fwd")], 1.0, dz_eps_lin
        ),
        dz_lin_rev=fit_deadzone(
            s.cmd_lin[_mask(s, "lin_deadzone_rev")], s.v[_mask(s, "lin_deadzone_rev")], -1.0, dz_eps_lin
        ),
        dz_ang_l=fit_deadzone(
            s.cmd_ang[_mask(s, "ang_deadzone_left")], s.w[_mask(s, "ang_deadzone_left")], 1.0, dz_eps_ang
        ),
        dz_ang_r=fit_deadzone(
            s.cmd_ang[_mask(s, "ang_deadzone_right")], s.w[_mask(s, "ang_deadzone_right")], -1.0, dz_eps_ang
        ),
    )


def fit_all(s: Session) -> dict[str, float]:
    c = _context(s)
    dz_lin_fwd, dz_lin_rev = c.dz_lin_fwd, c.dz_lin_rev
    dz_ang_l, dz_ang_r = c.dz_ang_l, c.dz_ang_r

    gf, gr, tf, tr = fit_drive_segments(s, s.cmd_lin, s.v, c.lin, dz_lin_fwd, dz_lin_rev, c.eps_lin)
    # Angular: buckets by sign (left/right); combine both for a single symmetric max_angular_speed.
    ga_l, ga_r, ta_l, ta_r = fit_drive_segments(s, s.cmd_ang, s.w, c.ang, dz_ang_l, dz_ang_r, c.eps_ang)

    def med(xs: list[float]) -> float:
        return float(np.median(xs)) if xs else float("nan")

    lag_ms, lag_onsets, lag_snr = fit_actuation_lag(s)

    return {
        "linear_deadzone_fwd": dz_lin_fwd,
        "linear_deadzone_rev": dz_lin_rev,
        "angular_deadzone_left": dz_ang_l,
        "angular_deadzone_right": dz_ang_r,
        "max_linear_speed_fwd": med(gf),
        "max_linear_speed_rev": med(gr),
        "max_angular_speed": med(ga_l + ga_r),
        "tau_linear_accel": med(tf + tr),
        "tau_linear_decel": fit_decay_tau(s, s.cmd_lin, s.v, _mask(s, "lin_step"), c.eps_lin),
        "tau_angular_accel": med(ta_l + ta_r),
        "steer_brake_coeff": fit_steer_brake(s),
        "actuation_lag_ms": lag_ms,
        "actuation_lag_onsets": float(lag_onsets),
        "actuation_lag_snr": lag_snr,
        "truth_dt_ms": s.truth_dt * 1000.0,
    }


def print_report(p: dict[str, float]) -> None:
    def f(key: str, unit: str = "") -> str:
        v = p[key]
        return "n/a" if (isinstance(v, float) and math.isnan(v)) else f"{v:.3f}{unit}"

    print("\n=== Robot plant fit ===")
    print(
        f"  linear deadzone   fwd {f('linear_deadzone_fwd')}  rev {f('linear_deadzone_rev')}  (cmd frac)"
    )
    print(
        f"  angular deadzone  left {f('angular_deadzone_left')}  right {f('angular_deadzone_right')}"
    )
    print(
        f"  max linear speed  fwd {f('max_linear_speed_fwd', ' m/s')}  rev {f('max_linear_speed_rev', ' m/s')}"
    )
    print(f"  max angular speed {f('max_angular_speed', ' rad/s')}")
    print(
        f"  tau linear        accel {f('tau_linear_accel', ' s')}  decel {f('tau_linear_decel', ' s')}"
    )
    print(f"  tau angular accel {f('tau_angular_accel', ' s')}")
    print(f"  steer-brake coeff {f('steer_brake_coeff')}  (fwd speed loss per unit |ang cmd|)")
    print(
        f"  actuation lag     {f('actuation_lag_ms', ' ms')}  (+/- {p['truth_dt_ms']:.0f} ms: one truth"
        f" frame, from {int(p['actuation_lag_onsets'])} onsets, snr {p['actuation_lag_snr']:.1f})"
    )
    if p["actuation_lag_onsets"] < 6:
        print("    note: too few in-frame command onsets to time the lag; rerun with the tag kept in view")
    elif p["actuation_lag_snr"] < 3.0:
        print("    note: weak onset signal; trust the highest-visibility run and treat this lag as +/-1 frame")
    if p["truth_dt_ms"] > 20.0:
        print(
            f"    note: truth is {1000.0 / p['truth_dt_ms']:.0f} fps; the lag needs >=60 fps capture to resolve well"
        )

    dz_vals = [v for v in (p["linear_deadzone_fwd"], p["linear_deadzone_rev"]) if not math.isnan(v)]
    dz = max(dz_vals) if dz_vals else float("nan")
    print("\nPaste into simulation/kinematic_sim.toml [our_robot]:")
    print(f"  max_linear_speed_fwd = {p['max_linear_speed_fwd']:.3f}")
    print(f"  max_linear_speed_rev = {p['max_linear_speed_rev']:.3f}")
    print(f"  max_angular_speed    = {p['max_angular_speed']:.3f}")
    print(f"  tau_linear_accel     = {p['tau_linear_accel']:.3f}")
    print(f"  tau_linear_decel     = {p['tau_linear_decel']:.3f}")
    print(f"  tau_angular_accel    = {p['tau_angular_accel']:.3f}")
    if not math.isnan(p["steer_brake_coeff"]):
        print(f"  steer_brake_coeff    = {p['steer_brake_coeff']:.3f}")
    print("\nUpdate simulation/kinematic_sim.toml [latency]:")
    print(
        f"  command_ms = {p['actuation_lag_ms']:.0f}   # already includes Crossfire + ESC + mechanical"
    )
    if not math.isnan(dz):
        print(
            f"\nRecommended config/main.toml [transmitter] lifted_deadzone_percent = {dz * 100:.0f}"
        )


def _phase_spans(s: Session) -> list[tuple[float, float, str]]:
    """Contiguous (start_t, end_t, label) spans of the protocol phase, for shading the timeline."""
    labels = [str(x) for x in s.label]
    spans: list[tuple[float, float, str]] = []
    i, n = 0, len(labels)
    while i < n:
        j = i
        while j < n and labels[j] == labels[i]:
            j += 1
        if labels[i] not in ("", "nan"):
            spans.append((float(s.t[i]), float(s.t[min(j, n - 1)]), labels[i]))
        i = j
    return spans


def _panel_timeline(ax, s, cmd, vel, vel_label, cmd_color, annotate):  # type: ignore[no-untyped-def]
    """One drive channel over the whole session: measured velocity (left axis) vs command (right axis).

    Truth dropouts are shaded red (velocity there is interpolation, not measurement) and every protocol
    phase gets a light band, so it is obvious which experiment produced which stretch of data.
    """
    import matplotlib.pyplot as plt

    gap_runs = _runs(s.gap)
    for k, (a, b) in enumerate(gap_runs):
        ax.axvspan(s.t[a], s.t[min(b, len(s.t) - 1)], color="red", alpha=0.10, lw=0,
                   label="tag dropout (interpolated)" if k == 0 else None)
    for t0, _t1, lbl in _phase_spans(s):
        ax.axvline(t0, color="0.6", ls=":", lw=0.5)
        if annotate:
            ax.text(t0, 1.02, lbl, rotation=90, va="bottom", ha="left", fontsize=5.5,
                    alpha=0.75, transform=ax.get_xaxis_transform())
    ax.plot(s.t, vel, color="0.15", lw=0.8, label=vel_label)
    ax.legend(fontsize=6, loc="upper right")
    ax.set_ylabel(vel_label, fontsize=8)
    axc = ax.twinx()
    axc.plot(s.t, cmd, color=cmd_color, lw=0.8, alpha=0.8, label="command")
    axc.set_ylim(-1.15, 1.15)
    axc.set_ylabel("command", fontsize=8, color=cmd_color)
    axc.tick_params(axis="y", labelsize=7, colors=cmd_color)
    ax.tick_params(labelsize=7)
    ax.set_zorder(axc.get_zorder() + 1)
    ax.patch.set_visible(False)


def _panel_deadzone(ax, s, mask_prefixes, cmd, vel, dz_eps, dz_vals, unit):  # type: ignore[no-untyped-def]
    """Staircase deadzone: median sustained |velocity| at each command level, with the picked onset."""
    for (prefix, sign, color, name), dz in zip(mask_prefixes, dz_vals):
        m = _mask(s, prefix)
        levels = _deadzone_levels(cmd[m], vel[m], sign)
        if levels:
            xs = sorted(levels)
            ax.plot(xs, [levels[x] for x in xs], "o-", color=color, ms=4, lw=1, label=name)
        if not math.isnan(dz):
            ax.axvline(dz, color=color, ls=":", lw=1)
    ax.axhline(dz_eps, color="gray", ls="--", lw=1, label="motion threshold")
    ax.set_xlabel("|command|", fontsize=8)
    ax.set_ylabel(f"|velocity| ({unit})", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.legend(fontsize=6, loc="upper left")


def _empty_note(ax) -> bool:  # type: ignore[no-untyped-def]
    """Mark a panel that had no in-frame data to fit, so a blank axis is not mistaken for zero."""
    ax.text(0.5, 0.5, "no in-frame data", ha="center", va="center", fontsize=9, color="0.5",
            transform=ax.transAxes)
    return True


def _panel_gain(ax, segs, fits, unit):  # type: ignore[no-untyped-def]
    """Steady-state speed vs effective command per held segment; the fit slope is the max speed."""
    if not segs:
        _empty_note(ax)
    xmax = 1.0
    for seg in segs:
        color = "C0" if seg.sign > 0 else "C1"
        ax.plot(seg.ceff, seg.vss, "o", color=color, ms=4)
        xmax = max(xmax, seg.ceff)
    xx = np.linspace(0.0, xmax, 20)
    for slope, color, name in fits:
        if not math.isnan(slope):
            ax.plot(xx, slope * xx, "-", color=color, lw=1, label=f"{name}: {slope:.2f} {unit}")
    ax.set_xlabel("effective command (deadzone removed)", fontsize=8)
    ax.set_ylabel(f"steady-state speed ({unit})", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.legend(fontsize=6, loc="upper left")


def _panel_rise(ax, segs, tau, unit):  # type: ignore[no-untyped-def]
    """Normalized rise curves per held segment, overlaid with the first-order model at the fitted tau."""
    tmax = 0.0
    drawn = 0
    for seg in segs:
        if math.isnan(seg.tau) or seg.vss <= 0:
            continue
        color = "C0" if seg.sign > 0 else "C1"
        ax.plot(seg.t, seg.mv / seg.vss, color=color, lw=0.7, alpha=0.5)
        tmax = max(tmax, float(seg.t[-1]))
        drawn += 1
    if drawn == 0:
        _empty_note(ax)
    if not math.isnan(tau) and tmax > 0:
        tt = np.linspace(0.0, tmax, 100)
        ax.plot(tt, 1.0 - np.exp(-tt / tau), "k--", lw=1.5, label=f"tau_accel = {tau:.3f} s")
    ax.axhline(1.0, color="gray", ls=":", lw=0.8)
    ax.set_xlabel("time since command step (s)", fontsize=8)
    ax.set_ylabel(f"velocity / steady state ({unit} norm.)", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.legend(fontsize=6, loc="lower right")


def _panel_decay(ax, decays, tau):  # type: ignore[no-untyped-def]
    """Zero-command coast tails on a log axis; a straight line there is a first-order decay at tau."""
    if not decays:
        _empty_note(ax)
    v0s = []
    tmax = 0.0
    for d in decays:
        ax.semilogy(d.t, d.mv, color="C0", lw=0.7, alpha=0.6)
        v0s.append(float(d.mv[0]))
        tmax = max(tmax, float(d.t[-1]))
    if not math.isnan(tau) and v0s and tmax > 0:
        tt = np.linspace(0.0, tmax, 100)
        ax.semilogy(tt, float(np.median(v0s)) * np.exp(-tt / tau), "k--", lw=1.5,
                    label=f"tau_decel = {tau:.3f} s")
        ax.legend(fontsize=6, loc="upper right")
    ax.set_xlabel("time since command release (s)", fontsize=8)
    ax.set_ylabel("|velocity| (log)", fontsize=8)
    ax.tick_params(labelsize=7)


def _panel_latency(ax, diag):  # type: ignore[no-untyped-def]
    """The pooled onset stack: faint per-onset accel windows, the averaged pulse, and the picked lag."""
    for w in diag.windows:
        ax.plot(diag.lags_ms, w - float(np.mean(w[: diag.pre])), color="gray", lw=0.5, alpha=0.15)
    ax.axvspan(diag.lags_ms[0], 0.0, color="gray", alpha=0.06, lw=0)  # pre-edge baseline region
    ax.axvline(0.0, color="k", lw=0.9)
    if diag.count >= 6:
        ax.plot(diag.lags_ms, diag.stack, color="C0", lw=2.0, label="mean accel (normalized)")
        ax.plot(diag.lags_ms, diag.smooth, color="C3", lw=1.0, alpha=0.8, label="smoothed")
        if not math.isnan(diag.lag_ms):
            ax.axvline(diag.lag_ms, color="C2", lw=2.0, label=f"lag = {diag.lag_ms:.0f} ms")
        # Zoom to the averaged pulse: individual onsets are ~10x noisier and would flatten it off-scale.
        amp = float(np.max(np.abs(diag.stack))) or 1.0
        ax.set_ylim(-3.0 * amp, 3.0 * amp)
    ax.set_xlabel("time after command edge (ms)", fontsize=8)
    ax.set_ylabel("stacked accel (norm.)", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.legend(fontsize=6, loc="upper right")
    ax.set_title(f"actuation-lag onset stack: {diag.count} in-frame onsets, snr {diag.snr:.1f}",
                 fontsize=9)


def plot_session(s: Session, params: dict[str, float], out: Path) -> None:
    try:
        import matplotlib
    except ModuleNotFoundError:
        print("\nmatplotlib not installed; skipping --plot (the fit above is unaffected).")
        return
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    c = _context(s)
    diag = _latency_stack(s)
    lin_segs = _drive_segments(s, s.cmd_lin, s.v, c.lin, c.dz_lin_fwd, c.dz_lin_rev, c.eps_lin)
    ang_segs = _drive_segments(s, s.cmd_ang, s.w, c.ang, c.dz_ang_l, c.dz_ang_r, c.eps_ang)
    decays = _decay_segments(s, s.cmd_lin, s.v, _mask(s, "lin_step"), c.eps_lin)

    fig = plt.figure(figsize=(13, 18))
    gs = fig.add_gridspec(6, 2, height_ratios=[1.1, 1.1, 1.0, 1.0, 1.0, 1.0], hspace=0.42, wspace=0.22)

    _panel_timeline(fig.add_subplot(gs[0, :]), s, s.cmd_lin, s.v, "forward speed (m/s)", "C0", True)
    _panel_timeline(fig.add_subplot(gs[1, :]), s, s.cmd_ang, s.w, "yaw rate (rad/s)", "C1", False)

    _panel_deadzone(
        fig.add_subplot(gs[2, 0]), s,
        [("lin_deadzone_fwd", 1.0, "C0", "fwd"), ("lin_deadzone_rev", -1.0, "C1", "rev")],
        s.cmd_lin, s.v, c.dz_eps_lin, [c.dz_lin_fwd, c.dz_lin_rev], "m/s",
    )
    _panel_deadzone(
        fig.add_subplot(gs[2, 1]), s,
        [("ang_deadzone_left", 1.0, "C0", "left"), ("ang_deadzone_right", -1.0, "C1", "right")],
        s.cmd_ang, s.w, c.dz_eps_ang, [c.dz_ang_l, c.dz_ang_r], "rad/s",
    )

    _panel_gain(
        fig.add_subplot(gs[3, 0]), lin_segs,
        [(params["max_linear_speed_fwd"], "C0", "fwd"), (params["max_linear_speed_rev"], "C1", "rev")],
        "m/s",
    )
    _panel_gain(
        fig.add_subplot(gs[3, 1]), ang_segs,
        [(params["max_angular_speed"], "0.3", "max angular")], "rad/s",
    )

    _panel_rise(fig.add_subplot(gs[4, 0]), lin_segs, params["tau_linear_accel"], "m/s")
    _panel_decay(fig.add_subplot(gs[4, 1]), decays, params["tau_linear_decel"])

    _panel_latency(fig.add_subplot(gs[5, :]), diag)

    fig.suptitle(f"Calibration diagnostics: {out.stem}", fontsize=12, y=0.995)
    fig.savefig(out, dpi=110, bbox_inches="tight")
    plt.close(fig)
    print(f"\nwrote {out}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "session_csv",
        type=Path,
        help="truth+command CSV from analyze_apriltag_mcap.py on a --drive recording",
    )
    parser.add_argument("--plot", type=Path, default=None)
    args = parser.parse_args()

    s = load_session(args.session_csv)
    params = fit_all(s)
    print_report(params)
    plot_path = args.session_csv.with_suffix(".png") if not args.plot else args.plot
    plot_session(s, params, plot_path)


if __name__ == "__main__":
    main()
