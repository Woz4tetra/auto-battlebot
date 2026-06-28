"""Fit the full Mrs Buff MK3 drivetrain plant from a calibration session.

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
        dt=dt,
        v_noise=v_noise,
        w_noise=w_noise,
        truth_dt=float(np.median(np.diff(tru_t))),
    )


# ---------------------------------------------------------------------------
# Per-parameter fits
# ---------------------------------------------------------------------------


def _mask(s: Session, prefix: str) -> np.ndarray:
    return np.array([str(lbl).startswith(prefix) for lbl in s.label])


def fit_deadzone(cmd: np.ndarray, vel: np.ndarray, sign: float, eps: float) -> float:
    """Smallest staircase command level (same sign) whose sustained motion exceeds eps.

    Works per command level, not per sample: the staircase holds each command for ~0.8 s, so the median
    |velocity| over that hold is compared to the threshold. This rejects single-sample noise spikes that a
    per-sample test would mistake for motion onset.
    """
    levels: dict[float, list[float]] = {}
    for c, vlist in zip(np.round(cmd, 3), vel):
        if np.sign(c) == sign and abs(c) > 1e-6:
            levels.setdefault(float(c), []).append(abs(vlist))
    moving = [abs(c) for c, vs in levels.items() if np.median(vs) > eps]
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
    active = mask & (np.abs(cmd) > 1e-3)
    min_hold = max(5, round(0.5 / s.dt))
    gf: list[float] = []
    gr: list[float] = []
    tf: list[float] = []
    tr: list[float] = []
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
        (gf if sign > 0 else gr).append(vss / ceff)
        if not math.isnan(tau):
            (tf if sign > 0 else tr).append(tau)
    return gf, gr, tf, tr


def fit_decay_tau(
    s: Session, cmd: np.ndarray, vel: np.ndarray, mask: np.ndarray, eps: float
) -> float:
    """Coast (decel) time constant from the zero-command tails after each step: |v| = V0 exp(-t/tau)."""
    quiet = mask & (np.abs(cmd) < 0.02)
    taus: list[float] = []
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
        if slope < 0:
            taus.append(-1.0 / slope)
    return float(np.median(taus)) if taus else float("nan")


def fit_steer_brake(s: Session) -> float:
    """Fractional forward-speed loss per unit |angular command| during the steer-brake grid.

    One steady-state forward-speed sample per grid cell (last half of each drive segment), so the rising
    transient does not bias the slope. speed/speed0 = 1 - coeff*|ang|.
    """
    drive = _mask(s, "steer_brake") & (s.cmd_lin > 0.5)
    levels: dict[float, float] = {}
    for a, b in _runs(drive):
        if b - a < max(5, round(0.5 / s.dt)):
            continue
        ang = round(float(np.median(np.abs(s.cmd_ang[a:b]))), 2)
        spd = float(np.median(s.v[int(0.5 * (a + b)) : b]))
        levels.setdefault(ang, spd)
    if 0.0 not in levels or len(levels) < 3:
        return float("nan")
    v0 = levels[0.0]
    if v0 <= 1e-3:
        return float("nan")
    angs = np.array([a for a in levels if a > 0.0])
    loss = np.array([1.0 - levels[a] / v0 for a in angs])
    coeff, *_ = np.linalg.lstsq(angs[:, None], loss, rcond=None)
    return float(coeff[0])


def fit_actuation_lag(s: Session) -> float:
    """Transport lag (ms) between command steps and motion, from the latency battery.

    Cross-correlates the command with acceleration (dv/dt), not velocity: acceleration responds at motion
    onset, so the estimate is the pure transport delay (Crossfire + ESC + mechanical) and is not inflated
    by the first-order velocity rise the way a command-vs-velocity correlation would be.
    """
    m = _mask(s, "latency")
    if m.sum() < 16:
        return float("nan")
    cmd = s.cmd_lin[m]
    accel = np.concatenate([[0.0], np.diff(s.v_raw[m])])
    cmd = cmd - cmd.mean()
    accel = accel - accel.mean()
    best_lag, best_corr = 0, -np.inf
    max_lag = min(12, len(accel) // 2)
    for lag in range(max_lag + 1):
        corr = float(np.dot(cmd[: len(cmd) - lag], accel[lag:]))
        if corr > best_corr:
            best_corr, best_lag = corr, lag
    return best_lag * s.dt * 1000.0


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------


def fit_all(s: Session) -> dict[str, float]:
    lin = _mask(s, "lin_step") | _mask(s, "lin_max")
    ang = _mask(s, "ang_step") | _mask(s, "ang_max")
    eps_lin = max(MOTION_EPS_LIN, ONSET_SIGMA * s.v_noise)
    eps_ang = max(MOTION_EPS_ANG, ONSET_SIGMA * s.w_noise)
    # The deadzone test compares a per-level MEDIAN over a ~0.8 s hold, which averages down the noise by
    # ~sqrt(N); use a gentler threshold than the per-sample motion floor so the onset is not pushed high.
    dz_eps_lin = max(MOTION_EPS_LIN, 2.0 * s.v_noise)
    dz_eps_ang = max(MOTION_EPS_ANG, 2.0 * s.w_noise)

    dz_lin_fwd = fit_deadzone(
        s.cmd_lin[_mask(s, "lin_deadzone_fwd")], s.v[_mask(s, "lin_deadzone_fwd")], 1.0, dz_eps_lin
    )
    dz_lin_rev = fit_deadzone(
        s.cmd_lin[_mask(s, "lin_deadzone_rev")], s.v[_mask(s, "lin_deadzone_rev")], -1.0, dz_eps_lin
    )
    dz_ang_l = fit_deadzone(
        s.cmd_ang[_mask(s, "ang_deadzone_left")],
        s.w[_mask(s, "ang_deadzone_left")],
        1.0,
        dz_eps_ang,
    )
    dz_ang_r = fit_deadzone(
        s.cmd_ang[_mask(s, "ang_deadzone_right")],
        s.w[_mask(s, "ang_deadzone_right")],
        -1.0,
        dz_eps_ang,
    )

    gf, gr, tf, tr = fit_drive_segments(s, s.cmd_lin, s.v, lin, dz_lin_fwd, dz_lin_rev, eps_lin)
    # Angular: buckets by sign (left/right); combine both for a single symmetric max_angular_speed.
    ga_l, ga_r, ta_l, ta_r = fit_drive_segments(s, s.cmd_ang, s.w, ang, dz_ang_l, dz_ang_r, eps_ang)

    def med(xs: list[float]) -> float:
        return float(np.median(xs)) if xs else float("nan")

    return {
        "linear_deadzone_fwd": dz_lin_fwd,
        "linear_deadzone_rev": dz_lin_rev,
        "angular_deadzone_left": dz_ang_l,
        "angular_deadzone_right": dz_ang_r,
        "max_linear_speed_fwd": med(gf),
        "max_linear_speed_rev": med(gr),
        "max_angular_speed": med(ga_l + ga_r),
        "tau_linear_accel": med(tf + tr),
        "tau_linear_decel": fit_decay_tau(s, s.cmd_lin, s.v, _mask(s, "lin_step"), eps_lin),
        "tau_angular_accel": med(ta_l + ta_r),
        "steer_brake_coeff": fit_steer_brake(s),
        "actuation_lag_ms": fit_actuation_lag(s),
        "truth_dt_ms": s.truth_dt * 1000.0,
    }


def print_report(p: dict[str, float]) -> None:
    def f(key: str, unit: str = "") -> str:
        v = p[key]
        return "n/a" if (isinstance(v, float) and math.isnan(v)) else f"{v:.3f}{unit}"

    print("\n=== Mrs Buff MK3 plant fit ===")
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
        f"  actuation lag     {f('actuation_lag_ms', ' ms')}  (+/- {p['truth_dt_ms']:.0f} ms: one truth frame)"
    )
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


def plot_session(s: Session, out: Path) -> None:
    try:
        import matplotlib
    except ModuleNotFoundError:
        print("\nmatplotlib not installed; skipping --plot (the fit above is unaffected).")
        return
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    ax[0].plot(s.t, s.cmd_lin, label="cmd_lin", lw=0.8)
    ax[0].plot(s.t, s.v, label="forward speed (m/s)", lw=0.8)
    ax[0].legend(fontsize=8)
    ax[0].set_ylabel("linear")
    ax[1].plot(s.t, s.cmd_ang, label="cmd_ang", lw=0.8)
    ax[1].plot(s.t, s.w, label="yaw rate (rad/s)", lw=0.8)
    ax[1].legend(fontsize=8)
    ax[1].set_ylabel("angular")
    ax[1].set_xlabel("t (s)")
    fig.suptitle("Calibration session: command vs measured velocity")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
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
    if args.plot:
        plot_session(s, args.plot)


if __name__ == "__main__":
    main()
