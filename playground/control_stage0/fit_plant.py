"""Fit the kinematic sim plant (max speeds + coast time constants) to real recordings.

In the autonomous segments of a fight recording, the actual transmitted drive command (opentx
channels = the trainer-mode mix of navigation + driver sticks) drove the robot, and the perceived
pose (our_x/our_y/yaw) is the response. (The navigation suggestion linear_x/angular_z does NOT drive
the robot in trainer mode, so it must not be used.) We identify a first-order velocity model per
axis from finite-differenced velocity:

    v[k+1] = a * v[k] + b * cmd[k]            (least squares, no intercept)
    tau    = -dt / ln(a)                       coast time constant (s)
    gain   = b / (1 - a)                       m/s (or rad/s) per unit command  -> max_*_speed

so the fit absorbs the whole real chain (ESC deadzone, drivetrain, mixing) into the aggregate
command->motion response the kinematic plant needs. An open-loop replay of the fitted plant against
the recorded trajectory validates it.

Caveats:
 - Only autonomous frames are used (in manual segments the logged command did not drive the robot).
 - Poses are perception output (noisy, with the flat-plane bias); the velocity finite-difference
   adds noise, which biases the AR coefficient `a` slightly low (tau slightly short). Treat the fit
   as a calibrated starting point, not ground truth.
 - The kinematic plant clips |command| to 1. linear_x stays within that, but angular_z can exceed
   it on the real robot, so set max_angular_command in the nav config (or widen the plant clip) for the
   fit to apply faithfully at large angular commands.

Run:
    source scripts/activate_python.sh
    python playground/control_stage0/fit_plant.py \
        data/recordings/auto_battlebot_main_2026-05-02_*[0-9]_repaired.mcap --plot fit_plant.png
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import diag_io  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

DT_MIN, DT_MAX = 0.025, 0.08  # accept ~12-40 Hz steps
TELEPORT_M = 0.12  # reject pose jumps > this between frames (~3 m/s; perception spike/track switch)
SMOOTH_WINDOW = 3  # light pose smoothing (~120 ms); larger over-attenuates velocity vs tau ~0.1 s
CHANNEL_MAX = (
    1000.0  # opentx raw channel range [-1000, 1000] (kChannelMax in opentx_transmitter.cpp)
)
# Only fit the gain above the ESC deadzone; near-zero commands move ~0 m/s and would bias
# the through-origin slope toward zero (most gentle driving sits inside the deadzone).
GAIN_MIN_CMD = 0.2


@dataclass
class Segment:
    """A contiguous run of valid autonomous frames from one recording."""

    t: np.ndarray  # frame times (s)
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray  # rad
    cmd_lin: np.ndarray
    cmd_ang: np.ndarray


def _wrap(a: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(a), np.cos(a))


def _smooth(a: np.ndarray, window: int = SMOOTH_WINDOW) -> np.ndarray:
    """Centered moving average to tame perception-pose noise before differentiating."""
    if window < 2 or len(a) < window:
        return a
    kernel = np.ones(window) / window
    return np.convolve(a, kernel, mode="same")


def _best_lag(cmd: np.ndarray, vel: np.ndarray, max_lag: int = 4) -> int:
    """Frames the response lags the command (actuation latency), by max cross-correlation."""
    cmd = cmd - cmd.mean()
    vel = vel - vel.mean()
    best, best_corr = 0, -np.inf
    for lag in range(max_lag + 1):
        if lag >= len(vel):
            break
        c = float(np.dot(cmd[: len(cmd) - lag], vel[lag:]))
        if c > best_corr:
            best_corr, best = c, lag
    return best


def extract_segments(path: Path) -> list[Segment]:
    """Contiguous runs of autonomous frames with sane dt and no pose teleports.

    The command is the ACTUAL transmitted drive channel (opentx ch_linear/ch_angular = the trainer-
    mode mix of navigation + driver sticks), normalized to ~[-1,1]. This is what really drove the
    robot regardless of auto/manual, so we do not restrict to autonomous frames; the navigation
    suggestion (linear_x/angular_z) is not used because the driver adds/subtracts from it.
    """
    df = diag_io.load_diagnostics(path)
    needed = {"our_x", "our_y", "our_yaw_deg", "ch_linear", "ch_angular"}
    if not needed.issubset(df.columns):
        return []
    df = df.dropna(subset=["our_x", "our_y", "our_yaw_deg", "ch_linear", "ch_angular"])
    t = df["timestamp_ns"].to_numpy() / 1e9
    x = df["our_x"].to_numpy()
    y = df["our_y"].to_numpy()
    yaw = np.deg2rad(df["our_yaw_deg"].to_numpy())
    cl = df["ch_linear"].to_numpy() / CHANNEL_MAX
    ca = -df["ch_angular"].to_numpy() / CHANNEL_MAX  # reverse_angular_channel = true (main.toml)

    dt = np.diff(t)
    step = np.hypot(np.diff(x), np.diff(y))
    good = (dt > DT_MIN) & (dt < DT_MAX) & (step < TELEPORT_M)

    segments: list[Segment] = []
    i = 0
    n = len(good)
    while i < n:
        if not good[i]:
            i += 1
            continue
        j = i
        while j < n and good[j]:
            j += 1
        sl = slice(i, j + 1)  # frames i..j inclusive
        if j - i >= 5:  # need a few steps to be useful
            segments.append(Segment(t[sl], x[sl], y[sl], yaw[sl], cl[sl], ca[sl]))
        i = j + 1
    return segments


def axis_velocity(seg: Segment, angular: bool) -> tuple[np.ndarray, np.ndarray]:
    """Per-step smoothed observed velocity and the command, for one axis."""
    dt = np.diff(seg.t)
    if angular:
        vel = _wrap(np.diff(_smooth(seg.yaw))) / dt
        cmd = seg.cmd_ang[:-1]
    else:
        # Signed forward speed: displacement projected on the heading.
        dx, dy = np.diff(_smooth(seg.x)), np.diff(_smooth(seg.y))
        vel = (dx * np.cos(seg.yaw[:-1]) + dy * np.sin(seg.yaw[:-1])) / dt
        cmd = seg.cmd_lin[:-1]
    return vel, cmd


@dataclass
class AxisFit:
    max_speed: float  # gain: m/s (or rad/s) per unit command, robust static slope
    tau: float  # coast time constant (s), from the AR coefficient
    r2: float  # AR fit quality
    lag_frames: int  # command -> response lag (actuation latency)
    n: int


def fit_axis(segments: list[Segment], dt: float, angular: bool) -> AxisFit:
    """Identify a first-order command->velocity model, robust to pose noise.

    - The static gain (max_speed) is the through-origin slope of velocity on the lag-aligned
      command; OLS slope stays unbiased under velocity noise, so this is the trustworthy number.
    - tau comes from the AR(1) coefficient on the smoothed, lag-aligned velocity (noisier).
    """
    lags = [_best_lag(*axis_velocity(s, angular)) for s in segments if len(s.t) > 6]
    lag = int(np.median(lags)) if lags else 0

    v_all, u_all, v_now, v_next, u_now = [], [], [], [], []
    for seg in segments:
        vel, cmd = axis_velocity(seg, angular)
        if len(vel) <= lag + 2:
            continue
        v = vel[lag:]  # response
        u = cmd[: len(cmd) - lag] if lag else cmd  # command that drove it
        m = min(len(v), len(u))
        v, u = v[:m], u[:m]
        v_all.append(v)
        u_all.append(u)
        v_now.append(v[:-1])
        v_next.append(v[1:])
        u_now.append(u[:-1])

    va, ua = np.concatenate(v_all), np.concatenate(u_all)
    # Static gain: through-origin OLS slope on the active region (above the deadzone) so deadzone
    # points (cmd != 0 but v ~ 0) do not drag the slope toward zero.
    active = np.abs(ua) > GAIN_MIN_CMD
    vg, ug = (va[active], ua[active]) if active.sum() > 20 else (va, ua)
    max_speed = float(np.dot(vg, ug) / np.dot(ug, ug)) if np.dot(ug, ug) > 0 else float("nan")

    v0, v1, u0 = np.concatenate(v_now), np.concatenate(v_next), np.concatenate(u_now)
    coef, *_ = np.linalg.lstsq(np.column_stack([v0, u0]), v1, rcond=None)
    a = float(coef[0])
    pred = a * v0 + coef[1] * u0
    ss_res = float(np.sum((v1 - pred) ** 2))
    ss_tot = float(np.sum((v1 - v1.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    tau = -dt / math.log(a) if 0.0 < a < 1.0 else float("nan")

    return AxisFit(max_speed, tau, r2, lag, len(v0))


def replay(
    seg: Segment, params: dict[str, float], reanchor_frames: int | None = None
) -> tuple[np.ndarray, np.ndarray]:
    """Forward sim of the fitted plant over a segment (mirrors Plant.step, no walls).

    If reanchor_frames is set, the pose and velocity are reset to the observed values every that
    many frames. Open-loop replay (reanchor_frames=None) integrates yaw-rate error into unbounded
    heading drift over long horizons, so it is a poor fidelity metric; the closed-loop sim never
    runs the plant open-loop for more than a tick, so short-horizon error is what matters.
    """
    dt = np.diff(seg.t)
    n = len(seg.t)
    px, py = np.empty(n), np.empty(n)
    px[0], py[0] = seg.x[0], seg.y[0]
    yaw = seg.yaw[0]
    v_obs, _ = axis_velocity(seg, angular=False)
    w_obs, _ = axis_velocity(seg, angular=True)
    v, w = float(v_obs[0]), float(w_obs[0])
    for k in range(n - 1):
        if reanchor_frames and k > 0 and k % reanchor_frames == 0:
            px[k], py[k], yaw = seg.x[k], seg.y[k], seg.yaw[k]
            v, w = float(v_obs[min(k, len(v_obs) - 1)]), float(w_obs[min(k, len(w_obs) - 1)])
        a_lin = 1.0 - math.exp(-dt[k] / max(params["tau_linear"], 1e-4))
        a_ang = 1.0 - math.exp(-dt[k] / max(params["tau_angular"], 1e-4))
        v += a_lin * (np.clip(seg.cmd_lin[k], -1, 1) * params["max_linear_speed"] - v)
        w += a_ang * (np.clip(seg.cmd_ang[k], -1, 1) * params["max_angular_speed"] - w)
        yaw = math.atan2(math.sin(yaw + w * dt[k]), math.cos(yaw + w * dt[k]))
        px[k + 1] = px[k] + v * math.cos(yaw) * dt[k]
        py[k + 1] = py[k] + v * math.sin(yaw) * dt[k]
    return px, py


def _short_label(path: Path) -> str:
    s = path.stem
    for prefix in ("auto_battlebot_main.toml_", "auto_battlebot_main_"):
        if s.startswith(prefix):
            s = s[len(prefix) :]
    return s.replace("_repaired", "")


def plot_per_recording(
    per_file: list[tuple[str, list[Segment]]], params: dict[str, float], out: Path
) -> None:
    """One row per recording: open-loop trajectory (recorded vs replay) + speed + yaw rate.

    Uses the longest continuous segment of each recording. The open-loop replay drifts over long
    horizons (see the printed short-horizon metrics); the trajectory view is kept to the recorded
    path so it stays readable while the replay's divergence is still visible.
    """
    files = [(lbl, segs) for lbl, segs in per_file if segs]
    nrows = len(files)
    fig, axes = plt.subplots(nrows, 3, figsize=(15, 3.6 * nrows), squeeze=False)
    for i, (label, segs) in enumerate(files):
        seg = max(segs, key=lambda s: len(s.t))
        px, py = replay(seg, params)  # open-loop (no re-anchoring)
        t = seg.t - seg.t[0]
        v_obs, _ = axis_velocity(seg, angular=False)
        w_obs, _ = axis_velocity(seg, angular=True)

        ax = axes[i][0]
        ax.plot(seg.x, seg.y, label="recorded", lw=1.3)
        ax.plot(px, py, "--", color="crimson", lw=1.3, label="open-loop replay")
        ax.plot(seg.x[0], seg.y[0], "ko", ms=6)
        ax.set_aspect("equal")
        margin = 0.6 * max(float(np.ptp(seg.x)), float(np.ptp(seg.y)), 0.5)
        ax.set_xlim(seg.x.min() - margin, seg.x.max() + margin)
        ax.set_ylim(seg.y.min() - margin, seg.y.max() + margin)
        ax.legend(fontsize=7, loc="upper right")
        ax.set_ylabel(f"{label}\ny (m)", fontsize=8)
        ax.set_xlabel("x (m)")

        axes[i][1].plot(t[:-1], v_obs, lw=0.7)
        axes[i][1].set_xlabel("t (s)")
        axes[i][1].set_ylabel("m/s")
        axes[i][2].plot(t[:-1], w_obs, lw=0.7, color="darkorange")
        axes[i][2].set_xlabel("t (s)")
        axes[i][2].set_ylabel("rad/s")

    axes[0][0].set_title("open-loop trajectory")
    axes[0][1].set_title("forward speed (recorded)")
    axes[0][2].set_title("yaw rate (recorded)")
    fig.suptitle("Plant fit per recording (longest continuous segment)", fontsize=12)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("files", nargs="+", type=Path, help="fight MCAP recording(s)")
    ap.add_argument("--plot", type=Path, default=None, help="write a validation plot to this path")
    args = ap.parse_args()

    per_file: list[tuple[str, list[Segment]]] = []
    segments: list[Segment] = []
    for path in args.files:
        if not path.exists():
            sys.exit(f"not found: {path}")
        segs = extract_segments(path)
        per_file.append((_short_label(path), segs))
        segments.extend(segs)
    if not segments:
        sys.exit("no usable driving segments found")

    all_dt = np.concatenate([np.diff(s.t) for s in segments])
    dt = float(np.median(all_dt))

    lin = fit_axis(segments, dt, angular=False)
    ang = fit_axis(segments, dt, angular=True)

    print(f"segments: {len(segments)}  | median dt: {dt * 1000:.1f} ms  | samples ~{lin.n}")
    print(
        "LINEAR :  max_linear_speed = %.3f m/s    tau_linear = %.3f s   (lag %d fr, R2 %.2f)"
        % (lin.max_speed, lin.tau, lin.lag_frames, lin.r2)
    )
    print(
        "ANGULAR:  max_angular_speed = %.3f rad/s  tau_angular = %.3f s  (lag %d fr, R2 %.2f)"
        % (ang.max_speed, ang.tau, ang.lag_frames, ang.r2)
    )
    print(
        f"actuation lag: ~{ang.lag_frames * dt * 1000:.0f} ms (angular), "
        f"{lin.lag_frames * dt * 1000:.0f} ms (linear)"
    )

    params = {
        "max_linear_speed": lin.max_speed,
        "max_angular_speed": ang.max_speed,
        "tau_linear": lin.tau if lin.tau == lin.tau else 0.1,  # fall back if AR gave nan
        "tau_angular": ang.tau if ang.tau == ang.tau else 0.1,
    }

    # Validate by short-horizon replay (re-anchored to the observed pose) on the longest segment.
    # This is the relevant metric: the closed-loop sim corrects every tick, so it never integrates
    # the plant open-loop for long. Pure open-loop drift is reported only for contrast.
    longest = max(segments, key=lambda s: len(s.t))

    def horizon_rmse(frames: int | None) -> float:
        px, py = replay(longest, params, reanchor_frames=frames)
        return float(np.sqrt(np.mean((px - longest.x) ** 2 + (py - longest.y) ** 2)))

    print(f"\nreplay fidelity on longest segment ({len(longest.t)} frames):")
    for horizon_s in (0.5, 1.0, 2.0):
        print(
            f"  re-anchored every {horizon_s:.1f}s: position RMSE = "
            f"{horizon_rmse(round(horizon_s / dt)) * 100:.1f} cm"
        )
    print(
        f"  pure open-loop (drift-dominated, not a fidelity metric): "
        f"{horizon_rmse(None) * 100:.0f} cm"
    )

    print("\nPaste into simulation/kinematic_sim.toml [our_robot]:")
    for key in ("max_linear_speed", "max_angular_speed", "tau_linear", "tau_angular"):
        print(f"  {key} = {params[key]:.3f}")

    if args.plot is not None:
        plot_per_recording(per_file, params, args.plot)
        print(f"\nwrote {args.plot}")


if __name__ == "__main__":
    main()
