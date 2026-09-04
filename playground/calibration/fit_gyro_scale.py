"""Fit the gyro scale factor from rotations through a known angle (runbook E3).

Integrating the gyro over a rotation of known size and comparing to that size gives one
number: how much the gyro over- or under-reports. Every heading estimate is multiplied by it,
so an unmeasured 2% here is 2% of heading error that no filter tuning removes.

    source scripts/activate_python.sh
    python playground/calibration/fit_gyro_scale.py \\
        --run 3600 LOG-a.TXT \\
        --run -3600 LOG-b.TXT \\
        --plot playground/calibration/out/gyro_scale.png

`--run` takes the true rotation in degrees, signed: positive counter-clockwise viewed from
above. Run both directions. A scale that differs between them is not a scale factor at all,
it is a mounting or axis-sign problem, and averaging the two would bury it.

The yaw axis comes from gravity, not from a note about how the jig is bolted on. At rest the
accelerometer reads +1 g on whichever axis points up, and the IMU puts gyro and accel on one
right-handed triad, so up plus the right-hand rule fixes positive yaw as counter-clockwise
from above. That survives a tilted remount, which no single gyro axis would.

Bias dominates the error budget on a slow rotation. A 0.05 deg/s bias over a 20 s turn is
1 deg, so the bias is taken from the still hold that opens the run and ramped to the one that
closes it. A run with only one still hold gets a constant bias and says so.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from auto_battlebot.velocity_jig import (
    find_still_segments,
    read_jig_log,
    still_stats,
)

# Below this the robot counts as stationary for the purpose of finding where the turn starts
# and stops. Well above the bias, well below a hand rotation.
TURN_EPS_DPS = 1.0


@dataclass
class Turn:
    """One rotation through a known angle."""

    path: Path
    reference_deg: float
    measured_deg: float
    t: np.ndarray
    yaw_dps: np.ndarray
    angle_deg: np.ndarray
    t0: float  # rotation start, seconds into the log
    t1: float
    stills: int
    bias_drift_dps: float
    up: np.ndarray
    peak_dps: float
    saturated: list[str]

    @property
    def scale(self) -> float:
        """Multiply raw gyro by this to get true angle."""
        return self.reference_deg / self.measured_deg if self.measured_deg else float("nan")

    @property
    def error(self) -> float:
        return self.measured_deg - self.reference_deg


def solve_turn(path: Path, reference_deg: float) -> Turn:
    log = read_jig_log(path)
    stills = find_still_segments(log)
    if not stills:
        raise ValueError(f"{path.name}: no still hold found, so no bias and no gravity axis")

    pre = still_stats(log, stills[0])
    up = pre.up
    if len(stills) > 1:
        post = still_stats(log, stills[-1])
        # Ramp the bias between the two holds. Gyro bias drifts, mostly thermally, and a
        # constant taken from the start leaves the drift integrated over the whole turn.
        span = max(post.t_mid - pre.t_mid, 1e-9)
        frac = np.clip((log.t - pre.t_mid) / span, 0.0, 1.0)[:, None]
        bias = pre.bias[None, :] * (1.0 - frac) + post.bias[None, :] * frac
        drift = float(np.degrees(np.linalg.norm(post.bias - pre.bias)))
    else:
        bias = pre.bias[None, :]
        drift = float("nan")

    # JigLog.gyro is rad/s, not deg/s. Everything below is degrees, so convert here and
    # nowhere else: a factor of 57.3 buried mid-pipeline turns ten turns into one sixth of
    # one, which is exactly convincing enough to be believed.
    yaw = np.degrees((log.gyro - bias) @ up)
    angle = np.concatenate([[0.0], np.cumsum(0.5 * (yaw[1:] + yaw[:-1]) * np.diff(log.t))])

    # The turn is the stretch between the holds that brackets it. Integrating the whole log
    # would fold both holds' residual bias into the answer.
    moving = np.abs(yaw) > TURN_EPS_DPS
    if not moving.any():
        raise ValueError(f"{path.name}: nothing turned faster than {TURN_EPS_DPS} deg/s")
    lo, hi = int(np.argmax(moving)), int(len(moving) - np.argmax(moving[::-1]) - 1)
    measured = float(angle[hi] - angle[lo])

    return Turn(
        path=path,
        reference_deg=reference_deg,
        measured_deg=measured,
        t=log.t - log.t[0],
        yaw_dps=yaw,
        angle_deg=angle - angle[lo],
        t0=float(log.t[lo] - log.t[0]),
        t1=float(log.t[hi] - log.t[0]),
        stills=len(stills),
        bias_drift_dps=drift,
        up=up,
        peak_dps=float(np.abs(yaw).max()),
        saturated=[a.name for a in log.saturation() if a.count],
    )


def _fmt(value: float, digits: int = 2) -> str:
    return "n/a" if not np.isfinite(value) else f"{value:.{digits}f}"


def make_plot(path: Path, turns: list[Turn], scale: float) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(13, 8.5))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.15, 1.0], hspace=0.34, wspace=0.24)

    # Yaw rate, with the integrated stretch shaded. Everything outside the shading is a still
    # hold and contributes only bias, so seeing where the boundary falls matters.
    ax = fig.add_subplot(gs[0, 0])
    for i, turn in enumerate(turns):
        ax.plot(turn.t, turn.yaw_dps, lw=0.9, color=f"C{i}", label=turn.path.name)
        ax.axvspan(turn.t0, turn.t1, color=f"C{i}", alpha=0.12, lw=0)
    ax.axhline(0.0, color="0.5", lw=1.0)
    ax.set_xlabel("time (s)", fontsize=9)
    ax.set_ylabel("yaw rate about gravity (deg/s)", fontsize=9)
    ax.set_title("Yaw rate, with the integrated stretch shaded", fontsize=10)
    ax.legend(fontsize=8)

    # The integral, against what it should have reached.
    ax = fig.add_subplot(gs[0, 1])
    for i, turn in enumerate(turns):
        ax.plot(turn.t, turn.angle_deg, lw=1.2, color=f"C{i}", label=turn.path.name)
        ax.axhline(turn.reference_deg, color=f"C{i}", ls=":", lw=1.2)
        ax.plot([turn.t1], [turn.measured_deg], "o", ms=7, color=f"C{i}")
    ax.axhline(0.0, color="0.5", lw=1.0)
    ax.set_xlabel("time (s)", fontsize=9)
    ax.set_ylabel("integrated angle (deg)", fontsize=9)
    ax.set_title(
        "Integrated angle. Dotted is the true rotation, dot is what the gyro read.", fontsize=10
    )
    ax.legend(fontsize=8)

    # Measured against true, with the fitted scale through the origin.
    ax = fig.add_subplot(gs[1, 0])
    x = np.array([t.reference_deg for t in turns])
    y = np.array([t.measured_deg for t in turns])
    for i, turn in enumerate(turns):
        ax.plot([turn.reference_deg], [turn.measured_deg], "o", ms=8, color=f"C{i}")
    lim = max(np.abs(np.concatenate([x, y]))) * 1.15
    xs = np.linspace(-lim, lim, 10)
    ax.plot(xs, xs, "--", lw=1.0, color="0.6", label="perfect gyro")
    ax.plot(xs, xs / scale, "-", lw=1.4, color="0.2", label=f"fitted scale {scale:.4f}")
    ax.axhline(0.0, color="0.7", lw=0.8)
    ax.axvline(0.0, color="0.7", lw=0.8)
    ax.set_xlabel("true rotation (deg)", fontsize=9)
    ax.set_ylabel("gyro integral (deg)", fontsize=9)
    ax.set_title("Measured against true", fontsize=10)
    ax.legend(fontsize=8)

    # Per-run error, so a direction-dependent difference is obvious.
    ax = fig.add_subplot(gs[1, 1])
    names = [t.path.name for t in turns]
    errs = [t.error / abs(t.reference_deg) * 100.0 for t in turns]
    colours = [f"C{i}" for i in range(len(turns))]
    ax.bar(range(len(turns)), errs, color=colours)
    ax.axhline(0.0, color="0.4", lw=1.0)
    ax.axhspan(-1.0, 1.0, color="0.88", zorder=0, label="1% pass bar")
    ax.set_xticks(range(len(turns)))
    ax.set_xticklabels(
        [f"{n}\n{t.reference_deg:+.0f} deg" for n, t in zip(names, turns)], fontsize=8
    )
    ax.set_ylabel("gyro error (%)", fontsize=9)
    ax.set_title(
        "Error per run. A difference between directions is a sign problem, not a scale.", fontsize=9
    )
    ax.legend(fontsize=8)

    fig.suptitle(f"Gyro scale: {scale:.4f}  ({len(turns)} rotations)", fontsize=12)
    fig.savefig(path, dpi=110, bbox_inches="tight")
    plt.close(fig)
    print(f"\nwrote {path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--run",
        nargs="+",
        action="append",
        required=True,
        metavar=("DEGREES", "LOG"),
        help="true rotation in degrees (signed, + is counter-clockwise) then its logs",
    )
    parser.add_argument("--plot", type=Path, default=None)
    args = parser.parse_args()

    turns: list[Turn] = []
    for group in args.run:
        if len(group) < 2:
            parser.error(f"--run {group[0]} needs at least one log file")
        try:
            reference = float(group[0])
        except ValueError:
            parser.error(f"--run takes an angle first, got {group[0]!r}")
        for name in group[1:]:
            turns.append(solve_turn(Path(name), reference))

    print(
        f"{'log':<14}{'true deg':>10}{'gyro deg':>10}{'error deg':>11}"
        f"{'error %':>9}{'scale':>8}{'peak dps':>10}{'holds':>7}{'drift dps':>11}"
    )
    for t in turns:
        print(
            f"{t.path.name:<14}{t.reference_deg:>10.1f}{t.measured_deg:>10.1f}"
            f"{t.error:>11.2f}{t.error / abs(t.reference_deg) * 100:>9.2f}"
            f"{t.scale:>8.4f}{t.peak_dps:>10.1f}{t.stills:>7}"
            f"{_fmt(t.bias_drift_dps, 4):>11}"
        )

    # Through the origin over signed angles, so both directions constrain one number.
    x = np.array([t.reference_deg for t in turns])
    y = np.array([t.measured_deg for t in turns])
    slope = float(np.sum(x * y) / np.sum(x * x))
    scale = 1.0 / slope
    print(f"\ngyro scale = {scale:.4f}   (multiply integrated gyro angle by this)")
    print(f"  the gyro reads {(slope - 1.0) * 100:+.2f}% against the reference")

    ccw = [t for t in turns if t.reference_deg > 0]
    cw = [t for t in turns if t.reference_deg < 0]
    if ccw and cw:
        a = float(np.mean([t.scale for t in ccw]))
        b = float(np.mean([t.scale for t in cw]))
        print(
            f"\ndirection check: counter-clockwise {a:.4f}, clockwise {b:.4f}, "
            f"{abs(a - b) / max(abs(a), 1e-9):.2%} apart"
        )
        if abs(a - b) > 0.01 * max(abs(a), 1e-9):
            print(
                "  Over 1%. That points at a mounting or axis-sign problem rather than a"
                " scale factor. Sort that before accepting either number."
            )
        else:
            print("  Under 1%, so the two directions agree and this is a scale factor.")
    else:
        print(
            "\nOnly one direction was run. A scale that differs between directions is a"
            " sign problem, and one direction cannot show it. Run the other way too."
        )

    worst = max(abs(t.error / t.reference_deg) for t in turns)
    print(
        f"\nworst single-run error {worst:.2%}"
        + ("  (over the runbook's 1% bar)" if worst > 0.01 else "  (under the 1% bar)")
    )
    for t in turns:
        if t.stills < 2:
            print(
                f"  {t.path.name}: only {t.stills} still hold, so the bias is a constant"
                " rather than ramped. Hold still at both ends of the next one."
            )
        if t.saturated:
            print(f"  {t.path.name}: SATURATED on {', '.join(t.saturated)}")

    if args.plot:
        make_plot(args.plot, turns, scale)

    print(f"\n[gyro]\nscale = {scale:.4f}")


if __name__ == "__main__":
    main()
