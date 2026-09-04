"""Render open-loop prediction videos against the measured match track.

For each joined recording, the plant model integrates forward from the measured
pose for a fixed horizon (default 3 s), then resets to the measurement and goes
again. The video shows where the model walks off and how fast: measured robot
in blue, predicted ghost in orange, error readout in the corner.

Prediction only runs while the auto switch was up (in manual segments the
logged channels did not drive the robot, so there is nothing to predict from)
and resets wait for a frame with a measured pose.

Usage:
    source scripts/activate_python.sh
    python playground/calibration/render_match_prediction.py \\
        --pair <replay>.mcap:<original>.mcap:2026-05-01_17-42-20 \\
        --params playground/calibration/out/plant_match.toml \\
        --cmd-lead-ms 40 \\
        --out-dir playground/calibration/out/match_fit/videos
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
from matplotlib.patches import Circle, Polygon, Rectangle

from auto_battlebot.calibration.match_windows import (
    MatchRun,
    _fill_grid,
    build_match_run,
    load_commands,
    load_replay_track,
)
from auto_battlebot.plant import (
    FULL_MODEL,
    ModelStructure,
    PlantParams,
    PlantState,
    delayed_commands,
    integrate_step,
    steady_state,
    wrap_angle,
)

ROBOT_HALF_M = 0.11  # matches sim_mrs_buff_mk3 radius
SUBSTEP_S = 0.005  # coarser than the fit's 2 ms; invisible at video scale


# ---------------------------------------------------------------------------
# Prediction
# ---------------------------------------------------------------------------


@dataclass
class PredictedTrack:
    pose: np.ndarray  # (N, 3) x, y, theta; nan where no prediction runs
    segment: np.ndarray  # (N,) reset segment id, -1 where none


def predict_track(
    run: MatchRun,
    params: PlantParams,
    structure: ModelStructure,
    reset_s: float,
) -> PredictedTrack:
    applied = structure.apply(params)
    u_lin, u_ang = delayed_commands(run.t, run.cmd_lin, run.cmd_ang, applied.delay_s, run.dt)
    n = len(run.t)
    pose = np.full((n, 3), np.nan)
    segment = np.full(n, -1, dtype=int)
    reset_steps = max(1, int(round(reset_s / run.dt)))
    substeps = max(1, int(round(run.dt / SUBSTEP_S)))
    sub_dt = run.dt / substeps

    state: PlantState | None = None
    seg = -1
    next_reset = 0
    for k in range(n):
        if not run.auto[k]:
            # Logged channels were not driving the robot; nothing to predict.
            state = None
            next_reset = k + 1
            continue
        if k >= next_reset and np.isfinite(run.x[k]):
            state = PlantState(
                x=float(run.x[k]),
                y=float(run.y[k]),
                theta=float(run.theta[k]),
                v=float(run.v[k]) if np.isfinite(run.v[k]) else 0.0,
                w=float(run.w[k]) if np.isfinite(run.w[k]) else 0.0,
            )
            seg += 1
            next_reset = k + reset_steps
        if state is None:
            continue
        pose[k] = (float(state.x), float(state.y), float(state.theta))
        segment[k] = seg
        v_target, w_target = steady_state(np.asarray(u_lin[k]), np.asarray(u_ang[k]), applied)
        for _ in range(substeps):
            state = integrate_step(state, v_target, w_target, sub_dt, applied)
    return PredictedTrack(pose=pose, segment=segment)


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------


def _robot_corners(x: float, y: float, theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    rot = np.array([[c, -s], [s, c]])
    square = ROBOT_HALF_M * np.array([[1, 1], [-1, 1], [-1, -1], [1, -1]], dtype=float)
    return square @ rot.T + (x, y)


def render_video(
    run: MatchRun,
    predicted: PredictedTrack,
    opp: np.ndarray,  # (N, 2), nan when absent
    out_path: Path,
    *,
    title: str,
    reset_s: float,
    holdout_from_s: float | None,
) -> None:
    n = len(run.t)
    x_lo, x_hi = np.nanpercentile(run.x, [0.2, 99.8])
    y_lo, y_hi = np.nanpercentile(run.y, [0.2, 99.8])

    fig, ax = plt.subplots(figsize=(7.2, 7.6))
    ax.set_aspect("equal")
    pad = 0.30
    ax.set_xlim(x_lo - pad, x_hi + pad)
    ax.set_ylim(y_lo - pad, y_hi + pad)
    ax.set_title(title, fontsize=10)
    ax.add_patch(
        Rectangle(
            (x_lo, y_lo),
            x_hi - x_lo,
            y_hi - y_lo,
            fill=False,
            edgecolor="0.6",
            linestyle="--",
            linewidth=1.0,
        )
    )
    ax.set_xlabel("field x [m]")
    ax.set_ylabel("field y [m]")

    meas_poly = Polygon(np.zeros((4, 2)), closed=True, facecolor="tab:blue", alpha=0.85, zorder=5)
    pred_poly = Polygon(
        np.zeros((4, 2)),
        closed=True,
        facecolor="none",
        edgecolor="tab:orange",
        linewidth=2.0,
        zorder=6,
    )
    ax.add_patch(meas_poly)
    ax.add_patch(pred_poly)
    (meas_head,) = ax.plot([], [], color="white", linewidth=1.6, zorder=7)
    (pred_head,) = ax.plot([], [], color="tab:orange", linewidth=1.6, zorder=7)
    (meas_trail,) = ax.plot([], [], color="tab:blue", linewidth=1.0, alpha=0.6, zorder=3)
    (pred_trail,) = ax.plot([], [], color="tab:orange", linewidth=1.2, alpha=0.8, zorder=4)
    (err_line,) = ax.plot([], [], color="tab:red", linewidth=1.0, linestyle=":", zorder=4)
    opp_patch = Circle((0, 0), 0.11, facecolor="0.4", alpha=0.7, zorder=5)
    ax.add_patch(opp_patch)
    opp_patch.set_visible(False)

    status = ax.text(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=9,
        family="monospace",
        zorder=10,
        bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "none"},
    )
    legend = (
        "blue = measured (replay perception)   orange = plant prediction\n"
        f"prediction resets to measurement every {reset_s:.0f} s"
    )
    ax.text(
        0.02,
        0.02,
        legend,
        transform=ax.transAxes,
        va="bottom",
        ha="left",
        fontsize=8,
        zorder=10,
        bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "none"},
    )

    # Stick readout: two horizontal bars, linear and angular in [-1, 1].
    bar_ax = ax.inset_axes((0.70, 0.015, 0.28, 0.10))
    bar_ax.set_xlim(-1, 1)
    bar_ax.set_ylim(-0.5, 1.5)
    bar_ax.axvline(0.0, color="0.7", linewidth=0.8)
    bar_ax.set_yticks([0, 1], labels=["ang", "lin"], fontsize=7)
    bar_ax.set_xticks([-1, 0, 1], labels=["-1", "cmd", "+1"], fontsize=7)
    lin_bar = bar_ax.barh(1, 0.0, height=0.6, color="tab:blue")[0]
    ang_bar = bar_ax.barh(0, 0.0, height=0.6, color="tab:green")[0]

    trail_steps = max(1, int(round(reset_s / run.dt)))

    def update(k: int):
        meas_ok = bool(np.isfinite(run.x[k]))
        meas_poly.set_visible(meas_ok)
        meas_head.set_visible(meas_ok)
        if meas_ok:
            meas_poly.set_xy(_robot_corners(run.x[k], run.y[k], run.theta[k]))
            meas_head.set_data(
                [run.x[k], run.x[k] + ROBOT_HALF_M * np.cos(run.theta[k])],
                [run.y[k], run.y[k] + ROBOT_HALF_M * np.sin(run.theta[k])],
            )
        lo = max(0, k - trail_steps)
        meas_trail.set_data(run.x[lo : k + 1], run.y[lo : k + 1])

        seg = predicted.segment[k]
        pred_ok = seg >= 0
        pred_poly.set_visible(pred_ok)
        pred_head.set_visible(pred_ok)
        err_text = "pred    off (manual)"
        if pred_ok:
            px, py, pth = predicted.pose[k]
            pred_poly.set_xy(_robot_corners(px, py, pth))
            pred_head.set_data(
                [px, px + ROBOT_HALF_M * np.cos(pth)],
                [py, py + ROBOT_HALF_M * np.sin(pth)],
            )
            sel = np.flatnonzero(predicted.segment[: k + 1] == seg)
            pred_trail.set_data(predicted.pose[sel, 0], predicted.pose[sel, 1])
            since_reset = (k - sel[0]) * run.dt
            if meas_ok:
                err_mm = float(np.hypot(px - run.x[k], py - run.y[k])) * 1e3
                err_deg = float(np.degrees(abs(wrap_angle(pth - run.theta[k]))))
                err_line.set_data([px, run.x[k]], [py, run.y[k]])
                err_text = (
                    f"pred    +{since_reset:4.1f} s   err {err_mm:5.0f} mm {err_deg:5.1f} deg"
                )
            else:
                err_line.set_data([], [])
                err_text = f"pred    +{since_reset:4.1f} s   (no measurement)"
        else:
            pred_trail.set_data([], [])
            err_line.set_data([], [])

        if np.isfinite(opp[k, 0]):
            opp_patch.set_center((opp[k, 0], opp[k, 1]))
            opp_patch.set_visible(True)
        else:
            opp_patch.set_visible(False)

        lin_bar.set_width(float(run.cmd_lin[k]))
        ang_bar.set_width(float(run.cmd_ang[k]))

        flags = []
        if not run.auto[k]:
            flags.append("MANUAL")
        if not np.isfinite(run.x[k]):
            flags.append("dropout")
        if run.contact[k]:
            flags.append("contact-gated")
        if holdout_from_s is not None:
            flags.append("holdout" if run.t[k] >= holdout_from_s else "train")
        status.set_text(f"t = {run.t[k]:6.1f} s   {' '.join(flags)}\n{err_text}")
        return ()

    fps = 1.0 / run.dt
    writer = animation.FFMpegWriter(fps=fps, bitrate=3000)
    anim = animation.FuncAnimation(fig, update, frames=n, blit=False)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    anim.save(str(out_path), writer=writer, dpi=110)
    plt.close(fig)
    print(f"wrote {out_path} ({n} frames at {fps:.1f} fps, {n * run.dt:.0f} s)")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def parse_triple(text: str) -> tuple[Path, Path, str]:
    parts = text.split(":")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected <replay.mcap>:<original.mcap>:<name>")
    return Path(parts[0]), Path(parts[1]), parts[2]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pair",
        type=parse_triple,
        action="append",
        required=True,
        metavar="REPLAY:ORIGINAL:NAME",
    )
    parser.add_argument(
        "--params",
        type=Path,
        default=Path("playground/calibration/out/plant_match.toml"),
    )
    parser.add_argument("--reset-s", type=float, default=3.0)
    parser.add_argument(
        "--cmd-lead-ms",
        type=float,
        default=40.0,
        help="same command lead the fit ran with; the params' delay_s lives on that timeline",
    )
    parser.add_argument(
        "--holdout-fraction",
        type=float,
        default=0.30,
        help="label train/holdout on the fit recording (name containing 17-42-20)",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("playground/calibration/out/match_fit/videos"),
    )
    args = parser.parse_args()

    params = PlantParams.from_toml(args.params)
    print(f"plant: {args.params} (delay {params.delay_s * 1000:.0f} ms on led timeline)")

    for replay, original, name in args.pair:
        print(f"\n{name}: loading")
        track = load_replay_track(replay)
        commands = load_commands(original)
        if args.cmd_lead_ms:
            commands.t_ns = commands.t_ns - int(args.cmd_lead_ms * 1e6)
        run = build_match_run(track, commands, name=name, role="video")
        _t, grid, _dt, _t0 = _fill_grid(track)
        opp = np.stack([grid["opp_x"], grid["opp_y"]], axis=1)

        predicted = predict_track(run, params, FULL_MODEL, args.reset_s)
        n_pred = int(np.count_nonzero(predicted.segment >= 0))
        n_seg = int(predicted.segment.max()) + 1
        print(f"  {len(run.t)} frames, {n_pred} predicted, {n_seg} reset segments")

        holdout_from_s = None
        if "17-42-20" in name:
            holdout_from_s = float(run.t[int(len(run.t) * (1 - args.holdout_fraction))])

        render_video(
            run,
            predicted,
            opp,
            args.out_dir / f"prediction_reset_{name}.mp4",
            title=(f"{name}: match-fit plant open loop, reset every {args.reset_s:.0f} s"),
            reset_s=args.reset_s,
            holdout_from_s=holdout_from_s,
        )


if __name__ == "__main__":
    main()
