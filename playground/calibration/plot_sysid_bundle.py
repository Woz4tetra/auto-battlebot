"""Plot a smoothed sysid bundle so each smoothing check can be judged by eye.

Reads a bundle written by ``smooth_tag_poses.py`` and writes, under ``<bundle>/plots/``:

- ``overview.png``: the whole session on one time axis. Which source fixed the pose at each
  moment (tag, YOLO, neither); the smoothed yaw rate against the robot IMU's on a log scale,
  with the 50 rad/s line no real drive crosses; pitch against the flat band the window gate
  uses; x and y against the box the near-rail gate assumes; and the fit windows, by kind.
- ``floor.png``: top-down, every tag and YOLO fix, the smoothed path, the box, and where each
  fit window starts.
- ``dropouts.png``: one top-down panel per tag dropout longer than ``--min-gap-s`` in which the
  robot stayed upright, 0.5 s either side. ``--compare`` overlays a second bundle of the same
  recording (for example one smoothed without ``--yolo``).

    python playground/calibration/plot_sysid_bundle.py \\
        playground/calibration/out/mujoco_sysid/<session>_yolo \\
        --compare playground/calibration/out/mujoco_sysid/<session>_tag
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
import numpy as np
import pandas as pd

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

TAG, YOLO, SMOOTH, OTHER, IMU = "#2a78d6", "#eda100", "#1baf7a", "#eb6834", "#6250d6"
INK, MUTED, SURF, GRID = "#0b0b0b", "#52514e", "#fcfcfb", "#e6e5e1"
ALIAS_RATE = 50.0  # rad/s: well past any real spin; a smoothed rate above it is an alias


def _style() -> None:
    plt.rcParams.update(
        {
            "font.size": 8,
            "axes.edgecolor": "#c3c2b7",
            "axes.labelcolor": INK,
            "xtick.color": MUTED,
            "ytick.color": MUTED,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "figure.facecolor": SURF,
            "axes.facecolor": SURF,
            "axes.grid": True,
            "grid.color": GRID,
            "grid.linewidth": 0.5,
        }
    )


def load(bundle: Path) -> dict:
    out = {
        "smoothed": pd.read_csv(bundle / "smoothed.csv"),
        "meas": pd.read_csv(bundle / "measurements.csv"),
        "cmd": pd.read_csv(bundle / "commands.csv"),
        "windows": pd.read_csv(bundle / "windows.csv"),
        "session": json.loads((bundle / "session.json").read_text()),
    }
    if "source" not in out["meas"]:
        out["meas"]["source"] = "tag"
    t0 = out["smoothed"]["stamp_ns"].iloc[0]
    for key in ("smoothed", "meas", "cmd"):
        out[key]["t"] = (out[key]["stamp_ns"] - t0) / 1e9
    out["windows"]["t"] = (out["windows"]["start_ns"] - t0) / 1e9
    out["t0"] = t0
    return out


def _imu_yaw_rate(cmd: pd.DataFrame, session: dict) -> tuple[np.ndarray, np.ndarray]:
    """BNO055 yaw rate, counter-clockwise, shifted by the lag the smoother measured."""
    agreement = session.get("imu_yaw_rate") or {}
    lag = float(agreement.get("lag_s") or 0.0)
    sign = -float(agreement.get("measured_sign") or 1.0)  # heading is clockwise
    t = cmd["t"].to_numpy() - lag
    heading = np.unwrap(np.radians(cmd["orientation_x"].to_numpy()))
    grid = np.arange(t[0], t[-1], 0.01)
    rate = sign * np.gradient(np.interp(grid, t, heading), grid)
    rate = np.convolve(rate, np.ones(5) / 5, mode="same")
    up = np.interp(grid, t, (cmd["is_upside_down"].to_numpy() == 0).astype(float)) > 0.5
    return grid[up], rate[up]


def overview(d: dict, out: Path) -> None:
    s, m, w, session = d["smoothed"], d["meas"], d["windows"], d["session"]
    opts = session["windows"]["options"]
    rest = float(opts["rest_pitch_rad"])
    limit = np.radians(float(opts["pitch_limit_deg"]))
    cx, cy = opts["box_center_xy"]
    half = float(opts["box_half_extent_m"]) - float(opts["robot_length_m"])
    fig, ax = plt.subplots(
        5,
        1,
        figsize=(13, 11),
        sharex=True,
        constrained_layout=True,
        gridspec_kw={"height_ratios": [0.5, 1.4, 1.0, 1.2, 0.5]},
    )

    ok = m[m["rejected"] == 0]
    for i, (src, color) in enumerate((("tag", TAG), ("yolo", YOLO))):
        tt = ok.loc[ok["source"] == src, "t"]
        ax[0].scatter(tt, np.full(len(tt), i), s=1, color=color, marker="|", label=src)
    ax[0].set_yticks([0, 1], ["tag", "YOLO"])
    ax[0].set_ylim(-0.6, 1.6)
    ax[0].set_title("A. Accepted fixes by source; white is a dropout nothing filled", loc="left")

    gi, ri = _imu_yaw_rate(d["cmd"], session)
    ax[1].semilogy(gi, np.abs(ri) + 1e-3, color=IMU, lw=0.5, label="|IMU yaw rate|")
    ax[1].semilogy(
        s["t"], np.abs(s["yaw_rate"]) + 1e-3, color=SMOOTH, lw=0.6, label="|smoothed yaw rate|"
    )
    ax[1].axhline(ALIAS_RATE, color=OTHER, ls="--", lw=0.8)
    ax[1].text(
        s["t"].iloc[0],
        ALIAS_RATE * 1.3,
        f"{ALIAS_RATE:.0f} rad/s: above this is an alias",
        color=OTHER,
        fontsize=7,
    )
    above = float((s["yaw_rate"].abs() > ALIAS_RATE).mean())
    ax[1].set_ylim(1e-2, 1e3)
    ax[1].set_ylabel("rad/s")
    ax[1].set_title(
        f"B. Yaw rate. Smoothed should trace the IMU; {above:.1%} of samples above "
        f"{ALIAS_RATE:.0f} rad/s",
        loc="left",
    )
    ax[1].legend(loc="upper right", frameon=False, ncol=2)

    up = s["upside_down"] == 0
    ax[2].plot(s["t"], np.degrees(np.where(up, s["pitch"], np.nan)), color=SMOOTH, lw=0.6)
    ax[2].axhspan(np.degrees(rest - limit), np.degrees(rest + limit), color=GRID, zorder=0)
    ax[2].axhline(np.degrees(rest), color=MUTED, lw=0.6, ls="--")
    ax[2].set_ylim(np.degrees(rest) - 30, np.degrees(rest) + 30)
    ax[2].set_ylabel("pitch, deg")
    ax[2].set_title(
        f"C. Smoothed pitch while upright; grey is the flat band "
        f"(rest {np.degrees(rest):.1f} +/- {np.degrees(limit):.0f} deg)",
        loc="left",
    )

    ax[3].plot(s["t"], s["x"] - cx, color=TAG, lw=0.7, label="x - box centre")
    ax[3].plot(s["t"], s["y"] - cy, color=OTHER, lw=0.7, label="y - box centre")
    for sgn in (-1, 1):
        ax[3].axhline(sgn * half, color=MUTED, ls="--", lw=0.8)
    ax[3].set_ylim(-1.5, 1.5)
    ax[3].set_ylabel("m")
    ax[3].set_title(
        "D. Position against the near-rail gate (dashed): outside it no window is cut", loc="left"
    )
    ax[3].legend(loc="upper right", frameon=False, ncol=2)

    colors = {"flat": SMOOTH, "nose_lift": OTHER}
    for kind, color in colors.items():
        ww = w[w["kind"] == kind]
        ax[4].broken_barh(
            list(zip(ww["t"], np.full(len(ww), 1.0))),
            (0.1, 0.8),
            color=color,
            label=f"{kind} ({len(ww)})",
        )
    ax[4].set_yticks([])
    ax[4].set_xlabel("session time, s")
    report = session["windows"]
    ax[4].set_title(
        f"E. Fit windows. Dropped: {report['windows_dropped']}; gate seconds "
        f"{ {k: round(v, 1) for k, v in report['gate_drop_seconds'].items()} }",
        loc="left",
    )
    ax[4].legend(loc="upper right", frameon=False, ncol=2)
    fig.savefig(out, dpi=120)
    plt.close(fig)


def floor(d: dict, out: Path) -> None:
    s, m, w, session = d["smoothed"], d["meas"], d["windows"], d["session"]
    opts = session["windows"]["options"]
    cx, cy = opts["box_center_xy"]
    half = float(opts["box_half_extent_m"])
    fig, ax = plt.subplots(figsize=(10, 8), constrained_layout=True)
    ok = m[m["rejected"] == 0]
    ax.plot(s["x"], s["y"], color=SMOOTH, lw=0.4, alpha=0.7, label="smoothed")
    for src, color in (("tag", TAG), ("yolo", YOLO)):
        mm = ok[ok["source"] == src]
        ax.scatter(mm["x"], mm["y"], s=1, color=color, label=f"{src} fix ({len(mm)})")
    rej = m[m["rejected"] == 1]
    ax.scatter(rej["x"], rej["y"], s=6, color=OTHER, marker="x", label=f"rejected ({len(rej)})")
    ax.add_patch(
        plt.Rectangle(
            (cx - half, cy - half),
            2 * half,
            2 * half,
            fill=False,
            ec=MUTED,
            ls="--",
            label="box the gates assume",
        )
    )
    ax.scatter(w["x0"], w["y0"], s=8, facecolor="none", ec=INK, lw=0.5, label="window starts")
    pts = ok[["x", "y"]]
    lo = np.minimum(pts.min().to_numpy(), [cx - half, cy - half]) - 0.1
    hi = np.maximum(pts.max().to_numpy(), [cx + half, cy + half]) + 0.1
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_aspect("equal")
    ax.set_xlabel("x, m")
    ax.set_ylabel("y, m")
    ax.set_title(
        "Floor: fixes, smoothed path, and the box the window gate assumes.\n"
        "Many fixes outside the box: set --box-center / --box-half-extent",
        loc="left",
    )
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), frameon=False, fontsize=7)
    fig.savefig(out, dpi=120)
    plt.close(fig)


def dropouts(d: dict, out: Path, min_gap_s: float, compare: dict | None) -> int:
    s, m, cmd = d["smoothed"], d["meas"], d["cmd"]
    tags = m[(m["source"] == "tag")]
    tt = np.unique(tags["t"].to_numpy())
    gaps = np.flatnonzero(np.diff(tt) > min_gap_s)
    spans = []
    for g in gaps:
        a, b = tt[g], tt[g + 1]
        k = (cmd["t"] > a - 0.2) & (cmd["t"] < b)
        if cmd.loc[k, "is_upside_down"].mean() > 0.2:
            continue
        spans.append((a, b))
    if not spans:
        return 0
    cols = 5
    rows = int(np.ceil(len(spans) / cols))
    fig, axes = plt.subplots(
        rows, cols, figsize=(3.3 * cols, 3.4 * rows + 0.6), constrained_layout=True, squeeze=False
    )
    yolo = m[(m["source"] == "yolo") & (m["rejected"] == 0)]
    for ax, (a, b) in zip(axes.flat, spans):

        def near(f: pd.DataFrame, a: float = a, b: float = b) -> pd.DataFrame:
            return f[(f["t"] > a - 0.5) & (f["t"] < b + 0.5)]

        if compare is not None:
            c = near(compare["smoothed"])
            ax.plot(c["x"], c["y"], color=OTHER, lw=1.1, label="compare: smoothed")
        sw, tw, yw = near(s), near(tags), near(yolo)
        ax.plot(sw["x"], sw["y"], color=SMOOTH, lw=1.7, label="smoothed")
        ax.scatter(yw["x"], yw["y"], s=6, color=YOLO, zorder=3, edgecolor="none", label="YOLO fix")
        ax.scatter(tw["x"], tw["y"], s=8, color=TAG, zorder=4, edgecolor="none", label="tag fix")
        pts = pd.concat([tw[["x", "y"]], yw[["x", "y"]], sw[["x", "y"]]])
        mx, my = (pts["x"].min() + pts["x"].max()) / 2, (pts["y"].min() + pts["y"].max()) / 2
        span = max(pts["x"].max() - pts["x"].min(), pts["y"].max() - pts["y"].min(), 0.15) / 2
        span += 0.05
        ax.set_xlim(mx - span, mx + span)
        ax.set_ylim(my - span, my + span)
        ax.set_aspect("equal")
        inside = yolo[(yolo["t"] > a) & (yolo["t"] < b)]
        ax.set_title(f"t = {a:.1f} s, gap {b - a:.2f} s, {len(inside)} YOLO fixes", loc="left")
    for ax in list(axes.flat)[len(spans) :]:
        ax.axis("off")
    h, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(h, labels, loc="upper center", ncol=4, frameon=False, bbox_to_anchor=(0.5, 1.02))
    fig.suptitle(
        "Tag dropouts, upright robot, 0.5 s either side (field frame, m). The smoothed "
        "path should run through the fixes, not loop or shoot off between them",
        y=1.05,
        fontsize=10,
    )
    fig.savefig(out, dpi=120, bbox_inches="tight")
    plt.close(fig)
    return len(spans)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("bundle", type=Path)
    parser.add_argument("--compare", type=Path, default=None, help="second bundle to overlay")
    parser.add_argument("--min-gap-s", type=float, default=0.3)
    parser.add_argument("--out", type=Path, default=None, help="default: <bundle>/plots")
    args = parser.parse_args()
    _style()
    out = args.out or args.bundle / "plots"
    out.mkdir(parents=True, exist_ok=True)
    d = load(args.bundle)
    compare = load(args.compare) if args.compare else None
    overview(d, out / "overview.png")
    floor(d, out / "floor.png")
    n = dropouts(d, out / "dropouts.png", args.min_gap_s, compare)
    print(f"wrote {out}/overview.png, floor.png, dropouts.png ({n} dropouts)")


if __name__ == "__main__":
    main()
