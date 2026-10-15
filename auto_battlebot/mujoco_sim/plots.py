"""Figures that make a MuJoCo plant fit and its validation readable at a glance.

``fit_mujoco_plant.py`` calls these from validation, so every validated fit directory gets them:

- ``loss.png``: CMA-ES best loss per generation for every restart, stage by stage. Restarts
  that end at the same loss found the same basin; one far above the rest got stuck.
- ``params.png``: where each restart's best parameters sit inside their search range. A
  parameter the restarts agree on is identified; one scattered across its range is not, and one
  pinned to a bound is fighting the model.
- ``rollouts.png``: held-out windows top-down, the measured robot against the MuJoCo and grey-box
  rollouts from the same start state and the same logged motor commands.
- ``errors.png``: position and heading error at each scoring horizon on the held-out windows, as
  multiples of the measurement noise, MuJoCo against the grey-box.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from auto_battlebot.mujoco_sim.fit import PARAM_SPECS, WindowErrors
from auto_battlebot.mujoco_sim.rollout import pose_from_qpos
from auto_battlebot.mujoco_sim.session import WindowBatch

MEAS, MUJOCO, GREY, OTHER, YOLO = "#2a78d6", "#1baf7a", "#eb6834", "#6250d6", "#eda100"
INK, MUTED, SURF, GRID = "#0b0b0b", "#52514e", "#fcfcfb", "#e6e5e1"


def _plt() -> Any:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

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
    return plt


def plot_loss(doc: dict[str, Any], out: Path) -> None:
    plt = _plt()
    fig, ax = plt.subplots(figsize=(11, 4.5), constrained_layout=True)
    colors = {"brake": MUJOCO, "coast": OTHER}
    edges: list[tuple[float, str]] = []
    for k, run in enumerate(doc["runs"]):
        x0 = 0
        for stage in run["stages"]:
            hist = np.asarray(stage["history"], dtype=float)
            xs = x0 + np.arange(len(hist))
            label = f"{run['zero_mode']} seed {run['seed']}" if x0 == 0 else None
            ax.plot(
                xs,
                hist,
                color=colors.get(run["zero_mode"], MUTED),
                lw=1.2,
                alpha=0.5 + 0.5 * (k % 2),
                label=label,
            )
            if k == 0:
                edges.append((x0, stage["stage"]))
            x0 += len(hist)
    for x, name in edges:
        ax.axvline(x, color=MUTED, lw=0.6, ls="--")
        ax.text(x + 0.3, 1.0, name, transform=ax.get_xaxis_transform(), va="top", color=MUTED)
    ax.set_yscale("log")
    ax.set_xlabel("generation, stages in order")
    ax.set_ylabel("best loss (Huber, noise units)")
    best = min(run["loss"] for run in doc["runs"])
    ax.set_title(
        f"CMA-ES best loss per generation, every restart. Final best {best:.2f}. "
        "Curves still falling at a stage's end want more generations",
        loc="left",
    )
    ax.legend(loc="upper right", frameon=False, fontsize=7)
    fig.savefig(out, dpi=120)
    plt.close(fig)


def plot_params(doc: dict[str, Any], out: Path) -> None:
    plt = _plt()
    runs = doc["runs"]
    best_k = int(np.argmin([r["loss"] if np.isfinite(r["loss"]) else np.inf for r in runs]))
    names = [s.name for s in PARAM_SPECS]
    fig, ax = plt.subplots(figsize=(10, 0.32 * len(names) + 1.5), constrained_layout=True)
    for i, spec in enumerate(PARAM_SPECS):
        for k, run in enumerate(runs):
            v = run["best"].get(spec.name)
            if v is None:
                continue
            u = spec.to_unit(float(v))
            mark = (
                dict(s=60, color=INK, marker="*", zorder=4)
                if k == best_k
                else dict(s=18, color=MUJOCO if run["zero_mode"] == "brake" else OTHER, alpha=0.8)
            )
            ax.scatter([u], [i], **mark)
        v = runs[best_k]["best"].get(spec.name)
        ax.text(
            1.02,
            i,
            f"{float(v):.4g}",
            va="center",
            fontsize=7,
            color=INK,
            transform=ax.get_yaxis_transform(),
        )
    ax.set_yticks(range(len(names)), names)
    ax.set_xlim(-0.03, 1.03)
    ax.invert_yaxis()
    ax.set_xlabel("position in the search range (0 = lower bound, 1 = upper)")
    ax.set_title(
        "Best parameters of every restart (star: the best run, value at right).\n"
        "Clustered = identified; scattered = the data does not pin it; at 0 or 1 = "
        "pinned to a bound",
        loc="left",
    )
    fig.savefig(out, dpi=120)
    plt.close(fig)


def _pick_windows(batch: WindowBatch, err: WindowErrors, n: int) -> list[int]:
    """The windows nearest the 10th, 50th and 90th percentile of MuJoCo error, both kinds."""
    score = np.nanmax(err.position, axis=1)
    picks: list[int] = []
    for kind in ("flat", "nose_lift"):
        idx = np.flatnonzero((batch.kind == kind) & np.isfinite(score))
        if idx.size == 0:
            continue
        order = idx[np.argsort(score[idx])]
        want = max(1, n // 2)
        at = np.unique(np.linspace(0, order.size - 1, want).round().astype(int))
        picks += order[at].tolist()
    return picks[:n]


def plot_rollouts(
    batch: WindowBatch,
    mj_qpos: np.ndarray,
    mj_dt: float,
    gb_qpos: np.ndarray,
    gb_dt: float,
    mj_err: WindowErrors,
    out: Path,
    n: int = 12,
) -> None:
    plt = _plt()
    picks = _pick_windows(batch, mj_err, n)
    if not picks:
        return
    mj = pose_from_qpos(mj_qpos)
    gb = pose_from_qpos(gb_qpos)
    cols = 4
    rows = int(np.ceil(len(picks) / cols))
    fig, axes = plt.subplots(
        rows, cols, figsize=(3.4 * cols, 3.6 * rows + 0.6), constrained_layout=True, squeeze=False
    )
    horizon = batch.steps * batch.dt
    for ax, i in zip(axes.flat, picks):
        tm = np.arange(mj["x"].shape[1]) * mj_dt
        tg = np.arange(gb["x"].shape[1]) * gb_dt
        km, kg = tm <= horizon, tg <= horizon
        ax.plot(gb["x"][i][kg], gb["y"][i][kg], color=GREY, lw=1.2, label="grey-box")
        ax.plot(mj["x"][i][km], mj["y"][i][km], color=MUJOCO, lw=1.8, label="MuJoCo")
        # A fix noisier than the session floor came from the second source (YOLO).
        second = (
            np.asarray(batch.meas_sigma_xy[i]) > batch.noise_xy[i] * 1.01
            if batch.meas_sigma_xy
            else np.zeros(len(batch.meas_x[i]), dtype=bool)
        )
        ax.scatter(
            batch.meas_x[i][~second],
            batch.meas_y[i][~second],
            s=7,
            color=MEAS,
            zorder=3,
            label="measured, tag",
        )
        ax.scatter(
            batch.meas_x[i][second],
            batch.meas_y[i][second],
            s=7,
            color=YOLO,
            zorder=3,
            label="measured, YOLO",
        )
        x0, y0, yaw0 = batch.state.x[i], batch.state.y[i], batch.state.yaw[i]
        ax.annotate(
            "",
            xy=(x0 + 0.06 * np.cos(yaw0), y0 + 0.06 * np.sin(yaw0)),
            xytext=(x0, y0),
            arrowprops=dict(arrowstyle="->", color=INK, lw=1.0),
        )
        pts_x = np.r_[batch.meas_x[i], mj["x"][i][km], x0]
        pts_y = np.r_[batch.meas_y[i], mj["y"][i][km], y0]
        cx, cy = (pts_x.min() + pts_x.max()) / 2, (pts_y.min() + pts_y.max()) / 2
        span = max(np.ptp(pts_x), np.ptp(pts_y), 0.1) / 2 + 0.04
        ax.set_xlim(cx - span, cx + span)
        ax.set_ylim(cy - span, cy + span)
        ax.set_aspect("equal")
        u = batch.u_raw[i, batch.pad :]
        e = mj_err.position[i]
        ax.set_title(
            f"{batch.kind[i]} #{batch.window_id[i]}: cmd L {u[:, 0].mean():+.2f} "
            f"R {u[:, 1].mean():+.2f}\nMuJoCo err at "
            + ", ".join(
                f"{h:.1f}s {1000 * v:.0f}mm" for h, v in zip(batch.horizons, e) if np.isfinite(v)
            ),
            loc="left",
            fontsize=7,
        )
    for ax in list(axes.flat)[len(picks) :]:
        ax.axis("off")
    h, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(h, labels, loc="upper center", ncol=3, frameon=False, bbox_to_anchor=(0.5, 1.02))
    fig.suptitle(
        "Held-out windows, best, median and worst by MuJoCo error. Arrow: start pose. "
        "Both models start there and get the logged motor commands",
        y=1.05,
        fontsize=10,
    )
    fig.savefig(out, dpi=120, bbox_inches="tight")
    plt.close(fig)


def plot_errors(batch: WindowBatch, mj: WindowErrors, gb: WindowErrors, out: Path) -> None:
    plt = _plt()
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), constrained_layout=True)
    flat = batch.kind == "flat"
    lift = ~flat
    groups = [
        ("MuJoCo flat", mj, flat, MUJOCO),
        ("grey-box flat", gb, flat, GREY),
        ("MuJoCo nose-lift", mj, lift, OTHER),
    ]
    for ax, (field, sigma, unit) in zip(
        axes,
        (
            ("position", "position_sigma", "position / noise"),
            ("heading", "heading_sigma", "heading / noise"),
        ),
    ):
        width = 0.25
        for g, (name, err, mask, color) in enumerate(groups):
            data = []
            for j in range(len(batch.horizons)):
                v = np.abs(getattr(err, field)[mask, j]) / getattr(err, sigma)[mask, j]
                data.append(v[np.isfinite(v)])
            pos = np.arange(len(batch.horizons)) + (g - 1) * width
            parts = ax.boxplot(
                data,
                positions=pos,
                widths=width * 0.9,
                patch_artist=True,
                showfliers=False,
                manage_ticks=False,
            )
            for box in parts["boxes"]:
                box.set(facecolor=color, alpha=0.6, edgecolor=color)
            for med in parts["medians"]:
                med.set(color=INK)
            ax.plot([], [], color=color, lw=6, alpha=0.6, label=f"{name} (n={int(mask.sum())})")
        ax.set_xticks(range(len(batch.horizons)), [f"{h:.1f} s" for h in batch.horizons])
        ax.set_yscale("log")
        ax.axhline(1.0, color=MUTED, lw=0.8, ls="--")
        ax.set_ylabel(unit)
        ax.set_xlabel("horizon after the window start")
        ax.set_title(
            f"{field.capitalize()} error on held-out windows; 1 = the measurement noise", loc="left"
        )
        ax.legend(loc="upper left", frameon=False, fontsize=7)
    fig.savefig(out, dpi=120)
    plt.close(fig)
