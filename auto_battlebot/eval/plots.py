"""Summary plots written alongside the scorer's CSV output."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def plot_headline(summary: pd.DataFrame, output: Path) -> None:
    """Class-agnostic recall vs class-aware mAP50, per candidate: the taxonomy-split cost."""
    candidates = summary["candidate"].unique()
    fig, ax = plt.subplots(figsize=(9, 5))
    x = np.arange(len(candidates))
    for offset, (level, metric, label) in enumerate(
        [
            ("agnostic", "recall", "found the robot (agnostic recall)"),
            ("archetype", "map50", "named the archetype (mAP@.5)"),
            ("instance", "map50", "named the instance (mAP@.5)"),
        ]
    ):
        values = [
            summary[(summary["candidate"] == c) & (summary["level"] == level)][metric].iloc[0]
            for c in candidates
        ]
        ax.bar(x + (offset - 1) * 0.25, values, width=0.25, label=label)
    ax.set_xticks(x)
    ax.set_xticklabels(candidates)
    ax.set_ylim(0, 1.05)
    ax.set_ylabel("score")
    ax.set_title("Detection vs naming: cost of splitting the opponent category")
    ax.legend(loc="lower right")
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(output / "headline.png", dpi=120)
    plt.close(fig)


def plot_confusion(confusion: dict, candidate: str, level: str, output: Path) -> None:
    gt_names = sorted({k[0] for k in confusion})
    pred_names = sorted({k[1] for k in confusion})
    grid = np.zeros((len(gt_names), len(pred_names)))
    for (gt, pred), count in confusion.items():
        grid[gt_names.index(gt), pred_names.index(pred)] = count
    fig, ax = plt.subplots(figsize=(2 + len(pred_names), 2 + len(gt_names) * 0.6))
    im = ax.imshow(grid, cmap="Blues")
    ax.set_xticks(range(len(pred_names)), pred_names, rotation=45, ha="right")
    ax.set_yticks(range(len(gt_names)), gt_names)
    ax.set_xlabel("predicted")
    ax.set_ylabel("ground truth")
    ax.set_title(f"{candidate} / {level} (IoU-matched boxes)")
    for i in range(len(gt_names)):
        for j in range(len(pred_names)):
            if grid[i, j]:
                ax.text(j, i, str(int(grid[i, j])), ha="center", va="center", fontsize=8)
    fig.colorbar(im, ax=ax, shrink=0.8)
    fig.tight_layout()
    fig.savefig(output / f"confusion_{candidate}_{level}.png", dpi=120)
    plt.close(fig)
