#!/usr/bin/env python3
"""
Plot YOLO training results from a results.csv file.

Usage:
    python3 plot_results.py results.csv
    python3 plot_results.py results.csv --output training_plot.png
    python3 plot_results.py results.csv --show
"""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import pandas as pd
import numpy as np


# Column groups to plot, each as (display_title, [(col_suffix, legend_label), ...])
PANEL_GROUPS = [
    (
        "Train Losses",
        [
            ("train/box_loss",  "box"),
            ("train/cls_loss",  "cls"),
            ("train/dfl_loss",  "dfl"),
            ("train/pose_loss", "pose"),
            ("train/kobj_loss", "kobj"),
            ("train/rle_loss",  "rle"),
        ],
    ),
    (
        "Val Losses",
        [
            ("val/box_loss",  "box"),
            ("val/cls_loss",  "cls"),
            ("val/dfl_loss",  "dfl"),
            ("val/pose_loss", "pose"),
            ("val/kobj_loss", "kobj"),
            ("val/rle_loss",  "rle"),
        ],
    ),
    (
        "Detection Metrics (B)",
        [
            ("metrics/mAP50(B)",     "mAP50"),
            ("metrics/mAP50-95(B)",  "mAP50-95"),
            ("metrics/precision(B)", "precision"),
            ("metrics/recall(B)",    "recall"),
        ],
    ),
    (
        "Pose Metrics (P)",
        [
            ("metrics/mAP50(P)",     "mAP50"),
            ("metrics/mAP50-95(P)",  "mAP50-95"),
            ("metrics/precision(P)", "precision"),
            ("metrics/recall(P)",    "recall"),
        ],
    ),
    (
        "Learning Rates",
        [
            ("lr/pg0", "pg0"),
            ("lr/pg1", "pg1"),
            ("lr/pg2", "pg2"),
            ("lr/pg3", "pg3"),
            ("lr/pg4", "pg4"),
            ("lr/pg5", "pg5"),
            ("lr/pg6", "pg6"),
            ("lr/pg7", "pg7"),
        ],
    ),
]

# Distinct colors for lines within each panel
LINE_COLORS = [
    "#E74C3C", "#3498DB", "#2ECC71", "#F39C12",
    "#9B59B6", "#1ABC9C", "#E67E22", "#34495E",
]


def load_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    # Strip surrounding whitespace from column names (YOLO sometimes adds spaces)
    df.columns = [c.strip() for c in df.columns]
    return df


def is_nonzero(series: pd.Series) -> bool:
    """Return True if the series contains any meaningful (non-zero, non-NaN) values."""
    return bool((series.dropna() != 0).any())


def plot_panel(ax: plt.Axes, df: pd.DataFrame, title: str, series_specs: list):
    """Plot one panel onto the given axes."""
    plotted = 0
    for col, label in series_specs:
        if col not in df.columns:
            continue
        series = pd.to_numeric(df[col], errors="coerce")
        if not is_nonzero(series):
            continue
        color = LINE_COLORS[plotted % len(LINE_COLORS)]
        ax.plot(df["epoch"], series, label=label, color=color, linewidth=1.5)
        plotted += 1

    ax.set_title(title, fontsize=10, fontweight="bold", pad=4)
    ax.set_xlabel("Epoch", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.grid(True, alpha=0.3, linewidth=0.5)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    if plotted > 0:
        ax.legend(fontsize=7, loc="best", framealpha=0.7)
    else:
        ax.text(
            0.5, 0.5, "No data", transform=ax.transAxes,
            ha="center", va="center", fontsize=9, color="gray"
        )

    return plotted


def make_figure(df: pd.DataFrame, csv_path: Path) -> plt.Figure:
    n_panels = len(PANEL_GROUPS)
    n_cols = 2
    n_rows = (n_panels + n_cols - 1) // n_cols

    fig = plt.figure(figsize=(14, n_rows * 3.8))
    fig.suptitle(
        f"YOLO Training Results — {csv_path.name}\n"
        f"Epochs: {int(df['epoch'].max())}   "
        f"Best mAP50(B): {pd.to_numeric(df.get('metrics/mAP50(B)', pd.Series()), errors='coerce').max():.4f}   "
        f"Best mAP50(P): {pd.to_numeric(df.get('metrics/mAP50(P)', pd.Series()), errors='coerce').max():.4f}",
        fontsize=11,
        fontweight="bold",
        y=0.995,
    )

    gs = gridspec.GridSpec(n_rows, n_cols, figure=fig, hspace=0.55, wspace=0.35)

    for idx, (title, series_specs) in enumerate(PANEL_GROUPS):
        row, col = divmod(idx, n_cols)
        ax = fig.add_subplot(gs[row, col])
        plot_panel(ax, df, title, series_specs)

    # If odd number of panels, hide the last empty cell
    if n_panels % n_cols != 0:
        ax_empty = fig.add_subplot(gs[n_rows - 1, n_cols - 1])
        ax_empty.set_visible(False)

    return fig


def main():
    parser = argparse.ArgumentParser(description="Plot YOLO training results CSV")
    parser.add_argument("csv", type=Path, help="Path to results.csv")
    parser.add_argument(
        "--output", "-o", type=Path, default=None,
        help="Output image path (default: <csv_stem>_plot.png next to the CSV)"
    )
    parser.add_argument(
        "--show", action="store_true",
        help="Display the plot interactively instead of saving"
    )
    parser.add_argument(
        "--dpi", type=int, default=150,
        help="Output DPI (default: 150)"
    )
    args = parser.parse_args()

    if not args.csv.exists():
        print(f"Error: file not found: {args.csv}", file=sys.stderr)
        sys.exit(1)

    df = load_csv(args.csv)

    if "epoch" not in df.columns:
        print("Error: CSV does not contain an 'epoch' column", file=sys.stderr)
        sys.exit(1)

    fig = make_figure(df, args.csv)

    if args.show:
        plt.show()
    else:
        output = args.output or args.csv.parent / f"{args.csv.stem}_plot.png"
        fig.savefig(output, dpi=args.dpi, bbox_inches="tight")
        print(f"Saved plot -> {output}")

    plt.close(fig)


if __name__ == "__main__":
    main()
