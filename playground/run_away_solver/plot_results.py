#!/usr/bin/env python3
"""Stage 3 of the run-away solver experiment: results.csv -> figures.

Writes figures to docs/experiments/control_improvement/assets/ for the solver
report. Reads only the per-tick results.csv from compare_solvers.py.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_ASSETS = REPO_ROOT / "docs" / "experiments" / "control_improvement" / "assets"
CANDIDATES = ["grid", "exact", "bnb"]
BUCKETS = ["0", "1", "2+"]


def opponent_bucket(n_opp: pd.Series) -> pd.Series:
    return pd.cut(n_opp, [-0.5, 0.5, 1.5, np.inf], labels=BUCKETS).astype(str)


def plot_radius_error(results: pd.DataFrame, out: Path) -> None:
    fig, axes = plt.subplots(1, len(BUCKETS), figsize=(13, 4), sharey=True)
    for ax, bucket in zip(axes, BUCKETS):
        for method in CANDIDATES:
            part = results[(results["method"] == method) & (results["bucket"] == bucket)]
            if part.empty:
                continue
            err_mm = np.sort((part["ref_radius"] - part["radius"]).to_numpy()) * 1e3
            ax.plot(err_mm, np.linspace(0, 1, len(err_mm)), label=method)
        ax.set_title(f"{bucket} opponents")
        ax.set_xlabel("radius error vs 1 mm brute force (mm)")
        ax.grid(True, alpha=0.3)
    axes[0].set_ylabel("CDF")
    axes[0].legend()
    fig.suptitle("Radius understatement per method")
    fig.tight_layout()
    fig.savefig(out / "run_away_solver_radius_error.png", dpi=120)
    plt.close(fig)


def plot_jitter(results: pd.DataFrame, out: Path) -> None:
    fig, ax = plt.subplots(figsize=(8, 4))
    width = 0.25
    for k, method in enumerate(CANDIDATES):
        means = []
        for bucket in BUCKETS:
            part = results[(results["method"] == method) & (results["bucket"] == bucket)]
            steps = []
            for _, rec in part.groupby("recording"):
                rec = rec.sort_values("timestamp_ns")
                steps.append(
                    np.hypot(rec["center_x"].diff(), rec["center_y"].diff()).dropna()
                )
            jitter = pd.concat(steps).abs() if steps else pd.Series(dtype=float)
            # Median is always zero (stale tracks repeat positions); the mean carries the
            # signal.
            means.append(jitter.mean() * 1e3 if len(jitter) else np.nan)
        ax.bar(np.arange(len(BUCKETS)) + k * width, means, width, label=method)
    ax.set_xticks(np.arange(len(BUCKETS)) + width)
    ax.set_xticklabels([f"{b} opponents" for b in BUCKETS])
    ax.set_ylabel("mean |center step| (mm)")
    ax.set_title("Frame-to-frame target jitter (drives navigation directly)")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / "run_away_solver_jitter.png", dpi=120)
    plt.close(fig)


def plot_timing(results: pd.DataFrame, out: Path) -> None:
    fig, ax = plt.subplots(figsize=(8, 4))
    data = []
    labels = []
    for method in CANDIDATES:
        part = results[results["method"] == method]
        data.append(part["elapsed_ns"].to_numpy() / 1e3)
        labels.append(method)
    ax.boxplot(data, tick_labels=labels, whis=(5, 95), showfliers=False)
    ax.set_yscale("log")
    ax.set_ylabel("per-tick solve time (us, repeats=1, x86 release)")
    ax.set_title("Solver wall time (x86; re-measure on Jetson before treating as final)")
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "run_away_solver_timing.png", dpi=120)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--results",
        type=Path,
        default=Path(__file__).resolve().parent / "out" / "results.csv",
    )
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_ASSETS)
    args = parser.parse_args()

    results = pd.read_csv(args.results)
    results["bucket"] = opponent_bucket(results["n_opp"])
    args.out_dir.mkdir(parents=True, exist_ok=True)
    plot_radius_error(results, args.out_dir)
    plot_jitter(results, args.out_dir)
    plot_timing(results, args.out_dir)
    print(f"figures written to {args.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
