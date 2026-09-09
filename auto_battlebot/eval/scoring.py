"""Run one candidate engine over a GT dataset and reduce it to summary rows.

This is the seam the CLI drives: loading, inference, metrics, and plots each live in
their own module, and the paired bootstrap consumes the stats returned here. The command
line is `training/model_eval/score.py`.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from auto_battlebot.eval.dataset import LEVELS, Taxonomy
from auto_battlebot.eval.detectors import build_detector, infer_frames
from auto_battlebot.eval.metrics import (
    compute_map,
    keypoint_metrics_from_stats,
    keypoint_per_frame,
    pr_from_counts,
    pr_per_frame,
)
from auto_battlebot.eval.plots import plot_confusion


def score_candidate(
    name: str,
    engine_path: Path,
    gt_frames: dict,
    images: dict[int, Path],
    class_labels: list[str],
    taxonomy: Taxonomy,
    args: argparse.Namespace,
) -> tuple[list[dict], dict]:
    """Return (summary rows, per-frame stats). Stats feed the paired bootstrap."""
    detector = build_detector(name, engine_path, class_labels, images, args)
    print(f"  {detector.describe()}")
    frames = infer_frames(gt_frames, images, detector, class_labels, taxonomy)
    kp_stats = keypoint_per_frame(frames, args.iou)
    keypoint_metrics = keypoint_metrics_from_stats(kp_stats)
    pr_counts: dict[str, dict[str, np.ndarray]] = {}
    rows = []
    for level in LEVELS:
        counts, confusion = pr_per_frame(frames, taxonomy, level, args.iou)
        pr_counts[level] = counts
        row = {
            "candidate": name,
            "level": level,
            **pr_from_counts(counts),
            **compute_map(frames, taxonomy, level),
            **keypoint_metrics,
        }
        rows.append(row)
        plot_confusion(confusion, name, level, args.output)
    stats = {"kp": kp_stats, "pr": pr_counts, "n_frames": len(frames)}
    return rows, stats
