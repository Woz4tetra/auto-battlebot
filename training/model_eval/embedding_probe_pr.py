#!/usr/bin/env python3
"""Precision/recall comparison: prototype-gated methods vs the deployed engine baseline.

Scores opponent detection on the validated eval frames (seed frames excluded), matching
predictions to GT opponent boxes greedily at IoU >= 0.5. Methods:

- baseline: engine proposals, opponent class, conf >= 0.6 (the deployed operating point)
- lowconf: engine proposals, opponent class, conf >= 0.05 (threshold lowering alone)
- simgate: any proposal with conf >= 0.05 and similarity >= the stage-3 operating point
- simgate_top1: simgate restricted to the single most similar proposal per frame
- union: baseline plus simgate_top1 (the proposed runtime rescue rule)

Also sweeps the engine confidence threshold and the similarity threshold into PR curves
per recording (figures/pr_curves.png). Embedder for similarity: clip at 0% padding, the
best stage-3 configuration.

Usage:
    cd training/model_eval && python embedding_probe_pr.py
"""

from __future__ import annotations

import matplotlib
import numpy as np
import pandas as pd

matplotlib.use("Agg")
import embedding_probe_common as common
import matplotlib.pyplot as plt
from embedding_probe_score import aligned_features, gallery_vectors

EMBEDDER = "clip"
PAD_KEY = "pad000"
MATCH_IOU = 0.5


def frame_tables(recording: str) -> tuple[pd.DataFrame, pd.DataFrame, float]:
    """Eval-frame proposals with similarity, GT opponent boxes, and the operating point."""
    candidates = common.load_candidates(recording)
    features = aligned_features(recording, EMBEDDER, PAD_KEY, len(candidates))
    gallery = gallery_vectors(candidates, features, "gallery")
    sims = (features @ gallery.T).max(axis=1)
    candidates = candidates.assign(sim=sims[candidates.index.to_numpy()])
    evals = candidates[candidates.role == "eval"]
    proposals = evals[evals.source == "proposal"].copy()
    gt_opponents = evals[(evals.source == "gt") & (evals.label == common.OPPONENT)].copy()
    metrics = pd.read_csv(common.OUTPUT_ROOT / "metrics.csv")
    threshold = float(
        metrics[
            (metrics.embedder == EMBEDDER)
            & (metrics.pad == PAD_KEY)
            & (metrics.variant == "gallery")
        ].threshold.iloc[0]
    )
    return proposals, gt_opponents, threshold


def score_predictions(
    predictions: pd.DataFrame, gt_opponents: pd.DataFrame, rank_column: str
) -> tuple[int, int, int]:
    """Greedy per-frame matching at IoU >= 0.5. Returns (tp, fp, n_gt)."""
    tp = fp = 0
    n_gt = len(gt_opponents)
    gt_by_stamp = dict(tuple(gt_opponents.groupby("stamp_ns")))
    for stamp, group in predictions.groupby("stamp_ns"):
        gt = gt_by_stamp.get(stamp)
        gt_boxes = gt[["x1", "y1", "x2", "y2"]].to_numpy() if gt is not None else np.zeros((0, 4))
        matched = np.zeros(len(gt_boxes), dtype=bool)
        pred_boxes = group[["x1", "y1", "x2", "y2"]].to_numpy()
        ious = common.iou_matrix(pred_boxes, gt_boxes)
        for row_position in np.argsort(-group[rank_column].to_numpy()):
            if len(gt_boxes) == 0:
                fp += 1
                continue
            available = np.where(~matched, ious[row_position], -1.0)
            best = int(np.argmax(available))
            if available[best] >= MATCH_IOU:
                matched[best] = True
                tp += 1
            else:
                fp += 1
    return tp, fp, n_gt


def method_predictions(
    proposals: pd.DataFrame, method: str, threshold: float
) -> tuple[pd.DataFrame, str]:
    """Prediction set and ranking column for one method."""
    opponent_class = proposals.label == common.OPPONENT
    if method == "baseline":
        return proposals[opponent_class & (proposals.conf >= common.DEPLOYED_CONF)], "conf"
    if method == "lowconf":
        return proposals[opponent_class], "conf"
    if method == "simgate":
        return proposals[proposals.sim >= threshold], "sim"
    if method == "simgate_top1":
        gated = proposals[proposals.sim >= threshold]
        top = gated.loc[gated.groupby("stamp_ns").sim.idxmax()]
        return top, "sim"
    if method == "union":
        base = proposals[opponent_class & (proposals.conf >= common.DEPLOYED_CONF)]
        gated = proposals[proposals.sim >= threshold]
        top = gated.loc[gated.groupby("stamp_ns").sim.idxmax()]
        return pd.concat([base, top]).drop_duplicates(), "conf"
    raise ValueError(method)


METHODS = ("baseline", "lowconf", "simgate", "simgate_top1", "union")


def sweep_curve(
    proposals: pd.DataFrame,
    gt_opponents: pd.DataFrame,
    column: str,
    subset: pd.DataFrame,
) -> tuple[list[float], list[float]]:
    precisions, recalls = [], []
    for threshold in np.quantile(subset[column], np.linspace(0.0, 0.999, 40)):
        tp, fp, n_gt = score_predictions(subset[subset[column] >= threshold], gt_opponents, column)
        if tp + fp == 0 or n_gt == 0:
            continue
        precisions.append(tp / (tp + fp))
        recalls.append(tp / n_gt)
    return recalls, precisions


def main() -> None:
    rows = []
    curves: dict[str, dict] = {}
    pooled: dict[str, np.ndarray] = {}
    for recording in sorted(common.SHORT_NAMES):
        short = common.short_name(recording)
        proposals, gt_opponents, threshold = frame_tables(recording)
        for method in METHODS:
            predictions, rank_column = method_predictions(proposals, method, threshold)
            tp, fp, n_gt = score_predictions(predictions, gt_opponents, rank_column)
            rows.append(
                {
                    "recording": short,
                    "method": method,
                    "tp": tp,
                    "fp": fp,
                    "n_gt": n_gt,
                    "recall": tp / n_gt if n_gt else float("nan"),
                    "precision": tp / (tp + fp) if tp + fp else float("nan"),
                }
            )
        opponent_class = proposals[proposals.label == common.OPPONENT]
        curves[short] = {
            "engine": sweep_curve(proposals, gt_opponents, "conf", opponent_class),
            "simgate": sweep_curve(proposals, gt_opponents, "sim", proposals),
        }
        pooled[short] = (proposals, gt_opponents, threshold)

    table = pd.DataFrame(rows)
    table.to_csv(common.OUTPUT_ROOT / "pr_comparison.csv", index=False)

    figures_dir = common.OUTPUT_ROOT / "figures"
    figures_dir.mkdir(exist_ok=True)
    shorts = [common.short_name(recording) for recording in sorted(common.SHORT_NAMES)]
    fig, axes = plt.subplots(2, 4, figsize=(16, 8))
    for axis, short in zip(axes.flat, shorts):
        engine_r, engine_p = curves[short]["engine"]
        gate_r, gate_p = curves[short]["simgate"]
        axis.plot(engine_r, engine_p, label="engine conf sweep", color="tab:blue")
        axis.plot(gate_r, gate_p, label="sim sweep (conf>=0.05)", color="tab:orange")
        base = table[(table.recording == short) & (table.method == "baseline")].iloc[0]
        gate = table[(table.recording == short) & (table.method == "simgate_top1")].iloc[0]
        axis.plot(base.recall, base.precision, "o", color="tab:blue", label="deployed (0.6)")
        axis.plot(gate.recall, gate.precision, "s", color="tab:orange", label="simgate top-1")
        axis.set_title(short)
        axis.set_xlabel("recall")
        axis.set_ylabel("precision")
        axis.set_xlim(0, 1)
        axis.set_ylim(0, 1.02)
    axes.flat[0].legend(fontsize=7)
    fig.suptitle(f"Opponent PR on eval frames: engine vs similarity gate ({EMBEDDER} {PAD_KEY})")
    fig.tight_layout()
    fig.savefig(figures_dir / "pr_curves.png", dpi=110)
    plt.close(fig)

    print(f"\nOpponent P/R on eval frames, IoU>=0.5 ({EMBEDDER} {PAD_KEY} similarity)")
    header = f"{'rec':7s}" + "".join(f" {method:>14s}" for method in METHODS)
    print(header + "   (recall/precision)")
    for short in shorts:
        cells = []
        for method in METHODS:
            row = table[(table.recording == short) & (table.method == method)].iloc[0]
            cells.append(f" {row.recall:5.1%}/{row.precision:5.1%}".rjust(15))
        print(f"{short:7s}" + "".join(cells))
    core = table[table.recording.isin(common.CORE_RECORDINGS)]
    print("\ncore totals:")
    for method in METHODS:
        sub = core[core.method == method]
        tp, fp, n_gt = sub.tp.sum(), sub.fp.sum(), sub.n_gt.sum()
        print(
            f"  {method:13s} recall {tp / n_gt:6.1%}  precision {tp / (tp + fp):6.1%}"
            f"  (tp={tp} fp={fp} gt={n_gt})"
        )


if __name__ == "__main__":
    main()
