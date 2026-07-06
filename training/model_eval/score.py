#!/usr/bin/env python3
"""Score candidate detector runs against a corrected YOLO ground-truth dataset.

Compares each candidate MCAP's detections (from a config/eval_candidate.toml run of the
same SVO as the GT labeling run) against the corrected labels, at three label levels:

    agnostic   every label collapses to "robot": pure localization (did it find the robot)
    archetype  labels map through taxonomy.yaml: NHRL archetype naming
    instance   labels as-is: per-robot-instance naming

The gap between agnostic recall and class-aware mAP is the cost of splitting the
OPPONENT category. Outputs a summary table (stdout + summary.csv) and plots.

--topic blob (default) scores /blob_detections; --topic keypoint scores
/keypoint_detections and, when the GT dataset has pose rows, adds keypoint metrics
over IoU-matched boxes: mean pixel error, PCK@0.1 of the GT box's longer side, and
heading error (angle of the front->back keypoint vector, mean degrees and accuracy
within 10 degrees).

With two or more candidates, a paired bootstrap reports whether each candidate differs
significantly from the baseline (the first candidate, or --baseline NAME). Because every
candidate is scored on the same GT frames, resampling frames paired cancels the
"this frame is just hard" variance, so a few hundred frames can resolve real differences.
mAP is dataset-level and left as a point estimate; the detection questions are covered by
the recall / precision / localization tests, which are frame-decomposable.

Usage:
    python training/model_eval/score.py data/eval/<svo_name> \
        --candidate generic=data/recordings/<run_a>.mcap \
        --candidate per_robot=data/recordings/<run_b>.mcap \
        [--topic blob|keypoint] [--taxonomy training/model_eval/taxonomy.yaml] \
        [--iou 0.5] [--baseline NAME] [--bootstrap 1000] [--seed 0] [--alpha 0.05] \
        [--output <dir>]
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch
import yaml
from torchmetrics.detection import MeanAveragePrecision

from auto_battlebot.mcap_io import (
    BLOB_DETECTIONS_TOPIC,
    KEYPOINT_DETECTIONS_TOPIC,
    Detections,
    match_stamps,
    read_detections,
)

LEVELS = ("agnostic", "archetype", "instance")
AGNOSTIC_LABEL = "robot"
STAMP_TOLERANCE_NS = 1_000_000
TOPICS = {"blob": BLOB_DETECTIONS_TOPIC, "keypoint": KEYPOINT_DETECTIONS_TOPIC}
PCK_FRACTION = 0.1

# Heading is the front->back keypoint vector. Assumes keypoint order [front, back]
# (dataset kpt_shape [2, 3], flip_idx [0, 1]).
FRONT_IDX = 0
BACK_IDX = 1
HEADING_THRESHOLD_DEG = 10.0
HEADING_MIN_VECTOR_PX = 1e-6  # skip when front/back coincide; angle is undefined
HEADING_ACC_KEY = f"kp_heading_acc@{HEADING_THRESHOLD_DEG:g}deg"
PCK_KEY = f"kp_pck@{PCK_FRACTION:g}"

# Metric polarity for the significance verdict. Anything not listed is lower-is-better
# (kp_err_px, kp_heading_err_deg, wrong_class_rate).
HIGHER_IS_BETTER = {
    "recall",
    "precision",
    "f1",
    "localization_recall",
    PCK_KEY,
    HEADING_ACC_KEY,
}

# Detection metrics tested per taxonomy level; keypoint metrics are level-independent.
DETECTION_METRICS = ("recall", "precision", "f1", "localization_recall")
KEYPOINT_METRICS = ("kp_err_px", PCK_KEY, "kp_heading_err_deg", HEADING_ACC_KEY)


@dataclass
class Frame:
    """One aligned frame: GT boxes and candidate predictions, as (N, 4) xyxy arrays.

    gt_keypoints / pred_keypoints hold one (K, 3) array per box: x, y in pixels plus
    visibility (GT) or confidence (prediction). Empty arrays for box-only rows."""

    gt_boxes: np.ndarray
    gt_labels: list[str]
    gt_keypoints: list[np.ndarray]
    pred_boxes: np.ndarray
    pred_labels: list[str]
    pred_scores: np.ndarray
    pred_keypoints: list[np.ndarray]


class Taxonomy:
    """Label mapping for each comparison level."""

    def __init__(self, path: Path | None) -> None:
        data = yaml.safe_load(path.read_text()) if path and path.exists() else {}
        data = data or {}
        self.archetypes: dict[str, str] = dict(data.get("archetypes", {}))
        self.exclude: set[str] = set(data.get("exclude", []))

    def map_label(self, label: str, level: str) -> str:
        if level == "agnostic":
            return AGNOSTIC_LABEL
        if level == "archetype":
            return self.archetypes.get(label, label)
        return label


GtFrame = tuple[np.ndarray, list[str], list[np.ndarray]]


def load_gt(dataset: Path) -> tuple[dict[int, GtFrame], list[str]]:
    """Read corrected YOLO labels; returns {stamp_ns: (boxes, labels, keypoints)} and names."""
    data = yaml.safe_load((dataset / "data.yaml").read_text())
    names: list[str] = list(data["names"])
    frames: dict[int, GtFrame] = {}
    for label_path in sorted((dataset / "labels").glob("*.txt")):
        image_path = _find_image(dataset / "images", label_path.stem)
        if image_path is None:
            continue
        width, height = _image_size(image_path)
        frames[int(label_path.stem)] = _parse_rows(label_path, names, width, height)
    if not frames:
        raise SystemExit(f"No labels found under {dataset}/labels")
    return frames, names


def _find_image(images_dir: Path, stem: str) -> Path | None:
    for ext in (".png", ".jpg", ".jpeg", ".bmp", ".webp"):
        candidate = images_dir / f"{stem}{ext}"
        if candidate.exists():
            return candidate
    return None


def _image_size(image_path: Path) -> tuple[int, int]:
    from PIL import Image

    with Image.open(image_path) as img:
        return img.size


def _parse_rows(label_path: Path, names: list[str], width: int, height: int) -> GtFrame:
    """Parse detect (5 values) and pose (5 + 3k values) rows into pixel coordinates."""
    boxes = []
    labels = []
    keypoints = []
    for line in label_path.read_text().splitlines():
        parts = line.split()
        if len(parts) < 5 or (len(parts) - 5) % 3 != 0:
            continue
        class_id = int(float(parts[0]))
        values = [float(v) for v in parts[1:]]
        cx, cy, w, h = values[:4]
        boxes.append(
            [
                (cx - w / 2) * width,
                (cy - h / 2) * height,
                (cx + w / 2) * width,
                (cy + h / 2) * height,
            ]
        )
        labels.append(names[class_id] if class_id < len(names) else f"class_{class_id}")
        kps = [
            [values[i] * width, values[i + 1] * height, values[i + 2]]
            for i in range(4, len(values), 3)
        ]
        keypoints.append(np.asarray(kps, dtype=np.float64).reshape(-1, 3))
    return np.asarray(boxes, dtype=np.float64).reshape(-1, 4), labels, keypoints


def align_frames(
    gt_frames: dict[int, GtFrame],
    candidate: list[Detections],
    taxonomy: Taxonomy,
    min_confidence: float,
) -> list[Frame]:
    """Pair each GT frame with the candidate's detections for the same SVO frame."""
    by_stamp = {d.stamp_ns: d for d in candidate}
    matches = match_stamps(sorted(gt_frames), sorted(by_stamp), STAMP_TOLERANCE_NS)
    missing = len(gt_frames) - len(matches)
    if missing:
        print(f"Warning: {missing}/{len(gt_frames)} GT frames have no candidate detections")

    frames = []
    for gt_stamp, (gt_boxes, gt_labels, gt_keypoints) in gt_frames.items():
        dets = by_stamp.get(matches.get(gt_stamp, -1))
        keep = [d for d in (dets.detections if dets else []) if d.confidence >= min_confidence]
        keep = [d for d in keep if d.label not in taxonomy.exclude]
        gt_keep = [i for i, lbl in enumerate(gt_labels) if lbl not in taxonomy.exclude]
        frames.append(
            Frame(
                gt_boxes=gt_boxes[gt_keep],
                gt_labels=[gt_labels[i] for i in gt_keep],
                gt_keypoints=[gt_keypoints[i] for i in gt_keep],
                pred_boxes=np.asarray(
                    [[d.x1, d.y1, d.x2, d.y2] for d in keep], dtype=np.float64
                ).reshape(-1, 4),
                pred_labels=[d.label for d in keep],
                pred_scores=np.asarray([d.confidence for d in keep], dtype=np.float64),
                pred_keypoints=[
                    np.asarray(
                        [[kp.x, kp.y, kp.confidence] for kp in d.keypoints], dtype=np.float64
                    ).reshape(-1, 3)
                    for d in keep
                ],
            )
        )
    return frames


def iou_matrix(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Pairwise IoU between (N, 4) and (M, 4) xyxy boxes."""
    if len(a) == 0 or len(b) == 0:
        return np.zeros((len(a), len(b)))
    x1 = np.maximum(a[:, None, 0], b[None, :, 0])
    y1 = np.maximum(a[:, None, 1], b[None, :, 1])
    x2 = np.minimum(a[:, None, 2], b[None, :, 2])
    y2 = np.minimum(a[:, None, 3], b[None, :, 3])
    inter = np.clip(x2 - x1, 0, None) * np.clip(y2 - y1, 0, None)
    area_a = (a[:, 2] - a[:, 0]) * (a[:, 3] - a[:, 1])
    area_b = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
    union = area_a[:, None] + area_b[None, :] - inter
    return np.where(union > 0, inter / union, 0.0)


def compute_map(frames: list[Frame], taxonomy: Taxonomy, level: str) -> dict[str, float]:
    """mAP@.5 / mAP@[.5:.95] / per-class AP@.5 via torchmetrics."""
    labels_seen = sorted(
        {taxonomy.map_label(lbl, level) for f in frames for lbl in f.gt_labels + f.pred_labels}
    )
    index = {name: i for i, name in enumerate(labels_seen)}
    metric = MeanAveragePrecision(iou_type="bbox", class_metrics=True)
    for f in frames:
        metric.update(
            [
                {
                    "boxes": torch.as_tensor(f.pred_boxes, dtype=torch.float32),
                    "scores": torch.as_tensor(f.pred_scores, dtype=torch.float32),
                    "labels": torch.as_tensor(
                        [index[taxonomy.map_label(lbl, level)] for lbl in f.pred_labels]
                    ),
                }
            ],
            [
                {
                    "boxes": torch.as_tensor(f.gt_boxes, dtype=torch.float32),
                    "labels": torch.as_tensor(
                        [index[taxonomy.map_label(lbl, level)] for lbl in f.gt_labels]
                    ),
                }
            ],
        )
    result = metric.compute()
    per_class = {}
    classes = [int(c) for c in result.get("classes", torch.tensor([])).reshape(-1)]
    aps = result.get("map_per_class", torch.tensor([])).reshape(-1)
    for class_idx, ap in zip(classes, aps):
        per_class[f"ap50_95/{labels_seen[class_idx]}"] = float(ap)
    return {
        "map50": float(result["map_50"]),
        "map50_95": float(result["map"]),
        **per_class,
    }


def match_indices(frame: Frame, iou_threshold: float) -> list[tuple[int | None, int | None]]:
    """Greedy class-blind IoU matching by descending confidence.

    Returns (gt_index, pred_index) pairs; unmatched GT as (g, None), unmatched
    predictions as (None, p)."""
    ious = iou_matrix(frame.gt_boxes, frame.pred_boxes)
    pairs: list[tuple[int | None, int | None]] = []
    used_gt: set[int] = set()
    used_pred: set[int] = set()
    order = np.argsort(-frame.pred_scores)
    for p in order:
        candidates = [(ious[g, p], g) for g in range(len(frame.gt_labels)) if g not in used_gt]
        best = max(candidates, default=(0.0, -1))
        if best[0] >= iou_threshold:
            used_gt.add(best[1])
            used_pred.add(int(p))
            pairs.append((best[1], int(p)))
    pairs.extend((g, None) for g in range(len(frame.gt_labels)) if g not in used_gt)
    pairs.extend((None, int(p)) for p in order if int(p) not in used_pred)
    return pairs


def match_frame(frame: Frame, taxonomy: Taxonomy, level: str, iou_threshold: float) -> list:
    """Index matches mapped to (gt_label|None, pred_label|None) pairs at a label level."""
    pairs = []
    for g, p in match_indices(frame, iou_threshold):
        gt_label = taxonomy.map_label(frame.gt_labels[g], level) if g is not None else None
        pred_label = taxonomy.map_label(frame.pred_labels[p], level) if p is not None else None
        pairs.append((gt_label, pred_label))
    return pairs


def pr_per_frame(
    frames: list[Frame], taxonomy: Taxonomy, level: str, iou_threshold: float
) -> tuple[dict[str, np.ndarray], dict[tuple[str, str], int]]:
    """Per-frame TP/FP/FN/wrong-class count arrays (length = #frames) plus confusion.

    Keeping the counts per frame lets the bootstrap resample frames and re-sum, rather
    than recomputing matches on every resample."""
    n = len(frames)
    tp = np.zeros(n)
    fp = np.zeros(n)
    fn = np.zeros(n)
    wrong = np.zeros(n)
    confusion: dict[tuple[str, str], int] = {}
    for fi, frame in enumerate(frames):
        for gt_label, pred_label in match_frame(frame, taxonomy, level, iou_threshold):
            key = (gt_label or "background", pred_label or "missed")
            confusion[key] = confusion.get(key, 0) + 1
            if gt_label is None:
                fp[fi] += 1
            elif pred_label is None:
                fn[fi] += 1
            elif gt_label == pred_label:
                tp[fi] += 1
            else:
                wrong[fi] += 1
    return {"tp": tp, "fp": fp, "fn": fn, "wrong": wrong}, confusion


def pr_from_counts(counts: dict[str, np.ndarray]) -> dict[str, float]:
    """Aggregate precision/recall/F1 from per-frame count arrays."""
    tp = float(counts["tp"].sum())
    fp = float(counts["fp"].sum())
    fn = float(counts["fn"].sum())
    wrong_class = float(counts["wrong"].sum())
    localized = tp + wrong_class  # right box regardless of name
    total_gt = tp + wrong_class + fn
    total_pred = tp + wrong_class + fp
    precision = tp / total_pred if total_pred else 0.0
    recall = tp / total_gt if total_gt else 0.0
    f1 = 2 * precision * recall / (precision + recall) if precision + recall else 0.0
    return {
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "localization_recall": localized / total_gt if total_gt else 0.0,
        "wrong_class_rate": wrong_class / localized if localized else 0.0,
    }


def _heading_error_deg(gt_kps: np.ndarray, pred_kps: np.ndarray) -> float | None:
    """Absolute angle (degrees) between GT and predicted front->back vectors.

    Returns None when the pair can't define a heading: fewer than two keypoints, front
    or back not visible in GT, or either vector shorter than HEADING_MIN_VECTOR_PX."""
    if len(gt_kps) <= max(FRONT_IDX, BACK_IDX):
        return None
    if gt_kps[FRONT_IDX, 2] <= 0 or gt_kps[BACK_IDX, 2] <= 0:
        return None
    gt_vec = gt_kps[BACK_IDX, :2] - gt_kps[FRONT_IDX, :2]
    pred_vec = pred_kps[BACK_IDX, :2] - pred_kps[FRONT_IDX, :2]
    if np.hypot(*gt_vec) < HEADING_MIN_VECTOR_PX or np.hypot(*pred_vec) < HEADING_MIN_VECTOR_PX:
        return None
    dot = float(gt_vec @ pred_vec)
    cross = float(gt_vec[0] * pred_vec[1] - gt_vec[1] * pred_vec[0])
    return abs(float(np.degrees(np.arctan2(cross, dot))))


def keypoint_per_frame(frames: list[Frame], iou_threshold: float) -> dict[str, np.ndarray]:
    """Per-frame keypoint sufficient statistics over IoU-matched box pairs.

    Returns count/sum arrays (length = #frames) so the bootstrap can resample frames and
    re-sum. Pixel error and PCK are per visible keypoint; heading is per matched box.
    Matching is class-blind, so these are label-level independent."""
    n = len(frames)
    stats = {
        key: np.zeros(n)
        for key in ("err_sum", "err_cnt", "pck_cnt", "head_err_sum", "head_cnt", "head_correct")
    }
    for fi, frame in enumerate(frames):
        for g, p in match_indices(frame, iou_threshold):
            if g is None or p is None:
                continue
            gt_kps = frame.gt_keypoints[g]
            pred_kps = frame.pred_keypoints[p]
            if len(gt_kps) == 0 or len(gt_kps) != len(pred_kps):
                continue
            box = frame.gt_boxes[g]
            norm = max(box[2] - box[0], box[3] - box[1])
            for k in range(len(gt_kps)):
                if gt_kps[k, 2] <= 0:  # visibility 0 = not labeled
                    continue
                err = float(np.hypot(*(gt_kps[k, :2] - pred_kps[k, :2])))
                stats["err_sum"][fi] += err
                stats["err_cnt"][fi] += 1
                if err <= PCK_FRACTION * norm:
                    stats["pck_cnt"][fi] += 1
            heading = _heading_error_deg(gt_kps, pred_kps)
            if heading is not None:
                stats["head_err_sum"][fi] += heading
                stats["head_cnt"][fi] += 1
                if heading <= HEADING_THRESHOLD_DEG:
                    stats["head_correct"][fi] += 1
    return stats


def keypoint_metrics_from_stats(stats: dict[str, np.ndarray]) -> dict[str, float]:
    """Aggregate keypoint metrics from per-frame stats. Empty when no keypoints matched."""
    err_cnt = float(stats["err_cnt"].sum())
    if err_cnt == 0:
        return {}
    metrics = {
        "kp_err_px": float(stats["err_sum"].sum()) / err_cnt,
        PCK_KEY: float(stats["pck_cnt"].sum()) / err_cnt,
    }
    head_cnt = float(stats["head_cnt"].sum())
    if head_cnt > 0:
        metrics["kp_heading_err_deg"] = float(stats["head_err_sum"].sum()) / head_cnt
        metrics[HEADING_ACC_KEY] = float(stats["head_correct"].sum()) / head_cnt
    return metrics


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
            summary.query("candidate == @c and level == @level")[metric].iloc[0] for c in candidates
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


def _resample_ratio(num: np.ndarray, den: np.ndarray, idx: np.ndarray) -> np.ndarray:
    """Ratio sum(num)/sum(den) over each resampled frame set; nan where den == 0.

    idx is (B, n_frames); num/den are per-frame arrays. Fully vectorized."""
    n = num[idx].sum(axis=1)
    d = den[idx].sum(axis=1)
    return np.where(d > 0, n / np.where(d > 0, d, 1.0), np.nan)


def _metric_samples(stats: dict, metric: str, level: str, idx: np.ndarray) -> np.ndarray:
    """Bootstrap distribution (length B) of `metric`, recomputed on each resample in idx.

    Pass a (1, n_frames) arange for the full-sample point estimate."""
    if metric in DETECTION_METRICS:
        c = stats["pr"][level]
        tp, fp, fn, wr = c["tp"], c["fp"], c["fn"], c["wrong"]
        if metric == "recall":
            return _resample_ratio(tp, tp + wr + fn, idx)
        if metric == "precision":
            return _resample_ratio(tp, tp + wr + fp, idx)
        if metric == "localization_recall":
            return _resample_ratio(tp + wr, tp + wr + fn, idx)
        # f1 from precision and recall on the same resample
        p = _resample_ratio(tp, tp + wr + fp, idx)
        r = _resample_ratio(tp, tp + wr + fn, idx)
        s = p + r
        return np.where(s > 0, 2 * p * r / np.where(s > 0, s, 1.0), 0.0)
    k = stats["kp"]
    if metric == "kp_err_px":
        return _resample_ratio(k["err_sum"], k["err_cnt"], idx)
    if metric == PCK_KEY:
        return _resample_ratio(k["pck_cnt"], k["err_cnt"], idx)
    if metric == "kp_heading_err_deg":
        return _resample_ratio(k["head_err_sum"], k["head_cnt"], idx)
    if metric == HEADING_ACC_KEY:
        return _resample_ratio(k["head_correct"], k["head_cnt"], idx)
    raise KeyError(metric)


def run_significance(
    stats_by_candidate: dict[str, dict], baseline: str, topic: str, args: argparse.Namespace
) -> pd.DataFrame:
    """Paired-bootstrap each candidate against the baseline on the same resampled frames.

    Reports delta (candidate - baseline) with an (1 - alpha) CI and a verdict: "better"/
    "worse" when the CI excludes 0 (direction from metric polarity), "ns" otherwise."""
    n = stats_by_candidate[baseline]["n_frames"]
    rng = np.random.default_rng(args.seed)
    idx = rng.integers(0, n, size=(args.bootstrap, n))
    full = np.arange(n)[None, :]
    lo_pct, hi_pct = 100 * args.alpha / 2, 100 * (1 - args.alpha / 2)

    plan = [(m, level) for level in LEVELS for m in DETECTION_METRICS]
    if topic == "keypoint":
        available = keypoint_metrics_from_stats(stats_by_candidate[baseline]["kp"])
        plan += [(m, "-") for m in KEYPOINT_METRICS if m in available]

    rows = []
    base = stats_by_candidate[baseline]
    for name, cand in stats_by_candidate.items():
        if name == baseline:
            continue
        for metric, level in plan:
            base_pt = float(_metric_samples(base, metric, level, full)[0])
            cand_pt = float(_metric_samples(cand, metric, level, full)[0])
            diff = _metric_samples(cand, metric, level, idx) - _metric_samples(
                base, metric, level, idx
            )
            if np.all(np.isnan(diff)):
                lo = hi = float("nan")
                verdict = "n/a"
            else:
                lo, hi = (float(v) for v in np.nanpercentile(diff, [lo_pct, hi_pct]))
                if lo > 0 or hi < 0:
                    improved = (cand_pt - base_pt > 0) == (metric in HIGHER_IS_BETTER)
                    verdict = "better" if improved else "worse"
                else:
                    verdict = "ns"
            rows.append(
                {
                    "candidate": name,
                    "baseline": baseline,
                    "level": level,
                    "metric": metric,
                    "baseline_val": base_pt,
                    "candidate_val": cand_pt,
                    "delta": cand_pt - base_pt,
                    "ci_low": lo,
                    "ci_high": hi,
                    "verdict": verdict,
                }
            )
    return pd.DataFrame(rows)


def parse_candidates(entries: list[str]) -> dict[str, Path]:
    candidates = {}
    for entry in entries:
        if "=" in entry:
            name, _, path = entry.partition("=")
        else:
            name, path = Path(entry).stem, entry
        candidates[name] = Path(path)
    return candidates


def score_candidate(
    name: str,
    mcap_path: Path,
    gt_frames: dict,
    taxonomy: Taxonomy,
    args: argparse.Namespace,
) -> tuple[list[dict], dict]:
    """Return (summary rows, per-frame stats). Stats feed the paired bootstrap."""
    detections = read_detections(mcap_path, TOPICS[args.topic])
    if not detections:
        raise SystemExit(f"No {TOPICS[args.topic]} messages in {mcap_path}")
    frames = align_frames(gt_frames, detections, taxonomy, args.conf)
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


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("gt", type=Path, help="corrected GT dataset dir (from edit_labels.py)")
    parser.add_argument(
        "--candidate",
        action="append",
        required=True,
        metavar="NAME=MCAP",
        help="candidate run, repeatable (bare path uses the file stem as name)",
    )
    parser.add_argument(
        "--topic",
        choices=sorted(TOPICS),
        default="blob",
        help="which model's detections to score (default: blob)",
    )
    parser.add_argument("--taxonomy", type=Path, help="label -> archetype mapping yaml")
    parser.add_argument("--iou", type=float, default=0.5, help="IoU match threshold")
    parser.add_argument("--conf", type=float, default=0.0, help="min confidence (0 = as recorded)")
    parser.add_argument(
        "--baseline",
        help="candidate name to compare the others against (default: first --candidate)",
    )
    parser.add_argument(
        "--bootstrap",
        type=int,
        default=1000,
        help="paired-bootstrap resamples for significance (0 disables)",
    )
    parser.add_argument("--seed", type=int, default=0, help="bootstrap RNG seed")
    parser.add_argument(
        "--alpha", type=float, default=0.05, help="significance level (0.05 = 95%% CI)"
    )
    parser.add_argument("--output", type=Path, help="plot/csv output dir (default: <gt>/scores)")
    args = parser.parse_args()
    args.output = args.output or (args.gt / "scores")
    args.output.mkdir(parents=True, exist_ok=True)

    gt_frames, names = load_gt(args.gt)
    taxonomy = Taxonomy(args.taxonomy)
    print(f"GT: {len(gt_frames)} frames, classes: {names}")

    rows = []
    stats_by_candidate: dict[str, dict] = {}
    for name, mcap_path in parse_candidates(args.candidate).items():
        print(f"Scoring {name}: {mcap_path}")
        candidate_rows, stats = score_candidate(name, mcap_path, gt_frames, taxonomy, args)
        rows.extend(candidate_rows)
        stats_by_candidate[name] = stats

    summary = pd.DataFrame(rows)
    core_cols = [
        "candidate",
        "level",
        "map50",
        "map50_95",
        "precision",
        "recall",
        "f1",
        "localization_recall",
        "wrong_class_rate",
    ]
    core_cols += [c for c in summary.columns if c.startswith("kp_")]
    print(summary[core_cols].to_string(index=False, float_format=lambda v: f"{v:.3f}"))
    summary.to_csv(args.output / "summary.csv", index=False)
    plot_headline(summary, args.output)

    outputs = "summary.csv, headline.png, confusion_*.png"
    baseline = args.baseline or next(iter(stats_by_candidate))
    if args.baseline and args.baseline not in stats_by_candidate:
        raise SystemExit(f"--baseline {args.baseline!r} is not one of {list(stats_by_candidate)}")
    if len(stats_by_candidate) >= 2 and args.bootstrap > 0:
        significance = run_significance(stats_by_candidate, baseline, args.topic, args)
        conf_pct = round(100 * (1 - args.alpha))
        print(
            f"\nPaired bootstrap vs baseline '{baseline}' "
            f"({args.bootstrap} resamples, {conf_pct}% CI on delta = candidate - baseline):"
        )
        print(
            significance.to_string(
                index=False,
                float_format=lambda v: f"{v:.3f}",
                columns=[
                    "candidate",
                    "level",
                    "metric",
                    "baseline_val",
                    "candidate_val",
                    "delta",
                    "ci_low",
                    "ci_high",
                    "verdict",
                ],
            )
        )
        significance.to_csv(args.output / "significance.csv", index=False)
        outputs += ", significance.csv"
    elif len(stats_by_candidate) < 2:
        print("\n(Only one candidate; add another --candidate for a significance comparison.)")

    print(f"Wrote {args.output}/{{{outputs}}}")


if __name__ == "__main__":
    main()
