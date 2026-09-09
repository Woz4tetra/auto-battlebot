"""Detection and keypoint metrics over matched GT/prediction frames.

Detection metrics are frame-decomposable (recall, precision, F1, localization recall) so
the paired bootstrap in `significance.py` can resample them. mAP is dataset-level and
stays a point estimate. Keypoint metrics are level-independent.
"""

from __future__ import annotations

import numpy as np
import torch
from torchmetrics.detection import MeanAveragePrecision

from auto_battlebot.eval.dataset import Frame, Taxonomy

PCK_FRACTION = 0.1

FRONT_IDX = 0
BACK_IDX = 1
HEADING_THRESHOLD_DEG = 10.0
HEADING_MIN_VECTOR_PX = 1e-6  # skip when front/back coincide; angle is undefined
HEADING_ACC_KEY = f"kp_heading_acc@{HEADING_THRESHOLD_DEG:g}deg"
PCK_KEY = f"kp_pck@{PCK_FRACTION:g}"


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
    metrics: dict[str, float] = {
        "kp_err_px": float(stats["err_sum"].sum()) / err_cnt,
        PCK_KEY: float(stats["pck_cnt"].sum()) / err_cnt,
    }
    head_cnt = float(stats["head_cnt"].sum())
    if head_cnt > 0:
        metrics["kp_heading_err_deg"] = float(stats["head_err_sum"].sum()) / head_cnt
        metrics[HEADING_ACC_KEY] = float(stats["head_correct"].sum()) / head_cnt
    # Sample sizes behind the ratios above. On an eval set where only a few classes carry
    # keypoints these are small enough to decide whether a delta is worth believing, so
    # they are reported next to the metric rather than left in the stats dict. Excluded
    # from KEYPOINT_METRICS: they are counts, not scores, and are not bootstrapped.
    metrics["kp_matched_kps"] = err_cnt
    metrics["kp_matched_boxes"] = head_cnt
    return metrics
