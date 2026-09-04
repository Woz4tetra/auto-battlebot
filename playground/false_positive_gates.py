#!/usr/bin/env python3
"""Diagnose what separates a detector's false positives from its true ones, and where they sit.

`depth_gated_subtraction.py` decides each detector box with a fixed threshold on two signals: the
fraction of the box that moved against the floor background, and the fraction that stands proud of
its surroundings in depth. This asks the prior question those thresholds assume an answer to: does
either signal separate a real robot from a false positive at all, and if so, where should the
threshold be?

Three outputs:

1. **Separability.** Every `yolo_only` box is matched to ground truth at IoU 0.5 and labelled true
   or false. The AUC of each signal is the probability that a random true box scores above a
   random false one. 0.5 is a coin flip. Below 0.5 means the signal is backwards: the boxes it
   likes are the wrong ones.

2. **Threshold sweep.** Precision, recall and F1 across the whole range of each gate, so the
   operating point is chosen from a curve rather than guessed. A gate whose curve never beats its
   own baseline has nothing to offer at any threshold.

3. **Static false positives.** Each false positive's box centre is projected onto the arena floor
   through that frame's pose and clustered in the field frame. A robot moves, so its detections
   scatter. A painted floor graphic does not, so its false positives pile into one tight cluster
   that persists across the recording. This is what names the logo rather than just counting it.

`--crops` additionally writes a mosaic of what the detector actually saw in each false positive,
sorted by confidence. A cluster count says a mistake repeats; the mosaic says what the mistake is.

Usage:
    python playground/false_positive_gates.py <subdataset> \
        --predictions training/data/eval_results/logo_fp_massd/yolo_only.json \
        -o training/data/eval_results/logo_fp_massd/gates
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from auto_battlebot.camera_geometry import load_frame_geometry, pixels_to_floor

# IoU at which a detector box counts as having found the ground-truth robot. Matches score.py.
MATCH_IOU = 0.5

# Field-frame radius within which two false positives are called the same static object. The
# arena is about 2.4 m square and a robot is roughly 0.25 m, so this is a robot-ish footprint.
CLUSTER_RADIUS_M = 0.20

# A cluster this size or larger is a fixture of the scene rather than a one-off mistake.
STATIC_CLUSTER_MIN_FRAMES = 5

SIGNALS = ("fg_fraction", "prom_fraction", "height_fraction")

CROP_SIZE = (150, 100)
CROP_COLUMNS = 8
CROP_PAD_PX = 6


def gt_boxes(subdataset: Path, stem: str, width: int, height: int) -> np.ndarray:
    path = subdataset / "labels" / f"{stem}.txt"
    if not path.exists():
        return np.zeros((0, 4))
    rows = []
    for line in path.read_text().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        cx, cy, box_w, box_h = (float(v) for v in parts[1:5])
        rows.append(
            [
                (cx - box_w / 2) * width,
                (cy - box_h / 2) * height,
                (cx + box_w / 2) * width,
                (cy + box_h / 2) * height,
            ]
        )
    return np.asarray(rows, dtype=np.float64).reshape(-1, 4)


def best_iou(box: np.ndarray, others: np.ndarray) -> float:
    if len(others) == 0:
        return 0.0
    x1 = np.maximum(box[0], others[:, 0])
    y1 = np.maximum(box[1], others[:, 1])
    x2 = np.minimum(box[2], others[:, 2])
    y2 = np.minimum(box[3], others[:, 3])
    inter = np.clip(x2 - x1, 0, None) * np.clip(y2 - y1, 0, None)
    area = (box[2] - box[0]) * (box[3] - box[1])
    areas = (others[:, 2] - others[:, 0]) * (others[:, 3] - others[:, 1])
    union = area + areas - inter
    return float(np.max(np.where(union > 0, inter / union, 0.0)))


def auc(positive: np.ndarray, negative: np.ndarray) -> float:
    """P(a random positive scores above a random negative), ties counted as half."""
    if len(positive) == 0 or len(negative) == 0:
        return float("nan")
    greater = (positive[:, None] > negative[None, :]).mean()
    tied = (positive[:, None] == negative[None, :]).mean()
    return float(greater + 0.5 * tied)


def sweep(values: np.ndarray, is_true: np.ndarray, total_gt: int) -> list[dict]:
    """Precision/recall/F1 as the gate threshold rises, keeping boxes at or above it."""
    rows = []
    for threshold in np.round(np.arange(0.0, 1.001, 0.05), 2):
        keep = values >= threshold
        kept_true = int(is_true[keep].sum())
        kept = int(keep.sum())
        precision = kept_true / kept if kept else 0.0
        recall = kept_true / total_gt if total_gt else 0.0
        f1 = 2 * precision * recall / (precision + recall) if precision + recall else 0.0
        rows.append(
            {
                "threshold": float(threshold),
                "kept": kept,
                "precision": round(precision, 4),
                "recall": round(recall, 4),
                "f1": round(f1, 4),
            }
        )
    return rows


def cluster_static(points: list[tuple[np.ndarray, str]]) -> list[dict]:
    """Greedy single-link clustering of field-frame false positives, largest first.

    Greedy is enough here: the question is whether a pile exists at a fixed spot, not where its
    boundary lies to the centimetre.
    """
    remaining = list(points)
    clusters = []
    while remaining:
        seed = remaining[0][0]
        near = [
            index
            for index, (point, _) in enumerate(remaining)
            if float(np.linalg.norm(point[:2] - seed[:2])) <= CLUSTER_RADIUS_M
        ]
        members = [remaining[index] for index in near]
        taken = set(near)
        remaining = [p for index, p in enumerate(remaining) if index not in taken]
        centre = np.mean([p[0][:2] for p in members], axis=0)
        clusters.append(
            {
                "centre_field_m": [round(float(v), 3) for v in centre],
                "false_positives": len(members),
                "frames": len(({p[1] for p in members})),
                "static": len({p[1] for p in members}) >= STATIC_CLUSTER_MIN_FRAMES,
            }
        )
    return sorted(clusters, key=lambda c: -c["false_positives"])


def crop_mosaic(crops: list[tuple[float, np.ndarray]], path: Path, limit: int) -> None:
    """Grid of the highest-confidence false positives, so the mistake can be named on sight."""
    if not crops:
        return
    tiles = [crop for _, crop in sorted(crops, key=lambda c: -c[0])[:limit]]
    blank = np.zeros((CROP_SIZE[1], CROP_SIZE[0], 3), np.uint8)
    while len(tiles) % CROP_COLUMNS:
        tiles.append(blank)
    rows = [np.hstack(tiles[i : i + CROP_COLUMNS]) for i in range(0, len(tiles), CROP_COLUMNS)]
    cv2.imwrite(str(path), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 94])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("subdataset", type=Path, help="eval sub dataset with labels/ and images/")
    parser.add_argument(
        "--predictions", type=Path, required=True, help="ungated arm JSON carrying the fractions"
    )
    parser.add_argument("-o", "--output", type=Path, required=True)
    parser.add_argument("--crops", type=Path, default=None, help="write an FP crop mosaic here")
    parser.add_argument("--crop-limit", type=int, default=48)
    args = parser.parse_args()

    payload = json.loads(args.predictions.read_text())
    images = args.subdataset / "images"

    records: list[dict] = []
    field_points: list[tuple[np.ndarray, str]] = []
    crops: list[tuple[float, np.ndarray]] = []
    total_gt = 0
    for stem, rows in payload.get("frames", {}).items():
        image_path = images / f"{stem}.png"
        image = cv2.imread(str(image_path))
        if image is None:
            continue
        height, width = image.shape[:2]
        truth = gt_boxes(args.subdataset, stem, width, height)
        total_gt += len(truth)
        geometry = load_frame_geometry(image_path)

        for row in rows:
            box = np.asarray(row["xyxy"], dtype=np.float64)
            matched = best_iou(box, truth) >= MATCH_IOU
            records.append(
                {
                    "stem": stem,
                    "is_true": matched,
                    **{key: float(row.get(key, float("nan"))) for key in SIGNALS},
                }
            )
            if matched:
                continue

            if args.crops is not None:
                x1 = max(0, int(box[0]) - CROP_PAD_PX)
                y1 = max(0, int(box[1]) - CROP_PAD_PX)
                x2 = min(width, int(box[2]) + CROP_PAD_PX)
                y2 = min(height, int(box[3]) + CROP_PAD_PX)
                if x2 - x1 >= 10 and y2 - y1 >= 10:
                    crop = cv2.resize(image[y1:y2, x1:x2], CROP_SIZE)
                    caption = (
                        f"{row.get('score', 0):.2f} fg{row.get('fg_fraction', 0):.2f} "
                        f"d{row.get('prom_fraction', 0):.2f}"
                    )
                    cv2.putText(
                        crop, caption, (3, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (255, 255, 255), 1
                    )
                    crops.append((float(row.get("score", 0.0)), crop))

            if geometry is None:
                continue
            # The bottom centre of a box is where the object meets the floor, so it is the point
            # that projects onto the arena rather than into the air above it.
            foot = np.array([[(box[0] + box[2]) / 2, box[3]]])
            point = pixels_to_floor(foot, geometry)[0]
            if np.isfinite(point).all():
                field_points.append((point, stem))

    is_true = np.array([r["is_true"] for r in records], dtype=bool)
    report: dict = {
        "predictions": str(args.predictions),
        "boxes": len(records),
        "true_positives": int(is_true.sum()),
        "false_positives": int((~is_true).sum()),
        "gt_boxes": total_gt,
        "signals": {},
    }
    for signal in SIGNALS:
        values = np.array([r[signal] for r in records], dtype=float)
        usable = np.isfinite(values)
        report["signals"][signal] = {
            "auc_true_over_false": round(
                auc(values[usable & is_true], values[usable & ~is_true]), 4
            ),
            "mean_true": round(float(np.nanmean(values[is_true])), 4),
            "mean_false": round(float(np.nanmean(values[~is_true])), 4),
            "sweep": sweep(values[usable], is_true[usable], total_gt),
        }

    clusters = cluster_static(field_points)
    report["false_positive_clusters"] = clusters[:10]
    report["static_clusters"] = sum(1 for c in clusters if c["static"])

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2))
    if args.crops is not None:
        args.crops.parent.mkdir(parents=True, exist_ok=True)
        crop_mosaic(crops, args.crops, args.crop_limit)

    print(
        f"{report['boxes']} boxes: {report['true_positives']} true, "
        f"{report['false_positives']} false"
    )
    for signal, stats in report["signals"].items():
        best = max(stats["sweep"], key=lambda r: r["f1"])
        print(
            f"  {signal:14s} AUC {stats['auc_true_over_false']:.3f}  "
            f"true {stats['mean_true']:.3f} vs false {stats['mean_false']:.3f}  "
            f"best F1 {best['f1']:.3f} at >= {best['threshold']}"
        )
    print(
        f"\nStatic false-positive clusters (>= {STATIC_CLUSTER_MIN_FRAMES} frames): "
        f"{report['static_clusters']}"
    )
    for cluster in clusters[:5]:
        flag = "static" if cluster["static"] else ""
        print(
            f"  {cluster['centre_field_m']}  {cluster['false_positives']} FPs over "
            f"{cluster['frames']} frames {flag}"
        )
    print(f"\nWrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
