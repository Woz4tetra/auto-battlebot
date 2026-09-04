#!/usr/bin/env python3
"""Stage 2 of the open-set detector probe: precision/recall vs the deployed baseline.

Scores the detections written by openset_probe_predict.py on the validated eval
frames (seed frames excluded), with the same greedy IoU >= 0.5 matching as
embedding_probe_pr.py. Two matching modes per method:

- strict: predictions matched against GT opponent boxes only. Any detection of
  our robot or the house bot counts as a false positive. Directly comparable to
  the baseline/lowconf rows in embedding_probe pr_comparison.csv.
- robot: predictions matched against all GT robot boxes (opponent, our robots,
  house_bot). Recall is still reported on opponent boxes; precision counts any
  robot hit as a true positive. This is what an identity-by-elimination
  architecture (opponent = robot that is not us) would see from the detector.

Operating points per method and mode:

- f1: score threshold maximizing pooled F1.
- worst: score threshold maximizing the worst-recording opponent recall,
  tie-broken by pooled precision. The probe's uniformity criterion.
- floor: everything above the stored floor (the recall ceiling, like lowconf).

Writes training/data/openset_probe/pr_openset.csv (per recording x method x mode x
operating point) and prints markdown tables for the report.

Usage:
    cd training/model_eval && python openset_probe_score.py [--eval-root DIR]
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import embedding_probe_common as common
import numpy as np
import pandas as pd
from openset_probe_predict import OUTPUT_ROOT

MATCH_IOU = 0.5
ROBOT_LABELS = ("mr_stabs_mk2", "mrs_buff_mk3", "opponent", "house_bot")
TARGET_RECALL = 0.8
STATIC_CELL_PX = 64
STATIC_FRAME_FRACTION = 0.25


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--eval-root", type=Path, default=common.DEFAULT_EVAL_ROOT)
    return parser.parse_args()


def score_frames(
    predictions: pd.DataFrame,
    gt_by_stamp: dict[int, tuple[np.ndarray, list[str]]],
    mode: str,
) -> tuple[int, int, int, int]:
    """Greedy per-frame matching. Returns (tp_opponent, tp_any, fp, n_gt_opponent)."""
    tp_opponent = tp_any = fp = n_gt_opponent = 0
    wanted = (common.OPPONENT,) if mode == "strict" else ROBOT_LABELS
    for stamp, (boxes, labels) in gt_by_stamp.items():
        keep = [index for index, label in enumerate(labels) if label in wanted]
        n_gt_opponent += sum(1 for index in keep if labels[index] == common.OPPONENT)
        group = predictions[predictions.stamp_ns == stamp]
        if not len(group):
            continue
        gt_boxes = boxes[keep] if len(keep) else np.zeros((0, 4))
        gt_labels = [labels[index] for index in keep]
        matched = np.zeros(len(gt_boxes), dtype=bool)
        pred_boxes = group[["x1", "y1", "x2", "y2"]].to_numpy()
        ious = common.iou_matrix(pred_boxes, gt_boxes)
        for row_position in np.argsort(-group.score.to_numpy()):
            if len(gt_boxes) == 0:
                fp += 1
                continue
            available = np.where(~matched, ious[row_position], -1.0)
            best = int(np.argmax(available))
            if available[best] >= MATCH_IOU:
                matched[best] = True
                tp_any += 1
                if gt_labels[best] == common.OPPONENT:
                    tp_opponent += 1
            else:
                fp += 1
    return tp_opponent, tp_any, fp, n_gt_opponent


def method_names(recordings: dict[str, dict]) -> list[str]:
    names = set()
    for recording in recordings:
        for path in (OUTPUT_ROOT / recording).glob("detections_*.csv"):
            names.add(path.stem.removeprefix("detections_"))
    return sorted(names)


def sweep_thresholds(scores: np.ndarray, count: int = 60) -> np.ndarray:
    if not len(scores):
        return np.array([0.0])
    return np.unique(np.quantile(scores, np.linspace(0.0, 0.995, count)))


def evaluate(
    detections: dict[str, pd.DataFrame],
    gt: dict[str, dict[int, tuple[np.ndarray, list[str]]]],
    mode: str,
    threshold: float,
) -> pd.DataFrame:
    rows = []
    for recording, frame_gt in gt.items():
        preds = detections[recording]
        preds = preds[preds.score >= threshold]
        tp_opponent, tp_any, fp, n_gt = score_frames(preds, frame_gt, mode)
        denominator = tp_any + fp
        rows.append(
            {
                "recording": common.SHORT_NAMES.get(recording, recording),
                "mode": mode,
                "threshold": threshold,
                "tp": tp_opponent,
                "tp_any": tp_any,
                "fp": fp,
                "n_gt": n_gt,
                "recall": tp_opponent / n_gt if n_gt else float("nan"),
                "precision": tp_any / denominator if denominator else float("nan"),
            }
        )
    return pd.DataFrame(rows)


def fp_static_fraction(
    preds: pd.DataFrame, frame_gt: dict[int, tuple[np.ndarray, list[str]]]
) -> float:
    """Fraction of non-robot detections whose center cell recurs across frames.

    Estimates how much of the false-positive volume full-rate static suppression
    would remove (91% of massD deployed false positives sat in fixed clusters).
    Computed on the false-positive set only, so parked robots are not counted.
    """
    if not len(preds):
        return float("nan")
    is_fp = np.ones(len(preds), dtype=bool)
    boxes = preds[["x1", "y1", "x2", "y2"]].to_numpy()
    for position, (_, row) in enumerate(preds.iterrows()):
        gt_boxes, gt_labels = frame_gt.get(row.stamp_ns, (np.zeros((0, 4)), []))
        robot_boxes = np.asarray(
            [box for box, label in zip(gt_boxes, gt_labels) if label in ROBOT_LABELS]
        )
        if len(robot_boxes):
            iou = common.iou_matrix(boxes[position : position + 1], robot_boxes)
            if iou.max() >= MATCH_IOU:
                is_fp[position] = False
    false_positives = preds[is_fp]
    if not len(false_positives):
        return float("nan")
    n_stamps = preds.stamp_ns.nunique()
    cx = ((false_positives.x1 + false_positives.x2) / 2 // STATIC_CELL_PX).astype(int)
    cy = ((false_positives.y1 + false_positives.y2) / 2 // STATIC_CELL_PX).astype(int)
    cell = cx * 10000 + cy
    frames_per_cell = false_positives.assign(cell=cell).groupby("cell").stamp_ns.nunique()
    static_cells = set(frames_per_cell[frames_per_cell >= STATIC_FRAME_FRACTION * n_stamps].index)
    return float(cell.isin(static_cells).mean())


def target_recall_rows(
    detections: dict[str, pd.DataFrame],
    gt: dict[str, dict[int, tuple[np.ndarray, list[str]]]],
    mode: str,
    method: str,
    point: str,
) -> list[pd.DataFrame]:
    """Per-recording threshold: max precision subject to recall >= TARGET_RECALL.

    Models a per-venue calibration (the 10-20 s pre-match window). Falls back to
    the max-recall threshold when the target is unreachable.
    """
    rows = []
    for recording, frame_gt in gt.items():
        preds = detections[recording]
        best = None
        fallback = None
        for threshold in sweep_thresholds(preds.score.to_numpy() if len(preds) else np.array([])):
            subset = preds[preds.score >= threshold] if len(preds) else preds
            tp_opponent, tp_any, fp, n_gt = score_frames(subset, frame_gt, mode)
            denominator = tp_any + fp
            row = {
                "recording": common.SHORT_NAMES.get(recording, recording),
                "mode": mode,
                "threshold": threshold,
                "tp": tp_opponent,
                "tp_any": tp_any,
                "fp": fp,
                "n_gt": n_gt,
                "recall": tp_opponent / n_gt if n_gt else float("nan"),
                "precision": tp_any / denominator if denominator else float("nan"),
            }
            if fallback is None or row["recall"] > fallback["recall"]:
                fallback = row
            if row["recall"] >= TARGET_RECALL and (
                best is None or row["precision"] > best["precision"]
            ):
                best = row
        chosen = best if best is not None else fallback
        chosen["fp_static_fraction"] = fp_static_fraction(
            preds[preds.score >= chosen["threshold"]] if len(preds) else preds, frame_gt
        )
        rows.append(pd.DataFrame([chosen]))
    return [pd.concat(rows, ignore_index=True).assign(method=method, point=point)]


def pooled(table: pd.DataFrame) -> tuple[float, float, float]:
    """(recall, precision, f1) pooled over recordings."""
    tp = table.tp.sum()
    tp_any = table.tp_any.sum()
    fp = table.fp.sum()
    n_gt = table.n_gt.sum()
    recall = tp / n_gt if n_gt else 0.0
    precision = tp_any / (tp_any + fp) if tp_any + fp else 0.0
    f1 = 2 * precision * recall / (precision + recall) if precision + recall else 0.0
    return recall, precision, f1


def main() -> None:
    args = parse_args()
    recordings = common.load_recordings(args.eval_root)
    gt: dict[str, dict[int, tuple[np.ndarray, list[str]]]] = {}
    for recording, entry in recordings.items():
        seeds = set(common.seed_stamps(entry))
        gt[recording] = {
            stamp: (np.asarray(boxes, dtype=np.float64), list(labels))
            for stamp, (boxes, labels) in entry["frames"].items()
            if stamp not in seeds
        }

    all_rows = []
    for method in method_names(recordings):
        detections = {}
        floors = []
        for recording in recordings:
            path = OUTPUT_ROOT / recording / f"detections_{method}.csv"
            frame = pd.read_csv(path) if path.exists() else pd.DataFrame()
            if len(frame):
                frame = frame[frame.role == "eval"]
            detections[recording] = frame
            manifest_path = OUTPUT_ROOT / recording / f"manifest_{method}.json"
            if manifest_path.exists():
                floors.append(json.loads(manifest_path.read_text())["floor"])
        scores = np.concatenate(
            [frame.score.to_numpy() for frame in detections.values() if len(frame)]
        )
        thresholds = sweep_thresholds(scores)
        for mode in ("strict", "robot"):
            candidates = [evaluate(detections, gt, mode, t) for t in thresholds]
            summaries = [(pooled(table), table) for table in candidates]
            by_f1 = max(summaries, key=lambda item: item[0][2])[1]
            by_worst = max(summaries, key=lambda item: (item[1].recall.min(), item[0][1]))[1]
            at_floor = candidates[0]
            for point, table in (("f1", by_f1), ("worst", by_worst), ("floor", at_floor)):
                table = table.assign(method=method, point=point)
                all_rows.append(table)
            all_rows += target_recall_rows(detections, gt, mode, method, "r80")

    result = pd.concat(all_rows, ignore_index=True)
    result.to_csv(OUTPUT_ROOT / "pr_openset.csv", index=False)
    print(f"wrote {OUTPUT_ROOT / 'pr_openset.csv'} ({len(result)} rows)")

    for mode in ("strict", "robot"):
        print(f"\n## mode={mode}, operating point = max pooled F1")
        subset = result[(result["mode"] == mode) & (result.point == "f1")]
        print("| method | threshold | pooled recall | pooled precision | worst recording recall |")
        print("|---|---|---|---|---|")
        for method, group in subset.groupby("method"):
            recall, precision, _ = pooled(group)
            print(
                f"| {method} | {group.threshold.iloc[0]:.3f} | {recall:.3f} "
                f"| {precision:.3f} | {group.recall.min():.3f} "
                f"({group.loc[group.recall.idxmin()].recording}) |"
            )


if __name__ == "__main__":
    main()
