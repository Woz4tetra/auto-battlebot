#!/usr/bin/env python3
"""Compare the C++ and Python inference paths for one detector on the same GT frames.

The C++ path is detections the perception stack recorded during playback
(/blob_detections or /keypoint_detections in one or more MCAPs). The Python path is
direct TensorRT inference on the GT images (auto_battlebot.trt_yolo, as score.py uses).
Both are scored with score.py's machinery on exactly the GT frames the recordings cover,
so the two columns are frame-for-frame comparable.

Beyond the metric table, matched C++/Python detection pairs are compared directly:
box center offset, and a linear fit of C++ keypoint coordinates against Python's
(slope != 1 means a scale error somewhere in the C++ path, intercept != 0 a translation).

Usage:
    python training/model_eval/compare_cpp_python.py training/data/nhrl_keypoints_eval_test \
        --engine data/models/yolo26n-pose_our_robots_2026-05-01_x86_64_sm89.engine \
        --labels mr_stabs_mk2,mrs_buff_mk3 --topic keypoint --conf 0.5 \
        --taxonomy training/model_eval/taxonomy_keypoint.yaml \
        --cpp-mcap 'data/recordings/auto_battlebot_experiments_eval_smoke_*.mcap'
"""

from __future__ import annotations

import argparse
import glob
from pathlib import Path

import numpy as np
import pandas as pd
from score import (
    LEVELS,
    Frame,
    Taxonomy,
    compute_map,
    infer_frames,
    iou_matrix,
    keypoint_metrics_from_stats,
    keypoint_per_frame,
    load_gt,
    pr_from_counts,
    pr_per_frame,
)

from auto_battlebot.mcap_io import (
    BLOB_DETECTIONS_TOPIC,
    KEYPOINT_DETECTIONS_TOPIC,
    Detections,
    match_stamps,
    read_detections,
)
from auto_battlebot.trt_yolo import TrtYoloModel

TOPICS = {"blob": BLOB_DETECTIONS_TOPIC, "keypoint": KEYPOINT_DETECTIONS_TOPIC}
STAMP_TOLERANCE_NS = 1_000_000


def read_cpp_detections(patterns: list[str], topic: str) -> dict[int, Detections]:
    """Union of recorded detections across MCAPs, deduped by frame stamp (first wins)."""
    by_stamp: dict[int, Detections] = {}
    paths = sorted({p for pattern in patterns for p in glob.glob(pattern)})
    if not paths:
        raise SystemExit(f"No MCAPs matched {patterns}")
    for path in paths:
        for dets in read_detections(path, topic):
            by_stamp.setdefault(dets.stamp_ns, dets)
    print(f"C++ detections: {len(by_stamp)} frames from {len(paths)} recording(s)")
    return by_stamp


def cpp_frames(
    gt_frames: dict,
    cpp_by_stamp: dict[int, Detections],
    matches: dict[int, int],
    taxonomy: Taxonomy,
) -> list[Frame]:
    """Build score.py Frames from the recorded C++ detections."""
    frames = []
    for gt_stamp, (gt_boxes, gt_labels, gt_keypoints) in gt_frames.items():
        dets = cpp_by_stamp[matches[gt_stamp]]
        keep = [d for d in dets.detections if d.label not in taxonomy.exclude]
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


def score_path(frames: list[Frame], taxonomy: Taxonomy, iou: float) -> dict[str, float]:
    """Flat {metric_name: value} over all levels plus keypoint metrics."""
    out: dict[str, float] = {}
    for level in LEVELS:
        counts, _confusion = pr_per_frame(frames, taxonomy, level, iou)
        for key, value in pr_from_counts(counts).items():
            out[f"{key} ({level})"] = value
        for key, value in compute_map(frames, taxonomy, level).items():
            if not key.startswith("ap50_95/"):
                out[f"{key} ({level})"] = value
    for key, value in keypoint_metrics_from_stats(keypoint_per_frame(frames, iou)).items():
        out[key] = value
    return out


def _matched_pairs(
    cpp_frame: Frame, py_frame: Frame, iou_threshold: float
) -> list[tuple[int, int]]:
    """Greedy IoU match of a frame's C++ detections to its Python detections."""
    ious = iou_matrix(cpp_frame.pred_boxes, py_frame.pred_boxes)
    if ious.size == 0:
        return []
    pairs = []
    used_py: set[int] = set()
    for ci in np.argsort(-cpp_frame.pred_scores):
        for pj in np.argsort(-ious[ci]):
            if int(pj) in used_py or ious[ci, pj] < iou_threshold:
                continue
            used_py.add(int(pj))
            pairs.append((int(ci), int(pj)))
            break
    return pairs


def cross_path_offsets(
    cpp: list[Frame], python: list[Frame], iou_threshold: float
) -> pd.DataFrame | None:
    """Match C++ detections to Python detections directly and fit cpp = a * python + b.

    Returns per-coordinate rows (kp_x, kp_y, box_cx, box_cy) with slope, intercept, mean
    offset, and pair count, or None when nothing matched or the model has no keypoints."""
    pairs_kp: list[tuple[np.ndarray, np.ndarray]] = []
    pairs_box: list[tuple[np.ndarray, np.ndarray]] = []
    for cpp_frame, py_frame in zip(cpp, python):
        for ci, pj in _matched_pairs(cpp_frame, py_frame, iou_threshold):
            cpp_box, py_box = cpp_frame.pred_boxes[ci], py_frame.pred_boxes[pj]
            pairs_box.append(
                (
                    np.array([(cpp_box[0] + cpp_box[2]) / 2, (cpp_box[1] + cpp_box[3]) / 2]),
                    np.array([(py_box[0] + py_box[2]) / 2, (py_box[1] + py_box[3]) / 2]),
                )
            )
            cpp_kp = cpp_frame.pred_keypoints[ci]
            py_kp = py_frame.pred_keypoints[pj]
            if len(cpp_kp) and len(cpp_kp) == len(py_kp):
                for k in range(len(cpp_kp)):
                    pairs_kp.append((cpp_kp[k, :2], py_kp[k, :2]))
    if not pairs_box:
        return None

    rows = []
    axes: list[tuple[str, list[tuple[np.ndarray, np.ndarray]], int]] = [
        ("box_cx", pairs_box, 0),
        ("box_cy", pairs_box, 1),
    ]
    if pairs_kp:
        axes += [("kp_x", pairs_kp, 0), ("kp_y", pairs_kp, 1)]
    for name, pairs, axis in axes:
        cpp_vals = np.array([p[0][axis] for p in pairs])
        py_vals = np.array([p[1][axis] for p in pairs])
        slope, intercept = np.polyfit(py_vals, cpp_vals, 1)
        rows.append(
            {
                "coordinate": name,
                "pairs": len(pairs),
                "slope": slope,
                "intercept_px": intercept,
                "mean_offset_px": float(np.mean(cpp_vals - py_vals)),
                "mean_abs_offset_px": float(np.mean(np.abs(cpp_vals - py_vals))),
            }
        )
    return pd.DataFrame(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("gt", type=Path, help="GT dataset dir or root of subdatasets")
    parser.add_argument("--engine", type=Path, required=True, help="TensorRT engine")
    parser.add_argument(
        "--labels", required=True, help="comma-separated GT label per engine class index"
    )
    parser.add_argument(
        "--topic", choices=sorted(TOPICS), required=True, help="recorded C++ detection topic"
    )
    parser.add_argument(
        "--cpp-mcap",
        action="append",
        required=True,
        help="MCAP(s) with recorded C++ detections; repeatable, globs allowed",
    )
    parser.add_argument("--taxonomy", type=Path, help="label -> archetype mapping yaml")
    parser.add_argument("--conf", type=float, default=0.5, help="Python inference confidence")
    parser.add_argument("--nms-iou", type=float, default=0.45, help="Python inference NMS IoU")
    parser.add_argument("--iou", type=float, default=0.5, help="IoU match threshold for scoring")
    args = parser.parse_args()

    gt_frames, names, images = load_gt(args.gt)
    taxonomy = Taxonomy(args.taxonomy)
    print(f"GT: {len(gt_frames)} frames, classes: {names}")

    cpp_by_stamp = read_cpp_detections(args.cpp_mcap, TOPICS[args.topic])
    matches = match_stamps(sorted(gt_frames), sorted(cpp_by_stamp), STAMP_TOLERANCE_NS)
    common = {stamp: gt_frames[stamp] for stamp in gt_frames if stamp in matches}
    print(f"Common frames (GT covered by C++ recordings): {len(common)}/{len(gt_frames)}")
    if not common:
        raise SystemExit("No GT frames covered by the C++ recordings")

    class_labels = [label.strip() for label in args.labels.split(",")]
    model = TrtYoloModel(
        str(args.engine),
        conf_threshold=args.conf,
        nms_iou_threshold=args.nms_iou,
        num_classes=len(class_labels),
    )
    print(f"Python path: {model.describe()}")

    frames_cpp = cpp_frames(common, cpp_by_stamp, matches, taxonomy)
    frames_py = infer_frames(common, images, model, class_labels, taxonomy)

    cpp_metrics = score_path(frames_cpp, taxonomy, args.iou)
    py_metrics = score_path(frames_py, taxonomy, args.iou)
    table = pd.DataFrame(
        {
            "metric": list(cpp_metrics),
            "cpp": list(cpp_metrics.values()),
            "python": [py_metrics.get(m, float("nan")) for m in cpp_metrics],
        }
    )
    table["delta"] = table["python"] - table["cpp"]
    print("\nScores on common frames (cpp = recorded playback, python = direct inference):")
    print(table.to_string(index=False, float_format=lambda v: f"{v:.3f}"))

    offsets = cross_path_offsets(frames_cpp, frames_py, args.iou)
    if offsets is not None:
        print("\nMatched C++ vs Python detections, linear fit cpp = slope * python + intercept:")
        print(offsets.to_string(index=False, float_format=lambda v: f"{v:.4f}"))
        print("(slope != 1: scale error; intercept != 0 with slope 1: translation)")


if __name__ == "__main__":
    main()
