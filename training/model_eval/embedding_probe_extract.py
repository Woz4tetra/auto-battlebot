#!/usr/bin/env python3
"""Stage 1 of the embedding prototype probe: candidate boxes on validated GT frames.

For every validated frame of every eval recording, records two candidate pools:

- GT boxes (all classes), straight from the labels.
- Detector proposals from the deployed bbox engine at a low confidence threshold,
  through the same preprocessing + NMS as the C++ pipeline (TrtYoloModel).

Writes per recording under training/data/embedding_probe/<recording>/:

- candidates.csv: one row per box (source, label, conf, xyxy, best IoU vs GT, role)
- manifest.json: seed stamps, counts, engine metadata
- seed_mosaic.png: the seed opponent crops
- opponent_timeline.png: every GT opponent crop in stamp order, for an identity /
  appearance-drift eyeball (checks the massD single-fight assumption)

Crops themselves are not stored; later stages re-crop from the dataset PNGs.

Usage:
    cd training/model_eval && python embedding_probe_extract.py [--eval-root DIR]
        [--engine PATH] [--conf 0.05] [--recordings 10-06,massd]
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import embedding_probe_common as common
import numpy as np
import pandas as pd
from tqdm import tqdm


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--eval-root", type=Path, default=common.DEFAULT_EVAL_ROOT)
    parser.add_argument("--engine", type=Path, default=common.DEFAULT_ENGINE)
    parser.add_argument("--conf", type=float, default=common.LOW_CONF)
    parser.add_argument("--nms-iou", type=float, default=common.NMS_IOU)
    parser.add_argument(
        "--recordings",
        type=str,
        default="",
        help="Comma-separated short names (see embedding_probe_common.SHORT_NAMES); default all",
    )
    return parser.parse_args()


def build_rows(
    stamp: int,
    boxes: np.ndarray,
    labels: list[str],
    detections: list,
    role: str,
) -> list[dict]:
    rows = []
    for index, (box, label) in enumerate(zip(boxes, labels)):
        rows.append(
            {
                "stamp_ns": stamp,
                "role": role,
                "source": "gt",
                "label": label,
                "conf": 1.0,
                "x1": box[0],
                "y1": box[1],
                "x2": box[2],
                "y2": box[3],
                "best_iou": 1.0,
                "best_gt_label": label,
                "best_gt_index": index,
            }
        )
    if detections:
        prop_boxes = np.asarray([det[0] for det in detections], dtype=np.float64)
        ious = common.iou_matrix(prop_boxes, boxes)
        for det_index, (xyxy, conf, class_id, _kps) in enumerate(detections):
            if len(boxes):
                best = int(np.argmax(ious[det_index]))
                best_iou = float(ious[det_index, best])
                best_label = labels[best]
            else:
                best, best_iou, best_label = -1, 0.0, ""
            rows.append(
                {
                    "stamp_ns": stamp,
                    "role": role,
                    "source": "proposal",
                    "label": common.ENGINE_LABELS[class_id]
                    if class_id < len(common.ENGINE_LABELS)
                    else f"class_{class_id}",
                    "conf": float(conf),
                    "x1": float(xyxy[0]),
                    "y1": float(xyxy[1]),
                    "x2": float(xyxy[2]),
                    "y2": float(xyxy[3]),
                    "best_iou": best_iou,
                    "best_gt_label": best_label,
                    "best_gt_index": best,
                }
            )
    return rows


def mosaic(crops: list[np.ndarray], tile: int = 96, columns: int = 12) -> np.ndarray:
    if not crops:
        return np.zeros((tile, tile, 3), dtype=np.uint8)
    tiles = [cv2.resize(c, (tile, tile), interpolation=cv2.INTER_AREA) for c in crops]
    rows = []
    for start in range(0, len(tiles), columns):
        row = tiles[start : start + columns]
        row += [np.zeros((tile, tile, 3), dtype=np.uint8)] * (columns - len(row))
        rows.append(np.hstack(row))
    return np.vstack(rows)


def opponent_crops(entry: dict, stamps: list[int]) -> list[np.ndarray]:
    crops = []
    for stamp in stamps:
        boxes, labels = entry["frames"][stamp]
        image = None
        for box, label in zip(boxes, labels):
            if label != common.OPPONENT:
                continue
            if image is None:
                image = cv2.imread(str(entry["images"][stamp]))
            if image is not None:
                crops.append(common.crop_box(image, box, common.DEFAULT_PAD))
    return crops


def main() -> None:
    args = parse_args()
    recordings = common.load_recordings(args.eval_root)
    wanted = {name.strip() for name in args.recordings.split(",") if name.strip()}

    from auto_battlebot.trt_yolo import TrtYoloModel

    model = TrtYoloModel(
        str(args.engine),
        conf_threshold=args.conf,
        nms_iou_threshold=args.nms_iou,
        num_classes=len(common.ENGINE_LABELS),
    )
    print(model.describe())

    summary = []
    for recording, entry in sorted(recordings.items()):
        short = common.short_name(recording)
        if wanted and short not in wanted:
            continue
        seeds = common.seed_stamps(entry)
        if not seeds:
            print(f"{short}: no opponent boxes in any validated frame, skipping")
            continue
        out_dir = common.recording_dir(recording)
        out_dir.mkdir(parents=True, exist_ok=True)

        rows: list[dict] = []
        for stamp in tqdm(entry["stamps"], desc=short):
            boxes, labels = entry["frames"][stamp]
            image = cv2.imread(str(entry["images"][stamp]))
            if image is None:
                raise SystemExit(f"Unreadable image {entry['images'][stamp]}")
            detections = model.infer(image)
            role = "seed" if stamp in seeds else "eval"
            rows.extend(build_rows(stamp, boxes, labels, detections, role))

        frame = pd.DataFrame(rows)
        frame.to_csv(out_dir / common.CANDIDATES_CSV, index=False)

        seed_crop_images = opponent_crops(entry, seeds)
        cv2.imwrite(str(out_dir / "seed_mosaic.png"), mosaic(seed_crop_images))
        timeline = opponent_crops(entry, entry["stamps"])
        cv2.imwrite(str(out_dir / "opponent_timeline.png"), mosaic(timeline))

        eval_frame = frame[frame.role == "eval"]
        manifest = {
            "recording": recording,
            "short_name": short,
            "engine": str(args.engine),
            "conf": args.conf,
            "nms_iou": args.nms_iou,
            "engine_labels": list(common.ENGINE_LABELS),
            "seed_stamps": seeds,
            "n_frames": len(entry["stamps"]),
            "n_seed_crops": len(seed_crop_images),
            "n_gt": int((frame.source == "gt").sum()),
            "n_proposals": int((frame.source == "proposal").sum()),
            "n_eval_opponent_gt": int(
                ((eval_frame.source == "gt") & (eval_frame.label == common.OPPONENT)).sum()
            ),
        }
        (out_dir / common.MANIFEST_JSON).write_text(json.dumps(manifest, indent=2) + "\n")
        summary.append(manifest)

    print(f"\n{'rec':8s} {'frames':>6s} {'seeds':>5s} {'gt':>5s} {'props':>6s} {'evalopp':>7s}")
    for manifest in summary:
        print(
            f"{manifest['short_name']:8s} {manifest['n_frames']:6d} "
            f"{manifest['n_seed_crops']:5d} {manifest['n_gt']:5d} "
            f"{manifest['n_proposals']:6d} {manifest['n_eval_opponent_gt']:7d}"
        )


if __name__ == "__main__":
    main()
