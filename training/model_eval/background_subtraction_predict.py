#!/usr/bin/env python3
"""Detect robots by field background subtraction and write predictions for score.py.

This grades background subtraction as if it were a detector. It is not a model: it has no
weights and no classes, it just finds what is on the arena floor that was not there before.
The point of scoring it is to see how far pure geometry plus a difference image gets, and
where it breaks, next to the trained detectors on the same frames.

How a prediction is made, per sub dataset:

1. Read the source recording named in data.yaml and rebuild `field -> camera` for every
   processed frame, by composing `field -> camera_world` (/tf_static, one per field init)
   with `camera_world -> camera` (/tf). This reproduces the poses
   `export_camera_transforms.py` exported for the GT frames exactly, so the background and
   the frames it is subtracted from live in the same geometry.
2. Warp a spread of recording frames onto the arena floor, into a metric top-down raster, and
   take the per-pixel median. Robots move over a fight, so the median is an empty floor. The
   GT frames alone are too few and too clustered for this: robots dwell near the centre and
   end up baked into the background.
3. For each GT frame, warp that floor back into the image through the frame's own pose,
   difference it, and turn the blobs into boxes.

Everything found is emitted as one class, because background subtraction cannot tell a robot
from a house bot from a dropped screw. Only score.py's `agnostic` level says anything real
about it; `archetype` and `instance` measure a classifier that is not there.

Usage:
    python training/model_eval/background_subtraction_predict.py \
        training/data/nhrl_keypoints_eval_test \
        --recordings data/saved_recordings \
        -o training/data/eval_results/bgsub_predictions.json
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml
from score import reviewed_stems

from auto_battlebot.background_subtraction import (
    SubtractionParams,
    find_blobs,
    subtract,
    warp_forward,
)
from auto_battlebot.camera_geometry import NOMINAL_FIELD_SIZE_M, load_frame_geometry
from auto_battlebot.floor_background import (
    FLOOR_MARGIN_M,
    RASTER_PX_PER_M,
    SCORE_REFERENCE_DIFF,
    FloorRaster,
    build_floor_background,
    find_recording,
    read_recording,
)

GT_BGR = (0, 200, 0)
PRED_BGR = (60, 60, 235)
INK = (245, 245, 245)
SHADOW = (20, 20, 20)
FONT = cv2.FONT_HERSHEY_SIMPLEX


def read_gt_boxes(
    subdataset: Path, stem: str, names: list[str], size: tuple[int, int]
) -> list[tuple]:
    """(x1, y1, x2, y2, label) from a YOLO label file, in pixels."""
    path = subdataset / "labels" / f"{stem}.txt"
    if not path.exists():
        return []
    width, height = size
    rows = []
    for line in path.read_text().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        class_id = int(float(parts[0]))
        cx, cy, box_w, box_h = (float(v) for v in parts[1:5])
        rows.append(
            (
                (cx - box_w / 2) * width,
                (cy - box_h / 2) * height,
                (cx + box_w / 2) * width,
                (cy + box_h / 2) * height,
                names[class_id] if class_id < len(names) else f"class_{class_id}",
            )
        )
    return rows


def label_text(
    image: np.ndarray, text: str, origin: tuple[int, int], color: tuple, scale: float = 0.5
) -> None:
    cv2.putText(image, text, origin, FONT, scale, SHADOW, 3, cv2.LINE_AA)
    cv2.putText(image, text, origin, FONT, scale, color, 1, cv2.LINE_AA)


def annotate(
    frame: np.ndarray,
    difference: np.ndarray,
    predictions: list[dict],
    gt_boxes: list[tuple],
    caption: str,
) -> np.ndarray:
    """The frame with the difference laid over it, GT in green and detections in red."""
    canvas = cv2.addWeighted(frame, 1.0, cv2.cvtColor(difference, cv2.COLOR_GRAY2BGR), 0.9, 0.0)

    for x1, y1, x2, y2, name in gt_boxes:
        cv2.rectangle(canvas, (int(x1), int(y1)), (int(x2), int(y2)), GT_BGR, 2)
        label_text(canvas, name, (int(x1), max(12, int(y1) - 5)), GT_BGR)

    for row in predictions:
        x1, y1, x2, y2 = (int(v) for v in row["xyxy"])
        cv2.rectangle(canvas, (x1, y1), (x2, y2), PRED_BGR, 2)
        label_text(
            canvas,
            f"{row['score']:.2f} {row['area']}px",
            (x1, min(canvas.shape[0] - 4, y2 + 14)),
            PRED_BGR,
        )

    label_text(canvas, caption, (10, 22), INK, 0.55)
    label_text(canvas, "green: ground truth", (10, 44), GT_BGR, 0.55)
    label_text(canvas, "red: background subtraction", (10, 64), PRED_BGR, 0.55)
    return canvas


def write_contact_sheet(tiles: list[np.ndarray], path: Path, columns: int = 5) -> None:
    """Grid of downscaled frames, so a whole recording can be scanned at once."""
    if not tiles:
        return
    width, height = 384, 216
    small = [cv2.resize(tile, (width, height)) for tile in tiles]
    rows = []
    for start in range(0, len(small), columns):
        row = small[start : start + columns]
        while len(row) < columns:
            row.append(np.zeros((height, width, 3), np.uint8))
        rows.append(np.hstack(row))
    cv2.imwrite(str(path), np.vstack(rows))


def predict_subdataset(
    subdataset: Path,
    roots: list[Path],
    params: SubtractionParams,
    args: argparse.Namespace,
    reviewed: set[str],
) -> dict[str, list[dict]]:
    data = yaml.safe_load((subdataset / "data.yaml").read_text())
    recording = read_recording(find_recording(str(data["source_mcap"]), roots))

    images = sorted((subdataset / "images").glob("*.png"))
    geometries = {path: load_frame_geometry(path) for path in images}
    usable = [path for path in images if geometries[path] is not None]
    if not usable:
        print(f"  {subdataset.name}: no usable poses, skipped")
        return {}

    first = geometries[usable[0]]
    intrinsics = (first.fx, first.fy, first.cx, first.cy)  # type: ignore[union-attr]
    sample = cv2.imread(str(usable[0]))
    image_size = (sample.shape[1], sample.shape[0])

    raster = FloorRaster(NOMINAL_FIELD_SIZE_M, args.raster_px_per_m, args.floor_margin_m)
    background, coverage = build_floor_background(
        recording, raster, args.background_samples, intrinsics
    )
    compare_mask = cv2.bitwise_and(raster.floor_mask, coverage)

    names = list(data.get("names", []))
    annotate_dir = None
    if args.annotate_dir is not None:
        annotate_dir = Path(args.annotate_dir) / subdataset.name
        annotate_dir.mkdir(parents=True, exist_ok=True)
    tiles: list[np.ndarray] = []

    predictions: dict[str, list[dict]] = {}
    for path in usable:
        geometry = geometries[path]
        homography = raster.image_from_raster(geometry)
        frame = cv2.imread(str(path))
        if frame is None:
            predictions[path.stem] = []
            continue

        warped_background = warp_forward(background, homography, image_size)
        valid = warp_forward(compare_mask, homography, image_size, nearest=True)
        valid = cv2.bitwise_and(valid, raster.in_front_mask(homography, image_size))
        valid = cv2.erode(valid, np.ones((11, 11), np.uint8))
        difference, foreground = subtract(frame, warped_background, valid, params)

        rows = []
        for x, y, width, height, area in find_blobs(foreground, params.min_area):
            window = difference[y : y + height, x : x + width]
            spot = foreground[y : y + height, x : x + width] > 0
            strength = float(window[spot].mean()) if spot.any() else 0.0
            rows.append(
                {
                    "xyxy": [float(x), float(y), float(x + width), float(y + height)],
                    "score": round(min(1.0, strength / SCORE_REFERENCE_DIFF), 4),
                    "class_id": 0,
                    "area": area,
                }
            )
        predictions[path.stem] = rows

        if annotate_dir is not None:
            gt_boxes = read_gt_boxes(subdataset, path.stem, names, image_size)
            reviewed = "reviewed" if path.stem in reviewed else "GT not reviewed"
            caption = (
                f"{subdataset.name}  {path.stem}  {len(rows)} det  {len(gt_boxes)} GT  ({reviewed})"
            )
            canvas = annotate(frame, difference, rows, gt_boxes, caption)
            cv2.imwrite(
                str(annotate_dir / f"{path.stem}.jpg"), canvas, [cv2.IMWRITE_JPEG_QUALITY, 92]
            )
            tiles.append(canvas)

    if annotate_dir is not None:
        write_contact_sheet(tiles, annotate_dir.parent / f"contact_{subdataset.name}.jpg")

    found = sum(len(rows) for rows in predictions.values())
    print(f"  {subdataset.name}: {len(usable)}/{len(images)} frames posed, {found} detections")
    return predictions


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("gt", type=Path, help="eval dataset root or a single sub dataset")
    parser.add_argument(
        "--recordings",
        type=Path,
        nargs="+",
        default=[Path("data/saved_recordings")],
        help="where to look for the source MCAPs named in each data.yaml",
    )
    parser.add_argument(
        "-o", "--output", type=Path, required=True, help="predictions JSON to write"
    )
    parser.add_argument(
        "--background-samples", type=int, default=160, help="recording frames in the floor median"
    )
    parser.add_argument("--raster-px-per-m", type=float, default=RASTER_PX_PER_M)
    parser.add_argument("--floor-margin-m", type=float, default=FLOOR_MARGIN_M)
    parser.add_argument("--threshold", type=int, default=35)
    parser.add_argument("--edge-tolerance", type=float, default=0.5)
    parser.add_argument("--min-area", type=int, default=400)
    parser.add_argument("--illumination", choices=("local", "global", "none"), default="local")
    parser.add_argument("--label", default="opponent", help="label to stamp on every detection")
    parser.add_argument(
        "--annotate-dir",
        type=Path,
        default=None,
        help="also write annotated frames (GT vs detections) and per-recording contact sheets here",
    )
    args = parser.parse_args()

    params = SubtractionParams(
        threshold=args.threshold,
        illumination=args.illumination,
        edge_tolerance=args.edge_tolerance,
        min_area=args.min_area,
    )

    root = args.gt
    subdatasets = (
        [root]
        if (root / "data.yaml").exists()
        else sorted(d for d in root.iterdir() if d.is_dir() and (d / "data.yaml").exists())
    )
    if not subdatasets:
        raise SystemExit(f"No data.yaml found under {root}")

    # Only frames that pass review are scored, so the annotations say which ones they are rather
    # than implying every GT box is trustworthy. Same precedence score.py uses.
    reviewed = reviewed_stems(root) or set()

    frames: dict[str, list[dict]] = {}
    for subdataset in subdatasets:
        frames.update(predict_subdataset(subdataset, list(args.recordings), params, args, reviewed))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(
            {
                "source": "background_subtraction_predict.py",
                "labels": [args.label],
                "params": {
                    "threshold": args.threshold,
                    "edge_tolerance": args.edge_tolerance,
                    "min_area": args.min_area,
                    "illumination": args.illumination,
                    "background_samples": args.background_samples,
                },
                "frames": frames,
            },
            indent=1,
        )
    )
    total = sum(len(rows) for rows in frames.values())
    print(f"Wrote {total} detections over {len(frames)} frames to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
