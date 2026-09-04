#!/usr/bin/env python3
"""Gate robot detections on depth prominence: how far a pixel stands in front of its surroundings.

An earlier version of this measured height above the fitted field plane. That inherits every
error in the plane fit and the pose chain, and it measures the wrong quantity: these robots are
5 to 10 cm tall, which is the noise floor of stereo at 2 to 3 m.

Prominence is local and needs no plane. The camera looks across the arena at an angle, so a robot
hides floor that would have been much further away than the robot's own height. Comparing a pixel
to the furthest depth in a window around it recovers that occlusion depth, which is tens of
centimetres even for a low wedge, and no field frame, pose or plane fit enters the calculation.

Arms:

    rgb_only        the existing difference mask, unchanged, as the control
    depth_only      prominence alone, range-gated, nothing else
    rgb_and_depth   difference mask AND prominence
    rgb_or_depth    either; says whether AND cuts noise or signal
    rgb_and_depth_permissive
                    AND, except pixels the stereo failed to measure do not veto
    yolo_only       the deployed detector, as it ships
    yolo_and_depth  deployed detector boxes kept only when enough of the box is prominent

`yolo_and_depth` is the arm that matters most. The measured failure out of the cage is precision,
not recall: the detector invents robots on unfamiliar background. A depth check cannot invent
recall, but it can throw away boxes that have nothing standing up inside them.

Depth comes from `playground/cache_gt_depth.py`. Extract with NEURAL_PLUS.

Usage:
    python playground/cache_gt_depth.py <gt> --depth-mode NEURAL_PLUS -o <cache>
    python playground/depth_gated_subtraction.py <gt> --depth-cache <cache> \
        --engine data/models/<detector>.engine -o <preds dir>
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from auto_battlebot.background_subtraction import (
    SubtractionParams,
    find_blobs,
    subtract,
    warp_forward,
)
from auto_battlebot.camera_geometry import NOMINAL_FIELD_SIZE_M, load_frame_geometry
from auto_battlebot.floor_background import (
    SCORE_REFERENCE_DIFF,
    FloorRaster,
    build_floor_background,
    find_recording,
    read_recording,
)
from auto_battlebot.trt_yolo import TrtYoloModel

REPO = Path(__file__).resolve().parents[1]

ARMS = (
    "rgb_only",
    "depth_only",
    "rgb_and_depth",
    "rgb_or_depth",
    "rgb_and_depth_permissive",
    "yolo_only",
    "yolo_and_depth",
)

# How far in front of its surroundings a pixel must stand. An oblique view turns a 0.1 m robot
# into a much larger occlusion step, so this is not the robot's height.
DEFAULT_PROMINENCE_M = 0.20

# The window the local background depth is taken from, as a max filter. Must be wider than a
# robot in pixels or the robot supplies its own background and its prominence collapses to zero.
DEFAULT_WINDOW_PX = 75

# Beyond this is the crowd, the ceiling, or a stereo mismatch, never a robot on the arena floor.
MAX_RANGE_M = 6.0

# An occlusion step larger than this is a scene boundary (the arena wall against the room behind
# it), not a robot standing on the floor.
MAX_PROMINENCE_M = 1.5

# Fraction of a detector box that must be prominent for `yolo_and_depth` to keep it.
DEFAULT_BOX_PROMINENT_FRACTION = 0.15

# Prominence that counts as full confidence when ranking depth-only blobs.
SCORE_REFERENCE_PROMINENCE_M = 0.40


def depth_prominence(
    depth_raw: np.ndarray, image_size: tuple[int, int], window_px: int, max_range_m: float
) -> tuple[np.ndarray, np.ndarray]:
    """(prominence in metres, measured mask) at image resolution.

    The local background is a max filter over `window_px`: the furthest valid depth nearby, which
    for a robot on an open floor is the floor it is standing in front of. Prominence is that
    background minus the pixel's own depth, so it is positive for anything closer than its
    surroundings and zero on flat floor.

    Invalid depth is filled with 0 before the max filter so it can never win, and stays NaN in
    the output so unmeasured pixels are never counted as prominent.
    """
    width, height = image_size
    depth = depth_raw.astype(np.float32)
    if depth.shape != (height, width):
        depth = cv2.resize(depth, (width, height), interpolation=cv2.INTER_NEAREST)

    measured = np.isfinite(depth) & (depth > 0.0) & (depth < max_range_m)
    filled = np.where(measured, depth, np.float32(0.0))

    kernel = np.ones((window_px, window_px), np.uint8)
    background = cv2.dilate(filled, kernel)

    prominence = background - np.where(measured, depth, np.float32(np.nan))
    # A step this large is a scene boundary, not an object standing on the floor.
    prominence = np.where(prominence > MAX_PROMINENCE_M, np.float32(np.nan), prominence)
    return prominence, measured.astype(np.uint8) * 255


def close_mask(mask: np.ndarray, close_px: int) -> np.ndarray:
    """Merge fragments of one object into a single blob."""
    if close_px <= 0:
        return mask
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_px, close_px))
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def blobs_to_rows(
    mask: np.ndarray,
    min_area: int,
    difference: np.ndarray | None,
    prominence: np.ndarray | None,
) -> list[dict]:
    """Boxes from a binary mask, ranked by whatever evidence the arm actually has."""
    rows = []
    for x, y, width, height, area in find_blobs(mask, min_area):
        spot = mask[y : y + height, x : x + width] > 0
        if not spot.any():
            continue
        if difference is not None:
            window = difference[y : y + height, x : x + width]
            score = float(window[spot].mean()) / SCORE_REFERENCE_DIFF
        else:
            window = prominence[y : y + height, x : x + width]  # type: ignore[index]
            finite = window[spot & np.isfinite(window)]
            score = float(finite.mean()) / SCORE_REFERENCE_PROMINENCE_M if finite.size else 0.0
        rows.append(
            {
                "xyxy": [float(x), float(y), float(x + width), float(y + height)],
                "score": round(float(min(1.0, max(0.0, score))), 4),
                "class_id": 0,
                "area": int(area),
            }
        )
    return rows


def box_prominent_fraction(prominence: np.ndarray, box: np.ndarray, threshold: float) -> float:
    """Fraction of measured pixels inside a box that stand proud of their surroundings.

    Measured pixels are the denominator, not the whole box: a box the stereo could not see into
    should be judged on what was actually observed rather than punished for the holes.
    """
    height, width = prominence.shape
    x1 = max(0, min(int(box[0]), width - 1))
    y1 = max(0, min(int(box[1]), height - 1))
    x2 = max(x1 + 1, min(int(box[2]), width))
    y2 = max(y1 + 1, min(int(box[3]), height))
    window = prominence[y1:y2, x1:x2]
    finite = np.isfinite(window)
    if not finite.any():
        return 0.0
    return float((window[finite] > threshold).mean())


def gt_box_mask(subdataset: Path, stem: str, shape: tuple[int, int]) -> np.ndarray:
    """Boolean mask of every GT box in one frame, for the in-box vs out-of-box diagnostic."""
    height, width = shape
    mask = np.zeros(shape, bool)
    path = subdataset / "labels" / f"{stem}.txt"
    if not path.exists():
        return mask
    for line in path.read_text().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        cx, cy, box_w, box_h = (float(v) for v in parts[1:5])
        x1 = max(0, int((cx - box_w / 2) * width))
        y1 = max(0, int((cy - box_h / 2) * height))
        x2 = max(0, int((cx + box_w / 2) * width))
        y2 = max(0, int((cy + box_h / 2) * height))
        mask[y1:y2, x1:x2] = True
    return mask


def detector_rows(
    model: TrtYoloModel, frame: np.ndarray, prominence: np.ndarray, args: argparse.Namespace
) -> tuple[list[dict], list[dict]]:
    """(all detector boxes, boxes surviving the prominence gate).

    The prominent fraction is carried on every ungated row so the gate can be swept offline
    without re-running inference and the floor median for each threshold.
    """
    kept, gated = [], []
    for box, score, _cls, _kp in model.infer(frame):
        row = {
            "xyxy": [float(v) for v in np.asarray(box, dtype=np.float64)],
            "score": round(float(score), 4),
            "class_id": 0,
        }
        fraction = box_prominent_fraction(
            prominence, np.asarray(box, dtype=np.float64), args.prominence
        )
        kept.append({**row, "prom_fraction": round(fraction, 4)})
        if fraction >= args.box_fraction:
            gated.append(row)
    return kept, gated


def predict_subdataset(
    subdataset: Path,
    depth_cache: Path,
    roots: list[Path],
    params: SubtractionParams,
    model: TrtYoloModel | None,
    args: argparse.Namespace,
) -> tuple[dict[str, dict[str, list[dict]]], dict]:
    """Predictions per arm for one sub dataset, plus depth diagnostics."""
    cache_path = depth_cache / f"{subdataset.name}.npz"
    if not cache_path.exists():
        print(f"  {subdataset.name}: no depth cache, skipped")
        return {}, {}

    data = yaml.safe_load((subdataset / "data.yaml").read_text())
    recording = read_recording(find_recording(str(data["source_mcap"]), roots))
    depth_store = np.load(cache_path)

    images = sorted((subdataset / "images").glob("*.png"))
    geometries = {path: load_frame_geometry(path) for path in images}
    usable = [
        path
        for path in images
        if geometries[path] is not None
        and path.stem in depth_store.files
        and (subdataset / "labels" / f"{path.stem}.txt").exists()
    ]
    if not usable:
        print(f"  {subdataset.name}: no frames with both pose and depth, skipped")
        return {}, {}

    first = geometries[usable[0]]
    intrinsics = (first.fx, first.fy, first.cx, first.cy)  # type: ignore[union-attr]
    sample = cv2.imread(str(usable[0]))
    image_size = (sample.shape[1], sample.shape[0])

    raster = FloorRaster(NOMINAL_FIELD_SIZE_M, args.raster_px_per_m, args.floor_margin_m)
    background, coverage = build_floor_background(
        recording, raster, args.background_samples, intrinsics
    )
    compare_mask = cv2.bitwise_and(raster.floor_mask, coverage)

    out: dict[str, dict[str, list[dict]]] = {arm: {} for arm in ARMS}
    in_box_prominent: list[float] = []
    out_box_prominent: list[float] = []

    for path in usable:
        geometry = geometries[path]
        homography = raster.image_from_raster(geometry)
        frame = cv2.imread(str(path))
        if frame is None:
            for arm in ARMS:
                out[arm][path.stem] = []
            continue

        warped_background = warp_forward(background, homography, image_size)
        valid = warp_forward(compare_mask, homography, image_size, nearest=True)
        valid = cv2.bitwise_and(valid, raster.in_front_mask(homography, image_size))
        valid = cv2.erode(valid, np.ones((11, 11), np.uint8))
        difference, rgb_fg = subtract(frame, warped_background, valid, params)

        # No plane, no pose: prominence is computed in the depth image alone.
        prominence, measured = depth_prominence(
            depth_store[path.stem], image_size, args.window_px, args.max_range_m
        )
        depth_fg = np.where(
            np.isfinite(prominence) & (prominence > args.prominence), 255, 0
        ).astype(np.uint8)

        unmeasured = cv2.bitwise_not(measured)
        masks = {
            "rgb_only": rgb_fg,
            "depth_only": depth_fg,
            "rgb_and_depth": cv2.bitwise_and(rgb_fg, depth_fg),
            "rgb_or_depth": cv2.bitwise_or(rgb_fg, depth_fg),
            "rgb_and_depth_permissive": cv2.bitwise_and(
                rgb_fg, cv2.bitwise_or(depth_fg, unmeasured)
            ),
        }
        for arm, mask in masks.items():
            use_difference = arm != "depth_only"
            out[arm][path.stem] = blobs_to_rows(
                close_mask(mask, args.close_px),
                params.min_area,
                difference if use_difference else None,
                None if use_difference else prominence,
            )

        if model is not None:
            kept, gated = detector_rows(model, frame, prominence, args)
            out["yolo_only"][path.stem] = kept
            out["yolo_and_depth"][path.stem] = gated

        # Diagnostic: is the prominent fraction actually higher inside GT boxes than outside?
        gt_mask = gt_box_mask(subdataset, path.stem, prominence.shape)
        finite = np.isfinite(prominence)
        fires = finite & (prominence > args.prominence)
        if (gt_mask & finite).any():
            in_box_prominent.append(float(fires[gt_mask & finite].mean()))
        if (~gt_mask & finite).any():
            out_box_prominent.append(float(fires[~gt_mask & finite].mean()))

    counts = {arm: sum(len(v) for v in out[arm].values()) for arm in ARMS}
    diagnostic = {
        "frames": len(usable),
        "prominence_threshold_m": args.prominence,
        "window_px": args.window_px,
        "fires_inside_gt_boxes": round(float(np.mean(in_box_prominent)), 4)
        if in_box_prominent
        else None,
        "fires_outside_gt_boxes": round(float(np.mean(out_box_prominent)), 4)
        if out_box_prominent
        else None,
        "detections": counts,
    }
    print(f"  {subdataset.name}: {len(usable)} frames, {counts}")
    print(
        f"    prominence fires: inside GT {diagnostic['fires_inside_gt_boxes']}"
        f"  outside {diagnostic['fires_outside_gt_boxes']}"
    )
    return out, diagnostic


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("gt", type=Path, help="eval dataset root")
    parser.add_argument("--depth-cache", type=Path, required=True)
    parser.add_argument(
        "--recordings", type=Path, nargs="+", default=[Path("data/saved_recordings")]
    )
    parser.add_argument("-o", "--output", type=Path, required=True, help="dir for per-arm JSON")
    parser.add_argument("--engine", default=None, help="detector engine for the yolo_* arms")
    parser.add_argument("--engine-conf", type=float, default=0.6)
    parser.add_argument("--background-samples", type=int, default=160)
    parser.add_argument("--raster-px-per-m", type=float, default=400.0)
    parser.add_argument("--floor-margin-m", type=float, default=0.06)
    parser.add_argument("--threshold", type=int, default=35)
    parser.add_argument("--edge-tolerance", type=float, default=0.5)
    parser.add_argument("--min-area", type=int, default=400)
    parser.add_argument("--illumination", choices=("local", "global", "none"), default="local")
    parser.add_argument("--prominence", type=float, default=DEFAULT_PROMINENCE_M)
    parser.add_argument("--window-px", type=int, default=DEFAULT_WINDOW_PX)
    parser.add_argument("--max-range-m", type=float, default=MAX_RANGE_M)
    parser.add_argument("--box-fraction", type=float, default=DEFAULT_BOX_PROMINENT_FRACTION)
    parser.add_argument("--close-px", type=int, default=0)
    parser.add_argument("--suffix", default="", help="appended to each arm's filename")
    args = parser.parse_args()

    params = SubtractionParams(
        threshold=args.threshold,
        edge_tolerance=args.edge_tolerance,
        min_area=args.min_area,
        illumination=args.illumination,
    )
    model = (
        TrtYoloModel(args.engine, conf_threshold=args.engine_conf, num_classes=2)
        if args.engine
        else None
    )

    subdatasets = sorted(d for d in args.gt.iterdir() if (d / "data.yaml").exists())
    merged: dict[str, dict[str, list[dict]]] = {arm: {} for arm in ARMS}
    diagnostics: dict[str, dict] = {}

    for subdataset in subdatasets:
        per_arm, diagnostic = predict_subdataset(
            subdataset, args.depth_cache, list(args.recordings), params, model, args
        )
        if diagnostic:
            diagnostics[subdataset.name] = diagnostic
        for arm, frames in per_arm.items():
            merged[arm].update(frames)

    args.output.mkdir(parents=True, exist_ok=True)
    for arm in ARMS:
        if not merged[arm]:
            continue
        payload = {"labels": ["opponent"], "frames": merged[arm]}
        (args.output / f"{arm}{args.suffix}.json").write_text(json.dumps(payload))
    (args.output / f"diagnostics{args.suffix}.json").write_text(json.dumps(diagnostics, indent=2))
    print(f"Wrote arms to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
