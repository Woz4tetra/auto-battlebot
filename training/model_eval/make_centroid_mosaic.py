#!/usr/bin/env python3
"""Render a grid of robot crops showing where each candidate position estimate lands.

One tile per matched robot, cropped tight so the estimates are actually separable: the
seg model's mask and predicted box, the hand-labeled GT box, and the four candidate
positions those produce (mask centroid, predicted box center, GT box center, keypoint
front/back midpoint). The figure for `mask_centroid_vs_box_2026-08-03.md`, where the
numbers say every box-derived position coincides while all of them miss the aim point.

Tiles are picked to span the aim-point error rather than to flatter it: examples are
drawn from across the distribution of box-center-to-midpoint distance, spread over
classes and recordings, with a minimum on-screen size so the crop stays legible.

Usage:
    python training/model_eval/make_centroid_mosaic.py training/data/nhrl_keypoints_eval_test \
        --seg-weights data/models_v2/yolo26n-seg_nhrl_robots_2026-04-27.pt \
        --seg-labels opponent,opponent,house_bot,mr_stabs_mk2,mrs_buff_mk3 \
        --taxonomy training/model_eval/taxonomy_merged.yaml \
        -o docs/experiments/perception_performance/assets/2026-08-03_mask_centroid/mosaic.png
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
from mask_centroid_vs_box import (
    box_centers,
    detect,
    gt_keep_indices,
    gt_matches,
    keypoint_midpoints,
    longer_sides,
    mask_centroid,
)
from score import BACK_IDX, FRONT_IDX, Taxonomy, load_gt

TILE = 380
PAD = 8
LEGEND_H = 54
CAPTION_H = 62
FONT = cv2.FONT_HERSHEY_SIMPLEX

# Categorical slots 1-3 of the validated default palette, as BGR. One hue per source of
# the estimate, not per mark: blue is the mask, orange is a bounding box, aqua is the
# hand-labeled keypoints. The predicted and ground-truth boxes share the orange because
# they are the same kind of thing; solid vs dashed and filled vs hollow separate them,
# which keeps the figure inside the three slots that validate on the all-pairs list.
# Every estimate also carries a distinct marker shape, so identity is never color alone.
CENTROID_BGR = (214, 120, 42)  # #2a78d6 blue
BOXCENTER_BGR = (52, 104, 235)  # #eb6834 orange
AIMPOINT_BGR = (122, 175, 27)  # #1baf7a aqua
INK = (238, 238, 238)
SURFACE = (26, 26, 26)

CROP_MARGIN = 1.9  # crop side, as a multiple of the robot's longer box side
MIN_BOX_PX = 70  # skip robots too small to read once cropped
EDGE_PX = 3.0  # a GT box within this of the frame border counts as truncated


def draw_marker(tile: np.ndarray, point: np.ndarray, color, shape: str) -> None:
    """A filled marker with a surface-colored ring so it reads over any background.

    Sizes descend square > diamond > circle, and the tiles draw them in that order, so
    a marker never fully hides one underneath it. That case is the common one here: the
    centroid and the box center usually land on the same pixel."""
    if not np.all(np.isfinite(point)):
        return
    x, y = int(round(point[0])), int(round(point[1]))
    if shape == "square":
        cv2.rectangle(tile, (x - 11, y - 11), (x + 11, y + 11), SURFACE, -1)
        cv2.rectangle(tile, (x - 9, y - 9), (x + 9, y + 9), color, -1)
    elif shape == "hollow_square":
        # Wider than the filled square so it still reads as a ring when the two coincide,
        # which is the common case.
        cv2.rectangle(tile, (x - 14, y - 14), (x + 14, y + 14), SURFACE, 4)
        cv2.rectangle(tile, (x - 14, y - 14), (x + 14, y + 14), color, 2)
    elif shape == "diamond":
        for size, fill in ((9, SURFACE), (7, color)):
            pts = np.array([[x, y - size], [x + size, y], [x, y + size], [x - size, y]])
            cv2.fillConvexPoly(tile, pts, fill, cv2.LINE_AA)
    else:  # circle
        cv2.circle(tile, (x, y), 6, SURFACE, -1, cv2.LINE_AA)
        cv2.circle(tile, (x, y), 4, color, -1, cv2.LINE_AA)


def draw_dashed_rect(
    tile: np.ndarray, p1: np.ndarray, p2: np.ndarray, color, dash: int = 9
) -> None:
    """Dashed rectangle, marking the hand-labeled box against the solid predicted one."""
    x1, y1 = int(p1[0]), int(p1[1])
    x2, y2 = int(p2[0]), int(p2[1])
    for a, b in (
        ((x1, y1), (x2, y1)),
        ((x2, y1), (x2, y2)),
        ((x2, y2), (x1, y2)),
        ((x1, y2), (x1, y1)),
    ):
        length = int(np.hypot(b[0] - a[0], b[1] - a[1]))
        if length == 0:
            continue
        for start in range(0, length, dash * 2):
            end = min(start + dash, length)
            sa = (
                int(a[0] + (b[0] - a[0]) * start / length),
                int(a[1] + (b[1] - a[1]) * start / length),
            )
            sb = (
                int(a[0] + (b[0] - a[0]) * end / length),
                int(a[1] + (b[1] - a[1]) * end / length),
            )
            cv2.line(tile, sa, sb, color, 2, cv2.LINE_AA)


def draw_tile(
    image: np.ndarray,
    box: np.ndarray,
    gt_box: np.ndarray,
    mask: np.ndarray | None,
    centroid: np.ndarray,
    center: np.ndarray,
    gt_center: np.ndarray,
    keypoints: np.ndarray,
    midpoint: np.ndarray,
) -> np.ndarray:
    """Crop around one robot and overlay the mask, both boxes and the four position estimates."""
    # Frame the union of the predicted and GT boxes, so a badly localized detection does
    # not push its GT box out of the crop.
    union = np.array(
        [
            min(box[0], gt_box[0]),
            min(box[1], gt_box[1]),
            max(box[2], gt_box[2]),
            max(box[3], gt_box[3]),
        ]
    )
    side = max(union[2] - union[0], union[3] - union[1]) * CROP_MARGIN
    cx, cy = (union[0] + union[2]) / 2, (union[1] + union[3]) / 2
    x1 = int(round(cx - side / 2))
    y1 = int(round(cy - side / 2))
    size = int(round(side))
    height, width = image.shape[:2]

    # Pad rather than clamp, so the robot stays centered when it sits near an edge.
    crop = np.full((size, size, 3), SURFACE, dtype=np.uint8)
    sx1, sy1 = max(0, x1), max(0, y1)
    sx2, sy2 = min(width, x1 + size), min(height, y1 + size)
    crop[sy1 - y1 : sy2 - y1, sx1 - x1 : sx2 - x1] = image[sy1:sy2, sx1:sx2]

    if mask is not None:
        sub = np.zeros((size, size), dtype=bool)
        sub[sy1 - y1 : sy2 - y1, sx1 - x1 : sx2 - x1] = mask[sy1:sy2, sx1:sx2]
        tint = np.full_like(crop, CENTROID_BGR, dtype=np.uint8)
        crop[sub] = cv2.addWeighted(crop, 0.62, tint, 0.38, 0)[sub]
        contours, _ = cv2.findContours(
            sub.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        cv2.drawContours(crop, contours, -1, CENTROID_BGR, 2, cv2.LINE_AA)

    scale = TILE / size
    tile = cv2.resize(crop, (TILE, TILE), interpolation=cv2.INTER_CUBIC)
    origin = np.array([x1, y1], dtype=np.float64)

    def to_tile(point: np.ndarray) -> np.ndarray:
        return (point - origin) * scale

    b1 = to_tile(box[:2])
    b2 = to_tile(box[2:])
    cv2.rectangle(
        tile,
        (int(b1[0]), int(b1[1])),
        (int(b2[0]), int(b2[1])),
        BOXCENTER_BGR,
        2,
        cv2.LINE_AA,
    )
    draw_dashed_rect(tile, to_tile(gt_box[:2]), to_tile(gt_box[2:]), BOXCENTER_BGR)

    if len(keypoints) > max(FRONT_IDX, BACK_IDX):
        front = to_tile(keypoints[FRONT_IDX, :2])
        back = to_tile(keypoints[BACK_IDX, :2])
        for color, thickness in ((SURFACE, 5), (AIMPOINT_BGR, 2)):
            cv2.line(
                tile,
                (int(front[0]), int(front[1])),
                (int(back[0]), int(back[1])),
                color,
                thickness,
                cv2.LINE_AA,
            )
        for point in (front, back):
            cv2.circle(tile, (int(point[0]), int(point[1])), 4, AIMPOINT_BGR, 1, cv2.LINE_AA)

    # Largest first: a coincident centroid stays visible on top of the box centers.
    draw_marker(tile, to_tile(gt_center), BOXCENTER_BGR, "hollow_square")
    draw_marker(tile, to_tile(center), BOXCENTER_BGR, "square")
    draw_marker(tile, to_tile(midpoint), AIMPOINT_BGR, "diamond")
    draw_marker(tile, to_tile(centroid), CENTROID_BGR, "circle")
    return tile


def caption_tile(tile: np.ndarray, lines: list[str]) -> np.ndarray:
    """Stack a caption strip under a tile, shrinking the text if it would overrun."""
    strip = np.full((CAPTION_H, TILE, 3), SURFACE, dtype=np.uint8)
    for row, text in enumerate(lines):
        scale = 0.46
        while scale > 0.28 and cv2.getTextSize(text, FONT, scale, 1)[0][0] > TILE - 12:
            scale -= 0.02
        cv2.putText(strip, text, (6, 18 + row * 19), FONT, scale, INK, 1, cv2.LINE_AA)
    return np.vstack([tile, strip])


def legend_strip(width: int) -> np.ndarray:
    """Marker key, drawn with the same shapes the tiles use."""
    strip = np.full((LEGEND_H, width, 3), SURFACE, dtype=np.uint8)
    entries = [
        ("mask centroid", CENTROID_BGR, "circle"),
        ("predicted box center (solid box)", BOXCENTER_BGR, "square"),
        ("GT box center (dashed box)", BOXCENTER_BGR, "hollow_square"),
        ("keypoint midpoint (aim point)", AIMPOINT_BGR, "diamond"),
    ]
    x = 16
    for text, color, shape in entries:
        draw_marker(strip, np.array([x + 8.0, LEGEND_H / 2]), color, shape)
        cv2.putText(strip, text, (x + 24, int(LEGEND_H / 2) + 5), FONT, 0.5, INK, 1, cv2.LINE_AA)
        x += 34 + cv2.getTextSize(text, FONT, 0.5, 1)[0][0] + 30
    return strip


def is_truncated(box: np.ndarray, keypoints: np.ndarray, width: int, height: int) -> bool:
    """Whether the robot runs off the frame, which makes it a misleading illustration.

    The labeler clips the box at the image border but places keypoints at the chassis
    ends even when those fall outside the frame. The resulting box-center-to-midpoint
    gap is an artifact of the clipping, not the geometry this figure is about. It moves
    the medians in the report by under 0.005, but it dominates the extreme tail, so
    those robots are excluded here."""
    if (
        box[0] <= EDGE_PX
        or box[1] <= EDGE_PX
        or box[2] >= width - EDGE_PX
        or box[3] >= height - EDGE_PX
    ):
        return True
    return any(
        not (0 <= keypoints[i, 0] <= width and 0 <= keypoints[i, 1] <= height)
        for i in (FRONT_IDX, BACK_IDX)
        if len(keypoints) > i
    )


def gather(args: argparse.Namespace) -> list[dict]:
    """Every GT robot the seg model matched, with all three estimates in image coords."""
    from ultralytics import YOLO

    gt_frames, names, images = load_gt(args.gt)
    taxonomy = Taxonomy(args.taxonomy)
    print(f"GT: {len(gt_frames)} frames, classes: {names}")
    seg_labels = [label.strip() for label in args.seg_labels.split(",")]
    seg_model = YOLO(str(args.seg_weights))

    candidates = []
    for stamp, (gt_boxes_all, gt_labels_all, gt_kps_all) in gt_frames.items():
        image = cv2.imread(str(images[stamp]))
        if image is None:
            raise SystemExit(f"Failed to read image {images[stamp]}")
        gt_idx = gt_keep_indices(gt_labels_all, taxonomy)
        gt_boxes = gt_boxes_all[gt_idx]
        gt_labels = [gt_labels_all[i] for i in gt_idx]
        gt_kps = [gt_kps_all[i] for i in gt_idx]
        gt_mids = keypoint_midpoints(gt_kps)
        gt_sides = longer_sides(gt_boxes)
        gt_centers = box_centers(gt_boxes)

        boxes, scores, labels, masks = detect(
            seg_model, image, seg_labels, taxonomy, args, want_masks=True
        )
        centers = box_centers(boxes)
        pairs = gt_matches(gt_boxes, gt_labels, boxes, labels, scores, args.iou)
        height, width = image.shape[:2]
        for g, p in pairs.items():
            if masks is None or p >= len(masks) or not np.isfinite(gt_mids[g, 0]):
                continue
            if gt_sides[g] < MIN_BOX_PX:
                continue
            if is_truncated(gt_boxes[g], gt_kps[g], width, height):
                continue
            mx, my, _ = mask_centroid(masks[p])
            if not np.isfinite(mx):
                continue
            centroid = np.array([mx, my])
            gt_center = gt_centers[g]
            candidates.append(
                {
                    "stamp": stamp,
                    "recording": images[stamp].parent.parent.name,
                    "image_path": images[stamp],
                    "label": gt_labels[g],
                    "box": boxes[p],
                    "gt_box": gt_boxes[g],
                    "mask": masks[p],
                    "centroid": centroid,
                    "center": centers[p],
                    "gt_center": gt_center,
                    "keypoints": gt_kps[g],
                    "midpoint": gt_mids[g],
                    "side": gt_sides[g],
                    "centroid_to_center": float(np.hypot(*(centroid - centers[p]))) / gt_sides[g],
                    "center_to_aim": float(np.hypot(*(centers[p] - gt_mids[g]))) / gt_sides[g],
                    "gt_center_to_aim": float(np.hypot(*(gt_center - gt_mids[g]))) / gt_sides[g],
                    "center_to_gt_center": float(np.hypot(*(centers[p] - gt_center))) / gt_sides[g],
                }
            )
    return candidates


def pick(candidates: list[dict], count: int) -> list[dict]:
    """Spread the picks over the aim-point error, across classes and recordings.

    Walks the candidates ordered by box-center-to-aim-point distance and takes evenly
    spaced quantiles, skipping a pick when its class or recording has already been used
    more than its share. That keeps one crowded recording or one large class from
    filling the grid while still showing the full range of the error."""
    if not candidates:
        raise SystemExit("No matched robots met the size threshold; lower MIN_BOX_PX")
    ordered = sorted(candidates, key=lambda c: c["center_to_aim"])
    per_class = max(1, count // max(1, len({c["label"] for c in ordered})) + 1)
    per_recording = max(1, count // 3)

    taken: set[int] = set()
    class_count: dict[str, int] = {}
    recording_count: dict[str, int] = {}

    def accept(i: int) -> None:
        taken.add(i)
        label, recording = ordered[i]["label"], ordered[i]["recording"]
        class_count[label] = class_count.get(label, 0) + 1
        recording_count[recording] = recording_count.get(recording, 0) + 1

    # One target per tile, evenly spaced over the whole error range. Searching outward
    # from each target keeps a skipped pick near the quantile it was meant to represent.
    for quantile in np.linspace(0, 1, count):
        target = min(len(ordered) - 1, int(round(quantile * (len(ordered) - 1))))
        order = sorted(range(len(ordered)), key=lambda i: abs(i - target))
        fallback = next((i for i in order if i not in taken), None)
        for i in order:
            item = ordered[i]
            if i in taken:
                continue
            if class_count.get(item["label"], 0) >= per_class:
                continue
            if recording_count.get(item["recording"], 0) >= per_recording:
                continue
            accept(i)
            break
        else:  # every remaining candidate is capped out; keep the quantile anyway
            if fallback is not None:
                accept(fallback)
    return [ordered[i] for i in sorted(taken)]


def build(chosen: list[dict], cols: int) -> np.ndarray:
    tiles = []
    for item in chosen:
        image = cv2.imread(str(item["image_path"]))
        tile = draw_tile(
            image,
            item["box"],
            item["gt_box"],
            item["mask"],
            item["centroid"],
            item["center"],
            item["gt_center"],
            item["keypoints"],
            item["midpoint"],
        )
        tiles.append(
            caption_tile(
                tile,
                [
                    item["label"],
                    f"centroid->box {item['centroid_to_center']:.3f}"
                    f"    box->GT box {item['center_to_gt_center']:.3f}",
                    f"box->aim {item['center_to_aim']:.3f}"
                    f"    GT box->aim {item['gt_center_to_aim']:.3f}",
                ],
            )
        )

    rows_n = (len(tiles) + cols - 1) // cols
    cell_h, cell_w = tiles[0].shape[:2]
    grid_w = cols * cell_w + (cols + 1) * PAD
    grid_h = rows_n * cell_h + (rows_n + 1) * PAD
    grid = np.full((grid_h, grid_w, 3), SURFACE, dtype=np.uint8)
    for i, tile in enumerate(tiles):
        r, c = divmod(i, cols)
        y = PAD + r * (cell_h + PAD)
        x = PAD + c * (cell_w + PAD)
        grid[y : y + cell_h, x : x + cell_w] = tile
    return np.vstack([legend_strip(grid_w), grid])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("gt", type=Path, help="eval dataset root (the dir score.py is given)")
    parser.add_argument("--seg-weights", type=Path, required=True, help="YOLO-seg .pt weights")
    parser.add_argument("--seg-labels", required=True, help="GT label per seg class index")
    parser.add_argument("--taxonomy", type=Path, help="label -> archetype mapping yaml")
    parser.add_argument("-o", "--output", type=Path, required=True, help="output PNG path")
    parser.add_argument("--cols", type=int, default=4, help="grid columns (default 4)")
    parser.add_argument("--count", type=int, default=12, help="tiles to draw (default 12)")
    parser.add_argument("--iou", type=float, default=0.5, help="IoU match threshold")
    parser.add_argument("--conf", type=float, default=0.5, help="inference confidence threshold")
    parser.add_argument("--nms-iou", type=float, default=0.45, help="inference NMS IoU threshold")
    parser.add_argument("--imgsz", type=int, default=640, help="inference image size")
    args = parser.parse_args()

    candidates = gather(args)
    chosen = pick(candidates, args.count)
    print(f"{len(candidates)} matched robots, drawing {len(chosen)}:")
    for item in chosen:
        print(
            f"  {item['label']:<13} centroid->box {item['centroid_to_center']:.3f}"
            f"  box->aim {item['center_to_aim']:.3f}  ({item['recording'][:28]})"
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), build(chosen, args.cols))
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
