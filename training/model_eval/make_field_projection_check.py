#!/usr/bin/env python3
"""Draw the arena floor onto eval frames so the exported camera transforms can be eyeballed."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from auto_battlebot.eval import BACK_IDX, FRONT_IDX, FrameKey, load_gt
from auto_battlebot.perception.camera_geometry import (
    NOMINAL_FIELD_SIZE_M,
    field_to_pixels,
    height_for_label,
    load_frame_geometry,
    pixels_to_floor,
)

TILE_W, TILE_H = 640, 360
PAD = 6
LEGEND_H = 52
CAPTION_H = 42
MAX_PROJECTED_COORD_PX = 20_000
FONT = cv2.FONT_HERSHEY_SIMPLEX

FLOOR_GRID_BGR = (214, 120, 42)
NOMINAL_ARENA_BGR = (52, 104, 235)
RANGE_RING_BGR = (122, 175, 27)
INK = (238, 238, 238)
SURFACE = (26, 26, 26)

GRID_STEP_M = 0.5
GRID_EXTENT_M = 3.0
RANGE_RINGS_M = (1.0, 2.0, 3.0)
POLYLINE_SAMPLES = 64


def draw_polyline(tile, points_field, geometry, color, scale, thickness=1, closed=False):
    """Project a field-frame polyline and draw the segments that stay in front of the camera."""
    pixels = field_to_pixels(points_field, geometry) * scale
    if closed:
        pixels = np.vstack([pixels, pixels[:1]])
    for a, b in zip(pixels[:-1], pixels[1:]):
        if not (np.all(np.isfinite(a)) and np.all(np.isfinite(b))):
            continue
        if max(abs(a[0]), abs(a[1]), abs(b[0]), abs(b[1])) > MAX_PROJECTED_COORD_PX:
            continue
        cv2.line(
            tile, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), color, thickness, cv2.LINE_AA
        )


def line_points(start, end, samples=POLYLINE_SAMPLES):
    """A straight field-frame segment, sampled so perspective curvature is preserved."""
    t = np.linspace(0, 1, samples)[:, None]
    return np.asarray(start) * (1 - t) + np.asarray(end) * t


def draw_floor(tile, geometry, scale):
    """Grid, nominal arena square and range rings, all on the floor plane."""
    ticks = np.arange(-GRID_EXTENT_M, GRID_EXTENT_M + 1e-9, GRID_STEP_M)
    for value in ticks:
        draw_polyline(
            tile,
            line_points([value, -GRID_EXTENT_M, 0.0], [value, GRID_EXTENT_M, 0.0]),
            geometry,
            FLOOR_GRID_BGR,
            scale,
        )
        draw_polyline(
            tile,
            line_points([-GRID_EXTENT_M, value, 0.0], [GRID_EXTENT_M, value, 0.0]),
            geometry,
            FLOOR_GRID_BGR,
            scale,
        )

    half = NOMINAL_FIELD_SIZE_M / 2
    corners = [(-half, -half), (half, -half), (half, half), (-half, half)]
    square = np.vstack(
        [line_points([*corners[i], 0.0], [*corners[(i + 1) % 4], 0.0]) for i in range(4)]
    )
    draw_polyline(tile, square, geometry, NOMINAL_ARENA_BGR, scale, thickness=2, closed=True)

    nadir = geometry.camera_position[:2]
    angles = np.linspace(0, 2 * np.pi, 128)
    for radius in RANGE_RINGS_M:
        ring = np.stack(
            [
                nadir[0] + radius * np.cos(angles),
                nadir[1] + radius * np.sin(angles),
                np.zeros_like(angles),
            ],
            axis=1,
        )
        draw_polyline(tile, ring, geometry, RANGE_RING_BGR, scale, closed=True)


def draw_round_trip(tile, geometry, gt_boxes, gt_labels, gt_keypoints, scale):
    """Round-trip each aim point through the floor and report the worst residual."""
    worst = 0.0
    for label, keypoints in zip(gt_labels, gt_keypoints):
        if len(keypoints) <= max(FRONT_IDX, BACK_IDX):
            continue
        if keypoints[FRONT_IDX, 2] <= 0 or keypoints[BACK_IDX, 2] <= 0:
            continue
        midpoint = (keypoints[FRONT_IDX, :2] + keypoints[BACK_IDX, :2]) / 2
        floor = pixels_to_floor(midpoint, geometry, height_for_label(label))[0]
        if not np.all(np.isfinite(floor)):
            continue
        back = field_to_pixels(floor, geometry)[0]
        if not np.all(np.isfinite(back)):
            continue
        worst = max(worst, float(np.hypot(*(back - midpoint))))
        original = (midpoint * scale).astype(int)
        returned = (back * scale).astype(int)
        cv2.circle(tile, tuple(original), 7, SURFACE, -1, cv2.LINE_AA)
        cv2.circle(tile, tuple(original), 6, INK, 1, cv2.LINE_AA)
        cv2.drawMarker(tile, tuple(returned), RANGE_RING_BGR, cv2.MARKER_CROSS, 11, 2, cv2.LINE_AA)
        _ = gt_boxes
    return worst


def caption(tile, lines):
    strip = np.full((CAPTION_H, TILE_W, 3), SURFACE, dtype=np.uint8)
    for row, text in enumerate(lines):
        cv2.putText(strip, text, (6, 17 + row * 17), FONT, 0.42, INK, 1, cv2.LINE_AA)
    return np.vstack([tile, strip])


def legend(width):
    strip = np.full((LEGEND_H, width, 3), SURFACE, dtype=np.uint8)
    entries = [
        (f"nominal {NOMINAL_FIELD_SIZE_M:.2f} m arena square", NOMINAL_ARENA_BGR),
        (f"{GRID_STEP_M:g} m floor grid", FLOOR_GRID_BGR),
        ("1/2/3 m range rings + round-trip cross", RANGE_RING_BGR),
    ]
    x = 16
    for text, color in entries:
        cv2.rectangle(strip, (x, LEGEND_H // 2 - 6), (x + 22, LEGEND_H // 2 + 6), color, -1)
        cv2.putText(strip, text, (x + 30, LEGEND_H // 2 + 5), FONT, 0.48, INK, 1, cv2.LINE_AA)
        x += 30 + cv2.getTextSize(text, FONT, 0.48, 1)[0][0] + 34
    return strip


def pick_frames(gt_frames, images, count: int, method: str) -> list[FrameKey]:
    """Frames with a usable pose, dealt round-robin across recordings."""
    usable: dict[str, list[int]] = {}
    for stamp in sorted(gt_frames):
        if load_frame_geometry(images[stamp], methods=(method,)) is None:
            continue
        usable.setdefault(images[stamp].parent.parent.name, []).append(stamp)
    if not usable:
        raise SystemExit("No frames with a usable camera transform; check export_camera_transforms")

    recordings = sorted(usable)
    per_recording = max(1, count // len(recordings) + 1)
    chosen: list[int] = []
    for index in range(per_recording):
        for recording in recordings:
            stamps = usable[recording]
            step = max(1, len(stamps) // per_recording)
            position = index * step
            if position < len(stamps) and len(chosen) < count:
                chosen.append(stamps[position])
    return chosen[:count]


def build(args) -> np.ndarray:
    gt_frames, _names, images = load_gt(args.gt)
    chosen = pick_frames(gt_frames, images, args.count, args.method)
    tiles = []
    for stamp in chosen:
        geometry = load_frame_geometry(images[stamp], methods=(args.method,))
        image = cv2.imread(str(images[stamp]))
        height, width = image.shape[:2]
        scale = TILE_W / width
        tile = cv2.resize(image, (TILE_W, TILE_H), interpolation=cv2.INTER_AREA)
        draw_floor(tile, geometry, scale)
        gt_boxes, gt_labels, gt_keypoints = gt_frames[stamp]
        residual = draw_round_trip(tile, geometry, gt_boxes, gt_labels, gt_keypoints, scale)
        tiles.append(
            caption(
                tile,
                [
                    images[stamp].parent.parent.name[:58],
                    f"cam height {geometry.camera_height_m:.2f} m   method {geometry.method}"
                    f"   round-trip {residual:.3f} px",
                ],
            )
        )

    cell_h, cell_w = tiles[0].shape[:2]
    rows_n = (len(tiles) + args.cols - 1) // args.cols
    grid_w = args.cols * cell_w + (args.cols + 1) * PAD
    grid_h = rows_n * cell_h + (rows_n + 1) * PAD
    grid = np.full((grid_h, grid_w, 3), SURFACE, dtype=np.uint8)
    for i, tile in enumerate(tiles):
        r, c = divmod(i, args.cols)
        grid[
            PAD + r * (cell_h + PAD) : PAD + r * (cell_h + PAD) + cell_h,
            PAD + c * (cell_w + PAD) : PAD + c * (cell_w + PAD) + cell_w,
        ] = tile
    return np.vstack([legend(grid_w), grid])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("gt", type=Path, help="eval dataset root")
    parser.add_argument("-o", "--output", type=Path, required=True, help="output PNG path")
    parser.add_argument("--count", type=int, default=10, help="frames to draw (default 10)")
    parser.add_argument("--cols", type=int, default=2, help="grid columns (default 2)")
    parser.add_argument(
        "--method", default="exact", help="transform method to require (default: exact)"
    )
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), build(args))
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
