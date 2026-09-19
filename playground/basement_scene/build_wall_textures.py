#!/usr/bin/env python3
"""Stone-wall albedos for the basement scene, from the target frame and its ZED depth.

The box stands in a corner of the basement: a stone wall 2.5 cm behind the far rail and
another 7 cm off the right rail. A panorama at infinity cannot stand in for walls that
close, so each one becomes a vertical plane in the scene (`[[walls]]` in the spec) with this
texture on it.

For each wall:

1. Plane from depth. The ZED points in a region behind the box are fitted to a vertical
   plane; its offset is the median of the inliers. The fit also reports how far its normal
   is from the box axis, which is the check that the wall really is square to the box.
2. Orthographic warp. The wall plane is mapped to the image through the fitted pose and K,
   and the target is resampled onto a grid at `--px-per-m`, columns from `start` to `end`
   and rows from `z_top` down, the layout `synthgen.cage_spec.wall_quads` expects.
3. Seen mask from depth, not from the picture. A texel counts as seen only if the ZED put
   that pixel within `--wall-tolerance` of the plane, so the hoist in front of the far wall,
   the rails and the bracket at the right edge are left out rather than painted onto it.
   Points at or below the rail tops are dropped too: the far rail stands inside the
   tolerance of the far wall.
4. Fill. The largest wholly seen rectangle, its lighting gradient flattened, is tiled over
   the whole wall in shifted, cross-faded copies (`texture_fill.offset_tile`). Every seen
   texel is then put back on top with a feathered edge, so the fitted view sees its own
   photo and every other view sees the same stone continued. The seen texels have the
   photo's broad lighting divided out first (`SHADING_SIGMA_M`): the scene's tubes light
   the wall, and a baked hot spot would show as a bright patch against the fill.

Writes `<out>/<name>_albedo.png`, `_seen.png` (the depth mask) and `.json`, and prints the
`[[walls]]` block to paste into the spec.

Usage:
    venv/bin/python playground/basement_scene/build_wall_textures.py runs/basement_scene \\
        --out training/data/environments/meatball_basement/walls
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from auto_battlebot.perception.cage_calibration import load_cage_calibration, world_from_camera
from auto_battlebot.perception.texture_fill import largest_seen_rectangle, offset_tile

# W-frame regions whose depth points belong to each wall: above the rail tops so the box
# itself stays out, and clear of the corner so neither wall votes for the other.
FAR_REGION = {"axis": 1, "min": 0.78, "max": 1.2, "along": (-0.7, 0.7)}
RIGHT_REGION = {"axis": 0, "min": 0.82, "max": 1.3, "along": (-0.5, 0.65)}
MIN_HEIGHT_M = 0.12
# Height above the mat below which nothing counts as wall: the rail tops are at 0.088 m.
SEEN_ABOVE_M = 0.11
FEATHER_PX = 9
# Scale of the photo's lighting divided out of the seen texels, metres. The scene's own tubes
# light the wall; left in, the real hot spot shows as a bright patch against the tiled fill.
SHADING_SIGMA_M = 0.25


@dataclass(frozen=True)
class Wall:
    name: str
    start: tuple[float, float]
    end: tuple[float, float]
    z_bottom: float
    z_top: float

    @property
    def length(self) -> float:
        return float(np.hypot(self.end[0] - self.start[0], self.end[1] - self.start[1]))


def fit_offset(points_w: np.ndarray, region: dict, tolerance: float) -> tuple[float, float, int]:
    """Offset of a vertical wall along `region['axis']`, and the tilt of its fitted normal."""
    axis = region["axis"]
    other = 1 - axis
    pick = (
        (points_w[:, axis] > region["min"])
        & (points_w[:, axis] < region["max"])
        & (points_w[:, other] > region["along"][0])
        & (points_w[:, other] < region["along"][1])
        & (points_w[:, 2] > MIN_HEIGHT_M)
    )
    chosen = points_w[pick]
    if len(chosen) < 500:
        raise SystemExit(f"only {len(chosen)} depth points in the wall region {region}")
    offset = float(np.median(chosen[:, axis]))
    for _ in range(3):
        inliers = chosen[np.abs(chosen[:, axis] - offset) < tolerance]
        offset = float(np.median(inliers[:, axis]))
    centre = inliers.mean(axis=0)
    normal = np.linalg.svd(inliers - centre)[2][2]
    axis_vector = np.eye(3)[axis]
    angle = float(np.degrees(np.arccos(min(1.0, abs(float(normal @ axis_vector))))))
    return offset, angle, len(inliers)


def image_from_wall(wall: Wall, tf_cam_from_w: np.ndarray, k: np.ndarray) -> np.ndarray:
    """3x3 map from wall coordinates (s along start->end in metres, z) to image pixels."""
    start = np.array([*wall.start, 0.0])
    along = np.array([wall.end[0] - wall.start[0], wall.end[1] - wall.start[1], 0.0])
    along /= np.linalg.norm(along)
    rotation, translation = tf_cam_from_w[:3, :3], tf_cam_from_w[:3, 3]
    columns = np.c_[rotation @ along, rotation @ np.array([0.0, 0.0, 1.0]),
                    rotation @ start + translation]  # fmt: skip
    return np.asarray(k @ columns)


def texture_from_wall(wall: Wall, px_per_m: float) -> np.ndarray:
    """3x3 map from wall coordinates (s, z) to texture pixels, row 0 at `z_top`."""
    return np.array(
        [[px_per_m, 0.0, 0.0], [0.0, -px_per_m, wall.z_top * px_per_m], [0.0, 0.0, 1.0]]
    )


def flatten_seen_shading(texture: np.ndarray, seen: np.ndarray, sigma_px: float) -> np.ndarray:
    """Divide the seen texels' broad brightness out, blurring over seen texels only.

    A plain blur would drag the unseen texels' placeholder value into the estimate along
    every edge of the seen region; normalised convolution uses the seen ones alone.
    """
    lab = cv2.cvtColor(texture, cv2.COLOR_BGR2Lab).astype(np.float32)
    weight = seen.astype(np.float32)
    lum = lab[:, :, 0] * weight
    low = cv2.GaussianBlur(lum, (0, 0), sigma_px) / np.maximum(
        cv2.GaussianBlur(weight, (0, 0), sigma_px), 1e-3
    )
    mean = float(lab[:, :, 0][seen].mean())
    lab[:, :, 0] = np.where(
        seen, np.clip(lab[:, :, 0] / np.maximum(low, 1.0) * mean, 0, 255), lab[:, :, 0]
    )
    return np.asarray(cv2.cvtColor(lab.astype(np.uint8), cv2.COLOR_Lab2BGR))


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("scene_dir", type=Path)
    parser.add_argument("--clip", default="2026-04-19T17-01-18_f4560")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--px-per-m", type=float, default=512.0)
    parser.add_argument("--wall-tolerance", type=float, default=0.04, help="metres")
    parser.add_argument("--floor-drop", type=float, default=0.78, help="concrete below the mat")
    parser.add_argument("--ceiling", type=float, default=1.35, help="wall top above the mat")
    parser.add_argument(
        "--reach", type=float, default=2.6, help="how far each wall runs from the corner"
    )
    args = parser.parse_args()

    target_dir = args.scene_dir / "targets" / args.clip
    target = cv2.imread(str(target_dir / "target.png"))
    xyz = np.load(target_dir / "xyz.npy").astype(np.float64)
    intrinsics = json.loads((args.scene_dir / "camera_rect.json").read_text())
    k = np.array(
        [
            [intrinsics["fx"], 0.0, intrinsics["cx"]],
            [0.0, intrinsics["fy"], intrinsics["cy"]],
            [0.0, 0.0, 1.0],
        ]
    )
    calibration = load_cage_calibration(args.scene_dir / "poses" / f"{args.clip}.toml")
    tf_w_from_cam = world_from_camera(calibration.tf_camera_from_fieldcenter)
    tf_cam_from_w = np.linalg.inv(tf_w_from_cam)

    valid = np.isfinite(xyz).all(axis=2)
    points_w = (tf_w_from_cam[:3, :3] @ xyz[valid].T).T + tf_w_from_cam[:3, 3]
    far_y, far_angle, far_n = fit_offset(points_w, FAR_REGION, args.wall_tolerance)
    right_x, right_angle, right_n = fit_offset(points_w, RIGHT_REGION, args.wall_tolerance)
    print(f"far wall   y = {far_y:.3f} m, normal {far_angle:.1f} deg off +y, {far_n} points")
    print(f"right wall x = {right_x:.3f} m, normal {right_angle:.1f} deg off +x, {right_n} points")

    # Both run away from the shared corner, so the texture's left edge meets at the corner.
    z_bottom, z_top = -args.floor_drop, args.ceiling
    walls = [
        Wall("far", (right_x, far_y), (right_x - args.reach, far_y), z_bottom, z_top),
        Wall("right", (right_x, far_y - args.reach), (right_x, far_y), z_bottom, z_top),
    ]
    distance_to = {
        "far": lambda p: np.abs(p[..., 1] - far_y),
        "right": lambda p: np.abs(p[..., 0] - right_x),
    }
    xyz_w = np.full_like(xyz, np.nan)
    xyz_w[valid] = points_w

    args.out.mkdir(parents=True, exist_ok=True)
    blocks = []
    for wall in walls:
        size = (
            int(round(wall.length * args.px_per_m)),
            int(round((z_top - z_bottom) * args.px_per_m)),
        )
        h_img_from_tex = image_from_wall(wall, tf_cam_from_w, k) @ np.linalg.inv(
            texture_from_wall(wall, args.px_per_m)
        )
        texture = cv2.warpPerspective(
            target, h_img_from_tex, size, flags=cv2.INTER_LANCZOS4 | cv2.WARP_INVERSE_MAP
        )
        close = (distance_to[wall.name](xyz_w) < args.wall_tolerance) & (
            xyz_w[..., 2] > SEEN_ABOVE_M
        )
        on_wall = close.astype(np.uint8) * 255
        on_wall = cv2.morphologyEx(on_wall, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        seen = cv2.warpPerspective(
            on_wall, h_img_from_tex, size, flags=cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP
        )
        seen = cv2.erode(seen, np.ones((5, 5), np.uint8)) > 127
        texture = flatten_seen_shading(texture, seen, SHADING_SIGMA_M * args.px_per_m)
        row0, row1, col0, col1 = largest_seen_rectangle(seen)
        tiled = offset_tile(texture[row0:row1, col0:col1], seen.shape, seed=len(blocks))
        # The tile is flattened to its mean; bring that mean to the seen texels' own.
        tiled = np.clip(
            tiled * (texture[seen].mean(axis=0) / np.maximum(tiled.reshape(-1, 3).mean(axis=0), 1)),
            0,
            255,
        )
        weight = cv2.GaussianBlur(
            cv2.erode(seen.astype(np.float32), np.ones((FEATHER_PX, FEATHER_PX), np.uint8)),
            (2 * FEATHER_PX + 1, 2 * FEATHER_PX + 1),
            0,
        )[..., None]
        filled = (texture * weight + tiled * (1.0 - weight)).astype(np.uint8)
        stem = f"meatball_basement_{wall.name}_wall"
        cv2.imwrite(str(args.out / f"{stem}_albedo.png"), filled)
        cv2.imwrite(str(args.out / f"{stem}_seen.png"), seen.astype(np.uint8) * 255)
        (args.out / f"{stem}_albedo.json").write_text(
            json.dumps(
                {
                    "wall": wall.name,
                    "clip": args.clip,
                    "start": list(wall.start),
                    "end": list(wall.end),
                    "z_bottom": z_bottom,
                    "z_top": z_top,
                    "px_per_m": args.px_per_m,
                    "seen_fraction": float(seen.mean()),
                    "tile_rectangle_rows_cols": [row0, row1, col0, col1],
                    "layout": "columns start->end, row 0 at z_top",
                },
                indent=2,
            )
            + "\n"
        )
        print(f"{stem}: {size[0]}x{size[1]}, {seen.mean():.1%} seen")
        blocks.append(
            "[[walls]]\n"
            f'name = "{wall.name}"\n'
            f"start = [{wall.start[0]:.3f}, {wall.start[1]:.3f}]\n"
            f"end = [{wall.end[0]:.3f}, {wall.end[1]:.3f}]\n"
            f"z_bottom = {z_bottom}\n"
            f"z_top = {z_top}\n"
            f'albedo = "{args.out / f"{stem}_albedo.png"}"\n'
        )
    print("\n" + "\n".join(blocks))


if __name__ == "__main__":
    main()
