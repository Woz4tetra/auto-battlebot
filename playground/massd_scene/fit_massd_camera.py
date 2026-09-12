#!/usr/bin/env python3
"""Fit the MassDestruction broadcast camera: focal length and pose, from the floor alone.

Nothing about this camera is calibrated. The clip is a YouTube broadcast, so the focal
length has to come out of the picture with the pose. The floor gives it: the plywood is a
square seen corner-on, so its two edge families meet at two vanishing points, and a
principal point at the image centre with square pixels turns their orthogonality into one
equation for f:

    (u1 - cx)(u2 - cx) + (v1 - cy)(v2 - cy) + f^2 = 0

The vanishing points come from a RANSAC over LSD segments on the floor, which locks onto
the rails, the painted squares, the pit rim and the plywood seams: every straight line in
the arena runs along one of the two axes. With f in hand the two axis directions fix the
plane's orientation, and three floor edges (the two far ones and the near one opposite the
far-left) fix the rest: their intersections are the far corner and the right corner, and
the distance between them is the mat size, which sets the range.

The fit is exact by construction, like the NHRL three-line fit: six unknowns, six
equations, no residual to report. Judge it on `<clip>_overlay.png` and on the sanity
checks in `poses/summary.csv`.

Scale in: `--mat-size`, the plywood floor edge to edge. 2.26 m is the MassD arena, from an
8 ft (2.4384 m) wall interior less a 2x4 rail on each side, cross-checked two ways against
our own metric ZED recording of the same venue (see the report).

Usage:
    venv/bin/python playground/massd_scene/fit_massd_camera.py runs/massd_scene
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from auto_battlebot.perception.cage_calibration import (
    CageCalibration,
    camera_from_world,
    pose_summary,
    save_cage_calibration,
)

MIN_SEGMENT_PX = 60
VP_ANGLE_TOLERANCE_DEG = 1.0
RANSAC_ITERATIONS = 6000
HEIGHT_RANGE_M = (0.3, 2.5)
TILT_RANGE_DEG = (40.0, 80.0)


def line_segments(image: np.ndarray, inside: np.ndarray) -> np.ndarray:
    """LSD segments whose midpoint sits on the floor, long enough to have a direction."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detector = cv2.createLineSegmentDetector(cv2.LSD_REFINE_ADV)
    detected = detector.detect(gray)[0]
    if detected is None:
        raise SystemExit("no line segments detected")
    segments = detected.reshape(-1, 4)
    length = np.hypot(segments[:, 2] - segments[:, 0], segments[:, 3] - segments[:, 1])
    midpoints = np.round(
        np.c_[(segments[:, 0] + segments[:, 2]) / 2, (segments[:, 1] + segments[:, 3]) / 2]
    ).astype(int)
    height, width = inside.shape
    midpoints[:, 0] = np.clip(midpoints[:, 0], 0, width - 1)
    midpoints[:, 1] = np.clip(midpoints[:, 1], 0, height - 1)
    on_floor = inside[midpoints[:, 1], midpoints[:, 0]]
    return segments[(length > MIN_SEGMENT_PX) & on_floor]


def _support(segments: np.ndarray, vanishing: np.ndarray) -> np.ndarray:
    """Which segments point at `vanishing`, by the angle between segment and sight line."""
    midpoints = np.c_[(segments[:, 0] + segments[:, 2]) / 2, (segments[:, 1] + segments[:, 3]) / 2]
    to_vp = vanishing[None, :] - midpoints
    to_vp = to_vp / np.linalg.norm(to_vp, axis=1, keepdims=True)
    direction = np.c_[segments[:, 2] - segments[:, 0], segments[:, 3] - segments[:, 1]]
    direction = direction / np.linalg.norm(direction, axis=1, keepdims=True)
    angle = np.degrees(np.arccos(np.clip(np.abs((to_vp * direction).sum(1)), 0.0, 1.0)))
    return angle < VP_ANGLE_TOLERANCE_DEG


def vanishing_point(
    segments: np.ndarray, usable: np.ndarray, seed: int
) -> tuple[np.ndarray, np.ndarray]:
    """RANSAC the vanishing point that the most segment length agrees on."""
    rng = np.random.default_rng(seed)
    homogeneous = np.cross(
        np.c_[segments[:, 0], segments[:, 1], np.ones(len(segments))],
        np.c_[segments[:, 2], segments[:, 3], np.ones(len(segments))],
    )
    length = np.hypot(segments[:, 2] - segments[:, 0], segments[:, 3] - segments[:, 1])
    pool = np.where(usable)[0]
    best_weight, best_inliers, best_vp = -1.0, usable, np.zeros(2)
    for _ in range(RANSAC_ITERATIONS):
        i, j = rng.choice(pool, 2, replace=False)
        candidate = np.cross(homogeneous[i], homogeneous[j])
        if abs(candidate[2]) < 1e-9:
            continue
        point = candidate[:2] / candidate[2]
        inliers = usable & _support(segments, point)
        weight = float(length[inliers].sum())
        if weight > best_weight:
            best_weight, best_inliers, best_vp = weight, inliers, point
    return _refit_vanishing_point(segments[best_inliers], best_vp), best_inliers


def _refit_vanishing_point(segments: np.ndarray, seed: np.ndarray) -> np.ndarray:
    """Least-squares vanishing point: the point closest to every inlier's line."""
    homogeneous = np.cross(
        np.c_[segments[:, 0], segments[:, 1], np.ones(len(segments))],
        np.c_[segments[:, 2], segments[:, 3], np.ones(len(segments))],
    )
    homogeneous = homogeneous / np.linalg.norm(homogeneous[:, :2], axis=1, keepdims=True)
    a, b = homogeneous[:, :2], -homogeneous[:, 2]
    solution, *_ = np.linalg.lstsq(a, b, rcond=None)
    return np.asarray(solution) if np.all(np.isfinite(solution)) else seed


def fit_edge(points: np.ndarray, vanishing: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The line through `vanishing` that best fits `points`; returns (point, unit direction)."""
    offset = points - vanishing
    angles = np.arctan2(offset[:, 1], offset[:, 0])
    best, best_cost = angles[0], np.inf
    for start in (np.median(angles), angles.mean()):
        grid = start + np.linspace(-0.05, 0.05, 4001)
        normals = np.c_[-np.sin(grid), np.cos(grid)]
        cost = ((offset @ normals.T) ** 2).sum(axis=0)
        if cost.min() < best_cost:
            best_cost, best = cost.min(), grid[int(np.argmin(cost))]
    return vanishing, np.array([math.cos(best), math.sin(best)])


def intersect(
    first: tuple[np.ndarray, np.ndarray], second: tuple[np.ndarray, np.ndarray]
) -> np.ndarray:
    (p1, u1), (p2, u2) = first, second
    t = np.linalg.solve(np.c_[u1, -u2], p2 - p1)
    return np.asarray(p1 + t[0] * u1)


def hull_edge_points(mask: np.ndarray, margin: int = 6) -> np.ndarray:
    """Hull outline points clear of the image border, where the mat edge is really seen."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contour = max(contours, key=cv2.contourArea).reshape(-1, 2).astype(np.float64)
    height, width = mask.shape
    keep = (
        (contour[:, 0] > margin)
        & (contour[:, 0] < width - 1 - margin)
        & (contour[:, 1] > margin)
        & (contour[:, 1] < height - 1 - margin)
    )
    return contour[keep]


def split_by_family(
    points: np.ndarray, vanishing: np.ndarray, tolerance_px: float = 4.0
) -> list[np.ndarray]:
    """Group outline points into the straight runs that aim at `vanishing`."""
    runs: list[list[np.ndarray]] = []
    current: list[np.ndarray] = []
    for point in points:
        if current:
            anchor = current[0]
            direction = anchor - vanishing
            direction = direction / np.linalg.norm(direction)
            normal = np.array([-direction[1], direction[0]])
            if abs((point - anchor) @ normal) > tolerance_px and len(current) > 1:
                runs.append(np.array(current))
                current = []
        current.append(point)
    if current:
        runs.append(np.array(current))
    return [run for run in runs if len(run) > 60]


def build_pose(
    vp_along: np.ndarray,
    vp_across: np.ndarray,
    focal: float,
    principal: np.ndarray,
    corner_px: np.ndarray,
    extent_units: float,
    mat_size: float,
    inside_px: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Camera pose in W from the two floor axes, the far corner and the mat size.

    W is the Blender world frame of `auto_battlebot.perception.cage_calibration`: mat
    centre at the origin, z up, +y the mat axis pointing away from the camera.
    """
    k = np.array([[focal, 0.0, principal[0]], [0.0, focal, principal[1]], [0.0, 0.0, 1.0]])
    k_inv = np.linalg.inv(k)
    axis_a = k_inv @ np.array([*vp_along, 1.0])
    axis_a /= np.linalg.norm(axis_a)
    axis_b = k_inv @ np.array([*vp_across, 1.0])
    axis_b /= np.linalg.norm(axis_b)
    up = np.cross(axis_a, axis_b)
    up /= np.linalg.norm(up)

    corner_ray = k_inv @ np.array([*corner_px, 1.0])
    corner_ray /= np.linalg.norm(corner_ray)
    if up @ corner_ray > 0:  # up must point back toward the camera
        up = -up

    # Signs: from the far corner the mat runs toward the floor the camera actually sees.
    inside_ray = k_inv @ np.array([*inside_px, 1.0])
    inside_offset = inside_ray * ((corner_ray @ up) / (inside_ray @ up)) - corner_ray
    axis_a = axis_a * np.sign(inside_offset @ axis_a)
    axis_b = axis_b * np.sign(inside_offset @ axis_b)

    # Range: the mat is `mat_size` across, and `extent_units` across at unit range.
    scale = mat_size / extent_units
    corner_cam = corner_ray * scale
    centre_cam = corner_cam + (axis_a + axis_b) * (mat_size / 2)

    forward = np.array([0.0, 0.0, 1.0])
    forward = forward - (forward @ up) * up
    y_axis = max((axis_a, -axis_a, axis_b, -axis_b), key=lambda a: a @ forward)
    x_axis = np.cross(y_axis, up)
    rotation_world_from_camera = np.c_[x_axis, y_axis, up].T
    translation = -rotation_world_from_camera @ centre_cam
    tf_world_from_camera = np.eye(4)
    tf_world_from_camera[:3, :3] = rotation_world_from_camera
    tf_world_from_camera[:3, 3] = translation
    return tf_world_from_camera, k


def draw_overlay(
    target: np.ndarray, tf_camera_from_world: np.ndarray, k: np.ndarray, mat_size: float
) -> np.ndarray:
    out = target.copy()
    half = mat_size / 2

    def project(points_w: np.ndarray) -> np.ndarray:
        camera = (tf_camera_from_world @ np.c_[points_w, np.ones(len(points_w))].T)[:3]
        pixels = k @ camera
        return (pixels[:2] / pixels[2]).T

    corners = np.array(
        [[-half, -half, 0], [half, -half, 0], [half, half, 0], [-half, half, 0]], float
    )
    cv2.polylines(out, [np.round(project(corners)).astype(np.int32)], True, (255, 0, 255), 3)
    for value in np.linspace(-half, half, 9):
        for segment in (
            [[value, -half, 0], [value, half, 0]],
            [[-half, value, 0], [half, value, 0]],
        ):
            line = np.round(project(np.array(segment, float))).astype(int)
            cv2.line(out, tuple(line[0]), tuple(line[1]), (0, 255, 255), 1)
    return out


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("scene_dir", type=Path)
    parser.add_argument("--mat-size", type=float, default=2.26, help="plywood floor edge, metres")
    parser.add_argument("--cage-id", default="massd_resurgence6")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    poses_dir = args.scene_dir / "poses"
    poses_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, Any]] = []
    intrinsics: dict[str, Any] | None = None

    for target_dir in sorted((args.scene_dir / "targets").iterdir()):
        target_path = target_dir / "target.png"
        if not target_path.exists():
            continue
        meta = json.loads((target_dir / "meta.json").read_text())
        target = cv2.imread(str(target_path))
        mask = cv2.imread(str(target_dir / "hull_mask.png"), cv2.IMREAD_GRAYSCALE)
        height, width = target.shape[:2]
        principal = np.array([width / 2.0, height / 2.0])

        segments = line_segments(target, mask > 127)
        usable = np.ones(len(segments), bool)
        vp_a, inliers_a = vanishing_point(segments, usable, args.seed)
        vp_b, inliers_b = vanishing_point(segments, usable & ~inliers_a, args.seed + 1)

        focal_squared = -(
            (vp_a[0] - principal[0]) * (vp_b[0] - principal[0])
            + (vp_a[1] - principal[1]) * (vp_b[1] - principal[1])
        )
        if focal_squared <= 0:
            raise SystemExit(
                f"{meta['clip']}: vanishing points are not orthogonal, no focal length"
            )
        focal = math.sqrt(focal_squared)

        outline = hull_edge_points(mask)
        edges: dict[str, list[np.ndarray]] = {}
        for name, vp in (("a", vp_a), ("b", vp_b)):
            runs = split_by_family(outline, vp)
            edges[name] = sorted(runs, key=lambda run: run[:, 1].mean())
        if not edges["a"] or not edges["b"]:
            raise SystemExit(f"{meta['clip']}: could not find a mat edge for both axes")

        # The far edges are the high ones; the near edge opposite the far "a" edge sets range.
        far_a = fit_edge(edges["a"][0], vp_a)
        far_b = fit_edge(edges["b"][0], vp_b)
        near_a = fit_edge(edges["a"][-1], vp_a) if len(edges["a"]) > 1 else None
        if near_a is None:
            raise SystemExit(
                f"{meta['clip']}: only one edge along the first axis, range undetermined"
            )

        corner_far = intersect(far_a, far_b)
        corner_side = intersect(far_b, near_a)

        k_seed = np.array([[focal, 0.0, principal[0]], [0.0, focal, principal[1]], [0.0, 0.0, 1.0]])
        k_inv = np.linalg.inv(k_seed)
        axis_a = k_inv @ np.array([*vp_a, 1.0])
        axis_a /= np.linalg.norm(axis_a)
        axis_b = k_inv @ np.array([*vp_b, 1.0])
        axis_b /= np.linalg.norm(axis_b)
        up = np.cross(axis_a, axis_b)
        up /= np.linalg.norm(up)
        ray_far = k_inv @ np.array([*corner_far, 1.0])
        ray_far /= np.linalg.norm(ray_far)
        ray_side = k_inv @ np.array([*corner_side, 1.0])
        ray_side /= np.linalg.norm(ray_side)
        if up @ ray_far > 0:
            up, axis_b = -up, -axis_b
        side_offset = ray_side * ((ray_far @ up) / (ray_side @ up)) - ray_far
        extent_units = float(np.linalg.norm(side_offset))

        moments = cv2.moments((mask > 127).astype(np.uint8), binaryImage=True)
        inside_px = np.array([moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]])

        tf_world_from_camera, k = build_pose(
            vp_a, vp_b, focal, principal, corner_far, extent_units, args.mat_size, inside_px
        )
        summary = pose_summary(tf_world_from_camera)
        tf_camera_from_world = camera_from_world(tf_world_from_camera)

        checks = {
            "height_ok": HEIGHT_RANGE_M[0] < summary["height_m"] < HEIGHT_RANGE_M[1],
            "tilt_ok": TILT_RANGE_DEG[0] < summary["tilt_from_down_deg"] < TILT_RANGE_DEG[1],
            "focal_ok": 400.0 < focal < 4000.0,
            "two_edges_seen": len(edges["a"]) > 1,
        }
        row: dict[str, Any] = {
            "clip": meta["clip"],
            "event": meta.get("event_group", meta.get("event")),
            "focal_px": round(focal, 2),
            "hfov_deg": round(math.degrees(2 * math.atan(width / 2 / focal)), 2),
            "extent_units": round(extent_units, 5),
            "vp_a": [round(float(v), 1) for v in vp_a],
            "vp_b": [round(float(v), 1) for v in vp_b],
            "vp_inliers": [int(inliers_a.sum()), int(inliers_b.sum())],
        }
        row.update({key: round(value, 4) for key, value in summary.items()})
        row.update(checks)
        row["status"] = (
            "ok"
            if all(checks.values())
            else "CHECK " + ",".join(key for key, value in checks.items() if not value)
        )
        rows.append(row)
        print(
            f"{meta['clip']}: {row['status']}  f {focal:.1f} px  "
            + ", ".join(f"{key} {value}" for key, value in summary.items())
        )

        header = (
            f"MassDestruction broadcast camera for {meta['clip']}, "
            f"frame {meta.get('source_frame')}.\n"
            f"Fitted, not measured: focal {focal:.1f} px and pose solved together from the floor's "
            f"two vanishing points\nand three edges by playground/massd_scene/fit_massd_camera.py. "
            f"Mat size {args.mat_size} m is the input scale.\n"
            f"Picture is the banner-cropped {width}x{height} broadcast frame."
        )
        save_cage_calibration(
            poses_dir / f"{meta['clip']}.toml",
            CageCalibration(args.cage_id, args.mat_size, args.mat_size, tf_camera_from_world),
            header,
        )
        cv2.imwrite(
            str(poses_dir / f"{meta['clip']}_overlay.png"),
            draw_overlay(target, tf_camera_from_world, k, args.mat_size),
        )
        intrinsics = {
            "fx": focal,
            "fy": focal,
            "cx": float(principal[0]),
            "cy": float(principal[1]),
            "width": width,
            "height": height,
            "note": "fitted by playground/massd_scene/fit_massd_camera.py, banner-cropped frame",
        }

    if intrinsics is not None:
        (args.scene_dir / "camera_rect.json").write_text(json.dumps(intrinsics, indent=2) + "\n")
    if rows:
        keys: list[str] = []
        for row in rows:
            keys += [key for key in row if key not in keys]
        with (poses_dir / "summary.csv").open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=keys)
            writer.writeheader()
            writer.writerows(rows)
        print(f"summary: {poses_dir / 'summary.csv'}")


if __name__ == "__main__":
    main()
