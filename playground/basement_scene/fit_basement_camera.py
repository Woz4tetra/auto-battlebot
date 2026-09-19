#!/usr/bin/env python3
"""Fit the basement drive-test box camera from ZED depth and the rail edges, K known.

The box is a 5 ft plywood floor inside 2x4 rails stood on edge, pushed into a corner of the
basement's stone walls. The camera is our own ZED on a stand beside it, so unlike the MassD
broadcast the intrinsics are known (`camera_rect.json`, read from the SVO) and the SVO has
metric depth. Two steps:

1. Floor plane from depth. RANSAC over the XYZ of the plywood-coloured pixels, refined by
   SVD. This fixes camera height and tilt without assuming anything about the box.
2. Box centre and yaw in that plane from the image. The rails give sharp edges: the floor
   meeting the left, right and far rails, and the inner and outer top edges of all four.
   Starting from the rail-top points in the depth, (x, y, yaw) is moved until those edges,
   projected through K, sit on the Canny edges of the target (mean clipped distance). Depth
   alone puts the right rail 5 cm off: the camera looks straight at its inner face and the
   stereo smears it.

Box size is an input (`--mat-size`, `--rail-height`, `--rail-thickness`), not a fit: the
depth measured 1.515 x 1.530 m inside the rails and 0.0875 m rail height, which is a 5 ft
square and a 3.5 in 2x4 to within the stereo's noise.

Reads `<scene>/targets/<clip>/{target.png, xyz.npy, meta.json}` and `<scene>/camera_rect.json`.
Writes `<scene>/poses/<clip>.toml` (CageCalibration, W frame: floor centre, z up, +y away
from the camera), `<scene>/poses/<clip>_overlay.png`, `<scene>/poses/summary.csv`, and
`<scene>/targets/<clip>/hull_mask.png`: the floor the camera sees, rails cut out.

Usage:
    venv/bin/python playground/basement_scene/fit_basement_camera.py runs/basement_scene
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from scipy.optimize import minimize

from auto_battlebot.perception.cage_calibration import (
    CageCalibration,
    camera_from_world,
    pose_summary,
    save_cage_calibration,
)

MAT_SIZE_M = 1.524  # 5 ft
RAIL_HEIGHT_M = 0.0889  # 2x4 on edge, 3.5 in
RAIL_THICKNESS_M = 0.0381  # 1.5 in
PLANE_INLIER_M = 0.005
RAIL_TOP_BAND_M = (0.078, 0.100)
EDGE_CLIP_PX = 15.0
# Plywood in HSV (OpenCV ranges): the pixels the floor plane is fitted to.
PLYWOOD_HSV = ((8, 40, 120), (30, 200, 255))


@dataclass(frozen=True)
class Box:
    size: float
    rail_height: float
    rail_thickness: float


@dataclass(frozen=True)
class Plane:
    """The floor in the camera frame: `origin` on it, `rotation` columns x, y, up."""

    origin: np.ndarray
    rotation: np.ndarray
    rms_m: float
    inliers: int


def floor_pixels(target: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(target, cv2.COLOR_BGR2HSV)
    plywood = cv2.inRange(hsv, *PLYWOOD_HSV)
    plywood = cv2.morphologyEx(plywood, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(plywood)
    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    floor = (labels == largest).astype(np.uint8)
    return cv2.erode(floor, np.ones((25, 25), np.uint8)) > 0


def fit_plane(xyz: np.ndarray, floor: np.ndarray, seed: int) -> Plane:
    points = xyz[floor]
    points = points[np.isfinite(points).all(axis=1)]
    rng = np.random.default_rng(seed)
    best = np.zeros(len(points), bool)
    for _ in range(2000):
        sample = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        if np.linalg.norm(normal) < 1e-9:
            continue
        normal /= np.linalg.norm(normal)
        inliers = np.abs((points - sample[0]) @ normal) < PLANE_INLIER_M
        if inliers.sum() > best.sum():
            best = inliers
    fitted = points[best]
    origin = fitted.mean(axis=0)
    normal = np.linalg.svd(fitted - origin)[2][2]
    if normal @ -origin < 0:  # up points back toward the camera
        normal = -normal
    x_axis = np.array([1.0, 0.0, 0.0]) - normal[0] * normal
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(normal, x_axis)
    if y_axis[2] < 0:  # +y runs away from the camera
        x_axis, y_axis = -x_axis, -y_axis
    rms = float(np.sqrt((((fitted - origin) @ normal) ** 2).mean()))
    return Plane(origin, np.c_[x_axis, y_axis, normal], rms, int(best.sum()))


def rail_top_start(xyz: np.ndarray, plane: Plane) -> np.ndarray:
    """(x, y, yaw) of the box from the depth points at rail-top height: the fit's seed."""
    valid = np.isfinite(xyz).all(axis=2)
    local = (xyz[valid] - plane.origin) @ plane.rotation
    band = (local[:, 2] > RAIL_TOP_BAND_M[0]) & (local[:, 2] < RAIL_TOP_BAND_M[1])
    points = local[band, :2]
    best_score, best_yaw = -1.0, 0.0
    for yaw in np.radians(np.arange(-10.0, 10.0, 0.05)):
        along = points @ np.array([math.cos(yaw), math.sin(yaw)])
        across = points @ np.array([-math.sin(yaw), math.cos(yaw)])
        score = sum(
            float(np.sort(np.histogram(v, bins=np.arange(v.min(), v.max(), 0.005))[0])[-2:].sum())
            for v in (along, across)
        )
        if score > best_score:
            best_score, best_yaw = score, float(yaw)
    along = points @ np.array([math.cos(best_yaw), math.sin(best_yaw)])
    across = points @ np.array([-math.sin(best_yaw), math.cos(best_yaw)])
    centre_rot = np.array([_mid_between_peaks(along), _mid_between_peaks(across)])
    rotation = np.array(
        [[math.cos(best_yaw), -math.sin(best_yaw)], [math.sin(best_yaw), math.cos(best_yaw)]]
    )
    return np.array([*(rotation @ centre_rot), best_yaw])


def _mid_between_peaks(values: np.ndarray) -> float:
    """Midpoint of the densest 5 mm bin in each half: the two opposite rail tops.

    Percentiles of the extent would not do: the stone wall stands at rail-top height just
    outside two of the rails and drags them outward.
    """
    counts, edges = np.histogram(values, bins=np.arange(values.min(), values.max() + 0.005, 0.005))
    centres = (edges[:-1] + edges[1:]) / 2
    half = len(centres) // 2
    low = centres[:half][np.argmax(counts[:half])]
    high = centres[half:][np.argmax(counts[half:])]
    return float((low + high) / 2)


def box_point(
    plane: Plane, params: np.ndarray, along: float, across: float, up: float
) -> np.ndarray:
    """A box-frame point (x along the box, y across, z up) in the camera frame."""
    cx, cy, yaw = params
    u = np.array([math.cos(yaw), math.sin(yaw)])
    v = np.array([-math.sin(yaw), math.cos(yaw)])
    xy = np.array([cx, cy]) + along * u + across * v
    return np.asarray(plane.origin + plane.rotation[:, :2] @ xy + plane.rotation[:, 2] * up)


def edge_lines(plane: Plane, params: np.ndarray, box: Box) -> list[tuple[np.ndarray, np.ndarray]]:
    """Every straight edge the target shows: three floor junctions and the rail-top edges."""
    inner = box.size / 2
    outer = inner + box.rail_thickness

    def p(a: float, b: float, z: float) -> np.ndarray:
        return box_point(plane, params, a, b, z)

    lines = [
        (p(-inner, -inner, 0), p(-inner, inner, 0)),
        (p(inner, -inner, 0), p(inner, inner, 0)),
        (p(-inner, inner, 0), p(inner, inner, 0)),
    ]
    z = box.rail_height
    for r in (inner, outer):
        lines += [
            (p(-r, -r, z), p(-r, r, z)),
            (p(r, -r, z), p(r, r, z)),
            (p(-r, r, z), p(r, r, z)),
            (p(-r, -r, z), p(r, -r, z)),
        ]
    return lines


def project(k: np.ndarray, points: np.ndarray) -> np.ndarray:
    pixels = (k @ np.atleast_2d(points).T).T
    return np.asarray(pixels[:, :2] / pixels[:, 2:])


def sample_lines(k: np.ndarray, lines: list[tuple[np.ndarray, np.ndarray]]) -> np.ndarray:
    t = np.linspace(0.0, 1.0, 200)[:, None]
    return np.vstack([project(k, a + (b - a) * t) for a, b in lines])


def refine(
    target: np.ndarray, k: np.ndarray, plane: Plane, start: np.ndarray, box: Box
) -> tuple[np.ndarray, float]:
    gray = cv2.GaussianBlur(cv2.cvtColor(target, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    edges = cv2.Canny(gray, 40, 100)
    distance = cv2.distanceTransform((edges == 0).astype(np.uint8), cv2.DIST_L2, 5)
    height, width = gray.shape

    def cost(params: np.ndarray) -> float:
        pixels = sample_lines(k, edge_lines(plane, params, box))
        inside = (
            (pixels[:, 0] > 1) & (pixels[:, 0] < width - 2)
            & (pixels[:, 1] > 1) & (pixels[:, 1] < height - 2)
        )  # fmt: skip
        if inside.sum() < 100:
            return EDGE_CLIP_PX
        sampled = cv2.remap(
            distance,
            pixels[inside, 0].astype(np.float32),
            pixels[inside, 1].astype(np.float32),
            cv2.INTER_LINEAR,
        )
        return float(np.minimum(sampled, EDGE_CLIP_PX).mean())

    best = None
    for dx in (-0.04, 0.0, 0.04):
        for dy in (-0.04, 0.0, 0.04):
            result = minimize(
                cost,
                start + np.array([dx, dy, 0.0]),
                method="Nelder-Mead",
                options={"xatol": 1e-4, "fatol": 1e-4, "maxiter": 1000},
            )
            if best is None or result.fun < best.fun:
                best = result
    assert best is not None
    return np.asarray(best.x), float(best.fun)


def world_from_camera_pose(plane: Plane, params: np.ndarray) -> np.ndarray:
    """T_W<-cam_cv, W the box frame: floor centre, z up, +y away from the camera."""
    cx, cy, yaw = params
    u = np.array([math.cos(yaw), math.sin(yaw)])
    v = np.array([-math.sin(yaw), math.cos(yaw)])
    camera_from_world = np.eye(4)
    camera_from_world[:3, :3] = np.c_[
        plane.rotation[:, :2] @ u, plane.rotation[:, :2] @ v, plane.rotation[:, 2]
    ]
    camera_from_world[:3, 3] = box_point(plane, params, 0.0, 0.0, 0.0)
    return np.asarray(np.linalg.inv(camera_from_world))


def hull_mask(
    shape: tuple[int, int], k: np.ndarray, plane: Plane, params: np.ndarray, box: Box
) -> np.ndarray:
    """The floor polygon the camera sees, with every rail's silhouette cut out of it."""
    inner = box.size / 2
    outer = inner + box.rail_thickness
    mask = np.zeros(shape, np.uint8)
    corners = ((-1, -1), (1, -1), (1, 1), (-1, 1))
    floor = [box_point(plane, params, a * inner, b * inner, 0.0) for a, b in corners]
    cv2.fillPoly(mask, [np.round(project(k, np.array(floor))).astype(np.int32)], 255)
    rails = (
        ((-outer, -inner), (-outer, outer)),  # left: x range, y range
        ((inner, outer), (-outer, outer)),
        ((-outer, outer), (inner, outer)),
        ((-outer, outer), (-outer, -inner)),
    )
    for (x0, x1), (y0, y1) in rails:
        corners = [
            box_point(plane, params, x, y, z)
            for x in (x0, x1)
            for y in (y0, y1)
            for z in (0.0, box.rail_height)
        ]
        hull = cv2.convexHull(np.round(project(k, np.array(corners))).astype(np.int32))
        cv2.fillConvexPoly(mask, hull, 0)
    return mask


def draw_overlay(
    target: np.ndarray, k: np.ndarray, plane: Plane, params: np.ndarray, box: Box, mask: np.ndarray
) -> np.ndarray:
    out = target.copy()
    out[mask > 0] = (0.6 * out[mask > 0] + np.array([0, 90, 0])).astype(np.uint8)
    for a, b in edge_lines(plane, params, box):
        line = np.round(project(k, np.array([a, b]))).astype(int)
        cv2.line(out, tuple(line[0]), tuple(line[1]), (255, 0, 255), 1, cv2.LINE_AA)
    return out


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("scene_dir", type=Path)
    parser.add_argument("--mat-size", type=float, default=MAT_SIZE_M)
    parser.add_argument("--rail-height", type=float, default=RAIL_HEIGHT_M)
    parser.add_argument("--rail-thickness", type=float, default=RAIL_THICKNESS_M)
    parser.add_argument("--cage-id", default="meatball_basement")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    box = Box(args.mat_size, args.rail_height, args.rail_thickness)
    intrinsics = json.loads((args.scene_dir / "camera_rect.json").read_text())
    k = np.array(
        [
            [intrinsics["fx"], 0.0, intrinsics["cx"]],
            [0.0, intrinsics["fy"], intrinsics["cy"]],
            [0.0, 0.0, 1.0],
        ]
    )
    poses_dir = args.scene_dir / "poses"
    poses_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, Any]] = []

    for target_dir in sorted((args.scene_dir / "targets").iterdir()):
        if not (target_dir / "xyz.npy").exists():
            continue
        clip = target_dir.name
        meta = json.loads((target_dir / "meta.json").read_text())
        target = cv2.imread(str(target_dir / "target.png"))
        xyz = np.load(target_dir / "xyz.npy")

        plane = fit_plane(xyz, floor_pixels(target), args.seed)
        start = rail_top_start(xyz, plane)
        params, edge_px = refine(target, k, plane, start, box)
        tf_world_from_camera = world_from_camera_pose(plane, params)
        summary = pose_summary(tf_world_from_camera)

        mask = hull_mask(target.shape[:2], k, plane, params, box)
        cv2.imwrite(str(target_dir / "hull_mask.png"), mask)
        cv2.imwrite(
            str(poses_dir / f"{clip}_overlay.png"),
            draw_overlay(target, k, plane, params, box, mask),
        )
        header = (
            f"Basement drive-test box camera for {clip} ({meta.get('source')}).\n"
            f"Fitted by playground/basement_scene/fit_basement_camera.py: floor plane from ZED "
            f"depth ({plane.rms_m * 1000:.1f} mm rms),\nbox centre and yaw from the rail edges "
            f"({edge_px:.2f} px mean edge distance). Box {args.mat_size} m inside the rails."
        )
        save_cage_calibration(
            poses_dir / f"{clip}.toml",
            CageCalibration(
                args.cage_id, args.mat_size, args.mat_size, camera_from_world(tf_world_from_camera)
            ),
            header,
        )
        row: dict[str, Any] = {
            "clip": clip,
            "plane_rms_mm": round(plane.rms_m * 1000, 2),
            "plane_inliers": plane.inliers,
            "start_xy_yaw": [round(float(v), 4) for v in (*start[:2], math.degrees(start[2]))],
            "fit_xy_yaw": [round(float(v), 4) for v in (*params[:2], math.degrees(params[2]))],
            "edge_px": round(edge_px, 3),
            "floor_px_seen": int((mask > 0).sum()),
        }
        row.update({key: round(value, 4) for key, value in summary.items()})
        row["status"] = "ok" if edge_px < 3.0 and plane.rms_m < 0.005 else "CHECK"
        rows.append(row)
        print(
            f"{clip}: {row['status']}  edge {edge_px:.2f} px  plane {plane.rms_m * 1000:.1f} mm  "
            + ", ".join(f"{key} {value:.3f}" for key, value in summary.items())
        )

    if not rows:
        raise SystemExit(f"no targets with xyz.npy under {args.scene_dir / 'targets'}")
    with (poses_dir / "summary.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
