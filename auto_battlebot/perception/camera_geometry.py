"""Project eval-dataset pixels onto the arena floor using the exported camera geometry.

`export_camera_transforms.py` writes `camera_info.json` and `camera_transforms/<stamp>.json`
per sub dataset. This turns those into floor positions: a pixel becomes a camera ray, the ray
is rotated into the field frame, and it is intersected with the horizontal plane the target
actually sits on. That is the same ray-plane projection the runtime does in
`project_keypoint_onto_plane` (src/transform_utils.cpp), so distances computed here are in the
units navigation consumes.

Heights come from `[robot_filter] keypoint_height_meters*` in `config/_common.toml`: our
robots' keypoints are labeled on the floor, house bots ride 0.12 m up, everything else takes
the 0.03 m default. Projecting a robot onto the wrong plane biases its range, so the label
matters.

Only `exact` and `interpolated` transforms are usable. `unavailable` has no pose at all and
`ambiguous` has two frames claiming the index, so both are dropped by default.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import numpy as np

# Mirrors [robot_filter] in config/_common.toml. GT labels are lowercase.
KEYPOINT_HEIGHT_M = {
    "house_bot": 0.12,
    "mr_stabs_mk2": 0.0,
    "mrs_buff_mk3": 0.0,
}
DEFAULT_KEYPOINT_HEIGHT_M = 0.03

USABLE_METHODS = ("exact", "interpolated")

# The arena is an 8 ft square. The per-frame `field_size_m` is the RANSAC fit's estimate and
# reads undersized or outright fails on some inits, so the nominal is what to draw.
NOMINAL_FIELD_SIZE_M = 2.4384


class CameraGeometry(Protocol):
    """Intrinsics plus a field-frame camera pose.

    `FrameGeometry` reads one from an exported sidecar; `floor_background.PoseGeometry`
    builds one from a pose matrix recovered off a recording's tf chain. The projection
    helpers here and the floor warps in `floor_background` take either.
    """

    fx: float
    fy: float
    cx: float
    cy: float
    tf_field_from_camera: np.ndarray  # 4x4

    @property
    def camera_position(self) -> np.ndarray: ...


@dataclass
class FrameGeometry:
    """One image's intrinsics and camera pose in the field frame."""

    stamp_ns: int
    fx: float
    fy: float
    cx: float
    cy: float
    tf_field_from_camera: np.ndarray  # 4x4
    method: str
    field_size_m: tuple[float, float]

    @property
    def camera_position(self) -> np.ndarray:
        """Camera origin in the field frame."""
        return self.tf_field_from_camera[:3, 3]

    @property
    def camera_height_m(self) -> float:
        return float(self.camera_position[2])


def height_for_label(label: str) -> float:
    return KEYPOINT_HEIGHT_M.get(label, DEFAULT_KEYPOINT_HEIGHT_M)


def load_camera_info(subdataset: Path) -> tuple[float, float, float, float] | None:
    """(fx, fy, cx, cy) from a sub dataset's camera_info.json, or None when absent."""
    path = subdataset / "camera_info.json"
    if not path.exists():
        return None
    matrix = json.loads(path.read_text())["K"]
    return float(matrix[0]), float(matrix[4]), float(matrix[2]), float(matrix[5])


def load_frame_geometry(
    image_path: Path, methods: tuple[str, ...] = USABLE_METHODS
) -> FrameGeometry | None:
    """Geometry for one eval image, or None when the pose is missing or untrustworthy.

    `image_path` is <subdataset>/images/<stamp_ns>.png, which locates both sidecars."""
    subdataset = image_path.parent.parent
    intrinsics = load_camera_info(subdataset)
    if intrinsics is None:
        return None
    transform_path = subdataset / "camera_transforms" / f"{image_path.stem}.json"
    if not transform_path.exists():
        return None
    record = json.loads(transform_path.read_text())
    if record.get("method") not in methods:
        return None
    transform = record.get("tf_field_from_camera", {}).get("matrix")
    if transform is None:
        return None
    size = record.get("field_size_m", {})
    fx, fy, cx, cy = intrinsics
    return FrameGeometry(
        stamp_ns=int(record["stamp_ns"]),
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
        tf_field_from_camera=np.asarray(transform, dtype=np.float64),
        method=str(record["method"]),
        field_size_m=(float(size.get("x", np.nan)), float(size.get("y", np.nan))),
    )


def pixels_to_floor(
    points_px: np.ndarray, geometry: FrameGeometry, plane_height_m: float = 0.0
) -> np.ndarray:
    """(N, 2) pixels to (N, 3) field-frame points on the plane z = plane_height_m.

    Rows that do not intersect the plane in front of the camera come back as NaN, which is
    what a pixel above the horizon does."""
    points = np.asarray(points_px, dtype=np.float64).reshape(-1, 2)
    out = np.full((len(points), 3), np.nan)
    if len(points) == 0:
        return out

    # Pixel -> normalized camera ray (+Z forward), then rotate into the field frame.
    rays_camera = np.stack(
        [
            (points[:, 0] - geometry.cx) / geometry.fx,
            (points[:, 1] - geometry.cy) / geometry.fy,
            np.ones(len(points)),
        ],
        axis=1,
    )
    rays_field = rays_camera @ geometry.tf_field_from_camera[:3, :3].T
    origin = geometry.camera_position

    # Intersect with the horizontal plane; only descending rays reach it.
    denominator = rays_field[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        distances = (plane_height_m - origin[2]) / denominator
    valid = np.isfinite(distances) & (distances > 0) & (np.abs(denominator) > 1e-9)
    out[valid] = origin + rays_field[valid] * distances[valid, None]
    return out


def field_to_pixels(points_field: np.ndarray, geometry: FrameGeometry) -> np.ndarray:
    """(N, 3) field-frame points to (N, 2) pixels. Points behind the camera come back NaN.

    The inverse of `pixels_to_floor`, so composing the two round-trips a pixel back onto
    itself. That identity is the cheapest check that a transform is not garbage."""
    points = np.asarray(points_field, dtype=np.float64).reshape(-1, 3)
    out = np.full((len(points), 2), np.nan)
    if len(points) == 0:
        return out
    transform = np.linalg.inv(geometry.tf_field_from_camera)
    camera_points = points @ transform[:3, :3].T + transform[:3, 3]
    in_front = camera_points[:, 2] > 1e-6
    out[in_front, 0] = (
        geometry.fx * camera_points[in_front, 0] / camera_points[in_front, 2] + geometry.cx
    )
    out[in_front, 1] = (
        geometry.fy * camera_points[in_front, 1] / camera_points[in_front, 2] + geometry.cy
    )
    return out


def ground_range_m(point_field: np.ndarray, geometry: FrameGeometry) -> float:
    """Horizontal distance from the camera's nadir to a field point, in metres."""
    if not np.all(np.isfinite(point_field[:2])):
        return float("nan")
    return float(np.hypot(*(point_field[:2] - geometry.camera_position[:2])))
