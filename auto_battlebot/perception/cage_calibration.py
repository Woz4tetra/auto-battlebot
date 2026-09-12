"""Fixed-camera cage calibration: the `config/cages/<id>.toml` format, and the frames it links.

The C++ reader is `src/field_filter/cage_calibration.cpp`; `save_cage_calibration` writes the
exact keys it parses (`calibration_id`, `field_size_x`, `field_size_y`, `translation` as a
3-list, `rotation` as a row-major 9-list), so a pose fitted in Python is what
`CalibratedFieldFilter` loads at match time.

Frames, written down once here and reused by every fit, render, and grading script:

- **hfield**: the frame `auto_battlebot.perception.field_pose.pose_from_corners` solves in.
  Origin at the mat centre, z up (r3 . t < 0 for a camera above the mat), and the corner
  winding of `field_object_corners`: object (-x, -y) binds to the image's top-left corner.
  For a camera at the near wall looking across the mat that puts hfield +x toward image
  bottom (the camera) and hfield +y to image right. `tf_camera_from_fieldcenter` in the
  TOML is `T_cam<-hfield` with the camera in OpenCV convention (z forward, y down).
- **W** (the Blender world and the true_battlebot map frame): mat centre, z up, +y away
  from the camera (image top), +x to image right. `WORLD_FROM_HFIELD` is Rz(-90 deg):
  hfield x -> W -y, hfield y -> W +x.
- **Blender camera**: looks down its own -z with +y up, so `OPENCV_TO_BLENDER_CAMERA`
  flips y and z, the same thing BlenderProc's
  `change_source_coordinate_frame_of_transformation_matrix(T, ["X", "-Y", "-Z"])` does.

The chain from a fitted pose to a BlenderProc camera:

    T_W<-cam_cv       = WORLD_FROM_HFIELD @ inv(T_cam<-hfield)
    cam2world_blender = T_W<-cam_cv @ OPENCV_TO_BLENDER_CAMERA
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

if sys.version_info >= (3, 11):
    import tomllib
else:  # pragma: no cover - Python 3.10 fallback
    import tomli as tomllib

_ROTATE_Z_MINUS_90 = np.array(
    [[0.0, 1.0, 0.0, 0.0], [-1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
)
WORLD_FROM_HFIELD: np.ndarray = _ROTATE_Z_MINUS_90
OPENCV_TO_BLENDER_CAMERA: np.ndarray = np.diag([1.0, -1.0, -1.0, 1.0])


@dataclass
class CageCalibration:
    calibration_id: str
    field_size_x: float
    field_size_y: float
    tf_camera_from_fieldcenter: np.ndarray  # 4x4, T_cam<-hfield, OpenCV camera

    @property
    def field_size_xy(self) -> tuple[float, float]:
        return self.field_size_x, self.field_size_y


def load_cage_calibration(path: Path) -> CageCalibration:
    with path.open("rb") as handle:
        data = tomllib.load(handle)
    translation = [float(v) for v in data["translation"]]
    rotation = [float(v) for v in data["rotation"]]
    if len(translation) != 3 or len(rotation) != 9:
        raise ValueError(f"{path}: translation needs 3 values and rotation 9")
    tf = np.eye(4)
    tf[:3, :3] = np.array(rotation).reshape(3, 3)
    tf[:3, 3] = translation
    return CageCalibration(
        calibration_id=str(data["calibration_id"]),
        field_size_x=float(data["field_size_x"]),
        field_size_y=float(data["field_size_y"]),
        tf_camera_from_fieldcenter=tf,
    )


def save_cage_calibration(path: Path, calibration: CageCalibration, header: str = "") -> None:
    """Write the TOML `cage_calibration.cpp` reads. `header` lines become leading comments."""
    tf = calibration.tf_camera_from_fieldcenter
    lines = [f"# {line}" if line else "#" for line in header.splitlines()]
    lines += [
        f'calibration_id = "{calibration.calibration_id}"',
        f"field_size_x = {calibration.field_size_x!r}",
        f"field_size_y = {calibration.field_size_y!r}",
        "",
        "# tf_camera_from_fieldcenter: the field centre in camera coordinates (OpenCV camera,",
        "# z forward, y down; field z up). translation in metres, rotation row-major 3x3.",
        "translation = [" + ", ".join(repr(float(v)) for v in tf[:3, 3]) + "]",
        "rotation = [" + ", ".join(repr(float(v)) for v in tf[:3, :3].reshape(-1)) + "]",
        "",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines))


# ------------------------------------------------------------------------------ frames


def world_from_camera(tf_camera_from_fieldcenter: np.ndarray) -> np.ndarray:
    """T_W<-cam_cv: the OpenCV camera pose in the W frame."""
    return np.asarray(WORLD_FROM_HFIELD @ np.linalg.inv(tf_camera_from_fieldcenter))


def camera_from_world(tf_world_from_camera: np.ndarray) -> np.ndarray:
    """Inverse of `world_from_camera`: T_cam<-hfield from T_W<-cam_cv."""
    return np.asarray(np.linalg.inv(tf_world_from_camera) @ WORLD_FROM_HFIELD)


def blender_cam2world(tf_camera_from_fieldcenter: np.ndarray) -> np.ndarray:
    """The matrix `bproc.camera.add_camera_pose` wants for this calibration."""
    return np.asarray(world_from_camera(tf_camera_from_fieldcenter) @ OPENCV_TO_BLENDER_CAMERA)


def rotation_rpy_sxyz(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """R = Rz(yaw) Ry(pitch) Rx(roll), the static-frame 'sxyz' Euler convention."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return np.asarray(rz @ ry @ rx)


def prior_from_position_rpy(
    position: tuple[float, float, float], rpy_deg: tuple[float, float, float]
) -> np.ndarray:
    """T_W<-cam_cv from a true_battlebot `metrics_tool/nhrl_cage_*.toml` camera block.

    Those files store `tf_map_from_camera` as position plus roll/pitch/yaw in degrees under
    the 'sxyz' convention, with the map frame equal to W and the camera in OpenCV
    convention. Roll near -150 deg is a camera looking down and across toward +y.
    """
    tf = np.eye(4)
    tf[:3, :3] = rotation_rpy_sxyz(*(math.radians(v) for v in rpy_deg))
    tf[:3, 3] = position
    return tf


def load_prior_pose(path: Path) -> np.ndarray:
    """T_W<-cam_cv from a true_battlebot metrics-tool camera TOML."""
    with path.open("rb") as handle:
        data = tomllib.load(handle)
    pos = data["camera"]["position"]
    rot = data["camera"]["rotation"]
    return prior_from_position_rpy(
        (float(pos["x"]), float(pos["y"]), float(pos["z"])),
        (float(rot["roll"]), float(rot["pitch"]), float(rot["yaw"])),
    )


def pose_summary(tf_world_from_camera: np.ndarray) -> dict[str, Any]:
    """Human-readable numbers for a camera pose in W.

    tilt_from_down_deg is the angle between the optical axis and straight down; yaw_deg is
    the heading of the optical axis projected on the floor, 0 along +y (looking away from
    the near wall), positive toward +x.
    """
    tf = np.asarray(tf_world_from_camera)
    forward = tf[:3, :3] @ np.array([0.0, 0.0, 1.0])
    tilt = math.degrees(math.acos(float(np.clip(-forward[2], -1.0, 1.0))))
    yaw = math.degrees(math.atan2(float(forward[0]), float(forward[1])))
    return {
        "x_m": float(tf[0, 3]),
        "y_m": float(tf[1, 3]),
        "height_m": float(tf[2, 3]),
        "tilt_from_down_deg": tilt,
        "yaw_deg": yaw,
    }
