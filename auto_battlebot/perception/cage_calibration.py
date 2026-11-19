"""Fixed-camera cage calibration: the `config/cages/<id>.toml` format and its frame chain."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from auto_battlebot.compat import tomllib

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
    tf_cvcamera_from_hfield: np.ndarray

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
        tf_cvcamera_from_hfield=tf,
    )


def save_cage_calibration(path: Path, calibration: CageCalibration, header: str = "") -> None:
    """Write the TOML `cage_calibration.cpp` reads. `header` lines become leading comments."""
    tf = calibration.tf_cvcamera_from_hfield
    lines = [f"# {line}" if line else "#" for line in header.splitlines()]
    lines += [
        f'calibration_id = "{calibration.calibration_id}"',
        f"field_size_x = {calibration.field_size_x!r}",
        f"field_size_y = {calibration.field_size_y!r}",
        "",
        "# tf_cvcamera_from_hfield: the field centre in camera coordinates (OpenCV camera,",
        "# z forward, y down; field z up). translation in metres, rotation row-major 3x3.",
        "translation = [" + ", ".join(repr(float(v)) for v in tf[:3, 3]) + "]",
        "rotation = [" + ", ".join(repr(float(v)) for v in tf[:3, :3].reshape(-1)) + "]",
        "",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines))


def world_from_camera(tf_cvcamera_from_hfield: np.ndarray) -> np.ndarray:
    """T_W<-cam_cv: the OpenCV camera pose in the W frame."""
    return np.asarray(WORLD_FROM_HFIELD @ np.linalg.inv(tf_cvcamera_from_hfield))


def camera_from_world(tf_world_from_camera: np.ndarray) -> np.ndarray:
    """Inverse of `world_from_camera`: T_cam<-hfield from T_W<-cam_cv."""
    return np.asarray(np.linalg.inv(tf_world_from_camera) @ WORLD_FROM_HFIELD)


def blender_cam2world(tf_cvcamera_from_hfield: np.ndarray) -> np.ndarray:
    """The matrix `bproc.camera.add_camera_pose` wants for this calibration."""
    return np.asarray(world_from_camera(tf_cvcamera_from_hfield) @ OPENCV_TO_BLENDER_CAMERA)


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
    """T_W<-cam_cv from a true_battlebot `metrics_tool/nhrl_cage_*.toml` camera block."""
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
    """Human-readable numbers for a camera pose in W."""
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
