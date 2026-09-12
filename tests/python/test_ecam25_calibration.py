"""The estimated e-CAM25 calibration must keep reproducing the spec sheet it was fitted to.

`config/cameras/ecam25_h01r1_estimated.toml` is not a measured calibration. It is a plumb_bob
approximation of a fisheye-class lens, pinned to the only three numbers e-con publishes: the
104.6 / 61.6 / 128.2 deg fields of view. These tests project rays at those three half-angles and
check they land on the sensor's half-width, half-height and half-diagonal, so a later edit cannot
silently move the file off the spec sheet.

They also reject the tempting alternative: a k1/k2/k3 least-squares fit tracks the underlying
fisheye curve three times better but reverses just inside the image corner, which makes
`initUndistortRectifyMap` fold the corners. The monotonicity assertion below is what catches that.
"""

from __future__ import annotations

import math
from pathlib import Path

import cv2
import numpy as np
import pytest

from auto_battlebot.perception.camera_calibration import (
    CameraCalibration,
    load_camera_calibration,
    rectify_maps,
)

CALIBRATION_PATH = (
    Path(__file__).resolve().parents[2] / "config/cameras/ecam25_h01r1_estimated.toml"
)

# The AR0234CS native mode the spec-sheet fields of view describe: 1920x1200 at 3 um pixels.
NATIVE_WIDTH, NATIVE_HEIGHT = 1920, 1200
# Full fields of view from e-con's spec sheet, in degrees.
FOV_HORIZONTAL_DEG, FOV_VERTICAL_DEG, FOV_DIAGONAL_DEG = 104.6, 61.6, 128.2


@pytest.fixture
def calibration() -> CameraCalibration:
    return load_camera_calibration(CALIBRATION_PATH)


def radius_for_angle(calibration: CameraCalibration, half_angle_deg: float) -> float:
    """Pixels from the principal point where a ray *half_angle_deg* off axis lands."""
    # A point on that ray, one meter down the optical axis. cv2.projectPoints applies the same
    # Brown-Conrady model the C++ Rectifier does.
    tangent = math.tan(math.radians(half_angle_deg))
    point = np.array([[[tangent, 0.0, 1.0]]], dtype=np.float64)
    pixels, _ = cv2.projectPoints(point, np.zeros(3), np.zeros(3), calibration.K, calibration.D)
    return float(pixels.reshape(2)[0] - calibration.cx)


def test_the_three_published_fields_of_view_are_reproduced(calibration: CameraCalibration) -> None:
    # The fit was done on the native 1920x1200 mode; the shipped file is its 16:9 crop, so the
    # half-extents to compare against are the sensor's, not the file's.
    half_width = NATIVE_WIDTH / 2
    half_height = NATIVE_HEIGHT / 2
    half_diagonal = math.hypot(half_width, half_height)

    assert radius_for_angle(calibration, FOV_HORIZONTAL_DEG / 2) == pytest.approx(
        half_width, abs=0.25
    )
    assert radius_for_angle(calibration, FOV_VERTICAL_DEG / 2) == pytest.approx(
        half_height, abs=0.25
    )
    assert radius_for_angle(calibration, FOV_DIAGONAL_DEG / 2) == pytest.approx(
        half_diagonal, abs=0.25
    )


def test_the_16_9_file_is_a_crop_of_the_native_mode(calibration: CameraCalibration) -> None:
    """Same focal length and same distortion as the 1920x1200 fit, only `cy` recenterd."""
    assert (calibration.width, calibration.height) == (1920, 1080)
    assert calibration.fx == calibration.fy
    assert (calibration.cx, calibration.cy) == (calibration.width / 2, calibration.height / 2)
    assert calibration.distortion[2:] == (0.0, 0.0, 0.0)  # p1, p2, k3 all unused


def test_distortion_stays_monotonic_out_to_the_image_corner(
    calibration: CameraCalibration,
) -> None:
    """`r_d(r)` must increase all the way to the corner, or rectification folds it.

    This is the assertion that rejects the better-residual k1/k2/k3 fit, which turns over at
    63.3 deg, inside the 64.1 deg corner.
    """
    k1, k2, _, _, k3 = calibration.distortion
    angles = np.linspace(1e-4, math.radians(FOV_DIAGONAL_DEG / 2), 400)
    radii = np.tan(angles)
    distorted = radii * (1.0 + k1 * radii**2 + k2 * radii**4 + k3 * radii**6)
    assert np.all(np.diff(distorted) > 0.0)


def test_rectified_field_and_border_are_what_the_header_claims(
    calibration: CameraCalibration,
) -> None:
    """Alpha 1.0 keeps every source pixel, so the rectified frame is wider and mostly border."""
    size = (calibration.width, calibration.height)
    map_x, map_y, k_rect = rectify_maps(calibration, size)

    assert k_rect[0, 0] == pytest.approx(576.4, abs=0.5)
    horizontal_fov = math.degrees(2 * math.atan(calibration.width / 2 / k_rect[0, 0]))
    assert horizontal_fov == pytest.approx(118.0, abs=0.5)

    filled = np.full((calibration.height, calibration.width), 255, dtype=np.uint8)
    rectified = cv2.remap(
        filled, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0
    )
    assert float((rectified == 0).mean()) == pytest.approx(0.362, abs=0.01)
