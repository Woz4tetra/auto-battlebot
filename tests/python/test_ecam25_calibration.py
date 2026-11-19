"""The estimated e-CAM25 calibration must keep reproducing the spec sheet it was fitted to."""

from __future__ import annotations

import math
import re
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

AR0234CS_NATIVE_WIDTH, AR0234CS_NATIVE_HEIGHT = 1920, 1200
ECON_SPEC_FOV_HORIZONTAL_DEG, ECON_SPEC_FOV_VERTICAL_DEG, ECON_SPEC_FOV_DIAGONAL_DEG = (
    104.6,
    61.6,
    128.2,
)


TANGENTIAL_AND_K3 = slice(2, 5)


@pytest.fixture
def calibration() -> CameraCalibration:
    return load_camera_calibration(CALIBRATION_PATH)


def radius_for_angle(calibration: CameraCalibration, half_angle_deg: float) -> float:
    """Pixels from the principal point where a ray *half_angle_deg* off axis lands."""
    tangent = math.tan(math.radians(half_angle_deg))
    point = np.array([[[tangent, 0.0, 1.0]]], dtype=np.float64)
    pixels, _ = cv2.projectPoints(point, np.zeros(3), np.zeros(3), calibration.K, calibration.D)
    return float(pixels.reshape(2)[0] - calibration.cx)


def test_distortion_vector_order_matches_the_cpp_writer() -> None:
    cpp = (
        Path(__file__).resolve().parents[2] / "src/rgbd_camera/camera_calibration.cpp"
    ).read_text()
    match = re.search(r"kNames\[\] = \{([^}]*)\}", cpp)
    assert match is not None
    cpp_order = re.findall(r'"(\w+)"', match.group(1))
    value_of = {"k1": 1.0, "k2": 2.0, "p1": 3.0, "p2": 4.0, "k3": 5.0}
    calibration = CameraCalibration(
        "t", 1920, 1080, 1.0, 1.0, 960.0, 540.0, (1.0, 2.0, 3.0, 4.0, 5.0)
    )
    assert list(calibration.D) == [value_of[name] for name in cpp_order]


def test_the_three_published_fields_of_view_are_reproduced(calibration: CameraCalibration) -> None:
    half_width = AR0234CS_NATIVE_WIDTH / 2
    half_height = AR0234CS_NATIVE_HEIGHT / 2
    half_diagonal = math.hypot(half_width, half_height)

    assert radius_for_angle(calibration, ECON_SPEC_FOV_HORIZONTAL_DEG / 2) == pytest.approx(
        half_width, abs=0.25
    )
    assert radius_for_angle(calibration, ECON_SPEC_FOV_VERTICAL_DEG / 2) == pytest.approx(
        half_height, abs=0.25
    )
    assert radius_for_angle(calibration, ECON_SPEC_FOV_DIAGONAL_DEG / 2) == pytest.approx(
        half_diagonal, abs=0.25
    )


def test_the_16_9_file_is_a_crop_of_the_native_mode(calibration: CameraCalibration) -> None:
    """Same focal length and same distortion as the 1920x1200 fit, only `cy` recenterd."""
    assert (calibration.width, calibration.height) == (1920, 1080)
    assert calibration.fx == calibration.fy
    assert (calibration.cx, calibration.cy) == (calibration.width / 2, calibration.height / 2)
    assert calibration.distortion_k1_k2_p1_p2_k3[TANGENTIAL_AND_K3] == (0.0, 0.0, 0.0)


def test_distortion_stays_monotonic_out_to_the_image_corner(
    calibration: CameraCalibration,
) -> None:
    """`r_d(r)` must increase all the way to the corner, or rectification folds it."""
    k1, k2, _, _, k3 = calibration.distortion_k1_k2_p1_p2_k3
    angles = np.linspace(1e-4, math.radians(ECON_SPEC_FOV_DIAGONAL_DEG / 2), 400)
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


def test_rectify_maps_default_keeps_every_source_pixel(calibration: CameraCalibration) -> None:
    size = (calibration.width, calibration.height)
    default_k = rectify_maps(calibration, size)[2]
    assert np.allclose(default_k, rectify_maps(calibration, size, 1.0)[2])
    assert default_k[0, 0] < rectify_maps(calibration, size, 0.0)[2][0, 0]
