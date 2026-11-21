"""auto_battlebot.perception.camera_geometry: floor projection agrees with the runtime."""

from __future__ import annotations

import numpy as np

from auto_battlebot.compat import tomllib
from auto_battlebot.paths import REPO_ROOT
from auto_battlebot.perception import camera_geometry as cg

CAMERA_HEIGHT_M = 2.0


def _downward_geometry() -> cg.FrameGeometry:
    tf = np.eye(4)
    tf[:3, :3] = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
    tf[:3, 3] = [0.3, -0.2, CAMERA_HEIGHT_M]
    return cg.FrameGeometry(
        stamp_ns=0,
        fx=500.0,
        fy=520.0,
        cx=320.0,
        cy=240.0,
        tf_field_from_camera=tf,
        method="exact",
        field_size_m=(2.4, 2.4),
    )


def test_projection_matches_cpp_project_keypoint_onto_plane() -> None:
    geometry = _downward_geometry()
    pixels = np.array([[100.0, 80.0], [320.0, 240.0], [500.0, 400.0]])
    height = 0.12
    floor = cg.pixels_to_floor(pixels, geometry, 0.0)
    raised = cg.pixels_to_floor(pixels, geometry, height)
    nadir = np.array([0.3, -0.2, 0.0])
    scale = (CAMERA_HEIGHT_M - height) / CAMERA_HEIGHT_M
    cpp_corrected = nadir[:2] + (floor[:, :2] - nadir[:2]) * scale
    np.testing.assert_allclose(raised[:, :2], cpp_corrected, atol=1e-9)
    np.testing.assert_allclose(raised[:, 2], height)


def test_keypoint_heights_match_common_toml() -> None:
    with open(REPO_ROOT / "config" / "_common.toml", "rb") as handle:
        robot_filter = tomllib.load(handle)["robot_filter"]
    per_label = robot_filter["keypoint_height_meters_per_label"]
    assert cg.DEFAULT_KEYPOINT_HEIGHT_M == robot_filter["keypoint_height_meters"]
    for label, height in cg.KEYPOINT_HEIGHT_M_BY_GT_LABEL.items():
        assert height == per_label[label.upper()], label


def test_pixels_floor_round_trip() -> None:
    geometry = _downward_geometry()
    pixels = np.array([[100.0, 80.0], [320.0, 240.0], [500.0, 400.0]])
    for plane_height in (0.0, 0.12):
        points = cg.pixels_to_floor(pixels, geometry, plane_height)
        np.testing.assert_allclose(cg.field_to_pixels(points, geometry), pixels, atol=1e-6)
