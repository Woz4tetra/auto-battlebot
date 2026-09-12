"""Frame conventions behind the fixed-camera cage calibration.

A synthetic camera above a 2.35 m mat, looking across it from the near wall, is projected
through a pinhole, solved back with `pose_from_corners`, and pushed through the frame chain
in `cage_calibration`. Every sign convention the render and grading scripts rely on is
asserted here rather than in a docstring.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from auto_battlebot.perception.cage_calibration import (
    OPENCV_TO_BLENDER_CAMERA,
    WORLD_FROM_HFIELD,
    CageCalibration,
    blender_cam2world,
    camera_from_world,
    load_cage_calibration,
    pose_summary,
    prior_from_position_rpy,
    save_cage_calibration,
    world_from_camera,
)
from auto_battlebot.perception.field_pose import (
    field_object_corners,
    order_corners,
    pose_from_corners,
    project_field_corners,
)

K = np.array([[876.5, 0.0, 959.1], [0.0, 875.8, 536.1], [0.0, 0.0, 1.0]])
MAT = (2.35, 2.35)


def look_at_world_from_camera(position: np.ndarray, target: np.ndarray) -> np.ndarray:
    """T_W<-cam_cv for an OpenCV camera at `position` looking at `target`, z up in W."""
    forward = target - position
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.array([0.0, 0.0, 1.0]))
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    tf = np.eye(4)
    tf[:3, :3] = np.stack([right, down, forward], axis=1)
    tf[:3, 3] = position
    return tf


def project_world_points(tf_world_from_camera: np.ndarray, points_w: np.ndarray) -> np.ndarray:
    tf_cam_from_world = np.linalg.inv(tf_world_from_camera)
    cam = (tf_cam_from_world[:3, :3] @ points_w.T).T + tf_cam_from_world[:3, 3]
    px = K @ cam.T
    return np.asarray((px[:2] / px[2]).T)


@pytest.fixture
def near_wall_camera() -> np.ndarray:
    return look_at_world_from_camera(np.array([0.05, -1.2, 1.5]), np.array([0.0, 0.2, 0.0]))


def test_pose_from_corners_recovers_the_camera_and_keeps_field_z_up(
    near_wall_camera: np.ndarray,
) -> None:
    half = MAT[0] / 2
    corners_w = np.array([[-half, -half, 0], [-half, half, 0], [half, half, 0], [half, -half, 0]])
    pixels = order_corners(project_world_points(near_wall_camera, corners_w))
    solved = pose_from_corners(pixels, MAT, K)
    assert solved is not None
    tf_cam_from_hfield, reprojection_px = solved
    assert reprojection_px < 1e-3

    # Field z faces the camera for a camera above the mat.
    r3, t = tf_cam_from_hfield[:3, 2], tf_cam_from_hfield[:3, 3]
    assert float(r3 @ t) < 0

    # The chain lands back on the synthetic camera, so WORLD_FROM_HFIELD is right.
    recovered = world_from_camera(tf_cam_from_hfield)
    assert np.allclose(recovered, near_wall_camera, atol=1e-6)
    assert np.allclose(camera_from_world(recovered), tf_cam_from_hfield, atol=1e-6)

    # Projecting the field corners through the solved pose reproduces the pixels.
    assert np.allclose(project_field_corners(tf_cam_from_hfield, MAT, K), pixels, atol=1e-3)


def test_world_from_hfield_is_a_quarter_turn_about_z() -> None:
    rotation = WORLD_FROM_HFIELD[:3, :3]
    assert np.allclose(rotation @ np.array([1.0, 0.0, 0.0]), [0.0, -1.0, 0.0])  # hfield x -> W -y
    assert np.allclose(rotation @ np.array([0.0, 1.0, 0.0]), [1.0, 0.0, 0.0])  # hfield y -> W +x
    assert np.allclose(rotation @ np.array([0.0, 0.0, 1.0]), [0.0, 0.0, 1.0])
    assert np.isclose(np.linalg.det(rotation), 1.0)
    # Object corner (-x, -y) binds to the image top-left, which is the far-left mat corner.
    assert np.allclose(field_object_corners(MAT)[0], [-MAT[0] / 2, -MAT[1] / 2])


def test_blender_camera_looks_down_its_minus_z(near_wall_camera: np.ndarray) -> None:
    cam2world = blender_cam2world(camera_from_world(near_wall_camera))
    optical_axis_cv = near_wall_camera[:3, :3] @ np.array([0.0, 0.0, 1.0])
    blender_minus_z = cam2world[:3, :3] @ np.array([0.0, 0.0, -1.0])
    assert np.allclose(blender_minus_z, optical_axis_cv)
    assert np.allclose(cam2world[:3, 3], near_wall_camera[:3, 3])
    assert np.allclose(OPENCV_TO_BLENDER_CAMERA @ OPENCV_TO_BLENDER_CAMERA, np.eye(4))


def test_pose_summary_and_priors(near_wall_camera: np.ndarray) -> None:
    summary = pose_summary(near_wall_camera)
    assert math.isclose(summary["height_m"], 1.5)
    assert 40 < summary["tilt_from_down_deg"] < 50  # atan(1.4 / 1.5) = 43 deg
    assert abs(summary["yaw_deg"]) < 3
    # A roll of -150 deg in the true_battlebot files is a camera looking down and toward +y.
    prior = prior_from_position_rpy((0.0, -1.14, 1.66), (-150.0, 0.0, 0.0))
    prior_summary = pose_summary(prior)
    assert math.isclose(prior_summary["tilt_from_down_deg"], 30.0, abs_tol=1e-6)
    assert abs(prior_summary["yaw_deg"]) < 1e-6


def test_cage_calibration_toml_round_trip(tmp_path: Path, near_wall_camera: np.ndarray) -> None:
    calibration = CageCalibration("unit_test", 2.35, 2.30, camera_from_world(near_wall_camera))
    path = tmp_path / "cage.toml"
    save_cage_calibration(path, calibration, header="written by a test\n")
    text = path.read_text()
    assert text.startswith("# written by a test")
    for key in ("calibration_id", "field_size_x", "field_size_y", "translation", "rotation"):
        assert f"\n{key} = " in text or text.startswith(f"{key} = ")
    loaded = load_cage_calibration(path)
    assert loaded.calibration_id == "unit_test"
    assert loaded.field_size_xy == (2.35, 2.30)
    assert np.allclose(loaded.tf_camera_from_fieldcenter, calibration.tf_camera_from_fieldcenter)


def _lines_from_pixels(pixels: np.ndarray) -> list[tuple[np.ndarray, float] | None]:
    lines: list[tuple[np.ndarray, float] | None] = []
    for i in range(4):
        a, b = pixels[i], pixels[(i + 1) % 4]
        direction = (b - a) / np.linalg.norm(b - a)
        normal = np.array([-direction[1], direction[0]])
        lines.append((normal, float(normal @ a)))
    return lines


def test_pose_from_lines_recovers_a_clipped_near_edge(near_wall_camera: np.ndarray) -> None:
    from auto_battlebot.perception.field_pose import pose_from_lines

    truth = camera_from_world(near_wall_camera)
    pixels = project_field_corners(truth, MAT, K)
    lines = _lines_from_pixels(pixels)
    # Corner 2 -> 3 is the near edge (largest image y); pretend the frame cut it off.
    near = int(np.argmax([(pixels[i][1] + pixels[(i + 1) % 4][1]) for i in range(4)]))
    lines[near] = None

    # Seed from a deliberately wrong prior: 20 cm off and 5 degrees rolled.
    perturbed = look_at_world_from_camera(np.array([0.2, -1.0, 1.7]), np.array([0.1, 0.4, 0.0]))
    solved = pose_from_lines(lines, MAT, K, camera_from_world(perturbed))
    assert solved is not None
    tf, rms = solved
    assert rms < 1e-3
    assert np.allclose(tf, truth, atol=1e-4)

    # Two lines are not enough.
    lines[(near + 1) % 4] = None
    assert pose_from_lines(lines, MAT, K, camera_from_world(perturbed)) is None


def test_side_lines_from_points_skips_border_and_flags_missing_side() -> None:
    from auto_battlebot.perception.field_pose import (
        contour_points_off_border,
        side_lines_from_points,
    )

    # A trapezoid whose bottom runs past the image bottom, as the cage-high framing does.
    height, width = 1080, 1920
    quad = np.array([[500.0, 100.0], [1400.0, 100.0], [2200.0, 1300.0], [-300.0, 1300.0]])
    mask = np.zeros((height, width), np.uint8)
    import cv2

    cv2.fillConvexPoly(mask, np.round(quad).astype(np.int32), 255)
    points = contour_points_off_border(mask, border_margin_px=4)
    assert points[:, 1].max() < height - 4
    lines = side_lines_from_points(points, quad)
    assert (
        lines[0] is not None and lines[1] is not None and lines[3] is not None
    )  # far, right, left
    assert lines[2] is None  # near edge is off the sensor
    far_normal, far_offset = lines[0]
    assert abs(abs(far_normal[1]) - 1.0) < 1e-3  # a horizontal line
    assert abs(far_offset * np.sign(far_normal[1]) - 100.0) < 1.5


def test_pixels_to_field_plane_inverts_projection(near_wall_camera: np.ndarray) -> None:
    from auto_battlebot.perception.field_pose import pixels_to_field_plane

    tf = camera_from_world(near_wall_camera)
    points = np.array([[0.3, -0.4, 0.0], [-0.8, 0.9, 0.0], [0.0, 0.0, 0.0]])
    pixels = project_field_corners(tf, MAT, K)  # any pixels work; use the corners plus these
    from auto_battlebot.perception.field_pose import project_points

    pixels = project_points(tf, points, K)
    recovered = pixels_to_field_plane(pixels, tf, K)
    assert np.allclose(recovered, points, atol=1e-6)
    # A pixel above the horizon misses the plane.
    sky = pixels_to_field_plane(np.array([[960.0, -5000.0]]), tf, K)
    assert np.isnan(sky).all()
