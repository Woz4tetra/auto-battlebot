"""Cage-mount poses put the camera where the mount says, looking where it says.

The cross-checks go through `auto_battlebot.perception.cage_calibration`, which owns the frame
conventions: a pose built here, converted back to `T_W<-cam_cv` and summarised, must report the
height, tilt and yaw that were asked for. That pins `cage_mount`'s hand-written Blender camera
basis against the repo's own definition of those frames.
"""

from __future__ import annotations

import math
import random

import numpy as np
import pytest
from synthgen.cage_mount import (
    WALLS,
    CageMount,
    CageMountRanges,
    mount_cam2world,
    mount_forward,
    mount_position,
    sample_cage_mount,
    wall_axes,
)

from auto_battlebot.perception.cage_calibration import OPENCV_TO_BLENDER_CAMERA, pose_summary

WALL_HALF = 2.4384 / 2
# The May 2026 fit: 1.16 m above the mat, 0.11 m inside the near wall, 31.9 deg off straight
# down, square to the wall.
FITTED = CageMount("near", 0.02, 1.16, 0.109, 31.9, 0.0, 0.0)


def summarize(mount: CageMount) -> dict[str, float]:
    """The pose's height/tilt/yaw as `cage_calibration` reports them."""
    cam2world = mount_cam2world(mount, WALL_HALF)
    world_from_camera = cam2world @ np.linalg.inv(OPENCV_TO_BLENDER_CAMERA)
    return pose_summary(world_from_camera)


def test_fitted_mount_round_trips_through_pose_summary() -> None:
    summary = summarize(FITTED)
    assert summary["height_m"] == pytest.approx(1.16)
    assert summary["x_m"] == pytest.approx(0.02)
    assert summary["y_m"] == pytest.approx(-(WALL_HALF - 0.109))
    assert summary["tilt_from_down_deg"] == pytest.approx(31.9)
    assert summary["yaw_deg"] == pytest.approx(0.0)


@pytest.mark.parametrize("wall", WALLS)
def test_every_wall_looks_inward_and_down(wall: str) -> None:
    mount = CageMount(wall, 0.1, 1.2, 0.1, 33.0, 4.0, 2.0)
    normal, _ = wall_axes(mount.wall)
    position = mount_position(mount, WALL_HALF)
    forward = mount_forward(mount)

    # Inside the cage, above the mat, and aimed across it and down.
    assert np.linalg.norm(position[:2]) < WALL_HALF * math.sqrt(2)
    assert position[2] == pytest.approx(1.2)
    assert float(np.dot(position[:2], normal[:2])) < 0  # the wall is opposite the inward normal
    assert float(np.dot(forward, normal)) > 0
    assert forward[2] < 0
    assert summarize(mount)["tilt_from_down_deg"] == pytest.approx(33.0)


def test_yaw_turns_toward_the_along_axis() -> None:
    summary = summarize(CageMount("near", 0.0, 1.2, 0.1, 32.0, 7.0, 0.0))
    # On the near wall the along axis is +x, and pose_summary's yaw is positive toward +x.
    assert summary["yaw_deg"] == pytest.approx(7.0)


def test_cam2world_is_a_right_handed_rotation() -> None:
    rotation = mount_cam2world(CageMount("left", -0.3, 1.3, 0.0, 40.0, -6.0, 3.0), WALL_HALF)[
        :3, :3
    ]
    assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9)
    assert float(np.linalg.det(rotation)) == pytest.approx(1.0)


def test_roll_rotates_the_image_axes_only() -> None:
    upright = mount_cam2world(CageMount("far", 0.0, 1.2, 0.1, 32.0, 0.0, 0.0), WALL_HALF)
    rolled = mount_cam2world(CageMount("far", 0.0, 1.2, 0.1, 32.0, 0.0, 10.0), WALL_HALF)
    assert np.allclose(upright[:3, 2], rolled[:3, 2], atol=1e-9)  # same optical axis
    assert np.allclose(upright[:3, 3], rolled[:3, 3], atol=1e-9)  # same position
    angle = math.degrees(math.acos(float(np.clip(np.dot(upright[:3, 1], rolled[:3, 1]), -1, 1))))
    assert angle == pytest.approx(10.0, abs=1e-6)


def test_straight_down_mount_still_builds_a_valid_basis() -> None:
    rotation = mount_cam2world(CageMount("near", 0.0, 1.4, 0.1, 0.0, 0.0, 0.0), WALL_HALF)[:3, :3]
    assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9)
    assert np.allclose(rotation[:, 2], [0.0, 0.0, 1.0], atol=1e-9)  # -forward is +z


def test_samples_stay_inside_their_ranges() -> None:
    random.seed(0)
    ranges = CageMountRanges(walls=("near", "right"))
    for _ in range(200):
        mount = sample_cage_mount(ranges, 1.2192)
        assert mount.wall in ranges.walls
        for value, (lo, hi) in (
            (mount.along_m, ranges.along_m),
            (mount.height_m, ranges.height_m),
            (mount.inset_m, ranges.inset_m),
            (mount.tilt_deg, ranges.tilt_deg),
            (mount.yaw_deg, ranges.yaw_deg),
            (mount.roll_deg, ranges.roll_deg),
        ):
            assert lo <= value <= hi


def test_unknown_wall_is_rejected() -> None:
    with pytest.raises(ValueError, match="unknown cage wall"):
        wall_axes("ceiling")
