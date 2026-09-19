"""The house bot's per-scene pose: on the mat, clear of the robots, any heading."""

from __future__ import annotations

import math
import random

import pytest
from synthgen.house_bot_pose import footprint_radius, sample_house_bot_pose

MAT_HALF = 2.35 / 2
RADIUS = footprint_radius((0.36, 0.36))


def test_footprint_stays_on_the_mat() -> None:
    rng = random.Random(0)
    for _ in range(500):
        pose = sample_house_bot_pose(MAT_HALF, RADIUS, [], 0.2, 0.05, rng)
        assert abs(pose.x_m) + RADIUS <= MAT_HALF + 1e-9
        assert abs(pose.y_m) + RADIUS <= MAT_HALF + 1e-9
        assert 0.0 <= pose.yaw_deg < 360.0


def test_keeps_clear_of_the_robots() -> None:
    robots = [[0.0, 0.0, 0.05], [0.4, -0.3, 0.05], [-0.45, 0.35, 0.05]]
    rng = random.Random(1)
    for _ in range(200):
        pose = sample_house_bot_pose(MAT_HALF, RADIUS, robots, 0.2, 0.05, rng)
        assert pose.clearance_margin_m >= 0.0
        for x, y, _ in robots:
            assert math.hypot(pose.x_m - x, pose.y_m - y) >= RADIUS + 0.2 + 0.05 - 1e-9


def test_spreads_over_the_whole_mat() -> None:
    rng = random.Random(2)
    poses = [sample_house_bot_pose(MAT_HALF, RADIUS, [], 0.2, 0.05, rng) for _ in range(400)]
    # Every quadrant is used, not one fitted corner.
    quadrants = {(p.x_m > 0, p.y_m > 0) for p in poses}
    assert len(quadrants) == 4


def test_crowded_mat_keeps_the_roomiest_draw() -> None:
    robots = [[x, y, 0.0] for x in (-0.8, -0.4, 0.0, 0.4, 0.8) for y in (-0.8, -0.4, 0.0, 0.4, 0.8)]
    pose = sample_house_bot_pose(MAT_HALF, RADIUS, robots, 0.2, 0.05, random.Random(3))
    assert pose.clearance_margin_m < 0.0


def test_too_big_for_the_mat_is_rejected() -> None:
    with pytest.raises(ValueError):
        sample_house_bot_pose(0.1, 0.2, [], 0.2, 0.05)
