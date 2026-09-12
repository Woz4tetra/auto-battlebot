"""Flying the camera off the wall and projecting it back must not change the convention.

`freefly` is the only place a pose can leave `CageMount`'s five-numbers-and-a-wall shape, so these
tests pin both directions: a mount pose survives a round trip through the free-flight
parameterisation unchanged, and a free pose projects onto the mount that actually describes it,
including on the wrong-sign case (a mount is on the wall *behind* the lens, not the one its optical
axis points at) and the near-vertical case where yaw and roll collapse into one rotation.
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
    sample_cage_mount,
    wall_axes,
)
from synthgen.freefly import (
    PITCH_LIMIT_DEG,
    FreeflyPose,
    clamp_mount_to_ranges,
    clamp_pose,
    format_mount_ranges_toml,
    freefly_cam2world,
    freefly_forward,
    freefly_from_cam2world,
    mount_from_cam2world,
    mount_residual,
    ranges_covering,
)

WALL_HALF = 2.4384 / 2
FITTED = CageMount("near", 0.02, 1.16, 0.109, 31.9, 0.0, 0.0)


def test_a_mount_pose_survives_the_freefly_round_trip() -> None:
    cam2world = mount_cam2world(CageMount("left", -0.31, 1.28, 0.07, 34.5, -6.0, 2.5), WALL_HALF)
    recovered = freefly_from_cam2world(cam2world)
    assert np.allclose(freefly_cam2world(recovered), cam2world, atol=1e-12)


def test_the_fitted_nhrl_mount_comes_back_as_itself() -> None:
    mount = mount_from_cam2world(mount_cam2world(FITTED, WALL_HALF), WALL_HALF)
    assert mount.wall == FITTED.wall
    assert mount.along_m == pytest.approx(FITTED.along_m)
    assert mount.height_m == pytest.approx(FITTED.height_m)
    assert mount.inset_m == pytest.approx(FITTED.inset_m)
    assert mount.tilt_deg == pytest.approx(FITTED.tilt_deg)
    assert mount.yaw_deg == pytest.approx(FITTED.yaw_deg, abs=1e-9)
    assert mount.roll_deg == pytest.approx(FITTED.roll_deg, abs=1e-9)


def test_every_sampled_mount_inverts_exactly() -> None:
    random.seed(7)
    ranges = CageMountRanges()
    worst = 0.0
    for _ in range(5000):
        mount = sample_cage_mount(ranges, WALL_HALF)
        cam2world = mount_cam2world(mount, WALL_HALF)
        recovered = mount_from_cam2world(cam2world, WALL_HALF)
        assert recovered.wall == mount.wall
        for name in ("along_m", "height_m", "inset_m", "tilt_deg", "yaw_deg", "roll_deg"):
            worst = max(worst, abs(getattr(recovered, name) - getattr(mount, name)))
    assert worst < 1e-9


def test_the_wall_is_the_one_behind_the_lens_not_the_one_ahead() -> None:
    """The sign that cost a render run: the mount's wall is opposite its inward normal."""
    for wall in WALLS:
        mount = CageMount(wall, 0.1, 1.2, 0.1, 33.0, 4.0, 0.0)
        cam2world = mount_cam2world(mount, WALL_HALF)
        normal, _ = wall_axes(wall)
        forward = -cam2world[:3, 2]
        # The camera sits against its wall and looks across the mat, so the optical axis points
        # along the inward normal while the lens position sits against the opposite one.
        assert float(np.dot(forward, normal)) > 0
        assert float(np.dot(cam2world[:3, 3], normal)) < 0
        assert mount_from_cam2world(cam2world, WALL_HALF).wall == wall


def test_any_flown_pose_reads_back_as_an_exact_mount() -> None:
    """Nothing is lost going free pose to mount: the mount parameterisation spans all of SE(3)."""
    flown = FreeflyPose(x_m=0.3, y_m=-0.4, z_m=1.5, yaw_deg=10.0, pitch_deg=-50.0, roll_deg=6.0)
    cam2world = freefly_cam2world(flown)
    mount = mount_from_cam2world(cam2world, WALL_HALF)
    residual = mount_residual(cam2world, mount, WALL_HALF)

    assert residual.position_m == pytest.approx(0.0, abs=1e-12)
    assert residual.angle_deg == pytest.approx(0.0, abs=1e-9)
    # Flown well inside the cage, so the mount says so with a large inset rather than by failing.
    assert mount.wall == "near"
    assert mount.inset_m == pytest.approx(WALL_HALF - 0.4)


def test_clamping_into_ranges_is_what_produces_a_real_residual() -> None:
    flown = FreeflyPose(x_m=0.3, y_m=-0.4, z_m=1.5, yaw_deg=0.0, pitch_deg=-50.0, roll_deg=0.0)
    cam2world = freefly_cam2world(flown)
    ranges = CageMountRanges(walls=("near",))
    mount = mount_from_cam2world(cam2world, WALL_HALF, walls=ranges.walls)
    clamped = clamp_mount_to_ranges(mount, ranges)
    residual = mount_residual(cam2world, clamped, WALL_HALF)

    assert clamped.inset_m == pytest.approx(ranges.inset_m[1])  # flown too far in, pulled back
    assert clamped.height_m == pytest.approx(ranges.height_m[1])  # and too high
    assert residual.position_m > 0.5
    assert residual.angle_deg == pytest.approx(0.0, abs=1e-9)  # tilt 40 deg is already in range


def test_clamping_a_mount_already_inside_the_ranges_changes_nothing() -> None:
    ranges = CageMountRanges()
    clamped = clamp_mount_to_ranges(FITTED, ranges)
    assert clamped == FITTED


def test_clamping_moves_a_disallowed_wall_onto_an_allowed_one() -> None:
    ranges = CageMountRanges(walls=("far",))
    assert clamp_mount_to_ranges(FITTED, ranges).wall == "far"


def test_a_mount_pose_has_no_residual() -> None:
    cam2world = mount_cam2world(FITTED, WALL_HALF)
    residual = mount_residual(cam2world, mount_from_cam2world(cam2world, WALL_HALF), WALL_HALF)
    assert residual.position_m == pytest.approx(0.0, abs=1e-12)
    assert residual.angle_deg == pytest.approx(0.0, abs=1e-9)


def test_a_near_vertical_pose_still_round_trips_as_a_matrix() -> None:
    """Straight down is gimbal lock: yaw and roll are one rotation, so only the pose survives."""
    for tilt_deg in (0.0, 0.5, 2.0):
        mount = CageMount("far", 0.0, 1.4, 0.1, tilt_deg, 12.0, 0.0)
        cam2world = mount_cam2world(mount, WALL_HALF)

        recovered = mount_from_cam2world(cam2world, WALL_HALF)
        assert np.allclose(mount_cam2world(recovered, WALL_HALF), cam2world, atol=1e-9)

        flown = freefly_from_cam2world(cam2world)
        assert np.allclose(freefly_cam2world(flown), cam2world, atol=1e-9)


def test_straight_down_freefly_pitch_is_minus_ninety() -> None:
    forward = freefly_forward(yaw_deg=40.0, pitch_deg=-90.0)
    assert np.allclose(forward, [0.0, 0.0, -1.0], atol=1e-12)


def test_clamp_keeps_pitch_off_the_poles_and_wraps_the_rest() -> None:
    clamped = clamp_pose(FreeflyPose(0.0, 0.0, 1.0, 370.0, -120.0, -400.0))
    assert clamped.pitch_deg == pytest.approx(-PITCH_LIMIT_DEG)
    assert clamped.yaw_deg == pytest.approx(10.0)
    assert clamped.roll_deg == pytest.approx(-40.0)

    assert clamp_pose(FreeflyPose(0.0, 0.0, 1.0, 0.0, 95.0, 0.0)).pitch_deg == pytest.approx(
        PITCH_LIMIT_DEG
    )
    assert clamp_pose(FreeflyPose(0.0, 0.0, 1.0, 180.0, 0.0, 0.0)).yaw_deg == pytest.approx(180.0)


def test_ranges_cover_every_marked_mount_plus_the_pads() -> None:
    mounts = [FITTED, CageMount("left", -0.30, 1.32, 0.05, 36.0, -4.0, 1.0)]
    ranges = ranges_covering(mounts, pad_m=0.02, pad_deg=1.0)

    assert set(ranges.walls) == {"near", "left"}
    assert ranges.height_m == pytest.approx((1.14, 1.34))
    assert ranges.tilt_deg == pytest.approx((30.9, 37.0))
    for mount in mounts:
        for name in ("along_m", "height_m", "inset_m", "tilt_deg", "yaw_deg", "roll_deg"):
            low, high = getattr(ranges, name)
            assert low <= getattr(mount, name) <= high


def test_ranges_need_at_least_one_mount() -> None:
    with pytest.raises(ValueError, match="no mounts marked"):
        ranges_covering([])


def test_the_ranges_block_parses_back_as_the_same_ranges() -> None:
    import tomllib
    from synthgen.configuration import _parse_mount

    ranges = ranges_covering([FITTED], pad_m=0.02, pad_deg=1.0)
    text = format_mount_ranges_toml(ranges, header="marked with pose_camera_server.py")
    parsed = _parse_mount(tomllib.loads(text)["cages"]["mount"])

    assert parsed.walls == ranges.walls
    assert parsed.aim == ranges.aim
    for name in ("along_m", "height_m", "inset_m", "tilt_deg", "yaw_deg", "roll_deg"):
        assert getattr(parsed, name) == pytest.approx(getattr(ranges, name), abs=1e-4)


def test_freefly_yaw_matches_the_pose_summary_convention() -> None:
    from auto_battlebot.perception.cage_calibration import OPENCV_TO_BLENDER_CAMERA, pose_summary

    pose = FreeflyPose(0.1, -0.9, 1.2, yaw_deg=7.0, pitch_deg=-58.0, roll_deg=0.0)
    cam2world = freefly_cam2world(pose)
    summary = pose_summary(cam2world @ np.linalg.inv(OPENCV_TO_BLENDER_CAMERA))

    assert summary["yaw_deg"] == pytest.approx(7.0)
    assert summary["tilt_from_down_deg"] == pytest.approx(90.0 - 58.0)
    assert summary["height_m"] == pytest.approx(1.2)
    assert math.isclose(summary["x_m"], 0.1)
