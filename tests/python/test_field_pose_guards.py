"""Guards in field_pose: C++ parity, corner ordering, and the border check."""

from __future__ import annotations

import itertools
import math
import re
from pathlib import Path

import numpy as np
import pytest

from auto_battlebot.compat import tomllib
from auto_battlebot.perception import field_pose
from auto_battlebot.perception.field_pose import (
    _fit_line,
    _intersect_side_lines,
    align_field_x_with_camera_x,
    field_object_corners,
    fit_plane_ransac,
    order_corners,
    pixels_to_field_plane,
    rectangle_angle,
    touches_border,
    transform_from_plane,
)

REPO = Path(__file__).resolve().parents[2]


def test_ransac_defaults_match_the_cpp_config() -> None:
    header = (REPO / "include/field_filter/config.hpp").read_text()
    iterations = re.search(r"int ransac_max_iterations = (\d+);", header)
    probability = re.search(r"double ransac_probability = ([\d.]+);", header)
    assert iterations is not None and probability is not None
    assert field_pose.RANSAC_MAX_ITERATIONS == int(iterations.group(1))
    assert field_pose.RANSAC_PROBABILITY == float(probability.group(1))
    with open(REPO / "config/_common.toml", "rb") as handle:
        common = tomllib.load(handle)
    assert field_pose.DISTANCE_THRESHOLD_M == common["field_filter"]["distance_threshold"]


def test_up_vector_matches_the_cpp_default() -> None:
    header = (REPO / "include/field_filter/point_cloud_field_filter.hpp").read_text()
    match = re.search(r"up_vector = Eigen::Vector3f\(([^)]*)\)", header)
    assert match is not None
    values = [float(v.strip().rstrip("f")) for v in match.group(1).split(",")]
    assert field_pose.UP_VECTOR.tolist() == values


def test_rectangle_angle_takes_the_top_edge_and_the_angle_nearer_zero() -> None:
    theta = math.radians(20.0)
    c0 = np.array([0.0, 0.0])
    c1 = c0 + 4.0 * np.array([math.cos(theta), math.sin(theta)])
    c3 = c0 + 2.0 * np.array([-math.sin(theta), math.cos(theta)])
    c2 = c1 + c3 - c0
    corners = np.array([c0, c1, c2, c3])
    assert rectangle_angle(corners) == pytest.approx(theta)
    assert rectangle_angle(corners[::-1]) == pytest.approx(theta)


def test_transform_from_plane_is_the_shortest_arc_onto_the_normal() -> None:
    up = np.array([0.0, 0.0, -1.0])
    normal = np.array([0.3, -0.5, 0.8])
    normal_unit = normal / np.linalg.norm(normal)
    rotation = transform_from_plane(np.array([1.0, 2.0, 3.0]), normal, up)
    np.testing.assert_allclose(rotation[:3, :3] @ up, normal_unit, atol=1e-9)
    axis = np.cross(up, normal_unit)
    np.testing.assert_allclose(rotation[:3, :3] @ axis, axis, atol=1e-9)
    np.testing.assert_allclose(rotation[:3, 3], [1.0, 2.0, 3.0])


class _CountingRng:
    def __init__(self) -> None:
        self.inner = np.random.default_rng(0)
        self.draws = 0

    def choice(self, *args: object, **kwargs: object) -> np.ndarray:
        self.draws += 1
        return self.inner.choice(*args, **kwargs)  # type: ignore[arg-type]


def test_ransac_stops_early_once_the_adaptive_bound_is_met() -> None:
    gen = np.random.default_rng(1)
    plane = np.c_[gen.uniform(-1, 1, (300, 2)), np.zeros(300)]
    clutter = gen.uniform(-1, 1, (300, 3)) + np.array([0.0, 0.0, 2.0])
    rng = _CountingRng()
    model, inliers = fit_plane_ransac(np.vstack([plane, clutter]), rng=rng)  # type: ignore[arg-type]
    assert 10 < rng.draws < 200
    assert abs(abs(model[2]) - 1.0) < 1e-3
    assert len(inliers) >= 290


def test_touches_border_flags_a_mask_cut_off_by_the_frame() -> None:
    inside = np.zeros((50, 60), dtype=np.uint8)
    inside[10:40, 10:50] = 255
    assert not touches_border(inside)
    for index in ((0, slice(20, 30)), (-1, slice(20, 30)), (slice(20, 30), 0), (slice(20, 30), -1)):
        cut = inside.copy()
        cut[index] = 255
        assert touches_border(cut)


def test_order_corners_is_independent_of_input_order() -> None:
    quad = np.array([[10.0, 10.0], [90.0, 14.0], [95.0, 80.0], [5.0, 70.0]])
    expected = order_corners(quad)
    np.testing.assert_allclose(expected, quad)
    for perm in itertools.permutations(range(4)):
        np.testing.assert_allclose(order_corners(quad[list(perm)]), expected)


def test_side_line_intersections_come_back_in_corner_order() -> None:
    quad = order_corners(np.array([[0.0, 0.0], [100.0, 5.0], [95.0, 60.0], [-5.0, 50.0]]))
    lines = []
    for i in range(4):
        a, b = quad[i], quad[(i + 1) % 4]
        lines.append(_fit_line(np.linspace(a, b, 30)))
    corners = _intersect_side_lines(lines)
    assert corners is not None
    np.testing.assert_allclose(corners, quad, atol=1e-6)


def test_object_corners_use_the_cpp_winding() -> None:
    source = (REPO / "src/field_filter/field_pose.cpp").read_text()
    body = source.split("field_object_corners(double size_x, double size_y)")[1].split("}")[0]
    signs = re.findall(r"Point2d\((-?)half_x, (-?)half_y\)", body)
    expected = [(-1 if sx else 1, -1 if sy else 1) for sx, sy in signs]
    got = field_object_corners((2.0, 2.0))
    assert [(int(np.sign(x)), int(np.sign(y))) for x, y in got] == expected


def _camera_from_field(turn: float) -> np.ndarray:
    pitch = 2.0
    base = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, math.cos(pitch), -math.sin(pitch)],
            [0.0, math.sin(pitch), math.cos(pitch)],
        ]
    )
    spin = np.array(
        [
            [math.cos(turn), -math.sin(turn), 0.0],
            [math.sin(turn), math.cos(turn), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    out = np.eye(4)
    out[:3, :3] = base @ spin
    return out


def test_alignment_picks_one_rotation_for_every_quarter_turn_of_a_square() -> None:
    expected = align_field_x_with_camera_x(_camera_from_field(0.1), (2.0, 2.0))
    for k in range(4):
        got = align_field_x_with_camera_x(_camera_from_field(0.1 + k * math.pi / 2), (2.0, 2.0))
        np.testing.assert_allclose(got, expected, atol=1e-9)


def test_alignment_of_a_rectangle_only_allows_half_turns() -> None:
    expected = align_field_x_with_camera_x(_camera_from_field(0.1), (3.0, 2.0))
    flipped = align_field_x_with_camera_x(_camera_from_field(0.1 + math.pi), (3.0, 2.0))
    np.testing.assert_allclose(flipped, expected, atol=1e-9)
    quarter = align_field_x_with_camera_x(_camera_from_field(0.1 + math.pi / 2), (3.0, 2.0))
    assert not np.allclose(quarter, expected, atol=1e-6)


@pytest.mark.parametrize("pixels", [np.zeros(2), np.zeros((4, 3)), np.zeros((2, 2, 2))])
def test_pixels_to_field_plane_rejects_bad_pixel_shape(pixels: np.ndarray) -> None:
    with pytest.raises(ValueError, match="pixels"):
        pixels_to_field_plane(pixels, np.eye(4), np.eye(3))


def test_pixels_to_field_plane_rejects_bad_intrinsics_shape() -> None:
    with pytest.raises(ValueError, match="intrinsics"):
        pixels_to_field_plane(np.zeros((1, 2)), np.eye(4), np.eye(4))
