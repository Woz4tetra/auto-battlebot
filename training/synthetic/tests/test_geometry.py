"""Tests for pure geometry helpers."""

import math

import numpy as np
import pytest
from synthgen.geometry import (
    min_distance_for_frame_fraction,
    model_to_blender_local,
    narrow_fov,
)


class TestModelToBlenderLocal:
    def test_axis_remap(self) -> None:
        np.testing.assert_allclose(model_to_blender_local([1.0, 2.0, 3.0]), [1.0, -3.0, 2.0])

    def test_accepts_ndarray(self) -> None:
        np.testing.assert_allclose(
            model_to_blender_local(np.array([0.1115, 0.0, -0.00313])), [0.1115, 0.00313, 0.0]
        )


class TestNarrowFov:
    def test_landscape(self) -> None:
        fov = math.radians(90)
        result = narrow_fov(fov, 1280, 720)
        expected = 2.0 * math.atan(math.tan(fov / 2.0) * 720 / 1280)
        assert result == pytest.approx(expected)
        assert result < fov

    def test_square_resolution(self) -> None:
        fov = math.radians(60)
        assert narrow_fov(fov, 512, 512) == pytest.approx(fov)

    def test_degenerate_resolution(self) -> None:
        fov = math.radians(60)
        assert narrow_fov(fov, 0, 720) == pytest.approx(fov)


class TestMinDistanceForFrameFraction:
    def test_degenerate_inputs(self) -> None:
        assert min_distance_for_frame_fraction(0.0, 1.0, 0.9) == 0.0
        assert min_distance_for_frame_fraction(1.0, 0.0, 0.9) == 0.0
        assert min_distance_for_frame_fraction(1.0, 1.0, 0.0) == 0.0

    def test_closed_form_against_projection(self) -> None:
        """The silhouette of the sphere at the returned distance must land at
        exactly max_frame_fraction of the half-frame."""
        radius = 0.25
        fov = math.radians(70)
        fraction = 0.85
        distance = min_distance_for_frame_fraction(radius, fov, fraction)
        # Silhouette angle theta: sin(theta) = radius / distance.
        theta = math.asin(radius / distance)
        projected_fraction = math.tan(theta) / math.tan(fov / 2.0)
        assert projected_fraction == pytest.approx(fraction, rel=1e-9)

    def test_smaller_fraction_pushes_camera_back(self) -> None:
        d_tight = min_distance_for_frame_fraction(0.25, math.radians(70), 0.5)
        d_loose = min_distance_for_frame_fraction(0.25, math.radians(70), 0.9)
        assert d_tight > d_loose
