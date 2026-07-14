"""Tests for image post-processing."""

import math

import numpy as np
import pytest
from synthgen.imaging import apply_object_motion_blur, directional_blur_kernel


class TestDirectionalBlurKernel:
    def test_forced_odd_and_minimum_size(self) -> None:
        assert directional_blur_kernel(1, 0.0).shape == (3, 3)
        assert directional_blur_kernel(8, 0.0).shape == (9, 9)

    def test_normalized(self) -> None:
        kernel = directional_blur_kernel(15, 1.0)
        assert kernel.sum() == pytest.approx(1.0)

    def test_horizontal_direction(self) -> None:
        kernel = directional_blur_kernel(7, 0.0)
        center_row = kernel[3, :]
        assert np.count_nonzero(center_row) == 7
        assert np.count_nonzero(kernel) == 7

    def test_vertical_direction(self) -> None:
        kernel = directional_blur_kernel(7, math.pi / 2)
        center_col = kernel[:, 3]
        assert np.count_nonzero(center_col) == 7
        assert np.count_nonzero(kernel) == 7


class TestApplyObjectMotionBlur:
    def test_pixels_outside_dilated_mask_untouched(self) -> None:
        rng = np.random.default_rng(0)
        image = rng.integers(0, 255, size=(50, 50, 3), dtype=np.uint8)
        seg = np.zeros((50, 50), dtype=np.int32)
        seg[20:26, 20:26] = 1

        blurred = apply_object_motion_blur(image, seg, 1, kernel_size=5, angle=0.0)

        assert blurred.shape == image.shape
        # Far corners are well outside the 5px-dilated mask.
        np.testing.assert_array_equal(blurred[:10, :10], image[:10, :10])
        np.testing.assert_array_equal(blurred[-10:, -10:], image[-10:, -10:])

    def test_masked_region_changed(self) -> None:
        rng = np.random.default_rng(1)
        image = rng.integers(0, 255, size=(50, 50, 3), dtype=np.uint8)
        seg = np.zeros((50, 50), dtype=np.int32)
        seg[20:31, 20:31] = 1

        blurred = apply_object_motion_blur(image, seg, 1, kernel_size=9, angle=0.0)
        assert not np.array_equal(blurred[22:29, 22:29], image[22:29, 22:29])

    def test_absent_category_is_noop(self) -> None:
        rng = np.random.default_rng(2)
        image = rng.integers(0, 255, size=(30, 30, 3), dtype=np.uint8)
        seg = np.zeros((30, 30), dtype=np.int32)
        blurred = apply_object_motion_blur(image, seg, 5, kernel_size=5, angle=0.3)
        np.testing.assert_array_equal(blurred, image)
