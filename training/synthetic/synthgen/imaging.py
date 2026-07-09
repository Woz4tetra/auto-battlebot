"""Image post-processing effects applied after rendering (no Blender imports)."""

import math

import cv2
import numpy as np


def directional_blur_kernel(kernel_size: int, angle: float) -> np.ndarray:
    """Build a normalized 1-D directional blur kernel.

    Args:
        kernel_size: Requested kernel edge length in pixels; forced odd and >= 3.
        angle: Blur direction in radians.

    Returns:
        A square float32 kernel that sums to 1.
    """
    ks = max(3, kernel_size) | 1
    kernel = np.zeros((ks, ks), dtype=np.float32)
    cx = ks // 2
    dx, dy = math.cos(angle), math.sin(angle)
    for i in range(ks):
        t = i - cx
        x = int(round(cx + t * dx))
        y = int(round(cx + t * dy))
        if 0 <= x < ks and 0 <= y < ks:
            kernel[y, x] = 1.0
    kernel /= max(kernel.sum(), 1.0)
    return kernel


def apply_object_motion_blur(
    image: np.ndarray,
    seg_map: np.ndarray,
    category_id: int,
    kernel_size: int,
    angle: float,
) -> np.ndarray:
    """Apply directional motion blur to pixels of a single *category_id*.

    A linear blur kernel of *kernel_size* pixels at *angle* radians is
    applied to the full image.  The segmentation mask (dilated by the kernel
    radius) selects which output pixels come from the blurred vs. original
    image, so only that object's region shows the streak.

    Args:
        image: RGB frame to blur.
        seg_map: Category-id segmentation map aligned with the frame.
        category_id: Which category's pixels receive the blur.
        kernel_size: Blur streak length in pixels.
        angle: Blur direction in radians.

    Returns:
        The frame with the blur composited in.
    """
    kernel = directional_blur_kernel(kernel_size, angle)
    ks = kernel.shape[0]

    seg_2d = seg_map.squeeze()
    mask = (seg_2d == category_id).astype(np.uint8) * 255

    dilate_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ks, ks))
    mask = np.asarray(cv2.dilate(mask, dilate_k, iterations=1))
    mask_f = mask[:, :, np.newaxis].astype(np.float32) / 255.0

    blurred = np.asarray(cv2.filter2D(image, -1, kernel))
    composited: np.ndarray = (blurred * mask_f + image * (1.0 - mask_f)).astype(np.uint8)
    return composited
