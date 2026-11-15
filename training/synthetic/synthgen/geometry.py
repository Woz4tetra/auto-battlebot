"""Pure geometric conversions and camera-framing math (no Blender imports)."""

import math

import numpy as np


def model_to_blender_local(vec: "list[float] | np.ndarray") -> np.ndarray:
    """Convert model-space (glTF Y-up) keypoints into Blender local (Z-up) axes."""
    x, y, z = [float(v) for v in vec]
    return np.array([x, -z, y], dtype=float)


def narrow_fov(sensor_fov: float, width_px: int, height_px: int) -> float:
    """Smaller of the horizontal and vertical field of view, in radians."""
    if width_px <= 0 or height_px <= 0:
        return sensor_fov
    short, long = (height_px, width_px) if width_px >= height_px else (width_px, height_px)
    narrow = 2.0 * math.atan(math.tan(sensor_fov / 2.0) * short / long)
    return min(sensor_fov, narrow)


def min_distance_for_frame_fraction(
    radius: float, min_fov: float, max_half_frame_fraction: float
) -> float:
    """Closest camera distance, in meters, that keeps a *radius* sphere within the fraction."""
    if radius <= 0.0 or min_fov <= 0.0 or max_half_frame_fraction <= 0.0:
        return 0.0
    k = max_half_frame_fraction * math.tan(min_fov / 2.0)
    if k <= 0.0:
        return 0.0
    return radius * math.sqrt(1.0 + k * k) / k
