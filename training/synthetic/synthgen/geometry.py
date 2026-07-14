"""Pure geometric conversions and camera-framing math (no Blender imports)."""

import math

import numpy as np


def model_to_blender_local(vec: "list[float] | np.ndarray") -> np.ndarray:
    """Convert model-space keypoints (OnShape/GLTF) into Blender local axes.

    GLTF import applies axis conversion (Y-up -> Z-up), so keypoints authored in
    the model's native frame must be remapped before projection/visibility tests.

    Args:
        vec: XYZ position in the model's native (GLTF, Y-up) frame.

    Returns:
        The position in Blender local (Z-up) axes.
    """
    x, y, z = [float(v) for v in vec]
    return np.array([x, -z, y], dtype=float)


def narrow_fov(sensor_fov: float, width_px: int, height_px: int) -> float:
    """Smaller of the horizontal/vertical field of view, in radians.

    Args:
        sensor_fov: Camera FOV along the longer image dimension (radians).
        width_px: Render width in pixels.
        height_px: Render height in pixels.

    Returns:
        The FOV along the narrower image dimension (radians); ``sensor_fov``
        unchanged when the resolution is degenerate.
    """
    if width_px <= 0 or height_px <= 0:
        return sensor_fov
    short, long = (height_px, width_px) if width_px >= height_px else (width_px, height_px)
    narrow = 2.0 * math.atan(math.tan(sensor_fov / 2.0) * short / long)
    return min(sensor_fov, narrow)


def min_distance_for_frame_fraction(
    radius: float, min_fov: float, max_frame_fraction: float
) -> float:
    """Closest camera distance that keeps a *radius* sphere within *max_frame_fraction*.

    Limits the projected silhouette of the robots' bounding sphere to
    *max_frame_fraction* of the narrower image dimension so a robot can never
    completely fill the frame, regardless of its size or the camera height.

    Args:
        radius: Bounding-sphere radius (meters).
        min_fov: Narrower camera field of view (radians).
        max_frame_fraction: Fraction of the half-frame the silhouette may fill.

    Returns:
        The minimum camera distance in meters (0.0 for degenerate inputs).
    """
    if radius <= 0.0 or min_fov <= 0.0 or max_frame_fraction <= 0.0:
        return 0.0
    # On the image plane a point at angle theta lands at tan(theta)/tan(fov/2) of
    # the half-frame. Solving tan(theta) = max_frame_fraction * tan(fov/2) for the
    # sphere silhouette (sin(theta) = radius/distance) gives this closed form.
    k = max_frame_fraction * math.tan(min_fov / 2.0)
    if k <= 0.0:
        return 0.0
    return radius * math.sqrt(1.0 + k * k) / k
