"""Camera math utilities for the simulation."""

from __future__ import annotations

import math
from typing import Sequence, cast

import cv2
import numpy as np
import numpy.typing as npt


def fov_to_intrinsics(fov_deg: float, width: int, height: int) -> tuple[float, float, float, float]:
    """Derive (fx, fy, cx, cy) from a vertical FOV and resolution.

    Camera FOV is vertical, so fy is computed from height.
    fx = fy assumes square pixels.
    """
    fov_rad = math.radians(fov_deg)
    fy = (height / 2.0) / math.tan(fov_rad / 2.0)
    fx = fy
    cx = width / 2.0
    cy = height / 2.0
    return fx, fy, cx, cy


def camera_view_matrix(
    cam_pos: Sequence[float], lookat: Sequence[float]
) -> npt.NDArray[np.float64]:
    """Compute a 4x4 view matrix (camera-from-world)."""
    cam_pos_arr = np.array(cam_pos, dtype=np.float64)
    lookat_arr = np.array(lookat, dtype=np.float64)
    forward = lookat_arr - cam_pos_arr
    forward /= np.linalg.norm(forward)
    world_up = np.array([0.0, 0.0, 1.0])
    right = np.cross(forward, world_up)
    right_norm = np.linalg.norm(right)
    if right_norm < 1e-6:
        world_up = np.array([0.0, 1.0, 0.0])
        right = np.cross(forward, world_up)
        right_norm = np.linalg.norm(right)
    right /= right_norm
    up = np.cross(right, forward)

    rotation: npt.NDArray[np.float64] = np.eye(4, dtype=np.float64)
    rotation[0, :3] = right
    rotation[1, :3] = -up
    rotation[2, :3] = forward
    t = -rotation[:3, :3] @ cam_pos_arr
    rotation[:3, 3] = t
    return rotation


def project_panorama(
    pano_bgr: npt.NDArray[np.uint8],
    cam_pos: Sequence[float],
    lookat: Sequence[float],
    fov_deg: float,
    width: int,
    height: int,
) -> npt.NDArray[np.uint8]:
    """Project an equirectangular panorama to a perspective image (BGR).

    Builds a camera-to-world rotation from `cam_pos`/`lookat`, then for every
    output pixel computes the corresponding world-space ray direction, converts
    it to equirectangular (lon, lat) UVs, and samples the panorama.
    """
    cam_pos_arr = np.array(cam_pos, dtype=np.float64)
    lookat_arr = np.array(lookat, dtype=np.float64)

    forward = lookat_arr - cam_pos_arr
    forward /= np.linalg.norm(forward)
    world_up = np.array([0.0, 0.0, 1.0])
    right = np.cross(forward, world_up)
    right_norm = np.linalg.norm(right)
    if right_norm < 1e-6:
        world_up = np.array([0.0, 1.0, 0.0])
        right = np.cross(forward, world_up)
        right_norm = np.linalg.norm(right)
    right /= right_norm
    up = np.cross(right, forward)

    # camera-to-world rotation: columns are right, up, forward
    rotation_c2w = np.column_stack([right, up, forward])  # (3, 3)

    fx, fy, cx, cy = fov_to_intrinsics(fov_deg, width, height)

    # Pixel grid -> normalised camera-space ray directions
    u = np.arange(width, dtype=np.float64)
    v = np.arange(height, dtype=np.float64)
    uu, vv = np.meshgrid(u, v)  # (H, W)
    dirs_cam = np.stack([(uu - cx) / fx, -(vv - cy) / fy, np.ones_like(uu)], axis=-1)  # (H, W, 3)
    dirs_cam /= np.linalg.norm(dirs_cam, axis=-1, keepdims=True)

    # Transform to world space
    dirs_world: npt.NDArray[np.float64] = dirs_cam @ rotation_c2w.T  # (H, W, 3)

    # World-space direction -> equirectangular UV
    dx = dirs_world[..., 0]
    dy = dirs_world[..., 1]
    dz = dirs_world[..., 2]
    lon = np.arctan2(dy, dx)  # [-pi, pi]
    lat = np.arcsin(np.clip(dz, -1, 1))  # [-pi/2, pi/2]

    pano_h, pano_w = pano_bgr.shape[:2]
    pano_u = ((lon / (2.0 * np.pi)) + 0.5) * (pano_w - 1)  # [0, W-1]
    pano_v = (0.5 - lat / np.pi) * (pano_h - 1)  # [0, H-1]

    result = cv2.remap(
        pano_bgr,
        pano_u.astype(np.float32),
        pano_v.astype(np.float32),
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_WRAP,
    )
    return cast("npt.NDArray[np.uint8]", result)


def ground_plane_homography(
    view_matrix: npt.NDArray[np.float64],
    intrinsics: tuple[float, float, float, float],
    metres_per_pixel: float,
    top_down_size: int,
) -> npt.NDArray[np.float64]:
    """Homography taking top-down arena pixels to camera image pixels.

    The arena floor is a plane, so its image is an exact homography of a top-down render of it --
    no ray marching, no geometry. That is what lets the simulated camera reuse the viewer's
    composition instead of rendering the world a second way.

    Args:
        view_matrix: 4x4 camera-from-world, as built by `camera_view_matrix`.
        intrinsics: (fx, fy, cx, cy) for the output image.
        metres_per_pixel: scale of the top-down render.
        top_down_size: side length of the (square) top-down render, in pixels.

    Returns:
        A 3x3 matrix suitable for ``cv2.warpPerspective``.
    """
    fx, fy, cx, cy = intrinsics
    camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)

    # The floor is z = 0, so the third column of the rotation drops out and world-to-image
    # collapses from 3x4 to 3x3.
    rotation = view_matrix[:3, :3]
    translation = view_matrix[:3, 3]
    world_to_image = camera_matrix @ np.column_stack([rotation[:, 0], rotation[:, 1], translation])

    # Top-down pixels to field metres: x right, y up, origin at the centre of the render.
    half = top_down_size / 2.0
    scale = metres_per_pixel
    pixels_to_world = np.array(
        [[scale, 0.0, -half * scale], [0.0, -scale, half * scale], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    return world_to_image @ pixels_to_world


def ground_plane_depth(
    view_matrix: npt.NDArray[np.float64],
    intrinsics: tuple[float, float, float, float],
    width: int,
    height: int,
) -> npt.NDArray[np.float32]:
    """Per-pixel depth of the floor plane, in metres along the optical axis.

    Pixels whose ray points at or above the horizon get NaN, which is what a stereo camera reports
    where it has no match.
    """
    fx, fy, cx, cy = intrinsics
    rotation = view_matrix[:3, :3]
    translation = view_matrix[:3, 3]
    camera_position = -rotation.T @ translation

    u, v = np.meshgrid(np.arange(width, dtype=np.float64), np.arange(height, dtype=np.float64))
    rays_camera = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u)], axis=-1)
    rays_world = rays_camera @ rotation  # rotation.T applied per row

    # Distance along the ray to z = 0, then converted to depth along the optical axis.
    with np.errstate(divide="ignore", invalid="ignore"):
        t = -camera_position[2] / rays_world[..., 2]
    depth = np.where(t > 0.0, t, np.nan) / np.linalg.norm(rays_camera, axis=-1)
    result: npt.NDArray[np.float32] = depth.astype(np.float32)
    return result
