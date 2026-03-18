"""Camera math utilities for the simulation."""

from __future__ import annotations

import math
from typing import Any, Sequence

import cv2
import numpy as np
import numpy.typing as npt


def fov_to_intrinsics(
    fov_deg: float, width: int, height: int
) -> tuple[float, float, float, float]:
    """Derive (fx, fy, cx, cy) from a vertical FOV and resolution.

    Genesis's camera FOV is vertical, so fy is computed from height.
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

    R: npt.NDArray[np.float64] = np.eye(4, dtype=np.float64)
    R[0, :3] = right
    R[1, :3] = -up
    R[2, :3] = forward
    t = -R[:3, :3] @ cam_pos_arr
    R[:3, 3] = t
    return R


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
    R_c2w = np.column_stack([right, up, forward])  # (3, 3)

    fx, fy, cx, cy = fov_to_intrinsics(fov_deg, width, height)

    # Pixel grid -> normalised camera-space ray directions
    u = np.arange(width, dtype=np.float64)
    v = np.arange(height, dtype=np.float64)
    uu, vv = np.meshgrid(u, v)  # (H, W)
    dirs_cam = np.stack(
        [(uu - cx) / fx, -(vv - cy) / fy, np.ones_like(uu)], axis=-1
    )  # (H, W, 3)
    dirs_cam /= np.linalg.norm(dirs_cam, axis=-1, keepdims=True)

    # Transform to world space
    dirs_world: npt.NDArray[np.float64] = dirs_cam @ R_c2w.T  # (H, W, 3)

    # World-space direction -> equirectangular UV
    dx = dirs_world[..., 0]
    dy = dirs_world[..., 1]
    dz = dirs_world[..., 2]
    lon = np.arctan2(dy, dx)  # [-pi, pi]
    lat = np.arcsin(np.clip(dz, -1, 1))  # [-pi/2, pi/2]

    pano_h, pano_w = pano_bgr.shape[:2]
    pano_u = ((lon / (2.0 * np.pi)) + 0.5) * (pano_w - 1)  # [0, W-1]
    pano_v = (0.5 - lat / np.pi) * (pano_h - 1)  # [0, H-1]

    result: npt.NDArray[np.uint8] = cv2.remap(
        pano_bgr,
        pano_u.astype(np.float32),
        pano_v.astype(np.float32),
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_WRAP,
    )
    return result
