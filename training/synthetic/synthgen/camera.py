"""Camera pose sampling (requires Blender).

The sampler guarantees an on-target pose: it never silently returns a pose
where the look-at target is out of frame. Retry exhaustion escalates from noise
resampling to geometry resampling (with halved noise) to a deterministic
zero-noise fallback aimed at the target, which always frames it.
"""

import math
import random

import blenderproc as bproc
import bpy
import mathutils
import numpy as np
from bpy_extras.object_utils import world_to_camera_view

from synthgen.configuration import CameraConfig
from synthgen.constants import CAMERA_GEOMETRY_RESAMPLES, CAMERA_TARGET_RETRIES
from synthgen.distractors import DistractorInstance, clear_blocking_distractors
from synthgen.geometry import min_distance_for_frame_fraction, narrow_fov
from synthgen.logsetup import get_logger
from synthgen.robots import RobotInstance

logger = get_logger(__name__)


def _sample_camera_position(
    look_at: list[float], min_dist: float, max_dist: float, height_range: tuple[float, float]
) -> np.ndarray:
    """Sample a camera position on a shell around the look-at point."""
    distance = random.uniform(min_dist, max_dist)
    azimuth = random.uniform(0, 2 * math.pi)
    height = random.uniform(height_range[0], height_range[1])
    return np.array(
        [
            look_at[0] + distance * math.cos(azimuth),
            look_at[1] + distance * math.sin(azimuth),
            height,
        ]
    )


def _pose_toward(cam_pos: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Build a cam2world matrix at *cam_pos* looking at *target*."""
    forward = target - cam_pos
    forward = forward / np.linalg.norm(forward)
    rotation = bproc.camera.rotation_from_forward_vec(forward)
    return np.asarray(bproc.math.build_transformation_mat(cam_pos, rotation))


def _target_in_frame(cam2world: np.ndarray, point: list[float]) -> bool:
    """True when *point* projects inside the camera frame for this pose."""
    camera = bpy.context.scene.camera
    camera.matrix_world = mathutils.Matrix(cam2world.tolist())
    bpy.context.view_layer.update()
    co_2d = world_to_camera_view(bpy.context.scene, camera, mathutils.Vector(point))
    return bool(0 <= co_2d.x <= 1 and 0 <= co_2d.y <= 1 and co_2d.z > 0)


def sample_camera_pose(
    look_at: list[float],
    min_dist: float,
    max_dist: float,
    height_range: tuple[float, float],
    noise: float,
    robot_center: list[float] | None = None,
) -> tuple[np.ndarray, bool]:
    """Sample a camera pose on a shell looking at a target point.

    When *robot_center* is provided, the look-at noise is resampled (up to
    ``CAMERA_TARGET_RETRIES`` times) until the robot center projects inside the
    camera frame. On exhaustion the camera geometry itself is resampled with
    halved noise (up to ``CAMERA_GEOMETRY_RESAMPLES`` rounds); the final
    fallback is a deterministic zero-noise pose aimed straight at the target,
    which is guaranteed to frame it.

    Args:
        look_at: Center of the camera position shell.
        min_dist: Minimum camera distance (meters).
        max_dist: Maximum camera distance (meters).
        height_range: Camera height range (meters).
        noise: Gaussian noise (meters) applied to the look-at target.
        robot_center: Point that must project in frame, or None to skip the
            acceptance test.

    Returns:
        ``(cam2world, used_fallback)`` where used_fallback marks the
        deterministic zero-noise escape hatch.
    """
    cam_pos = _sample_camera_position(look_at, min_dist, max_dist, height_range)

    if robot_center is None:
        target = np.array(
            [
                look_at[0] + random.gauss(0, noise),
                look_at[1] + random.gauss(0, noise),
                look_at[2] + random.gauss(0, noise),
            ]
        )
        return _pose_toward(cam_pos, target), False

    round_noise = noise
    for geometry_round in range(CAMERA_GEOMETRY_RESAMPLES + 1):
        if geometry_round > 0:
            cam_pos = _sample_camera_position(look_at, min_dist, max_dist, height_range)
            round_noise *= 0.5
        for _ in range(CAMERA_TARGET_RETRIES):
            target = np.array(
                [
                    look_at[0] + random.gauss(0, round_noise),
                    look_at[1] + random.gauss(0, round_noise),
                    look_at[2] + random.gauss(0, round_noise),
                ]
            )
            cam2world = _pose_toward(cam_pos, target)
            if _target_in_frame(cam2world, robot_center):
                return cam2world, False

    # Deterministic fallback: aim straight at the target with zero noise. The
    # target then projects at the frame center, so the acceptance test holds.
    cam2world = _pose_toward(cam_pos, np.array(robot_center, dtype=float))
    return cam2world, True


def camera_min_fov() -> float:
    """Smaller of the horizontal/vertical camera field of view, in radians."""
    scene = bpy.context.scene
    sensor_fov = float(scene.camera.data.angle)  # FOV along the longer image dimension
    return narrow_fov(sensor_fov, int(scene.render.resolution_x), int(scene.render.resolution_y))


def scene_bounding_radius(scene_robots: list[RobotInstance], centroid: list[float]) -> float:
    """World-space radius of the sphere around *centroid* enclosing all robot meshes."""
    center = mathutils.Vector(centroid)
    max_r2 = 0.0
    for robot in scene_robots:
        for mesh in robot.meshes:
            obj = mesh.blender_obj
            for corner in obj.bound_box:
                world_corner = obj.matrix_world @ mathutils.Vector(corner)
                max_r2 = max(max_r2, (world_corner - center).length_squared)
    return math.sqrt(max_r2)


def setup_scene_cameras(
    scene_robots: list[RobotInstance],
    cam_cfg: CameraConfig,
    images_per_scene: int,
    remaining: int,
    active_distractors: list[DistractorInstance],
    robot_positions: list[list[float]],
    ignore_obstructions: bool,
) -> tuple[list[np.ndarray], int, int]:
    """Sample camera poses looking at the robot centroid and clear blocking distractors.

    Returns:
        ``(cam_poses, cam_count, fallback_count)`` where fallback_count is the
        number of poses that needed the deterministic zero-noise fallback.
    """
    centroid = [sum(p[i] for p in robot_positions) / len(robot_positions) for i in range(3)]
    cam_count = min(images_per_scene, remaining)

    # Push the camera back far enough that the robots' bounding sphere can never
    # dominate the frame (near-full-frame robots produce confusing annotations).
    min_dist = cam_cfg.min_distance
    max_dist = cam_cfg.max_distance
    radius = scene_bounding_radius(scene_robots, centroid)
    safe_min_dist = min_distance_for_frame_fraction(
        radius, camera_min_fov(), cam_cfg.max_frame_fraction
    )
    min_dist = max(min_dist, safe_min_dist)
    max_dist = max(max_dist, min_dist)

    cam_poses: list[np.ndarray] = []
    fallback_count = 0
    for _ in range(cam_count):
        pose, used_fallback = sample_camera_pose(
            look_at=centroid,
            min_dist=min_dist,
            max_dist=max_dist,
            height_range=cam_cfg.height_range,
            noise=cam_cfg.look_at_noise,
            robot_center=centroid,
        )
        if used_fallback:
            fallback_count += 1
            logger.warning("Camera target retries exhausted; using deterministic centered pose")
        cam_poses.append(pose)

    if ignore_obstructions:
        return cam_poses, cam_count, fallback_count

    all_keypoints_world: list[mathutils.Vector] = []
    for robot in scene_robots:
        wmat = np.array(robot.parent.matrix_world)
        for kp in [robot.kp_front, robot.kp_back]:
            all_keypoints_world.append(mathutils.Vector((wmat @ np.append(kp, 1.0))[:3]))
    clear_blocking_distractors(cam_poses, all_keypoints_world, active_distractors)
    return cam_poses, cam_count, fallback_count
