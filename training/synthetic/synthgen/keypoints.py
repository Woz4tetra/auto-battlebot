"""Keypoint projection and annotation building (requires Blender).

Pixel statistics are measured here; the keep/drop decisions themselves live in
the pure ``synthgen.gating`` module so they stay unit-testable.
"""

import bpy
import mathutils
import numpy as np
from bpy_extras.object_utils import world_to_camera_view

from synthgen.annotations import (
    YoloAnnotation,
    bbox_from_category_segmap,
    check_keypoint_visibility,
)
from synthgen.constants import (
    MIN_KEYPOINT_BBOX_DIM_PX,
    NHRL_DISTRACTOR_INSTANCE_ID_BASE,
    ROBOT_CATEGORY_ID,
)
from synthgen.distractors import DistractorInstance
from synthgen.gating import (
    RobotGateVerdict,
    RobotPixelStats,
    evaluate_robot_gate,
)
from synthgen.logsetup import get_logger
from synthgen.reporting import DistractorSkipReason, RunStats
from synthgen.robots import RobotInstance

logger = get_logger(__name__)


def project_keypoint_to_2d(
    kp_local: np.ndarray, world_mat: np.ndarray
) -> tuple[float, float, float] | None:
    """Project a 3D model-space keypoint to normalized 2D image coordinates.

    Uses Blender's built-in projection which handles all camera conventions.

    Args:
        kp_local: Keypoint in the object's local (Blender) frame.
        world_mat: The object's 4x4 world matrix.

    Returns:
        ``(x_norm, y_norm, depth)`` or None if behind the camera.
    """
    kp_world = world_mat @ np.append(kp_local, 1.0)
    point_3d = mathutils.Vector(kp_world[:3])

    scene = bpy.context.scene
    camera = scene.camera
    co_2d = world_to_camera_view(scene, camera, point_3d)

    if co_2d.z <= 0:
        return None

    x_norm = co_2d.x
    y_norm = 1.0 - co_2d.y  # Blender is bottom-left origin, YOLO is top-left
    return (x_norm, y_norm, co_2d.z)


def _project_keypoint_pair(
    kp_front: np.ndarray,
    kp_back: np.ndarray,
    world_mat: np.ndarray,
    depth_map: np.ndarray,
    img_w: int,
    img_h: int,
    ignore_occlusion: bool,
) -> list[tuple[float, float, int]]:
    """Project a front/back keypoint pair to image space with visibility flags."""
    keypoints_2d: list[tuple[float, float, int]] = []
    for kp_local in (kp_front, kp_back):
        proj = project_keypoint_to_2d(kp_local, world_mat)
        if proj is None:
            keypoints_2d.append((0.0, 0.0, 0))
            continue
        x_n, y_n, depth = proj
        vis = check_keypoint_visibility(
            x_n, y_n, depth, depth_map, img_w, img_h, ignore_occlusion=ignore_occlusion
        )
        keypoints_2d.append((x_n, y_n, vis))
    return keypoints_2d


def measure_robot_pixels(
    robot: RobotInstance,
    cat_seg: np.ndarray,
    inst_seg: np.ndarray | None,
    clean_inst_seg: np.ndarray | None,
    img_w: int,
    img_h: int,
) -> tuple[RobotPixelStats, tuple[float, float, float, float] | None]:
    """Measure one robot's pixel statistics for the gate.

    Args:
        robot: The robot to measure.
        cat_seg: Category-id segmap (fallback when no instance segmap exists).
        inst_seg: Instance-id segmap, or None.
        clean_inst_seg: Distractor-free instance segmap, or None.
        img_w: Image width in pixels.
        img_h: Image height in pixels.

    Returns:
        ``(stats, bbox)`` where bbox is the normalized YOLO box or None.
    """
    if inst_seg is not None:
        seg_for_bbox = inst_seg
        seg_id = robot.instance_id
        tracked = True
    else:
        seg_for_bbox = cat_seg
        seg_id = ROBOT_CATEGORY_ID
        tracked = False

    visible_px = int(np.sum(seg_for_bbox.squeeze() == seg_id))
    bbox = bbox_from_category_segmap(seg_for_bbox, seg_id, img_w, img_h)
    bbox_w_px = int(bbox[2] * img_w) if bbox is not None else 0
    bbox_h_px = int(bbox[3] * img_h) if bbox is not None else 0

    unobstructed_px: int | None = None
    if clean_inst_seg is not None and inst_seg is not None:
        unobstructed_px = int(np.sum(clean_inst_seg.squeeze() == seg_id))

    stats = RobotPixelStats(
        instance_id=robot.instance_id,
        name=robot.name,
        tracked=tracked,
        visible_px=visible_px,
        bbox_w_px=bbox_w_px,
        bbox_h_px=bbox_h_px,
        unobstructed_px=unobstructed_px,
    )
    return stats, bbox


def build_robot_keypoint_annotations(
    scene_robots: list[RobotInstance],
    cat_seg: np.ndarray,
    inst_seg: np.ndarray | None,
    clean_inst_seg: np.ndarray | None,
    depth_map: np.ndarray,
    img_w: int,
    img_h: int,
    min_vis: float,
    ignore_obstructions: bool,
) -> tuple[list[YoloAnnotation], list[RobotGateVerdict]]:
    """Build keypoint annotations for every robot that passes its gate.

    Returns:
        ``(annotations, verdicts)`` — one verdict per scene robot; the frame
        decision from the verdicts is made by ``gating.decide_keypoint_frame``.
    """
    annotations: list[YoloAnnotation] = []
    verdicts: list[RobotGateVerdict] = []
    for robot in scene_robots:
        stats, bbox = measure_robot_pixels(robot, cat_seg, inst_seg, clean_inst_seg, img_w, img_h)
        verdict = evaluate_robot_gate(stats, min_vis, ignore_obstructions)
        verdicts.append(verdict)
        if not verdict.annotate or bbox is None:
            continue
        keypoints_2d = _project_keypoint_pair(
            robot.kp_front,
            robot.kp_back,
            np.array(robot.parent.matrix_world),
            depth_map,
            img_w,
            img_h,
            ignore_occlusion=ignore_obstructions,
        )
        annotations.append((robot.class_id, bbox, keypoints_2d))
    return annotations, verdicts


def build_distractor_keypoint_annotations(
    active_distractors: list[DistractorInstance],
    inst_seg: np.ndarray | None,
    depth_map: np.ndarray,
    img_w: int,
    img_h: int,
    nhrl_class_id: int,
    ignore_occlusion: bool,
    stats: RunStats,
) -> list[YoloAnnotation]:
    """Keypoint+bbox annotations for CAD distractors carrying keypoint sidecars.

    Distractors never trigger a frame discard: the clean segmentation pass
    hides them, so there is no unobstructed reference to gate visibility on.
    Only the (occluded) instance bbox size and per-keypoint depth checks apply.
    """
    annotations: list[YoloAnnotation] = []
    if inst_seg is None:
        return annotations
    for distractor in active_distractors:
        if distractor.instance_id < NHRL_DISTRACTOR_INSTANCE_ID_BASE:
            continue
        if distractor.kp_front is None or distractor.kp_back is None:
            continue

        bbox = bbox_from_category_segmap(inst_seg, distractor.instance_id, img_w, img_h)
        if bbox is None:
            stats.record_distractor_skip(DistractorSkipReason.NOT_IN_SEG)
            continue
        _cx, _cy, bw, bh = bbox
        if int(bw * img_w) < MIN_KEYPOINT_BBOX_DIM_PX or int(bh * img_h) < MIN_KEYPOINT_BBOX_DIM_PX:
            stats.record_distractor_skip(DistractorSkipReason.BBOX_TOO_SMALL)
            continue

        keypoints_2d = _project_keypoint_pair(
            distractor.kp_front,
            distractor.kp_back,
            np.array(distractor.parent.matrix_world),
            depth_map,
            img_w,
            img_h,
            ignore_occlusion=ignore_occlusion,
        )
        annotations.append((nhrl_class_id, bbox, keypoints_2d))
    return annotations
