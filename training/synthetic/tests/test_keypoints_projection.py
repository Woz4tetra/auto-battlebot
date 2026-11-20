"""Keypoint projection into YOLO's top-left image origin."""

from types import SimpleNamespace

import numpy as np
import pytest
from synthgen.reporting import RunStats

BLENDER_TOP_Y = 0.9
BLENDER_DEPTH_M = 2.5


def test_projected_point_y_is_flipped_to_top_left_origin(
    blender_stubs: SimpleNamespace,
) -> None:
    from synthgen import keypoints

    blender_stubs.bpy_extras_object_utils.world_to_camera_view.return_value = SimpleNamespace(
        x=0.25, y=BLENDER_TOP_Y, z=BLENDER_DEPTH_M
    )
    projected = keypoints.project_keypoint_to_2d(np.zeros(3), np.eye(4))
    assert projected is not None
    x_norm, y_norm, depth = projected
    assert x_norm == pytest.approx(0.25)
    assert y_norm == pytest.approx(1.0 - BLENDER_TOP_Y)
    assert depth == pytest.approx(BLENDER_DEPTH_M)


def test_point_behind_camera_has_no_projection(blender_stubs: SimpleNamespace) -> None:
    from synthgen import keypoints

    blender_stubs.bpy_extras_object_utils.world_to_camera_view.return_value = SimpleNamespace(
        x=0.5, y=0.5, z=-1.0
    )
    assert keypoints.project_keypoint_to_2d(np.zeros(3), np.eye(4)) is None


IMG_W = 100
IMG_H = 100
DISTRACTOR_ID = 1000
NHRL_CLASS_ID = 4


def make_distractor(parent_matrix: np.ndarray) -> SimpleNamespace:
    return SimpleNamespace(
        instance_id=DISTRACTOR_ID,
        kp_front=np.zeros(3),
        kp_back=np.ones(3),
        parent=SimpleNamespace(matrix_world=parent_matrix),
    )


def annotate_distractor(segmap: np.ndarray, stats: RunStats) -> list:
    from synthgen import keypoints

    return keypoints.build_distractor_keypoint_annotations(
        [make_distractor(np.eye(4))],
        segmap,
        np.zeros((IMG_H, IMG_W)),
        IMG_W,
        IMG_H,
        NHRL_CLASS_ID,
        True,
        stats,
    )


def test_distractor_missing_from_segmap_is_counted_not_frame_fatal(
    blender_stubs: SimpleNamespace,
) -> None:
    from synthgen.reporting import DistractorSkipReason

    stats = RunStats()
    assert annotate_distractor(np.zeros((IMG_H, IMG_W), dtype=int), stats) == []
    assert stats.distractor_skip_counts[DistractorSkipReason.NOT_IN_SEG] == 1
    assert not stats.robot_skip_counts


def test_distractor_with_tiny_bbox_is_counted_not_frame_fatal(
    blender_stubs: SimpleNamespace,
) -> None:
    from synthgen.reporting import DistractorSkipReason

    segmap = np.zeros((IMG_H, IMG_W), dtype=int)
    segmap[10:12, 10:12] = DISTRACTOR_ID
    stats = RunStats()
    assert annotate_distractor(segmap, stats) == []
    assert stats.distractor_skip_counts[DistractorSkipReason.BBOX_TOO_SMALL] == 1


def test_distractor_with_occluded_or_off_frame_keypoints_still_yields_its_annotation(
    blender_stubs: SimpleNamespace,
) -> None:
    from synthgen import keypoints
    from synthgen.annotations import KeypointVisibility

    blender_stubs.bpy_extras_object_utils.world_to_camera_view.side_effect = [
        SimpleNamespace(x=0.5, y=0.5, z=2.5),
        SimpleNamespace(x=1.5, y=0.5, z=2.5),
    ]
    segmap = np.zeros((IMG_H, IMG_W), dtype=int)
    segmap[20:60, 20:60] = DISTRACTOR_ID

    annotations = keypoints.build_distractor_keypoint_annotations(
        [make_distractor(np.eye(4))],
        segmap,
        np.full((IMG_H, IMG_W), 9.0),
        IMG_W,
        IMG_H,
        NHRL_CLASS_ID,
        False,
        RunStats(),
    )

    assert len(annotations) == 1, "one bad keypoint must not discard a frame of good robots"
    flags = [kp[2] for kp in annotations[0][2]]
    assert flags == [KeypointVisibility.OCCLUDED, KeypointVisibility.OUT_OF_FRAME]
