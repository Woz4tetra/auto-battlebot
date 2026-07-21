"""Tests for the frame keep/drop policy."""

from synthgen.constants import (
    MIN_KEYPOINT_BBOX_DIM_PX,
    PROMINENT_UNLABELED_ROBOT_MIN_PX,
    SEG_FLOOR_CLASS_ID,
)
from synthgen.gating import (
    RobotPixelStats,
    decide_keypoint_frame,
    decide_seg_frame,
    evaluate_robot_gate,
)
from synthgen.reporting import DropReason, RobotSkipReason


def make_stats(
    visible_px: int = 5000,
    bbox_w_px: int = 100,
    bbox_h_px: int = 100,
    unobstructed_px: "int | None" = 5000,
    tracked: bool = True,
    instance_id: int = 1,
) -> RobotPixelStats:
    return RobotPixelStats(
        instance_id=instance_id,
        name=f"robot_{instance_id}",
        tracked=tracked,
        visible_px=visible_px,
        bbox_w_px=bbox_w_px,
        bbox_h_px=bbox_h_px,
        unobstructed_px=unobstructed_px,
    )


class TestRobotGate:
    def test_healthy_robot_annotated(self) -> None:
        verdict = evaluate_robot_gate(make_stats(), 0.10, False)
        assert verdict.annotate is True
        assert verdict.skip_reason is None
        assert verdict.frame_fatal is False

    def test_not_rendered_robot_skipped_not_fatal(self) -> None:
        verdict = evaluate_robot_gate(
            make_stats(visible_px=0, bbox_w_px=0, bbox_h_px=0), 0.10, False
        )
        assert verdict.annotate is False
        assert verdict.skip_reason is RobotSkipReason.NOT_RENDERED
        assert verdict.frame_fatal is False

    def test_untracked_zero_pixels(self) -> None:
        verdict = evaluate_robot_gate(
            make_stats(visible_px=0, bbox_w_px=0, bbox_h_px=0, tracked=False, unobstructed_px=None),
            0.10,
            False,
        )
        assert verdict.skip_reason is RobotSkipReason.ZERO_PIXELS
        assert verdict.frame_fatal is False

    def test_tiny_robot_skipped_but_frame_kept(self) -> None:
        # A small distant robot: below the bbox minimum and below prominence.
        verdict = evaluate_robot_gate(
            make_stats(
                visible_px=PROMINENT_UNLABELED_ROBOT_MIN_PX - 1,
                bbox_w_px=MIN_KEYPOINT_BBOX_DIM_PX - 1,
                bbox_h_px=MIN_KEYPOINT_BBOX_DIM_PX,
            ),
            0.10,
            False,
        )
        assert verdict.annotate is False
        assert verdict.skip_reason is RobotSkipReason.BBOX_TOO_SMALL
        assert verdict.frame_fatal is False

    def test_bbox_boundary(self) -> None:
        at_min = make_stats(bbox_w_px=MIN_KEYPOINT_BBOX_DIM_PX, bbox_h_px=MIN_KEYPOINT_BBOX_DIM_PX)
        assert evaluate_robot_gate(at_min, 0.10, False).annotate is True
        below = make_stats(
            bbox_w_px=MIN_KEYPOINT_BBOX_DIM_PX - 1, bbox_h_px=MIN_KEYPOINT_BBOX_DIM_PX
        )
        assert evaluate_robot_gate(below, 0.10, False).skip_reason is RobotSkipReason.BBOX_TOO_SMALL

    def test_prominent_skipped_robot_is_fatal(self) -> None:
        # Short-edge bbox: fails the min-edge check yet is plenty visible.
        verdict = evaluate_robot_gate(
            make_stats(
                visible_px=PROMINENT_UNLABELED_ROBOT_MIN_PX,
                bbox_w_px=200,
                bbox_h_px=MIN_KEYPOINT_BBOX_DIM_PX - 1,
            ),
            0.10,
            False,
        )
        assert verdict.skip_reason is RobotSkipReason.BBOX_TOO_SMALL
        assert verdict.frame_fatal is True

    def test_prominence_boundary(self) -> None:
        below = evaluate_robot_gate(
            make_stats(
                visible_px=PROMINENT_UNLABELED_ROBOT_MIN_PX - 1, bbox_w_px=200, bbox_h_px=30
            ),
            0.10,
            False,
        )
        assert below.frame_fatal is False

    def test_low_visibility_skip(self) -> None:
        # Below the visibility threshold but below prominence: skip, keep the frame.
        verdict = evaluate_robot_gate(
            make_stats(visible_px=PROMINENT_UNLABELED_ROBOT_MIN_PX - 1, unobstructed_px=5000),
            0.10,
            False,
        )
        assert verdict.skip_reason is RobotSkipReason.LOW_VISIBILITY
        assert verdict.frame_fatal is False  # below prominence

    def test_low_visibility_prominent_is_fatal(self) -> None:
        # Heavily occluded but still large on screen: labeling through the
        # occluder is wrong and not labeling is a false negative, so drop.
        verdict = evaluate_robot_gate(
            make_stats(visible_px=2000, unobstructed_px=50000), 0.10, False
        )
        assert verdict.skip_reason is RobotSkipReason.LOW_VISIBILITY
        assert verdict.frame_fatal is True

    def test_visibility_boundary(self) -> None:
        at_threshold = make_stats(visible_px=500, unobstructed_px=5000)
        assert evaluate_robot_gate(at_threshold, 0.10, False).annotate is True

    def test_no_clean_pixels(self) -> None:
        verdict = evaluate_robot_gate(make_stats(unobstructed_px=0), 0.10, False)
        assert verdict.skip_reason is RobotSkipReason.NO_CLEAN_PIXELS

    def test_pixel_fraction_fallback_without_clean_pass(self) -> None:
        # 100x100 bbox but only 500 visible pixels: 5% < 10% minimum.
        verdict = evaluate_robot_gate(make_stats(visible_px=500, unobstructed_px=None), 0.10, False)
        assert verdict.skip_reason is RobotSkipReason.LOW_PIXEL_FRACTION_FALLBACK

    def test_ignore_obstructions_skips_occlusion_checks(self) -> None:
        for stats in (
            make_stats(visible_px=400, unobstructed_px=5000),
            make_stats(unobstructed_px=0),
            make_stats(visible_px=500, unobstructed_px=None),
        ):
            verdict = evaluate_robot_gate(stats, 0.10, True)
            assert verdict.annotate is True, stats

    def test_ignore_obstructions_still_enforces_bbox_size(self) -> None:
        verdict = evaluate_robot_gate(
            make_stats(
                bbox_w_px=MIN_KEYPOINT_BBOX_DIM_PX - 1,
                bbox_h_px=MIN_KEYPOINT_BBOX_DIM_PX - 1,
                visible_px=90,
            ),
            0.10,
            True,
        )
        assert verdict.skip_reason is RobotSkipReason.BBOX_TOO_SMALL


class TestKeypointFrameDecision:
    def test_keep_with_annotations(self) -> None:
        verdicts = [evaluate_robot_gate(make_stats(), 0.10, False)]
        decision = decide_keypoint_frame(verdicts, annotation_count=1)
        assert decision.keep is True

    def test_drop_when_no_annotations(self) -> None:
        decision = decide_keypoint_frame([], annotation_count=0)
        assert decision.keep is False
        assert decision.drop_reason is DropReason.KP_NO_ANNOTATIONS

    def test_drop_on_fatal_verdict(self) -> None:
        fatal = evaluate_robot_gate(
            make_stats(visible_px=5000, bbox_w_px=200, bbox_h_px=MIN_KEYPOINT_BBOX_DIM_PX - 1),
            0.10,
            False,
        )
        assert fatal.frame_fatal
        healthy = evaluate_robot_gate(make_stats(instance_id=2), 0.10, False)
        decision = decide_keypoint_frame([healthy, fatal], annotation_count=1)
        assert decision.keep is False
        assert decision.drop_reason is DropReason.KP_PROMINENT_ROBOT_UNLABELED
        assert "robot_1" in decision.detail

    def test_tiny_skip_keeps_frame(self) -> None:
        tiny = evaluate_robot_gate(
            make_stats(
                visible_px=PROMINENT_UNLABELED_ROBOT_MIN_PX - 1,
                bbox_w_px=MIN_KEYPOINT_BBOX_DIM_PX - 1,
                bbox_h_px=MIN_KEYPOINT_BBOX_DIM_PX - 1,
            ),
            0.10,
            False,
        )
        healthy = evaluate_robot_gate(make_stats(instance_id=2), 0.10, False)
        decision = decide_keypoint_frame([tiny, healthy], annotation_count=1)
        assert decision.keep is True


class TestSegFrameDecision:
    ROBOT_IDS = {4, 5}

    def test_floor_only(self) -> None:
        decision = decide_seg_frame({SEG_FLOOR_CLASS_ID}, set(), self.ROBOT_IDS)
        assert decision.drop_reason is DropReason.SEG_FLOOR_ONLY

    def test_no_annotations(self) -> None:
        decision = decide_seg_frame({SEG_FLOOR_CLASS_ID, 4}, set(), self.ROBOT_IDS)
        assert decision.drop_reason is DropReason.SEG_NO_ANNOTATIONS

    def test_visible_class_unannotated(self) -> None:
        decision = decide_seg_frame({SEG_FLOOR_CLASS_ID, 4}, {4}, self.ROBOT_IDS)
        assert decision.drop_reason is DropReason.SEG_VISIBLE_CLASS_UNANNOTATED
        assert str(SEG_FLOOR_CLASS_ID) in decision.detail

    def test_no_robot_visible(self) -> None:
        visible = {SEG_FLOOR_CLASS_ID, 2}
        decision = decide_seg_frame(visible, visible, self.ROBOT_IDS)
        assert decision.drop_reason is DropReason.SEG_NO_ROBOT_VISIBLE

    def test_keep(self) -> None:
        visible = {SEG_FLOOR_CLASS_ID, 4}
        decision = decide_seg_frame(visible, visible, self.ROBOT_IDS)
        assert decision.keep is True
