"""Frame keep/drop policy (no Blender imports).

Every decision about whether a robot gets annotated and whether a frame gets
written lives here, so the policy is unit-testable and every negative outcome
carries a reason.

Keypoint-mode policy: a robot that fails its annotation gate is left unlabeled
and the frame is kept, UNLESS the robot is prominently visible — an unlabeled
prominent robot would be a false negative in the training data, so those frames
are dropped instead. This replaces the old behavior where any gate failure
discarded the whole frame (even for a few-pixel sliver at the frame edge),
which was the main source of unexplained frame drops.
"""

from dataclasses import dataclass

from synthgen.constants import (
    MIN_KEYPOINT_BBOX_DIM_PX,
    PROMINENT_UNLABELED_ROBOT_MIN_PX,
    SEG_FLOOR_CLASS_ID,
)
from synthgen.reporting import DropReason, RobotSkipReason


@dataclass(frozen=True)
class RobotPixelStats:
    """Per-robot pixel measurements for one frame, computed from the segmaps.

    Attributes:
        instance_id: The robot's instance id in the instance segmap.
        name: Robot name for log messages.
        tracked: True when per-instance stats are available (instance segmap
            was rendered). Without it, robots share one category id and
            per-robot prominence cannot be established.
        visible_px: Robot pixels in the rendered (occluded) segmap.
        bbox_w_px: Bounding-box width in pixels (0 when the robot has no bbox).
        bbox_h_px: Bounding-box height in pixels.
        unobstructed_px: Robot pixels in the distractor-free clean pass, or
            None when the clean pass was unavailable (or skipped).
    """

    instance_id: int
    name: str
    tracked: bool
    visible_px: int
    bbox_w_px: int = 0
    bbox_h_px: int = 0
    unobstructed_px: int | None = None


@dataclass(frozen=True)
class RobotGateVerdict:
    """Outcome of the annotation gate for one robot in one frame."""

    stats: RobotPixelStats
    annotate: bool
    skip_reason: RobotSkipReason | None = None
    frame_fatal: bool = False
    detail: str = ""


@dataclass(frozen=True)
class FrameDecision:
    """Whether to write a frame, and why not."""

    keep: bool
    drop_reason: DropReason | None = None
    detail: str = ""


def evaluate_robot_gate(
    stats: RobotPixelStats,
    min_visibility: float,
    ignore_obstructions: bool,
) -> RobotGateVerdict:
    """Decide whether one robot gets a keypoint annotation.

    Args:
        stats: The robot's pixel measurements for this frame.
        min_visibility: Minimum visible/unobstructed pixel fraction
            (``output.min_robot_visibility``).
        ignore_obstructions: When True, all occlusion checks are skipped
            (``output.ignore_obstructions``).

    Returns:
        The gate verdict; ``frame_fatal`` is set when the robot is skipped but
        prominently visible, which should drop the whole frame.
    """
    skip_reason: RobotSkipReason | None = None
    detail = ""

    if stats.visible_px <= 0 or (stats.bbox_w_px <= 0 and stats.bbox_h_px <= 0):
        skip_reason = RobotSkipReason.NOT_RENDERED if stats.tracked else RobotSkipReason.ZERO_PIXELS
        detail = "no visible pixels"
    elif stats.bbox_w_px < MIN_KEYPOINT_BBOX_DIM_PX or stats.bbox_h_px < MIN_KEYPOINT_BBOX_DIM_PX:
        skip_reason = RobotSkipReason.BBOX_TOO_SMALL
        detail = f"bbox {stats.bbox_w_px}x{stats.bbox_h_px} < {MIN_KEYPOINT_BBOX_DIM_PX}px min"
    elif not ignore_obstructions:
        if stats.unobstructed_px is not None:
            if stats.unobstructed_px <= 0:
                skip_reason = RobotSkipReason.NO_CLEAN_PIXELS
                detail = "no pixels in the distractor-free pass"
            else:
                fraction = stats.visible_px / stats.unobstructed_px
                if fraction < min_visibility:
                    skip_reason = RobotSkipReason.LOW_VISIBILITY
                    detail = f"visibility {fraction:.3f} < {min_visibility:.3f}"
        else:
            bbox_area_px = max(1, stats.bbox_w_px * stats.bbox_h_px)
            if stats.visible_px < bbox_area_px * min_visibility:
                skip_reason = RobotSkipReason.LOW_PIXEL_FRACTION_FALLBACK
                detail = (
                    f"{stats.visible_px}px < {min_visibility:.3f} of bbox area {bbox_area_px}px"
                )

    if skip_reason is None:
        return RobotGateVerdict(stats=stats, annotate=True)

    frame_fatal = stats.tracked and stats.visible_px >= PROMINENT_UNLABELED_ROBOT_MIN_PX
    return RobotGateVerdict(
        stats=stats,
        annotate=False,
        skip_reason=skip_reason,
        frame_fatal=frame_fatal,
        detail=detail,
    )


def decide_keypoint_frame(verdicts: list[RobotGateVerdict], annotation_count: int) -> FrameDecision:
    """Decide whether a keypoint-mode frame gets written.

    Args:
        verdicts: Gate verdicts for every robot in the scene.
        annotation_count: Total annotations built for the frame (robots plus
            keypoint-carrying distractors).

    Returns:
        The frame decision with a drop reason when applicable.
    """
    fatal = [v for v in verdicts if v.frame_fatal]
    if fatal:
        worst = max(fatal, key=lambda v: v.stats.visible_px)
        reason_name = worst.skip_reason.name if worst.skip_reason is not None else "unknown"
        detail = (
            f"robot {worst.stats.instance_id} ({worst.stats.name}) visible "
            f"{worst.stats.visible_px}px but skipped ({reason_name}: {worst.detail})"
        )
        return FrameDecision(
            keep=False, drop_reason=DropReason.KP_PROMINENT_ROBOT_UNLABELED, detail=detail
        )
    if annotation_count == 0:
        return FrameDecision(
            keep=False, drop_reason=DropReason.KP_NO_ANNOTATIONS, detail="no annotations built"
        )
    return FrameDecision(keep=True)


def decide_seg_frame(
    visible_category_ids: set[int],
    annotated_category_ids: set[int],
    robot_class_ids: set[int],
) -> FrameDecision:
    """Decide whether a segmentation-mode frame gets written.

    Args:
        visible_category_ids: Foreground class ids present in the segmap.
        annotated_category_ids: Class ids that produced polygon annotations.
        robot_class_ids: The class ids that represent target robots.

    Returns:
        The frame decision with a drop reason when applicable.
    """
    if visible_category_ids == {SEG_FLOOR_CLASS_ID}:
        return FrameDecision(
            keep=False, drop_reason=DropReason.SEG_FLOOR_ONLY, detail="only floor visible"
        )
    if not annotated_category_ids:
        return FrameDecision(
            keep=False,
            drop_reason=DropReason.SEG_NO_ANNOTATIONS,
            detail="all contours filtered",
        )
    missing = visible_category_ids - annotated_category_ids
    if missing:
        return FrameDecision(
            keep=False,
            drop_reason=DropReason.SEG_VISIBLE_CLASS_UNANNOTATED,
            detail=f"visible classes without labels: {sorted(missing)}",
        )
    if not (annotated_category_ids & robot_class_ids):
        return FrameDecision(
            keep=False,
            drop_reason=DropReason.SEG_NO_ROBOT_VISIBLE,
            detail="no target robot class annotated",
        )
    return FrameDecision(keep=True)
