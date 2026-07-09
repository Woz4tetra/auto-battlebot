"""Drop-reason taxonomy and run statistics.

Every path that discards a frame or skips an annotation maps to exactly one enum
member, so the end-of-run summary can account for every image that was rendered
but not written. Logging and in-memory counters only; ``RunStats.record`` is the
seam where a file sink could be added later.
"""

import enum
from collections import Counter
from dataclasses import dataclass, field
from typing import Any


class DropReason(str, enum.Enum):
    """Why a whole rendered frame was discarded."""

    # Segmentation mode
    SEG_FLOOR_ONLY = "seg_floor_only"
    SEG_NO_ANNOTATIONS = "seg_no_annotations"
    SEG_VISIBLE_CLASS_UNANNOTATED = "seg_visible_class_unannotated"
    SEG_NO_ROBOT_VISIBLE = "seg_no_robot_visible"
    # Keypoint mode
    KP_PROMINENT_ROBOT_UNLABELED = "kp_prominent_robot_unlabeled"
    KP_NO_ANNOTATIONS = "kp_no_annotations"
    # Whole-scene failure (counted once per planned frame so totals reconcile)
    SCENE_NO_SEGMAPS = "scene_no_segmaps"


class RobotSkipReason(str, enum.Enum):
    """Why one robot was left out of a frame's annotations."""

    NOT_RENDERED = "not_rendered"
    ZERO_PIXELS = "zero_pixels"
    BBOX_TOO_SMALL = "bbox_too_small"
    NO_CLEAN_PIXELS = "no_clean_pixels"
    LOW_VISIBILITY = "low_visibility"
    LOW_PIXEL_FRACTION_FALLBACK = "low_pixel_fraction_fallback"


class DistractorSkipReason(str, enum.Enum):
    """Why a keypoint-carrying distractor was left out (never frame-fatal)."""

    NOT_IN_SEG = "distractor_not_in_seg"
    BBOX_TOO_SMALL = "distractor_bbox_too_small"


class RunAnomaly(str, enum.Enum):
    """Noteworthy events that are not frame drops."""

    CAMERA_TARGET_FALLBACK = "camera_target_fallback"
    RUN_SHORTFALL = "run_shortfall"
    SCENE_RENDER_FAILED = "scene_render_failed"


@dataclass
class FrameVerdict:
    """Outcome of processing one rendered frame."""

    scene_idx: int
    frame_in_scene: int
    written: bool
    global_idx: int | None = None
    drop_reason: DropReason | None = None
    robot_skips: dict[int, RobotSkipReason] = field(default_factory=dict)
    detail: str = ""


class RunStats:
    """In-memory counters for frames, drops, and anomalies across a run."""

    def __init__(self) -> None:
        self.frames_written = 0
        self.frames_dropped = 0
        self.drop_counts: Counter[DropReason] = Counter()
        self.robot_skip_counts: Counter[RobotSkipReason] = Counter()
        self.distractor_skip_counts: Counter[DistractorSkipReason] = Counter()
        self.anomaly_counts: Counter[RunAnomaly] = Counter()

    def record(self, verdict: FrameVerdict) -> None:
        """Fold one frame verdict into the counters."""
        if verdict.written:
            self.frames_written += 1
        else:
            self.frames_dropped += 1
            if verdict.drop_reason is not None:
                self.drop_counts[verdict.drop_reason] += 1
        for reason in verdict.robot_skips.values():
            self.robot_skip_counts[reason] += 1

    def record_distractor_skip(self, reason: DistractorSkipReason) -> None:
        """Count a skipped distractor annotation."""
        self.distractor_skip_counts[reason] += 1

    def record_anomaly(self, reason: RunAnomaly) -> None:
        """Count a run-level anomaly (camera fallback, shortfall, ...)."""
        self.anomaly_counts[reason] += 1

    @staticmethod
    def _format_counter(counter: "Counter[Any]") -> str:
        parts = []
        for key, count in counter.most_common():
            name = key.name if isinstance(key, enum.Enum) else str(key)
            parts.append(f"{name}: {count}")
        return "   ".join(parts)

    def summary_lines(self, requested: int, written: int, scenes: int) -> list[str]:
        """Build the human-readable end-of-run summary.

        Args:
            requested: Number of images the run was asked to produce.
            written: Number of images actually written.
            scenes: Number of scene attempts consumed.

        Returns:
            Lines ready to be logged one by one.
        """
        short = max(0, requested - written)
        headline = f"Run summary: {written}/{requested} images written across {scenes} scenes"
        if self.frames_dropped or short:
            headline += f" ({self.frames_dropped} frames dropped, {short} short)"
        lines = [headline]
        if self.drop_counts:
            lines.append(f"  Drops by reason:  {self._format_counter(self.drop_counts)}")
        if self.robot_skip_counts:
            lines.append(
                f"  Robot skips (frame kept): {self._format_counter(self.robot_skip_counts)}"
            )
        if self.distractor_skip_counts:
            lines.append(
                f"  Distractor annotation skips: "
                f"{self._format_counter(self.distractor_skip_counts)}"
            )
        if self.anomaly_counts:
            lines.append(f"  Anomalies: {self._format_counter(self.anomaly_counts)}")
        return lines
