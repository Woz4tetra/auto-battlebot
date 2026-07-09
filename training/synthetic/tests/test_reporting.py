"""Tests for run statistics and the end-of-run summary."""

from synthgen.reporting import (
    DistractorSkipReason,
    DropReason,
    FrameVerdict,
    RobotSkipReason,
    RunAnomaly,
    RunStats,
)


def _written(scene: int, frame: int) -> FrameVerdict:
    return FrameVerdict(scene_idx=scene, frame_in_scene=frame, written=True, global_idx=0)


def _dropped(reason: DropReason) -> FrameVerdict:
    return FrameVerdict(scene_idx=0, frame_in_scene=0, written=False, drop_reason=reason)


class TestRunStats:
    def test_counts_written_and_dropped(self) -> None:
        stats = RunStats()
        stats.record(_written(0, 0))
        stats.record(_dropped(DropReason.KP_NO_ANNOTATIONS))
        stats.record(_dropped(DropReason.KP_NO_ANNOTATIONS))
        stats.record(_dropped(DropReason.SEG_FLOOR_ONLY))

        assert stats.frames_written == 1
        assert stats.frames_dropped == 3
        assert stats.drop_counts[DropReason.KP_NO_ANNOTATIONS] == 2
        assert stats.drop_counts[DropReason.SEG_FLOOR_ONLY] == 1

    def test_counts_robot_skips_on_kept_frames(self) -> None:
        stats = RunStats()
        verdict = FrameVerdict(
            scene_idx=1,
            frame_in_scene=2,
            written=True,
            global_idx=5,
            robot_skips={2: RobotSkipReason.BBOX_TOO_SMALL, 3: RobotSkipReason.NOT_RENDERED},
        )
        stats.record(verdict)
        assert stats.robot_skip_counts[RobotSkipReason.BBOX_TOO_SMALL] == 1
        assert stats.robot_skip_counts[RobotSkipReason.NOT_RENDERED] == 1

    def test_anomalies_and_distractor_skips(self) -> None:
        stats = RunStats()
        stats.record_anomaly(RunAnomaly.CAMERA_TARGET_FALLBACK)
        stats.record_anomaly(RunAnomaly.CAMERA_TARGET_FALLBACK)
        stats.record_distractor_skip(DistractorSkipReason.BBOX_TOO_SMALL)
        assert stats.anomaly_counts[RunAnomaly.CAMERA_TARGET_FALLBACK] == 2
        assert stats.distractor_skip_counts[DistractorSkipReason.BBOX_TOO_SMALL] == 1


class TestSummaryLines:
    def test_clean_run_headline_only(self) -> None:
        stats = RunStats()
        for i in range(3):
            stats.record(_written(0, i))
        lines = stats.summary_lines(requested=3, written=3, scenes=1)
        assert len(lines) == 1
        assert "3/3 images written across 1 scenes" in lines[0]
        assert "dropped" not in lines[0]

    def test_shortfall_summary(self) -> None:
        stats = RunStats()
        stats.record(_written(0, 0))
        stats.record(_dropped(DropReason.KP_PROMINENT_ROBOT_UNLABELED))
        stats.record_anomaly(RunAnomaly.RUN_SHORTFALL)
        lines = stats.summary_lines(requested=5, written=1, scenes=2)

        assert "1/5 images written" in lines[0]
        assert "(1 frames dropped, 4 short)" in lines[0]
        joined = "\n".join(lines)
        assert "KP_PROMINENT_ROBOT_UNLABELED: 1" in joined
        assert "RUN_SHORTFALL: 1" in joined
