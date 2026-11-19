"""Camera sampling: a returned pose frames the target, or says it fell back to a centred aim."""

from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest
from synthgen.constants import CAMERA_GEOMETRY_RESAMPLES, CAMERA_TARGET_RETRIES

ROBOT_CENTER = [0.5, -0.25, 0.1]
SAMPLE_ARGS = ([0.0, 0.0, 0.1], 2.0, 3.0, (1.0, 2.0), 0.4)


def install_pose_recorder(camera: Any, monkeypatch: pytest.MonkeyPatch) -> list[np.ndarray]:
    aimed_at: list[np.ndarray] = []

    def pose_toward(cam_pos: np.ndarray, target: np.ndarray) -> np.ndarray:
        aimed_at.append(np.asarray(target, dtype=float))
        pose = np.eye(4)
        pose[:3, 3] = cam_pos
        return pose

    monkeypatch.setattr(camera, "_pose_toward", pose_toward)
    return aimed_at


def test_sample_pose_always_frames_target(
    blender_stubs: SimpleNamespace, monkeypatch: pytest.MonkeyPatch
) -> None:
    from synthgen import camera

    aimed_at = install_pose_recorder(camera, monkeypatch)
    checks = []

    def never_in_frame(cam2world: np.ndarray, point: list[float], lens: Any = None) -> bool:
        checks.append(point)
        return False

    monkeypatch.setattr(camera, "move_camera_and_check_point_in_written_view", never_in_frame)
    sample = camera.sample_camera_pose(*SAMPLE_ARGS, robot_center=ROBOT_CENTER)
    assert sample.used_zero_noise_fallback
    assert len(checks) == (CAMERA_GEOMETRY_RESAMPLES + 1) * CAMERA_TARGET_RETRIES
    np.testing.assert_allclose(aimed_at[-1], ROBOT_CENTER)


def test_sample_pose_keeps_first_pose_that_frames_target(
    blender_stubs: SimpleNamespace, monkeypatch: pytest.MonkeyPatch
) -> None:
    from synthgen import camera

    install_pose_recorder(camera, monkeypatch)
    monkeypatch.setattr(camera, "move_camera_and_check_point_in_written_view", lambda *_: True)
    sample = camera.sample_camera_pose(*SAMPLE_ARGS, robot_center=ROBOT_CENTER)
    assert not sample.used_zero_noise_fallback
