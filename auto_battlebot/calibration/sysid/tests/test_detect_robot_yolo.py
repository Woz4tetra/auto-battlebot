"""detect_robot_yolo loads the same engines and class order as the desktop keypoint model."""

from __future__ import annotations

from auto_battlebot.calibration.sysid import detect_robot_yolo
from auto_battlebot.compat import tomllib
from auto_battlebot.paths import REPO_ROOT


def _keypoint_model() -> dict:
    desktop = tomllib.loads((REPO_ROOT / "config" / "_desktop.toml").read_text())
    return desktop["keypoint_model"]


def test_engine_candidates_match_desktop_config() -> None:
    assert detect_robot_yolo.ENGINE_CANDIDATES == _keypoint_model()["engine"]["candidates"]


def test_labels_match_desktop_config() -> None:
    assert detect_robot_yolo.LABELS == _keypoint_model()["label_indices"]
