"""simulation viewer builds frames without opening a window."""

from __future__ import annotations

import sys
from pathlib import Path

import cv2
import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "simulation"))

from config.kinematic import KinematicSimConfig  # noqa: E402
from viewer import Viewer  # noqa: E402


def test_construct_and_compose_open_no_window(monkeypatch: pytest.MonkeyPatch) -> None:
    def refuse(*_args: object, **_kwargs: object) -> None:
        raise AssertionError("window opened before first render")

    monkeypatch.setattr(cv2, "namedWindow", refuse)
    monkeypatch.setattr(cv2, "imshow", refuse)
    viewer = Viewer(KinematicSimConfig(), [])
    assert viewer._window_open is False
