"""simulation viewer builds frames without opening a window."""

from __future__ import annotations

import cv2
import pytest

from auto_battlebot.kinematic_sim.config.kinematic import KinematicSimConfig
from auto_battlebot.kinematic_sim.viewer import Viewer


def test_construct_and_compose_open_no_window(monkeypatch: pytest.MonkeyPatch) -> None:
    def refuse(*_args: object, **_kwargs: object) -> None:
        raise AssertionError("window opened before first render")

    monkeypatch.setattr(cv2, "namedWindow", refuse)
    monkeypatch.setattr(cv2, "imshow", refuse)
    viewer = Viewer(KinematicSimConfig(), [])
    assert viewer._window_open is False


def test_render_does_not_touch_sim_state(monkeypatch: pytest.MonkeyPatch) -> None:
    from auto_battlebot.kinematic_sim import kinematic_sim_server

    monkeypatch.setattr(cv2, "namedWindow", lambda *_a, **_k: None)
    monkeypatch.setattr(cv2, "setMouseCallback", lambda *_a, **_k: None)
    monkeypatch.setattr(cv2, "imshow", lambda *_a, **_k: None)
    monkeypatch.setattr(cv2, "waitKey", lambda *_a, **_k: -1)
    cfg = KinematicSimConfig()
    server = kinematic_sim_server.KinematicServer(cfg)
    server._reset()
    server._sim_time = 3.5
    server._tick = 175
    before_pose = tuple(server._plant.pose())
    before_opponents = [(o.x, o.y, o.yaw) for o in server._opponents]

    Viewer(cfg, []).render(
        server._plant, server._opponents, server._tick, server._sim_time, (0.0, 0.0)
    )

    assert server._sim_time == 3.5
    assert server._tick == 175
    assert tuple(server._plant.pose()) == before_pose
    assert [(o.x, o.y, o.yaw) for o in server._opponents] == before_opponents
