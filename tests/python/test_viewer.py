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


def test_camera_frame_has_no_outline_or_overlay_pixels(monkeypatch: pytest.MonkeyPatch) -> None:
    import numpy as np

    from auto_battlebot.kinematic_sim import kinematic_sim_server
    from auto_battlebot.kinematic_sim.viewer import COLLISION_BGR

    cfg = KinematicSimConfig()
    server = kinematic_sim_server.KinematicServer(cfg)
    server._reset()
    viewer = Viewer(cfg, [])
    world = viewer.compose_world(server._plant, server._opponents)
    full = viewer.compose(server._plant, server._opponents, 0, 0.0, (1.0, 1.0))
    assert not (world == np.array(COLLISION_BGR, dtype=world.dtype)).all(axis=2).any()
    assert (full != world).any()
    assert (full == np.array(COLLISION_BGR, dtype=full.dtype)).all(axis=2).any()


@pytest.mark.parametrize("render_delay_s", [0.0, 0.05])
def test_sim_time_advances_by_dt_per_tick_whatever_the_render_cost(
    monkeypatch: pytest.MonkeyPatch, render_delay_s: float
) -> None:
    import time
    from types import SimpleNamespace

    from auto_battlebot.kinematic_sim import kinematic_sim_server

    cfg = KinematicSimConfig()
    cfg.sim.max_ticks = 4
    server = kinematic_sim_server.KinematicServer(cfg)
    monkeypatch.setattr(kinematic_sim_server, "recv_all", lambda *_a: b"")
    monkeypatch.setattr(
        kinematic_sim_server,
        "unpack_request",
        lambda _data: SimpleNamespace(linear_x=0.0, angular_z=0.0),
    )
    monkeypatch.setattr(server, "_send_frame", lambda *_a: None)
    server._viewer = SimpleNamespace(render=lambda *_a: time.sleep(render_delay_s))
    server._run_client(object(), None)
    assert server._tick == 4
    assert server._sim_time == pytest.approx(4 * cfg.sim.dt)
