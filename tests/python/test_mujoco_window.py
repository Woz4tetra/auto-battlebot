"""MujocoWindow joins the viewer render thread on close and closes before interpreter exit."""

from __future__ import annotations

import atexit
import contextlib
import threading
import time
from types import SimpleNamespace
from typing import Any

import mujoco
import numpy as np
import pytest

import auto_battlebot.kinematic_sim.mujoco_window as mujoco_window
from auto_battlebot.kinematic_sim.mujoco_window import MujocoWindow

XML = "<mujoco><worldbody><geom name='g' type='sphere' size='0.1'/></worldbody></mujoco>"


class FakeHandle:
    def __init__(self, stop: threading.Event) -> None:
        self._stop = stop
        self.opt = SimpleNamespace(geomgroup=np.zeros(6, dtype=np.uint8))
        self.cam = SimpleNamespace(lookat=np.zeros(3), distance=0.0, elevation=0.0, azimuth=0.0)

    @contextlib.contextmanager
    def lock(self) -> Any:
        yield

    def close(self) -> None:
        self._stop.set()

    def is_running(self) -> bool:
        return not self._stop.is_set()

    def sync(self) -> None:
        return None


def _fake_launch(threads: list[threading.Thread]) -> Any:
    def launch_passive(model: Any, data: Any, **_: Any) -> FakeHandle:
        stop = threading.Event()

        def render_loop() -> None:
            stop.wait()
            time.sleep(0.3)

        thread = threading.Thread(target=render_loop, daemon=True)
        thread.start()
        threads.append(thread)
        return FakeHandle(stop)

    return launch_passive


def test_close_joins_the_viewer_render_thread(monkeypatch: pytest.MonkeyPatch) -> None:
    threads: list[threading.Thread] = []
    monkeypatch.setattr(mujoco_window.mujoco.viewer, "launch_passive", _fake_launch(threads))
    registered: list[Any] = []
    monkeypatch.setattr(atexit, "register", registered.append)
    monkeypatch.setattr(atexit, "unregister", lambda f: None)
    model = mujoco.MjModel.from_xml_string(XML)
    window = MujocoWindow(2.0, 2.0)
    window._launch(model, mujoco.MjData(model))
    assert window.close in registered
    assert threads[0].is_alive()
    window.close()
    assert not threads[0].is_alive()
