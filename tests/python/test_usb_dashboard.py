from __future__ import annotations

import importlib.util
import io
import queue
import sys
import threading
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "firmware/mr_stabs_mk2/scripts/usb_dashboard.py"


@pytest.fixture(scope="module")
def dash() -> ModuleType:
    spec = importlib.util.spec_from_file_location("usb_dashboard_under_test", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class DamagedBridge:
    def __init__(self, dash: ModuleType) -> None:
        self.dash = dash
        self.opens = 0

    def open(self, request: bytes) -> tuple[int, queue.Queue[Any]]:
        self.opens += 1
        stream: queue.Queue[Any] = queue.Queue()
        stream.put(self.dash.DamagedFrameError())
        return self.opens, stream

    def close(self, stream_id: int) -> None:
        pass

    def note(self, text: str) -> None:
        pass


def handler(command: str) -> SimpleNamespace:
    errors: list[tuple[int, str]] = []
    return SimpleNamespace(
        command=command,
        path="/tuning",
        headers={},
        rfile=io.BytesIO(),
        wfile=io.BytesIO(),
        send_error=lambda code, message: errors.append((code, message)),
        errors=errors,
    )


def test_post_not_retried_on_damage(dash: ModuleType) -> None:
    bridge = DamagedBridge(dash)
    post = handler("POST")
    dash.relay(bridge, post)
    assert bridge.opens == 1
    assert [code for code, _ in post.errors] == [502]


def test_get_retried_on_damage(dash: ModuleType) -> None:
    bridge = DamagedBridge(dash)
    get = handler("GET")
    dash.relay(bridge, get)
    assert bridge.opens == dash.GET_TRIES
    assert [code for code, _ in get.errors] == [502]


def test_unreadable_frame_fails_every_stream(dash: ModuleType) -> None:
    bridge = object.__new__(dash.Bridge)
    bridge._log = None
    bridge._streams_lock = threading.Lock()
    bridge._streams = {1: queue.Queue(), 2: queue.Queue()}
    bridge._handle_line(dash.FRAME_START + b"Q")
    for stream in bridge._streams.values():
        assert isinstance(stream.get_nowait(), dash.DamagedFrameError)


def test_end_frame_ends_only_its_stream(dash: ModuleType) -> None:
    bridge = object.__new__(dash.Bridge)
    bridge._log = None
    bridge._streams_lock = threading.Lock()
    bridge._streams = {1: queue.Queue(), 2: queue.Queue()}
    bridge._handle_line(dash.FRAME_START + b"E 1")
    assert isinstance(bridge._streams[1].get_nowait(), dash.StreamEnd)
    assert bridge._streams[2].empty()
