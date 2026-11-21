"""export_svo_depth decodes sequentially instead of seeking."""

from __future__ import annotations

import sys
from types import ModuleType, SimpleNamespace

import pytest

from auto_battlebot.script_loader import load_script


@pytest.fixture(scope="module")
def export_svo_depth():
    added = False
    if "pyzed" not in sys.modules:
        try:
            import pyzed.sl  # noqa: F401
        except ImportError:
            pyzed = ModuleType("pyzed")
            sl = ModuleType("pyzed.sl")
            sl.ERROR_CODE = SimpleNamespace(SUCCESS=0)
            pyzed.sl = sl
            sys.modules["pyzed"] = pyzed
            sys.modules["pyzed.sl"] = sl
            added = True
    yield load_script("training/synthetic/scenes/basement_scene/export_svo_depth.py")
    if added:
        del sys.modules["pyzed"]
        del sys.modules["pyzed.sl"]


class FakeCamera:
    def __init__(self, frames: int, ok, fail) -> None:
        self.frames = frames
        self.ok = ok
        self.fail = fail
        self.grabs = 0

    def grab(self, runtime) -> int:
        if self.grabs >= self.frames:
            return self.fail
        self.grabs += 1
        return self.ok

    def set_svo_position(self, index: int) -> None:
        raise AssertionError("seeking smears decode")


def _camera(module, frames: int) -> FakeCamera:
    codes = module.sl.ERROR_CODE
    return FakeCamera(frames, codes.SUCCESS, codes.FAILURE)


def test_every_frame_up_to_target_is_grabbed_in_order(export_svo_depth) -> None:
    camera = _camera(export_svo_depth, 10)
    seen: list[int] = []
    export_svo_depth.decode_sequentially(camera, None, 6, seen.append)
    assert seen == list(range(7))
    assert camera.grabs == 7


def test_running_out_of_frames_exits(export_svo_depth) -> None:
    with pytest.raises(SystemExit, match="ran out of frames at 3"):
        export_svo_depth.decode_sequentially(_camera(export_svo_depth, 3), None, 6, lambda i: None)
