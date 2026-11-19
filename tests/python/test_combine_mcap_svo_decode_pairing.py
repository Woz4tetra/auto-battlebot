"""combine_mcap_svo pairs the Nth decoded JPEG with the Nth SVO message and closes the decoder."""

from __future__ import annotations

import importlib.util
from collections.abc import Iterator
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "combine_mcap_svo.py"
SPEC = importlib.util.spec_from_file_location("combine_mcap_svo", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
combine = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(combine)


def test_jpegs_come_out_in_message_order() -> None:
    jpegs = iter([b"first", b"second"])
    path = Path("a.svo2")
    assert combine._next_left_jpeg(jpegs, 0, 2, path) == b"first"
    assert combine._next_left_jpeg(jpegs, 1, 2, path) == b"second"


def test_running_out_of_jpegs_names_the_frame_index() -> None:
    with pytest.raises(RuntimeError, match="ran out at frame 1"):
        combine._next_left_jpeg(iter([]), 1, 2, Path("a.svo2"))


def test_decoder_is_closed_when_the_merge_aborts(monkeypatch: pytest.MonkeyPatch) -> None:
    closed: list[bool] = []

    def fake_jpegs(*_args: object, **_kwargs: object) -> Iterator[bytes]:
        try:
            yield b"frame"
        finally:
            closed.append(True)

    monkeypatch.setattr(combine.svo2, "sample_fps", lambda *_a: 30.0)
    monkeypatch.setattr(combine.svo2, "iter_left_jpegs", fake_jpegs)
    with pytest.raises(ValueError, match="abort"):
        with combine._left_jpegs_closed_on_exit(Path("a.svo2"), "topic", "ffmpeg") as jpegs:
            next(jpegs)
            raise ValueError("abort")
    assert closed == [True]
