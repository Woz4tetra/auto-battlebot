"""The two validators write validation_state.json via a temp file and an atomic replace."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest


@pytest.mark.parametrize(
    "script",
    ["training/deeplab/validate_segmask_dataset.py", "training/yolo/validate_yolo_dataset.py"],
)
def test_save_state_is_atomic(load_training, monkeypatch, tmp_path: Path, script: str) -> None:
    module = load_training(script)
    owner = next(
        value
        for value in vars(module).values()
        if isinstance(value, type)
        and "save_state" in vars(value)
        and value.__module__ == module.__name__
    )
    state_file = tmp_path / "validation_state.json"
    state_file.write_text(json.dumps({"a.jpg": "pass"}))
    fake = SimpleNamespace(
        state_file=state_file, validation_state={"a.jpg": "fail", "b.jpg": "pass"}
    )

    def interrupted(obj, handle, **kwargs):
        handle.write('{"a.jpg": "fa')
        raise KeyboardInterrupt

    monkeypatch.setattr(module.json, "dump", interrupted)
    with pytest.raises(KeyboardInterrupt):
        owner.save_state(fake)
    assert json.loads(state_file.read_text()) == {"a.jpg": "pass"}
