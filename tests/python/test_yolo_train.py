"""training/yolo/train.py: cache cleanup never blocks training."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest


@pytest.fixture(scope="module")
def train(load_training):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "training" / "yolo"))
    return load_training("training/yolo/train.py")


def test_cache_cleanup_failure_does_not_abort(
    train, monkeypatch: pytest.MonkeyPatch, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    def fail(*_args: object, **_kwargs: object) -> None:
        raise OSError("disk gone")

    monkeypatch.setattr(train, "sweep", fail)
    dataset = tmp_path / "data.yaml"
    dataset.write_text("path: .\n")
    train.reclaim_stale_caches(str(dataset), 7.0)
    assert "cache cleanup skipped" in capsys.readouterr().out
