"""training/yolo/crop_yolo_dataset.py: crops validate everything before writing."""

from __future__ import annotations

from pathlib import Path

import pytest
from PIL import Image


@pytest.fixture(scope="module")
def cropper(load_training):
    return load_training("training/yolo/crop_yolo_dataset.py")


def _recording(root: Path) -> Path:
    (root / "images").mkdir(parents=True)
    (root / "labels").mkdir()
    return root


def test_validation_precedes_writes(cropper, tmp_path: Path) -> None:
    recording = _recording(tmp_path / "rec")
    Image.new("RGB", (200, 100)).save(recording / "images" / "ok.png")
    Image.new("RGB", (50, 50)).save(recording / "images" / "small.png")
    with pytest.raises(SystemExit):
        cropper.plan_crop([recording], (10, 10, 100, 60), 2.0)
    assert Image.open(recording / "images" / "ok.png").size == (200, 100)


def test_unsupported_row_fails_planning(cropper, tmp_path: Path) -> None:
    recording = _recording(tmp_path / "rec")
    Image.new("RGB", (200, 100)).save(recording / "images" / "a.png")
    (recording / "labels" / "a.txt").write_text("0 0.1 0.1 0.2 0.2 0.3 0.3\n")
    with pytest.raises(SystemExit):
        cropper.plan_crop([recording], (0, 0, 100, 50), 2.0)
