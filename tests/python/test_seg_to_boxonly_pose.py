"""training/yolo/seg_to_boxonly_pose.py: dotted stems keep their full name."""

from __future__ import annotations

from pathlib import Path

import pytest


@pytest.fixture(scope="module")
def converter(load_training):
    return load_training("training/yolo/seg_to_boxonly_pose.py")


def test_dotted_stem_finds_image(converter, tmp_path: Path) -> None:
    (tmp_path / "labels").mkdir()
    (tmp_path / "images").mkdir()
    label = tmp_path / "labels" / "1729981569.1952372.txt"
    label.write_text("0 0 0 1 1\n")
    image = tmp_path / "images" / "1729981569.1952372.jpg"
    image.write_bytes(b"i")
    assert converter.find_paired_image(label) == image
