"""training/yolo/test_tensorrt_image.py: annotated output names never collide."""

from __future__ import annotations

from pathlib import Path

import pytest


@pytest.fixture(scope="module")
def cli(load_training):
    return load_training("training/yolo/test_tensorrt_image.py")


def test_output_names_unique(cli) -> None:
    paths = [Path("train/a.jpg"), Path("val/a.jpg"), Path("val/a.png"), Path("other/val/a.jpg")]
    names = cli.output_names(paths)
    assert len(set(names.values())) == len(paths)


def test_num_classes_from_names() -> None:
    from auto_battlebot.perception.trt_yolo import resolve_num_classes

    assert resolve_num_classes(3, ["a", "b"]) == 3
    assert resolve_num_classes(0, ["a", "b"]) == 2
    assert resolve_num_classes(0, None) == 0
