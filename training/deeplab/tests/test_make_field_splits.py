"""training/deeplab/make_field_splits.py: scene-level splits, nested ladder, mask naming."""

from __future__ import annotations

import random
from pathlib import Path

import pytest


@pytest.fixture(scope="module")
def splits(load_training):
    return load_training("training/deeplab/make_field_splits.py")


def test_single_scene_field_stays_in_train(splits) -> None:
    assert splits.choose_eval_scenes({"only": 100}, 0.2, random.Random(0)) == []


def test_two_scene_field_keeps_one_scene_on_each_side(splits) -> None:
    chosen = splits.choose_eval_scenes({"a": 50, "b": 50}, 0.9, random.Random(0))
    assert len(chosen) == 1


def test_ladder_rungs_are_nested(splits) -> None:
    frames = {"s1": [f"a{i}" for i in range(10)], "s2": [f"b{i}" for i in range(7)], "s3": ["c0"]}
    order = splits.scene_spread_order(frames, random.Random(3))
    assert sorted(order) == sorted(n for names in frames.values() for n in names)
    assert len({name[0] for name in order[:3]}) == 3


def test_mask_named_after_image(splits, tmp_path: Path) -> None:
    source = tmp_path / "src"
    source.mkdir()
    (source / "img.jpg").write_bytes(b"i")
    (source / "twin_mask.png").write_bytes(b"m")
    record = {"path": "img.jpg", "mask": "twin_mask.png"}
    target = tmp_path / "out"
    splits.build_tree([record], source, target, "copy")
    assert sorted(p.name for p in target.iterdir()) == ["img.jpg", "img_mask.png"]
