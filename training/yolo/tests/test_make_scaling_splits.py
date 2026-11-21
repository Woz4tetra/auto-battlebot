"""training/yolo/make_scaling_splits.py: nested arms and whole-scene val picks."""

from __future__ import annotations

from collections import Counter
from pathlib import Path

import pytest


@pytest.fixture(scope="module")
def scaling(load_training):
    return load_training("training/yolo/make_scaling_splits.py")


def _images() -> dict[str, list[Path]]:
    sizes = {"a": 40, "b": 25, "c": 10, "d": 60, "e": 5, "f": 30}
    return {scene: [Path(f"/{scene}/{i}.jpg") for i in range(n)] for scene, n in sizes.items()}


def test_arms_nested_and_frame_matched(scaling) -> None:
    images = _images()
    arms = scaling.nested_scene_arms(sorted(images), images, seed=0)
    assert set(arms["scene50"]) <= set(arms["scene75"])
    pool = [p for frames in images.values() for p in frames]
    random_arms = scaling.nested_random_arms(pool, seed=0)
    assert set(random_arms["rand50"]) <= set(random_arms["rand75"])
    assert random_arms == scaling.nested_random_arms(list(pool), seed=0)
    assert random_arms != scaling.nested_random_arms(pool, seed=1)
    assert len(random_arms["rand75"]) == int(len(pool) * 0.75)
    assert len(random_arms["rand50"]) == int(len(pool) * 0.5)


def test_val_scenes_are_whole_scenes(scaling) -> None:
    images = _images()
    classes = {scene: Counter({1: len(frames) // 2}) for scene, frames in images.items()}
    picked = scaling.choose_val_scenes(images, classes, frac=0.15, count=2, seed=0, trials=200)
    assert len(picked) == 2
    assert set(picked) <= set(images)
