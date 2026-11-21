"""training/yolo split scripts: remainders go to train, scene keys cover every filename scheme."""

from __future__ import annotations

import pytest


@pytest.fixture(scope="module")
def splitter(load_training):
    return load_training("training/yolo/split_yolo_dataset.py")


@pytest.fixture(scope="module")
def by_scene(load_training):
    return load_training("training/yolo/split_by_scene.py")


def test_remainder_goes_to_train(splitter) -> None:
    counts = splitter.split_counts(31_465, 0.9, 0.1, 0.0)
    assert sum(counts.values()) == 31_465
    assert counts["test"] == 0


@pytest.mark.parametrize(
    ("stem", "scene"),
    [
        ("our_robot_kp__rec7-000123_jpg.rf.abc123", "our_robot_kp__rec7"),
        ("nhrl_may25_fight3_yolo_seg__frame_12", "nhrl_may25_fight3"),
        ("recap_export__frame_4", "recap_export"),
    ],
)
def test_scene_patterns_cover_all_schemes(by_scene, stem: str, scene: str) -> None:
    assert by_scene.scene_key(stem) == scene


def test_unknown_scheme_has_no_scene(by_scene) -> None:
    assert by_scene.scene_key("random_name_7") is None
