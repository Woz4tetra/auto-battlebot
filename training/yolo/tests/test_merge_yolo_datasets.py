"""merge_yolo_datasets.py and merge_validation_state.py: validation state decides what is merged."""

from __future__ import annotations

from pathlib import Path

import pytest


@pytest.fixture(scope="module")
def merger(load_training):
    return load_training("training/yolo/merge_yolo_datasets.py")


@pytest.fixture(scope="module")
def state_merger(load_training):
    return load_training("training/yolo/merge_validation_state.py")


@pytest.fixture(scope="module")
def validator(load_training):
    return load_training("training/yolo/validate_yolo_dataset.py")


def test_dataset_without_state_is_excluded(merger, tmp_path: Path) -> None:
    image = tmp_path / "images" / "a.jpg"
    assert not merger.is_image_passing(image, tmp_path, None, include_unvalidated=False)


def test_unvalidated_frame_follows_flag(merger, tmp_path: Path) -> None:
    image = tmp_path / "images" / "a.jpg"
    state = {"images/b.jpg": "pass"}
    assert not merger.is_image_passing(image, tmp_path, state, include_unvalidated=False)
    assert merger.is_image_passing(image, tmp_path, state, include_unvalidated=True)


def test_failed_frame_is_never_merged(merger, tmp_path: Path) -> None:
    image = tmp_path / "images" / "a.jpg"
    state = {"images/a.jpg": "fail"}
    assert not merger.is_image_passing(image, tmp_path, state, include_unvalidated=True)


def test_state_keys_match_validate_yolo_dataset(merger, validator, tmp_path: Path) -> None:
    image = tmp_path / "images" / "a.jpg"
    key = str(image.relative_to(tmp_path))
    state = {key: "pass"}
    assert merger.is_image_passing(image, tmp_path, state, include_unvalidated=False)
    fake = type("V", (), {"dataset_path": tmp_path, "image_annotation_pairs": [(image, image)]})()
    assert validator.YOLODatasetValidator.key_for(fake, 0) == key


def test_existing_root_state_wins(state_merger) -> None:
    merged = {"a": "fail", "b": "pass"}
    overridden = state_merger.apply_existing(merged, {"a": "pass", "c": "fail"})
    assert merged == {"a": "pass", "b": "pass", "c": "fail"}
    assert overridden == 1
