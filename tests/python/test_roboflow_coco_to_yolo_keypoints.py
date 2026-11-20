"""The data.yaml carries the keys Ultralytics requires, whatever Roboflow called the split."""

from __future__ import annotations

from pathlib import Path

import yaml

from playground import roboflow_coco_to_yolo_keypoints as converter


def _written(tmp_path: Path, split_map: dict[str, str]) -> dict:
    converter.write_data_yaml(tmp_path, ["robot"], 4, split_map)
    return yaml.safe_load((tmp_path / "data.yaml").read_text())


def test_roboflow_valid_split_is_written_under_the_val_key(tmp_path: Path) -> None:
    data = _written(tmp_path, {"train": "images/train", "valid": "images/valid"})

    assert data["val"] == "images/valid"
    assert "valid" not in data


def test_an_existing_val_split_keeps_the_val_key(tmp_path: Path) -> None:
    data = _written(tmp_path, {"train": "images/train", "val": "images/val"})

    assert data["val"] == "images/val"


def test_train_and_val_are_both_present_for_a_roboflow_export(tmp_path: Path) -> None:
    data = _written(tmp_path, {"train": "t", "valid": "v", "test": "x"})

    assert {"train", "val", "test"} <= data.keys()
    assert data["kpt_shape"] == [4, 3]
    assert data["nc"] == 1
