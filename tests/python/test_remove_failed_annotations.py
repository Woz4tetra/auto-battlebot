"""remove_failed_annotations.py and validate_yolo_dataset.py agree on the backup dir and layouts."""

from __future__ import annotations

import json
from pathlib import Path

import pytest


@pytest.fixture(scope="module")
def remover(load_training):
    return load_training("training/yolo/remove_failed_annotations.py")


@pytest.fixture(scope="module")
def validator(load_training):
    return load_training("training/yolo/validate_yolo_dataset.py")


def test_backup_dir_name_shared(remover, validator) -> None:
    assert remover.BACKUP_DIR_NAME == validator.BACKUP_DIR_NAME


def test_segmask_pair_is_moved(remover, tmp_path: Path) -> None:
    frame = tmp_path / "scene" / "f1.jpg"
    frame.parent.mkdir()
    frame.write_bytes(b"i")
    (tmp_path / "scene" / "f1_mask.png").write_bytes(b"m")
    pairs = remover.find_image_annotation_pairs(tmp_path)
    assert pairs == [(frame, tmp_path / "scene" / "f1_mask.png")]


def test_yolo_pair_is_found(remover, tmp_path: Path) -> None:
    (tmp_path / "images").mkdir()
    (tmp_path / "labels").mkdir()
    (tmp_path / "images" / "a.jpg").write_bytes(b"i")
    (tmp_path / "labels" / "a.txt").write_text("0 0 0 1 1\n")
    assert remover.find_image_annotation_pairs(tmp_path) == [
        (tmp_path / "images" / "a.jpg", tmp_path / "labels" / "a.txt")
    ]


def test_keys_relative_to_state_file_root(remover, tmp_path: Path) -> None:
    nested = tmp_path / "rec_a"
    nested.mkdir()
    (nested / "validation_state.json").write_text(json.dumps({"images/a.jpg": "fail"}))
    (tmp_path / "validation_state.json").write_text("{}")
    roots = [root for root, _ in remover.find_datasets_with_state(tmp_path)]
    assert sorted(roots) == sorted([tmp_path, nested])


def test_backup_dir_is_not_paired_again(remover, tmp_path: Path) -> None:
    backup = tmp_path / remover.BACKUP_DIR_NAME / "images"
    backup.mkdir(parents=True)
    (backup / "a.jpg").write_bytes(b"i")
    (backup / "a.txt").write_text("0 0 0 1 1\n")
    assert remover.find_image_annotation_pairs(tmp_path) == []


def test_conflicting_scene_yamls_report(validator, tmp_path: Path, capsys) -> None:
    for scene, names in (("s1", ["a", "b"]), ("s2", ["x"])):
        (tmp_path / scene).mkdir()
        (tmp_path / scene / "data.yaml").write_text(f"names: {names}\n")
    validator.find_class_yaml(tmp_path)
    assert "WARNING" in capsys.readouterr().out


def test_thumbnails_match_thumb_box(validator) -> None:
    from collections import OrderedDict
    from threading import Lock
    from types import SimpleNamespace

    from PIL import Image

    fake = SimpleNamespace(
        thumb_px=160,
        show_labels=False,
        cache_lock=Lock(),
        thumb_cache=OrderedDict(),
        cache_limit=8,
    )
    fake.thumb_box = lambda: validator.YOLODatasetValidator.thumb_box(fake)
    fake.thumb_key = lambda index: validator.YOLODatasetValidator.thumb_key(fake, index)
    fake.draw_annotations_off_main_thread = lambda index, box, detail: Image.new(
        "RGB", (box[0], 20)
    )
    thumb = validator.YOLODatasetValidator.render_thumb(fake, 0)
    assert thumb.size == fake.thumb_box()
