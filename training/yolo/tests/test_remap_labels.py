"""training/yolo/remap_labels.py: class-id remapping of label rows."""

from __future__ import annotations

from pathlib import Path

import pytest


@pytest.fixture(scope="module")
def remap(load_training):
    return load_training("training/yolo/remap_labels.py")


def test_remap_rewrites_class_ids_only(remap) -> None:
    unmapped: set[int] = set()
    result = remap.remap_label_lines(["1 0.5 0.5 0.1 0.1", "7 0.2 0.2 0.1 0.1"], {1: 0}, unmapped)
    assert result[0] == ["0 0.5 0.5 0.1 0.1", "7 0.2 0.2 0.1 0.1"]
    assert result[1] == {1, 7}
    assert result[2] == {0, 7}
    assert unmapped == {7}


def test_classes_txt_left_alone(remap) -> None:
    unmapped: set[int] = set()
    lines, before, after, malformed = remap.remap_label_lines(["person", "robot"], {0: 1}, unmapped)
    assert lines == []
    assert before == after == set()
    assert malformed == 2


def test_classes_txt_file_is_not_rewritten(remap, tmp_path: Path) -> None:
    labels = tmp_path / "labels"
    labels.mkdir()
    classes = labels / "classes.txt"
    classes.write_text("person\nrobot\n")
    remap.init_worker(str(tmp_path), str(labels), None, {0: 1}, True, False)
    result = remap.process_label_file(str(classes))
    assert result["written"] is False
    assert classes.read_text() == "person\nrobot\n"
