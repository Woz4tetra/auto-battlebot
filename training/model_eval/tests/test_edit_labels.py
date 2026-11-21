"""training/model_eval/edit_labels.py: data.yaml discovery and label file placement."""

from __future__ import annotations

from pathlib import Path

import pytest


@pytest.fixture(scope="module")
def editor(load_training):
    return load_training("training/model_eval/edit_labels.py")


def test_find_data_yaml_two_levels_deep(editor, tmp_path: Path) -> None:
    nested = tmp_path / "conf015" / "recording_a"
    nested.mkdir(parents=True)
    (nested / "data.yaml").write_text("names: [a]\n")
    assert editor._find_data_yaml(tmp_path) == nested / "data.yaml"


def test_find_data_yaml_prefers_the_dataset_root(editor, tmp_path: Path) -> None:
    (tmp_path / "data.yaml").write_text("names: [a]\n")
    (tmp_path / "sub").mkdir()
    (tmp_path / "sub" / "data.yaml").write_text("names: [b]\n")
    assert editor._find_data_yaml(tmp_path) == tmp_path / "data.yaml"


def test_label_path_swaps_images_dir(editor) -> None:
    assert editor._label_path_for(Path("dataset/images/x.png")) == Path("dataset/labels/x.txt")
    assert editor._label_path_for(Path("parent/rec/images/x.png")) == Path(
        "parent/rec/labels/x.txt"
    )
    assert editor._label_path_for(Path("images/rec/images/x.png")) == Path(
        "images/rec/labels/x.txt"
    )
