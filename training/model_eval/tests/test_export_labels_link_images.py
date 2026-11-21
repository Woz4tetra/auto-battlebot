"""export_labels.link_images never deletes a real images directory."""

from __future__ import annotations

from pathlib import Path

import pytest


@pytest.fixture(scope="module")
def export_labels(load_training):
    return load_training("training/model_eval/export_labels.py")


def test_replaces_a_stale_symlink(export_labels, tmp_path: Path) -> None:
    (tmp_path / "images").mkdir()
    dataset = tmp_path / "ds"
    dataset.mkdir()
    (dataset / "images").symlink_to(tmp_path / "elsewhere")
    export_labels.link_images(dataset / "images")
    assert (dataset / "images").resolve() == (tmp_path / "images").resolve()


def test_real_directory_is_an_error_and_keeps_its_files(export_labels, tmp_path: Path) -> None:
    link = tmp_path / "ds" / "images"
    link.mkdir(parents=True)
    (link / "keep.png").write_bytes(b"x")
    with pytest.raises(SystemExit):
        export_labels.link_images(link)
    assert (link / "keep.png").exists()
