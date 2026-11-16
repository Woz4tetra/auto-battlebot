"""training/model_eval/prelabel_dataset.py: only empty label files are ever candidates."""

from __future__ import annotations

from pathlib import Path

import pytest


@pytest.fixture(scope="module")
def prelabel(load_training):
    return load_training("training/model_eval/prelabel_dataset.py")


def _dataset(root: Path, labels: dict[str, str | None]) -> None:
    (root / "images").mkdir(parents=True)
    (root / "labels").mkdir()
    for stem, content in labels.items():
        (root / "images" / f"{stem}.png").write_bytes(b"x")
        if content is not None:
            (root / "labels" / f"{stem}.txt").write_text(content)


def test_never_overwrites_nonempty_labels(prelabel, tmp_path: Path) -> None:
    _dataset(tmp_path, {"done": "0 0.5 0.5 0.1 0.1\n", "blank": "", "absent": None})
    stems = [p.stem for p in prelabel.empty_frames(tmp_path)]
    assert stems == ["absent", "blank"]


def test_missing_labels_dir_is_fully_labelled(prelabel, tmp_path: Path) -> None:
    (tmp_path / "images").mkdir()
    for stem in ("a", "b"):
        (tmp_path / "images" / f"{stem}.png").write_bytes(b"x")
    assert [p.stem for p in prelabel.empty_frames(tmp_path)] == ["a", "b"]
