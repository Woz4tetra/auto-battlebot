"""Frame identity in the eval GT loader.

Run with ``venv/bin/pytest tests/python``.

Frames were keyed by stamp alone until 2026-09-18, which silently dropped 65 of
cage_high_x50_conf044's 636 frames: every recording sampled from video starts at stamp 0,
so the nine recordings' frame 0 collapsed into one.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from PIL import Image

from auto_battlebot.eval import FrameKey, load_gt, reviewed_frames

NAMES = ["mr_stabs_mk2", "mrs_buff_mk3", "opponent"]


def write_frame(dataset: Path, stem: str, rows: str = "2 0.5 0.5 0.2 0.2\n") -> None:
    """One image and its label file inside a dataset directory."""
    (dataset / "images").mkdir(parents=True, exist_ok=True)
    (dataset / "labels").mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.zeros((40, 60, 3), dtype=np.uint8)).save(dataset / "images" / f"{stem}.png")
    (dataset / "labels" / f"{stem}.txt").write_text(rows)


def write_dataset(dataset: Path, stems: list[str]) -> None:
    dataset.mkdir(parents=True, exist_ok=True)
    (dataset / "data.yaml").write_text("names:\n" + "".join(f"- {n}\n" for n in NAMES))
    for stem in stems:
        write_frame(dataset, stem)


@pytest.fixture
def two_recordings(tmp_path: Path) -> Path:
    """A root of two subdatasets that both number their first frame 0."""
    write_dataset(tmp_path / "rec_a", ["0000000000000000000", "0000000007500000000"])
    write_dataset(tmp_path / "rec_b", ["0000000000000000000", "0000000007500000000"])
    return tmp_path


def test_same_stamp_in_two_recordings_stays_two_frames(two_recordings: Path) -> None:
    frames, names, images = load_gt(two_recordings)
    assert len(frames) == 4
    assert len(images) == 4
    assert names == NAMES
    assert FrameKey("rec_a", 0) in frames
    assert FrameKey("rec_b", 0) in frames
    assert images[FrameKey("rec_a", 0)] != images[FrameKey("rec_b", 0)]


def test_each_frame_keeps_its_own_labels(two_recordings: Path) -> None:
    (two_recordings / "rec_b" / "labels" / "0000000000000000000.txt").write_text(
        "2 0.5 0.5 0.2 0.2\n1 0.25 0.25 0.1 0.1\n"
    )
    frames, _, _ = load_gt(two_recordings)
    assert frames[FrameKey("rec_a", 0)][1] == ["opponent"]
    assert frames[FrameKey("rec_b", 0)][1] == ["opponent", "mrs_buff_mk3"]


def test_single_dataset_root_keys_on_dot(tmp_path: Path) -> None:
    write_dataset(tmp_path, ["0000000000000000000"])
    frames, _, _ = load_gt(tmp_path)
    assert list(frames) == [FrameKey(".", 0)]


def test_review_state_selects_per_recording(two_recordings: Path) -> None:
    """A verdict on one recording's frame 0 must not reject another's."""
    (two_recordings / "validation_state.json").write_text(
        '{"rec_a/images/0000000000000000000.png": "pass",'
        ' "rec_b/images/0000000000000000000.png": "fail",'
        ' "rec_b/images/0000000007500000000.png": "pass"}'
    )
    assert reviewed_frames(two_recordings) == {
        ("rec_a", "0000000000000000000"),
        ("rec_b", "0000000007500000000"),
    }
    frames, _, _ = load_gt(two_recordings)
    assert set(frames) == {FrameKey("rec_a", 0), FrameKey("rec_b", 7500000000)}


def test_edit_state_used_only_without_validation_state(two_recordings: Path) -> None:
    (two_recordings / ".edit_state.json").write_text(
        '{"reviewed": ["rec_a/images/0000000000000000000.png"]}'
    )
    assert set(load_gt(two_recordings)[0]) == {FrameKey("rec_a", 0)}
    (two_recordings / "validation_state.json").write_text(
        '{"rec_b/images/0000000000000000000.png": "pass"}'
    )
    assert set(load_gt(two_recordings)[0]) == {FrameKey("rec_b", 0)}
