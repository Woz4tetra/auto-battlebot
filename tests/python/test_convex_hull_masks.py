"""training/deeplab/convex_hull_masks.py: speck rule and metadata replication."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest


@pytest.fixture(scope="module")
def hulls(load_training):
    return load_training("training/deeplab/convex_hull_masks.py")


def test_split_field_keeps_small_side(hulls) -> None:
    mask = np.zeros((100, 100), dtype=np.uint8)
    mask[10:90, 10:70] = 1
    mask[10:90, 74:84] = 1
    out, stats = hulls.convex_hull_mask(mask, 0.2, 0.05)
    assert stats.blobs_rescued == 1
    assert stats.blobs_dropped == 0
    assert out[50, 78] == 1


def test_speck_below_both_thresholds_is_dropped(hulls) -> None:
    mask = np.zeros((100, 100), dtype=np.uint8)
    mask[10:90, 10:70] = 1
    mask[95:97, 95:97] = 1
    out, stats = hulls.convex_hull_mask(mask, 0.2, 0.05)
    assert stats.blobs_dropped == 1
    assert out[95, 95] == 0


def test_metadata_is_copied_not_linked(hulls, tmp_path: Path) -> None:
    source = tmp_path / "src"
    source.mkdir()
    state = source / "validation_state.json"
    state.write_text("{}")
    image = source / "a.jpg"
    image.write_bytes(b"x")
    dest = tmp_path / "out"
    hulls.replicate_entry(state, dest / "validation_state.json")
    hulls.replicate_entry(image, dest / "a.jpg")
    assert (dest / "validation_state.json").stat().st_ino != state.stat().st_ino
    assert (dest / "a.jpg").stat().st_ino == image.stat().st_ino
    (dest / "validation_state.json").write_text('{"x": "pass"}')
    assert state.read_text() == "{}"
