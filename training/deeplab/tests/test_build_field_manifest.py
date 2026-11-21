"""training/deeplab/build_field_manifest.py: duplicate pairing across splits, relabel resolution."""

from __future__ import annotations

import os
from pathlib import Path

import pytest


@pytest.fixture(scope="module")
def manifest(load_training):
    return load_training("training/deeplab/build_field_manifest.py")


def _record(path: str, canonical: str, prefixed: bool, split: str) -> dict:
    return {
        "path": path,
        "mask": path.replace(".jpg", "_mask.png"),
        "canonical": canonical,
        "prefixed": prefixed,
        "split": split,
        "duplicate_of": None,
    }


def test_pair_duplicates_across_splits(manifest) -> None:
    original = _record("train/a.jpg", "a", False, "train")
    copy = _record("val/floor_dataset__a.jpg", "a", True, "val")
    lone = _record("val/floor_dataset__b.jpg", "b", True, "val")
    assert manifest.pair_duplicates([original, copy, lone]) == 1
    assert copy["duplicate_of"] == "train/a.jpg"
    assert lone["duplicate_of"] is None


def test_verify_duplicates_keeps_newer_mask(manifest, tmp_path: Path) -> None:
    for name, content in (
        ("a.jpg", b"img"),
        ("old_mask.png", b"old"),
        ("new_mask.png", b"new"),
        ("copy.jpg", b"img"),
    ):
        (tmp_path / name).write_bytes(content)
    os.utime(tmp_path / "old_mask.png", (1_000, 1_000))
    os.utime(tmp_path / "new_mask.png", (2_000, 2_000))
    original = {"path": "a.jpg", "mask": "old_mask.png", "duplicate_of": None}
    duplicate = {"path": "copy.jpg", "mask": "new_mask.png", "duplicate_of": "a.jpg"}
    report = manifest.verify_duplicates(
        {"frames": [original, duplicate]}, tmp_path, "md5", workers=1
    )
    assert report["same_image_different_mask"] == 1
    assert original["newer_mask"] == "new_mask.png"
