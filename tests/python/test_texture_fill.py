"""Hole filling for textures warped from one photo: what gets filled, and with what."""

from __future__ import annotations

import numpy as np

from auto_battlebot.perception.texture_fill import (
    largest_seen_rectangle,
    mirror_fill,
    offset_tile,
)


def test_mirror_fill_reflects_across_the_hole_edge() -> None:
    # A ramp seen in columns 0..9 and a hole in columns 10..13 at the image edge: each hole
    # column takes the value mirrored across column 9.
    image = np.tile(np.arange(14, dtype=np.uint8) * 10, (6, 1))
    unseen = np.zeros(image.shape, bool)
    unseen[:, 10:14] = True
    filled = mirror_fill(image, unseen)
    assert (filled[:, :10] == image[:, :10]).all()
    for col in range(10, 14):
        assert (filled[:, col] == image[0, 18 - col]).all()


def test_mirror_fill_closes_a_hole_deeper_than_the_seen_band() -> None:
    image = np.zeros((40, 8), np.uint8)
    image[:6] = 100
    unseen = np.zeros(image.shape, bool)
    unseen[6:] = True
    filled = mirror_fill(image, unseen, max_passes=12)
    assert (filled == 100).all()


def test_largest_seen_rectangle_finds_the_block() -> None:
    seen = np.zeros((64, 96), bool)
    seen[8:40, 16:80] = True
    seen[48:52, 0:8] = True  # a small island that must lose
    row0, row1, col0, col1 = largest_seen_rectangle(seen, step=4)
    assert (row0, row1, col0, col1) == (8, 40, 16, 80)
    assert seen[row0:row1, col0:col1].all()


def test_offset_tile_covers_the_shape_at_the_patch_mean() -> None:
    rng = np.random.default_rng(0)
    patch = rng.integers(60, 200, size=(30, 50, 3), dtype=np.uint8)
    out = offset_tile(patch, (97, 211))
    assert out.shape == (97, 211, 3)
    assert out.min() > 0
    assert abs(float(out.mean()) - float(patch.mean())) < 12.0
