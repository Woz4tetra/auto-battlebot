"""Fill the unseen part of a texture warped from one photo, by reflecting what was seen.

A floor or wall texture built from a single fixed camera has holes: the strip hidden behind a
rail, the wall below the box, anything past the frame edge. A flat mean colour in there reads
as a painted band from any other viewpoint. Reflecting the seen texture across the edge of
the hole carries the grain, the stone pattern and the lighting gradient on into it instead.

`mirror_fill` suits a narrow hole beside a large seen area (the floor strip behind a rail).
A hole larger than what was seen, like a wall seen only in a band above a box, comes out
kaleidoscopic from it: reflections of reflections fan out from every concave corner. There
`largest_seen_rectangle` and `offset_tile` cover the texture with shifted, overlapping copies
of one clean seen rectangle instead. Flipped copies would match at every seam but stack into
mirror axes the eye picks out at once; shifted ones with blended overlaps do not.

Used by `training/synthetic/build_cage_floor_texture.py` (floor albedo) and
`playground/basement_scene/build_wall_textures.py` (wall albedo).
"""

from __future__ import annotations

import cv2
import numpy as np


def _mirror_pass(image: np.ndarray, unseen: np.ndarray) -> np.ndarray:
    """One reflection across the nearest seen pixel; returns the unseen pixels it filled."""
    seen = (~unseen).astype(np.uint8)
    boundary = seen - cv2.erode(seen, np.ones((3, 3), np.uint8))
    rows, cols = np.nonzero(boundary)
    if len(rows) == 0:
        return np.zeros_like(unseen)
    source = np.ones(unseen.shape, np.uint8)
    source[rows, cols] = 0
    _, labels = cv2.distanceTransformWithLabels(
        source, cv2.DIST_L2, 5, labelType=cv2.DIST_LABEL_PIXEL
    )
    lookup_rows = np.zeros(labels.max() + 1, np.int64)
    lookup_cols = np.zeros(labels.max() + 1, np.int64)
    lookup_rows[labels[rows, cols]] = rows
    lookup_cols[labels[rows, cols]] = cols
    hole_rows, hole_cols = np.nonzero(unseen)
    nearest = labels[hole_rows, hole_cols]
    mirror_rows = 2 * lookup_rows[nearest] - hole_rows
    mirror_cols = 2 * lookup_cols[nearest] - hole_cols
    height, width = unseen.shape
    ok = (mirror_rows >= 0) & (mirror_rows < height) & (mirror_cols >= 0) & (mirror_cols < width)
    ok[ok] &= ~unseen[mirror_rows[ok], mirror_cols[ok]]
    image[hole_rows[ok], hole_cols[ok]] = image[mirror_rows[ok], mirror_cols[ok]]
    filled = np.zeros_like(unseen)
    filled[hole_rows[ok], hole_cols[ok]] = True
    return filled


def mirror_fill(image: np.ndarray, unseen: np.ndarray, max_passes: int = 8) -> np.ndarray:
    """Fill `unseen` pixels of `image` by reflection, repeating until the hole is closed.

    A pass reflects each hole pixel across its nearest seen pixel and takes the value there
    if that point was seen. A hole deeper than the seen band beside it is not closed in one
    pass, so later passes reflect what earlier ones filled: a strip of wall 0.8 m tall ends
    up tiled, flipped, down a 2 m wall. Pixels still unseen after `max_passes` keep their
    value.

    Args:
        image: HxW or HxWxC texture; not modified.
        unseen: HxW bool, True where the photo saw nothing.
        max_passes: upper bound on reflection passes.

    Returns:
        The filled copy of `image`.
    """
    out: np.ndarray = image.copy()
    remaining = unseen.copy()
    for _ in range(max_passes):
        if not remaining.any():
            break
        filled = _mirror_pass(out, remaining)
        if not filled.any():
            break
        remaining &= ~filled
    return out


def largest_seen_rectangle(seen: np.ndarray, step: int = 4) -> tuple[int, int, int, int]:
    """The largest axis-aligned rectangle of seen pixels, as (row0, row1, col0, col1).

    Maximal rectangle over the histogram of seen runs, row by row, on a `step`-downsampled
    mask; the bounds are shrunk to the downsampled cells that are wholly seen.
    """
    height, width = seen.shape
    cells = seen[: height // step * step, : width // step * step]
    cells = np.asarray(cells.reshape(height // step, step, width // step, step).all(axis=(1, 3)))
    runs = np.zeros(cells.shape[1], np.int64)
    best = (0, 0, 0, 0, 0)  # area, row0, row1, col0, col1 in cells
    for row in range(cells.shape[0]):
        runs = np.where(cells[row], runs + 1, 0)
        stack: list[int] = []
        for col in range(len(runs) + 1):
            current = runs[col] if col < len(runs) else 0
            while stack and runs[stack[-1]] >= current:
                top = stack.pop()
                left = stack[-1] + 1 if stack else 0
                area = int(runs[top]) * (col - left)
                if area > best[0]:
                    best = (area, row - int(runs[top]) + 1, row + 1, left, col)
            stack.append(col)
    _, row0, row1, col0, col1 = best
    return row0 * step, row1 * step, col0 * step, col1 * step


def flatten_luminance(image: np.ndarray, sigma_px: float) -> np.ndarray:
    """Divide out the low-frequency brightness of a BGR image, keeping its mean.

    A seen patch carries the photo's lighting gradient; copies of it laid side by side would
    show that gradient as a step at every seam.
    """
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2Lab).astype(np.float32)
    low = cv2.GaussianBlur(lab[:, :, 0], (0, 0), sigma_px)
    lab[:, :, 0] = np.clip(lab[:, :, 0] / np.maximum(low, 1.0) * float(lab[:, :, 0].mean()), 0, 255)
    return np.asarray(cv2.cvtColor(lab.astype(np.uint8), cv2.COLOR_Lab2BGR))


def offset_tile(
    patch: np.ndarray, shape: tuple[int, int], overlap: float = 0.25, seed: int = 0
) -> np.ndarray:
    """Cover an image of `shape` with copies of `patch`, blended where they overlap.

    Copies step by the patch size less `overlap` of it, each row of copies shifted by a
    random fraction of a step, and overlaps are cross-faded with linear ramps. The patch's
    low-frequency brightness is flattened first (`flatten_luminance`) so the fade does not
    have to hide a lighting step.
    """
    height, width = shape
    tile_h, tile_w = patch.shape[:2]
    flat = flatten_luminance(patch, min(tile_h, tile_w) / 4).astype(np.float32)
    ramp_h = max(1, int(tile_h * overlap))
    ramp_w = max(1, int(tile_w * overlap))
    weight = np.outer(
        np.minimum(np.minimum(np.arange(tile_h) + 1, tile_h - np.arange(tile_h)) / ramp_h, 1.0),
        np.minimum(np.minimum(np.arange(tile_w) + 1, tile_w - np.arange(tile_w)) / ramp_w, 1.0),
    ).astype(np.float32)
    step_h, step_w = tile_h - ramp_h, tile_w - ramp_w
    rng = np.random.default_rng(seed)
    total = np.zeros((height + 2 * tile_h, width + 3 * tile_w, 3), np.float32)
    weights = np.zeros(total.shape[:2], np.float32)
    for top in range(0, height + tile_h, step_h):
        shift = int(rng.integers(0, step_w))
        for left in range(-shift, width + tile_w, step_w):
            rows = slice(top, top + tile_h)
            cols = slice(left + tile_w, left + 2 * tile_w)
            total[rows, cols] += flat * weight[..., None]
            weights[rows, cols] += weight
    out = (
        total[:height, tile_w : tile_w + width]
        / np.maximum(weights[:height, tile_w : tile_w + width], 1e-6)[..., None]
    )
    return np.asarray(np.clip(out, 0, 255).astype(np.uint8))
