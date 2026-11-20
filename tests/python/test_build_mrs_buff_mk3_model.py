"""CoACD hulls that bulge below the CAD floor are cut off at it."""

from __future__ import annotations

import numpy as np
import trimesh

from playground.calibration import build_mrs_buff_mk3_model as build

FLOOR = 0.00035


def _box(z_low: float) -> np.ndarray:
    return np.asarray(trimesh.creation.box(extents=(0.02, 0.02, 0.02)).vertices) + [
        0,
        0,
        z_low + 0.01,
    ]


def test_hull_bulging_below_the_floor_is_cut_at_the_floor() -> None:
    hull = build.clip_hull_to_floor(_box(FLOOR - 0.001), FLOOR)
    assert hull is not None
    assert float(hull.bounds[0, 2]) == FLOOR


def test_hull_above_the_floor_is_untouched() -> None:
    assert build.clip_hull_to_floor(_box(FLOOR + 0.001), FLOOR) is None
