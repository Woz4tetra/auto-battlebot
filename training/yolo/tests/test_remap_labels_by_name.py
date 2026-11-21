"""training/yolo/remap_labels_by_name.py: one box per source class, grouped before the collapse."""

from __future__ import annotations

import pytest


@pytest.fixture(scope="module")
def remap(load_training):
    return load_training("training/yolo/remap_labels_by_name.py")


def _ring(class_id: int, x0: float, y0: float, x1: float, y1: float) -> str:
    steps = 20
    points = (
        [(x0 + (x1 - x0) * i / steps, y0) for i in range(steps)]
        + [(x1, y0 + (y1 - y0) * i / steps) for i in range(steps)]
        + [(x1 - (x1 - x0) * i / steps, y1) for i in range(steps)]
        + [(x0, y1 - (y1 - y0) * i / steps) for i in range(steps)]
    )
    return " ".join([str(class_id)] + [f"{v:.4f}" for point in points for v in point])


def test_one_box_per_source_class(remap) -> None:
    left = _ring(0, 0.10, 0.10, 0.30, 0.30)
    right = _ring(1, 0.60, 0.60, 0.90, 0.90)
    rows = remap.convert_rows(f"{left}\n{right}\n", {0: 0, 1: 0}, to_bbox=True)
    assert len(rows) == 2
    assert all(row.startswith("0 ") for row in rows)
    for row in rows:
        _, _, _, w, h = (float(v) for v in row.split())
        assert w < 0.5 and h < 0.5


def test_dropped_class_vanishes(remap) -> None:
    rows = remap.convert_rows(_ring(5, 0.1, 0.1, 0.3, 0.3) + "\n", {0: 0}, to_bbox=True)
    assert rows == []
