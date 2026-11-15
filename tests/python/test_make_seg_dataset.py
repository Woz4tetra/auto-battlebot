"""training/yolo/make_seg_dataset.py: polygon rows and floor cutouts."""

from __future__ import annotations

import numpy as np
import pytest
from shapely.geometry import Point, Polygon


@pytest.fixture(scope="module")
def seg(load_training):
    return load_training("training/yolo/make_seg_dataset.py")


def test_seg_line_has_no_closure_point(seg) -> None:
    square = Polygon([(0.1, 0.1), (0.9, 0.1), (0.9, 0.9), (0.1, 0.9)])
    line = seg.to_yolo_seg_line(3, square)
    assert line is not None
    values = line.split()[1:]
    assert len(values) == 8


def _square(x0: float, y0: float, x1: float, y1: float) -> list[tuple[float, float]]:
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def test_floor_holes_survive_as_cutouts(seg) -> None:
    mask = np.zeros((100, 100), dtype=np.uint8)
    mask[5:95, 5:95] = 255
    mask[40:60, 40:60] = 0
    pieces = seg._pieces_from_mask(mask, 99, 1e-4)
    assert pieces
    centre = Point(0.5, 0.5)
    assert not any(piece.contains(centre) for piece in pieces)
    assert sum(piece.area for piece in pieces) < 0.8


def test_invalid_robot_polygon_still_cut_from_floor(seg) -> None:
    bowtie = [(0.3, 0.3), (0.7, 0.7), (0.7, 0.3), (0.3, 0.7)]
    rows = [(0, _square(0.05, 0.05, 0.95, 0.95)), (1, bowtie)]
    floor_polys, obstacle_polys = seg.gather_polygons(rows, floor_class_id=0)
    mask = seg._build_floor_minus_obstacle_mask(rows, floor_polys, obstacle_polys, 0, 100, 99)
    assert mask is not None
    assert mask[50, 35] == 0 or mask[35, 50] == 0
    assert mask[10, 10] == 255
