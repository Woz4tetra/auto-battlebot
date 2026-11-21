"""training/yolo/make_seg_dataset.py: polygon rows and floor cutouts."""

from __future__ import annotations

import numpy as np
import pytest
from shapely.errors import GEOSException
from shapely.geometry import MultiPolygon, Point, Polygon


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


class _BatchRejectingFloor:
    is_empty = False
    is_valid = True

    def __init__(self, polygon: Polygon) -> None:
        self.polygon = polygon

    def difference(self, other):
        if isinstance(other, MultiPolygon):
            raise GEOSException("batched difference failed")
        return self.polygon.difference(other)


def test_difference_falls_back_to_one_obstacle_at_a_time(seg, monkeypatch) -> None:
    floor = _BatchRejectingFloor(Polygon(_square(0.0, 0.0, 1.0, 1.0)))
    obstacles = MultiPolygon(
        [Polygon(_square(0.1, 0.1, 0.3, 0.3)), Polygon(_square(0.6, 0.6, 0.9, 0.9))]
    )
    real_explode = seg.explode_to_polygons
    monkeypatch.setattr(
        seg,
        "explode_to_polygons",
        lambda g: [g.polygon] if isinstance(g, _BatchRejectingFloor) else real_explode(g),
    )
    with pytest.raises(GEOSException):
        floor.difference(obstacles)
    result = seg.robust_difference(floor, obstacles)
    assert result.area == pytest.approx(1.0 - 0.04 - 0.09)
    assert not result.contains(Point(0.2, 0.2))
    assert result.contains(Point(0.5, 0.5))
