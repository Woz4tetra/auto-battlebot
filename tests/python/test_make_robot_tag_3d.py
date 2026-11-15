"""The printed robot tag mesh is the tag a top-down camera decodes, not its mirror."""

from __future__ import annotations

import cv2
import numpy as np
import pytest

from playground.calibration import make_robot_tag_3d as tag3d

TAG_MM = 100.0
MARGIN_MM = 10.0
PPM = 8.0


def _top_down_image(tag_id: int, flip_rows: bool = False) -> np.ndarray:
    dictionary = cv2.aruco.getPredefinedDictionary(tag3d.DICTIONARY)
    grid = tag3d.bit_grid(dictionary, tag_id, dictionary.markerSize + 2 * tag3d.BORDER_BITS)
    if flip_rows:
        grid = grid[::-1]
    boxes = tag3d.black_cell_boxes(np.array([MARGIN_MM, MARGIN_MM]), TAG_MM, grid, 2.0, 0.6)
    plate = TAG_MM + 2 * MARGIN_MM
    size = round(plate * PPM)
    image = np.full((size, size), 255, np.uint8)
    for box in boxes:
        (x0, y0, _), (x1, y1, _) = box.bounds
        col0, col1 = round(x0 * PPM), round(x1 * PPM)
        row0, row1 = round(size - y1 * PPM), round(size - y0 * PPM)
        image[row0:row1, col0:col1] = 0
    return image


def _decoded_ids(image: np.ndarray) -> list[int]:
    dictionary = cv2.aruco.getPredefinedDictionary(tag3d.DICTIONARY)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    _corners, ids, _rejected = detector.detectMarkers(image)
    return [] if ids is None else [int(i) for i in ids.ravel()]


@pytest.mark.parametrize("tag_id", [0, 7])
def test_mesh_decodes_unmirrored(tag_id: int) -> None:
    assert _decoded_ids(_top_down_image(tag_id)) == [tag_id]


def test_vertically_mirrored_mesh_does_not_decode() -> None:
    assert _decoded_ids(_top_down_image(0, flip_rows=True)) != [0]
