#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path

import numpy as np

TEST_DIR = Path(__file__).resolve().parent
FLOOR_DIR = TEST_DIR.parent
sys.path.insert(0, str(FLOOR_DIR))

from export_dataset import contour_to_yolo_polygon  # noqa: E402


class TestExportPolygon(unittest.TestCase):
    def test_polygon_is_normalized(self) -> None:
        mask = np.zeros((100, 100), dtype=np.uint8)
        mask[20:80, 30:90] = 255
        poly = contour_to_yolo_polygon(
            mask_u8=mask,
            epsilon_ratio=0.002,
            min_polygon_points=4,
        )
        self.assertGreater(len(poly), 0)
        self.assertEqual(len(poly) % 2, 0)
        self.assertGreaterEqual(min(poly), 0.0)
        self.assertLessEqual(max(poly), 1.0)


if __name__ == "__main__":
    unittest.main()
