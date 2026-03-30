#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path

import numpy as np

TEST_DIR = Path(__file__).resolve().parent
FLOOR_DIR = TEST_DIR.parent
sys.path.insert(0, str(FLOOR_DIR))

from quality_filter import cleanup_mask, iou  # noqa: E402


class TestQualityMetrics(unittest.TestCase):
    def test_iou(self) -> None:
        a = np.zeros((10, 10), dtype=np.uint8)
        b = np.zeros((10, 10), dtype=np.uint8)
        a[2:8, 2:8] = 1
        b[4:9, 4:9] = 1
        value = iou(a, b)
        self.assertGreater(value, 0.0)
        self.assertLess(value, 1.0)

    def test_cleanup_mask_keeps_main_blob(self) -> None:
        mask = np.zeros((64, 64), dtype=np.uint8)
        mask[10:50, 10:50] = 255
        mask[3, 3] = 255  # noise
        cleaned = cleanup_mask(mask, open_kernel=3, close_kernel=3)
        self.assertEqual(int(cleaned[3, 3]), 0)
        self.assertEqual(int(cleaned[20, 20]), 1)


if __name__ == "__main__":
    unittest.main()
