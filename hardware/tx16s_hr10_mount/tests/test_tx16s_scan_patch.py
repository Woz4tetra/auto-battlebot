"""Footprint guard in hardware/tx16s_hr10_mount/scan_patch.py."""

import importlib.util
from types import ModuleType

import numpy as np
import pytest

from auto_battlebot.paths import REPO_ROOT

MOUNT_DIR = REPO_ROOT / "hardware" / "tx16s_hr10_mount"


@pytest.fixture()
def scan_patch() -> ModuleType:
    spec = importlib.util.spec_from_file_location("scan_patch", MOUNT_DIR / "scan_patch.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _grid(scan_patch: ModuleType) -> tuple[np.ndarray, np.ndarray]:
    xs = scan_patch.WINDOW_X[0] + scan_patch.CELL * (np.arange(72) + 0.5)
    ys = scan_patch.WINDOW_Y[0] + scan_patch.CELL * (np.arange(100) + 0.5)
    return xs, ys


def test_corner_cell_outside_footprint_is_not_counted(scan_patch: ModuleType) -> None:
    xs, ys = _grid(scan_patch)
    empty = np.zeros((len(xs), len(ys)), dtype=bool)
    empty[0, 0] = True
    assert scan_patch.empty_cells_under_footprint(xs, ys, empty) == 0


def test_empty_cell_under_footprint_is_counted(scan_patch: ModuleType) -> None:
    xs, ys = _grid(scan_patch)
    empty = np.zeros((len(xs), len(ys)), dtype=bool)
    cx = int(np.argmin(np.abs(xs - scan_patch.FOOTPRINT_CENTER[0])))
    cy = int(np.argmin(np.abs(ys - scan_patch.FOOTPRINT_CENTER[1])))
    empty[cx, cy] = True
    assert scan_patch.empty_cells_under_footprint(xs, ys, empty) == 1
