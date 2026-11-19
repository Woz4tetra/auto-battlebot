"""Winding of the TX16S handle bar section loops that tx16s_hr10_handle_clamp relies on."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

HANDLE_DIR = Path(__file__).resolve().parents[2] / "hardware" / "tx16s_hr10_handle_clamp"
RING_CENTER_YZ = (60.0, 0.0)
RING_RADIUS = 10.0
RING_POINTS = 2000


def _load_bar_scan():
    spec = importlib.util.spec_from_file_location("bar_scan", HANDLE_DIR / "bar_scan.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _signed_area(loop: np.ndarray) -> float:
    y, z = loop[:, 0], loop[:, 1]
    return 0.5 * float(np.sum(y * np.roll(z, -1) - np.roll(y, -1) * z))


def _ring_vertices(x: float) -> np.ndarray:
    t = np.linspace(-np.pi, np.pi, RING_POINTS, endpoint=False)
    y = RING_CENTER_YZ[0] + RING_RADIUS * np.cos(t)
    z = RING_CENTER_YZ[1] + RING_RADIUS * np.sin(t)
    return np.stack([np.full_like(t, x), y, z], axis=1)


def _assert_ccw_from_minus_pi(loop: np.ndarray) -> None:
    centre = loop.mean(axis=0)
    first = loop[0] - centre
    assert _signed_area(loop) > 0
    assert np.arctan2(first[1], first[0]) == pytest.approx(-np.pi + np.pi / len(loop), abs=0.15)


def test_section_loop_runs_ccw_from_minus_pi() -> None:
    bar_scan = _load_bar_scan()
    loop = bar_scan.section_loop(_ring_vertices(60.0), 60.0)
    _assert_ccw_from_minus_pi(loop)


def test_stored_sections_run_ccw_from_minus_pi() -> None:
    sections = np.load(HANDLE_DIR / "tx16s_handle_bar_sections.npz")
    for loop in sections["loops"]:
        _assert_ccw_from_minus_pi(loop)
