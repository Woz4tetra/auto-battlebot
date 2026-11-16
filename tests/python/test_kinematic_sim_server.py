"""kinematic_sim_server constants that mirror the C++ side."""

from __future__ import annotations

import math
import re
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "simulation"))

import kinematic_sim_server  # noqa: E402


def test_default_opponent_radius() -> None:
    source = (REPO_ROOT / "src" / "robot_filter" / "ground_truth_robot_filter.cpp").read_text()
    length, width = (float(v) for v in re.search(r"Size\{([\d.]+), ([\d.]+),", source).groups())
    assert kinematic_sim_server.DEFAULT_OPPONENT_RADIUS_M == pytest.approx(
        0.5 * math.hypot(length, width), abs=5e-4
    )
