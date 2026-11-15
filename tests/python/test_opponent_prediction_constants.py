"""Opponent-prediction fixtures and evaluations use the C++ estimator's numbers."""

from __future__ import annotations

import re
from pathlib import Path

from playground.opponent_prediction import (
    evaluate_opponent_prediction,
    make_opponent_filter_golden_fixture,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_HPP = REPO_ROOT / "include" / "robot_filter" / "motion_estimator_config.hpp"
ESTIMATOR_HPP = REPO_ROOT / "include" / "robot_filter" / "kalman_motion_estimator.hpp"


def _cpp_default(name: str) -> float:
    return float(re.search(rf"double {name} = ([\d.]+);", CONFIG_HPP.read_text()).group(1))


def test_golden_fixture_sigmas_match_cpp_defaults() -> None:
    fixture = make_opponent_filter_golden_fixture
    assert fixture.KEYPOINT_SIGMA == _cpp_default("opponent_position_sigma_m")
    assert fixture.BLOB_SIGMA == _cpp_default("blob_position_sigma_m")


def test_eval_latency_matches_estimator_documentation() -> None:
    milliseconds = int(
        re.search(r"measured (\d+) ms perception latency", ESTIMATOR_HPP.read_text()).group(1)
    )
    assert evaluate_opponent_prediction.LATENCY_S == milliseconds / 1000.0
