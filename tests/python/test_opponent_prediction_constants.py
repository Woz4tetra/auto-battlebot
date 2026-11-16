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
RUNNER_CONFIG_HPP = REPO_ROOT / "include" / "runner_config.hpp"


def _cpp_default(name: str) -> float:
    return float(re.search(rf"double {name} = ([\d.]+);", CONFIG_HPP.read_text()).group(1))


def test_golden_fixture_sigmas_match_cpp_defaults() -> None:
    fixture = make_opponent_filter_golden_fixture
    assert fixture.KEYPOINT_SIGMA == _cpp_default("opponent_position_sigma_m")
    assert fixture.BLOB_SIGMA == _cpp_default("blob_position_sigma_m")


def test_rewind_history_covers_eval_latency() -> None:
    capacity = int(re.search(r"kSnapshotCapacity = (\d+);", ESTIMATOR_HPP.read_text()).group(1))
    loop_rate_hz = float(
        re.search(r"max_loop_rate\(([\d.]+)\)", RUNNER_CONFIG_HPP.read_text()).group(1)
    )
    assert capacity / loop_rate_hz > evaluate_opponent_prediction.LATENCY_S
