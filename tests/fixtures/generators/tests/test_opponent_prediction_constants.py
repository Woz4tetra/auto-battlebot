"""The opponent filter golden fixture uses the C++ estimator's numbers."""

from __future__ import annotations

import re

from auto_battlebot.paths import REPO_ROOT
from auto_battlebot.script_loader import load_script

make_opponent_filter_golden_fixture = load_script(
    "tests/fixtures/generators/make_opponent_filter_golden_fixture.py"
)


CONFIG_HPP = REPO_ROOT / "include" / "robot_filter" / "motion_estimator_config.hpp"


def _cpp_default(name: str) -> float:
    return float(re.search(rf"double {name} = ([\d.]+);", CONFIG_HPP.read_text()).group(1))


def test_golden_fixture_sigmas_match_cpp_defaults() -> None:
    fixture = make_opponent_filter_golden_fixture
    assert fixture.KEYPOINT_SIGMA == _cpp_default("opponent_position_sigma_m")
