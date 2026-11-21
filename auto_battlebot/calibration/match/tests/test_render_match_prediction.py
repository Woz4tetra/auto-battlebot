"""render_match_prediction draws the robot at the size the simulation uses."""

from __future__ import annotations

from auto_battlebot.calibration.match import render_match_prediction
from auto_battlebot.compat import tomllib
from auto_battlebot.paths import REPO_ROOT


def test_robot_half_matches_sim() -> None:
    sim = tomllib.loads((REPO_ROOT / "simulation" / "sim_mrs_buff_mk3.toml").read_text())
    assert render_match_prediction.ROBOT_HALF_M == sim["our_robot"]["radius"]
