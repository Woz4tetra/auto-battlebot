"""Playground scripts that replay rl_eval_server logs use its policy tick."""

from __future__ import annotations

import re

from auto_battlebot.nav import repro_1025
from auto_battlebot.paths import REPO_ROOT
from auto_battlebot.rl import fit_provisional_ekf_error


def _server_tick_s() -> float:
    source = (REPO_ROOT / "auto_battlebot" / "kinematic_sim" / "rl_eval_server.py").read_text()
    return float(re.search(r"^TICK_S = ([\d.]+)", source, re.M).group(1))


def test_repro_1025_tick_matches_eval_server() -> None:
    assert repro_1025.TICK_S == _server_tick_s()


def test_tick_matches_eval_server() -> None:
    assert fit_provisional_ekf_error.TICK_S == _server_tick_s()
