"""PolicyOpponent steps and sizes the opponent like the RL environment and the C++ config."""

from __future__ import annotations

import math
import sys
from pathlib import Path

from auto_battlebot.compat import tomllib
from auto_battlebot.rl.envs.warp_nav import WarpNavConfig

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "simulation"))

import policy_opponent  # noqa: E402


def test_policy_period_matches_env() -> None:
    assert policy_opponent.POLICY_PERIOD_S == WarpNavConfig().policy_dt_s


def test_size_matches_config() -> None:
    common = tomllib.loads((REPO_ROOT / "config" / "_common.toml").read_text())
    sizes = common["robot_filter"]["robot_size_meters_per_label"]["MR_STABS_MK2"]
    assert list(policy_opponent.MR_STABS_MK2_SIZE_M) == sizes
    assert policy_opponent.MR_STABS_MK2_HALF_DIAGONAL_M == 0.5 * math.hypot(sizes[0], sizes[1])
