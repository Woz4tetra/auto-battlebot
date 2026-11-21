"""Result rows without episode_s are scored as 20 s episodes; rows with it use their own length."""

from __future__ import annotations

import math
from typing import Any

import pytest

from playground.rl import evaluate_nav_policy as evaluate


def _row(index: int, **overrides: Any) -> dict[str, Any]:
    row: dict[str, Any] = {
        "index": index,
        "strikes": 0,
        "hits_taken": 0,
        "first_strike_s": math.inf,
        "first_hit_s": math.inf,
        "danger_s": 0.0,
    }
    return row | overrides


def test_rows_without_episode_s_are_rejected() -> None:
    with pytest.raises(KeyError, match="episode_s"):
        evaluate.table([_row(0), _row(1)])


def test_rows_with_episode_s_use_their_own_length() -> None:
    result = evaluate.table([_row(0, episode_s=30.0), _row(1, episode_s=30.0)])

    assert result["seconds_per_episode"] == pytest.approx(30.0)
    assert result["median_time_to_first_strike_s"] == pytest.approx(30.0)
