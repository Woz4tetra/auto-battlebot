"""fit_sensing_model footprint radii follow the configured robot sizes."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from auto_battlebot.compat import tomllib
from playground.sensing_model import fit_sensing_model

REPO_ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize("label", ["mrs_buff_mk3", "mr_stabs_mk2", "house_bot"])
def test_radius_matches_config_sizes(label: str) -> None:
    common = tomllib.loads((REPO_ROOT / "config" / "_common.toml").read_text())
    length, width, _height = common["robot_filter"]["robot_size_meters_per_label"][label.upper()]
    assert fit_sensing_model.RADIUS_BY_LABEL[label] == pytest.approx(
        0.25 * (length + width), abs=5e-4
    )


def _rows(stamps: list[float]) -> np.ndarray:
    return np.array([[t, 0.0, 0.0, 0.0] for t in stamps])


def test_a_measurement_logged_twice_in_a_row_is_counted_once() -> None:
    kept = fit_sensing_model.drop_repeated_measurements(_rows([1.0, 1.0, 1.033, 1.033, 1.066]))

    assert kept[:, 0].tolist() == [1.0, 1.033, 1.066]


def test_stamps_within_ten_milliseconds_collapse_and_wider_gaps_survive() -> None:
    kept = fit_sensing_model.drop_repeated_measurements(_rows([1.0, 1.009, 1.021, 1.0311]))

    assert kept[:, 0].tolist() == [1.0, 1.021, 1.0311]


def test_the_first_row_is_always_kept() -> None:
    assert len(fit_sensing_model.drop_repeated_measurements(_rows([5.0]))) == 1
