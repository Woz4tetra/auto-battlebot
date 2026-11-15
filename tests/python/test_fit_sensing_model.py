"""fit_sensing_model footprint radii follow the configured robot sizes."""

from __future__ import annotations

from pathlib import Path

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
