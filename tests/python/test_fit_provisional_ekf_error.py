"""The appended ekf_error table lands under robots.ours however the source file ends."""

from __future__ import annotations

from pathlib import Path

import pytest

from auto_battlebot.compat import tomllib
from playground.rl import fit_provisional_ekf_error as fit

REPO_ROOT = Path(__file__).resolve().parents[2]
SIGMA = {k: [0.1, 0.2] for k in ("along", "cross", "theta", "v", "omega")}
TAU = {k: 0.5 for k in SIGMA}


def _append(text: str) -> dict:
    return tomllib.loads(fit.append_ekf_error(text, "# source\n", SIGMA, TAU))


def test_table_follows_a_trailing_randomization_table_without_joining_it() -> None:
    parsed = _append("[robots.ours]\nname = 1\n\n[randomization]\nseed = 3\n")

    assert parsed["robots"]["ours"]["ekf_error"]["sigma"]["theta"] == [0.1, 0.2]
    assert parsed["robots"]["ours"]["ekf_error"]["tau_s"]["omega"] == 0.5
    assert parsed["randomization"] == {"seed": 3}


def test_table_follows_tables_that_come_after_randomization() -> None:
    parsed = _append("[robots.ours]\nname = 1\n\n[randomization]\nseed = 3\n\n[later]\nk = 2\n")

    assert "ekf_error" not in parsed["randomization"]
    assert "ekf_error" not in parsed["later"]
    assert parsed["robots"]["ours"]["ekf_error"]["tau_s"]["along"] == 0.5


def test_source_without_a_final_newline_still_parses() -> None:
    parsed = _append("[robots.ours]\nname = 1")

    assert parsed["robots"]["ours"]["name"] == 1
    assert "ekf_error" in parsed["robots"]["ours"]


def test_real_provisional_base_gains_the_table_under_robots_ours() -> None:
    text = (REPO_ROOT / "simulation/sensing_models/provisional.toml").read_text()

    parsed = _append(text)

    assert "ekf_error" in parsed["robots"]["ours"]
    assert "ekf_error" not in parsed["randomization"]


def test_source_that_already_has_the_table_is_rejected_by_the_toml_parser() -> None:
    text = fit.append_ekf_error("[robots.ours]\nname = 1\n", "# s\n", SIGMA, TAU)

    with pytest.raises(tomllib.TOMLDecodeError):
        tomllib.loads(fit.append_ekf_error(text, "# s\n", SIGMA, TAU))
