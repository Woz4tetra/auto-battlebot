"""playground/nav_eval/step_test.py converts slew rates with the profile's motor speed."""

from __future__ import annotations

from pathlib import Path

from auto_battlebot.compat import tomllib
from playground.nav_eval import step_test

REPO_ROOT = Path(__file__).resolve().parents[2]


def test_max_motor_rpm_matches_profile() -> None:
    profile = tomllib.loads((REPO_ROOT / "config" / "mr_stabs_mk2_fid_zed_box.toml").read_text())
    assert step_test.MAX_MOTOR_RPM == profile["transmitter"]["max_motor_rpm"]
