"""auto_battlebot/nav/step_test.py converts slew rates with the profile's motor speed."""

from __future__ import annotations

from auto_battlebot.config_profile import load_profile_toml
from auto_battlebot.nav import step_test


def test_max_motor_rpm_matches_profile() -> None:
    profile = load_profile_toml("mr_stabs_mk2_fid_zed_box")
    assert step_test.MAX_MOTOR_RPM == profile["transmitter"]["max_motor_rpm"]
