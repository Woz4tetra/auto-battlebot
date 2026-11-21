"""The velocity jig console converts IMU counts with the firmware's own scale factors."""

from __future__ import annotations

import re

from auto_battlebot.paths import REPO_ROOT

JIG = REPO_ROOT / "firmware" / "velocity_jig"


def _constant(text: str, pattern: str) -> float:
    match = re.search(pattern, text, re.MULTILINE)
    assert match, pattern
    return float(match.group(1))


def test_console_scale_factors_match_firmware_config() -> None:
    config_h = (JIG / "include" / "config.h").read_text()
    console = (JIG / "scripts" / "console.py").read_text()
    for name in ("GYRO_DPS_PER_LSB", "ACCEL_G_PER_LSB"):
        firmware = _constant(config_h, rf"constexpr float {name} = ([0-9.]+)f;")
        assert _constant(console, rf"^{name} = ([0-9.]+)$") == firmware, name
