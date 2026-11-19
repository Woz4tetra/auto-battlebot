from __future__ import annotations

import importlib.util
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "firmware/mr_stabs_mk2/scripts/log_drive_twitch.py"
UPDOWN_H = ROOT / "firmware/mr_stabs_mk2/include/updown_sensor.h"
CRSF_BRIDGE_H = ROOT / "firmware/mr_stabs_mk2/include/crsf_bridge.h"

spec = importlib.util.spec_from_file_location("log_drive_twitch_under_test", SCRIPT)
assert spec is not None and spec.loader is not None
twitch = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = twitch
spec.loader.exec_module(twitch)


def header_float(name: str) -> float:
    match = re.search(rf"{name}\s*=\s*(-?[\d.]+);", UPDOWN_H.read_text())
    assert match is not None, name
    return float(match.group(1))


def test_flip_thresholds_are_firmware_thresholds_on_negated_gravity_z() -> None:
    assert twitch.UPSIDE_DOWN_BELOW == -header_float("UPSIDE_DOWN_THRESHOLD")
    assert twitch.RIGHT_SIDE_UP_ABOVE == -header_float("RIGHT_SIDE_UP_THRESHOLD")


def test_flip_switch_down_matches_crsf_bridge_enum() -> None:
    match = re.search(r"\bDOWN\s*=\s*(\d+)", CRSF_BRIDGE_H.read_text())
    assert match is not None
    assert twitch.FLIP_SWITCH_DOWN == int(match.group(1))
