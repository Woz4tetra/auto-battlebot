from __future__ import annotations

import importlib.util
import re
import sys

from auto_battlebot.paths import REPO_ROOT

ROOT = REPO_ROOT
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


def row_of(count: int) -> str:
    return ",".join(str(i) for i in range(count))


def test_pad_row_full_width_passes_through() -> None:
    row = row_of(len(twitch.COLUMNS))
    assert twitch.pad_row(row) == row


def test_pad_row_pre_guard_firmware_keeps_seq_last() -> None:
    short = len(twitch.COLUMNS) - len(twitch.GUARD_COLUMNS)
    padded = twitch.pad_row(row_of(short))
    assert padded is not None
    cells = padded.split(",")
    assert len(cells) == len(twitch.COLUMNS)
    assert cells[-1] == str(short - 1)
    assert cells[short - 1 : short - 1 + len(twitch.GUARD_COLUMNS)] == ["nan"] * len(
        twitch.GUARD_COLUMNS
    )


def test_pad_row_pre_yaw_firmware_pads_trailing_columns() -> None:
    short = len(twitch.COLUMNS) - len(twitch.OPTIONAL_COLUMNS)
    padded = twitch.pad_row(row_of(short))
    assert padded is not None
    cells = padded.split(",")
    assert len(cells) == len(twitch.COLUMNS)
    assert cells[short:] == ["nan"] * (len(twitch.COLUMNS) - short)


def test_pad_row_rejects_truncated_and_oversized_rows() -> None:
    assert twitch.pad_row(row_of(len(twitch.COLUMNS) - len(twitch.OPTIONAL_COLUMNS) - 1)) is None
    assert twitch.pad_row(row_of(len(twitch.COLUMNS) + 1)) is None
