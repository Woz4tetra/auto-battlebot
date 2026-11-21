"""The golden fixture steps the plant at its own substep so C++ and Python agree step for step."""

from __future__ import annotations

import inspect

from auto_battlebot.calibration.jig import drive_protocol
from auto_battlebot.control.plant import SUBSTEP_S
from auto_battlebot.script_loader import load_script

fixture = load_script("tests/fixtures/generators/make_plant_golden_fixture.py")


def test_dt_equals_plant_substep() -> None:
    assert fixture.DT == SUBSTEP_S


def test_record_stride_is_ten_milliseconds() -> None:
    assert fixture.RECORD_STRIDE * fixture.DT == 0.01


def test_command_tick_matches_drive_rate() -> None:
    default_rate = inspect.signature(drive_protocol.play).parameters["rate_hz"].default
    assert fixture.CMD_TICK_S == 1.0 / default_rate
