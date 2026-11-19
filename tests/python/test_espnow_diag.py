from __future__ import annotations

import importlib.util
import re
import sys
from pathlib import Path
from types import ModuleType

import pytest

ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = ROOT / "firmware"
PROTOCOL_HEADER = FIRMWARE / "shared" / "espnow_protocol" / "src" / "espnow_protocol.h"
DIAG_SCRIPT = FIRMWARE / "espnow_dongle" / "scripts" / "espnow_diag.py"


@pytest.fixture(scope="module")
def diag() -> ModuleType:
    spec = importlib.util.spec_from_file_location("espnow_diag_under_test", DIAG_SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def header_constant(name: str) -> int:
    match = re.search(rf"\b{name}\s*=\s*(0x[0-9A-Fa-f]+|\d+)", PROTOCOL_HEADER.read_text())
    assert match is not None, name
    return int(match.group(1), 0)


def firmware_constant(path: Path, name: str) -> int:
    match = re.search(rf"\b{name}\s*=\s*(\d+)", path.read_text())
    assert match is not None, name
    return int(match.group(1))


def test_diag_constants_match_header(diag: ModuleType) -> None:
    assert int.from_bytes(diag.SERIAL_MAGIC, "little") == header_constant("SERIAL_MAGIC")
    assert diag.SERIAL_OVERHEAD == header_constant("SERIAL_OVERHEAD")
    for name in (
        "MSG_COMMAND",
        "MSG_TELEMETRY",
        "MSG_TUNABLE_SET",
        "MSG_LINK_STATS",
        "MSG_PING",
        "MSG_OTA_BEGIN",
        "MSG_OTA_CHUNK",
        "MSG_OTA_END",
        "MSG_OTA_ABORT",
        "MSG_OTA_STATUS",
    ):
        assert getattr(diag, name) == header_constant(name), name


def test_periods_match_firmware_headers(diag: ModuleType) -> None:
    link_header = FIRMWARE / "mr_stabs_mk2" / "include" / "espnow_link.h"
    dongle_main = FIRMWARE / "espnow_dongle" / "src" / "main.cpp"
    assert diag.TELEMETRY_PERIOD_MS == firmware_constant(link_header, "TELEMETRY_PERIOD_MS")
    assert diag.LINK_STATS_PERIOD_MS == firmware_constant(dongle_main, "LINK_STATS_PERIOD_MS")


def test_frame_parser_magic_split_across_feeds(diag: ModuleType) -> None:
    frame = diag.encode_frame(diag.MSG_PING, b"")
    parser = diag.FrameParser()
    frames, _ = parser.feed(frame[:1])
    assert frames == []
    frames, _ = parser.feed(frame[1:])
    assert frames == [(diag.MSG_PING, b"")]


def test_frame_parser_skips_boot_banner(diag: ModuleType) -> None:
    frame = diag.encode_frame(diag.MSG_PING, b"")
    parser = diag.FrameParser()
    banner = b"\nespnow_dongle mac=AA:BB:CC:DD:EE:FF channel=1\n"
    frames, _ = parser.feed(banner + frame)
    assert frames == [(diag.MSG_PING, b"")]


DIAG_WEB_APP = FIRMWARE / "espnow_dongle" / "scripts" / "espnow_diag_web" / "app.js"
MOTOR_GUARD_HEADER = FIRMWARE / "mr_stabs_mk2" / "lib" / "motor_guard" / "motor_guard.h"
ROBOT_MAIN = FIRMWARE / "mr_stabs_mk2" / "src" / "main.cpp"


def cpp_enum_names(name: str) -> list[str]:
    match = re.search(
        rf"enum class {name} : uint8_t \{{([^}}]*)\}}", MOTOR_GUARD_HEADER.read_text()
    )
    assert match is not None, name
    return [item.strip() for item in match.group(1).split(",") if item.strip()]


def js_label_array(name: str) -> list[str]:
    match = re.search(rf"const {name} = \[([^\]]*)\]", DIAG_WEB_APP.read_text())
    assert match is not None, name
    return re.findall(r'"([^"]*)"', match.group(1))


@pytest.mark.parametrize(
    ("js_name", "cpp_name"), [("GUARD_STATE", "State"), ("GUARD_SENSORS", "Sensors")]
)
def test_guard_labels_follow_motor_guard_enum_order(js_name: str, cpp_name: str) -> None:
    cpp = cpp_enum_names(cpp_name)
    js = [label.replace(" ", "_") for label in js_label_array(js_name)]
    assert js == cpp


def test_expected_robot_percents_match_firmware_substitution() -> None:
    source = ROBOT_MAIN.read_text()
    assert "radio_data->a_percent = -100.0f * cmd.channel_a;" in source
    assert "radio_data->b_percent = 100.0f * cmd.channel_b;" in source
    assert "-100 * channelA * (upsideDown ? -1 : 1), 100 * channelB" in DIAG_WEB_APP.read_text()
