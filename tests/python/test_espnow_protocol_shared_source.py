"""The ESP-NOW wire format is one source file built by the robot, the dongle, and the library."""

from __future__ import annotations

import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SHARED_DIR = "firmware/shared/espnow_protocol/src"


def test_one_protocol_source_exists() -> None:
    tracked = subprocess.run(
        ["git", "ls-files", "*/espnow_protocol.cpp"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=True,
    ).stdout.split()
    assert tracked == [f"{SHARED_DIR}/espnow_protocol.cpp"]


def test_library_builds_the_shared_source() -> None:
    assert SHARED_DIR in (ROOT / "CMakeLists.txt").read_text()


def test_firmware_builds_find_the_shared_library() -> None:
    for firmware in ("espnow_dongle", "mr_stabs_mk2"):
        ini = (ROOT / "firmware" / firmware / "platformio.ini").read_text()
        assert "lib_extra_dirs = ../shared" in ini, firmware
