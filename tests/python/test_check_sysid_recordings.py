"""check_sysid_recordings thresholds track the code that consumes the same recordings."""

from __future__ import annotations

import re
from pathlib import Path

from auto_battlebot.perception.sysid_windows import WindowOptions
from playground.calibration import check_sysid_recordings as check

REPO_ROOT = Path(__file__).resolve().parents[2]


def test_tag_coverage_matches_smoother() -> None:
    assert check.TAG_COVERAGE_PASS == WindowOptions().coverage_min


def test_telemetry_hz_matches_firmware() -> None:
    header = (REPO_ROOT / "firmware/mr_stabs_mk2/include/espnow_link.h").read_text()
    period_ms = int(re.search(r"TELEMETRY_PERIOD_MS\s*=\s*(\d+)", header).group(1))
    assert check.TELEMETRY_HZ == 1000.0 / period_ms
