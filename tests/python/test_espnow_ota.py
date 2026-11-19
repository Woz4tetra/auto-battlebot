"""The OTA sender window and the robot's acknowledgement cadence stay compatible."""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TRACKER_H = ROOT / "firmware/shared/espnow_protocol/src/command_tracker.h"
OTA_PY = ROOT / "firmware/mr_stabs_mk2/scripts/espnow_ota.py"


def test_ack_every_stays_below_host_window() -> None:
    ack_every = int(re.search(r"ACK_EVERY = (\d+);", TRACKER_H.read_text()).group(1))  # type: ignore[union-attr]
    window = int(re.search(r"^WINDOW = (\d+)", OTA_PY.read_text(), re.M).group(1))  # type: ignore[union-attr]
    assert 0 < ack_every < window
