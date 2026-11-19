"""The OTA sender window and the robot's acknowledgement cadence stay compatible."""

from __future__ import annotations

import importlib.util
import re
import sys
from pathlib import Path
from types import ModuleType

import pytest

ROOT = Path(__file__).resolve().parents[2]
TRACKER_H = ROOT / "firmware/shared/espnow_protocol/src/command_tracker.h"
PROTOCOL_H = ROOT / "firmware/shared/espnow_protocol/src/espnow_protocol.h"
OTA_PY = ROOT / "firmware/mr_stabs_mk2/scripts/espnow_ota.py"
BLOCK_ERASE_S = 0.150


class RecordingPort:
    def __init__(self) -> None:
        self.writes: list[bytes] = []

    def write(self, data: bytes) -> None:
        self.writes.append(data)


@pytest.fixture(scope="module")
def ota() -> ModuleType:
    spec = importlib.util.spec_from_file_location("espnow_ota_under_test", OTA_PY)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def status(ota: ModuleType, next_offset: int) -> object:
    return ota.OtaStatus(ota.OTA_RECEIVING, next_offset, 0, b"\x00" * 16)


def test_ack_every_stays_below_host_window() -> None:
    ack_every = int(re.search(r"ACK_EVERY = (\d+);", TRACKER_H.read_text()).group(1))  # type: ignore[union-attr]
    window = int(re.search(r"^WINDOW_CHUNKS = (\d+)", OTA_PY.read_text(), re.M).group(1))  # type: ignore[union-attr]
    assert 0 < ack_every < window


def test_chunk_size_matches_protocol_header(ota: ModuleType) -> None:
    match = re.search(r"OTA_CHUNK_DATA = (\d+);", PROTOCOL_H.read_text())
    assert match is not None
    assert ota.CHUNK == int(match.group(1))


def test_progress_timeout_exceeds_block_erase(ota: ModuleType) -> None:
    assert ota.PROGRESS_TIMEOUT_S > BLOCK_ERASE_S


def test_one_goback_per_dropped_chunk(ota: ModuleType) -> None:
    transfer = ota.Transfer(RecordingPort(), None, b"\x01" * (ota.CHUNK * ota.WINDOW_CHUNKS))
    transfer.fill_window()
    in_flight_end = transfer.send_offset
    assert in_flight_end == ota.CHUNK * ota.WINDOW_CHUNKS
    transfer.apply(status(ota, ota.CHUNK))
    assert transfer.send_offset == ota.CHUNK
    transfer.send_offset = in_flight_end
    transfer.apply(status(ota, ota.CHUNK))
    assert transfer.send_offset == in_flight_end
