"""Closing the trainer link leaves the robot at zero command and disarmed."""

from __future__ import annotations

import threading

from auto_battlebot.calibration.jig import drive_protocol as dp


class _FakeSerial:
    def __init__(self) -> None:
        self.writes: list[str] = []

    def write(self, data: bytes) -> None:
        self.writes.append(data.decode())

    def flush(self) -> None:
        pass

    def close(self) -> None:
        pass


def _link() -> tuple[dp.TrainerLink, _FakeSerial]:
    link = dp.TrainerLink.__new__(dp.TrainerLink)
    serial = _FakeSerial()
    link._serial = serial
    link._stop = threading.Event()
    link._reader = None
    return link, serial


def test_close_zeroes_both_channels() -> None:
    link, serial = _link()
    link.close()
    assert f"trainer {dp.TRAINER_CHANNEL_0} 0\r\n" in serial.writes
    assert f"trainer {dp.TRAINER_CHANNEL_1} 0\r\n" in serial.writes
