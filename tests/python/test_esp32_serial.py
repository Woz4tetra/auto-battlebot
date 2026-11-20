from __future__ import annotations

from typing import Any

import pytest

from auto_battlebot.robot_link import esp32_serial


class FakeSerial:
    instances: list[FakeSerial] = []

    def __init__(self, **kwargs: Any) -> None:
        self.kwargs = kwargs
        self.events: list[tuple[str, Any]] = []
        FakeSerial.instances.append(self)

    def __setattr__(self, name: str, value: Any) -> None:
        if name in ("port", "baudrate", "dtr", "rts"):
            self.events.append((name, value))
        object.__setattr__(self, name, value)

    def open(self) -> None:
        self.events.append(("open", None))


@pytest.fixture(autouse=True)
def fake_serial(monkeypatch: pytest.MonkeyPatch) -> None:
    FakeSerial.instances.clear()
    monkeypatch.setattr(esp32_serial.serial, "Serial", FakeSerial)


def test_both_lines_are_high_before_the_port_opens() -> None:
    esp32_serial.open_esp32_serial("/dev/ttyACM0", 921600, 0.01)
    events = FakeSerial.instances[0].events
    assert events[-1] == ("open", None)
    before_open = dict(events[:-1])
    assert before_open["dtr"] is True and before_open["rts"] is True
    assert before_open["port"] == "/dev/ttyACM0" and before_open["baudrate"] == 921600


def test_the_port_is_not_opened_by_the_constructor() -> None:
    esp32_serial.open_esp32_serial("/dev/ttyACM0", 115200, 0.05)
    assert "port" not in FakeSerial.instances[0].kwargs


def test_timeout_and_exclusivity_are_passed_through() -> None:
    esp32_serial.open_esp32_serial("/dev/ttyACM0", 115200, 0.05, exclusive=False)
    assert FakeSerial.instances[0].kwargs == {"timeout": 0.05, "exclusive": False}
