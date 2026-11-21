from __future__ import annotations

import time
from types import SimpleNamespace

import pytest

from auto_battlebot.calibration.jig import jig_link
from auto_battlebot.calibration.jig.drive_protocol import play
from auto_battlebot.calibration.jig.jig_link import (
    TRANSMITTER_USB_IDS,
    JigLink,
    PortInfo,
    find_jig_port,
    skew_ppm,
)
from auto_battlebot.calibration.jig.velocity_jig import ClockProbe

STREAM_ROW = b"S 1000,5,1,2,3,4,5,6\n"


class FakeSerial:
    def __init__(self, replies: dict[str, list[bytes]]):
        self.replies = replies
        self.chunks: list[bytes] = []

    def write(self, data: bytes) -> None:
        self.chunks += self.replies.get(data.decode().strip(), [])

    def flush(self) -> None:
        pass

    def close(self) -> None:
        pass

    @property
    def in_waiting(self) -> int:
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, n: int = 1) -> bytes:
        if not self.chunks:
            return b""
        out, self.chunks[0] = self.chunks[0][:n], self.chunks[0][n:]
        if not self.chunks[0]:
            self.chunks.pop(0)
        return out


def make_link(monkeypatch, replies: dict[str, list[bytes]]) -> JigLink:
    fake = FakeSerial(replies)
    monkeypatch.setattr(jig_link.serial, "Serial", lambda *a, **k: fake)
    return JigLink("/dev/fake")


def probe(at_jig_ms: float, offset_ms: float) -> ClockProbe:
    return ClockProbe(
        offset_ms=offset_ms, residual_ms=0.0, at_host_ms=at_jig_ms + offset_ms, at_jig_ms=at_jig_ms
    )


def test_find_port_skips_transmitter(monkeypatch):
    transmitter = PortInfo("/dev/ttyACM0", *next(iter(TRANSMITTER_USB_IDS)), None, None, None)
    other = PortInfo("/dev/ttyACM1", 0x1234, 0x5678, None, None, None)
    monkeypatch.setattr(jig_link, "list_ports", lambda: [transmitter, other])
    assert find_jig_port() == other


def test_get_consumes_trailing_end(monkeypatch):
    link = make_link(
        monkeypatch, {"GET a.csv": [b"SIZE 5\n", b"hello\nEND\n"], "DEL a.csv": [b"OK\n"]}
    )
    assert link.get("a.csv") == b"hello"
    assert link.delete("a.csv") is True


def test_get_keeps_bytes_buffered_after_size_header(monkeypatch):
    link = make_link(monkeypatch, {"GET a.csv": [b"SIZE 5\nhel", b"lo\nEND\n"]})
    assert link.get("a.csv") == b"hello"


@pytest.mark.parametrize(
    "stop_reply",
    [
        [b"END\n", b"recording LOG-7\n"],
        [b"recording LOG-7\n", b"END\n"],
        [STREAM_ROW, STREAM_ROW, b"END\n", b"recording LOG-7\n"],
    ],
    ids=["end_first", "end_last", "rows_in_flight"],
)
def test_stream_end_then_recording(monkeypatch, stop_reply):
    link = make_link(monkeypatch, {"STREAM": [STREAM_ROW], "": stop_reply, "DEL a.csv": [b"OK\n"]})
    assert len(list(link.stream(0.2))) == 1
    assert link.take_recording() == "LOG-7"
    assert link.delete("a.csv") is True


def test_skew_ppm_zero_on_reboot():
    assert skew_ppm(probe(100_000.0, 5.0), probe(10_000.0, 1000.0)) == 0.0


def test_skew_ppm_measures_drift():
    assert skew_ppm(probe(0.0, 0.0), probe(30_000.0, 0.9)) == pytest.approx(30.0)


def test_command_log_time_origin_matches_clock_probes(monkeypatch):
    class Link:
        mix = None

        def send(self, linear, angular):
            return 0, 0

        def measured(self):
            return SimpleNamespace(ch_a=0.0, ch_b=0.0, weapon=0.0, arm=0.0)

        def to_body(self, a, b):
            return 0.0, 0.0

    program = SimpleNamespace(duration_s=0.05, at=lambda t: (0.0, 0.0), kind="test", segments=None)
    jig = make_link(monkeypatch, {"TIME": [b"TIME 1000 1100\n"]})
    before = time.monotonic()
    sample = jig.time_probe()
    played = play(Link(), program, rate_hz=200.0)
    after = time.monotonic()
    assert sample is not None
    assert before <= sample.host_mid_ms / 1e3 <= after
    assert played.commands
    assert all(before <= c.t_monotonic_s <= after for c in played.commands)
