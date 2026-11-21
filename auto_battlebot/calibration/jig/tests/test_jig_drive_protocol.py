"""Guards in drive_protocol: trainer scaling, mix, readback, and the play loop."""

from __future__ import annotations

import re
import struct
import time
from types import SimpleNamespace
from typing import Any

import pytest

from auto_battlebot.calibration.jig import drive_protocol
from auto_battlebot.calibration.jig.drive_protocol import (
    Armed,
    CommandSample,
    MixConfig,
    PlayResult,
    TrainerLink,
    _ChannelDecoder,
    play,
)
from auto_battlebot.paths import REPO_ROOT

REPO = REPO_ROOT


def _mix(**kwargs: Any) -> MixConfig:
    base: dict[str, Any] = {
        "reverse_angular": False,
        "read_invert_a": False,
        "read_invert_b": False,
    }
    base.update(kwargs)
    return MixConfig(**base)


def test_trainer_max_matches_the_cpp_transmitter() -> None:
    text = (REPO / "src/transmitter/opentx_transmitter.cpp").read_text()
    match = re.search(r"kOpenTxTrainerOutputMax\s*=\s*(\d+)", text)
    assert match is not None
    assert drive_protocol.TRAINER_MAX == int(match.group(1))


def test_tank_and_direct_mixes_send_different_channel_pairs() -> None:
    assert _mix(mode="tank").to_channels(0.5, 0.25) == (375, 125)
    assert _mix(mode="direct").to_channels(0.5, 0.25) == (250, 125)


def test_angular_command_keeps_priority_over_forward_authority() -> None:
    a, b = _mix().to_channels(1.0, 0.24)
    assert (a, b) == (500, 260)
    assert _mix().to_channels(0.9, 1.0) == (500, -500)


def test_deliverable_leaves_the_forward_room_after_the_turn() -> None:
    mix = _mix()
    assert mix.deliverable(1.0, 0.24) == pytest.approx((0.76, 0.24))
    assert mix.deliverable(1.0, -0.24) == pytest.approx((0.76, -0.24))
    assert _mix(reverse_angular=True).deliverable(1.0, 0.24) == pytest.approx((0.76, 0.24))


def _packet(phase: int, values: list[int]) -> bytes:
    payload = struct.pack("<16h", *values)
    chk = phase ^ len(payload + b"\0")
    for b in payload:
        chk ^= b
    return bytes([0xA3, 0xA4, 0xA5, phase, len(payload) + 1]) + payload + bytes([chk])


def test_a_sync_split_across_two_reads_still_decodes() -> None:
    decoder = _ChannelDecoder()
    packet = _packet(0, list(range(16)))
    assert decoder.feed(b"\x00\x00" + packet[:2]) == []
    out = decoder.feed(packet[2:])
    assert out == [(0, list(range(16)))]


def _link(mix: MixConfig, channels: list[int]) -> Any:
    import threading

    link: Any = object.__new__(TrainerLink)
    link.mix = mix
    link._lock = threading.Lock()
    link._packets = 1
    link._channels = channels
    return link


def test_measured_raw_ignores_the_read_inversions() -> None:
    link = _link(_mix(read_invert_b=True), [512, 256] + [0] * 30)
    raw = link.measured_raw()
    assert raw[:2] == [0.5, 0.25]


def _samples(
    pairs: list[tuple[tuple[float, float], tuple[float, float]]], mix: MixConfig
) -> PlayResult:
    result = PlayResult(mix=mix)
    for i, ((lin, ang), (m_lin, m_ang)) in enumerate(pairs):
        result.commands.append(
            CommandSample(
                t_monotonic_s=i * 0.02,
                linear=lin,
                angular=ang,
                trim=0.0,
                channel_a=0,
                channel_b=0,
                meas_linear=m_lin,
                meas_angular=m_ang,
            )
        )
    return result


def test_contamination_is_zero_on_a_hand_driven_run() -> None:
    result = _samples([((0.0, 0.0), (0.9, 0.9))] * 30, _mix())
    result.manual = True
    assert result.contamination == 0.0


def test_contamination_ignores_step_edge_lag() -> None:
    pairs = [((0.0, 0.0), (0.0, 0.0))] * 10
    pairs += [((0.5, 0.0), (0.0, 0.0))] * 2 + [((0.5, 0.0), (0.5, 0.0))] * 20
    assert _samples(pairs, _mix()).contamination == pytest.approx(0.0)


def test_contamination_reports_a_high_percentile_not_one_outlier() -> None:
    pairs = [((0.5, 0.0), (0.5, 0.0))] * 60
    pairs[30] = ((0.5, 0.0), (0.5, 0.9))
    assert _samples(pairs, _mix()).contamination == pytest.approx(0.0)


def test_contamination_compares_against_what_the_mix_could_deliver() -> None:
    pairs = [((1.0, 0.24), (0.76, 0.24))] * 30
    assert _samples(pairs, _mix()).contamination == pytest.approx(0.0)


class _Program:
    duration_s = 0.06
    kind = "step"
    segments: list[Any] = []

    def __init__(self) -> None:
        self.asked: list[float] = []

    def at(self, t: float) -> tuple[float, float]:
        self.asked.append(t)
        return 0.1, 0.0


class _Link:
    def __init__(self) -> None:
        self.mix = _mix()
        self.events: list[tuple[str, float, float]] = []

    def send(self, linear: float, angular: float) -> tuple[int, int]:
        self.events.append(("send", linear, angular))
        return 0, 0

    def disarm(self) -> None:
        self.events.append(("disarm", 0.0, 0.0))

    def measured(self) -> SimpleNamespace:
        return SimpleNamespace(ch_a=0.0, ch_b=0.0, weapon=0.0, arm=0.0)

    def to_body(self, a: float, b: float) -> tuple[float, float]:
        return a, b


def test_pause_disarms_before_the_operator_callback_and_stops_the_program_clock() -> None:
    link = _Link()
    program = _Program()
    seen: list[str] = []
    started = time.monotonic()

    def on_pause(index: int, count: int) -> None:
        seen.append(link.events[-1][0])
        time.sleep(0.2)

    result = play(
        link,  # type: ignore[arg-type]
        program,
        rate_hz=200.0,
        pause_at=[0.02],
        on_pause=on_pause,
    )
    assert seen == ["disarm"]
    assert len(result.pauses) == 1
    assert result.pauses[0].held_s >= 0.2
    assert started <= result.pauses[0].t_start <= time.monotonic()
    assert program.asked[-1] > 0.04
    assert result.completed


def test_play_ends_on_a_zero_command_however_it_stops() -> None:
    link = _Link()
    result = play(link, _Program(), rate_hz=200.0, timeout_s=0.01)  # type: ignore[arg-type]
    assert result.aborted_reason == "hard wall-clock timeout"
    assert link.events[-1] == ("send", 0.0, 0.0)


def test_armed_disarms_on_keyboard_interrupt() -> None:
    link = _Link()
    with pytest.raises(KeyboardInterrupt):
        with Armed(link):  # type: ignore[arg-type]
            raise KeyboardInterrupt
    assert link.events == [("disarm", 0.0, 0.0)]
