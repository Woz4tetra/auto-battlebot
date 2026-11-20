"""velocity_jig_drive rest and sample-count guards."""

from __future__ import annotations

import argparse
import math
import os
import select
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

import pytest

from auto_battlebot.calibration.jig import drive_protocol as dp
from auto_battlebot.calibration.jig import excitation as ex
from auto_battlebot.calibration.jig.velocity_jig import ClockProbe, read_command_log
from playground.calibration import velocity_jig_drive as drive


def _program(duration_s: float, at) -> ex.Program:
    return ex.Program(
        name="p", kind="sine", channel="linear", role="r", duration_s=duration_s, at=at
    )


def test_sine_ending_on_zero_crossing_is_not_at_rest() -> None:
    program = _program(4.0, lambda t: (math.sin(2 * math.pi * 0.5 * t), 0.0))
    assert not drive.ends_at_rest(program)


def test_zero_tail_is_at_rest() -> None:
    program = _program(4.0, lambda t: (0.5 if t < 2.0 else 0.0, 0.0))
    assert drive.ends_at_rest(program)


def test_nonzero_last_cell_is_not_at_rest() -> None:
    program = _program(4.0, lambda t: (0.0, 0.0) if t < 3.9 else (0.3, 0.0))
    assert not drive.ends_at_rest(program)


def _probe() -> ClockProbe:
    return ClockProbe(offset_ms=0.0, residual_ms=0.1, at_host_ms=0.0, at_jig_ms=1000.0)


def test_one_khz_log_passes_sample_gate() -> None:
    out = drive.gates(0, _probe(), _probe(), 0.0, samples=6000, hold_s=5.0, contamination=0.0)
    assert out == []


def test_short_log_fails_sample_gate() -> None:
    out = drive.gates(0, _probe(), _probe(), 0.0, samples=3000, hold_s=5.0, contamination=0.0)
    assert any("samples" in g.reason for g in out)


class _FakeLink:
    def __init__(self, payload: bytes) -> None:
        self.payload = payload

    def download(self, name: str, dest: Path) -> int:
        dest.write_bytes(self.payload)
        return len(self.payload)


def test_download_log_keeps_the_jig_name_when_the_session_directory_has_no_such_log(
    tmp_path: Path,
) -> None:
    dest, written = drive.download_log(_FakeLink(b"new"), tmp_path, "LOG-007.csv")  # type: ignore[arg-type]

    assert dest == tmp_path / "LOG-007.csv"
    assert dest.read_bytes() == b"new"
    assert written == 3


def test_download_log_renames_instead_of_overwriting_a_log_left_by_an_earlier_card(
    tmp_path: Path,
) -> None:
    (tmp_path / "LOG-007.csv").write_bytes(b"earlier card")

    dest, written = drive.download_log(_FakeLink(b"new card"), tmp_path, "LOG-007.csv")  # type: ignore[arg-type]

    assert dest != tmp_path / "LOG-007.csv"
    assert dest.name.startswith("LOG-007-") and dest.suffix == ".csv"
    assert (tmp_path / "LOG-007.csv").read_bytes() == b"earlier card"
    assert dest.read_bytes() == b"new card"


def test_command_log_t_host_s_is_the_monotonic_stamp_the_probes_use(tmp_path: Path) -> None:
    stamps = [123456.789012, 123456.794012, 123456.799012]
    samples = [dp.CommandSample(t, 0.1, 0.0, 0.0, 0, 0) for t in stamps]
    path = tmp_path / "LOG-1.cmd.csv"
    drive.write_command_csv(path, samples, "meta")
    data = read_command_log(path)
    assert data["t_host_s"].tolist() == pytest.approx(stamps, abs=1e-6)


def _stdin_pty(monkeypatch: pytest.MonkeyPatch):
    master, slave = os.openpty()
    stream = os.fdopen(slave, "r")
    monkeypatch.setattr(sys, "stdin", stream)
    return master, slave, stream


def _pending(fd: int) -> bool:
    return bool(select.select([fd], [], [], 0.05)[0])


def _tap_later(master: int, delay_s: float) -> threading.Thread:
    def tap() -> None:
        time.sleep(delay_s)
        os.write(master, b"\n")

    thread = threading.Thread(target=tap)
    thread.start()
    return thread


def test_drain_stdin_default_window_swallows_a_second_enter_200_ms_late(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    master, slave, stream = _stdin_pty(monkeypatch)
    try:
        thread = _tap_later(master, 0.2)
        drive.drain_stdin()
        thread.join()
        assert not _pending(slave)
    finally:
        stream.close()
        os.close(master)


def test_drain_stdin_without_a_window_leaves_the_late_enter_for_the_prompt(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    master, slave, stream = _stdin_pty(monkeypatch)
    try:
        thread = _tap_later(master, 0.2)
        drive.drain_stdin(0.0)
        thread.join()
        assert _pending(slave)
    finally:
        stream.close()
        os.close(master)


def test_drain_stdin_window_covers_the_double_tap_gap() -> None:
    assert drive.SETTLE_S >= 0.2


def test_prompt_drains_before_it_reads(monkeypatch: pytest.MonkeyPatch) -> None:
    events: list[str] = []
    monkeypatch.setattr(drive, "drain_stdin", lambda: events.append("drain"))
    monkeypatch.setattr("builtins.input", lambda _="": events.append("input") or "")
    drive.prompt("go")
    assert events == ["drain", "input"]


class _StopLink:
    def __init__(self, outcomes: list) -> None:
        self.outcomes = list(outcomes)
        self.calls = 0

    def wait_stopped(self, timeout: float) -> tuple[int, int]:
        self.calls += 1
        outcome = self.outcomes.pop(0)
        if isinstance(outcome, Exception):
            raise outcome
        return outcome


def test_confirm_stopped_prompts_before_it_waits_on_the_jig(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    link = _StopLink([(6000, 0)])
    prompts: list[int] = []
    monkeypatch.setattr(drive, "prompt", lambda msg: prompts.append(link.calls))
    assert drive.confirm_stopped(link) == (6000, 0)  # type: ignore[arg-type]
    assert prompts == [0]


def test_confirm_stopped_gives_up_the_counts_instead_of_hanging(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    link = _StopLink([TimeoutError(), TimeoutError()])
    prompts: list[str] = []
    monkeypatch.setattr(drive, "prompt", prompts.append)
    assert drive.confirm_stopped(link) == (0, 0)  # type: ignore[arg-type]
    assert link.calls == 2
    assert len(prompts) == 2


def test_confirm_stopped_takes_a_late_summary_after_the_second_press(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    link = _StopLink([TimeoutError(), (5000, 2)])
    monkeypatch.setattr(drive, "prompt", lambda msg: None)
    assert drive.confirm_stopped(link) == (5000, 2)  # type: ignore[arg-type]


class _ReopenLink:
    port = "/dev/ttyACM0"
    serial_number = "JIG123"
    closed = False

    def close(self) -> None:
        self.closed = True


def _port(device: str) -> SimpleNamespace:
    return SimpleNamespace(device=device, serial_number="JIG123")


def test_reopen_after_replug_follows_the_serial_number_to_the_new_device_path(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    found = iter([None, None, _port("/dev/ttyACM1")])
    opened: list[str] = []
    monkeypatch.setattr(drive.jl, "find_port_by_serial", lambda serial: next(found))
    monkeypatch.setattr(
        drive.jl,
        "JigLink",
        lambda device, serial_number=None: opened.append(device) or SimpleNamespace(port=device),
    )
    monkeypatch.setattr(drive.time, "sleep", lambda s: None)
    old = _ReopenLink()
    new = drive.reopen_after_replug(old, timeout=60.0)  # type: ignore[arg-type]
    assert old.closed
    assert opened == ["/dev/ttyACM1"]
    assert new.port == "/dev/ttyACM1"


def test_reopen_after_replug_waits_for_the_cdc_endpoint_before_opening(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    events: list[str] = []
    monkeypatch.setattr(drive.jl, "find_port_by_serial", lambda serial: _port("/dev/ttyACM0"))
    monkeypatch.setattr(
        drive.jl,
        "JigLink",
        lambda device, serial_number=None: events.append("open") or SimpleNamespace(port=device),
    )
    monkeypatch.setattr(drive.time, "sleep", lambda s: events.append(f"sleep {s}"))
    drive.reopen_after_replug(_ReopenLink(), timeout=60.0)  # type: ignore[arg-type]
    assert events == [f"sleep {drive.CDC_ENDPOINT_SETTLE_S}", "open"]


def test_reopen_after_replug_retries_an_open_that_fails_once(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    attempts: list[str] = []

    def open_link(device: str, serial_number: str | None = None) -> SimpleNamespace:
        attempts.append(device)
        if len(attempts) == 1:
            raise OSError("busy")
        return SimpleNamespace(port=device)

    monkeypatch.setattr(drive.jl, "find_port_by_serial", lambda serial: _port("/dev/ttyACM0"))
    monkeypatch.setattr(drive.jl, "JigLink", open_link)
    monkeypatch.setattr(drive.time, "sleep", lambda s: None)
    drive.reopen_after_replug(_ReopenLink(), timeout=60.0)  # type: ignore[arg-type]
    assert len(attempts) == 2


def test_reopen_after_replug_times_out_when_the_jig_never_returns(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(drive.jl, "find_port_by_serial", lambda serial: None)
    monkeypatch.setattr(drive.time, "sleep", lambda s: None)
    with pytest.raises(TimeoutError):
        drive.reopen_after_replug(_ReopenLink(), timeout=0.0)  # type: ignore[arg-type]


def test_stop_recording_with_the_cable_out_presses_b_first_and_skips_the_stop_summary(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    events: list[str] = []
    replugged = SimpleNamespace(port="/dev/ttyACM1")
    monkeypatch.setattr(drive, "prompt", lambda msg: events.append("prompt"))
    monkeypatch.setattr(
        drive, "reopen_after_replug", lambda link: events.append("replug") or replugged
    )
    monkeypatch.setattr(drive, "confirm_stopped", lambda link: events.append("summary") or (1, 1))
    out, samples, dropped = drive.stop_recording(object(), True, lambda msg: None)  # type: ignore[arg-type]
    assert events == ["prompt", "replug"]
    assert (out, samples, dropped) == (replugged, 0, 0)


def test_stop_recording_with_the_cable_in_reads_the_stop_summary(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    link = object()
    monkeypatch.setattr(drive, "confirm_stopped", lambda link: (6000, 3))
    monkeypatch.setattr(drive, "reopen_after_replug", lambda link: pytest.fail("replugged"))
    out, samples, dropped = drive.stop_recording(link, False, lambda msg: None)  # type: ignore[arg-type]
    assert (out, samples, dropped) == (link, 6000, 3)


def _args(**overrides: object) -> argparse.Namespace:
    args = drive.build_parser().parse_args([])
    for key, value in overrides.items():
        setattr(args, key, value)
    return args


def test_mix_defaults_read_channel_b_negated() -> None:
    mix = drive.mix_from_args(_args())
    assert mix.read_invert_b is True
    assert mix.read_invert_a is False


def test_no_read_invert_b_flag_reads_channel_b_straight() -> None:
    assert drive.mix_from_args(_args(no_read_invert_b=True)).read_invert_b is False


def test_run_table_log_file_is_the_renamed_destination_not_the_jig_name(tmp_path: Path) -> None:
    renamed = tmp_path / "LOG-007-101500.csv"
    table = drive.run_table(
        dest=renamed,
        command_file=None,
        program=_program(4.0, lambda t: (0.0, 0.0)),
        rep=1,
        pauses=[],
        verdict="pass",
        gate_note="",
        warnings=["w"],
        contamination=0.01234,
        notes="",
        encoder="attached",
        trim=0.0,
        now=datetime(2026, 11, 20, 10, 15, tzinfo=timezone.utc),
    )
    assert table["log_file"] == "LOG-007-101500.csv"
    assert table["id"] == "2026-11-20T10:15:00Z-p-1"
    assert table["contamination"] == 0.0123


def test_planned_steps_counts_the_unplug_and_coast_steps() -> None:
    rest = _program(4.0, lambda t: (0.5 if t < 2.0 else 0.0, 0.0))
    sine = _program(4.0, lambda t: (math.sin(2 * math.pi * 0.5 * t), 0.0))
    moving = ex.WaveformDecl("p", "sine", "linear", "r", {})
    still = ex.WaveformDecl("p", "manual", "linear", "r", {}, motion=False)
    base = drive.STEPS_AT_REST + drive.STEPS_UNPLUG_B_PRESS_REPLUG
    assert drive.planned_steps(moving, rest) == base
    assert drive.planned_steps(moving, sine) == base + 1
    assert drive.planned_steps(still, sine) == drive.STEPS_AT_REST
