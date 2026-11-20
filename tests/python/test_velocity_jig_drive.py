"""velocity_jig_drive rest and sample-count guards."""

from __future__ import annotations

import math
from pathlib import Path

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
