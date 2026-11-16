"""velocity_jig_drive rest and sample-count guards."""

from __future__ import annotations

import math

from auto_battlebot.calibration.jig import excitation as ex
from auto_battlebot.calibration.jig.velocity_jig import ClockProbe
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
