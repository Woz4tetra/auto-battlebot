from __future__ import annotations

import numpy as np
import pytest

from auto_battlebot.calibration.jig import excitation as ex
from auto_battlebot.calibration.jig.velocity_jig import PauseWindow, ProtocolSegment


def _decl(kind: str, channel: str = "linear", **params: object) -> ex.WaveformDecl:
    return ex.WaveformDecl(name=f"t_{kind}", kind=kind, channel=channel, role="r", params=params)


def test_program_zero_after_end() -> None:
    builder = ex._Builder()
    builder.hold(1.0, 0.5, 0.0, "hold")
    builder.hold(1.0, 0.8, 0.0, "last")
    program = ex._finish(_decl("step"), builder)
    assert program.at(0.5) == (0.5, 0.0)
    assert program.at(1.5) == (0.8, 0.0)
    assert program.at(program.duration_s) == (0.0, 0.0)
    assert program.at(program.duration_s + 5.0) == (0.0, 0.0)
    assert program.at(-1.0) == (0.0, 0.0)


def test_cap_amplitudes_preserves_ratios() -> None:
    scaled = ex.cap_amplitudes([0.25, 0.5, 0.75, 1.0], 0.5)
    assert max(scaled) == pytest.approx(0.5)
    assert scaled == pytest.approx([0.125, 0.25, 0.375, 0.5])
    assert ex.cap_amplitudes([0.25, 1.0], None) == [0.25, 1.0]


WEB_TOOL_PRBS_SEED_ACE1 = [-1, 1, -1, -1, -1, 1, -1, -1, 1, 1, 1, -1, -1, -1, 1, -1, 1, 1, 1, -1]


def _prbs(channel: str, **params: object) -> ex.Program:
    base = {"amplitude": 1.0, "bit_ms": 60.0, "duration_s": 30.0, "seed": 0xACE1}
    return ex.build(_decl("prbs", channel, **{**base, **params}))


def _sample_bits(program: ex.Program, axis: int, n: int) -> np.ndarray:
    return np.array([program.at((i + 0.5) * 0.060)[axis] for i in range(n)])


def test_prbs_matches_web_tool_sequence() -> None:
    bits = _sample_bits(_prbs("linear"), 0, len(WEB_TOOL_PRBS_SEED_ACE1))
    assert bits.tolist() == WEB_TOOL_PRBS_SEED_ACE1


def test_combined_prbs_decorrelated() -> None:
    program = _prbs("combined")
    linear = _sample_bits(program, 0, 400)
    angular = _sample_bits(program, 1, 400)
    assert not np.array_equal(linear, angular)
    assert abs(float(np.corrcoef(linear, angular)[0, 1])) < 0.2


def _cell_program() -> ex.Program:
    return ex.build(_decl("step", amplitudes=[0.5, 1.0], shuttle=True, hold_s=1.0, coast_s=1.5))


def test_pause_points_only_between_cells() -> None:
    program = _cell_program()
    assert program.segments is not None
    points = ex.pause_points(program)
    assert points
    for t in points:
        before = [s for s in program.segments if s.t1 == t]
        after = [s for s in program.segments if s.t0 == t]
        assert before[0].linear == 0.0 and before[0].angular == 0.0
        assert abs(after[0].linear) > 0.0 or abs(after[0].angular) > 0.0
    coasts_after_holds = [s.t1 for s in program.segments if s.label == "coast"]
    assert program.segments[-1].t1 not in points
    assert len(points) == len(coasts_after_holds) - 1


def test_continuous_waveforms_have_no_pause_points() -> None:
    assert ex.pause_points(_prbs("linear")) == []


def test_segment_times_shifted_by_pauses() -> None:
    segments = [
        ProtocolSegment(0.0, 1.0, 0.5, 0.0, "a"),
        ProtocolSegment(1.0, 2.5, 0.0, 0.0, "coast"),
        ProtocolSegment(2.5, 3.5, 1.0, 0.0, "b"),
        ProtocolSegment(3.5, 5.0, 0.0, 0.0, "coast"),
        ProtocolSegment(5.0, 6.0, 0.7, 0.0, "c"),
    ]
    pauses = [
        PauseWindow(t_start=100.0, t_end=104.0, program_t=2.5),
        PauseWindow(t_start=110.0, t_end=111.5, program_t=5.0),
    ]
    shifted = ex.shift_segments(segments, pauses)
    assert [(s.label, s.t0, s.t1) for s in shifted] == [
        ("a", 0.0, 1.0),
        ("coast", 1.0, 2.5),
        ("b", 6.5, 7.5),
        ("coast", 7.5, 9.0),
        ("c", 10.5, 11.5),
    ]
