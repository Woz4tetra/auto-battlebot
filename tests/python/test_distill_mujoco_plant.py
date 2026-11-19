from __future__ import annotations

import numpy as np
import pytest

from playground.calibration import distill_mujoco_plant as distill


class _Collision:
    rest_pitch_rad = 0.0


class _EchoSim:
    def __init__(self, *args: object, **kwargs: object) -> None:
        self.last_lin = 0.0
        self.forward_speed = 0.0
        self.yaw_rate = 0.0
        self.pitch = 0.0

    def step(self, lin: float, ang: float, dt: float) -> None:
        self.last_lin = lin

    def pose(self) -> tuple[float, float, float]:
        return self.last_lin, 0.0, 0.0


def test_trace_cmd_lin_is_the_stick_held_over_the_step_ending_at_each_sample(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(distill, "_model", lambda: (None, _Collision()))
    monkeypatch.setattr(distill, "ClosedLoopSim", _EchoSim)
    tape = distill.Tape(
        "ramp",
        distill.TapeKind.STEP,
        np.array([0.1, 0.2, 0.3, 0.4]),
        np.zeros(4),
    )

    trace = distill.simulate_tape(None, tape, 16.0, None, 1.0)  # type: ignore[arg-type]

    assert trace.cmd_lin[0] == 0.0
    np.testing.assert_allclose(trace.cmd_lin, trace.x)
    np.testing.assert_allclose(trace.cmd_lin[1:], tape.lin)


def test_train_tapes_have_unique_names_and_enum_kinds() -> None:
    tapes = distill.train_tapes()
    assert len({t.name for t in tapes}) == len(tapes)
    assert all(isinstance(t.kind, distill.TapeKind) for t in tapes)
