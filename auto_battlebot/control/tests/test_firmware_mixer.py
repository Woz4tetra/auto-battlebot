from __future__ import annotations

import numpy as np
import pytest

from auto_battlebot.control import plant
from auto_battlebot.control.firmware_mixer import closed_loop_rollout, motor_to_plant
from auto_battlebot.control.plant import ModelStructure, PlantParams, PlantState
from auto_battlebot.mujoco_sim.firmware import SAMPLE_PERIOD_S

NO_LAG_DELAYED = ModelStructure("delayed", use_delay=True, use_lag=False)


def test_motor_to_plant_sign_convention() -> None:
    forward_lin, forward_ang = motor_to_plant(-100.0, -100.0)
    assert forward_lin == pytest.approx(-1.0), "tail-first mixer output is negative linear"
    left_wheel_slower_lin, left_wheel_slower_ang = motor_to_plant(0.0, 100.0)
    assert left_wheel_slower_ang == pytest.approx(0.5), "right wheel faster is counter-clockwise"
    assert left_wheel_slower_lin == pytest.approx(0.5)
    _, clockwise_ang = motor_to_plant(100.0, 0.0)
    assert clockwise_ang == pytest.approx(-0.5)


def test_closed_loop_rejects_mismatched_stick_shapes() -> None:
    sticks = np.zeros((2, 10))
    with pytest.raises(ValueError, match="share one"):
        closed_loop_rollout(
            sticks, np.zeros((2, 9)), sticks.astype(bool), PlantParams(), PlantState(0, 0, 0, 0, 0)
        )


def test_closed_loop_rejects_history_with_wrong_batch() -> None:
    sticks = np.zeros((2, 10))
    with pytest.raises(ValueError, match="history"):
        closed_loop_rollout(
            sticks,
            sticks,
            sticks.astype(bool),
            PlantParams(),
            PlantState(0, 0, 0, 0, 0),
            history=(np.zeros((3, 4)), np.zeros((3, 4))),
        )


@pytest.mark.parametrize("delay_s", [0.0, 0.025, 0.037])
def test_closed_loop_delay_matches_plant_delayed_commands(delay_s: float) -> None:
    rng = np.random.default_rng(5)
    n = 60
    a = np.repeat(rng.uniform(-60.0, 0.0, (1, n // 6)), 6, axis=1)
    b = np.repeat(rng.uniform(-40.0, 40.0, (1, n // 6)), 6, axis=1)
    auto = np.zeros((1, n), dtype=bool)
    params = PlantParams(delay_s=delay_s)
    out = closed_loop_rollout(
        a, b, auto, params, PlantState(0, 0, 0, 0, 0), structure=NO_LAG_DELAYED
    )
    cmd_lin, cmd_ang = motor_to_plant(out["left"][0], out["right"][0])
    t = np.arange(n) * SAMPLE_PERIOD_S
    delayed_lin, delayed_ang = plant.delayed_commands(t, cmd_lin, cmd_ang, delay_s, SAMPLE_PERIOD_S)
    _, traj = plant.simulate(
        delayed_lin,
        delayed_ang,
        SAMPLE_PERIOD_S,
        params,
        structure=ModelStructure("s", use_lag=False, use_delay=False),
    )
    for key in ("v", "w", "x", "y", "theta"):
        np.testing.assert_allclose(out[key][0], traj[key], atol=1e-9, err_msg=key)
