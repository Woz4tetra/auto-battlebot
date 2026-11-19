"""The interim MRS BUFF Mk3 fit builds windows that the grey-box predictor replays exactly."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from auto_battlebot.config_profile import load_profile_toml
from auto_battlebot.control import plant
from auto_battlebot.mujoco_sim import greybox
from auto_battlebot.mujoco_sim.actuator import delay_tape
from auto_battlebot.mujoco_sim.fit import window_errors
from playground.calibration import fit_mrs_buff_mk3_interim_plant as interim

REPO_ROOT = Path(__file__).resolve().parents[2]


def test_zero_deadzone_matches_processor() -> None:
    common = load_profile_toml("_common")
    assert interim.ZERO_DEADZONE == common["transmitter"]["zero_deadzone_percent"] / 100.0


def test_synthetic_windows_replay_through_greybox() -> None:
    m4 = plant.PlantParams(delay_s=0.04)
    batch = interim.build_windows(m4, 4, 0)
    qpos, record_dt = greybox.predict(batch, m4)
    err = window_errors(batch, qpos, record_dt)
    assert np.nanmax(np.abs(err.position)) < 2e-3
    assert np.nanmax(np.abs(err.heading)) < 2e-3


@pytest.mark.parametrize("shift_s", [0.005, -0.005])
def test_window_alignment_is_sensitive_to_one_plant_step(shift_s: float) -> None:
    m4 = plant.PlantParams(delay_s=0.04)
    batch = interim.build_windows(m4, 4, 0)
    qpos, record_dt = greybox.predict(batch, m4)
    shifted = np.roll(qpos, 1 if shift_s > 0 else -1, axis=1)
    err = window_errors(batch, shifted, record_dt)
    assert np.nanmax(np.abs(err.position)) > 2e-3


def test_window_starts_at_preroll_time() -> None:
    m4 = plant.PlantParams(delay_s=0.04)
    n_plant = int(round((interim.PREROLL_S + interim.WINDOW_S) / interim.PLANT_DT))
    rng = np.random.default_rng(0)
    lin, ang = zip(*(interim.command_tape(rng, n_plant) for _ in range(2)))
    lin_d = delay_tape(np.array(lin), interim.PLANT_DT, m4.delay_s)
    ang_d = delay_tape(np.array(ang), interim.PLANT_DT, m4.delay_s)
    _, traj = plant.simulate(lin_d, ang_d, interim.PLANT_DT, m4)
    state_times = (np.arange(n_plant) + 1) * interim.PLANT_DT
    k = int(np.argmin(np.abs(state_times - interim.PREROLL_S)))
    batch = interim.build_windows(m4, 2, 0)
    np.testing.assert_allclose(batch.state.x, traj["x"][:, k], atol=1e-9)
