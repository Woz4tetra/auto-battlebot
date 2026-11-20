from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from auto_battlebot.compat import tomllib
from auto_battlebot.control.plant import PlantParams, simulate

REPO_ROOT = Path(__file__).resolve().parents[2]

from auto_battlebot.kinematic_sim.config.kinematic import PlantConfig
from auto_battlebot.kinematic_sim.plants.kinematic import KinematicPlant

DT = 0.02
ARENA_M = 100.0
FOOTPRINT_OFFSET_TOLERANCE_M = 1e-3


def _common_footprint_offset(label: str) -> list[float]:
    with open(REPO_ROOT / "config" / "_common.toml", "rb") as handle:
        common = tomllib.load(handle)
    return list(common["robot_filter"]["footprint_center_offset_per_label"][label])


def test_kinematic_plant_trajectory_matches_control_plant() -> None:
    cfg = PlantConfig(
        start_pos=[0.0, 0.0],
        start_yaw_deg=0.0,
        max_linear_speed=0.5,
        max_linear_speed_fwd=0.6,
        max_linear_speed_rev=0.4,
        max_angular_speed=2.0,
        tau_linear_accel=0.05,
        tau_linear_decel=0.09,
        tau_angular_accel=0.06,
        tau_angular_decel=0.08,
        steer_brake_coeff=0.1,
        angular_droop_coeff=0.05,
        deadzone_linear=0.04,
        deadzone_linear_rev=0.06,
        deadzone_angular=0.03,
        deadzone_angular_right=0.05,
    )
    params = PlantParams(
        dz_lin_fwd=0.04,
        dz_lin_rev=0.06,
        dz_ang_l=0.03,
        dz_ang_r=0.05,
        k_fwd=0.6,
        k_rev=0.4,
        k_ang=2.0,
        tau_lin_a=0.05,
        tau_lin_d=0.09,
        tau_ang_a=0.06,
        tau_ang_d=0.08,
        delay_s=0.0,
        c_sb=0.1,
        c_ad=0.05,
    )
    n = 200
    t = np.arange(n) * DT
    cmd_lin = 0.8 * np.sign(np.sin(t * 1.7))
    cmd_ang = 0.7 * np.cos(t * 2.3)
    _, traj = simulate(cmd_lin, cmd_ang, DT, params)

    plant = KinematicPlant(cfg, ARENA_M, ARENA_M)
    xs, ys, yaws = [], [], []
    for lin, ang in zip(cmd_lin, cmd_ang):
        plant.step(float(lin), float(ang), DT)
        xs.append(plant.x)
        ys.append(plant.y)
        yaws.append(plant.yaw)

    assert np.allclose(xs, traj["x"], atol=1e-6), "x"
    assert np.allclose(ys, traj["y"], atol=1e-6), "y"
    assert np.allclose(np.cos(yaws), np.cos(traj["theta"]), atol=1e-6), "yaw"


def test_mujoco_footprint_offset_matches_config() -> None:
    pytest.importorskip("mujoco")
    from auto_battlebot.kinematic_sim.plants.mujoco_plant import MujocoPlant

    plant = MujocoPlant(PlantConfig(), 2.35, 2.35)
    expected = _common_footprint_offset("MR_STABS_MK2")
    for got, want in zip(plant.footprint_center_offset, expected):
        assert math.isclose(got, want, abs_tol=FOOTPRINT_OFFSET_TOLERANCE_M)


def test_kinematic_footprint_is_the_pose() -> None:
    plant = KinematicPlant(PlantConfig(), ARENA_M, ARENA_M)
    assert plant.footprint_center_offset == (0.0, 0.0)
