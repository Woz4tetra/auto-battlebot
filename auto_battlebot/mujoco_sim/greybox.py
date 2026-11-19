"""The grey-box plant (auto_battlebot/control/plant.py) fit on the same windows as MuJoCo."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import fields, replace

import numpy as np
from scipy.optimize import least_squares

from auto_battlebot.control import plant
from auto_battlebot.control.firmware_mixer import motor_to_plant
from auto_battlebot.mujoco_sim.actuator import delay_tape
from auto_battlebot.mujoco_sim.fit import window_errors
from auto_battlebot.mujoco_sim.session import WindowBatch

PLANT_DT = 0.005

FITTED: dict[str, tuple[float, float]] = {
    "k_fwd": (0.5, 8.0),
    "k_rev": (0.5, 8.0),
    "k_ang": (5.0, 120.0),
    "tau_lin_a": (0.01, 0.4),
    "tau_lin_d": (0.01, 0.4),
    "tau_ang_a": (0.01, 0.4),
    "tau_ang_d": (0.01, 0.4),
    "dz_lin_fwd": (0.0, 0.1),
    "dz_lin_rev": (0.0, 0.1),
    "dz_ang_l": (0.0, 0.1),
    "dz_ang_r": (0.0, 0.1),
}
DELAY_GRID_S = tuple(np.arange(0.0, 0.1001, 0.01))


def predict(batch: WindowBatch, p: plant.PlantParams) -> tuple[np.ndarray, float]:
    """Grey-box trajectories as chassis qpos (N, S, 7), flat (no pitch), every PLANT_DT."""
    every = int(round(PLANT_DT / batch.dt))
    shifted = np.moveaxis(delay_tape(np.moveaxis(batch.u_raw, 1, 2), batch.dt, p.delay_s), 2, 1)[
        :, batch.pad :: every
    ]
    u_lin, u_ang = motor_to_plant(100.0 * shifted[..., 0], 100.0 * shifted[..., 1])
    st = batch.state
    forward = st.vx_field * np.cos(st.yaw) + st.vy_field * np.sin(st.yaw)
    state0 = plant.PlantState(x=st.x, y=st.y, theta=st.yaw, v=forward, w=st.yaw_rate)
    _, traj = plant.simulate(u_lin, u_ang, PLANT_DT, p, state=state0)
    x = np.concatenate([st.x[:, None], traj["x"]], axis=1)
    y = np.concatenate([st.y[:, None], traj["y"]], axis=1)
    yaw = np.concatenate([st.yaw[:, None], traj["theta"]], axis=1)
    qpos = np.zeros(x.shape + (7,))
    qpos[..., 0], qpos[..., 1] = x, y
    qpos[..., 3], qpos[..., 6] = np.cos(yaw / 2), np.sin(yaw / 2)
    return qpos, PLANT_DT


def _residuals(batch: WindowBatch, p: plant.PlantParams) -> np.ndarray:
    qpos, record_dt = predict(batch, p)
    err = window_errors(batch, qpos, record_dt)
    pos = err.position_err_m / err.position_sigma_m
    yaw = err.heading_err_rad / err.heading_sigma_rad
    out = np.concatenate([pos.ravel(), yaw.ravel()])
    return np.asarray(np.nan_to_num(out, nan=0.0))


def fit_greybox(
    batch: WindowBatch,
    start: plant.PlantParams | None = None,
    delays: Sequence[float] = DELAY_GRID_S,
) -> tuple[plant.PlantParams, float]:
    """Profile the delay on a grid (as jig_fit does) with a Huber least squares inside."""
    flat = batch.take(np.nonzero(batch.kind == "flat")[0])
    base = start or plant.PlantParams()
    names = [n for n in FITTED if n in {f.name for f in fields(base)}]
    lo = np.array([FITTED[n][0] for n in names])
    hi = np.array([FITTED[n][1] for n in names])
    best, best_cost = base, np.inf
    for delay in delays:
        p0 = replace(base, delay_s=float(delay))
        x0 = np.clip([getattr(p0, n) for n in names], lo, hi)

        def resid(x: np.ndarray, p0: plant.PlantParams = p0) -> np.ndarray:
            return _residuals(flat, replace(p0, **dict(zip(names, map(float, x)))))

        sol = least_squares(resid, x0, bounds=(lo, hi), loss="huber", f_scale=3.0, max_nfev=200)
        if sol.cost < best_cost:
            best_cost = float(sol.cost)
            best = replace(p0, **dict(zip(names, map(float, sol.x))))
    return best, best_cost
