"""Open-loop plant prediction over a match run, reset from the measured track every few seconds."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from auto_battlebot.calibration.match.match_windows import MatchRun
from auto_battlebot.control.plant import (
    ModelStructure,
    PlantParams,
    PlantState,
    delayed_commands,
    integrate_step,
    steady_state,
)

ROBOT_HALF_M = 0.11
SUBSTEP_S = 0.005


@dataclass
class PredictedTrack:
    pose: np.ndarray
    segment: np.ndarray
    has_prediction: np.ndarray


def predict_track(
    run: MatchRun,
    params: PlantParams,
    structure: ModelStructure,
    reset_s: float,
) -> PredictedTrack:
    applied = structure.apply(params)
    u_lin, u_ang = delayed_commands(run.t, run.cmd_lin, run.cmd_ang, applied.delay_s, run.dt)
    n = len(run.t)
    pose = np.full((n, 3), np.nan)
    segment = np.full(n, -1, dtype=int)
    reset_steps = max(1, int(round(reset_s / run.dt)))
    substeps = max(1, int(round(run.dt / SUBSTEP_S)))
    sub_dt = run.dt / substeps

    state: PlantState | None = None
    seg = -1
    next_reset = 0
    for k in range(n):
        if not run.auto[k]:
            state = None
            next_reset = k + 1
            continue
        if k >= next_reset and np.isfinite(run.x[k]):
            state = PlantState(
                x=float(run.x[k]),
                y=float(run.y[k]),
                theta=float(run.theta[k]),
                v=float(run.v_body_mps[k]) if np.isfinite(run.v_body_mps[k]) else 0.0,
                w=float(run.yaw_rate_radps[k]) if np.isfinite(run.yaw_rate_radps[k]) else 0.0,
            )
            seg += 1
            next_reset = k + reset_steps
        if state is None:
            continue
        pose[k] = (float(state.x), float(state.y), float(state.theta))
        segment[k] = seg
        v_target, w_target = steady_state(np.asarray(u_lin[k]), np.asarray(u_ang[k]), applied)
        for _ in range(substeps):
            state = integrate_step(state, v_target, w_target, sub_dt, applied)
    return PredictedTrack(pose=pose, segment=segment, has_prediction=segment >= 0)
