"""Mr Stabs Mk2's pitch response to a forward stick step, and an armature refit."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from auto_battlebot.mujoco_sim.actuator import PlantParams
from auto_battlebot.mujoco_sim.closed_loop import ClosedLoopSim
from auto_battlebot.mujoco_sim.mass_properties import MassProperties
from auto_battlebot.mujoco_sim.mjcf import CollisionSet

STICK_TICK_S = 0.01
FLIP_LIFT_DEG = 90.0


@dataclass(frozen=True)
class StickTape:
    """Body-axis stick fractions on a uniform STICK_TICK_S grid from the step's start."""

    linear_frac_fwd: np.ndarray
    angular_frac_ccw: np.ndarray

    @classmethod
    def resample(
        cls, t_s: np.ndarray, linear: np.ndarray, angular: np.ndarray, duration_s: float
    ) -> StickTape:
        """Zero-order hold of samples at `t_s` (seconds from the step's start) onto the grid."""
        grid = np.arange(0.0, duration_s, STICK_TICK_S)
        idx = np.clip(np.searchsorted(t_s, grid, side="right") - 1, 0, len(t_s) - 1)
        return cls(np.asarray(linear)[idx], np.asarray(angular)[idx])


def ideal_step(size: float, slew_per_s: float = 0.0, duration_s: float = 2.0) -> StickTape:
    """A straight forward step to `size` at t = 0, ramped at `slew_per_s` when that is above 0."""
    t = np.arange(0.0, duration_s, STICK_TICK_S)
    linear = np.full(len(t), size)
    if slew_per_s > 0.0:
        linear = np.minimum(linear, slew_per_s * (t + STICK_TICK_S))
    return StickTape(linear, np.zeros(len(t)))


def pitch_response(
    mp: MassProperties, collision: CollisionSet, params: PlantParams, tape: StickTape
) -> tuple[float, np.ndarray]:
    """(rest pitch, pitch after each tick), radians, from rest on open floor."""
    sim = ClosedLoopSim(mp, collision, params, (0.0, 0.0, 0.0))
    rest = sim.pitch
    out = np.empty(len(tape.linear_frac_fwd))
    for i, (lin, ang) in enumerate(zip(tape.linear_frac_fwd, tape.angular_frac_ccw)):
        sim.step(float(lin), float(ang), STICK_TICK_S)
        out[i] = sim.pitch
    return rest, out


def nose_lift_deg(rest_rad: float, pitch_rad: np.ndarray) -> float:
    return float(np.clip(math.degrees(rest_rad - float(np.min(pitch_rad))), 0.0, 180.0))


def sim_lift_deg(
    mp: MassProperties, collision: CollisionSet, params: PlantParams, tape: StickTape
) -> float:
    rest, pitch = pitch_response(mp, collision, params, tape)
    return nose_lift_deg(rest, pitch)


@dataclass(frozen=True)
class ArmatureFit:
    armature_scale: float
    rms_error_deg: float
    grid: list[float]
    rms_by_scale: list[float]
    sim_lift_by_scale: list[list[float]]


def fit_armature(
    mp: MassProperties,
    collision: CollisionSet,
    params: PlantParams,
    tapes: Sequence[StickTape],
    measured_lift_deg: Sequence[float],
    grid: Sequence[float],
) -> ArmatureFit:
    """The grid armature scale minimizing RMS peak-lift error over the steps."""
    measured = np.minimum(np.asarray(measured_lift_deg, dtype=float), FLIP_LIFT_DEG)
    lifts = [
        [sim_lift_deg(mp, collision, params.replace(armature_scale=float(s)), t) for t in tapes]
        for s in grid
    ]
    rms = [
        float(np.sqrt(np.mean((np.minimum(np.asarray(row), FLIP_LIFT_DEG) - measured) ** 2)))
        for row in lifts
    ]
    best = int(np.argmin(rms))
    return ArmatureFit(float(grid[best]), rms[best], [float(s) for s in grid], rms, lifts)


def slew_sweep(
    mp: MassProperties,
    collision: CollisionSet,
    params: PlantParams,
    size: float,
    rates_per_s: Sequence[float],
) -> list[tuple[float, float]]:
    """(slew rate, nose lift in degrees) for an ideal step to `size` at each rate; 0 is no limit."""
    return [
        (float(r), sim_lift_deg(mp, collision, params, ideal_step(size, r))) for r in rates_per_s
    ]


def loosest_slew(sweep: Sequence[tuple[float, float]], limit_deg: float) -> float | None:
    """The highest rate whose lift stays under `limit_deg`, 0 meaning no limit, None if none do."""
    ordered = sorted(sweep, key=lambda rl: math.inf if rl[0] <= 0.0 else rl[0])
    best: float | None = None
    for rate, lift in ordered:
        if lift >= limit_deg:
            break
        best = rate
    return best
