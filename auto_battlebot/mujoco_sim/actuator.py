"""DC gearmotor as a MuJoCo `general` actuator, and the command tape that drives it.

A brushless motor behind a synchronous-rectifying ESC is, averaged over a PWM period, a voltage
source behind a resistance with back-EMF. At the wheel:

    torque = (eta N Kt / R) * V  -  (eta N^2 Kt Ke / R) * qvel

which is affine in the control and the joint speed, so it is exactly MuJoCo's `general` actuator
with `gainprm[0] = eta N Kt / R` and `biasprm[2] = -eta N^2 Kt Ke / R` (Ke = Kt in SI units).
The control is the applied voltage V; `forcerange` carries the current limit.

The command tape turns the logged per-motor percent (after the firmware mixer) into V: delay,
deadzone, a one-term throttle curve, the left/right gain ratio, and the logged pack voltage.
A zero command either shorts the motor (brake: V = 0, the back-EMF term brakes) or lets it
freewheel (coast). Coast has no fixed-parameter form, so the rollout sets V to the back-EMF
voltage N Ke qvel on those steps, which zeroes the motor torque.
"""

from __future__ import annotations

from dataclasses import dataclass, fields

import numpy as np

# ReadyToSky 35 A ESC; the phase-current limit the ESC enforces is not published.
DEFAULT_CURRENT_LIMIT_A = 35.0
NOMINAL_PACK_V = 15.2


@dataclass
class PlantParams:
    """Everything the fit can move. Mass properties are not here: they are fixed from CAD."""

    delay_s: float = 0.03  # firmware command to motion; the radio part is measured separately
    deadzone_left: float = 0.02  # fraction of full command
    deadzone_right: float = 0.02
    curve: float = 0.0  # throttle curvature: g(x) = x (1 + curve (1 - |x|))
    resistance_ohm: float = 0.2
    efficiency: float = 0.8
    lr_gain_ratio: float = 1.0  # left gain over right gain
    joint_frictionloss: float = 2e-3  # N m at the wheel
    joint_damping: float = 1e-4  # N m s / rad at the wheel
    wheel_mu_slide: float = 1.0
    wheel_mu_torsion: float = 5e-3
    wheel_mu_roll: float = 1e-4
    skid_mu: float = 0.3
    solref_timeconst: float = 0.02
    solimp_dmax: float = 0.95
    armature_scale: float = 1.0
    brake_on_zero: bool = True
    current_limit_a: float = DEFAULT_CURRENT_LIMIT_A

    def replace(self, **changes: float | bool) -> PlantParams:
        values = {f.name: getattr(self, f.name) for f in fields(self)}
        values.update(changes)
        return PlantParams(**values)


@dataclass(frozen=True)
class MotorConstants:
    gear_ratio: float
    kt: float  # N m / A at the motor

    def gain(self, params: PlantParams) -> float:
        """gainprm[0]: wheel torque per applied volt."""
        return params.efficiency * self.gear_ratio * self.kt / params.resistance_ohm

    def bias(self, params: PlantParams) -> float:
        """biasprm[2]: wheel torque per wheel rad/s (negative, the back-EMF brake)."""
        return -params.efficiency * self.gear_ratio**2 * self.kt**2 / params.resistance_ohm

    def force_limit(self, params: PlantParams) -> float:
        return params.efficiency * self.gear_ratio * self.kt * params.current_limit_a

    def back_emf_volts(self, qvel: np.ndarray) -> np.ndarray:
        return np.asarray(self.gear_ratio * self.kt * qvel)

    def free_speed(self, volts: float) -> float:
        """Wheel rad/s with no load and no friction."""
        return volts / (self.gear_ratio * self.kt)


def shape_command(
    u: np.ndarray, deadzone: float | np.ndarray, curve: float | np.ndarray
) -> np.ndarray:
    """Per-motor command in [-1, 1] to the fraction of pack voltage applied.

    Commands inside the deadzone map to exactly zero, which the rollout reads as the zero-command
    state (brake or coast). Outside it the command is rescaled to start from zero and bent by
    the one-term curve, which keeps g(0) = 0 and g(1) = 1.
    """
    u = np.clip(np.asarray(u, dtype=float), -1.0, 1.0)
    dz = np.asarray(deadzone, dtype=float)
    mag = np.clip((np.abs(u) - dz) / (1.0 - dz), 0.0, 1.0)
    bent = mag * (1.0 + np.asarray(curve, dtype=float) * (1.0 - mag))
    return np.asarray(np.sign(u) * bent)


def side_gains(lr_gain_ratio: float | np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Split the left/right ratio symmetrically so the mean gain is untouched."""
    root = np.sqrt(np.asarray(lr_gain_ratio, dtype=float))
    return root, 1.0 / root


def delay_tape(tape: np.ndarray, dt: float, delay_s: float) -> np.ndarray:
    """Shift a (..., T) tape later by delay_s, holding the first sample. Linear interpolation."""
    tape = np.asarray(tape, dtype=float)
    steps = delay_s / dt
    t = np.arange(tape.shape[-1], dtype=float) - steps
    lo = np.clip(np.floor(t).astype(int), 0, tape.shape[-1] - 1)
    hi = np.clip(lo + 1, 0, tape.shape[-1] - 1)
    frac = np.clip(t - np.floor(t), 0.0, 1.0)
    frac = np.where(t < 0, 0.0, frac)
    return np.asarray(tape[..., lo] * (1.0 - frac) + tape[..., hi] * frac)
