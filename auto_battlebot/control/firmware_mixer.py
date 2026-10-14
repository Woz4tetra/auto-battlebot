"""Batched mirror of the Mr Stabs Mk2 firmware mixer, and the plant closed around it.

`mix_motor_outputs` (firmware/mr_stabs_mk2/src/main.cpp) turns the arbitrated sticks into motor
commands. With the flip switch down it runs `yaw_control::YawController` on every BNO055 sample
and its output replaces the steer stick; otherwise the steer stick passes through a 1% deadband.
`auto_battlebot/mujoco_sim/firmware.py` mirrors both line for line as scalar Python stepping one
robot. This module is the same logic over a batch dimension, so the sysid fit can roll out
hundreds of windows at once, plus `closed_loop_rollout`, which runs `plant.integrate_step`
behind the mixer with the heading and gyro read back from the simulated state.

Conventions follow the firmware: percents in [-100, 100], `a_percent` positive drives tail-first
(stick back), a positive steer output speeds up the left wheel and turns the robot clockwise,
headings in degrees growing clockwise, rates in degrees per second clockwise-positive. The plant
is counter-clockwise-positive in radians; `heading_from_theta` and `gyro_from_w` convert.

Upside-down driving is not mirrored: the sysid gates drop inverted windows before they get here.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from auto_battlebot.control.plant import (
    FULL_MODEL,
    SUBSTEP_S,
    ModelStructure,
    PlantParams,
    PlantState,
    integrate_step,
    steady_state,
)
from auto_battlebot.mujoco_sim.firmware import (
    GYRO_RANGE_DPS,
    SAMPLE_PERIOD_S,
    TURN_THRESHOLD_PERCENT,
    YawConfig,
)


def heading_from_theta(theta: Any) -> np.ndarray:
    """BNO055 heading (clockwise degrees in [0, 360)) from plant theta (counter-clockwise rad)."""
    out: np.ndarray = np.mod(-np.degrees(np.asarray(theta, dtype=float)), 360.0)
    return out


def gyro_from_w(w: Any) -> np.ndarray:
    """What get_yaw_rate() reads: clockwise deg/s, clipped at the gyro range."""
    out: np.ndarray = np.clip(
        -np.degrees(np.asarray(w, dtype=float)), -GYRO_RANGE_DPS, GYRO_RANGE_DPS
    )
    return out


def wrap_degrees(angle: Any) -> np.ndarray:
    """yaw_control::wrap_degrees: [-180, 180)."""
    a = np.asarray(angle, dtype=float)
    out: np.ndarray = a - 360.0 * np.floor((a + 180.0) / 360.0)
    return out


def motor_to_plant(left_cmd: Any, right_cmd: Any) -> tuple[np.ndarray, np.ndarray]:
    """Motor percents to the plant's normalized (linear, angular) command.

    The mixer writes left = -a + steer and right = -a - steer. Forward is positive linear, and a
    counter-clockwise turn (right wheel faster) is positive angular, as in `plant.py`.
    """
    left = np.asarray(left_cmd, dtype=float)
    right = np.asarray(right_cmd, dtype=float)
    return (left + right) / 200.0, (right - left) / 200.0


@dataclass
class LoopState:
    """Per-window YawController state, plus whether the loop is running. All shaped (B,)."""

    active: np.ndarray
    setpoint: np.ndarray
    capturing: np.ndarray
    capture_time: np.ndarray
    integral: np.ndarray
    output: np.ndarray
    rate_command: np.ndarray

    @classmethod
    def idle(cls, batch: int) -> LoopState:
        z = np.zeros(batch)
        return cls(
            active=np.zeros(batch, dtype=bool),
            setpoint=z.copy(),
            capturing=np.zeros(batch, dtype=bool),
            capture_time=z.copy(),
            integral=z.copy(),
            output=z.copy(),
            rate_command=z.copy(),
        )

    @classmethod
    def from_telemetry(
        cls,
        active: Any,
        heading0_deg: Any,
        recorded_heading_deg: Any,
        pid_setpoint: Any,
        pid_output: Any,
        yaw_rate_cmd: Any,
        yaw_rate: Any,
        b_percent: Any,
        a_percent: Any,
        config: YawConfig | None = None,
    ) -> LoopState:
        """Seed the loop from the telemetry sample at a window start.

        The setpoint is carried over as an offset from the recorded heading, so it lands in the
        simulated heading frame whatever the BNO055's absolute heading is. The integral is what
        is left of the recorded output once the feedforward and proportional terms come off;
        the firmware does not report it, but it reports everything else in that sum.
        """
        c = config or YawConfig()
        active = np.asarray(active, dtype=bool)
        heading0 = np.asarray(heading0_deg, dtype=float)
        offset = wrap_degrees(np.asarray(pid_setpoint, dtype=float) - recorded_heading_deg)
        rate_cmd = np.nan_to_num(np.asarray(yaw_rate_cmd, dtype=float))
        turning = np.abs(np.asarray(b_percent, dtype=float)) > c.turn_threshold
        reverse = np.clip(np.asarray(a_percent, dtype=float) / 100.0, 0.0, 1.0)
        kp = c.kp * (1.0 + (c.reverse_kp_scale - 1.0) * reverse)
        ff = c.feedforward * (1.0 + (c.reverse_ff_scale - 1.0) * reverse)
        ff_term = np.where(turning, ff * rate_cmd, 0.0)
        p_term = kp * (rate_cmd - np.asarray(yaw_rate, dtype=float))
        integral = np.clip(
            np.asarray(pid_output, dtype=float) - ff_term - p_term, -c.i_max, c.i_max
        )
        integral = np.where(active, np.nan_to_num(integral), 0.0)
        # A released turn with the rate command at zero is still braking (capturing): a heading
        # hold would be commanding k_heading times a nonzero error instead.
        idle = ~turning & (np.abs(np.asarray(a_percent, dtype=float)) < c.idle_throttle)
        capturing = active & ~turning & ~idle & (rate_cmd == 0.0) & (np.abs(offset) > 0.5)
        return cls(
            active=active,
            setpoint=np.where(active, np.mod(heading0 + offset, 360.0), heading0),
            capturing=capturing,
            capture_time=np.zeros(active.shape),
            integral=integral,
            output=np.where(active, np.nan_to_num(np.asarray(pid_output, dtype=float)), 0.0),
            rate_command=np.where(active, rate_cmd, 0.0),
        )

    def copy(self) -> LoopState:
        return LoopState(*(np.copy(getattr(self, f)) for f in self.__dataclass_fields__))


class BatchedMixer:
    """`mix_motor_outputs` plus `YawController`, stepped once per BNO055 sample.

    The firmware runs the yaw loop only when a fresh IMU sample lands (every 10 ms), and the
    steer output it computes holds until the next one. Stepping the whole mixer at that period
    keeps the loop exact and costs at most one control loop of stick latency in passthrough.
    """

    def __init__(self, state: LoopState, config: YawConfig | None = None) -> None:
        self.c = config or YawConfig()
        self.s = state

    def step(
        self,
        a_percent: np.ndarray,
        b_percent: np.ndarray,
        auto_steer: np.ndarray,
        heading_deg: np.ndarray,
        rate_dps: np.ndarray,
        dt: float = SAMPLE_PERIOD_S,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """One sample. Returns (left, right, steer_output) percents."""
        c, s = self.c, self.s
        a = np.asarray(a_percent, dtype=float)
        b = np.asarray(b_percent, dtype=float)
        auto = np.asarray(auto_steer, dtype=bool)

        # Engaging: hold the heading the robot has now.
        engage = auto & ~s.active
        s.setpoint = np.where(engage, heading_deg, s.setpoint)
        for name in ("capture_time", "integral", "output", "rate_command"):
            setattr(s, name, np.where(engage, 0.0, getattr(s, name)))
        s.capturing = s.capturing & ~engage
        s.active = auto

        turning = np.abs(b) > c.turn_threshold
        idle = ~turning & (np.abs(a) < c.idle_throttle)

        # Released turn: brake at zero rate until slow or timed out, then hold where it stopped.
        releasing = ~turning & s.capturing
        s.capture_time = np.where(releasing, s.capture_time + dt, s.capture_time)
        captured = releasing & (
            (np.abs(rate_dps) < c.capture_rate) | (s.capture_time >= c.capture_timeout)
        )
        s.setpoint = np.where(captured | idle, heading_deg, s.setpoint)
        s.capturing = np.where(turning, True, np.where(captured | idle, False, s.capturing))
        s.capture_time = np.where(turning, 0.0, s.capture_time)

        hold = np.clip(
            c.k_heading * wrap_degrees(s.setpoint - heading_deg), -c.hold_rate_max, c.hold_rate_max
        )
        rate_command = np.where(turning, b / 100.0 * c.max_rate, np.where(s.capturing, 0.0, hold))
        ff_command = np.where(turning, rate_command, 0.0)
        reverse = np.clip(a / 100.0, 0.0, 1.0)
        kp = c.kp * (1.0 + (c.reverse_kp_scale - 1.0) * reverse)
        ff = c.feedforward * (1.0 + (c.reverse_ff_scale - 1.0) * reverse)
        error = rate_command - rate_dps
        unsaturated = ff * ff_command + kp * error + s.integral
        integrate = (np.abs(unsaturated) < 100.0) | ((unsaturated > 0.0) != (error > 0.0))
        integral = np.where(
            integrate, np.clip(s.integral + c.ki * error * dt, -c.i_max, c.i_max), s.integral
        )
        loop_out = np.clip(ff * ff_command + kp * error + integral, -100.0, 100.0)

        run = auto & ~idle
        s.integral = np.where(run, integral, np.where(auto & idle, s.integral, 0.0))
        s.rate_command = np.where(run, rate_command, 0.0)
        s.output = np.where(run, loop_out, 0.0)

        passthrough = np.where(np.abs(b) > TURN_THRESHOLD_PERCENT, b, 0.0)
        steer = np.where(auto, s.output, passthrough)
        left = -a + steer
        right = -a - steer
        peak = np.maximum(np.maximum(np.abs(left), np.abs(right)), 100.0)
        return left / peak * 100.0, right / peak * 100.0, steer


def closed_loop_rollout(
    a_percent: np.ndarray,
    b_percent: np.ndarray,
    auto_steer: np.ndarray,
    p: PlantParams,
    state0: PlantState,
    loop0: LoopState | None = None,
    history: tuple[np.ndarray, np.ndarray] | None = None,
    structure: ModelStructure = FULL_MODEL,
    dt: float = SAMPLE_PERIOD_S,
    substep_s: float = SUBSTEP_S,
) -> dict[str, np.ndarray]:
    """Sticks through the mixer into the plant, closed around the simulated heading and gyro.

    `a_percent`, `b_percent` and `auto_steer` are (B, N) on a `dt` grid starting at the window
    start. `history` is the plant command (linear, angular) on the same grid for the samples
    before the start, (B, M), most recent last, so the transport delay reads real commands for
    its first `delay_s`. Returns per-sample x, y, theta, v, w and the motor commands.
    """
    p = structure.apply(p)
    a = np.asarray(a_percent, dtype=float)
    b = np.asarray(b_percent, dtype=float)
    auto = np.asarray(auto_steer, dtype=bool)
    batch, n = a.shape
    if history is None:
        hist_lin = np.zeros((batch, 0))
        hist_ang = np.zeros((batch, 0))
    else:
        hist_lin, hist_ang = (np.asarray(h, dtype=float) for h in history)
    m = hist_lin.shape[1]
    cmd_lin = np.concatenate([hist_lin, np.zeros((batch, n))], axis=1)
    cmd_ang = np.concatenate([hist_ang, np.zeros((batch, n))], axis=1)

    mixer = BatchedMixer(loop0.copy() if loop0 is not None else LoopState.idle(batch))
    state = state0.copy()
    substeps = max(1, int(round(dt / substep_s)))
    sub_dt = dt / substeps
    shift = p.delay_s / dt if structure.use_delay else 0.0
    out = {k: np.zeros((batch, n)) for k in ("x", "y", "theta", "v", "w", "left", "right")}

    for k in range(n):
        left, right, _steer = mixer.step(
            a[:, k], b[:, k], auto[:, k], heading_from_theta(state.theta), gyro_from_w(state.w), dt
        )
        cmd_lin[:, m + k], cmd_ang[:, m + k] = motor_to_plant(left, right)
        out["left"][:, k] = left
        out["right"][:, k] = right
        # The command in effect `delay_s` before this step, linear between grid samples and held
        # for the step, the same sampling as `plant.delayed_commands` and `plant.simulate`; zero
        # before the history starts.
        pos = m + k - shift
        if pos < 0.0:
            u_lin = np.zeros(batch)
            u_ang = np.zeros(batch)
        else:
            lo = int(np.floor(pos))
            hi = min(lo + 1, m + k)
            frac = pos - lo
            u_lin = cmd_lin[:, lo] * (1.0 - frac) + cmd_lin[:, hi] * frac
            u_ang = cmd_ang[:, lo] * (1.0 - frac) + cmd_ang[:, hi] * frac
        v_target, w_target = steady_state(u_lin, u_ang, p)
        if not structure.use_lag:
            state = PlantState(state.x, state.y, state.theta, v_target, w_target)
        for _ in range(substeps):
            state = integrate_step(state, v_target, w_target, sub_dt, p)
        out["x"][:, k] = state.x
        out["y"][:, k] = state.y
        out["theta"][:, k] = state.theta
        out["v"][:, k] = state.v
        out["w"][:, k] = state.w
    return out
