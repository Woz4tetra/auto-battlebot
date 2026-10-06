"""Mr Stabs Mk2 firmware layer between the radio and the ESCs, for closed-loop simulation.

Mirrors `mix_motor_outputs`, `get_filtered_angular_z` (firmware/mr_stabs_mk2/src/main.cpp) and
`pid::Pid` (firmware/mr_stabs_mk2/lib/pid/) line for line. The fit never uses this: it replays the
logged per-motor commands, which already come out of the mixer.

Two firmware generations are mirrored, because validate.py replays recordings made before the
2026-10-05 PID rework:

- `legacy=True`: the old firmware. Heading hold ran every control loop with `PidV1`, whose
  integral accumulates the raw error and multiplies by dt only on output, and whose tolerance
  check returns before the derivative term updates its previous error.
- `legacy=False` (default): the current firmware. Heading hold runs once per BNO055 sample
  (100 Hz) with `Pid`, a dt-correct integral and no tolerance-band kick, and its P and D terms
  scale up with reverse throttle (`REVERSE_GAIN_SCALE`) because driving tail-first is
  directionally unstable.

The BNO055 heading (`orientation_x`) is degrees in [0, 360) and grows clockwise seen from above;
`heading_from_yaw` converts the sim's counterclockwise yaw into it. That sign is an assumption
until a recording pins it (plan validation check 5).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

TURNING_COOLDOWN_TIME = 0.25
ANGULAR_SCALE = 1.0
TURN_THRESHOLD_PERCENT = 1.0
SAMPLE_PERIOD_S = 0.01  # updown_sensor::SAMPLE_INTERVAL
REVERSE_GAIN_SCALE = 12.5


def heading_from_yaw(yaw_rad: float) -> float:
    return (-math.degrees(yaw_rad)) % 360.0


def _wrap(error: float, min_input: float, max_input: float) -> float:
    span = max_input - min_input
    half = span / 2.0
    shifted = error + half
    return shifted - math.floor(shifted / span) * span - half


@dataclass
class PidV1:
    """pid::Pid before 2026-10-05, quirks included."""

    kp: float = 0.08
    ki: float = 0.01
    kd: float = 0.01
    kf: float = 0.0
    i_zone: float = -1.0
    i_max: float = 1000.0
    tolerance: float = 2.0
    continuous: bool = True
    min_input: float = -180.0
    max_input: float = 180.0
    i_accum: float = 0.0
    prev_error: float = 0.0
    has_prev_error: bool = False

    def reset(self) -> None:
        self.i_accum = 0.0
        self.prev_error = 0.0
        self.has_prev_error = False

    def _wrap(self, error: float) -> float:
        return _wrap(error, self.min_input, self.max_input)

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        error = setpoint - measurement
        if self.continuous:
            error = self._wrap(error)
        if abs(error) < self.tolerance or dt <= 0.0:
            return 0.0
        out = self.kp * error
        if self.ki != 0.0:
            if self.i_zone < 0.0 or abs(error) < self.i_zone:
                self.i_accum += error
            if self.i_max != 0.0:
                limit = self.i_max / self.ki
                self.i_accum = (
                    min(self.i_accum, limit) if self.i_accum > 0 else max(self.i_accum, -limit)
                )
            out += self.ki * self.i_accum * dt
        if self.kd != 0.0:
            if not self.has_prev_error:
                self.prev_error = error
                self.has_prev_error = True
            else:
                d_err = error - self.prev_error
                if self.continuous:
                    d_err = self._wrap(d_err)
                out += self.kd * d_err / dt
                self.prev_error = error
        out += self.kf * setpoint
        return out


@dataclass
class Pid:
    """pid::Pid as of 2026-10-05, with the gains main.cpp configures."""

    kp: float = 0.08
    ki: float = 0.01
    kd: float = 0.002
    kf: float = 0.0
    i_zone: float = -1.0
    i_max: float = 20.0
    tolerance: float = 2.0
    continuous: bool = True
    min_input: float = -180.0
    max_input: float = 180.0
    i_accum: float = 0.0
    prev_error: float = 0.0
    has_prev_error: bool = False

    def reset(self) -> None:
        self.i_accum = 0.0
        self.prev_error = 0.0
        self.has_prev_error = False

    def _wrap(self, error: float) -> float:
        return _wrap(error, self.min_input, self.max_input)

    def update(
        self, setpoint: float, measurement: float, dt: float, pd_scale: float = 1.0
    ) -> float:
        error = setpoint - measurement
        if self.continuous:
            error = self._wrap(error)
        if dt <= 0.0:
            return 0.0
        p = self.kp * error
        i = 0.0
        if self.ki != 0.0:
            if self.i_zone < 0.0 or abs(error) < self.i_zone:
                self.i_accum += error * dt
            if self.i_max != 0.0:
                limit = abs(self.i_max / self.ki)
                self.i_accum = max(min(self.i_accum, limit), -limit)
            i = self.ki * self.i_accum
        d = 0.0
        if self.kd != 0.0:
            if not self.has_prev_error:
                self.prev_error = error
                self.has_prev_error = True
            else:
                d_err = error - self.prev_error
                if self.continuous:
                    d_err = self._wrap(d_err)
                d = self.kd * d_err / dt
                self.prev_error = error
        out = pd_scale * (p + d) + i + self.kf * setpoint
        if abs(error) < self.tolerance:
            return 0.0
        return out


@dataclass
class FirmwareMixer:
    """Stick percents and the sensed heading in, per-motor percents out, like the robot.

    `step` is one control loop. In the current firmware the heading only changes when a BNO055
    sample lands, every `sample_period`, and heading hold runs on those samples alone.
    """

    auto_steer: bool = True
    legacy: bool = False
    pid: PidV1 | Pid | None = None
    sample_period: float = SAMPLE_PERIOD_S
    angle_setpoint: float = 0.0
    pid_output: float = 0.0
    was_turning: bool = False
    cooldown_timer: float = 0.0
    _was_auto_steer: bool = False
    _held_heading: float | None = field(default=None, repr=False)
    _since_sample: float = field(default=0.0, repr=False)

    def __post_init__(self) -> None:
        if self.pid is None:
            self.pid = PidV1() if self.legacy else Pid()

    def reset_angle_pid(self, heading_deg: float) -> None:
        assert self.pid is not None
        self.pid.reset()
        self.angle_setpoint = heading_deg
        self.pid_output = 0.0
        self.was_turning = False
        self.cooldown_timer = 0.0

    def _sample(self, heading_deg: float, dt: float) -> tuple[float, bool]:
        """The heading the firmware sees this loop, and whether it is a new sample."""
        if self.legacy:
            return heading_deg, True
        self._since_sample += dt
        if self._held_heading is None or self._since_sample >= self.sample_period - 1e-9:
            self._held_heading = heading_deg
            self._since_sample = 0.0
            return heading_deg, True
        return self._held_heading, False

    def _filtered_angular(
        self, b_percent: float, heading_deg: float, dt: float, fresh: bool, reverse: float
    ) -> float:
        angular = b_percent * ANGULAR_SCALE
        if abs(angular) > TURN_THRESHOLD_PERCENT:
            self.angle_setpoint = heading_deg
            self.was_turning = True
            self.cooldown_timer = TURNING_COOLDOWN_TIME * (1.0 - reverse)
            return angular
        if self.was_turning:
            self.cooldown_timer -= dt
            if self.cooldown_timer <= 0.0:
                self.reset_angle_pid(heading_deg)
        if self.was_turning:
            return 0.0
        if isinstance(self.pid, PidV1):
            return self.pid.update(self.angle_setpoint, heading_deg, dt)
        assert self.pid is not None
        if not fresh:
            return self.pid_output
        pd_scale = 1.0 + (REVERSE_GAIN_SCALE - 1.0) * reverse
        return self.pid.update(self.angle_setpoint, heading_deg, self.sample_period, pd_scale)

    def step(
        self,
        a_percent: float,
        b_percent: float,
        heading_deg: float,
        dt: float,
        upside_down: bool = False,
    ) -> tuple[float, float]:
        heading_deg, fresh = self._sample(heading_deg, dt)
        reverse = 0.0
        if not self.legacy and not upside_down:
            reverse = min(max(a_percent / 100.0, 0.0), 1.0)
        if self.auto_steer and not self._was_auto_steer:
            self.reset_angle_pid(heading_deg)
        self._was_auto_steer = self.auto_steer
        if self.auto_steer:
            self.pid_output = self._filtered_angular(b_percent, heading_deg, dt, fresh, reverse)
        else:
            self.pid_output = b_percent
        left = -a_percent + self.pid_output
        right = -a_percent - self.pid_output
        peak = max(abs(left), abs(right))
        if peak > 100.0:
            left, right = left / peak * 100.0, right / peak * 100.0
        return left, right
