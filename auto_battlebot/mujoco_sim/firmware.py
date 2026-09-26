"""Mr Stabs Mk2 firmware layer between the radio and the ESCs, for closed-loop simulation.

Mirrors `mix_motor_outputs`, `get_filtered_angular_z` and `pid::Pid` in
firmware/mr_stabs_mk2/src/ line for line, including their quirks: the integral accumulates the
raw error and multiplies by dt only on output, and the tolerance check returns before the
derivative term updates its previous error. The fit never uses this: it replays the logged
per-motor commands, which already come out of the mixer.

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


def heading_from_yaw(yaw_rad: float) -> float:
    return (-math.degrees(yaw_rad)) % 360.0


@dataclass
class Pid:
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
        span = self.max_input - self.min_input
        half = span / 2.0
        shifted = error + half
        return shifted - math.floor(shifted / span) * span - half

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
class FirmwareMixer:
    """Stick percents and the sensed heading in, per-motor percents out, like the robot."""

    auto_steer: bool = True
    pid: Pid = field(default_factory=Pid)
    angle_setpoint: float = 0.0
    pid_output: float = 0.0
    was_turning: bool = False
    cooldown_timer: float = 0.0
    _was_auto_steer: bool = False

    def reset_angle_pid(self, heading_deg: float) -> None:
        self.pid.reset()
        self.angle_setpoint = heading_deg
        self.pid_output = 0.0
        self.was_turning = False
        self.cooldown_timer = 0.0

    def _filtered_angular(self, b_percent: float, heading_deg: float, dt: float) -> float:
        angular = b_percent * ANGULAR_SCALE
        if abs(angular) > TURN_THRESHOLD_PERCENT:
            self.angle_setpoint = heading_deg
            self.was_turning = True
            self.cooldown_timer = TURNING_COOLDOWN_TIME
            return angular
        if self.was_turning:
            self.cooldown_timer -= dt
            if self.cooldown_timer <= 0.0:
                self.reset_angle_pid(heading_deg)
        if self.was_turning:
            return 0.0
        return self.pid.update(self.angle_setpoint, heading_deg, dt)

    def step(
        self, a_percent: float, b_percent: float, heading_deg: float, dt: float
    ) -> tuple[float, float]:
        if self.auto_steer and not self._was_auto_steer:
            self.reset_angle_pid(heading_deg)
        self._was_auto_steer = self.auto_steer
        if self.auto_steer:
            self.pid_output = self._filtered_angular(b_percent, heading_deg, dt)
        else:
            self.pid_output = b_percent
        left = -a_percent + self.pid_output
        right = -a_percent - self.pid_output
        peak = max(abs(left), abs(right))
        if peak > 100.0:
            left, right = left / peak * 100.0, right / peak * 100.0
        return left, right
