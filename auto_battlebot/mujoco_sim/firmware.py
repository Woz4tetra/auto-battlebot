"""Mr Stabs Mk2 firmware layer between the radio and the ESCs, for closed-loop simulation.

Mirrors `mix_motor_outputs` (firmware/mr_stabs_mk2/src/main.cpp) and
`yaw_control::YawController` (firmware/mr_stabs_mk2/lib/yaw_control/) line for line. The fit
never uses this: it replays the logged per-motor commands, which already come out of the mixer.

Two firmware generations are mirrored, because validate.py replays recordings made before
the yaw-rate loop:

- `legacy=True`: heading hold with `PidV1` every control loop, quirks included: the integral
  accumulates the raw error and multiplies by dt only on output, and the tolerance check
  returns before the derivative term updates its previous error. The stick passes through
  while turning and for a 0.25 s coast after.
- `legacy=False` (default): the current firmware. In auto steer, `YawController` runs once
  per BNO055 sample (100 Hz): the stick commands a yaw rate, heading hold commands the rate
  back to the held heading, and an inner loop on the gyro sets the differential. Under 5%
  throttle with the stick centered it idles. It runs upside down too; outside auto steer the
  stick passes through with a 1% deadband.

The BNO055 heading (`orientation_x`) is degrees in [0, 360) and grows clockwise seen from above;
`heading_from_yaw` converts the sim's counterclockwise yaw into it, and rates here are
clockwise-positive to match. That sign is an assumption until a recording pins it (plan
validation check 5); the firmware checks its own gyro sign at runtime.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

TURNING_COOLDOWN_TIME = 0.25  # legacy only
ANGULAR_SCALE = 1.0  # legacy only
TURN_THRESHOLD_PERCENT = 1.0
SAMPLE_PERIOD_S = 0.01  # updown_sensor::SAMPLE_INTERVAL
GYRO_RANGE_DPS = 2000.0  # BNO055 gyro full scale in its fusion modes


def heading_from_yaw(yaw_rad: float) -> float:
    return (-math.degrees(yaw_rad)) % 360.0


def gyro_from_yaw_rate(yaw_rate_rad_s: float) -> float:
    """What the firmware's get_yaw_rate() reads: clockwise deg/s, clipped at the gyro range."""
    rate = -math.degrees(yaw_rate_rad_s)
    return max(-GYRO_RANGE_DPS, min(GYRO_RANGE_DPS, rate))


def _wrap(error: float, min_input: float, max_input: float) -> float:
    span = max_input - min_input
    half = span / 2.0
    shifted = error + half
    return shifted - math.floor(shifted / span) * span - half


def wrap_degrees(angle: float) -> float:
    """yaw_control::wrap_degrees: [-180, 180)."""
    return angle - 360.0 * math.floor((angle + 180.0) / 360.0)


@dataclass
class PidV1:
    """pid::Pid before 2026-10-05, quirks included, with the gains main.cpp used then."""

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
class YawConfig:
    """yaw_control::Config defaults."""

    max_rate: float = 2000.0
    feedforward: float = 1.0 / 27.0
    kp: float = 0.02
    ki: float = 0.02
    i_max: float = 20.0
    k_heading: float = 6.0
    hold_rate_max: float = 360.0
    reverse_kp_scale: float = 4.0
    reverse_ff_scale: float = 0.25
    turn_threshold: float = TURN_THRESHOLD_PERCENT
    capture_rate: float = 45.0
    capture_timeout: float = 0.3
    idle_throttle: float = 5.0


@dataclass
class YawController:
    """yaw_control::YawController."""

    config: YawConfig = field(default_factory=YawConfig)
    setpoint: float = 0.0
    capturing: bool = False
    capture_time: float = 0.0
    integral: float = 0.0
    output: float = 0.0
    rate_command: float = 0.0

    def reset(self, heading: float) -> None:
        self.setpoint = heading
        self.capturing = False
        self.capture_time = 0.0
        self.integral = 0.0
        self.output = 0.0
        self.rate_command = 0.0

    def update(
        self, turn_percent: float, throttle: float, heading: float, rate: float, dt: float
    ) -> float:
        c = self.config
        turning = abs(turn_percent) > c.turn_threshold
        if not turning and abs(throttle) < c.idle_throttle:
            self.capturing = False
            self.setpoint = heading
            self.rate_command = 0.0
            self.output = 0.0
            return self.output
        feedforward_command = 0.0
        if turning:
            self.rate_command = turn_percent / 100.0 * c.max_rate
            feedforward_command = self.rate_command
            self.capturing = True
            self.capture_time = 0.0
        else:
            if self.capturing:
                self.capture_time += dt
                if abs(rate) < c.capture_rate or self.capture_time >= c.capture_timeout:
                    self.capturing = False
                    self.setpoint = heading
            if self.capturing:
                self.rate_command = 0.0
            else:
                command = c.k_heading * wrap_degrees(self.setpoint - heading)
                self.rate_command = max(-c.hold_rate_max, min(c.hold_rate_max, command))
        reverse = min(max(throttle / 100.0, 0.0), 1.0)
        kp = c.kp * (1.0 + (c.reverse_kp_scale - 1.0) * reverse)
        feedforward = c.feedforward * (1.0 + (c.reverse_ff_scale - 1.0) * reverse)
        error = self.rate_command - rate
        unsaturated = feedforward * feedforward_command + kp * error + self.integral
        if abs(unsaturated) < 100.0 or (unsaturated > 0.0) != (error > 0.0):
            self.integral = max(-c.i_max, min(c.i_max, self.integral + c.ki * error * dt))
        out = feedforward * feedforward_command + kp * error + self.integral
        self.output = max(-100.0, min(100.0, out))
        return self.output


@dataclass
class FirmwareMixer:
    """Stick percents, the sensed heading and gyro rate in, per-motor percents out.

    `step` is one control loop. In the current firmware the heading and gyro only change when a
    BNO055 sample lands, every `sample_period`, and the yaw loop runs on those samples alone.
    `pid_output` is the differential the mixer adds to the left wheel and takes off the right.
    """

    auto_steer: bool = True
    legacy: bool = False
    pid: PidV1 = field(default_factory=PidV1)
    yaw: YawController = field(default_factory=YawController)
    sample_period: float = SAMPLE_PERIOD_S
    pid_output: float = 0.0
    # legacy heading-hold state
    angle_setpoint: float = 0.0
    was_turning: bool = False
    cooldown_timer: float = 0.0
    _was_auto_steer: bool = False
    # current firmware state
    yaw_loop_active: bool = False
    _was_upside_down: bool = field(default=False, repr=False)
    _held: tuple[float, float] | None = field(default=None, repr=False)
    _since_sample: float = field(default=0.0, repr=False)

    def reset_angle_pid(self, heading_deg: float) -> None:
        """Legacy heading-hold reset."""
        self.pid.reset()
        self.angle_setpoint = heading_deg
        self.pid_output = 0.0
        self.was_turning = False
        self.cooldown_timer = 0.0

    def _legacy_angular(self, b_percent: float, heading_deg: float, dt: float) -> float:
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

    def _sample(self, heading_deg: float, rate_dps: float, dt: float) -> tuple[float, float, bool]:
        """The heading and gyro rate the firmware sees this loop, and whether they are new."""
        self._since_sample += dt
        if self._held is None or self._since_sample >= self.sample_period - 1e-9:
            self._held = (heading_deg, rate_dps)
            self._since_sample = 0.0
            return heading_deg, rate_dps, True
        return self._held[0], self._held[1], False

    def _legacy_step(
        self, a_percent: float, b_percent: float, heading_deg: float, dt: float
    ) -> None:
        if self.auto_steer and not self._was_auto_steer:
            self.reset_angle_pid(heading_deg)
        self._was_auto_steer = self.auto_steer
        if self.auto_steer:
            self.pid_output = self._legacy_angular(b_percent, heading_deg, dt)
        else:
            self.pid_output = b_percent

    def _current_step(
        self,
        a_percent: float,
        b_percent: float,
        heading_deg: float,
        rate_dps: float,
        dt: float,
        upside_down: bool,
    ) -> None:
        heading, rate, fresh = self._sample(heading_deg, rate_dps, dt)
        # a_percent arrives already negated when inverted; stick back drives tail-first either way.
        tail_first_throttle = -a_percent if upside_down else a_percent
        if not self.auto_steer:
            self.pid_output = b_percent if abs(b_percent) > TURN_THRESHOLD_PERCENT else 0.0
            self.yaw_loop_active = False
            return
        if not self.yaw_loop_active or upside_down != self._was_upside_down:
            self.yaw.reset(heading)
            self.yaw_loop_active = True
        if fresh:
            self.pid_output = self.yaw.update(
                b_percent, tail_first_throttle, heading, rate, self.sample_period
            )
        self._was_upside_down = upside_down

    def step(
        self,
        a_percent: float,
        b_percent: float,
        heading_deg: float,
        dt: float,
        upside_down: bool = False,
        yaw_rate_dps: float = 0.0,
    ) -> tuple[float, float]:
        """One control loop. yaw_rate_dps is the gyro, clockwise; legacy ignores it."""
        if self.legacy:
            self._legacy_step(a_percent, b_percent, heading_deg, dt)
        else:
            self._current_step(a_percent, b_percent, heading_deg, yaw_rate_dps, dt, upside_down)
        left = -a_percent + self.pid_output
        right = -a_percent - self.pid_output
        peak = max(abs(left), abs(right))
        if peak > 100.0:
            left, right = left / peak * 100.0, right / peak * 100.0
        return left, right
