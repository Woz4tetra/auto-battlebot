"""`pursuit`: `PursuitNavigation`'s control law as a policy, the toy's scripted baseline.

It reads the opponent's body-frame position from the observation and steers at it with the same
PD heading loop, angle threshold, distance ramp, turn scale and steer brake as
`src/navigation/pursuit_navigation.cpp`. Defaults are the Mr Stabs Mk2 profile's
(`config/mr_stabs_mk2_desktop.toml`), except `max_linear_command`, which the evaluation sets per
arm because 0.25 is a bench setting, not a fighting one.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import torch

from auto_battlebot.rl.env import EnvSpec
from auto_battlebot.rl.policies.base import POLICIES, PolicyState


@dataclass
class PursuitConfig:
    angular_kp: float = 0.06
    angular_kd: float = 0.005
    max_linear_command: float = 0.25
    angle_threshold: float = 1.7
    velocity_ramp_far_distance: float = 1.5
    velocity_ramp_near_distance: float = 1.0
    velocity_ramp_min_scale: float = 0.35
    stop_distance: float = 0.0


@POLICIES.register("pursuit")
class PursuitPolicy:
    def __init__(self, spec: EnvSpec, cfg: PursuitConfig | None = None) -> None:
        self.cfg = cfg or PursuitConfig()
        self.dt = spec.policy_dt_s
        self.ix = spec.index("opp_x")
        self.iy = spec.index("opp_y")
        self.iknown = spec.index("opp_known")

    def act(
        self, obs: torch.Tensor, state: PolicyState | None = None, deterministic: bool = True
    ) -> tuple[torch.Tensor, PolicyState | None]:
        c = self.cfg
        x = obs[:, self.ix] * 2.0
        y = obs[:, self.iy] * 2.0
        known = obs[:, self.iknown] > 0.5
        error = torch.atan2(y, x)
        distance = torch.hypot(x, y)
        # NaN marks a world with no previous error yet (episode start), as prev_timestamp_ = 0 does.
        prev = state["prev_error"] if state is not None else torch.full_like(error, math.nan)
        prev = torch.where(torch.isnan(prev), error, prev)
        d_term = c.angular_kd * (torch.remainder(error - prev + math.pi, 2 * math.pi) - math.pi)
        angular = c.angular_kp * error + d_term / self.dt

        near, far = c.velocity_ramp_near_distance, c.velocity_ramp_far_distance
        ramp = (distance - near) / (far - near)
        speed = torch.where(
            distance <= near,
            torch.ones_like(distance),
            torch.where(
                distance >= far,
                torch.full_like(distance, c.velocity_ramp_min_scale),
                1.0 - ramp * (1.0 - c.velocity_ramp_min_scale),
            ),
        )
        turn = 1.0 - error.abs() / c.angle_threshold * 0.5
        brake = (1.0 - 0.5 * angular.abs()).clamp(min=0.0)
        linear = torch.where(
            error.abs() >= c.angle_threshold, torch.zeros_like(speed), speed * turn * brake
        )
        if c.max_linear_command > 0.0:
            linear = linear.clamp(-c.max_linear_command, c.max_linear_command)
        stop = distance < c.stop_distance
        linear = torch.where(known & ~stop, linear, torch.zeros_like(linear))
        angular = torch.where(known & ~stop, angular, torch.zeros_like(angular))
        action = torch.stack([linear, angular], dim=1).clamp(-1.0, 1.0)
        return action, {"prev_error": torch.where(known, error, prev)}

    def reset_state(self, state: PolicyState | None, done: torch.Tensor) -> PolicyState | None:
        if state is None:
            return None
        return {"prev_error": state["prev_error"].masked_fill(done, math.nan)}
