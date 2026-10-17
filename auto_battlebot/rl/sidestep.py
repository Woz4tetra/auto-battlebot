"""A scripted controller that beats a pursuer by out-turning it, on true state, batched in torch.

It circles the opponent at `orbit_m`, aiming `orbit_lead_deg` further round towards its rear on
the side we are already on. A pursuer turns to keep us in front of it, at 4 to 10 rad/s; Mr Stabs
Mk2 turns at up to 37 rad/s in the Warp plant, so on a small circle we can get round faster than
the pursuer turns. Once our bearing from its heading passes `attack_deg` it drives at the
opponent's centre for the strike. Commands are slew-limited: a step from 0 to full throttle
wheelies the chassis over.

It plays the expert for the pursuit beatable-range map (`playground/rl/pursuit_map.py`) and the
behaviour-cloning warm start (`training/rl/pretrain_sidestep.py`). Actions are the policy's:
(linear, angular) in [-1, 1], positive angular turning counter-clockwise.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import torch


@dataclass(frozen=True)
class SidestepConfig:
    orbit_m: float = 0.4  # radius we circle the opponent at while working round to its rear
    orbit_lead_deg: float = 60.0  # aim this far further round the circle than where we are
    attack_deg: float = 120.0  # ram only once our bearing from its heading is past this
    # Heading loop, tuned on a 90 degree step in the Warp plant: settles in 0.4 s; gain 0.8
    # oscillated at +-17 rad/s through the 40 to 80 ms command latency.
    gain: float = 0.3  # angular action per radian of heading error
    max_angular: float = 0.5  # about 19 rad/s; full angular spins the chassis over
    lead_s: float = 0.1  # heading error is taken this far ahead at the current yaw rate
    max_linear: float = 0.7
    # Linear action is capped so speed x yaw rate stays under this: harder turns roll it over.
    max_lateral_mps2: float = 8.0
    # Inside `slow_m` of touching, linear action is capped at `strike_linear`: ramming the
    # immovable opponent at 2.6 m/s stopped it in one step and pitched it over its nose.
    slow_m: float = 0.3
    strike_linear: float = 0.4
    contact_m: float = 0.03  # gap at which we count as touching and back off
    back_off_s: float = 0.25  # reverse this long after touching; pressing on rides up and flips
    # Most each action may change per 20 ms step. Swinging linear from -0.5 to +0.6 in 0.16 s
    # while backing at 2 m/s pitched the nose to -84 degrees and flipped it.
    linear_slew: float = 0.04
    angular_slew: float = 0.1
    back_off_linear: float = -0.3
    switch_deg: float = 30.0  # change orbit direction only once this far over the centre line


def wrap(a: torch.Tensor) -> torch.Tensor:
    return torch.remainder(a + math.pi, 2.0 * math.pi) - math.pi


class Sidestep:
    """The controller for a batch of worlds. It keeps the orbit direction per world, so it does
    not reverse every time we cross the pursuer's centre line, and its last action, for the
    slew limit."""

    def __init__(self, worlds: int, device: torch.device | str, cfg: SidestepConfig) -> None:
        self.cfg = cfg
        self.side = torch.ones(worlds, device=device)
        self.last = torch.zeros(worlds, 2, device=device)
        self.back_off = torch.zeros(worlds, device=device)

    def __call__(self, tr: dict[str, torch.Tensor]) -> torch.Tensor:
        """[W, 2] actions from `WarpNavEnv.truth()` plus `reach`, the centre distance at which
        the two footprints touch along the line between them ([W])."""
        cfg = self.cfg
        xy, yaw, opp_xy, opp_yaw = tr["xy"], tr["yaw"], tr["opp_xy"], tr["opp_yaw"]
        rel = xy - opp_xy
        bearing = torch.atan2(rel[:, 1], rel[:, 0])
        phi = wrap(bearing - opp_yaw)
        switch = math.radians(cfg.switch_deg)
        self.side = torch.where(phi > switch, 1.0, torch.where(phi < -switch, -1.0, self.side))
        # Round the circle away from its nose: the side we are on, towards its rear.
        aim = bearing + self.side * math.radians(cfg.orbit_lead_deg)
        waypoint = opp_xy + cfg.orbit_m * torch.stack([torch.cos(aim), torch.sin(aim)], 1)
        behind = phi.abs() > math.radians(cfg.attack_deg)
        target = torch.where(behind[:, None], opp_xy, waypoint)
        to = target - xy
        err = wrap(torch.atan2(to[:, 1], to[:, 0]) - yaw - tr["yaw_rate"] * cfg.lead_s)
        angular = (cfg.gain * err).clamp(-cfg.max_angular, cfg.max_angular)
        linear = cfg.max_linear * torch.cos(err).clamp(min=0.0)
        # Full linear action is about 2.6 m/s and full angular about 37 rad/s.
        lateral_cap = cfg.max_lateral_mps2 / (2.6 * 37.0 * angular.abs().clamp(min=1e-3))
        linear = torch.minimum(linear, lateral_cap)
        gap = rel.norm(dim=1) - tr["reach"]
        linear = torch.where(gap < cfg.slow_m, linear.clamp(max=cfg.strike_linear), linear)
        touching = gap < cfg.contact_m
        self.back_off = torch.where(
            touching, torch.full_like(self.back_off, cfg.back_off_s), self.back_off - 0.02
        )
        backing = self.back_off > 0.0
        linear = torch.where(backing, torch.full_like(linear, cfg.back_off_linear), linear)
        angular = torch.where(backing, torch.zeros_like(angular), angular)
        want = torch.stack([linear, angular], 1)
        slew = torch.tensor([cfg.linear_slew, cfg.angular_slew], device=want.device)
        self.last = self.last + torch.maximum(torch.minimum(want - self.last, slew), -slew)
        return self.last.clone()
