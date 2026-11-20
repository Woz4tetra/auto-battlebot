"""A scripted controller that beats a pursuer by out-turning it, on true state, batched in torch."""

from __future__ import annotations

import math
from dataclasses import dataclass

import torch


@dataclass(frozen=True)
class SidestepConfig:
    orbit_m: float = 0.4
    orbit_lead_deg: float = 90.0
    attack_deg: float = 75.0
    gain: float = 0.3
    max_angular: float = 0.5
    lead_s: float = 0.1
    max_linear: float = 1.0
    max_lateral_mps2: float = 12.0
    slow_m: float = 0.45
    strike_linear: float = 0.25
    contact_m: float = 0.03
    back_off_s: float = 0.0
    linear_slew: float = 0.1
    angular_slew: float = 0.1
    back_off_linear: float = -0.3
    switch_deg: float = 30.0


def wrap(a: torch.Tensor) -> torch.Tensor:
    return torch.remainder(a + math.pi, 2.0 * math.pi) - math.pi


class Sidestep:
    """The controller for a batch of worlds."""

    def __init__(self, worlds: int, device: torch.device | str, cfg: SidestepConfig) -> None:
        self.cfg = cfg
        self.side = torch.ones(worlds, device=device)
        self.last = torch.zeros(worlds, 2, device=device)
        self.back_off = torch.zeros(worlds, device=device)

    def __call__(self, tr: dict[str, torch.Tensor]) -> torch.Tensor:
        """Actions from WarpNavEnv.truth() plus reach."""
        cfg = self.cfg
        xy, yaw, opp_xy, opp_yaw = tr["xy"], tr["yaw"], tr["opp_xy"], tr["opp_yaw"]
        rel = xy - opp_xy
        bearing = torch.atan2(rel[:, 1], rel[:, 0])
        phi = wrap(bearing - opp_yaw)
        switch = math.radians(cfg.switch_deg)
        self.side = torch.where(phi > switch, 1.0, torch.where(phi < -switch, -1.0, self.side))
        aim = bearing + self.side * math.radians(cfg.orbit_lead_deg)
        waypoint = opp_xy + cfg.orbit_m * torch.stack([torch.cos(aim), torch.sin(aim)], 1)
        behind = phi.abs() > math.radians(cfg.attack_deg)
        target = torch.where(behind[:, None], opp_xy, waypoint)
        to = target - xy
        err = wrap(torch.atan2(to[:, 1], to[:, 0]) - yaw - tr["yaw_rate"] * cfg.lead_s)
        angular = (cfg.gain * err).clamp(-cfg.max_angular, cfg.max_angular)
        linear = cfg.max_linear * torch.cos(err).clamp(min=0.0)
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
