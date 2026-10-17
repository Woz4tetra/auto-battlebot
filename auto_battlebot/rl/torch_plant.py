"""The grey-box drivetrain plant and the command path, batched in torch for the RL environments.

`auto_battlebot/control/plant.py` is the reference; this port steps one substep for every world
at once with per-world parameters, so domain randomization is a tensor draw. The command path in
front of it mirrors `DifferentialDriveProcessor`: saturation with angular priority, then the zero
deadzone per wheel.
"""

from __future__ import annotations

from dataclasses import dataclass, fields

import torch

from auto_battlebot.control.plant import PlantParams


@dataclass
class TorchPlantParams:
    """`PlantParams` as per-world tensors, each [W]."""

    dz_lin_fwd: torch.Tensor
    dz_lin_rev: torch.Tensor
    dz_ang_l: torch.Tensor
    dz_ang_r: torch.Tensor
    k_fwd: torch.Tensor
    k_rev: torch.Tensor
    k_ang: torch.Tensor
    tau_lin_a: torch.Tensor
    tau_lin_d: torch.Tensor
    tau_ang_a: torch.Tensor
    tau_ang_d: torch.Tensor
    c_sb: torch.Tensor
    c_ad: torch.Tensor
    c_drift: torch.Tensor
    c_drift_bias: torch.Tensor

    @classmethod
    def from_params(
        cls, p: PlantParams, worlds: int, device: torch.device, dtype: torch.dtype
    ) -> TorchPlantParams:
        values = {
            f.name: torch.full((worlds,), float(getattr(p, f.name)), device=device, dtype=dtype)
            for f in fields(cls)
        }
        return cls(**values)

    def scale_(self, name: str, mask: torch.Tensor, factor: torch.Tensor) -> None:
        """Multiply one parameter by `factor` in the `mask` worlds, in place."""
        value = getattr(self, name)
        setattr(self, name, torch.where(mask, value * factor, value))


def saturate(action: torch.Tensor, limit: float = 1.0, zero_deadzone: float = 0.02) -> torch.Tensor:
    """`DifferentialDriveProcessor::process` without the channel reversal: angular keeps priority
    inside `|linear| + |angular| <= limit`, then each wheel loses its zero deadzone."""
    angular = action[:, 1].clamp(-1.0, 1.0)
    headroom = (limit - angular.abs()).clamp(min=0.0)
    linear = torch.maximum(torch.minimum(action[:, 0], headroom), -headroom)
    left = linear + angular
    right = linear - angular

    def deadzone(v: torch.Tensor) -> torch.Tensor:
        mag = ((v.abs() - zero_deadzone) / (1.0 - zero_deadzone)).clamp(0.0, 1.0)
        return torch.where(v.abs() <= zero_deadzone, torch.zeros_like(v), torch.sign(v) * mag)

    left, right = deadzone(left), deadzone(right)
    return torch.stack([0.5 * (left + right), 0.5 * (left - right)], dim=1)


def _effective(u: torch.Tensor, dz_pos: torch.Tensor, dz_neg: torch.Tensor) -> torch.Tensor:
    dz = torch.where(u >= 0.0, dz_pos, dz_neg).clamp(0.0, 0.95)
    return torch.sign(u) * (u.abs() - dz).clamp(min=0.0) / (1.0 - dz)


def steady_state(
    u_lin: torch.Tensor, u_ang: torch.Tensor, p: TorchPlantParams
) -> tuple[torch.Tensor, torch.Tensor]:
    """`plant.steady_state`: target body speed and yaw rate for a held command."""
    lin = _effective(u_lin, p.dz_lin_fwd, p.dz_lin_rev)
    ang = _effective(u_ang, p.dz_ang_l, p.dz_ang_r)
    v = torch.where(lin >= 0.0, p.k_fwd, p.k_rev) * lin
    w = p.k_ang * ang
    v = v * (1.0 - p.c_sb * ang.abs()).clamp(min=0.0)
    w = w * (1.0 - p.c_ad * lin.abs()).clamp(min=0.0)
    w = w + p.c_drift * lin + p.c_drift_bias * torch.sign(lin)
    return v, w


def _alpha(dt: float, tau: torch.Tensor) -> torch.Tensor:
    return torch.where(tau > 1e-9, torch.exp(-dt / tau.clamp(min=1e-9)), torch.zeros_like(tau))


def integrate(
    state: torch.Tensor,
    v_target: torch.Tensor,
    w_target: torch.Tensor,
    dt: float,
    p: TorchPlantParams,
) -> torch.Tensor:
    """`plant.integrate_step` on a [W, 5] state `(x, y, theta, v, w)`: exact arc, then lag."""
    x, y, th, v, w = state.unbind(1)
    th1 = th + w * dt
    turning = w.abs() > 1e-6
    w_safe = torch.where(turning, w, torch.ones_like(w))
    dx = torch.where(turning, v / w_safe * (torch.sin(th1) - torch.sin(th)), v * dt * torch.cos(th))
    dy = torch.where(
        turning, -v / w_safe * (torch.cos(th1) - torch.cos(th)), v * dt * torch.sin(th)
    )
    a_lin = _alpha(dt, torch.where(v_target.abs() > v.abs(), p.tau_lin_a, p.tau_lin_d))
    a_ang = _alpha(dt, torch.where(w_target.abs() > w.abs(), p.tau_ang_a, p.tau_ang_d))
    return torch.stack(
        [
            x + dx,
            y + dy,
            th1,
            a_lin * v + (1 - a_lin) * v_target,
            a_ang * w + (1 - a_ang) * w_target,
        ],
        dim=1,
    )


class CommandDelay:
    """Per-world transport delay as a ring of past commands at the substep rate."""

    def __init__(
        self, worlds: int, capacity: int, device: torch.device, dtype: torch.dtype
    ) -> None:
        self.ring = torch.zeros(worlds, capacity, 2, device=device, dtype=dtype)
        self.capacity = capacity
        self.head = 0
        self.lag = torch.zeros(worlds, dtype=torch.long, device=device)
        self.rows = torch.arange(worlds, device=device)

    def set_lag(self, mask: torch.Tensor, steps: torch.Tensor) -> None:
        self.lag = torch.where(mask, steps.clamp(0, self.capacity - 1), self.lag)
        self.ring[mask] = 0.0

    def push(self, command: torch.Tensor) -> torch.Tensor:
        """Write this substep's command, return the one issued `lag` substeps ago."""
        self.head = (self.head + 1) % self.capacity
        self.ring[:, self.head] = command
        return self.ring[self.rows, (self.head - self.lag) % self.capacity]
