"""`mlp_gaussian`: an MLP actor with a tanh-squashed Gaussian head and input normalization.

The network is a sequential stack of Dense layers with ELU, which RTNeural runs, and the running
mean and std are folded into the first Dense layer at export, so the C++ side needs no
normalization step (plan section 11).
"""

from __future__ import annotations

from dataclasses import dataclass

import torch
from torch import nn

from auto_battlebot.rl.env import EnvSpec
from auto_battlebot.rl.policies.base import POLICIES, PolicyState


class RunningNorm(nn.Module):
    """Running mean and variance (Chan's parallel update), frozen once `frozen` is set."""

    mean: torch.Tensor
    var: torch.Tensor
    count: torch.Tensor

    def __init__(self, dim: int, eps: float = 1e-4) -> None:
        super().__init__()
        self.register_buffer("mean", torch.zeros(dim))
        self.register_buffer("var", torch.ones(dim))
        self.register_buffer("count", torch.tensor(eps))
        self.frozen = False

    def update(self, x: torch.Tensor) -> None:
        if self.frozen:
            return
        x = x.reshape(-1, x.shape[-1]).float()
        batch_mean = x.mean(0)
        batch_var = x.var(0, unbiased=False)
        n = x.shape[0]
        total = self.count + n
        delta = batch_mean - self.mean
        self.mean += delta * n / total
        self.var = (
            self.var * self.count + batch_var * n + delta**2 * self.count * n / total
        ) / total
        self.count = total

    @property
    def std(self) -> torch.Tensor:
        # A constant input (a flag that never fires in a stage) keeps a std of 1, not 0.
        return torch.where(self.var > 1e-8, self.var.sqrt(), torch.ones_like(self.var))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return (x - self.mean) / self.std


def mlp(sizes: list[int], activation: type[nn.Module] = nn.ELU) -> nn.Sequential:
    layers: list[nn.Module] = []
    for i in range(len(sizes) - 1):
        layers.append(nn.Linear(sizes[i], sizes[i + 1]))
        if i < len(sizes) - 2:
            layers.append(activation())
    return nn.Sequential(*layers)


@dataclass
class MlpGaussianConfig:
    hidden: tuple[int, ...] = (256, 256, 128)
    init_log_std: float = -0.5
    min_log_std: float = -4.0


@POLICIES.register("mlp_gaussian")
class MlpGaussianPolicy(nn.Module):
    def __init__(self, spec: EnvSpec, cfg: MlpGaussianConfig | None = None) -> None:
        super().__init__()
        cfg = cfg or MlpGaussianConfig()
        self.cfg = cfg
        self.spec = spec
        self.norm: RunningNorm = RunningNorm(spec.obs_dim)
        self.actor: nn.Sequential = mlp([spec.obs_dim, *cfg.hidden, spec.action_dim])
        # Small last layer: the policy starts near zero action instead of saturating the tanh.
        last = self.actor[-1]
        assert isinstance(last, nn.Linear)
        nn.init.uniform_(last.weight, -1e-3, 1e-3)
        nn.init.zeros_(last.bias)
        self.log_std = nn.Parameter(torch.full((spec.action_dim,), cfg.init_log_std))

    def distribution(self, obs: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """Mean and std of the pre-tanh Gaussian."""
        mean = self.actor(self.norm(obs))
        std = self.log_std.clamp(min=self.cfg.min_log_std).exp().expand_as(mean)
        return mean, std

    @torch.no_grad()
    def act(
        self, obs: torch.Tensor, state: PolicyState | None = None, deterministic: bool = True
    ) -> tuple[torch.Tensor, PolicyState | None]:
        mean, std = self.distribution(obs)
        u = mean if deterministic else mean + std * torch.randn_like(mean)
        return torch.tanh(u), state

    def reset_state(self, state: PolicyState | None, done: torch.Tensor) -> PolicyState | None:
        return state
