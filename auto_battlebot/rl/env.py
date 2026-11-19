"""The source seam: `VecEnv` and what it returns."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

import torch

from auto_battlebot.rl.registry import Registry


@dataclass(frozen=True)
class EnvSpec:
    obs_names: tuple[str, ...]
    privileged_names: tuple[str, ...]
    obs_spec_version: int
    policy_dt_s: float
    max_episode_steps: int
    action_dim: int = 2

    @property
    def obs_dim(self) -> int:
        return len(self.obs_names)

    @property
    def privileged_dim(self) -> int:
        return len(self.privileged_names)

    def index(self, name: str) -> int:
        return self.obs_names.index(name)


@dataclass
class Step:
    """One policy step; where `done`, `obs` is the next episode and `final_*` the last state."""

    obs: torch.Tensor
    privileged: torch.Tensor
    reward: torch.Tensor
    terminated: torch.Tensor
    truncated: torch.Tensor
    final_obs: torch.Tensor
    final_privileged: torch.Tensor
    episode: dict[str, torch.Tensor] = field(default_factory=dict)

    @property
    def done(self) -> torch.Tensor:
        return self.terminated | self.truncated


class VecEnv(Protocol):
    spec: EnvSpec
    num_worlds: int
    device: torch.device

    def reset(self, seed: int) -> tuple[torch.Tensor, torch.Tensor]:
        """Start every world. Returns `(obs, privileged)`."""
        ...

    def step(self, action: torch.Tensor) -> Step:
        """Apply `action` ([W, 2] in [-1, 1]) for one policy period."""
        ...


SOURCES: Registry[type] = Registry("source")
