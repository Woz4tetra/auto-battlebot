"""The source seam: `VecEnv` and what it returns.

Every environment is batched over worlds and auto-resets a world the step it finishes, so a
learner never waits on a slow world. Observations are one flat vector per world whose layout is
named in `EnvSpec.obs_names`; the privileged vector is the critic's extra input and never reaches
the actor or the exported bundle.
"""

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
    """One policy step for every world.

    `obs` and `privileged` already belong to the next episode in worlds that finished; the
    `final_*` tensors hold the last state of the finished episode, which the learner bootstraps
    from on truncation. `episode` carries per-world totals, valid in rows where `done` is set.
    """

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
