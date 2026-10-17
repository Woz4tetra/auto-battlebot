"""The policy seam. Learned and scripted policies share it, so one evaluator scores both."""

from __future__ import annotations

from typing import Any, Protocol

import torch

from auto_battlebot.rl.registry import Registry

PolicyState = dict[str, torch.Tensor]


class Policy(Protocol):
    def act(
        self, obs: torch.Tensor, state: PolicyState | None, deterministic: bool
    ) -> tuple[torch.Tensor, PolicyState | None]:
        """Actions in [-1, 1], [W, 2], and the next recurrent state."""
        ...

    def reset_state(self, state: PolicyState | None, done: torch.Tensor) -> PolicyState | None:
        """Clear the recurrent state of worlds that started a new episode."""
        ...


POLICIES: Registry[Any] = Registry("policy")
