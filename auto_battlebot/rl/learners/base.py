"""The learner seam: one `iterate()` is one unit of training, whatever the algorithm."""

from __future__ import annotations

from typing import Any, Protocol

import torch

from auto_battlebot.rl.registry import Registry


class Learner(Protocol):
    env_steps: int

    def iterate(self) -> dict[str, float]: ...

    def state_dict(self) -> dict[str, Any]: ...

    def load_state_dict(self, state: dict[str, Any]) -> None: ...


LEARNERS: Registry[Any] = Registry("learner")


class EpisodeLog:
    """Per-episode totals from `Step.episode`, accumulated between `iterate()` calls."""

    def __init__(self) -> None:
        self.rows: dict[str, list[torch.Tensor]] = {}

    def add(self, episode: dict[str, torch.Tensor], done: torch.Tensor) -> None:
        if not episode or not bool(done.any()):
            return
        for key, value in episode.items():
            self.rows.setdefault(key, []).append(value[done].detach())

    def summary(self) -> dict[str, float]:
        if not self.rows:
            return {}
        cat = {k: torch.cat(v).float() for k, v in self.rows.items()}
        self.rows = {}
        out = {"episodes": float(next(iter(cat.values())).numel())}
        for key, value in cat.items():
            finite = value[torch.isfinite(value)]
            out[f"ep_{key}"] = float(finite.mean()) if finite.numel() else float("nan")
        if "strikes" in cat:
            out["ep_success"] = float((cat["strikes"] > 0).float().mean())
        return out
