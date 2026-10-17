"""Training config: one TOML per arm, one table per seam; each table picks an entry by `type`."""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, TypeVar

from auto_battlebot.compat import tomllib

T = TypeVar("T")


def build(cls: type[T], table: dict[str, Any], where: str) -> T:
    """`cls(**table)` that names unknown keys instead of raising a bare TypeError."""
    if not dataclasses.is_dataclass(cls):
        raise TypeError(f"{cls} is not a dataclass")
    known = {f.name: f for f in dataclasses.fields(cls)}
    unknown = sorted(set(table) - set(known))
    if unknown:
        raise ValueError(f"[{where}] unknown keys {unknown}; valid: {sorted(known)}")
    values = {}
    for key, value in table.items():
        default = known[key].default
        if isinstance(value, list) and isinstance(default, tuple):
            value = tuple(value)
        values[key] = value
    return cls(**values)


@dataclass
class Budget:
    env_steps: int = 200_000_000
    wall_clock_s: float = 1800.0
    eval_every_s: float = 120.0


@dataclass
class EvalConfig:
    episodes: int = 2048
    seed: int = 12345


@dataclass
class RunConfig:
    name: str
    seed: int
    source: dict[str, Any]
    policy: dict[str, Any]
    learner: dict[str, Any] = field(default_factory=dict)
    budget: Budget = field(default_factory=Budget)
    eval: EvalConfig = field(default_factory=EvalConfig)

    @classmethod
    def load(cls, path: Path, overrides: dict[str, Any] | None = None) -> RunConfig:
        with open(path, "rb") as f:
            data = tomllib.load(f)
        data.update(overrides or {})
        return cls(
            name=str(data.get("name", path.stem)),
            seed=int(data.get("seed", 0)),
            source=dict(data["source"]),
            policy=dict(data["policy"]),
            learner=dict(data.get("learner", {})),
            budget=build(Budget, data.get("budget", {}), "budget"),
            eval=build(EvalConfig, data.get("eval", {}), "eval"),
        )
