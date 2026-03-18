"""Opponent behaviour strategies for the simulation."""

from __future__ import annotations

import abc
import math
import random
from typing import TYPE_CHECKING, Any

import numpy as np
import numpy.typing as npt

if TYPE_CHECKING:
    from genesis.engine.entities.rigid_entity import RigidEntity


class OpponentBehavior(abc.ABC):
    """Base class for all opponent behaviour strategies."""

    @abc.abstractmethod
    def step(self, entity: RigidEntity, dt: float) -> None: ...


class StaticBehavior(OpponentBehavior):
    def step(self, entity: RigidEntity, dt: float) -> None:
        pass


class RandomWalkBehavior(OpponentBehavior):
    def __init__(self, speed: float, arena_half_w: float, arena_half_h: float) -> None:
        self.speed: float = speed
        self.arena_half_w: float = arena_half_w
        self.arena_half_h: float = arena_half_h
        self.target: npt.NDArray[np.floating[Any]] = self._random_target()

    def _random_target(self) -> npt.NDArray[np.floating[Any]]:
        x = random.uniform(-self.arena_half_w * 0.8, self.arena_half_w * 0.8)
        y = random.uniform(-self.arena_half_h * 0.8, self.arena_half_h * 0.8)
        return np.array([x, y])

    def step(self, entity: RigidEntity, dt: float) -> None:
        pos: npt.NDArray[np.floating[Any]] = entity.get_pos().cpu().numpy().squeeze()
        current = pos[:2]
        diff = self.target - current
        dist: float = float(np.linalg.norm(diff))
        if dist < 0.05:
            self.target = self._random_target()
            return
        direction = diff / dist
        step = direction * min(self.speed * dt, dist)
        new_pos = pos.copy()
        new_pos[0] += step[0]
        new_pos[1] += step[1]
        entity.set_pos(new_pos)


class CircularBehavior(OpponentBehavior):
    def __init__(
        self,
        speed: float,
        radius: float = 0.5,
        center: tuple[float, float] = (0.0, 0.0),
    ) -> None:
        self.speed: float = speed
        self.radius: float = radius
        self.center: npt.NDArray[np.floating[Any]] = np.array(center)
        self.angle: float = 0.0

    def step(self, entity: RigidEntity, dt: float) -> None:
        self.angle += (self.speed / max(self.radius, 0.01)) * dt
        x = self.center[0] + self.radius * math.cos(self.angle)
        y = self.center[1] + self.radius * math.sin(self.angle)
        pos: npt.NDArray[np.floating[Any]] = entity.get_pos().cpu().numpy().squeeze()
        pos[0] = x
        pos[1] = y
        entity.set_pos(pos)


def make_opponent_behavior(
    cfg: dict[str, Any], arena_half_w: float, arena_half_h: float
) -> OpponentBehavior:
    behavior: str = cfg.get("behavior", "static")
    speed: float = cfg.get("speed", 0.3)
    if behavior == "random_walk":
        return RandomWalkBehavior(speed, arena_half_w, arena_half_h)
    elif behavior == "circular":
        return CircularBehavior(speed)
    else:
        return StaticBehavior()
