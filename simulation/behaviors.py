"""Opponent behaviour strategies for the simulation."""

from __future__ import annotations

import abc
import math
import random
from typing import TYPE_CHECKING, Any

import numpy as np
import numpy.typing as npt

from config.robot import RobotConfig

if TYPE_CHECKING:
    from genesis.engine.entities.rigid_entity import RigidEntity

DOF_VX, DOF_VY = 0, 1


class OpponentBehavior(abc.ABC):
    """Base class for all opponent behaviour strategies."""

    @abc.abstractmethod
    def step(self, entity: RigidEntity, dt: float) -> None: ...


class StaticBehavior(OpponentBehavior):
    def step(self, entity: RigidEntity, dt: float) -> None:
        entity.set_dofs_velocity(np.array([0.0, 0.0]), [DOF_VX, DOF_VY])


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
            entity.set_dofs_velocity(np.array([0.0, 0.0]), [DOF_VX, DOF_VY])
            return
        direction = diff / dist
        vel = direction * self.speed
        entity.set_dofs_velocity(np.array([vel[0], vel[1]]), [DOF_VX, DOF_VY])


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
        vel_x = -self.speed * math.sin(self.angle)
        vel_y = self.speed * math.cos(self.angle)
        entity.set_dofs_velocity(np.array([vel_x, vel_y]), [DOF_VX, DOF_VY])


def make_opponent_behavior(
    cfg: RobotConfig, arena_half_w: float, arena_half_h: float
) -> OpponentBehavior:
    if cfg.behavior == "random_walk":
        return RandomWalkBehavior(cfg.speed, arena_half_w, arena_half_h)
    elif cfg.behavior == "circular":
        return CircularBehavior(cfg.speed)
    else:
        return StaticBehavior()
