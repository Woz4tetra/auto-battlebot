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
    from sim_types import WheelDriveInfo

DOF_VX, DOF_VY = 0, 1


def _apply_velocity(
    entity: RigidEntity,
    vx: float,
    vy: float,
    wheels: WheelDriveInfo | None,
) -> None:
    """Set entity velocity, using wheels when available, else base DOFs."""
    if wheels is not None:
        speed = math.hypot(vx, vy)
        heading = math.atan2(vy, vx)
        entity_pos = entity.get_pos().cpu().numpy().squeeze()
        v_left = speed / wheels.wheel_radius
        v_right = speed / wheels.wheel_radius
        velocities = [v_left] * wheels.n_left + [v_right] * wheels.n_right
        entity.control_dofs_velocity(velocities, dofs_idx_local=wheels.all_dofs)
    else:
        entity.set_dofs_velocity(np.array([vx, vy]), [DOF_VX, DOF_VY])


def _stop(entity: RigidEntity, wheels: WheelDriveInfo | None) -> None:
    """Stop the entity."""
    if wheels is not None:
        velocities = [0.0] * (wheels.n_left + wheels.n_right)
        entity.control_dofs_velocity(velocities, dofs_idx_local=wheels.all_dofs)
    else:
        entity.set_dofs_velocity(np.array([0.0, 0.0]), [DOF_VX, DOF_VY])


class OpponentBehavior(abc.ABC):
    """Base class for all opponent behaviour strategies."""

    def __init__(self, wheels: WheelDriveInfo | None = None) -> None:
        self._wheels = wheels

    @abc.abstractmethod
    def step(self, entity: RigidEntity, dt: float) -> None: ...


class StaticBehavior(OpponentBehavior):
    def step(self, entity: RigidEntity, dt: float) -> None:
        _stop(entity, self._wheels)


class RandomWalkBehavior(OpponentBehavior):
    def __init__(
        self,
        speed: float,
        arena_half_w: float,
        arena_half_h: float,
        wheels: WheelDriveInfo | None = None,
    ) -> None:
        super().__init__(wheels)
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
            _stop(entity, self._wheels)
            return
        direction = diff / dist
        vel = direction * self.speed
        _apply_velocity(entity, float(vel[0]), float(vel[1]), self._wheels)


class CircularBehavior(OpponentBehavior):
    def __init__(
        self,
        speed: float,
        radius: float = 0.5,
        center: tuple[float, float] = (0.0, 0.0),
        wheels: WheelDriveInfo | None = None,
    ) -> None:
        super().__init__(wheels)
        self.speed: float = speed
        self.radius: float = radius
        self.center: npt.NDArray[np.floating[Any]] = np.array(center)
        self.angle: float = 0.0

    def step(self, entity: RigidEntity, dt: float) -> None:
        self.angle += (self.speed / max(self.radius, 0.01)) * dt
        vel_x = -self.speed * math.sin(self.angle)
        vel_y = self.speed * math.cos(self.angle)
        _apply_velocity(entity, vel_x, vel_y, self._wheels)


def make_opponent_behavior(
    cfg: RobotConfig,
    arena_half_w: float,
    arena_half_h: float,
    wheels: WheelDriveInfo | None = None,
) -> OpponentBehavior:
    if cfg.behavior == "random_walk":
        return RandomWalkBehavior(cfg.speed, arena_half_w, arena_half_h, wheels)
    elif cfg.behavior == "circular":
        return CircularBehavior(cfg.speed, wheels=wheels)
    else:
        return StaticBehavior(wheels)
