"""Opponent behaviour strategies for the simulation."""

from __future__ import annotations

import abc
import math
import random
from typing import TYPE_CHECKING

import numpy as np
import numpy.typing as npt

from config.robot import RobotConfig
from sim_types.scene_handles import MocapHandle, RobotHandle

if TYPE_CHECKING:
    import mujoco


def _get_pos(
    data: mujoco.MjData, handle: RobotHandle | MocapHandle
) -> npt.NDArray[np.float64]:
    if isinstance(handle, MocapHandle):
        return data.mocap_pos[handle.mocap_id][:2].copy()
    return data.xpos[handle.body_id][:2].copy()


def _apply_velocity(
    data: mujoco.MjData,
    handle: RobotHandle | MocapHandle,
    vx: float,
    vy: float,
    dt: float,
) -> None:
    if isinstance(handle, MocapHandle):
        data.mocap_pos[handle.mocap_id][0] += vx * dt
        data.mocap_pos[handle.mocap_id][1] += vy * dt
        return
    w = handle.wheel_drive
    speed = math.hypot(vx, vy)
    v_wheel = speed / w.wheel_radius
    for aid in w.left_act_ids:
        data.ctrl[aid] = v_wheel
    for aid in w.right_act_ids:
        data.ctrl[aid] = v_wheel


def _stop(data: mujoco.MjData, handle: RobotHandle | MocapHandle) -> None:
    if isinstance(handle, MocapHandle):
        return
    for aid in handle.wheel_drive.all_act_ids:
        data.ctrl[aid] = 0.0


class OpponentBehavior(abc.ABC):
    def __init__(self, handle: RobotHandle | MocapHandle) -> None:
        self._handle = handle

    @abc.abstractmethod
    def step(self, data: mujoco.MjData, dt: float) -> None: ...


class StaticBehavior(OpponentBehavior):
    def step(self, data: mujoco.MjData, dt: float) -> None:
        _stop(data, self._handle)


class RandomWalkBehavior(OpponentBehavior):
    def __init__(
        self,
        handle: RobotHandle | MocapHandle,
        speed: float,
        arena_half_w: float,
        arena_half_h: float,
    ) -> None:
        super().__init__(handle)
        self.speed = speed
        self.arena_half_w = arena_half_w
        self.arena_half_h = arena_half_h
        self.target: npt.NDArray[np.float64] = self._random_target()

    def _random_target(self) -> npt.NDArray[np.float64]:
        x = random.uniform(-self.arena_half_w * 0.8, self.arena_half_w * 0.8)
        y = random.uniform(-self.arena_half_h * 0.8, self.arena_half_h * 0.8)
        return np.array([x, y])

    def step(self, data: mujoco.MjData, dt: float) -> None:
        current = _get_pos(data, self._handle)
        diff = self.target - current
        dist = float(np.linalg.norm(diff))
        if dist < 0.05:
            self.target = self._random_target()
            _stop(data, self._handle)
            return
        direction = diff / dist
        vel = direction * self.speed
        _apply_velocity(data, self._handle, float(vel[0]), float(vel[1]), dt)


class CircularBehavior(OpponentBehavior):
    def __init__(
        self,
        handle: RobotHandle | MocapHandle,
        speed: float,
        radius: float = 0.5,
        center: tuple[float, float] = (0.0, 0.0),
    ) -> None:
        super().__init__(handle)
        self.speed = speed
        self.radius = radius
        self.center = np.array(center)
        self.angle = 0.0

    def step(self, data: mujoco.MjData, dt: float) -> None:
        self.angle += (self.speed / max(self.radius, 0.01)) * dt
        vel_x = -self.speed * math.sin(self.angle)
        vel_y = self.speed * math.cos(self.angle)
        _apply_velocity(data, self._handle, vel_x, vel_y, dt)


def make_opponent_behavior(
    cfg: RobotConfig,
    arena_half_w: float,
    arena_half_h: float,
    handle: RobotHandle | MocapHandle,
) -> OpponentBehavior:
    if cfg.behavior == "random_walk":
        return RandomWalkBehavior(handle, cfg.speed, arena_half_w, arena_half_h)
    elif cfg.behavior == "circular":
        return CircularBehavior(handle, cfg.speed)
    else:
        return StaticBehavior(handle)
