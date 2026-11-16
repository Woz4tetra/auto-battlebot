"""What the sim server needs from a plant, and the hazard bookkeeping every plant shares."""

from __future__ import annotations

import math
from typing import Protocol

import numpy as np
from config.kinematic import ObstacleConfig

Pose = tuple[float, float, float]  # x, y, yaw in the field frame


class PlantInterface(Protocol):
    """Our robot's drivetrain, as the server and viewer use it."""

    @property
    def v_mps(self) -> float: ...

    @property
    def w_radps(self) -> float: ...

    @property
    def fell_in(self) -> bool: ...

    @property
    def upside_down(self) -> bool: ...

    @property
    def footprint_center_offset(self) -> tuple[float, float]:
        """Centre of the robot's footprint in its body frame (x forward, y left), in metres from
        the pose. Hazard clearance is scored from there, as the C++ side measures it
        (RobotDescription::footprint_center_offset), and the viewer centres the sprite there."""
        ...

    @property
    def wall_hits(self) -> int: ...

    @property
    def block_hits(self) -> int: ...

    @property
    def min_hazard_clearance(self) -> float: ...

    def set_moving_blocks(self, blocks: list[tuple[float, float, float, float]]) -> None:
        """(x, y, yaw, radius) of each opponent that is also an obstacle, refreshed every tick."""

    def step(self, linear_cmd: float, angular_cmd: float, dt: float) -> None: ...

    def pose(self) -> Pose: ...

    def collision_outlines(self) -> list[np.ndarray]:
        """Top-down outline of each shape our robot collides with the moving blocks through."""
        ...


def footprint_center(pose: Pose, offset: tuple[float, float]) -> tuple[float, float]:
    """Field-frame footprint centre of a robot at `pose`. Mirrors hazard_geometry.hpp."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * offset[0] - s * offset[1], y + s * offset[0] + c * offset[1]


def disc_outline(x: float, y: float, radius: float, points: int = 48) -> np.ndarray:
    theta = np.linspace(0.0, 2.0 * math.pi, points, endpoint=False)
    return np.stack([x + radius * np.cos(theta), y + radius * np.sin(theta)], axis=1)


class HazardMonitor:
    """Hole fall-in and closest approach to any hazard, from the chassis centre."""

    def __init__(self, obstacles: list[ObstacleConfig], robot_radius: float) -> None:
        self._robot_radius = robot_radius
        self._static_blocks = [
            (o.center[0], o.center[1], o.radius + robot_radius)
            for o in obstacles
            if o.kind == "wall_block"
        ]
        self._holes = [(o.center[0], o.center[1], o.radius) for o in obstacles if o.kind == "hole"]
        self._moving_blocks: list[tuple[float, float, float]] = []
        self.fell_in = False
        self.min_clearance = float("inf")

    def set_moving_blocks(self, blocks: list[tuple[float, float, float, float]]) -> None:
        self._moving_blocks = [(x, y, r + self._robot_radius) for x, y, _, r in blocks]

    def blocks(self) -> list[tuple[float, float, float]]:
        """Static and moving blocks, grown by the robot radius."""
        return self._static_blocks + self._moving_blocks

    def record(self, x: float, y: float) -> None:
        for hx, hy, hr in self._holes:
            gap = math.hypot(x - hx, y - hy) - hr
            self.min_clearance = min(self.min_clearance, gap)
            if gap < 0.0:
                self.fell_in = True
        for bx, by, br in self.blocks():
            self.min_clearance = min(self.min_clearance, math.hypot(x - bx, y - by) - br)
