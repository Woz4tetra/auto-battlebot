"""Mr Stabs Mk2 rigid-body plant (auto_battlebot.mujoco_sim.closed_loop) behind the sim's
plant interface.

Walls and wall blocks are real contact geometry here, so the arena is not shrunk by the robot
radius the way the kinematic clamp shrinks it. Holes stay a centre-crosses-the-lip test, and
once the robot falls in it stops stepping. The hit counters count ticks with any contact, which
matches the kinematic plant's one-hit-per-tick behaviour while it pushes into a wall.
"""

from __future__ import annotations

import math
from pathlib import Path

from config.kinematic import ObstacleConfig, PlantConfig

from auto_battlebot.mujoco_sim import mass_properties
from auto_battlebot.mujoco_sim.actuator import PlantParams
from auto_battlebot.mujoco_sim.closed_loop import ClosedLoopSim, Disc, load_fit_params
from auto_battlebot.mujoco_sim.mjcf import CollisionSet
from plants.base import HazardMonitor, Pose

REPO_ROOT = Path(__file__).resolve().parents[2]


class MujocoPlant:
    def __init__(
        self,
        cfg: PlantConfig,
        arena_w: float,
        arena_h: float,
        obstacles: list[ObstacleConfig] | None = None,
        moving_block_radii: list[float] | None = None,
    ) -> None:
        mcfg = cfg.mujoco
        if mcfg.fit_file:
            params = load_fit_params(REPO_ROOT / mcfg.fit_file)
        else:
            print("MujocoPlant: no [our_robot.mujoco] fit_file, running unfit PlantParams defaults")
            params = PlantParams()
        obstacles = obstacles or []
        self._sim = ClosedLoopSim(
            mass_properties.load(),
            CollisionSet.load(),
            params,
            start=(cfg.start_pos[0], cfg.start_pos[1], math.radians(cfg.start_yaw_deg)),
            arena=(arena_w, arena_h),
            blocks=[
                Disc(o.center[0], o.center[1], o.radius)
                for o in obstacles
                if o.kind == "wall_block"
            ],
            moving_block_radii=moving_block_radii or [],
            pack_voltage=mcfg.pack_voltage,
            auto_steer=mcfg.auto_steer,
            timestep=mcfg.timestep,
        )
        self._hazards = HazardMonitor(obstacles, cfg.radius)
        self.wall_hits = 0
        self.block_hits = 0

    @property
    def v(self) -> float:
        return 0.0 if self.fell_in else self._sim.forward_speed

    @property
    def w(self) -> float:
        return 0.0 if self.fell_in else self._sim.yaw_rate

    @property
    def fell_in(self) -> bool:
        return self._hazards.fell_in

    @property
    def min_hazard_clearance(self) -> float:
        return self._hazards.min_clearance

    def set_moving_blocks(self, blocks: list[tuple[float, float, float]]) -> None:
        self._hazards.set_moving_blocks(blocks)
        self._sim.set_moving_blocks([(x, y) for x, y, _ in blocks])

    def _on_substep(self, x: float, y: float) -> bool:
        self._hazards.record(x, y)
        return self._hazards.fell_in

    def step(self, linear_cmd: float, angular_cmd: float, dt: float) -> None:
        if self.fell_in:
            return
        self._sim.step(linear_cmd, angular_cmd, dt, on_substep=self._on_substep)
        self.wall_hits += int(self._sim.wall_contact)
        self.block_hits += int(self._sim.block_contact)

    def pose(self) -> Pose:
        return self._sim.pose()
