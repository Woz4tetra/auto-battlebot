"""Mr Stabs Mk2 rigid-body plant (`closed_loop`) behind the sim's plant interface."""

from __future__ import annotations

import math
from pathlib import Path

import cv2
import mujoco
import numpy as np

from auto_battlebot.kinematic_sim.config.kinematic import (
    ObstacleConfig,
    OpponentConfig,
    PlantConfig,
)
from auto_battlebot.kinematic_sim.plants.base import HazardMonitor, Pose, footprint_center
from auto_battlebot.mujoco_sim import mass_properties
from auto_battlebot.mujoco_sim.actuator import PlantParams
from auto_battlebot.mujoco_sim.closed_loop import (
    ARENA_WALL_PREFIX,
    BLOCK_PREFIX,
    BoxSize,
    ClosedLoopSim,
    Disc,
    MovingBlock,
    load_fit_params,
)
from auto_battlebot.mujoco_sim.mjcf import CollisionSet

REPO_ROOT = Path(__file__).resolve().parents[3]

FOOTPRINT_SEGMENTS = 32


def _geom_points(model: mujoco.MjModel, geom: int) -> np.ndarray | None:
    """Points in the geom's own frame whose convex hull is the shape, or None for unknown types."""
    kind = model.geom_type[geom]
    size = np.asarray(model.geom_size[geom], dtype=float)
    theta = np.linspace(0.0, 2.0 * math.pi, FOOTPRINT_SEGMENTS, endpoint=False)
    ring = np.stack([np.cos(theta), np.sin(theta)], axis=1)
    if kind == mujoco.mjtGeom.mjGEOM_MESH:
        mesh = model.geom_dataid[geom]
        start = model.mesh_vertadr[mesh]
        return np.asarray(model.mesh_vert[start : start + model.mesh_vertnum[mesh]], dtype=float)
    if kind == mujoco.mjtGeom.mjGEOM_BOX:
        signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        corners: np.ndarray = signs * size[:3]
        return corners
    if kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
        rim = ring * size[0]
        return np.concatenate(
            [np.column_stack([rim, np.full(len(rim), z)]) for z in (-size[1], size[1])]
        )
    if kind == mujoco.mjtGeom.mjGEOM_SPHERE:
        zero = np.zeros(len(ring))
        circles = (
            np.column_stack([ring, zero]),
            np.column_stack([ring[:, 0], zero, ring[:, 1]]),
            np.column_stack([zero, ring]),
        )
        sphere: np.ndarray = np.concatenate(circles) * size[0]
        return sphere
    return None


class _Footprints:
    """Top-down outline of every contact geom except floor, walls and static blocks."""

    def __init__(self, model: mujoco.MjModel) -> None:
        self._geoms: list[tuple[int, np.ndarray]] = []
        self._robot_geoms: set[int] = set()
        for geom in range(model.ngeom):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom) or ""
            if model.geom_contype[geom] == 0 and model.geom_conaffinity[geom] == 0:
                continue
            if name.startswith((ARENA_WALL_PREFIX, BLOCK_PREFIX)):
                continue
            points = _geom_points(model, geom)
            if points is not None:
                self._geoms.append((geom, points))
                body = model.geom_bodyid[geom]
                if body != 0 and model.body_mocapid[body] < 0:
                    self._robot_geoms.add(geom)

    def outlines(self, data: mujoco.MjData) -> list[np.ndarray]:
        return [self._outline(data, geom, points) for geom, points in self._geoms]

    def robot_footprint_center(
        self, data: mujoco.MjData, pose: tuple[float, float, float]
    ) -> tuple[float, float]:
        """Centre of our robot's footprint in its body frame (x forward, y left), from the pose."""
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        relative = np.concatenate(
            [
                self._outline(data, geom, points) - (x, y)
                for geom, points in self._geoms
                if geom in self._robot_geoms
            ]
        )
        body = relative @ np.array([[c, -s], [s, c]])
        centre = (body.min(axis=0) + body.max(axis=0)) / 2.0
        return float(centre[0]), float(centre[1])

    @staticmethod
    def _outline(data: mujoco.MjData, geom: int, points: np.ndarray) -> np.ndarray:
        world = data.geom_xpos[geom] + points @ data.geom_xmat[geom].reshape(3, 3).T
        hull = cv2.convexHull(world[:, :2].astype(np.float32))
        return np.asarray(hull, dtype=float).reshape(-1, 2)


def _moving_block(opponent: OpponentConfig) -> MovingBlock:
    if not opponent.collision_box:
        return MovingBlock(opponent.hazard_radius)
    if len(opponent.collision_box) != 3:
        raise ValueError(
            f"[[opponents]] collision_box is [length, width, height] in m, "
            f"got {opponent.collision_box}"
        )
    length, width, height = opponent.collision_box
    return MovingBlock(opponent.hazard_radius, box=BoxSize(length, width, height))


class MujocoPlant:
    def __init__(
        self,
        cfg: PlantConfig,
        arena_w: float,
        arena_h: float,
        obstacles: list[ObstacleConfig] | None = None,
        moving_blocks: list[OpponentConfig] | None = None,
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
            moving_blocks=[_moving_block(o) for o in moving_blocks or []],
            pack_voltage=mcfg.pack_voltage,
            flip_switch=mcfg.flip_switch,
            timestep=mcfg.timestep,
        )
        self._hazards = HazardMonitor(obstacles, cfg.radius)
        self._footprints = _Footprints(self._sim.model)
        self._footprint_center_offset = self._footprints.robot_footprint_center(
            self._sim.data, self.pose()
        )
        self.wall_hits = 0
        self.block_hits = 0

    @property
    def v_mps(self) -> float:
        return 0.0 if self.fell_in else self._sim.forward_speed

    @property
    def w_radps(self) -> float:
        return 0.0 if self.fell_in else self._sim.yaw_rate

    @property
    def model(self) -> mujoco.MjModel:
        return self._sim.model

    @property
    def data(self) -> mujoco.MjData:
        return self._sim.data

    @property
    def pitch(self) -> float:
        """Chassis pitch, rad. At rest it sits on the skid, about 11 deg nose-down."""
        return self._sim.pitch

    @property
    def upside_down(self) -> bool:
        return self._sim.upside_down

    @property
    def footprint_center_offset(self) -> tuple[float, float]:
        return self._footprint_center_offset

    @property
    def fell_in(self) -> bool:
        return self._hazards.fell_in

    @property
    def min_hazard_clearance(self) -> float:
        return self._hazards.min_clearance

    def set_moving_blocks(self, blocks: list[tuple[float, float, float, float]]) -> None:
        self._hazards.set_moving_blocks(blocks)
        self._sim.set_moving_blocks([(x, y, yaw) for x, y, yaw, _ in blocks])

    def _on_substep(self, x: float, y: float) -> bool:
        yaw = self._sim.pose()[2]
        self._hazards.record(*footprint_center(Pose(x, y, yaw), self._footprint_center_offset))
        return self._hazards.fell_in

    def step(self, linear_cmd: float, angular_cmd: float, dt: float) -> None:
        if self.fell_in:
            return
        self._sim.step(linear_cmd, angular_cmd, dt, on_substep=self._on_substep)
        self.wall_hits += int(self._sim.wall_contact)
        self.block_hits += int(self._sim.block_contact)

    def pose(self) -> Pose:
        return Pose(*self._sim.pose())

    def collision_outlines(self) -> list[np.ndarray]:
        return self._footprints.outlines(self._sim.data)
