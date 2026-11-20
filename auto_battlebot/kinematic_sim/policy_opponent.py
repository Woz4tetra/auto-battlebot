"""An opponent driven by an RL navigation bundle on its own Mr Stabs Mk2 plant."""

from __future__ import annotations

import math
from collections import deque
from pathlib import Path

import torch

from auto_battlebot.kinematic_sim.config.kinematic import (
    ObstacleConfig,
    OpponentConfig,
    PlantConfig,
    PlantType,
)
from auto_battlebot.kinematic_sim.plants.base import Pose, footprint_center
from auto_battlebot.kinematic_sim.plants.mujoco_plant import MujocoPlant
from auto_battlebot.rl.export.bundle_policy import BundlePolicy, Hazard, HazardKind, PolicyView
from auto_battlebot.rl.export.bundle_policy import Pose as ViewPose
from auto_battlebot.rl.torch_plant import saturate

REPO_ROOT = Path(__file__).resolve().parents[2]

POLICY_PERIOD_S = 0.02

HAZARD_KINDS = {"hole": HazardKind.HOLE, "wall_block": HazardKind.WALL_BLOCK}

MR_STABS_MK2_SIZE_M = (0.137, 0.16, 0.0703)
MR_STABS_MK2_HALF_DIAGONAL_M = 0.5 * math.hypot(MR_STABS_MK2_SIZE_M[0], MR_STABS_MK2_SIZE_M[1])


class PolicyOpponent:
    def __init__(
        self,
        cfg: OpponentConfig,
        our_robot: PlantConfig,
        us: OpponentConfig,
        others: list[OpponentConfig],
        arena: tuple[float, float],
        obstacles: list[ObstacleConfig],
        latency_ticks: int,
        dt: float,
    ) -> None:
        """Build the opponent plant against our robot and other opponents."""
        plant_cfg = PlantConfig(
            plant=PlantType.MUJOCO,
            mujoco=our_robot.mujoco,
            start_pos=list(cfg.start_pos),
            start_yaw_deg=cfg.heading_deg,
            radius=our_robot.radius,
        )
        self._plant = MujocoPlant(plant_cfg, arena[0], arena[1], obstacles, [us, *others])
        self._policy = BundlePolicy(REPO_ROOT / cfg.policy_bundle)
        print(f"sim: policy opponent runs {self._policy.run_id}")
        self._us_half_diagonal = us.hazard_radius
        self._other_radius = [o.hazard_radius for o in others]
        self._field_half = (arena[0] / 2.0, arena[1] / 2.0)
        self._static = [
            Hazard(o.center[0], o.center[1], o.radius, HAZARD_KINDS[o.kind]) for o in obstacles
        ]
        self._commands: deque[tuple[float, float]] = deque(
            [(0.0, 0.0)] * latency_ticks, maxlen=latency_ticks + 1
        )
        self._period = max(1, round(POLICY_PERIOD_S / dt))
        self._ticks_until_step = 0
        self._action = (0.0, 0.0)
        self._dt = dt
        self._target: Pose | None = None
        self._target_velocity = (0.0, 0.0)
        self._target_yaw_rate = 0.0
        self._others: list[Pose] = []
        self._others_velocity: list[tuple[float, float]] = []

    def observe(self, us: Pose, us_yaw_rate: float, others: list[Pose]) -> None:
        """Hand over this tick's poses of our robot and the other opponents."""

        def velocity(new: Pose, old: Pose | None) -> tuple[float, float]:
            if old is None:
                return 0.0, 0.0
            return (new[0] - old[0]) / self._dt, (new[1] - old[1]) / self._dt

        self._target_velocity = velocity(us, self._target)
        self._target = us
        self._target_yaw_rate = us_yaw_rate
        old: list[Pose | None] = list(self._others) or [None] * len(others)
        self._others_velocity = [velocity(p, o) for p, o in zip(others, old)]
        self._others = others

    def step(self, dt: float) -> None:
        if self._target is None:
            raise RuntimeError("PolicyOpponent.step before observe")
        if self._ticks_until_step <= 0:
            action = torch.tensor([self._policy.act(self._view())])
            self._action = tuple(saturate(action)[0].tolist())
            self._ticks_until_step = self._period
        self._ticks_until_step -= 1
        self._commands.append(self._action)
        self._plant.set_moving_blocks(
            [(*self._target, self._us_half_diagonal)]
            + [(*p, r) for p, r in zip(self._others, self._other_radius)]
        )
        self._plant.step(*self._commands[0], dt)

    def _view(self) -> PolicyView:
        assert self._target is not None
        tracked = [
            Hazard(p[0], p[1], r, HazardKind.TRACKED, v[0], v[1])
            for p, r, v in zip(self._others, self._other_radius, self._others_velocity)
        ]
        return PolicyView(
            our_pose=ViewPose(*self._plant.pose()),
            our_speed_mps=self._plant.v_mps,
            our_yaw_rate_rad_s=self._plant.w_radps,
            target_pose=ViewPose(*self._target),
            target_velocity_mps=self._target_velocity,
            target_yaw_rate_rad_s=self._target_yaw_rate,
            target_half_diagonal_m=self._us_half_diagonal,
            field_half_m=self._field_half,
            hazards=self._static + tracked,
        )

    def pose(self) -> Pose:
        """Footprint centre and heading."""
        x, y, yaw = self._plant.pose()
        cx, cy = footprint_center(Pose(x, y, yaw), self._plant.footprint_center_offset)
        return Pose(cx, cy, yaw)


def our_robot_box() -> OpponentConfig:
    """Our robot as the contact box the policy opponent's plant collides with."""
    return OpponentConfig(
        hazard_radius=MR_STABS_MK2_HALF_DIAGONAL_M, collision_box=list(MR_STABS_MK2_SIZE_M)
    )
