"""An opponent driven by an RL navigation bundle on its own Mr Stabs Mk2 plant.

`[[opponents]] behavior = "policy"` runs `policy_bundle` through `BundlePolicy` the way
`RlPolicyNavigation` runs it on our robot, so a config can put the same policy on both robots.
The opponent steps the MuJoCo plant our robot uses, with the fit and pack voltage from
`[our_robot.mujoco]` and the same `[latency] command_ms`, and attacks our robot.

Each plant carries the other robot as a moving box (`collision_box`), placed at its footprint
centre every tick, which is the shape the RL environment gives its opponent. Two boxes pushed by
two plants is not one contact solve: each robot feels the other as an immovable mocap body.

The opponent sees ground truth: our footprint centre, its finite-difference velocity, no sigmas,
ages or stale flags. That is the observation the policy trained on with sensing off. Our robot
still sees the opponent through the sensing model and the C++ filter.

`pose()` is the opponent's footprint centre, not its axle, so the C++ side, the viewer and our
plant's contact box all see the robot where its footprint is, as the training opponent's pose is
its box centre.
"""

from __future__ import annotations

import math
from collections import deque
from pathlib import Path

import torch
from config.kinematic import ObstacleConfig, OpponentConfig, PlantConfig, PlantType
from plants.base import Pose, footprint_center
from plants.mujoco_plant import MujocoPlant

from auto_battlebot.rl import obs_spec
from auto_battlebot.rl.export.bundle_policy import BundlePolicy, Hazard, PolicyView
from auto_battlebot.rl.torch_plant import saturate

REPO_ROOT = Path(__file__).resolve().parents[1]

# The RL environment's policy step. The policy holds its action between steps at a finer [sim] dt.
POLICY_PERIOD_S = 0.02

HAZARD_KINDS = {"hole": obs_spec.HOLE, "wall_block": obs_spec.WALL_BLOCK}

MR_STABS_MK2_SIZE_M = (0.137, 0.16, 0.0703)  # length, width, height
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
        """`us` is our robot as a contact box; `others` the remaining opponents with a
        hazard_radius, in the order `observe` hands their poses over."""
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
        """Our robot's footprint-centre pose and yaw rate, and the other opponents' poses, this
        tick. Velocities come from the change since the last call."""

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
            # The transmitter's drive processor sits between the policy and the robot, as
            # SimTransmitter puts it in front of our plant and the RL environment trains with it.
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
            Hazard(p[0], p[1], r, obs_spec.TRACKED, v[0], v[1])
            for p, r, v in zip(self._others, self._other_radius, self._others_velocity)
        ]
        return PolicyView(
            our_pose=self._plant.pose(),
            our_speed=self._plant.v,
            our_yaw_rate=self._plant.w,
            target_pose=self._target,
            target_velocity=self._target_velocity,
            target_yaw_rate=self._target_yaw_rate,
            target_half_diagonal=self._us_half_diagonal,
            field_half=self._field_half,
            hazards=self._static + tracked,
        )

    def pose(self) -> Pose:
        """Footprint centre and heading."""
        x, y, yaw = self._plant.pose()
        cx, cy = footprint_center((x, y, yaw), self._plant.footprint_center_offset)
        return cx, cy, yaw


def our_robot_box() -> OpponentConfig:
    """Our robot as the contact box the policy opponent's plant collides with, centred on its
    footprint: Mr Stabs Mk2's [robot_sizes] entry in config/_common.toml."""
    return OpponentConfig(
        hazard_radius=MR_STABS_MK2_HALF_DIAGONAL_M, collision_box=list(MR_STABS_MK2_SIZE_M)
    )
