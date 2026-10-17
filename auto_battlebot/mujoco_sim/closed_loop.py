"""Mr Stabs Mk2 on CPU MuJoCo, driven closed loop from body-axis stick commands in an arena.

The command chain is the robot's. Linear and angular stick fractions become the CRSF percents
the firmware reads: channel 0 carries linear and the firmware negates it (a = -linear), and the
transmitter reverses the angular channel (b = -angular). The firmware mixer and its heading hold
turn those into per-motor percents, the drivetrain delay holds them, and the command-tape shaping
(deadzone, throttle curve, left/right gain, pack voltage) turns them into wheel volts. Motor
polarity is +1: the sim has no ESC wiring to undo, so a positive per-motor command drives
forward.

The flip switch decides when the firmware treats the robot as inverted and negates the throttle
before the mixer (`FlipSwitch`). In the power-on DOWN position that follows the IMU's upside-down
reading, with its hysteresis, so a flipped robot keeps driving wedge-first and the heading hold
restarts on the flip.

Arena walls and wall blocks are static geoms, and hazards that move (the house bot) are mocap
cylinders, so wall contact is rigid-body contact rather than a position clamp.
"""

from __future__ import annotations

import json
import math
from collections import deque
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from auto_battlebot.mujoco_sim.actuator import (
    NOMINAL_PACK_V,
    MotorConstants,
    PlantParams,
    shape_command,
    side_gains,
)
from auto_battlebot.mujoco_sim.firmware import (
    FirmwareMixer,
    FlipSwitch,
    UpdownSensor,
    drive_inverted,
    gyro_from_yaw_rate,
    heading_from_yaw,
)
from auto_battlebot.mujoco_sim.mass_properties import MassProperties
from auto_battlebot.mujoco_sim.mjcf import (
    FLOOR_CONTYPE,
    ROBOT_CONTYPE,
    CollisionSet,
    build_mjcf,
)
from auto_battlebot.mujoco_sim.rollout import (
    InitialState,
    ModelIndex,
    initial_qpos_qvel,
    pose_from_qpos,
    rest_height,
)

WALL_HEIGHT_M = 0.3  # tall enough that a backflip lands inside the arena
WALL_THICKNESS_M = 0.05
BLOCK_HEIGHT_M = 0.3
PARKED_XY = (50.0, 50.0)  # moving blocks wait here until their first position arrives
ARENA_WALL_PREFIX = "arena_wall_"
BLOCK_PREFIX = "wall_block_"
MOVING_BLOCK_PREFIX = "moving_block_"


@dataclass(frozen=True)
class Disc:
    x: float
    y: float
    radius: float


@dataclass(frozen=True)
class MovingBlock:
    """A robot ours can hit, placed every tick by `ClosedLoopSim.set_moving_blocks`.

    A cylinder of `radius`, or with `box` set, a box of full size (length, width, height) in m with
    its length along the robot's heading: the shape the RL environment gives the opponent
    (warp_nav.OPP_SIZE_M).
    """

    radius: float
    box: tuple[float, float, float] | None = None


def load_fit_params(path: Path) -> PlantParams:
    """Best run of a fit_mujoco_plant.py fit.json: the lowest-loss restart across zero modes."""
    doc = json.loads(Path(path).read_text())
    runs = [r for r in doc["runs"] if math.isfinite(r["loss"])]
    if not runs:
        raise ValueError(f"{path}: no run with a finite loss")
    return PlantParams(**min(runs, key=lambda r: r["loss"])["best"])


def _add_static_geoms(
    spec: mujoco.MjSpec,
    arena: tuple[float, float] | None,
    blocks: Sequence[Disc],
    moving_blocks: Sequence[MovingBlock],
) -> None:
    # Zero friction, like the floor: MuJoCo takes the max of the pair, so the robot's own geom
    # friction governs every wall contact.
    common = {"contype": FLOOR_CONTYPE, "conaffinity": ROBOT_CONTYPE, "friction": [0.0, 0.0, 0.0]}
    if arena is not None:
        half_w, half_h = arena[0] / 2.0, arena[1] / 2.0
        t = WALL_THICKNESS_M / 2.0
        z = WALL_HEIGHT_M / 2.0
        # Inner faces sit on the arena boundary.
        walls = (
            ([half_w + t, 0.0, z], [t, half_h + 2 * t, z]),
            ([-half_w - t, 0.0, z], [t, half_h + 2 * t, z]),
            ([0.0, half_h + t, z], [half_w + 2 * t, t, z]),
            ([0.0, -half_h - t, z], [half_w + 2 * t, t, z]),
        )
        for i, (pos, size) in enumerate(walls):
            spec.worldbody.add_geom(
                name=f"{ARENA_WALL_PREFIX}{i}",
                type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=pos,
                size=size,
                **common,
            )
    for i, b in enumerate(blocks):
        spec.worldbody.add_geom(
            name=f"{BLOCK_PREFIX}{i}",
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            pos=[b.x, b.y, BLOCK_HEIGHT_M / 2.0],
            size=[b.radius, BLOCK_HEIGHT_M / 2.0, 0.0],
            **common,
        )
    for i, block in enumerate(moving_blocks):
        # Resting on the floor: set_moving_blocks moves x, y and yaw, never the height.
        height = BLOCK_HEIGHT_M if block.box is None else block.box[2]
        body = spec.worldbody.add_body(
            name=f"{MOVING_BLOCK_PREFIX}{i}",
            mocap=True,
            pos=[*PARKED_XY, height / 2.0],
        )
        if block.box is None:
            shape, size = mujoco.mjtGeom.mjGEOM_CYLINDER, [block.radius, height / 2.0, 0.0]
        else:
            shape, size = mujoco.mjtGeom.mjGEOM_BOX, [v / 2.0 for v in block.box]
        body.add_geom(name=f"{MOVING_BLOCK_PREFIX}{i}", type=shape, size=size, **common)


class ClosedLoopSim:
    """One robot, stepped a controller tick at a time. See the module docstring for the chain."""

    def __init__(
        self,
        mp: MassProperties,
        collision: CollisionSet | None,
        params: PlantParams,
        start: tuple[float, float, float],
        arena: tuple[float, float] | None = None,
        blocks: Sequence[Disc] = (),
        moving_blocks: Sequence[MovingBlock] = (),
        pack_voltage: float = NOMINAL_PACK_V,
        flip_switch: FlipSwitch = FlipSwitch.DOWN,
        timestep: float = 1e-3,
    ) -> None:
        self._params = params
        self._timestep = timestep
        self._pack_voltage = pack_voltage
        spec = mujoco.MjSpec.from_string(build_mjcf(mp, params, collision, timestep))
        _add_static_geoms(spec, arena, blocks, moving_blocks)
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        self._index = ModelIndex.of(self.model)
        self._motor = MotorConstants(gear_ratio=mp.gear_ratio, kt=mp.motor_kt)
        gain_l, gain_r = side_gains(params.lr_gain_ratio)
        self._side_gain = np.array([float(gain_l), float(gain_r)])
        self._deadzone = np.array([params.deadzone_left, params.deadzone_right])
        self._flip_switch = flip_switch
        self._updown = UpdownSensor()
        self._mixer = FirmwareMixer(auto_steer=flip_switch == FlipSwitch.DOWN)
        self.inverted = False  # the firmware's is_upside_down: the throttle is being negated
        delay_steps = max(0, round(params.delay_s / timestep))
        self._queue: deque[tuple[float, float]] = deque([(0.0, 0.0)] * delay_steps)

        names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
            for i in range(self.model.ngeom)
        ]
        self._wall_geoms = {i for i, n in enumerate(names) if n.startswith(ARENA_WALL_PREFIX)}
        self._block_geoms = {
            i
            for i, n in enumerate(names)
            if n.startswith(BLOCK_PREFIX) or n.startswith(MOVING_BLOCK_PREFIX)
        }
        self._mocap_ids = [
            int(self.model.body(f"{MOVING_BLOCK_PREFIX}{i}").mocapid[0])
            for i in range(len(moving_blocks))
        ]
        self.wall_contact = False
        self.block_contact = False

        x, y, yaw = start
        pitch = collision.rest_pitch_rad if collision is not None else 0.0
        zero = np.zeros(1)
        state = InitialState(
            x=np.array([x]),
            y=np.array([y]),
            yaw=np.array([yaw]),
            vx=zero,
            vy=zero,
            yaw_rate=zero,
            pitch=np.array([pitch]),
            wheel_left=zero,
            wheel_right=zero,
        )
        qpos, qvel = initial_qpos_qvel(
            self.model,
            self._index,
            state,
            rest_height(mp, collision, timestep),
            mp.gear_ratio,
        )
        self.data.qpos[:] = qpos[0]
        self.data.qvel[:] = qvel[0]
        mujoco.mj_forward(self.model, self.data)

    def set_moving_blocks(self, poses: Sequence[tuple[float, float, float]]) -> None:
        """(x, y, yaw) of each moving block, in the order they were given to the constructor."""
        if len(poses) != len(self._mocap_ids):
            raise ValueError(f"{len(poses)} moving blocks, model has {len(self._mocap_ids)}")
        for mocap, (x, y, yaw) in zip(self._mocap_ids, poses):
            self.data.mocap_pos[mocap, :2] = (x, y)
            self.data.mocap_quat[mocap] = (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))

    def _wheel_volts(self, left_percent: float, right_percent: float) -> np.ndarray:
        u = np.array([left_percent, right_percent]) / 100.0
        shaped = shape_command(u, self._deadzone, self._params.curve)
        volts = shaped * self._pack_voltage * self._side_gain
        if not self._params.brake_on_zero:
            emf = self._motor.back_emf_volts(self.data.qvel[self._index.wheel_dofs])
            volts = np.where(shaped == 0.0, emf, volts)
        return np.asarray(volts)

    def _record_contacts(self) -> None:
        for c in self.data.contact[: self.data.ncon]:
            pair = (int(c.geom1), int(c.geom2))
            if any(g in self._wall_geoms for g in pair):
                self.wall_contact = True
            if any(g in self._block_geoms for g in pair):
                self.block_contact = True

    def step(
        self,
        linear: float,
        angular: float,
        dt: float,
        on_substep: Callable[[float, float], bool] | None = None,
    ) -> None:
        """Hold the stick for dt. `on_substep(x, y)` returning True ends the tick early.

        wall_contact and block_contact report whether any substep of this tick touched one.
        """
        a_percent = -100.0 * float(np.clip(linear, -1.0, 1.0))
        b_percent = -100.0 * float(np.clip(angular, -1.0, 1.0))
        self.wall_contact = False
        self.block_contact = False
        for _ in range(max(1, round(dt / self._timestep))):
            heading = heading_from_yaw(self.pose()[2])
            self.inverted = drive_inverted(self._flip_switch, self._updown.update(self.up_z))
            left, right = self._mixer.step(
                -a_percent if self.inverted else a_percent,
                b_percent,
                heading,
                self._timestep,
                upside_down=self.inverted,
                yaw_rate_dps=gyro_from_yaw_rate(self.yaw_rate),
            )
            self._queue.append((left, right))
            self.data.ctrl[:] = self._wheel_volts(*self._queue.popleft())
            mujoco.mj_step(self.model, self.data)
            self._record_contacts()
            if on_substep is not None and on_substep(*self.pose()[:2]):
                break

    def pose(self) -> tuple[float, float, float]:
        p = pose_from_qpos(self.data.qpos[:7])
        return float(p["x"]), float(p["y"]), float(p["yaw"])

    @property
    def pitch(self) -> float:
        return float(pose_from_qpos(self.data.qpos[:7])["pitch"])

    @property
    def up_z(self) -> float:
        """The z component of the chassis +Z axis in the field frame, 1 - 2(x^2 + y^2) for a wxyz
        quaternion, read straight off the free joint: 1 upright, -1 on its back."""
        _, qx, qy, _ = self.data.qpos[3:7]
        return float(1.0 - 2.0 * (qx * qx + qy * qy))

    @property
    def upside_down(self) -> bool:
        """Chassis +Z points below the horizon: the robot has gone past 90 deg of roll or pitch."""
        return self.up_z < 0.0

    @property
    def forward_speed(self) -> float:
        yaw = self.pose()[2]
        return float(self.data.qvel[0] * math.cos(yaw) + self.data.qvel[1] * math.sin(yaw))

    @property
    def yaw_rate(self) -> float:
        """Field-frame yaw rate. The free joint's angular velocity is in the body frame."""
        rot = np.empty(9)
        mujoco.mju_quat2Mat(rot, self.data.qpos[3:7])
        return float((rot.reshape(3, 3) @ self.data.qvel[3:6])[2])
