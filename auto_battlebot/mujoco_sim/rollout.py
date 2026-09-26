"""Batched open-loop rollouts of the Mr Stabs Mk2 MJCF on MuJoCo Warp, with a CPU reference.

One world per (candidate, window) pair. Every parameter the fit moves either lives in a Model
field that MuJoCo Warp batches per world (friction, solref, solimp, joint damping and
frictionloss, rotor inertia, actuator gain, bias and force limit) or is folded into the voltage
tape on the host (delay, deadzone, throttle curve, left/right ratio, pack voltage). The
zero-command mode is a per-world flag the control kernel reads each step.

A step is: write ctrl from the tape (the back-EMF voltage on coast steps), mjw.step, record the
chassis pose every `record_every` steps. The whole step is captured once as a CUDA graph and
replayed, which is 3 to 5x faster than launching it from Python.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import mujoco
import numpy as np

from auto_battlebot.mujoco_sim.actuator import MotorConstants, PlantParams
from auto_battlebot.mujoco_sim.mass_properties import MassProperties
from auto_battlebot.mujoco_sim.mjcf import (
    CHASSIS_GEOM_PREFIX,
    ROTOR_BODIES,
    SKID_GEOM,
    WHEEL_GEOMS,
    WHEEL_JOINTS,
    CollisionSet,
    build_mjcf,
    rotor_diaginertia,
    skid_friction,
    solimp,
    solref,
    wheel_friction,
)

BATCHED_FIELDS = (
    "geom_friction",
    "geom_solref",
    "geom_solimp",
    "dof_damping",
    "dof_frictionloss",
    "body_inertia",
    "actuator_gainprm",
    "actuator_biasprm",
    "actuator_forcerange",
)
# Contacts and constraint rows per world. The hull pieces, skid and both treads rarely give more
# than 8 contacts; the headroom covers a robot on its side.
NCONMAX = 24
NJMAX = 160


@dataclass
class ModelIndex:
    """Where each fitted quantity sits in the compiled model."""

    wheel_geoms: np.ndarray
    skid_geoms: np.ndarray
    wheel_dofs: np.ndarray
    wheel_qpos: np.ndarray
    rotor_bodies: np.ndarray
    rotor_dofs: np.ndarray
    rotor_qpos: np.ndarray

    @classmethod
    def of(cls, model: mujoco.MjModel) -> ModelIndex:
        def geom_ids(pred: object) -> np.ndarray:
            names = [
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
                for i in range(model.ngeom)
            ]
            return np.array([i for i, n in enumerate(names) if pred(n)], dtype=int)  # type: ignore[operator]

        return cls(
            wheel_geoms=geom_ids(lambda n: n in WHEEL_GEOMS),
            skid_geoms=geom_ids(lambda n: n == SKID_GEOM or n.startswith(CHASSIS_GEOM_PREFIX)),
            wheel_dofs=np.array([model.joint(n).dofadr[0] for n in WHEEL_JOINTS]),
            wheel_qpos=np.array([model.joint(n).qposadr[0] for n in WHEEL_JOINTS]),
            rotor_bodies=np.array([model.body(n).id for n in ROTOR_BODIES]),
            rotor_dofs=np.array([model.joint(n).dofadr[0] for n in ROTOR_BODIES]),
            rotor_qpos=np.array([model.joint(n).qposadr[0] for n in ROTOR_BODIES]),
        )


@dataclass
class ParamArrays:
    """Per-world values of the batched Model fields, as host arrays."""

    geom_friction: np.ndarray  # (W, ngeom, 3)
    geom_solref: np.ndarray  # (W, ngeom, 2)
    geom_solimp: np.ndarray  # (W, ngeom, 5)
    dof_damping: np.ndarray  # (W, nv)
    dof_frictionloss: np.ndarray  # (W, nv)
    body_inertia: np.ndarray  # (W, nbody, 3)
    actuator_gainprm: np.ndarray  # (W, nu, 10)
    actuator_biasprm: np.ndarray  # (W, nu, 10)
    actuator_forcerange: np.ndarray  # (W, nu, 2)
    coast: np.ndarray  # (W,) 1.0 where a zero command freewheels


def param_arrays(
    model: mujoco.MjModel,
    index: ModelIndex,
    mp: MassProperties,
    params: list[PlantParams],
) -> ParamArrays:
    n = len(params)
    motor = MotorConstants(gear_ratio=mp.gear_ratio, kt=mp.motor_kt)
    out = ParamArrays(
        geom_friction=np.repeat(model.geom_friction[None], n, axis=0).copy(),
        geom_solref=np.repeat(model.geom_solref[None], n, axis=0).copy(),
        geom_solimp=np.repeat(model.geom_solimp[None], n, axis=0).copy(),
        dof_damping=np.repeat(model.dof_damping[None], n, axis=0).copy(),
        dof_frictionloss=np.repeat(model.dof_frictionloss[None], n, axis=0).copy(),
        body_inertia=np.repeat(model.body_inertia[None], n, axis=0).copy(),
        actuator_gainprm=np.repeat(model.actuator_gainprm[None], n, axis=0).copy(),
        actuator_biasprm=np.repeat(model.actuator_biasprm[None], n, axis=0).copy(),
        actuator_forcerange=np.repeat(model.actuator_forcerange[None], n, axis=0).copy(),
        coast=np.zeros(n),
    )
    robot_geoms = np.concatenate([index.wheel_geoms, index.skid_geoms])
    for w, p in enumerate(params):
        out.geom_friction[w, index.wheel_geoms] = wheel_friction(p)
        out.geom_friction[w, index.skid_geoms] = skid_friction(p)
        out.geom_solref[w, robot_geoms] = solref(p)
        out.geom_solimp[w, robot_geoms] = solimp(p)
        out.dof_damping[w, index.wheel_dofs] = p.joint_damping
        out.dof_frictionloss[w, index.wheel_dofs] = p.joint_frictionloss
        axial = mp.rotor_inertia * p.armature_scale
        out.body_inertia[w, index.rotor_bodies] = rotor_diaginertia(axial)
        out.actuator_gainprm[w, :, 0] = motor.gain(p)
        out.actuator_biasprm[w, :, 2] = motor.bias(p)
        limit = motor.force_limit(p)
        out.actuator_forcerange[w] = (-limit, limit)
        out.coast[w] = 0.0 if p.brake_on_zero else 1.0
    return out


@dataclass
class InitialState:
    """Per-world start: planar pose and velocity in the field frame, plus pitch."""

    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    vx: np.ndarray  # field frame
    vy: np.ndarray
    yaw_rate: np.ndarray
    pitch: np.ndarray  # nose-down positive
    wheel_left: np.ndarray  # rad/s
    wheel_right: np.ndarray


def initial_qpos_qvel(
    model: mujoco.MjModel,
    index: ModelIndex,
    state: InitialState,
    height: float,
    gear_ratio: float,
) -> tuple[np.ndarray, np.ndarray]:
    n = len(state.x)
    qpos = np.repeat(model.qpos0[None], n, axis=0).astype(float)
    qvel = np.zeros((n, model.nv))
    qpos[:, 0] = state.x
    qpos[:, 1] = state.y
    qpos[:, 2] = height
    half_yaw, half_pitch = state.yaw / 2, state.pitch / 2
    # q = q_yaw * q_pitch (intrinsic z then y)
    qpos[:, 3] = np.cos(half_yaw) * np.cos(half_pitch)
    qpos[:, 4] = -np.sin(half_yaw) * np.sin(half_pitch)
    qpos[:, 5] = np.cos(half_yaw) * np.sin(half_pitch)
    qpos[:, 6] = np.sin(half_yaw) * np.cos(half_pitch)
    qvel[:, 0] = state.vx
    qvel[:, 1] = state.vy
    # Free-joint angular velocity is in the body frame; for a pitched body a pure field yaw
    # rate has components on body x and z.
    qvel[:, 3] = -np.sin(state.pitch) * state.yaw_rate
    qvel[:, 5] = np.cos(state.pitch) * state.yaw_rate
    wheels = np.stack([state.wheel_left, state.wheel_right], axis=1)
    qvel[:, index.wheel_dofs] = wheels
    qvel[:, index.rotor_dofs] = gear_ratio * wheels
    return qpos, qvel


def pose_from_qpos(qpos7: np.ndarray) -> dict[str, np.ndarray]:
    """x, y, yaw, pitch, roll from free-joint position + wxyz quaternion, any leading shape."""
    w, x, y, z = (qpos7[..., i] for i in range(3, 7))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return {
        "x": qpos7[..., 0],
        "y": qpos7[..., 1],
        "yaw": yaw,
        "pitch": pitch,
        "roll": roll,
        "z": qpos7[..., 2],
    }


def rest_height(mp: MassProperties, collision: CollisionSet | None, timestep: float) -> float:
    """Chassis origin height after settling at rest; windows start from it."""
    model = mujoco.MjModel.from_xml_string(build_mjcf(mp, PlantParams(), collision, timestep))
    data = mujoco.MjData(model)
    for _ in range(int(2.0 / timestep)):
        mujoco.mj_step(model, data)
    return float(data.qpos[2])


def cpu_rollout(
    mp: MassProperties,
    collision: CollisionSet | None,
    params: PlantParams,
    state: InitialState,
    volts: np.ndarray,
    zero: np.ndarray,
    timestep: float,
    record_every: int,
    height: float,
) -> np.ndarray:
    """Reference rollout of one world on CPU MuJoCo. volts/zero: (T, 2). Returns (S, 7) qpos."""
    model = mujoco.MjModel.from_xml_string(build_mjcf(mp, params, collision, timestep))
    data = mujoco.MjData(model)
    index = ModelIndex.of(model)
    qpos, qvel = initial_qpos_qvel(model, index, state, height, mp.gear_ratio)
    data.qpos[:] = qpos[0]
    data.qvel[:] = qvel[0]
    motor = MotorConstants(gear_ratio=mp.gear_ratio, kt=mp.motor_kt)
    out = []
    for t in range(volts.shape[0]):
        if t % record_every == 0:
            out.append(data.qpos[:7].copy())
        ctrl = volts[t].copy()
        if not params.brake_on_zero:
            emf = motor.back_emf_volts(data.qvel[index.wheel_dofs])
            ctrl = np.where(zero[t] > 0.5, emf, ctrl)
        data.ctrl[:] = ctrl
        mujoco.mj_step(model, data)
    return np.asarray(out)


class WarpRollout:
    """A fixed-size batch of worlds on the GPU, reused across CMA-ES generations.

    Tapes are (W, T, 2) volts and a (W, T, 2) zero-command mask. `run` resets every world to its
    initial state, replays the captured step graph T times and returns (W, S, 7) chassis qpos
    sampled every `record_every` steps (S = ceil(T / record_every)).
    """

    def __init__(
        self,
        mp: MassProperties,
        collision: CollisionSet | None,
        nworld: int,
        steps: int,
        timestep: float = 1e-3,
        record_every: int = 5,
    ) -> None:
        import mujoco_warp as mjw
        import warp as wp

        self._wp = wp
        self._mjw = mjw
        self.mp = mp
        self.nworld = nworld
        self.steps = steps
        self.timestep = timestep
        self.record_every = record_every
        self.samples = (steps + record_every - 1) // record_every
        self.mjm = mujoco.MjModel.from_xml_string(
            build_mjcf(mp, PlantParams(), collision, timestep)
        )
        self.mjd = mujoco.MjData(self.mjm)
        mujoco.mj_forward(self.mjm, self.mjd)
        self.index = ModelIndex.of(self.mjm)
        self.height = rest_height(mp, collision, timestep)
        self.m = mjw.put_model(self.mjm, batch_sizes={f: nworld for f in BATCHED_FIELDS})
        self.d = mjw.put_data(self.mjm, self.mjd, nworld=nworld, nconmax=NCONMAX, njmax=NJMAX)
        self.tape_v = wp.zeros((nworld, steps, 2), dtype=float)
        self.tape_zero = wp.zeros((nworld, steps, 2), dtype=float)
        self.coast = wp.zeros(nworld, dtype=float)
        self.record = wp.zeros((nworld, self.samples, 7), dtype=float)
        self.step_counter = wp.zeros(1, dtype=int)
        self.wheel_dofs: Any = wp.array(self.index.wheel_dofs.astype(np.int32), dtype=int)
        self.emf_per_radps = float(mp.gear_ratio * mp.motor_kt)
        self._graph: Any = None

    def set_params(self, params: list[PlantParams]) -> None:
        if len(params) != self.nworld:
            raise ValueError(f"{len(params)} parameter sets for {self.nworld} worlds")
        arrays = param_arrays(self.mjm, self.index, self.mp, params)
        for name in BATCHED_FIELDS:
            getattr(self.m, name).assign(getattr(arrays, name).astype(np.float32))
        self.coast.assign(arrays.coast.astype(np.float32))

    def set_tapes(self, volts: np.ndarray, zero: np.ndarray) -> None:
        self.tape_v.assign(volts.astype(np.float32))
        self.tape_zero.assign(zero.astype(np.float32))

    def run(self, state: InitialState) -> np.ndarray:
        wp, mjw = self._wp, self._mjw
        mjw.reset_data(self.m, self.d)
        qpos, qvel = initial_qpos_qvel(self.mjm, self.index, state, self.height, self.mp.gear_ratio)
        self.d.qpos.assign(qpos.astype(np.float32))
        self.d.qvel.assign(qvel.astype(np.float32))
        self.step_counter.zero_()
        if self._graph is None:
            self._one_step()  # compile kernels outside the capture
            self.step_counter.zero_()
            with wp.ScopedCapture() as capture:
                self._one_step()
            self._graph = capture.graph
        for _ in range(self.steps):
            wp.capture_launch(self._graph)
        wp.synchronize()
        return np.asarray(self.record.numpy(), dtype=float)

    def _one_step(self) -> None:
        from auto_battlebot.mujoco_sim.warp_kernels import advance, control, record

        wp, mjw = self._wp, self._mjw
        wp.launch(
            control,
            dim=self.nworld,
            inputs=[
                self.step_counter,
                self.tape_v,
                self.tape_zero,
                self.coast,
                self.d.qvel,
                self.wheel_dofs,
                self.emf_per_radps,
                self.steps,
            ],
            outputs=[self.d.ctrl],
        )
        wp.launch(
            record,
            dim=self.nworld,
            inputs=[self.step_counter, self.d.qpos, self.record_every, self.samples],
            outputs=[self.record],
        )
        mjw.step(self.m, self.d)
        wp.launch(advance, dim=1, inputs=[self.step_counter])
