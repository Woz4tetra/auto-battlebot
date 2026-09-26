"""Sanity checks on the Mr Stabs Mk2 MJCF, run on CPU MuJoCo before any fitting.

1. Nose lift under a slow torque ramp lands on the analytic threshold (momentum form).
2. Top speed at a set voltage (no friction) matches KV x V / N at the wheel.
3. Zero-command decel tau is reachable with a plausible winding resistance.

Plus the timestep study: halve dt until the flip threshold and the tau stop moving.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from auto_battlebot.mujoco_sim.actuator import MotorConstants, PlantParams
from auto_battlebot.mujoco_sim.mass_properties import MassProperties
from auto_battlebot.mujoco_sim.mjcf import (
    CHASSIS_GEOM_PREFIX,
    SKID_GEOM,
    WHEEL_GEOMS,
    WHEEL_JOINTS,
    CollisionSet,
    build_mjcf,
)

GRAVITY = 9.81


def pitch_of(quat_wxyz: np.ndarray) -> float:
    """Rotation about body y; positive is nose-down in FLU."""
    w, x, y, z = quat_wxyz
    return float(np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0)))


def make(
    mp: MassProperties, params: PlantParams, collision: CollisionSet | None, dt: float
) -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_string(build_mjcf(mp, params, collision, timestep=dt))
    return model, mujoco.MjData(model)


def settle(model: mujoco.MjModel, data: mujoco.MjData, seconds: float = 2.0) -> None:
    data.ctrl[:] = 0.0
    for _ in range(int(seconds / model.opt.timestep)):
        mujoco.mj_step(model, data)


def analytic_lift_accel(mp: MassProperties, rest_pitch: float, rotor: str = "momentum") -> float:
    """Forward acceleration (m/s^2) at which the skid unloads, from moments about the axle.

    With the robot at its rest pitch: the drive reaction on the chassis, F r plus the spin-up
    torque of whatever turns with the wheel, plus the chassis's own inertial load at its COM
    height, against gravity at the COM's lever arm.

    `rotor` picks the rotor's share of the spin-up torque:
      "momentum"   N J_r alpha. The rotor's angular momentum is J_r N omega_wheel, and the
                   gearbox housing reacts exactly its rate of change. This is the physics.
      "reflected"  N^2 J_r alpha, the plan's first estimate (0.29 g). N^2 J_r is the inertia
                   the motor pushes against at the wheel, not what the chassis reacts.
      "none"       no rotor at all.
    """
    c, s = np.cos(rest_pitch), np.sin(rest_pitch)
    x_c = mp.chassis.com[0] * c + mp.chassis.com[2] * s
    h_c = -mp.chassis.com[0] * s + mp.chassis.com[2] * c
    rotor_term = {
        "momentum": mp.gear_ratio * mp.rotor_inertia,
        "reflected": mp.reflected_inertia,
        "none": 0.0,
    }[rotor]
    j_wheel = mp.wheel_left.fullinertia[1] + rotor_term
    r = mp.wheel_radius
    denom = mp.total_mass * r + 2.0 * j_wheel / r + mp.chassis.mass * h_c
    return float(mp.chassis.mass * GRAVITY * x_c / denom)


def nose_contact_force(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    """Normal force on the chassis hull and skid, i.e. everything but the wheels."""
    total = 0.0
    wrench = np.zeros(6)
    for i in range(data.ncon):
        con = data.contact[i]
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) or "" for g in con.geom]
        if any(n == SKID_GEOM or n.startswith(CHASSIS_GEOM_PREFIX) for n in names):
            mujoco.mj_contactForce(model, data, i, wrench)
            total += float(wrench[0])
    return total


def simulated_lift_accel(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    torque_rate: float = 0.1,
    wheel_mu: float = 3.0,
    skid_mu: float = 0.01,
    lift_deg: float = 3.0,
) -> float:
    """Ramp wheel torque until the nose unloads; return the forward accel it took.

    The actuators are switched to pure torque (gain 1, no back-EMF) so the ramp is exact, and
    the wheels get `wheel_mu` so they don't spin out before the nose can lift. The analytic
    threshold assumes a frictionless nose, so the hull and skid get `skid_mu` near zero: with a
    realistic 0.3 the sliding skid stick-slips, rocks the chassis, and trips the lift early by an
    amount that depends on the timestep (0.64 g at 1 ms, 0.93 g at 0.25 ms against 1.20 g).
    The nose counts as lifted once the pitch sits `lift_deg` above rest for 20 ms; the
    acceleration is a line fit to the chassis speed over the 100 ms before that. The skid
    contact force can't mark it: the sliding skid chatters on and off the floor well below
    the threshold.
    """
    model.actuator_gainprm[:, 0] = 1.0
    model.actuator_biasprm[:, :] = 0.0
    model.actuator_forcerange[:, :] = [-10.0, 10.0]
    for i in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
        if name in WHEEL_GEOMS:
            model.geom_friction[i, 0] = wheel_mu
        elif name == SKID_GEOM or name.startswith(CHASSIS_GEOM_PREFIX):
            model.geom_friction[i, 0] = skid_mu
    settle(model, data)
    rest = pitch_of(data.qpos[3:7])
    dt = model.opt.timestep
    speeds: list[float] = []
    unloaded_steps = 0
    t = 0.0
    while t < 10.0:
        data.ctrl[:] = torque_rate * t
        mujoco.mj_step(model, data)
        speeds.append(float(data.qvel[0]))
        t += dt
        unloaded = np.degrees(rest - pitch_of(data.qpos[3:7])) > lift_deg
        unloaded_steps = unloaded_steps + 1 if unloaded else 0
        if unloaded_steps * dt >= 0.02:
            end = len(speeds) - unloaded_steps
            n = int(0.1 / dt)
            if end <= n:
                return float("nan")
            window = np.asarray(speeds[end - n : end])
            return float(np.polyfit(np.arange(n) * dt, window, 1)[0])
    return float("nan")


def simulated_top_speed(
    model: mujoco.MjModel, data: mujoco.MjData, volts: float, seconds: float = 4.0
) -> float:
    """Forward speed after driving straight at `volts`; joint friction and damping zeroed.

    The voltage ramps up over `seconds` and then holds for one more second: a step to full
    voltage from rest wheelies the robot over, as the real one does.
    """
    for name in WHEEL_JOINTS:
        dof = model.joint(name).dofadr[0]
        model.dof_frictionloss[dof] = 0.0
        model.dof_damping[dof] = 0.0
    settle(model, data)
    steps = int(seconds / model.opt.timestep)
    for i in range(steps + int(1.0 / model.opt.timestep)):
        data.ctrl[:] = volts * min(1.0, i / steps)
        mujoco.mj_step(model, data)
    return float(data.qvel[0])


def ideal_top_speed(mp: MassProperties, volts: float) -> float:
    motor = MotorConstants(gear_ratio=mp.gear_ratio, kt=mp.motor_kt)
    return motor.free_speed(volts) * mp.wheel_radius


def simulated_decel_tau(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    params: PlantParams,
    motor: MotorConstants,
    volts: float = 4.0,
) -> float:
    """Drive at `volts`, release to zero command, and time the drop to 1/e of the speed."""
    settle(model, data)
    data.ctrl[:] = volts
    for _ in range(int(1.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    v0 = float(data.qvel[0])
    t = 0.0
    dofs = [model.joint(n).dofadr[0] for n in WHEEL_JOINTS]
    while t < 3.0:
        if params.brake_on_zero:
            data.ctrl[:] = 0.0
        else:
            data.ctrl[:] = motor.back_emf_volts(data.qvel[dofs])
        mujoco.mj_step(model, data)
        t += model.opt.timestep
        if data.qvel[0] <= v0 / np.e:
            return float(t)
    return float("nan")


def resistance_for_tau(mp: MassProperties, params: PlantParams, target_tau: float) -> float:
    """Winding resistance that gives `target_tau` in brake mode, ignoring joint friction.

    Per wheel the back-EMF brake is eta N^2 Kt^2 / R against the inertia it decelerates: wheel,
    reflected rotor, and half the robot's mass at the wheel radius.
    """
    j_eff = (
        mp.reflected_inertia * params.armature_scale
        + mp.wheel_left.fullinertia[1]
        + 0.5 * mp.total_mass * mp.wheel_radius**2
    )
    return float(target_tau * params.efficiency * mp.gear_ratio**2 * mp.motor_kt**2 / j_eff)


@dataclass
class CheckReport:
    dt: float
    rest_pitch_deg: float
    lift_accel_g: float
    lift_accel_nominal_skid_g: float
    lift_accel_analytic_g: float
    lift_accel_plan_reflected_g: float
    lift_accel_analytic_no_rotor_g: float
    top_speed: float
    top_speed_ideal: float
    decel_tau: float
    resistance_for_measured_tau: float


def run_checks(
    mp: MassProperties,
    params: PlantParams,
    collision: CollisionSet | None,
    dt: float,
    volts: float,
    measured_tau: float = 0.078,
) -> CheckReport:
    motor = MotorConstants(gear_ratio=mp.gear_ratio, kt=mp.motor_kt)
    model, data = make(mp, params, collision, dt)
    settle(model, data)
    rest = pitch_of(data.qpos[3:7])
    lift = simulated_lift_accel(*make(mp, params, collision, dt))
    lift_nominal = simulated_lift_accel(*make(mp, params, collision, dt), skid_mu=params.skid_mu)
    speed = simulated_top_speed(*make(mp, params, collision, dt), volts=volts)
    tau = simulated_decel_tau(*make(mp, params, collision, dt), params=params, motor=motor)
    return CheckReport(
        dt=dt,
        rest_pitch_deg=float(np.degrees(rest)),
        lift_accel_g=lift / GRAVITY,
        lift_accel_nominal_skid_g=lift_nominal / GRAVITY,
        lift_accel_analytic_g=analytic_lift_accel(mp, rest) / GRAVITY,
        lift_accel_plan_reflected_g=analytic_lift_accel(mp, rest, rotor="reflected") / GRAVITY,
        lift_accel_analytic_no_rotor_g=analytic_lift_accel(mp, rest, rotor="none") / GRAVITY,
        top_speed=speed,
        top_speed_ideal=ideal_top_speed(mp, volts),
        decel_tau=tau,
        resistance_for_measured_tau=resistance_for_tau(mp, params, measured_tau),
    )
