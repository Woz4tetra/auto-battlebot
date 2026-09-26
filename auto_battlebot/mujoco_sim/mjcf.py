"""MJCF for Mr Stabs Mk2, built from the mass-property table and the convex collision pieces.

The chassis on a free joint with the full CAD inertia tensor, two wheels on hinges about y,
and one rotor body per gearmotor, coupled to its wheel at the gear ratio by a joint equality.

The rotor is a body rather than wheel-joint `armature`. Armature adds N^2 J_r to the wheel's
joint inertia, which is right for how hard the motor must push, but passes no reaction to the
parent body. The chassis does react to the rotor spinning up, by the rate of change of the
rotor's angular momentum, N J_r alpha (not N^2 J_r alpha, which is 22.6 times larger). A rotor
body keeps both right, and the gyroscopic coupling in yaw with it. The planetary gearbox turns
the rotor the same way as the wheel, so the coupling coefficient is +N.

The chassis collides through its CoACD pieces plus a small sphere at the nose skid, where the
robot rests (the COM sits ahead of the axle). Robot geoms carry the friction; the floor's is
zero, so MuJoCo's max-combination rule leaves the robot's value in charge of each contact.

Geom and joint names are stable: the rollout looks them up to batch per-world parameters.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from xml.sax.saxutils import quoteattr

import numpy as np

from auto_battlebot.compat import tomllib
from auto_battlebot.mujoco_sim.actuator import MotorConstants, PlantParams
from auto_battlebot.mujoco_sim.mass_properties import ASSET_DIR, MassProperties

COLLISION_DIR = ASSET_DIR / "collision"
SKID_RADIUS_M = 0.0015
WHEEL_JOINTS = ("wheel_left", "wheel_right")
WHEEL_GEOMS = ("wheel_left_tread", "wheel_right_tread")
ROTOR_BODIES = ("rotor_left", "rotor_right")
GEAR_EQUALITIES = ("gear_left", "gear_right")
# The rotor's mass is already in the CAD chassis; the body only needs to carry inertia.
ROTOR_MASS_KG = 1e-4
# Gear-coupling stiffness: time constant at the 2-step minimum for a 1 ms step.
GEAR_SOLREF = (0.002, 1.0)
SKID_GEOM = "nose_skid"
CHASSIS_GEOM_PREFIX = "chassis_hull_"
ROBOT_CONTYPE = 2  # robot geoms touch the floor but not each other
FLOOR_CONTYPE = 1


@dataclass(frozen=True)
class CollisionSet:
    pieces: tuple[Path, ...]  # convex OBJ pieces in the body frame
    skid_point: np.ndarray  # lowest nose point, body frame
    rest_pitch_rad: float  # nose-down pitch with both wheels and the skid on the floor

    @classmethod
    def load(cls, directory: Path = COLLISION_DIR) -> CollisionSet:
        meta = tomllib.loads((directory / "collision.toml").read_text())
        return cls(
            pieces=tuple(directory / p for p in meta["pieces"]),
            skid_point=np.asarray(meta["skid_point_m"], dtype=float),
            rest_pitch_rad=float(meta["rest_pitch_rad"]),
        )


def rotor_diaginertia(axial: float) -> tuple[float, float, float]:
    """Rotor principal inertias. Only the axial one matters; the radial ones sit clear of the
    triangle-inequality limit (a thin disc's J/2 lands on it and fails to compile)."""
    return (0.6 * axial, axial, 0.6 * axial)


def _vec(values: np.ndarray | list[float] | tuple[float, ...]) -> str:
    return " ".join(f"{float(v):.9g}" for v in values)


def solref(params: PlantParams) -> tuple[float, float]:
    return (params.solref_timeconst, 1.0)


def solimp(params: PlantParams) -> tuple[float, float, float, float, float]:
    return (0.9, params.solimp_dmax, 0.001, 0.5, 2.0)


def wheel_friction(params: PlantParams) -> tuple[float, float, float]:
    return (params.wheel_mu_slide, params.wheel_mu_torsion, params.wheel_mu_roll)


def skid_friction(params: PlantParams) -> tuple[float, float, float]:
    return (params.skid_mu, 1e-3, 1e-4)


def initial_height(mp: MassProperties) -> float:
    return mp.wheel_radius


def build_mjcf(
    mp: MassProperties,
    params: PlantParams,
    collision: CollisionSet | None = None,
    timestep: float = 1e-3,
) -> str:
    """The model XML. `collision=None` leaves out the hull (skid sphere only), for tests."""
    motor = MotorConstants(gear_ratio=mp.gear_ratio, kt=mp.motor_kt)
    gain = motor.gain(params)
    bias = motor.bias(params)
    force = motor.force_limit(params)
    rotor_axial = mp.rotor_inertia * params.armature_scale
    half_width = mp.tread_width / 2.0
    sref, simp = solref(params), solimp(params)

    assets = []
    hull_geoms = []
    if collision is not None:
        for i, piece in enumerate(collision.pieces):
            assets.append(f'    <mesh name="hull_{i}" file={quoteattr(str(piece))}/>')
            hull_geoms.append(
                f'      <geom name="{CHASSIS_GEOM_PREFIX}{i}" type="mesh" mesh="hull_{i}" '
                f'contype="{ROBOT_CONTYPE}" conaffinity="{FLOOR_CONTYPE}" condim="3" '
                f'friction="{_vec(skid_friction(params))}" group="3"/>'
            )
        skid = collision.skid_point + np.array([0.0, 0.0, SKID_RADIUS_M])
    else:
        # No hull: put the skid at the bottom-front of the chassis COM box.
        skid = np.array([0.07, 0.0, -0.018 + SKID_RADIUS_M])

    wheels = []
    for name, geom, body, sign in (
        (WHEEL_JOINTS[0], WHEEL_GEOMS[0], mp.wheel_left, 1.0),
        (WHEEL_JOINTS[1], WHEEL_GEOMS[1], mp.wheel_right, -1.0),
    ):
        axle_y = sign * mp.track_half_width
        axial, radial = body.fullinertia[1], 0.5 * (body.fullinertia[0] + body.fullinertia[2])
        wheels.append(
            f"""      <body name="{name}" pos="{_vec([0.0, axle_y, 0.0])}">
        <inertial pos="0 0 0" mass="{body.mass:.9g}" diaginertia="{_vec([radial, axial, radial])}"/>
        <joint name="{name}" type="hinge" axis="0 1 0"
               damping="{params.joint_damping:.9g}" frictionloss="{params.joint_frictionloss:.9g}"/>
        <geom name="{geom}" type="cylinder" size="{mp.wheel_radius:.9g} {half_width:.9g}"
              zaxis="0 1 0" contype="{ROBOT_CONTYPE}" conaffinity="{FLOOR_CONTYPE}" condim="6"
              friction="{_vec(wheel_friction(params))}" rgba="0.1 0.1 0.1 1"/>
      </body>"""
        )
    rotors = []
    for rotor, sign in ((ROTOR_BODIES[0], 1.0), (ROTOR_BODIES[1], -1.0)):
        rotors.append(
            f"""      <body name="{rotor}" pos="{_vec([0.0, sign * mp.track_half_width, 0.0])}">
        <inertial pos="0 0 0" mass="{ROTOR_MASS_KG}"
                  diaginertia="{_vec(rotor_diaginertia(rotor_axial))}"/>
        <joint name="{rotor}" type="hinge" axis="0 1 0"/>
      </body>"""
        )
    equalities = [
        f'    <joint name="{eq}" joint1="{rotor}" joint2="{wheel}" '
        f'polycoef="0 {mp.gear_ratio:.9g} 0 0 0" solref="{_vec(GEAR_SOLREF)}"/>'
        for eq, rotor, wheel in zip(GEAR_EQUALITIES, ROTOR_BODIES, WHEEL_JOINTS)
    ]

    c = mp.chassis
    asset_block = "\n".join(assets)
    hull_block = "\n".join(hull_geoms)
    wheel_block = "\n".join(wheels + rotors)
    equality_block = "\n".join(equalities)
    return f"""<mujoco model="mr_stabs_mk2">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{timestep:.9g}" integrator="implicitfast" cone="elliptic"/>
  <default>
    <geom solref="{_vec(sref)}" solimp="{_vec(simp)}"/>
  </default>
  <asset>
{asset_block}
  </asset>
  <worldbody>
    <geom name="floor" type="plane" size="3 3 0.1" contype="{FLOOR_CONTYPE}"
          conaffinity="{ROBOT_CONTYPE}" friction="0 0 0" condim="1"/>
    <body name="chassis" pos="{_vec([0.0, 0.0, initial_height(mp)])}">
      <freejoint name="root"/>
      <inertial pos="{_vec(c.com)}" mass="{c.mass:.9g}" fullinertia="{_vec(c.fullinertia)}"/>
{hull_block}
      <geom name="{SKID_GEOM}" type="sphere" size="{SKID_RADIUS_M}" pos="{_vec(skid)}"
            contype="{ROBOT_CONTYPE}" conaffinity="{FLOOR_CONTYPE}" condim="4"
            friction="{_vec(skid_friction(params))}"/>
{wheel_block}
    </body>
  </worldbody>
  <equality>
{equality_block}
  </equality>
  <actuator>
    <general name="motor_left" joint="{WHEEL_JOINTS[0]}" biastype="affine" gainprm="{gain:.9g}"
             biasprm="0 0 {bias:.9g}" ctrlrange="-30 30" forcerange="{-force:.9g} {force:.9g}"/>
    <general name="motor_right" joint="{WHEEL_JOINTS[1]}" biastype="affine" gainprm="{gain:.9g}"
             biasprm="0 0 {bias:.9g}" ctrlrange="-30 30" forcerange="{-force:.9g} {force:.9g}"/>
  </actuator>
</mujoco>
"""
