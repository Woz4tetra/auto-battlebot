"""MJCF for a drive robot, built from its mass-property table and convex collision pieces."""

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
FOLLOWER_JOINTS = ("follower_left", "follower_right")
FOLLOWER_GEOMS = ("follower_left_tread", "follower_right_tread")
BELT_EQUALITIES = ("belt_left", "belt_right")
ROTOR_BODIES = ("rotor_left", "rotor_right")
GEAR_EQUALITIES = ("gear_left", "gear_right")
ROTOR_MASS_KG = 1e-4
# Gear-coupling stiffness: time constant at the 2-step minimum for a 1 ms step.
GEAR_SOLREF = (0.002, 1.0)
SKID_GEOM = "nose_skid"
CHASSIS_GEOM_PREFIX = "chassis_hull_"
ROBOT_CONTYPE = 2
FLOOR_CONTYPE = 1


@dataclass(frozen=True)
class CollisionSet:
    pieces: tuple[Path, ...]  # convex OBJ pieces in the body frame
    skid_point: np.ndarray | None  # lowest nose point, body frame; None on four wheels
    rest_pitch_rad: float  # nose-down pitch with both wheels and the skid on the floor

    @classmethod
    def load(cls, directory: Path = COLLISION_DIR) -> CollisionSet:
        meta = tomllib.loads((directory / "collision.toml").read_text())
        return cls(
            pieces=tuple(directory / p for p in meta["pieces"]),
            skid_point=(
                np.asarray(meta["skid_point_m"], dtype=float) if "skid_point_m" in meta else None
            ),
            rest_pitch_rad=float(meta.get("rest_pitch_rad", 0.0)),
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
    model_name: str = "mr_stabs_mk2",
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
        skid = (
            None
            if collision.skid_point is None
            else collision.skid_point + np.array([0.0, 0.0, SKID_RADIUS_M])
        )
    else:
        # No hull: put the skid at the bottom-front of the chassis COM box.
        skid = np.array([0.07, 0.0, -0.018 + SKID_RADIUS_M])

    wheels = []
    sides = [
        (WHEEL_JOINTS[0], WHEEL_GEOMS[0], mp.wheel_left, 1.0, mp.axle_x),
        (WHEEL_JOINTS[1], WHEEL_GEOMS[1], mp.wheel_right, -1.0, mp.axle_x),
    ]
    if mp.follower_axle_x is not None:
        sides += [
            (FOLLOWER_JOINTS[0], FOLLOWER_GEOMS[0], mp.wheel_left, 1.0, mp.follower_axle_x),
            (FOLLOWER_JOINTS[1], FOLLOWER_GEOMS[1], mp.wheel_right, -1.0, mp.follower_axle_x),
        ]
    for name, geom, body, sign, axle_x in sides:
        axle_y = sign * mp.track_half_width
        axial, radial = body.fullinertia[1], 0.5 * (body.fullinertia[0] + body.fullinertia[2])
        wheels.append(
            f"""      <body name="{name}" pos="{_vec([axle_x, axle_y, 0.0])}">
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
        rotor_pos = _vec([mp.axle_x, sign * mp.track_half_width, 0.0])
        rotors.append(
            f"""      <body name="{rotor}" pos="{rotor_pos}">
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
    if mp.follower_axle_x is not None:
        equalities += [
            f'    <joint name="{eq}" joint1="{follower}" joint2="{wheel}" '
            f'polycoef="0 1 0 0 0" solref="{_vec(GEAR_SOLREF)}"/>'
            for eq, follower, wheel in zip(BELT_EQUALITIES, FOLLOWER_JOINTS, WHEEL_JOINTS)
        ]
    skid_block = (
        ""
        if skid is None
        else f"""
      <geom name="{SKID_GEOM}" type="sphere" size="{SKID_RADIUS_M}" pos="{_vec(skid)}"
            contype="{ROBOT_CONTYPE}" conaffinity="{FLOOR_CONTYPE}" condim="4"
            friction="{_vec(skid_friction(params))}"/>"""
    )

    c = mp.chassis
    asset_block = "\n".join(assets)
    hull_block = "\n".join(hull_geoms)
    wheel_block = "\n".join(wheels + rotors)
    equality_block = "\n".join(equalities)
    return f"""<mujoco model="{model_name}">
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
{hull_block}{skid_block}
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
