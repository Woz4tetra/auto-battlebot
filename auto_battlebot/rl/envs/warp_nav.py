"""The full RL training environment: a drive robot on MuJoCo Warp (plan sections 5 to 8).

`WarpNavConfig.robot` picks the robot (`ROBOTS`): Mr Stabs Mk2 or Mrs Buff Mk3. Each world holds
that robot's fitted MJCF, four arena walls, up to three wall blocks, a house
bot and an opponent, all walls and robots other than ours as mocap bodies, so every world places
its own arena per episode without per-world model geometry. Holes are not geometry: the chassis
centre crossing a hole's radius ends the episode.

One policy step is 20 ms: two launches of a captured graph of 10 ms of physics substeps, each
substep running the robot's command chain in Warp (`warp_kernels.py`), then the sensing model
between launches when it is on. Rewards, sensing, observations (`obs_spec.build_v1`), episode
resets and domain randomization run in torch on the same GPU arrays.

Curriculum stages (plan section 11):

| Stage | Opponents | Hazards | Sensing | Randomization |
| --- | --- | --- | --- | --- |
| A | static | none, 2.5 m arena | true state | nominal plant |
| B | adds straight, circle, random_walk | up to 1 hole | true state | half ranges |
| C | adds pursuit, corner | full | sensing model | half ranges |
| D | adds replay | full | sensing model | full ranges |
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import torch

from auto_battlebot.mujoco_sim import mass_properties
from auto_battlebot.mujoco_sim.actuator import MotorConstants
from auto_battlebot.mujoco_sim.actuator import PlantParams as MjPlantParams
from auto_battlebot.mujoco_sim.closed_loop import load_fit_params
from auto_battlebot.mujoco_sim.mjcf import (
    CHASSIS_GEOM_PREFIX,
    FLOOR_CONTYPE,
    FOLLOWER_GEOMS,
    FOLLOWER_JOINTS,
    ROBOT_CONTYPE,
    ROTOR_BODIES,
    SKID_GEOM,
    WHEEL_GEOMS,
    WHEEL_JOINTS,
    CollisionSet,
    build_mjcf,
)
from auto_battlebot.mujoco_sim.rollout import BATCHED_FIELDS, ModelIndex, rest_height
from auto_battlebot.perception.opponent_filter import OpponentFilter, OpponentFilterConfig
from auto_battlebot.perception.sensing_model import (
    Context,
    MeasurementModel,
    OurEkfErrorModel,
    SensingModelFile,
    TruePoses,
)
from auto_battlebot.rl import obs_spec
from auto_battlebot.rl.env import SOURCES, EnvSpec, Step
from auto_battlebot.rl.envs.opponents import (
    BEHAVIOURS,
    PURSUIT,
    REPLAY,
    Opponents,
    box_support,
)
from auto_battlebot.rl.envs.policy_opponents import PolicyDriver, PolicyOpponents
from auto_battlebot.rl.envs.replay_tracks import ReplayTracks
from auto_battlebot.rl.torch_plant import saturate

REPO = Path(__file__).resolve().parents[3]
DEFAULT_FIT = "playground/calibration/out/mujoco_sysid/fit_2026-10-15_yolo_brake/fit.json"
MK3_INTERIM_FIT = "playground/calibration/out/mujoco_sysid/mk3_interim/fit.json"
WALL_HEIGHT_M = 0.3
WALL_T_M = 0.05
PARKED = 50.0
N_SLOTS = 3  # holes and wall blocks
WALL_MOCAPS = 4
BLOCK_MOCAPS = 3
HOUSE_MOCAP = WALL_MOCAPS + BLOCK_MOCAPS
OPP_MOCAP = HOUSE_MOCAP + 1
# Footprints as length, width, height (config/_common.toml robot_size_meters_per_label).
ROBOT_SIZE_M = {
    "mr_stabs_mk2": (0.137, 0.16, 0.0703),
    "mrs_buff_mk3": (0.2225, 0.2535, 0.0861),
}
OPP_SIZE_M = ROBOT_SIZE_M["mrs_buff_mk3"]  # the largest opponent footprint, the MJCF placeholder
# HOUSE_BOT footprint half-diagonal from config/_common.toml (0.254 x 0.3302 m), the radius the
# C++ hazard assembler gives a tracked house bot.
HOUSE_RADIUS_M = 0.2083
OUR_HALF_DIAGONAL_M = 0.5 * math.hypot(0.137, 0.16)


@dataclass(frozen=True)
class RobotProfile:
    """What the environment needs to know about the robot it trains.

    `command`: "firmware" runs Mr Stabs Mk2's onboard mixer and gyro yaw loop; "direct" is a
    radio mix straight to open-loop ESCs (Mrs Buff Mk3, no eRPM feedback, no IMU). `weapon`:
    "wedge" scores contact on a box at the nose (`strike_x_m`, `strike_half_width_m`); "disk"
    scores the weapon's sweep cylinder from mass_properties.toml. `opponents`: the robots whose
    footprints the opponent box is drawn from, one per episode.
    """

    fit_file: str
    command: str
    weapon: str
    opponents: tuple[str, ...]
    # Opponents driven by trained policies (`policy_opponents.py`), drawn in a stage's
    # `policy` share of episodes.
    drivers: tuple[PolicyDriver, ...] = ()
    # Which contacts are hits taken (`warp_kernels.in_opponent_arc`): "angle", anywhere in the
    # opponent's front arc, or "front_face", only its front face inside the arc.
    arc_rule: str = "angle"


MK2_POLICY_DRIVER = PolicyDriver(
    "mr_stabs_mk2_policy",
    "mr_stabs_mk2",
    "config/mr_stabs_mk2_desktop.toml",
    "runs/rl_nav/rl_nav_ppo_p4_s7_2026-10-18_04-04-10/checkpoint.pt",
)
ROBOTS = {
    "mr_stabs_mk2": RobotProfile(DEFAULT_FIT, "firmware", "wedge", ("mrs_buff_mk3",)),
    "mrs_buff_mk3": RobotProfile(
        MK3_INTERIM_FIT,
        "direct",
        "disk",
        ("mr_stabs_mk2", "mrs_buff_mk3"),
        drivers=(
            MK2_POLICY_DRIVER,
            PolicyDriver("self_play", "mrs_buff_mk3", "config/_common.toml", ""),
        ),
        arc_rule="front_face",
    ),
}


@dataclass(frozen=True)
class Stage:
    opponents: tuple[str, ...]
    hazards: int  # most holes and wall blocks per arena
    holes_only: bool
    sensing: bool
    random: float  # domain randomization: 0 nominal plant, 1 full ranges
    arena: tuple[float, float]  # side length range, m
    run_away: float  # fraction of RUN_AWAY episodes
    house: float  # probability of a house bot
    policy: float = 0.0  # share of opponents driven by a policy, for robots that have drivers


STAGE_B_OPPONENTS = ("static", "straight", "circle", "random_walk")
STAGE_C_OPPONENTS = (*STAGE_B_OPPONENTS, "pursuit", "corner")
STAGES = {
    "A": Stage(("static",), 0, True, False, 0.0, (2.5, 2.5), 0.0, 0.0),
    "B": Stage(
        ("static", "straight", "circle", "random_walk"), 1, True, False, 0.5, (1.5, 2.5), 0.2, 0.0
    ),
    "C": Stage(
        ("static", "straight", "circle", "random_walk", "pursuit", "corner"),
        3,
        False,
        True,
        0.5,
        (1.5, 2.5),
        0.2,
        0.5,
        0.25,
    ),
    "D": Stage(BEHAVIOURS, 3, False, True, 1.0, (1.5, 2.5), 0.2, 0.5, 0.3),
    # Pursuit focus: stage D with half the episodes against a pursuer, so the rest of the mix is
    # still trained while pursuit gets enough episodes to learn from. Pair with the pursuit
    # difficulty curriculum in train_nav_policy.py.
    "P": Stage((*BEHAVIOURS, *("pursuit",) * 5), 3, False, True, 1.0, (1.5, 2.5), 0.2, 0.5, 0.3),
    # Three quarters pursuit (16 of 22 draws), for runs with --anchor holding the rest of the
    # mix: at half, pursuit success stalled at 0.22 to 0.31 on level 0.4 for 1,500 iterations.
    "P3": Stage((*BEHAVIOURS, *("pursuit",) * 15), 3, False, True, 1.0, (1.5, 2.5), 0.2, 0.5, 0.3),
    # Evaluation (gates G4, G5): stage D's mix on the nominal plant, so the lockstep sim, which
    # runs the fit as it is, plays the same robot. Pair with `holdout=True` for the held-out
    # pursuit corner and replay tracks.
    "EVAL": Stage(BEHAVIOURS, 3, False, True, 0.0, (1.5, 2.5), 0.2, 0.5, 0.3),
    # Pursuit only, attack only, open arena, nominal plant: the pursuit difficulty map and the
    # pursuit evaluation set (playground/rl/pursuit_map.py).
    "PURSUIT": Stage(("pursuit",), 0, True, True, 0.0, (1.5, 2.5), 0.0, 0.0),
    # Diagnostics for a stalled stage C: stage B's opponents and hazards with sensing on, and
    # stage C's opponents and hazards with sensing off.
    "DIAG_SENSE": Stage(STAGE_B_OPPONENTS, 1, True, True, 0.5, (1.5, 2.5), 0.2, 0.0),
    "DIAG_OPP": Stage(STAGE_C_OPPONENTS, 3, False, False, 0.5, (1.5, 2.5), 0.2, 0.5),
}

PRIVILEGED_NAMES = (
    "true_opp_x",
    "true_opp_y",
    "true_opp_heading_sin",
    "true_opp_heading_cos",
    "true_phi_sin",
    "true_phi_cos",
    "true_our_speed",
    "true_our_yaw_rate",
    "true_pitch",
    "engaged",
    "struck",
    "latency",
    "friction_scale",
    "pack_volts",
    "episode_progress",
    "run_away",
    *(f"behaviour_{b}" for b in BEHAVIOURS),
    "opp_speed",
)


@dataclass
class WarpNavConfig:
    worlds: int = 4096
    stage: str = "A"
    timestep_s: float = 0.002
    policy_dt_s: float = 0.02
    measure_period_s: float = 0.01
    episode_s: float = 20.0
    robot: str = "mr_stabs_mk2"  # a key of ROBOTS
    fit_file: str = ""  # empty: the robot's own (RobotProfile.fit_file)
    sensing_model: str = "simulation/sensing_models/provisional_rl.toml"
    arc_half_angle_deg: float = 60.0
    strike_x_m: float = 0.06
    strike_half_width_m: float = 0.081
    danger_margin_m: float = 0.10
    # reward["hole_near_per_s"] ramps from 0 at this distance outside a hole's edge to full weight
    # at the edge, so the policy learns to keep clear before the terminal hole penalty.
    hole_margin_m: float = 0.15
    # Under this body speed, unless our wedge is on the opponent from outside its arc, the robot
    # counts as standing still and pays reward["still_per_s"]. Spinning in place still counts: a
    # robot turning on the spot is as easy to hit as a parked one.
    still_speed_mps: float = 0.2
    stage_offset_m: float = 0.25
    engagement_gap_s: float = 0.5
    # Continued contact inside the opponent's arc is charged as another hit taken every this many
    # seconds: a weapon keeps hitting a robot pinned against it. Before, only the first touch
    # counted, so freezing nose-to-nose with a pursuer cost one hit for a whole episode.
    arc_rehit_s: float = 0.5
    latency_s: tuple[float, float] = (0.04, 0.08)
    pack_volts: tuple[float, float] = (14.0, 16.4)
    flip_hold_s: float = 0.5
    gamma: float = 0.99
    holdout: bool = False  # evaluation: draw only the held-out pursuit corner and replay tracks
    nconmax: int = 48
    njmax: int = 640
    reward: dict[str, float] = field(
        default_factory=lambda: {
            "strike_base": 3.0,
            "strike_rear": 7.0,
            "push_per_s": 1.0,
            "hit_taken": -10.0,
            "danger_per_s": -1.0,
            "hole": -10.0,
            # Off by default. Mrs Buff Mk3's stage C fine-tune with hole -30 still fell in on
            # 0.05 to 0.07 of training episodes, mostly with no hit taken: a terminal penalty
            # alone gives little credit to the steps that drove there.
            "hole_near_per_s": 0.0,
            "block_event": -0.5,
            "wall_event": -0.2,
            "flip": -20.0,
            "nose_lift_per_s": -0.5,
            # A moving target beats a parked one even when moving does not lead to a strike. The
            # 2026-10-17 policy parked for whole episodes in a facing opponent's front arc,
            # waiting for it to turn (rl_navigation_report.md); waiting cost nothing. 20 s parked
            # now costs as much as one hit taken. Applies in both modes.
            "still_per_s": -0.5,
            # 5x the plan's -0.01: a stage C evaluation (2026-10-17) found the deterministic policy
            # moving its linear command 0.11 per 20 ms step, 0.18 before a tip, and flipping in
            # 40% of episodes; chatter spins the rotors up and down and their reaction lifts the
            # chassis, which no slew limit stops any more.
            "command_change": -0.05,
            "shaping": 1.0,
            # Weight of the flank term in the attack potential (`_potential`): being beside or
            # behind the opponent at close range. A pursuer turns to keep us in its arc, so the
            # point-behind distance alone barely changes while we circle it.
            "flank": 2.0,
        }
    )


def resolved_fit_file(cfg: WarpNavConfig) -> str:
    return cfg.fit_file or ROBOTS[cfg.robot].fit_file


def wrap(a: torch.Tensor) -> torch.Tensor:
    return torch.remainder(a + math.pi, 2.0 * math.pi) - math.pi


def smoothstep(u: torch.Tensor) -> torch.Tensor:
    u = u.clamp(0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


FLANK_RANGE_M = 0.8


def flank_potential(rel: torch.Tensor, opp_yaw: torch.Tensor, arc: float) -> torch.Tensor:
    """0 to 1: how far our bearing from the opponent's heading is outside half its arc, at most
    when beside or behind it, times how close we are (1 at its centre, 0 past FLANK_RANGE_M)."""
    phi = wrap(torch.atan2(rel[:, 1], rel[:, 0]) - opp_yaw).abs()
    side = smoothstep((phi - 0.5 * arc) / (0.5 * math.pi - 0.5 * arc))
    near = (1.0 - rel.norm(dim=1) / FLANK_RANGE_M).clamp(0.0, 1.0)
    out: torch.Tensor = side * near
    return out


def _scaled(rng: tuple[float, float], s: float) -> tuple[float, float]:
    """A multiplier range around 1 shrunk by s (0 nominal, 1 full)."""
    return 1.0 - s * (1.0 - rng[0]), 1.0 + s * (rng[1] - 1.0)


def build_model(
    mp: Any,
    params: MjPlantParams,
    collision: CollisionSet,
    timestep: float,
    model_name: str = "mr_stabs_mk2",
) -> mujoco.MjModel:
    spec = mujoco.MjSpec.from_string(build_mjcf(mp, params, collision, timestep, model_name))
    # MJ_MINMU, not zero: MuJoCo Warp warns that a zero coefficient can produce NaN, and contacts
    # take the larger of the pair's values, so the robot's own friction still governs.
    common = {
        "contype": FLOOR_CONTYPE,
        "conaffinity": ROBOT_CONTYPE,
        "friction": [1e-5, 1e-5, 1e-5],
    }
    for i in range(WALL_MOCAPS):
        body = spec.worldbody.add_body(
            name=f"arena_wall_{i}", mocap=True, pos=[PARKED + 10 * i, PARKED, WALL_HEIGHT_M / 2]
        )
        body.add_geom(
            name=f"arena_wall_{i}",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[WALL_T_M / 2, 1.0, WALL_HEIGHT_M / 2],
            **common,
        )
    for i in range(BLOCK_MOCAPS):
        body = spec.worldbody.add_body(
            name=f"wall_block_{i}",
            mocap=True,
            pos=[PARKED, PARKED + 10 * (i + 1), WALL_HEIGHT_M / 2],
        )
        body.add_geom(
            name=f"wall_block_{i}",
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            size=[0.1, WALL_HEIGHT_M / 2, 0],
            **common,
        )
    body = spec.worldbody.add_body(name="house_bot", mocap=True, pos=[-PARKED, PARKED, 0.12])
    body.add_geom(
        name="house_bot",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=[HOUSE_RADIUS_M, 0.12, 0],
        **common,
    )
    body = spec.worldbody.add_body(
        name="opponent", mocap=True, pos=[-PARKED, -PARKED, OPP_SIZE_M[2] / 2]
    )
    body.add_geom(
        name="opponent",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[OPP_SIZE_M[0] / 2, OPP_SIZE_M[1] / 2, OPP_SIZE_M[2] / 2],
        **common,
    )
    return spec.compile()


@SOURCES.register("warp_nav")
class WarpNavEnv:
    def __init__(
        self, cfg: WarpNavConfig, device: torch.device | str = "cuda", seed: int = 0
    ) -> None:
        import mujoco_warp as mjw
        import warp as wp

        from auto_battlebot.rl.envs import warp_kernels

        self._wp, self._mjw, self._k = wp, mjw, warp_kernels
        wp.init()
        self.cfg = cfg
        self.device = torch.device(device)
        self.dtype = torch.float32
        self.num_worlds = w = cfg.worlds
        self.substeps = int(round(cfg.measure_period_s / cfg.timestep_s))
        self.launches = int(round(cfg.policy_dt_s / cfg.measure_period_s))
        self.max_steps = int(round(cfg.episode_s / cfg.policy_dt_s))
        self.profile = ROBOTS[cfg.robot]
        drivers = ("scripted", *(d.name for d in self.profile.drivers))
        # The opponent's driver joins the critic's inputs only for a robot that has drivers, so
        # Mr Stabs Mk2's privileged vector, and the checkpoints trained on it, stay as they were.
        self.privileged_names = (
            *PRIVILEGED_NAMES,
            *((f"driver_{d}" for d in drivers) if self.profile.drivers else ()),
        )
        self.spec = EnvSpec(
            obs_names=obs_spec.OBS_NAMES,
            privileged_names=self.privileged_names,
            obs_spec_version=obs_spec.OBS_SPEC_VERSION,
            policy_dt_s=cfg.policy_dt_s,
            max_episode_steps=self.max_steps,
        )
        self.gen = torch.Generator(device=self.device)
        self.gen.manual_seed(seed)
        self.arc = math.radians(cfg.arc_half_angle_deg)
        self.stage = cfg.stage

        asset = mass_properties.asset_dir(cfg.robot)
        self.mp = mass_properties.load(asset / "mass_properties.toml")
        self.collision = CollisionSet.load(asset / "collision")
        size = ROBOT_SIZE_M[cfg.robot]
        self.our_half = (size[0] / 2, size[1] / 2)
        self.our_half_diag = math.hypot(*self.our_half)
        self.opp_sizes = torch.tensor(
            [ROBOT_SIZE_M[r] for r in self.profile.opponents], device=self.device, dtype=self.dtype
        )
        if self.profile.weapon == "disk" and self.mp.weapon is None:
            raise ValueError(f"{cfg.robot} scores a disk weapon but has no [weapon] sweep")
        self.fit = load_fit_params(REPO / resolved_fit_file(cfg))
        # The fit's rolling friction (7.9e-6) is under MJ_MINMU, which MuJoCo Warp warns can give
        # NaN with condim 6. The difference is far below anything the fit resolved.
        self.fit = self.fit.replace(wheel_mu_roll=max(self.fit.wheel_mu_roll, 1e-5))
        self.motor = MotorConstants(gear_ratio=self.mp.gear_ratio, kt=self.mp.motor_kt)
        self.mjm = build_model(self.mp, self.fit, self.collision, cfg.timestep_s, cfg.robot)
        mjd = mujoco.MjData(self.mjm)
        mujoco.mj_forward(self.mjm, mjd)
        self.index = ModelIndex.of(self.mjm)
        self.height = rest_height(self.mp, self.collision, cfg.timestep_s)
        # geom_rbound and geom_aabb are the broadphase bounds. Compiled once from the MJCF's
        # placeholder sizes, they would cull contacts with the outer part of any wall block or
        # opponent this environment sizes larger, so they are batched and kept in step with
        # geom_size (`_sync_bounds`).
        batched = {
            f: w
            for f in (
                *BATCHED_FIELDS,
                "geom_size",
                "geom_rbound",
                "geom_aabb",
                "body_pos",
                "body_ipos",
            )
        }
        self.m = mjw.put_model(self.mjm, batch_sizes=batched)
        # A robot with many collision pieces (Mrs Buff Mk3's 30) pairs each with the floor, walls
        # and opponent in broadphase, so the buffers grow with the model.
        nconmax = max(cfg.nconmax, 3 * self.mjm.ngeom)
        njmax = max(cfg.njmax, 6 * nconmax)
        self.d = mjw.put_data(self.mjm, mjd, nworld=w, nconmax=nconmax, njmax=njmax)

        t = wp.to_torch
        self.qpos = t(self.d.qpos)
        self.qvel = t(self.d.qvel)
        self.mocap_pos = t(self.d.mocap_pos)
        self.mocap_quat = t(self.d.mocap_quat)
        self.geom_size = t(self.m.geom_size)
        self.geom_rbound = t(self.m.geom_rbound)
        self.geom_aabb = t(self.m.geom_aabb)  # [W, ngeom, 2 (center, half size), 3]
        self.geom_friction = t(self.m.geom_friction)
        self.body_pos = t(self.m.body_pos)
        self.body_ipos = t(self.m.body_ipos)
        self.body_inertia = t(self.m.body_inertia)
        self.dof_damping = t(self.m.dof_damping)
        self.gainprm = t(self.m.actuator_gainprm)
        self.biasprm = t(self.m.actuator_biasprm)
        self.forcerange = t(self.m.actuator_forcerange)
        self.base = {
            k: v[0].clone()
            for k, v in (
                ("geom_size", self.geom_size),
                ("geom_friction", self.geom_friction),
                ("body_pos", self.body_pos),
                ("body_ipos", self.body_ipos),
                ("body_inertia", self.body_inertia),
                ("dof_damping", self.dof_damping),
            )
        }

        names = [
            mujoco.mj_id2name(self.mjm, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
            for i in range(self.mjm.ngeom)
        ]
        role = np.zeros(self.mjm.ngeom, dtype=np.int32)
        for i, n in enumerate(names):
            if (
                n.startswith(CHASSIS_GEOM_PREFIX)
                or n == SKID_GEOM
                or n in WHEEL_GEOMS
                or n in FOLLOWER_GEOMS
            ):
                role[i] = 1
            elif n == "opponent":
                role[i] = 2
            elif n.startswith("arena_wall_"):
                role[i] = 3
            elif n.startswith("wall_block_"):
                role[i] = 4
            elif n == "house_bot":
                role[i] = 5
        self.geom_role: Any = wp.array(role, dtype=int)
        self.wall_geoms = [names.index(f"arena_wall_{i}") for i in range(WALL_MOCAPS)]
        self.block_geoms = [names.index(f"wall_block_{i}") for i in range(BLOCK_MOCAPS)]
        self.opp_geom = names.index("opponent")
        self.wheel_geoms = [names.index(g) for g in (*WHEEL_GEOMS, *FOLLOWER_GEOMS) if g in names]
        self.skid_geoms = [
            i for i, n in enumerate(names) if n == SKID_GEOM or n.startswith(CHASSIS_GEOM_PREFIX)
        ]
        bodies = {self.mjm.body(i).name for i in range(self.mjm.nbody)}
        self.wheel_bodies = [
            self.mjm.body(n).id for n in (*WHEEL_JOINTS, *FOLLOWER_JOINTS) if n in bodies
        ]
        self.rotor_bodies = [self.mjm.body(n).id for n in ROTOR_BODIES]
        self.chassis_body = self.mjm.body("chassis").id
        self.mocap_ids = {
            n: int(self.mjm.body(n).mocapid[0])
            for n in (
                *(f"arena_wall_{i}" for i in range(WALL_MOCAPS)),
                *(f"wall_block_{i}" for i in range(BLOCK_MOCAPS)),
                "house_bot",
                "opponent",
            )
        }
        assert (
            self.mocap_ids["opponent"] == OPP_MOCAP and self.mocap_ids["house_bot"] == HOUSE_MOCAP
        )

        cap = int(math.ceil(0.12 / cfg.timestep_s)) + 4
        self.head = wp.zeros(1, dtype=int)
        self.cmd = wp.zeros((w, 2), dtype=float)
        self.transport_ring = wp.zeros((w, cap, 2), dtype=float)
        self.drive_ring = wp.zeros((w, cap, 2), dtype=float)
        self.transport_lag = wp.zeros(w, dtype=int)
        self.drive_lag = wp.zeros(w, dtype=int)
        self.fw = wp.zeros((w, warp_kernels.FW_COLUMNS), dtype=float)
        self.shaping = wp.zeros((w, 9), dtype=float)
        self.upside_wp = wp.zeros(w, dtype=int)
        self.moving: Any = wp.array(np.array([OPP_MOCAP, HOUSE_MOCAP], dtype=np.int32), dtype=int)
        self.mover_vel = wp.zeros((w, 2, 3), dtype=float)
        self.flags_wp = wp.zeros((w, 7), dtype=float)
        self.wheel_dofs: Any = wp.array(self.index.wheel_dofs.astype(np.int32), dtype=int)
        self.emf = float(self.mp.gear_ratio * self.mp.motor_kt)
        self.direct = 1 if self.profile.command == "direct" else 0
        self.arc_rule = (
            warp_kernels.ARC_FRONT_FACE
            if self.profile.arc_rule == "front_face"
            else warp_kernels.ARC_ANGLE
        )
        self.weapon_kind = (
            warp_kernels.WEAPON_DISK if self.profile.weapon == "disk" else warp_kernels.WEAPON_WEDGE
        )
        sweep = self.mp.weapon
        self.sweep = wp.vec4(
            *(
                (sweep.center[0], sweep.center[2], sweep.radius, sweep.half_width)
                if sweep is not None
                else (0.0, 0.0, 0.0, 0.0)
            )
        )
        self.sweep_y = float(sweep.center[1]) if sweep is not None else 0.0
        self.cmd_t = t(self.cmd)
        self.fw_t = t(self.fw)
        self.shaping_t = t(self.shaping)
        self.transport_lag_t = t(self.transport_lag)
        self.drive_lag_t = t(self.drive_lag)
        self.mover_vel_t = t(self.mover_vel)
        self.flags = t(self.flags_wp)
        self.upside = t(self.upside_wp)
        self.transport_ring_t = t(self.transport_ring)
        self.drive_ring_t = t(self.drive_ring)
        self._graph: Any = None

        dev, dt = self.device, self.dtype
        z = torch.zeros(w, device=dev, dtype=dt)
        zb = torch.zeros(w, dtype=torch.bool, device=dev)
        self.arena_half = torch.full((w, 2), 1.25, device=dev, dtype=dt)
        self.slot_xy = torch.zeros(w, N_SLOTS, 2, device=dev, dtype=dt)
        self.slot_r = torch.zeros(w, N_SLOTS, device=dev, dtype=dt)
        self.slot_kind = torch.zeros(w, N_SLOTS, dtype=torch.long, device=dev)
        self.slot_present = torch.zeros(w, N_SLOTS, dtype=torch.bool, device=dev)
        self.house_present = zb.clone()
        self.house_vel = torch.zeros(w, 2, device=dev, dtype=dt)
        self.opp_half = torch.tensor(OPP_SIZE_M[:2], device=dev, dtype=dt).repeat(w, 1) / 2
        self.opp_model = torch.zeros(w, dtype=torch.long, device=dev)  # index into opponents
        self.run_away = zb.clone()
        self.t = z.clone()
        self.steps = torch.zeros(w, dtype=torch.long, device=dev)
        self.prev_action = torch.zeros(w, 2, device=dev, dtype=dt)
        self.engaged = zb.clone()
        self.struck = zb.clone()
        self.apart_s = z.clone()
        self.arc_s = z.clone()  # seconds of arc contact since the last hit charged
        self.prev_arc = zb.clone()
        self.prev_wall = zb.clone()
        self.prev_block = zb.clone()
        self.flip_s = z.clone()
        self.latency = z + 0.06
        self.friction_scale = z + 1.0
        self.pack = z + 15.2
        self.hist_action = torch.zeros(w, obs_spec.HISTORY, 2, device=dev, dtype=dt)
        self.hist_speed = torch.zeros(w, obs_spec.HISTORY, device=dev, dtype=dt)
        self.hist_rate = torch.zeros(w, obs_spec.HISTORY, device=dev, dtype=dt)
        self.last_speed = z.clone()
        self.last_rate = z.clone()
        self.stats_keys = (
            "strikes",
            "rear_strikes",
            "hits_taken",
            "danger_s",
            "wall_events",
            "block_contacts",
            "holes",
            "flips",
            "nose_lift_s",
            "still_s",
            "sim_failures",
            "reward",
        )
        self.stats = {k: z.clone() for k in self.stats_keys}
        self.first_strike_s = torch.full((w,), math.inf, device=dev, dtype=dt)
        self.first_hit_s = torch.full((w,), math.inf, device=dev, dtype=dt)
        self.opponents = Opponents(w, dev, dt, self.gen, our_half=self.our_half)
        self.policy_opp = PolicyOpponents(
            self.profile.drivers, self.spec, w, cfg.policy_dt_s, dev, dt
        )
        self.opponents.replay = ReplayTracks(cfg.holdout, dev, dt, episode_s=cfg.episode_s)
        self.success_window: list[float] = []
        self.pursuit_window: list[float] = []  # attack episodes against a pursuer
        self.episode_stage = torch.full((w,), -1, dtype=torch.long, device=dev)
        self.episode_pursuit_level = torch.full((w,), -1.0, device=dev, dtype=dt)

        self.sensing = False
        self.spec_file = SensingModelFile.load(REPO / cfg.sensing_model)
        self.measure = MeasurementModel(
            self.spec_file, ("ours", "opponent"), w, dev, dt, seed=seed + 101
        )
        self.ekf = OurEkfErrorModel(self.spec_file, w, "ours", 0.2, dev, dt, seed=seed + 202)
        self.opp_filter = OpponentFilter(
            w,
            OpponentFilterConfig(
                opponent_yaw_accel_psd=30.0,
                opponent_heading_sigma_rad=0.15,
                opponent_max_coast_s=0.1,
                render_lead_s=0.016,
            ),
            field_half_extent=self.arena_half,
            device=dev,
            dtype=dt,
        )
        self.set_stage(cfg.stage)

    # ------------------------------------------------------------------ curriculum

    def set_stage(self, stage: str) -> None:
        self.stage = stage
        self.stage_index = list(STAGES).index(stage)
        self.stage_cfg = STAGES[stage]
        was_sensing = getattr(self, "sensing", False)
        self.sensing = bool(self.stage_cfg.sensing)
        if self.sensing and not was_sensing and hasattr(self, "qpos"):
            # Worlds mid-episode start their sensing state now rather than at their next reset.
            self._reset_sensing(torch.ones(self.num_worlds, dtype=torch.bool, device=self.device))
        allowed = self.stage_cfg.opponents
        if "replay" in allowed and self.opponents.replay is None:
            allowed = tuple(b for b in allowed if b != "replay")
        self.allowed = tuple(BEHAVIOURS.index(b) for b in allowed)

    def record_success(self, episode: dict[str, torch.Tensor], done: torch.Tensor) -> None:
        # Only episodes that started in the current stage count: with desynchronized episodes,
        # the first ones to end after an advance began in the easier stage and would pass it.
        done = done & (self.episode_stage == self.stage_index)
        if not bool(done.any()):
            return
        attack_ok = episode["strikes"][done] > 0
        run_ok = (
            (episode["hits_taken"][done] == 0)
            & (episode["holes"][done] == 0)
            & (episode["flips"][done] == 0)
        )
        ok = torch.where(episode["run_away"][done] > 0.5, run_ok, attack_ok)
        self.success_window.extend(ok.float().tolist())
        self.success_window = self.success_window[-2000:]
        # Likewise only pursuers drawn at the current level: the window refills within an
        # iteration, and episodes from the easier level raised job 43 from 0.4 to 0.7 in 7
        # iterations.
        level = self.episode_pursuit_level[done] == self.opponents.pursuit_level
        scripted = episode["driver"][done] == 0
        chased = (
            (episode["behaviour"][done] == PURSUIT)
            & (episode["run_away"][done] < 0.5)
            & level
            & scripted
        )
        self.pursuit_window.extend(ok[chased].float().tolist())
        self.pursuit_window = self.pursuit_window[-1000:]

    @property
    def success_rate(self) -> float:
        return float(np.mean(self.success_window)) if self.success_window else 0.0

    @property
    def pursuit_success_rate(self) -> float:
        return float(np.mean(self.pursuit_window)) if self.pursuit_window else 0.0

    # ------------------------------------------------------------------ randomness

    def _rand(self, *shape: int) -> torch.Tensor:
        return torch.rand(*shape, generator=self.gen, device=self.device, dtype=self.dtype)

    def _uniform(self, lo: float, hi: float, *shape: int) -> torch.Tensor:
        return lo + (hi - lo) * self._rand(*(shape or (self.num_worlds,)))

    # ------------------------------------------------------------------ episodes

    def reset(self, seed: int | None = None) -> tuple[torch.Tensor, torch.Tensor]:
        if seed is not None:
            self.gen.manual_seed(seed)
        everyone = torch.ones(self.num_worlds, dtype=torch.bool, device=self.device)
        self._reset_worlds(everyone)
        return self._observe()

    def desynchronize(self) -> None:
        self.steps = (self._rand(self.num_worlds) * self.max_steps).long()

    def _reset_worlds(self, mask: torch.Tensor) -> None:
        self._mjw.reset_data(self.m, self.d, self._wp.from_torch(mask.to(torch.int32)))
        s = float(self.stage_cfg.random)
        self._reset_arena(mask)
        oxy, opp_foot = self._reset_opponent(mask, s)
        self._reset_hazards(mask, oxy, opp_foot)
        hxy = self._reset_house(mask, oxy, opp_foot)
        xy = self._reset_our_start(mask, oxy, hxy)
        self._randomize_plant(mask, s)
        self._sync_bounds()
        self._reset_bookkeeping(mask, xy)

    def _sync_bounds(self) -> None:
        """Broadphase bounds of the resized scene geoms, from their current sizes, as MuJoCo
        compiles them: a box's half sizes, a cylinder's (r, r, half height), each centred."""
        for geoms, cylinder in (
            (self.wall_geoms, False),
            (self.block_geoms, True),
            ([self.opp_geom], False),
        ):
            for g in geoms:
                size = self.geom_size[:, g]
                half = torch.stack([size[:, 0], size[:, 0], size[:, 1]], 1) if cylinder else size
                self.geom_aabb[:, g, 0] = 0.0
                self.geom_aabb[:, g, 1] = half
                self.geom_rbound[:, g] = (
                    torch.hypot(size[:, 0], size[:, 1]) if cylinder else size.norm(dim=1)
                )

    def _reset_arena(self, mask: torch.Tensor) -> None:
        m1 = mask[:, None]
        lo, hi = self.stage_cfg.arena
        half = torch.stack([self._uniform(lo, hi), self._uniform(lo, hi)], 1) / 2
        self.arena_half = torch.where(m1, half, self.arena_half)
        self.opp_filter.bounds = self.arena_half.clone()
        hx, hy = self.arena_half[:, 0], self.arena_half[:, 1]
        t = WALL_T_M / 2
        walls = (
            (hx + t, torch.zeros_like(hx), hy + 2 * t, 0),
            (-hx - t, torch.zeros_like(hx), hy + 2 * t, 0),
            (torch.zeros_like(hx), hy + t, hx + 2 * t, 1),
            (torch.zeros_like(hx), -hy - t, hx + 2 * t, 1),
        )
        for i, (x, y, length, axis) in enumerate(walls):
            g = self.wall_geoms[i]
            size = torch.stack(
                [torch.full_like(x, t), length, torch.full_like(x, WALL_HEIGHT_M / 2)], 1
            )
            if axis == 1:
                size = size[:, [1, 0, 2]]
            self.geom_size[:, g] = torch.where(m1, size, self.geom_size[:, g])
            pos = torch.stack([x, y, torch.full_like(x, WALL_HEIGHT_M / 2)], 1)
            self.mocap_pos[:, i] = torch.where(m1, pos, self.mocap_pos[:, i])

    def _reset_opponent(self, mask: torch.Tensor, s: float) -> tuple[torch.Tensor, torch.Tensor]:
        n = self.num_worlds
        m1 = mask[:, None]
        model = torch.randint(0, len(self.opp_sizes), (n,), generator=self.gen, device=self.device)
        self.policy_opp.draw(mask, self.stage_cfg.policy, self.gen)
        model = self.policy_opp.robot_of(self.policy_opp.driver, model, self.profile.opponents)
        self.opp_model = torch.where(mask, model, self.opp_model)
        osize = self.opp_sizes[model] * self._uniform(*_scaled((0.8, 1.2), s))[:, None]
        self.opp_half = torch.where(m1, osize[:, :2] / 2, self.opp_half)
        g = self.opp_geom
        self.geom_size[:, g] = torch.where(m1, osize / 2, self.geom_size[:, g])
        opp_foot = self.opp_half.norm(dim=1)
        reach = self.arena_half - opp_foot[:, None] - 0.05
        oxy = (self._rand(n, 2) * 2 - 1) * reach
        oyaw = self._uniform(-math.pi, math.pi)
        # Behaviour first, so a replay opponent starts on its track and the rest of the scene is
        # placed around where it really is.
        speed_scale = self._uniform(*_scaled((0.8, 1.2), s))
        self.opponents.reset(mask, self.allowed, speed_scale, self.cfg.holdout)
        replaying = mask & (self.opponents.p.behaviour == REPLAY)
        if bool(replaying.any()) and self.opponents.replay is not None:
            rxy, ryaw = self.opponents.replay.sample(
                self.opponents.replay_index, self.opponents.replay_t, reach
            )
            oxy = torch.where(replaying[:, None], rxy, oxy)
            oyaw = torch.where(replaying, ryaw, oyaw)
        opos = torch.cat([oxy, (osize[:, 2:3] / 2)], 1)
        self.mocap_pos[:, OPP_MOCAP] = torch.where(m1, opos, self.mocap_pos[:, OPP_MOCAP])
        oq = torch.stack(
            [
                torch.cos(oyaw / 2),
                torch.zeros_like(oyaw),
                torch.zeros_like(oyaw),
                torch.sin(oyaw / 2),
            ],
            1,
        )
        self.mocap_quat[:, OPP_MOCAP] = torch.where(m1, oq, self.mocap_quat[:, OPP_MOCAP])
        return oxy, opp_foot

    def _reset_hazards(self, mask: torch.Tensor, oxy: torch.Tensor, opp_foot: torch.Tensor) -> None:
        n = self.num_worlds
        m1 = mask[:, None]
        sc = self.stage_cfg
        # Up to N_SLOTS holes or wall blocks, clear of the opponent's start.
        count = torch.randint(0, int(sc.hazards) + 1, (n,), generator=self.gen, device=self.device)
        present = torch.arange(N_SLOTS, device=self.device)[None] < count[:, None]
        kind = torch.where(self._rand(n, N_SLOTS) < 0.5, obs_spec.HOLE, obs_spec.WALL_BLOCK)
        if sc.holes_only:
            kind = torch.full_like(kind, obs_spec.HOLE)
        radius = 0.1 + 0.2 * self._rand(n, N_SLOTS)
        sxy = (self._rand(n, N_SLOTS, 2) * 2 - 1) * (
            self.arena_half[:, None, :] - radius[..., None]
        )
        clear = (sxy - oxy[:, None]).norm(dim=-1) > radius + opp_foot[:, None] + 0.15
        present = present & clear
        self.slot_present = torch.where(m1, present, self.slot_present)
        self.slot_kind = torch.where(m1, kind, self.slot_kind)
        self.slot_r = torch.where(m1, radius, self.slot_r)
        self.slot_xy = torch.where(mask[:, None, None], sxy, self.slot_xy)
        for i in range(BLOCK_MOCAPS):
            is_block = self.slot_present[:, i] & (self.slot_kind[:, i] == obs_spec.WALL_BLOCK)
            pos = torch.cat(
                [self.slot_xy[:, i], torch.full((n, 1), WALL_HEIGHT_M / 2, device=self.device)], 1
            )
            parked = torch.tensor(
                [PARKED, PARKED + 10.0 * (i + 1), WALL_HEIGHT_M / 2], device=self.device
            ).expand(n, 3)
            self.mocap_pos[:, WALL_MOCAPS + i] = torch.where(
                m1, torch.where(is_block[:, None], pos, parked), self.mocap_pos[:, WALL_MOCAPS + i]
            )
            gi = self.block_geoms[i]
            bsize = torch.stack(
                [
                    self.slot_r[:, i],
                    torch.full((n,), WALL_HEIGHT_M / 2, device=self.device),
                    torch.zeros(n, device=self.device),
                ],
                1,
            )
            self.geom_size[:, gi] = torch.where(m1, bsize, self.geom_size[:, gi])

    def _reset_house(
        self, mask: torch.Tensor, oxy: torch.Tensor, opp_foot: torch.Tensor
    ) -> torch.Tensor:
        n = self.num_worlds
        m1 = mask[:, None]
        sc = self.stage_cfg
        house = self._rand(n) < float(sc.house)
        hxy = (self._rand(n, 2) * 2 - 1) * (self.arena_half - HOUSE_RADIUS_M - 0.05)
        house = house & ((hxy - oxy).norm(dim=1) > HOUSE_RADIUS_M + opp_foot + 0.15)
        self.house_present = torch.where(mask, house, self.house_present)
        hpos = torch.cat([hxy, torch.full((n, 1), 0.12, device=self.device)], 1)
        hparked = torch.tensor([-PARKED, PARKED, 0.12], device=self.device).expand(n, 3)
        self.mocap_pos[:, HOUSE_MOCAP] = torch.where(
            m1, torch.where(house[:, None], hpos, hparked), self.mocap_pos[:, HOUSE_MOCAP]
        )
        self.house_vel = torch.where(m1, torch.zeros_like(self.house_vel), self.house_vel)
        return hxy

    def _reset_our_start(
        self, mask: torch.Tensor, oxy: torch.Tensor, hxy: torch.Tensor
    ) -> torch.Tensor:
        n = self.num_worlds
        m1 = mask[:, None]
        # Clear of the opponent, hazards and house bot, by rejection with a few retries.
        our_reach = self.arena_half - self.our_half_diag - 0.05
        xy = (self._rand(n, 2) * 2 - 1) * our_reach
        for _ in range(12):
            bad = (xy - oxy).norm(dim=1) < 0.5
            bad |= (
                ((xy[:, None] - self.slot_xy).norm(dim=-1) < self.slot_r + 0.2)
                .logical_and(self.slot_present)
                .any(1)
            )
            bad |= self.house_present & ((xy - hxy).norm(dim=1) < HOUSE_RADIUS_M + 0.25)
            if not bool((bad & mask).any()):
                break
            xy = torch.where(bad[:, None], (self._rand(n, 2) * 2 - 1) * our_reach, xy)
        yaw = self._uniform(-math.pi, math.pi)
        pitch = self.collision.rest_pitch_rad
        qpos0 = torch.as_tensor(self.mjm.qpos0, device=self.device, dtype=self.dtype).expand(n, -1)
        q = qpos0.clone()
        q[:, 0], q[:, 1], q[:, 2] = xy[:, 0], xy[:, 1], self.height
        q[:, 3] = torch.cos(yaw / 2) * math.cos(pitch / 2)
        q[:, 4] = -torch.sin(yaw / 2) * math.sin(pitch / 2)
        q[:, 5] = torch.cos(yaw / 2) * math.sin(pitch / 2)
        q[:, 6] = torch.sin(yaw / 2) * math.cos(pitch / 2)
        self.qpos[:] = torch.where(m1, q, self.qpos)
        self.qvel[:] = torch.where(m1, torch.zeros_like(self.qvel), self.qvel)
        return xy

    def _reset_bookkeeping(self, mask: torch.Tensor, xy: torch.Tensor) -> None:
        n = self.num_worlds
        m1 = mask[:, None]
        sc = self.stage_cfg
        self.run_away = torch.where(mask, self._rand(n) < float(sc.run_away), self.run_away)
        self.episode_stage = torch.where(
            mask, torch.full_like(self.episode_stage, self.stage_index), self.episode_stage
        )
        self.episode_pursuit_level = torch.where(
            mask,
            torch.full_like(self.episode_pursuit_level, self.opponents.pursuit_level),
            self.episode_pursuit_level,
        )
        false = torch.zeros_like(self.engaged)
        for name in ("engaged", "struck", "prev_arc", "prev_wall", "prev_block"):
            setattr(self, name, torch.where(mask, false, getattr(self, name)))
        zero = torch.zeros_like(self.t)
        for name in ("t", "apart_s", "arc_s", "flip_s", "last_speed", "last_rate"):
            setattr(self, name, torch.where(mask, zero, getattr(self, name)))
        self.steps = torch.where(mask, torch.zeros_like(self.steps), self.steps)
        self.prev_action = torch.where(m1, torch.zeros_like(self.prev_action), self.prev_action)
        self.hist_action = torch.where(
            mask[:, None, None], torch.zeros_like(self.hist_action), self.hist_action
        )
        self.hist_speed = torch.where(m1, torch.zeros_like(self.hist_speed), self.hist_speed)
        self.hist_rate = torch.where(m1, torch.zeros_like(self.hist_rate), self.hist_rate)
        for k, v in self.stats.items():
            self.stats[k] = torch.where(mask, torch.zeros_like(v), v)
        inf = torch.full_like(self.first_strike_s, math.inf)
        self.first_strike_s = torch.where(mask, inf, self.first_strike_s)
        self.first_hit_s = torch.where(mask, inf, self.first_hit_s)
        self.fw_t[mask] = 0.0
        self.transport_ring_t[mask] = 0.0
        self.drive_ring_t[mask] = 0.0
        self.upside[mask] = 0
        self.opponents.history[mask] = 0.0
        self.opponents.history[mask, :, 0:2] = xy[mask, None, :]
        if self.sensing:
            self._reset_sensing(mask)

    def _randomize_plant(self, mask: torch.Tensor, s: float) -> None:
        n = self.num_worlds
        fit = self.fit
        m1 = mask[:, None]
        mu = self._uniform(*_scaled((0.6, 1.5), s))
        self.friction_scale = torch.where(mask, mu, self.friction_scale)
        fr = self.base["geom_friction"].expand(n, -1, -1).clone()
        for g in (*self.wheel_geoms, *self.skid_geoms):
            fr[:, g, 0] = fr[:, g, 0] * mu
            fr[:, g, 1] = fr[:, g, 1] * mu
        # The fit's rolling friction (7.9e-6) sits under MJ_MINMU; floor it there.
        fr = fr.clamp(min=1e-5)
        self.geom_friction[:] = torch.where(mask[:, None, None], fr, self.geom_friction)

        track = self._uniform(*_scaled((0.9, 1.1), s))
        bp = self.base["body_pos"].expand(n, -1, -1).clone()
        for b in (*self.wheel_bodies, *self.rotor_bodies):
            bp[:, b, 1] = bp[:, b, 1] * track
        self.body_pos[:] = torch.where(mask[:, None, None], bp, self.body_pos)

        cg = self._uniform(*_scaled((0.9, 1.2), s))
        ip = self.base["body_ipos"].expand(n, -1, -1).clone()
        ip[:, self.chassis_body, 2] = ip[:, self.chassis_body, 2] * cg
        self.body_ipos[:] = torch.where(mask[:, None, None], ip, self.body_ipos)
        inertia = self.base["body_inertia"].expand(n, -1, -1).clone()
        pitch_i = self._uniform(*_scaled((0.9, 1.2), s))
        inertia[:, self.chassis_body, 1] = inertia[:, self.chassis_body, 1] * pitch_i
        arm = self._uniform(1.0 - s * 0.1, 1.0 + s * 0.2) * fit.armature_scale
        axial = self.mp.rotor_inertia * arm
        for b in self.rotor_bodies:
            inertia[:, b] = torch.stack([0.6 * axial, axial, 0.6 * axial], 1)
        self.body_inertia[:] = torch.where(mask[:, None, None], inertia, self.body_inertia)

        log_lo, log_hi = math.log(1e-6), math.log(1e-3)
        damping = (
            torch.exp(self._uniform(log_lo, log_hi))
            if s > 0
            else torch.full((n,), fit.joint_damping, device=self.device)
        )
        dd = self.base["dof_damping"].expand(n, -1).clone()
        for dof in (*self.index.wheel_dofs, *self.index.follower_dofs):
            dd[:, int(dof)] = damping
        self.dof_damping[:] = torch.where(m1, dd, self.dof_damping)

        eff = (
            self._uniform(0.68, 0.80)
            if s > 0
            else torch.full((n,), fit.efficiency, device=self.device)
        )
        eff = fit.efficiency + s * (eff - fit.efficiency)
        res = fit.resistance_ohm * self._uniform(*_scaled((0.85, 1.15), s))
        nkt = self.mp.gear_ratio * self.mp.motor_kt
        gain = eff * nkt / res
        bias = -eff * self.mp.gear_ratio**2 * self.mp.motor_kt**2 / res
        limit = eff * nkt * fit.current_limit_a
        self.gainprm[:, :, 0] = torch.where(m1, gain[:, None].expand(-1, 2), self.gainprm[:, :, 0])
        self.biasprm[:, :, 2] = torch.where(m1, bias[:, None].expand(-1, 2), self.biasprm[:, :, 2])
        fr2 = torch.stack([-limit, limit], 1)[:, None, :].expand(-1, 2, -1)
        self.forcerange[:] = torch.where(mask[:, None, None], fr2, self.forcerange)

        root = math.sqrt(fit.lr_gain_ratio)
        veer = self._uniform(1.0 - s * 0.15, 1.0)
        side = self._rand(n) < 0.5
        pack_lo, pack_hi = self.cfg.pack_volts
        pack = 15.2 + s * (self._uniform(pack_lo, pack_hi) - 15.2)
        self.pack = torch.where(mask, pack, self.pack)
        shaping = torch.stack(
            [
                torch.full((n,), fit.deadzone_left, device=self.device),
                torch.full((n,), fit.deadzone_right, device=self.device),
                torch.full((n,), fit.curve, device=self.device),
                torch.full((n,), root, device=self.device),
                torch.full((n,), 1.0 / root, device=self.device),
                pack,
                torch.where(side, veer, torch.ones_like(veer)),
                torch.where(side, torch.ones_like(veer), veer),
                torch.full((n,), 0.0 if fit.brake_on_zero else 1.0, device=self.device),
            ],
            1,
        )
        self.shaping_t[:] = torch.where(m1, shaping, self.shaping_t)

        lo, hi = self.cfg.latency_s
        latency = 0.5 * (lo + hi) + s * (self._uniform(lo, hi) - 0.5 * (lo + hi))
        self.latency = torch.where(mask, latency, self.latency)
        dt = self.cfg.timestep_s
        drive = max(0, round(fit.delay_s / dt))
        transport = torch.round((latency - fit.delay_s) / dt).long().clamp(min=0).to(torch.int32)
        self.transport_lag_t[:] = torch.where(mask, transport, self.transport_lag_t)
        self.drive_lag_t[:] = torch.where(
            mask, torch.full_like(self.drive_lag_t, drive), self.drive_lag_t
        )

    def _reset_sensing(self, mask: torch.Tensor) -> None:
        now = self.t
        self.measure.reset(mask, now)
        self.ekf.reset(mask, now, torch.zeros_like(now))
        f = self.opp_filter
        f.active = f.active & ~mask
        f.heading_initialized = f.heading_initialized & ~mask
        f.output_stale = f.output_stale & ~mask
        zero = torch.zeros_like(f.consecutive_flips)
        f.consecutive_flips = torch.where(mask, zero, f.consecutive_flips)
        f.consecutive_rejects = torch.where(mask, zero, f.consecutive_rejects)
        f.snap_count = torch.where(mask, zero, f.snap_count)
        f.snap_start = torch.where(mask, zero, f.snap_start)
        f.stamp = torch.where(mask, now, f.stamp)
        f.last_measured = torch.where(mask, now - 1.0, f.last_measured)

    def export_scenarios(self) -> list[dict[str, Any]]:
        """Each world's current episode as a plain dict `LockstepWorld` can replay. Call right
        after `reset(seed)`."""
        out = []
        p = self.opponents.p
        osize = self.geom_size[:, self.opp_geom] * 2
        q = self.qpos
        yaw = torch.atan2(
            2 * (q[:, 3] * q[:, 6] + q[:, 4] * q[:, 5]), 1 - 2 * (q[:, 5] ** 2 + q[:, 6] ** 2)
        )
        opp = self.mocap_pos[:, OPP_MOCAP]
        oq = self.mocap_quat[:, OPP_MOCAP]
        oyaw = torch.atan2(
            2 * (oq[:, 0] * oq[:, 3] + oq[:, 1] * oq[:, 2]), 1 - 2 * (oq[:, 2] ** 2 + oq[:, 3] ** 2)
        )
        house = self.mocap_pos[:, HOUSE_MOCAP]
        to = lambda v: v.detach().cpu().tolist()  # noqa: E731
        cols = {
            k: to(getattr(p, k))
            for k in (
                "behaviour",
                "speed",
                "radius",
                "direction",
                "noise",
                "lead",
                "delay_steps",
                "turn_rate",
                "corner",
                "grip",
            )
        }
        for i in range(self.num_worlds):
            out.append(
                {
                    "seed": i,
                    "robot": self.cfg.robot,
                    "opp_model": self.profile.opponents[int(self.opp_model[i])],
                    "driver": int(self.policy_opp.driver[i]),
                    "holdout": self.cfg.holdout,
                    "arena_half": to(self.arena_half[i]),
                    "our_pose": [float(q[i, 0]), float(q[i, 1]), float(yaw[i])],
                    "opp_pose": [float(opp[i, 0]), float(opp[i, 1]), float(oyaw[i])],
                    "opp_size": to(osize[i]),
                    "opp_params": {k: v[i] for k, v in cols.items()},
                    "replay_index": int(self.opponents.replay_index[i]),
                    "replay_t": float(self.opponents.replay_t[i]),
                    "slots": [
                        {
                            "present": bool(self.slot_present[i, k]),
                            "kind": int(self.slot_kind[i, k]),
                            "x": float(self.slot_xy[i, k, 0]),
                            "y": float(self.slot_xy[i, k, 1]),
                            "r": float(self.slot_r[i, k]),
                        }
                        for k in range(N_SLOTS)
                    ],
                    "house": {
                        "present": bool(self.house_present[i]),
                        "x": float(house[i, 0]),
                        "y": float(house[i, 1]),
                    },
                    "run_away": float(self.run_away[i]),
                    "latency": float(self.latency[i]),
                    "episode_s": self.cfg.episode_s,
                }
            )
        return out

    # ------------------------------------------------------------------ truth

    def truth(self) -> dict[str, torch.Tensor]:
        q = self.qpos
        w_, x_, y_, z_ = q[:, 3], q[:, 4], q[:, 5], q[:, 6]
        yaw = torch.atan2(2 * (w_ * z_ + x_ * y_), 1 - 2 * (y_ * y_ + z_ * z_))
        pitch = torch.asin((2 * (w_ * y_ - z_ * x_)).clamp(-1.0, 1.0))
        up_z = 1 - 2 * (x_ * x_ + y_ * y_)
        vx, vy = self.qvel[:, 0], self.qvel[:, 1]
        # World yaw rate: rotate the body angular velocity; its z row is the third row of R.
        r20 = 2 * (x_ * z_ - w_ * y_)
        r21 = 2 * (y_ * z_ + w_ * x_)
        r22 = 1 - 2 * (x_ * x_ + y_ * y_)
        omega = r20 * self.qvel[:, 3] + r21 * self.qvel[:, 4] + r22 * self.qvel[:, 5]
        op = self.mocap_pos[:, OPP_MOCAP]
        oq = self.mocap_quat[:, OPP_MOCAP]
        oyaw = torch.atan2(
            2 * (oq[:, 0] * oq[:, 3] + oq[:, 1] * oq[:, 2]), 1 - 2 * (oq[:, 2] ** 2 + oq[:, 3] ** 2)
        )
        # Clones: the qpos and mocap tensors are live views the next physics launch overwrites.
        return {
            "xy": q[:, :2].clone(),
            "yaw": yaw,
            "pitch": pitch,
            "up_z": up_z,
            "velocity": torch.stack([vx, vy], 1).clone(),
            "speed": vx * torch.cos(yaw) + vy * torch.sin(yaw),
            "yaw_rate": omega,
            "opp_xy": op[:, :2].clone(),
            "opp_yaw": oyaw,
        }

    def expert_view(self) -> dict[str, torch.Tensor]:
        """`truth()` plus `reach`: the centre distance at which our footprint and the opponent's
        touch along the line between them. The scripted sidestep controller reads this."""
        tr = self.truth()
        rel = tr["xy"] - tr["opp_xy"]
        normal = rel / rel.norm(dim=1, keepdim=True).clamp(min=1e-6)
        ours = torch.tensor(self.our_half, device=self.device, dtype=self.dtype)
        tr["reach"] = box_support(normal, tr["opp_yaw"], self.opp_half) + box_support(
            normal, tr["yaw"], ours.expand(self.num_worlds, 2)
        )
        return tr

    # ------------------------------------------------------------------ stepping

    def _substeps(self) -> None:
        wp, mjw, kern = self._wp, self._mjw, self._k
        d = self.d
        dt = self.cfg.timestep_s
        for _ in range(self.substeps):
            wp.launch(
                kern.command_path,
                dim=self.num_worlds,
                inputs=[
                    self.head,
                    self.cmd,
                    self.transport_ring,
                    self.transport_lag,
                    self.drive_ring,
                    self.drive_lag,
                    self.fw,
                    d.qpos,
                    d.qvel,
                    self.wheel_dofs,
                    self.shaping,
                    self.emf,
                    dt,
                    self.upside_wp,
                    self.direct,
                ],
                outputs=[d.ctrl],
            )
            wp.launch(
                kern.move_mocap,
                dim=(self.num_worlds, 2),
                inputs=[d.mocap_pos, d.mocap_quat, self.moving, self.mover_vel, dt],
            )
            mjw.step(self.m, d)
            wp.launch(
                kern.classify_contacts,
                dim=d.naconmax,
                inputs=[
                    d.nacon,
                    d.contact.geom,
                    d.contact.pos,
                    d.contact.worldid,
                    self.geom_role,
                    d.qpos,
                    d.mocap_pos,
                    d.mocap_quat,
                    OPP_MOCAP,
                    self.arc,
                    self.cfg.strike_x_m,
                    self.cfg.strike_half_width_m,
                    self.weapon_kind,
                    self.sweep,
                    self.sweep_y,
                    self.m.geom_size,
                    self.opp_geom,
                    self.arc_rule,
                ],
                outputs=[self.flags_wp],
            )
            wp.launch(kern.advance, dim=1, inputs=[self.head])

    def _physics(self) -> None:
        wp = self._wp
        if self._graph is None:
            self._substeps()  # compile outside the capture
            with wp.ScopedCapture() as capture:
                self._substeps()
            self._graph = capture.graph
        wp.capture_launch(self._graph)

    def _measure(self, tr: dict[str, torch.Tensor]) -> None:
        n = self.num_worlds
        xy = torch.stack([tr["xy"], tr["opp_xy"]], 1)
        yaw = torch.stack([tr["yaw"], tr["opp_yaw"]], 1)
        opp_v = self.mover_vel_t[:, 0, :2]
        vel = torch.stack([tr["velocity"], opp_v], 1)
        rate = torch.stack([tr["yaw_rate"], self.mover_vel_t[:, 0, 2]], 1)
        ctx = Context.from_rect_arena(xy, 0.1, self.arena_half)
        out = self.measure.step(self.t, TruePoses(xy=xy, yaw=yaw, velocity=vel, yaw_rate=rate), ctx)
        f = self.opp_filter
        before = f.output_stale.clone()
        var = torch.full((n,), 0.012**2, device=self.device, dtype=self.dtype)
        key = torch.ones(n, dtype=torch.bool, device=self.device)
        f.update(out.stamp, out.measured[:, 1], out.xy[:, 1], out.yaw[:, 1], key, var)
        f.output_stale = torch.where(out.delivered, f.output_stale, before)

    def step(self, action: torch.Tensor) -> Step:
        cfg, rw = self.cfg, self.cfg.reward
        dt = cfg.policy_dt_s
        action = action.to(self.device, self.dtype).clamp(-1.0, 1.0)
        # History: this step's action with the speed and yaw rate the policy saw when choosing it.
        self.hist_action = torch.cat([action[:, None], self.hist_action[:, :-1]], 1)
        self.hist_speed = torch.cat([self.last_speed[:, None], self.hist_speed[:, :-1]], 1)
        self.hist_rate = torch.cat([self.last_rate[:, None], self.hist_rate[:, :-1]], 1)
        self.cmd_t[:] = saturate(action)

        tr0 = self.truth()
        phi0 = self._potential(tr0)
        self.opponents.record_us(tr0["xy"], tr0["velocity"], tr0["yaw"])
        self.mover_vel_t[:, 0] = self.opponents.command(
            tr0["opp_xy"],
            tr0["opp_yaw"],
            self.arena_half,
            self.opp_half.norm(dim=1),
            dt,
            opp_half=self.opp_half,
            override=self._policy_opponent_command(tr0) if self.policy_opp.available else None,
        )
        self._house_command()
        self.flags.zero_()
        for _ in range(self.launches):
            self._physics()
            self.t = self.t + cfg.measure_period_s
            if self.sensing:
                self._measure(self.truth())

        tr = self.truth()
        f = self.flags
        contact = f[:, 0] > 0.5
        arc = f[:, 1] > 0.5
        wedge = f[:, 2] > 0.5
        attack = ~self.run_away
        reward = torch.zeros(self.num_worlds, device=self.device, dtype=self.dtype)

        starting = contact & ~self.engaged
        strike = starting & wedge & ~arc & attack
        u = (f[:, 3] - self.arc) / (math.pi - self.arc)
        reward += torch.where(strike, rw["strike_base"] + rw["strike_rear"] * smoothstep(u), 0.0)
        self.arc_s = torch.where(arc, self.arc_s + dt, torch.zeros_like(self.arc_s))
        hit = arc & (~self.prev_arc | (self.arc_s >= cfg.arc_rehit_s))
        self.arc_s = torch.where(hit, torch.zeros_like(self.arc_s), self.arc_s)
        reward += torch.where(hit, rw["hit_taken"], 0.0)
        self.prev_arc = arc
        self.stats["strikes"] += strike.float()
        self.stats["rear_strikes"] += (strike & (f[:, 3] > 2 * math.pi / 3)).float()
        self.stats["hits_taken"] += hit.float()
        self.first_strike_s = torch.where(
            strike & torch.isinf(self.first_strike_s), self.t, self.first_strike_s
        )
        self.first_hit_s = torch.where(
            hit & torch.isinf(self.first_hit_s), self.t, self.first_hit_s
        )
        self.struck = self.struck | strike
        self.engaged = self.engaged | contact
        self.apart_s = torch.where(contact, torch.zeros_like(self.apart_s), self.apart_s + dt)
        ended = self.engaged & (self.apart_s >= cfg.engagement_gap_s)
        self.engaged &= ~ended
        self.struck &= ~ended
        pushing = wedge & ~arc & self.struck & attack
        reward += torch.where(pushing, rw["push_per_s"] * dt, 0.0)

        rel = tr["xy"] - tr["opp_xy"]
        phi_us = wrap(torch.atan2(rel[:, 1], rel[:, 0]) - tr["opp_yaw"])
        reach = self.opp_half.norm(dim=1) + self.our_half_diag + cfg.danger_margin_m
        danger = ~contact & (phi_us.abs() < self.arc) & (rel.norm(dim=1) < reach)
        reward += torch.where(danger, rw["danger_per_s"] * dt, 0.0)
        self.stats["danger_s"] += danger.float() * dt

        # Pushing or pinning the opponent with our wedge from outside its arc is slow on purpose
        # and exempt. Any other contact is not: pinned in its arc is the worst place to sit.
        pushing_wedge = wedge & ~arc
        still = ~pushing_wedge & (tr["velocity"].norm(dim=1) < cfg.still_speed_mps)
        reward += torch.where(still, rw["still_per_s"] * dt, 0.0)
        self.stats["still_s"] += still.float() * dt

        wall = f[:, 4] > 0.5
        wall_event = wall & ~self.prev_wall & ~(wedge & contact)
        reward += torch.where(wall_event, rw["wall_event"], 0.0)
        self.stats["wall_events"] += wall_event.float()
        self.prev_wall = wall
        block = (f[:, 5] > 0.5) | (f[:, 6] > 0.5)
        block_event = block & ~self.prev_block
        reward += torch.where(block_event, rw["block_event"], 0.0)
        self.stats["block_contacts"] += block_event.float()
        self.prev_block = block

        is_hole = self.slot_present & (self.slot_kind == obs_spec.HOLE)
        edge = (tr["xy"][:, None] - self.slot_xy).norm(dim=-1) - self.slot_r
        in_hole = ((edge < 0.0) & is_hole).any(1)
        reward += torch.where(in_hole, rw["hole"], 0.0)
        near = torch.where(is_hole, 1.0 - edge / cfg.hole_margin_m, 0.0).clamp(0.0, 1.0)
        reward += rw["hole_near_per_s"] * dt * near.amax(1)
        self.stats["holes"] += in_hole.float()
        tipped = (tr["up_z"] < 0.0) | (tr["pitch"].abs() > math.radians(80))
        self.flip_s = torch.where(tipped, self.flip_s + dt, torch.zeros_like(self.flip_s))
        flipped = self.flip_s >= cfg.flip_hold_s
        reward += torch.where(flipped, rw["flip"], 0.0)
        self.stats["flips"] += flipped.float()
        # Pitch is positive nose-down (rollout.py), so a lifted nose reads negative.
        lift = tr["pitch"] < -math.radians(30)
        reward += torch.where(lift, rw["nose_lift_per_s"] * dt, 0.0)
        self.stats["nose_lift_s"] += lift.float() * dt

        # A world whose solver diverged (rare: a hard wheelie at the iteration limit) ends with no
        # reward for the step, so a physics artifact never becomes a training signal.
        diverged = ~torch.isfinite(self.qpos).all(1) | (self.qvel.abs().amax(1) > 1e4)
        self.stats["sim_failures"] += diverged.float()
        terminated = in_hole | flipped | diverged
        phi1 = torch.where(terminated, torch.zeros_like(phi0), self._potential(tr))
        reward += rw["shaping"] * (cfg.gamma * phi1 - phi0)
        reward += rw["command_change"] * (action - self.prev_action).abs().sum(1)
        reward = torch.where(diverged | ~torch.isfinite(reward), torch.zeros_like(reward), reward)
        self.prev_action = action
        self.stats["reward"] += reward
        self.steps += 1
        truncated = (self.steps >= self.max_steps) & ~terminated

        obs, priv = self._observe(tr)
        done = terminated | truncated
        episode: dict[str, torch.Tensor] = {}
        final_obs, final_priv = obs, priv
        if bool(done.any()):
            episode = {k: v.clone() for k, v in self.stats.items()}
            episode["first_strike_s"] = self.first_strike_s.clone()
            episode["first_hit_s"] = self.first_hit_s.clone()
            episode["run_away"] = self.run_away.float()
            episode["behaviour"] = self.opponents.p.behaviour.float()
            episode["driver"] = self.policy_opp.driver.float()
            episode["opp_speed"] = self.opponents.p.speed.clone()
            episode["opp_turn_rate"] = self.opponents.p.turn_rate.clone()
            episode["opp_delay_s"] = self.opponents.p.delay_steps.float() * cfg.policy_dt_s
            episode["seconds"] = self.steps.float() * self.spec.policy_dt_s
            self.record_success(episode, done)
            self._reset_worlds(done)
            obs2, priv2 = self._observe()
            obs = torch.where(done[:, None], obs2, obs)
            priv = torch.where(done[:, None], priv2, priv)
        return Step(
            obs=obs,
            privileged=priv,
            reward=reward,
            terminated=terminated,
            truncated=truncated,
            final_obs=final_obs,
            final_privileged=final_priv,
            episode=episode,
        )

    def _policy_opponent_command(
        self, tr: dict[str, torch.Tensor]
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Policy-driven opponents' velocities: each sees the arena from its side, exactly."""
        n = self.num_worlds
        zeros = torch.zeros(n, device=self.device, dtype=self.dtype)
        false = torch.zeros(n, dtype=torch.bool, device=self.device)
        hz_xy, hz_r, hz_kind, hz_vel, hz_present = self._hazards()
        inp = obs_spec.ObsInputs(
            our_xy=tr["opp_xy"],
            our_yaw=tr["opp_yaw"],
            our_speed=self.policy_opp.v,
            our_yaw_rate=self.policy_opp.w_rate,
            our_age_s=zeros,
            our_stale=false,
            run_away=false,
            hist_action=self.policy_opp.hist_action,
            hist_speed=self.policy_opp.hist_speed,
            hist_yaw_rate=self.policy_opp.hist_rate,
            opp_present=~false,
            opp_xy=tr["xy"],
            opp_yaw=tr["yaw"],
            opp_velocity=tr["velocity"],
            opp_yaw_rate=tr["yaw_rate"],
            opp_position_sigma=zeros,
            opp_heading_sigma=zeros,
            opp_yaw_rate_sigma=zeros,
            opp_half_diagonal=zeros + self.our_half_diag,
            opp_age_s=zeros,
            opp_stale=false,
            field_half=self.arena_half,
            hazard_xy=hz_xy,
            hazard_radius=hz_r,
            hazard_kind=hz_kind,
            hazard_velocity=hz_vel,
            hazard_present=hz_present,
        )
        velocity, active = self.policy_opp.command(inp, tr["opp_yaw"])
        return active, velocity

    def _hazards(
        self,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """Holes, wall blocks and the house bot, as the observation takes them."""
        n = self.num_worlds
        hz_xy = torch.cat([self.slot_xy, self.mocap_pos[:, HOUSE_MOCAP, None, :2]], 1)
        hz_r = torch.cat([self.slot_r, torch.full((n, 1), HOUSE_RADIUS_M, device=self.device)], 1)
        hz_kind = torch.cat(
            [
                self.slot_kind,
                torch.full((n, 1), obs_spec.TRACKED, device=self.device, dtype=torch.long),
            ],
            1,
        )
        hz_vel = torch.cat([torch.zeros_like(self.slot_xy), self.mover_vel_t[:, 1, None, :2]], 1)
        hz_present = torch.cat([self.slot_present, self.house_present[:, None]], 1)
        return hz_xy, hz_r, hz_kind, hz_vel, hz_present

    def _house_command(self) -> None:
        n = self.num_worlds
        noise = torch.randn(n, 2, generator=self.gen, device=self.device, dtype=self.dtype)
        self.house_vel = 0.95 * self.house_vel + 0.05 * noise * 0.4
        hp = self.mocap_pos[:, HOUSE_MOCAP, :2]
        lim = self.arena_half - HOUSE_RADIUS_M
        out = ((hp >= lim) & (self.house_vel > 0)) | ((hp <= -lim) & (self.house_vel < 0))
        self.house_vel = torch.where(out, -self.house_vel, self.house_vel)
        v = torch.where(
            self.house_present[:, None], self.house_vel, torch.zeros_like(self.house_vel)
        )
        self.mover_vel_t[:, 1, :2] = v
        self.mover_vel_t[:, 1, 2] = 0.0

    def _potential(self, tr: dict[str, torch.Tensor]) -> torch.Tensor:
        heading = torch.stack([torch.cos(tr["opp_yaw"]), torch.sin(tr["opp_yaw"])], 1)
        stage = tr["opp_xy"] - self.cfg.stage_offset_m * heading
        front = tr["opp_xy"] + self.cfg.stage_offset_m * heading
        attack = -(tr["xy"] - stage).norm(dim=1) + self.cfg.reward["flank"] * flank_potential(
            tr["xy"] - tr["opp_xy"], tr["opp_yaw"], self.arc
        )
        flee = (tr["xy"] - front).norm(dim=1).clamp(max=1.5)
        return torch.where(self.run_away, flee, attack)

    # ------------------------------------------------------------------ observation

    def _our_last_delivery(self) -> torch.Tensor:
        """When our robot's last measurement reached the filter: its capture stamp plus the
        latency. `OurEkfErrorModel` starts a gap from this, as the real EKF does from its last
        update; the capture stamp alone is always a latency old and never leaves the gap."""
        stamp: torch.Tensor = self.measure.last_measured_stamp[:, 0] + self.measure.params.latency_s
        return stamp

    def _observe(
        self, tr: dict[str, torch.Tensor] | None = None
    ) -> tuple[torch.Tensor, torch.Tensor]:
        tr = tr or self.truth()
        n = self.num_worlds
        zeros = torch.zeros(n, device=self.device, dtype=self.dtype)
        false = torch.zeros(n, dtype=torch.bool, device=self.device)
        opp_v = self.mover_vel_t[:, 0, :2]
        if self.sensing:
            ours = self.ekf.step(
                self.t,
                tr["xy"],
                tr["yaw"],
                tr["velocity"],
                tr["yaw_rate"],
                self._our_last_delivery(),
                self.measure.params.frame_period_s,
            )
            our_xy, our_yaw, our_v, our_w, our_stale = (
                ours.xy,
                ours.yaw,
                ours.forward_speed,
                ours.yaw_rate,
                ours.stale,
            )
            our_age = (self.t - self.measure.last_measured_stamp[:, 0]).clamp(0.0, 10.0)
            est = self.opp_filter.coast(self.t)
            present = est.active
            opp_xy = torch.stack([est.x, est.y], 1)
            opp_yaw, opp_vel, opp_w = est.yaw, torch.stack([est.vx, est.vy], 1), est.omega
            sig = (est.position_sigma_m, est.heading_sigma_rad, est.yaw_rate_sigma_rad_s)
            opp_age = (self.t - self.opp_filter.last_measured).clamp(0.0, 10.0)
            opp_stale = est.is_stale
        else:
            our_xy, our_yaw, our_v, our_w, our_stale = (
                tr["xy"],
                tr["yaw"],
                tr["speed"],
                tr["yaw_rate"],
                false,
            )
            our_age = zeros
            present = ~false
            opp_xy, opp_yaw, opp_vel, opp_w = (
                tr["opp_xy"],
                tr["opp_yaw"],
                opp_v,
                self.mover_vel_t[:, 0, 2],
            )
            sig = (zeros, zeros, zeros)
            opp_age, opp_stale = zeros, false
        self.last_speed, self.last_rate = our_v, our_w

        hz_xy, hz_r, hz_kind, hz_vel, hz_present = self._hazards()
        inp = obs_spec.ObsInputs(
            our_xy=our_xy,
            our_yaw=our_yaw,
            our_speed=our_v,
            our_yaw_rate=our_w,
            our_age_s=our_age,
            our_stale=our_stale,
            run_away=self.run_away,
            hist_action=self.hist_action,
            hist_speed=self.hist_speed,
            hist_yaw_rate=self.hist_rate,
            opp_present=present,
            opp_xy=opp_xy,
            opp_yaw=opp_yaw,
            opp_velocity=opp_vel,
            opp_yaw_rate=opp_w,
            opp_position_sigma=sig[0],
            opp_heading_sigma=sig[1],
            opp_yaw_rate_sigma=sig[2],
            opp_half_diagonal=self.opp_half.norm(dim=1),
            opp_age_s=opp_age,
            opp_stale=opp_stale,
            field_half=self.arena_half,
            hazard_xy=hz_xy,
            hazard_radius=hz_r,
            hazard_kind=hz_kind,
            hazard_velocity=hz_vel,
            hazard_present=hz_present,
        )
        obs = obs_spec.build_v1(inp)

        c, s_ = torch.cos(tr["yaw"]), torch.sin(tr["yaw"])
        rel = tr["opp_xy"] - tr["xy"]
        heading = wrap(tr["opp_yaw"] - tr["yaw"])
        phi = wrap(torch.atan2(-rel[:, 1], -rel[:, 0]) - tr["opp_yaw"])
        onehot = torch.nn.functional.one_hot(self.opponents.p.behaviour, len(BEHAVIOURS)).to(
            self.dtype
        )
        priv = torch.cat(
            [
                torch.stack(
                    [
                        (c * rel[:, 0] + s_ * rel[:, 1]) / 2.0,
                        (-s_ * rel[:, 0] + c * rel[:, 1]) / 2.0,
                        torch.sin(heading),
                        torch.cos(heading),
                        torch.sin(phi),
                        torch.cos(phi),
                        tr["speed"] / 2.5,
                        tr["yaw_rate"] / 15.0,
                        tr["pitch"],
                        self.engaged.to(self.dtype),
                        self.struck.to(self.dtype),
                        self.latency / 0.1,
                        self.friction_scale,
                        self.pack / 15.2,
                        self.steps.to(self.dtype) / self.max_steps,
                        self.run_away.to(self.dtype),
                    ],
                    1,
                ),
                onehot,
                (self.opponents.p.speed / 2.5)[:, None],
            ],
            1,
        )
        if self.policy_opp.available:
            drivers = torch.nn.functional.one_hot(
                self.policy_opp.driver, len(self.policy_opp.names)
            ).to(self.dtype)
            priv = torch.cat([priv, drivers], 1)
        return obs, priv
