"""One RL evaluation scenario on CPU MuJoCo, for the lockstep sim (plan gates G4 and G5).

The Warp environment exports its held-out evaluation episodes as scenarios
(`WarpNavEnv.export_scenarios`); this module plays one of them against whatever drives the
robot, so the real C++ binary (`RlPolicyNavigation` or `MotionProfileNavigation`) runs the same
episodes the Python evaluation scored:

- the same model: `warp_nav.build_model` (fitted MJCF, mocap walls, blocks, house bot, opponent)
- the same command chain, scalar: transport delay, `FirmwareMixer` (Mr Stabs Mk2) or the radio's
  direct mix (Mrs Buff Mk3), drivetrain delay, shaping
- the same opponent behaviours (`opponents.Opponents` with one world on the CPU), and the same
  policy-driven opponents (`policy_opponents.PolicyOpponents`) for scenarios that drew one

The robot is the scenario's (`robot`, Mr Stabs Mk2 when absent).
- the same contact classification and episode scoring as `WarpNavEnv.step`, tick by tick
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any

import mujoco
import numpy as np
import torch

from auto_battlebot.mujoco_sim import mass_properties
from auto_battlebot.mujoco_sim.actuator import (
    NOMINAL_PACK_V,
    MotorConstants,
    shape_command,
    side_gains,
)
from auto_battlebot.mujoco_sim.closed_loop import load_fit_params
from auto_battlebot.mujoco_sim.firmware import FirmwareMixer, gyro_from_yaw_rate, heading_from_yaw
from auto_battlebot.mujoco_sim.mjcf import (
    CHASSIS_GEOM_PREFIX,
    FOLLOWER_GEOMS,
    SKID_GEOM,
    WHEEL_GEOMS,
    CollisionSet,
)
from auto_battlebot.mujoco_sim.rollout import ModelIndex, pose_from_qpos, rest_height
from auto_battlebot.rl import obs_spec
from auto_battlebot.rl.env import EnvSpec
from auto_battlebot.rl.envs.opponents import OpponentParams, Opponents
from auto_battlebot.rl.envs.policy_opponents import PolicyOpponents
from auto_battlebot.rl.envs.replay_tracks import ReplayTracks
from auto_battlebot.rl.envs.warp_nav import (
    BLOCK_MOCAPS,
    FLANK_RANGE_M,
    HOUSE_MOCAP,
    HOUSE_RADIUS_M,
    OPP_MOCAP,
    REPO,
    ROBOT_SIZE_M,
    ROBOTS,
    WALL_HEIGHT_M,
    WALL_MOCAPS,
    WALL_T_M,
    WarpNavConfig,
    build_model,
    resolved_fit_file,
    smoothstep,
)


def geom_role(name: str) -> int:
    """`classify_contacts` roles: 1 our robot, 2 opponent, 3 wall, 4 wall block, 5 house bot."""
    ours = name in WHEEL_GEOMS or name in FOLLOWER_GEOMS or name == SKID_GEOM
    if ours or name.startswith(CHASSIS_GEOM_PREFIX):
        return 1
    roles = {"opponent": 2, "house_bot": 5}
    if name in roles:
        return roles[name]
    if name.startswith("arena_wall_"):
        return 3
    if name.startswith("wall_block_"):
        return 4
    return 0


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class EpisodeScore:
    """The `WarpNavEnv` per-episode totals, accumulated one tick at a time."""

    strikes: float = 0.0
    rear_strikes: float = 0.0
    hits_taken: float = 0.0
    danger_s: float = 0.0
    wall_events: float = 0.0
    block_contacts: float = 0.0
    holes: float = 0.0
    flips: float = 0.0
    nose_lift_s: float = 0.0
    still_s: float = 0.0
    sim_failures: float = 0.0
    reward: float = 0.0
    first_strike_s: float = math.inf
    first_hit_s: float = math.inf
    run_away: float = 0.0
    behaviour: float = 0.0
    seconds: float = 0.0
    episode_s: float = 20.0  # the time limit, so a scorer can fill "never struck" with it
    outcome: str = "TIMEOUT"
    extra: dict[str, Any] = field(default_factory=dict)


class LockstepWorld:
    def __init__(
        self, scenario: dict[str, Any], cfg: WarpNavConfig | None = None, timestep: float = 1e-3
    ) -> None:
        # Scenarios record the episode length they were exported with; older ones ran 20 s.
        self.cfg = cfg or WarpNavConfig(
            episode_s=float(scenario.get("episode_s", 20.0)),
            robot=str(scenario.get("robot", "mr_stabs_mk2")),
        )
        self.sc = scenario
        self.timestep = timestep
        self.profile = ROBOTS[self.cfg.robot]
        asset = mass_properties.asset_dir(self.cfg.robot)
        mp = mass_properties.load(asset / "mass_properties.toml")
        collision = CollisionSet.load(asset / "collision")
        self.mp = mp
        size = ROBOT_SIZE_M[self.cfg.robot]
        self.our_half = (size[0] / 2, size[1] / 2)
        self.our_half_diag = math.hypot(*self.our_half)
        fit = load_fit_params(REPO / resolved_fit_file(self.cfg))
        fit = fit.replace(wheel_mu_roll=max(fit.wheel_mu_roll, 1e-5))
        self.fit = fit
        self.model = build_model(mp, fit, collision, timestep, self.cfg.robot)
        self.data = mujoco.MjData(self.model)
        self.index = ModelIndex.of(self.model)
        self.motor = MotorConstants(gear_ratio=mp.gear_ratio, kt=mp.motor_kt)
        names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
            for i in range(self.model.ngeom)
        ]
        self.role = np.array([geom_role(n) for n in names], dtype=int)
        self._build_scene(scenario, names)
        self._place_robot(scenario, mp, collision)
        self._init_command_chain(scenario)
        self._init_opponent(scenario)
        p = scenario["opp_params"]
        self.score = EpisodeScore(
            run_away=float(scenario.get("run_away", 0.0)),
            behaviour=float(p["behaviour"]),
            episode_s=self.cfg.episode_s,
        )
        self.t = 0.0
        self.engaged = self.struck = self.prev_arc = self.prev_wall = self.prev_block = False
        self.apart_s = 0.0
        self.arc_s = 0.0
        self.flip_s = 0.0
        self.done = False
        self.arc = math.radians(self.cfg.arc_half_angle_deg)

    def _build_scene(self, scenario: dict[str, Any], names: list[str]) -> None:
        half = np.asarray(scenario["arena_half"], dtype=float)
        self.half = half
        t = WALL_T_M / 2
        walls = (
            (half[0] + t, 0.0, half[1] + 2 * t, 0),
            (-half[0] - t, 0.0, half[1] + 2 * t, 0),
            (0.0, half[1] + t, half[0] + 2 * t, 1),
            (0.0, -half[1] - t, half[0] + 2 * t, 1),
        )
        for i, (x, y, length, axis) in enumerate(walls):
            g = names.index(f"arena_wall_{i}")
            size = [t, length, WALL_HEIGHT_M / 2] if axis == 0 else [length, t, WALL_HEIGHT_M / 2]
            self.model.geom_size[g] = size
            self.data.mocap_pos[i] = [x, y, WALL_HEIGHT_M / 2]
        self.slots = scenario["slots"]
        for i in range(BLOCK_MOCAPS):
            s = self.slots[i]
            g = names.index(f"wall_block_{i}")
            if s["present"] and s["kind"] == obs_spec.WALL_BLOCK:
                self.model.geom_size[g] = [s["r"], WALL_HEIGHT_M / 2, 0.0]
                self.data.mocap_pos[WALL_MOCAPS + i] = [s["x"], s["y"], WALL_HEIGHT_M / 2]
        osize = np.asarray(scenario["opp_size"], dtype=float)
        self.opp_geom = names.index("opponent")
        self.model.geom_size[self.opp_geom] = osize / 2
        self.opp_half = osize[:2] / 2
        ox, oy, oyaw = scenario["opp_pose"]
        self.data.mocap_pos[OPP_MOCAP] = [ox, oy, osize[2] / 2]
        self.data.mocap_quat[OPP_MOCAP] = [math.cos(oyaw / 2), 0, 0, math.sin(oyaw / 2)]
        self.house = scenario["house"]
        if self.house["present"]:
            self.data.mocap_pos[HOUSE_MOCAP] = [self.house["x"], self.house["y"], 0.12]
        self.house_v = np.zeros(2)
        self.rng = np.random.default_rng(int(scenario.get("seed", 0)))

    def _place_robot(self, scenario: dict[str, Any], mp: Any, collision: CollisionSet) -> None:
        x, y, yaw = scenario["our_pose"]
        pitch = collision.rest_pitch_rad
        q = self.data.qpos
        q[0], q[1], q[2] = x, y, rest_height(mp, collision, self.timestep)
        q[3] = math.cos(yaw / 2) * math.cos(pitch / 2)
        q[4] = -math.sin(yaw / 2) * math.sin(pitch / 2)
        q[5] = math.cos(yaw / 2) * math.sin(pitch / 2)
        q[6] = math.sin(yaw / 2) * math.cos(pitch / 2)
        mujoco.mj_forward(self.model, self.data)

    def _init_command_chain(self, scenario: dict[str, Any]) -> None:
        fit, timestep = self.fit, self.timestep
        gain_l, gain_r = side_gains(fit.lr_gain_ratio)
        self.side_gain = np.array([float(gain_l), float(gain_r)])
        self.deadzone = np.array([fit.deadzone_left, fit.deadzone_right])
        self.mixer = FirmwareMixer(auto_steer=True)
        self.direct = self.profile.command == "direct"
        latency = float(scenario.get("latency", 0.06))
        transport = max(0, round((latency - fit.delay_s) / timestep))
        drive = max(0, round(fit.delay_s / timestep))
        self.transport: deque[tuple[float, float]] = deque([(0.0, 0.0)] * transport)
        self.drive: deque[tuple[float, float]] = deque([(0.0, 0.0)] * drive)
        self.upside = False

    def _init_opponent(self, scenario: dict[str, Any]) -> None:
        self.opponents = Opponents(
            1,
            torch.device("cpu"),
            torch.float64,
            torch.Generator().manual_seed(int(scenario.get("seed", 0))),
            our_half=self.our_half,
        )
        self.driver = int(scenario.get("driver", 0))
        self.policy_opp: PolicyOpponents | None = None
        if self.driver > 0:
            spec = EnvSpec(
                obs_names=obs_spec.OBS_NAMES,
                privileged_names=(),
                obs_spec_version=obs_spec.OBS_SPEC_VERSION,
                policy_dt_s=self.cfg.policy_dt_s,
                max_episode_steps=1,
            )
            driver = self.profile.drivers[self.driver - 1]
            if not driver.checkpoint:
                raise ValueError(
                    f"scenario {scenario.get('seed')}: {driver.name} has no fixed policy"
                )
            self.policy_opp = PolicyOpponents(
                self.profile.drivers,
                spec,
                1,
                self.cfg.policy_dt_s,
                torch.device("cpu"),
                torch.float32,
            )
            self.policy_opp.driver[:] = self.driver
        p = scenario["opp_params"]
        self.opponents.p = OpponentParams(
            **{
                k: torch.tensor(
                    [v],
                    dtype=torch.long
                    if k in ("behaviour", "delay_steps", "corner")
                    else torch.float64,
                )
                for k, v in p.items()
            }
        )
        if p["behaviour"] == 6:
            self.opponents.replay = ReplayTracks(
                bool(scenario.get("holdout", True)),
                torch.device("cpu"),
                torch.float64,
                episode_s=self.cfg.episode_s,
            )
            self.opponents.replay_index = torch.tensor([int(scenario["replay_index"])])
            self.opponents.replay_t = torch.tensor([float(scenario["replay_t"])])
        x, y, _ = scenario["our_pose"]
        self.opponents.history[:, :, 0] = x
        self.opponents.history[:, :, 1] = y
        self.opp_vel = np.zeros(3)

    # ------------------------------------------------------------------ truth

    def pose(self) -> tuple[float, float, float]:
        pq = pose_from_qpos(self.data.qpos[:7])
        return float(pq["x"]), float(pq["y"]), float(pq["yaw"])

    @property
    def pitch(self) -> float:
        return float(pose_from_qpos(self.data.qpos[:7])["pitch"])

    @property
    def yaw_rate(self) -> float:
        rot = np.empty(9)
        mujoco.mju_quat2Mat(rot, self.data.qpos[3:7])
        return float((rot.reshape(3, 3) @ self.data.qvel[3:6])[2])

    @property
    def up_z(self) -> float:
        rot = np.empty(9)
        mujoco.mju_quat2Mat(rot, self.data.qpos[3:7])
        return float(rot[8])

    @property
    def forward_speed(self) -> float:
        yaw = self.pose()[2]
        return float(self.data.qvel[0] * math.cos(yaw) + self.data.qvel[1] * math.sin(yaw))

    def opponent_pose(self) -> tuple[float, float, float]:
        p = self.data.mocap_pos[OPP_MOCAP]
        q = self.data.mocap_quat[OPP_MOCAP]
        yaw = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
        return float(p[0]), float(p[1]), yaw

    def house_pose(self) -> tuple[float, float, float] | None:
        if not self.house["present"]:
            return None
        p = self.data.mocap_pos[HOUSE_MOCAP]
        return float(p[0]), float(p[1]), 0.0

    # ------------------------------------------------------------------ stepping

    def _wheel_volts(self, left: float, right: float) -> np.ndarray:
        u = np.array([left, right]) / 100.0
        shaped = shape_command(u, self.deadzone, self.fit.curve)
        volts = shaped * NOMINAL_PACK_V * self.side_gain
        if not self.fit.brake_on_zero:
            # Coast: a zero command lets the wheel freewheel, the back-EMF voltage.
            emf = self.motor.back_emf_volts(self.data.qvel[self.index.wheel_dofs])
            volts = np.where(shaped == 0.0, emf, volts)
        return np.asarray(volts)

    def _sweep_hits(self, rot: np.ndarray) -> bool:
        """`warp_kernels.weapon_sweep_hit`: the weapon's sweep cylinder within the margin of the
        opponent's box, at the same sample points."""
        from auto_battlebot.rl.envs.warp_kernels import SWEEP_ANGLES, SWEEP_MARGIN_M

        sweep = self.mp.weapon
        ox, oy, oyaw = self.opponent_pose()
        centre = np.array([ox, oy, self.data.mocap_pos[OPP_MOCAP][2]])
        half = self.model.geom_size[self.opp_geom]
        c, s_ = math.cos(oyaw), math.sin(oyaw)
        for k in range(SWEEP_ANGLES):
            a = 2.0 * math.pi * k / SWEEP_ANGLES
            for r in (sweep.radius, 0.5 * sweep.radius, 0.0):
                for face in (-1, 0, 1):
                    local = sweep.center + np.array(
                        [r * math.cos(a), sweep.half_width * face, r * math.sin(a)]
                    )
                    d = self.data.qpos[:3] + rot @ local - centre
                    lx, ly = c * d[0] + s_ * d[1], -s_ * d[0] + c * d[1]
                    q = np.maximum(np.abs([lx, ly, d[2]]) - half, 0.0)
                    if float(np.linalg.norm(q)) < SWEEP_MARGIN_M:
                        return True
        return False

    def _in_arc(self, p: np.ndarray, ox: float, oy: float, oyaw: float, phi: float) -> bool:
        """`warp_kernels.in_opponent_arc`: a hit taken."""
        from auto_battlebot.rl.envs.warp_kernels import FRONT_FACE_MARGIN_M

        if abs(phi) >= self.arc:
            return False
        if self.profile.arc_rule == "front_face":
            along = math.cos(oyaw) * (p[0] - ox) + math.sin(oyaw) * (p[1] - oy)
            return along > self.opp_half[0] - FRONT_FACE_MARGIN_M
        return True

    def _on_weapon(self, rot: np.ndarray, p: np.ndarray) -> bool:
        """A contact with the opponent lands on our weapon: Mk3's disk sweep, Mk2's wedge."""
        if self.profile.weapon == "disk":
            return self._sweep_hits(rot)
        b = rot.T @ (p - self.data.qpos[:3])
        return bool(b[0] > self.cfg.strike_x_m and abs(b[1]) < self.cfg.strike_half_width_m)

    def _classify(self, flags: np.ndarray) -> None:
        x, y, _ = self.pose()
        ox, oy, oyaw = self.opponent_pose()
        flat = np.empty(9)
        mujoco.mju_quat2Mat(flat, self.data.qpos[3:7])
        rot = flat.reshape(3, 3)
        for c in self.data.contact[: self.data.ncon]:
            r0, r1 = self.role[c.geom1], self.role[c.geom2]
            if r0 == 1:
                other = r1
            elif r1 == 1:
                other = r0
            else:
                continue
            if other == 3:
                flags[4] = 1
            elif other == 4:
                flags[5] = 1
            elif other == 5:
                flags[6] = 1
            elif other == 2:
                p = c.pos
                phi = wrap(math.atan2(p[1] - oy, p[0] - ox) - oyaw)
                flags[0] = 1
                if self._in_arc(np.asarray(p), ox, oy, oyaw, phi):
                    flags[1] = 1
                    continue
                if self._on_weapon(rot, np.asarray(p)):
                    flags[2] = 1
                    flags[3] = max(flags[3], abs(phi))

    def step(self, linear: float, angular: float, dt: float) -> None:
        """Hold the command for one tick of `dt`, then score the tick like `WarpNavEnv.step`."""
        if self.done:
            return
        x0, y0, yaw0 = self.pose()
        v0 = np.array([self.data.qvel[0], self.data.qvel[1]])
        ox, oy, oyaw = self.opponent_pose()
        phi0 = self._potential((x0, y0), (ox, oy, oyaw))
        self.opponents.record_us(
            torch.tensor([[x0, y0]], dtype=torch.float64),
            torch.from_numpy(v0[None].copy()),
            torch.tensor([yaw0], dtype=torch.float64),
        )
        foot = torch.tensor([float(np.linalg.norm(self.opp_half))], dtype=torch.float64)
        half = torch.tensor([self.half], dtype=torch.float64)
        cmd = self.opponents.command(
            torch.tensor([[ox, oy]], dtype=torch.float64),
            torch.tensor([oyaw], dtype=torch.float64),
            half,
            foot,
            dt,
            opp_half=torch.as_tensor(np.asarray(self.opp_half)[None], dtype=torch.float64),
            override=self._policy_override(x0, y0, yaw0, v0, ox, oy, oyaw),
        )
        self.opp_vel = cmd[0].numpy()
        self._house_command()

        flags = np.zeros(7)
        a_percent = -100.0 * float(np.clip(linear, -1.0, 1.0))
        b_percent = -100.0 * float(np.clip(angular, -1.0, 1.0))
        sub = self.timestep
        for _ in range(max(1, round(dt / sub))):
            self.transport.append((a_percent, b_percent))
            a, b = self.transport.popleft()
            if self.direct:
                # The radio's mix: left = linear - angular, right = linear + angular.
                self.drive.append((-a + b, -a - b))
                self.data.ctrl[:] = self._wheel_volts(*self.drive.popleft())
                self._move_mocaps(sub)
                mujoco.mj_step(self.model, self.data)
                self._classify(flags)
                continue
            up = self.up_z
            if up < -0.1:
                self.upside = True
            elif up > 0.1:
                self.upside = False
            heading = heading_from_yaw(self.pose()[2])
            a_eff = -a if self.upside else a
            left, right = self.mixer.step(
                a_eff,
                b,
                heading,
                sub,
                upside_down=self.upside,
                yaw_rate_dps=gyro_from_yaw_rate(self.yaw_rate),
            )
            self.drive.append((left, right))
            self.data.ctrl[:] = self._wheel_volts(*self.drive.popleft())
            self._move_mocaps(sub)
            mujoco.mj_step(self.model, self.data)
            self._classify(flags)
        self.t += dt
        self._score(flags, phi0, dt)

    def _policy_override(
        self,
        x: float,
        y: float,
        yaw: float,
        v: np.ndarray,
        ox: float,
        oy: float,
        oyaw: float,
    ) -> tuple[torch.Tensor, torch.Tensor] | None:
        """The policy-driven opponent's velocity, as `WarpNavEnv._policy_opponent_command`."""
        po = self.policy_opp
        if po is None:
            return None
        f = torch.float32

        def t(values: list[float]) -> torch.Tensor:
            return torch.tensor([values], dtype=f)

        slots = self.slots
        house = self.house_pose()
        hz_xy = t([[sl["x"], sl["y"]] for sl in slots] + [list(house[:2]) if house else [0.0, 0.0]])
        hz_r = t([sl["r"] for sl in slots] + [HOUSE_RADIUS_M])
        hz_kind = torch.tensor([[sl["kind"] for sl in slots] + [obs_spec.TRACKED]])
        hz_vel = t([[0.0, 0.0]] * len(slots) + [list(self.house_v)])
        hz_present = torch.tensor(
            [[bool(sl["present"]) for sl in slots] + [bool(self.house["present"])]]
        )
        zero = torch.zeros(1, dtype=f)
        false = torch.zeros(1, dtype=torch.bool)
        inp = obs_spec.ObsInputs(
            our_xy=t([ox, oy]),
            our_yaw=t([oyaw])[0],
            our_speed=po.v,
            our_yaw_rate=po.w_rate,
            our_age_s=zero,
            our_stale=false,
            run_away=false,
            hist_action=po.hist_action,
            hist_speed=po.hist_speed,
            hist_yaw_rate=po.hist_rate,
            opp_present=~false,
            opp_xy=t([x, y]),
            opp_yaw=t([yaw])[0],
            opp_velocity=t([float(v[0]), float(v[1])]),
            opp_yaw_rate=t([self.yaw_rate])[0],
            opp_position_sigma=zero,
            opp_heading_sigma=zero,
            opp_yaw_rate_sigma=zero,
            opp_half_diagonal=zero + self.our_half_diag,
            opp_age_s=zero,
            opp_stale=false,
            field_half=t(list(self.half)),
            hazard_xy=hz_xy,
            hazard_radius=hz_r,
            hazard_kind=hz_kind,
            hazard_velocity=hz_vel,
            hazard_present=hz_present,
        )
        velocity, active = po.command(inp, t([oyaw])[0])
        return active, velocity.to(torch.float64)

    def _move_mocaps(self, dt: float) -> None:
        p = self.data.mocap_pos[OPP_MOCAP]
        q = self.data.mocap_quat[OPP_MOCAP]
        yaw = (
            math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
            + self.opp_vel[2] * dt
        )
        p[0] += self.opp_vel[0] * dt
        p[1] += self.opp_vel[1] * dt
        self.data.mocap_quat[OPP_MOCAP] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        if self.house["present"]:
            h = self.data.mocap_pos[HOUSE_MOCAP]
            h[0] += self.house_v[0] * dt
            h[1] += self.house_v[1] * dt

    def _house_command(self) -> None:
        if not self.house["present"]:
            return
        self.house_v = 0.95 * self.house_v + 0.05 * self.rng.standard_normal(2) * 0.4
        hp = self.data.mocap_pos[HOUSE_MOCAP, :2]
        lim = self.half - HOUSE_RADIUS_M
        out = ((hp >= lim) & (self.house_v > 0)) | ((hp <= -lim) & (self.house_v < 0))
        self.house_v = np.where(out, -self.house_v, self.house_v)

    def _potential(self, xy: tuple[float, float], opp: tuple[float, float, float]) -> float:
        heading = np.array([math.cos(opp[2]), math.sin(opp[2])])
        o = np.array(opp[:2])
        us = np.array(xy)
        if self.score.run_away > 0.5:
            return min(float(np.linalg.norm(us - (o + self.cfg.stage_offset_m * heading))), 1.5)
        rel = us - o
        phi = abs(math.remainder(math.atan2(rel[1], rel[0]) - opp[2], 2 * math.pi))
        u = min(max((phi - 0.5 * self.arc) / (0.5 * math.pi - 0.5 * self.arc), 0.0), 1.0)
        near = min(max(1.0 - float(np.linalg.norm(rel)) / FLANK_RANGE_M, 0.0), 1.0)
        flank = u * u * (3.0 - 2.0 * u) * near
        behind = -float(np.linalg.norm(us - (o - self.cfg.stage_offset_m * heading)))
        return behind + self.cfg.reward["flank"] * flank

    def _score(self, f: np.ndarray, phi0: float, dt: float) -> None:
        cfg, s = self.cfg, self.score
        r = self._score_contacts(f, dt) + self._score_obstacles(f)
        x, y, _ = self.pose()
        ox, oy, oyaw = self.opponent_pose()
        hazard_r, terminated, diverged = self._score_hazards(x, y, dt)
        r += hazard_r
        phi1 = 0.0 if terminated else self._potential((x, y), (ox, oy, oyaw))
        r += cfg.reward["shaping"] * (cfg.gamma * phi1 - phi0)
        s.reward += 0.0 if diverged else r
        s.seconds = self.t
        if terminated or self.t >= cfg.episode_s - 1e-9:
            self.done = True

    def _score_contacts(self, f: np.ndarray, dt: float) -> float:
        """Strike, hit taken, push and danger zone, as `WarpNavEnv.step` scores them."""
        cfg, rw, s = self.cfg, self.cfg.reward, self.score
        contact, arc, wedge = f[0] > 0.5, f[1] > 0.5, f[2] > 0.5
        attack = s.run_away < 0.5
        r = 0.0
        strike = contact and not self.engaged and wedge and not arc and attack
        if strike:
            u = (f[3] - self.arc) / (math.pi - self.arc)
            r += rw["strike_base"] + rw["strike_rear"] * float(smoothstep(torch.tensor(u)))
            s.strikes += 1
            s.rear_strikes += float(f[3] > 2 * math.pi / 3)
            s.first_strike_s = min(s.first_strike_s, self.t)
        self.arc_s = self.arc_s + dt if arc else 0.0
        if arc and (not self.prev_arc or self.arc_s >= cfg.arc_rehit_s):
            self.arc_s = 0.0
            r += rw["hit_taken"]
            s.hits_taken += 1
            s.first_hit_s = min(s.first_hit_s, self.t)
        self.prev_arc = arc
        self.struck = self.struck or strike
        self.engaged = self.engaged or contact
        self.apart_s = 0.0 if contact else self.apart_s + dt
        if self.engaged and self.apart_s >= cfg.engagement_gap_s:
            self.engaged = self.struck = False
        if wedge and not arc and self.struck and attack:
            r += rw["push_per_s"] * dt
        x, y, _ = self.pose()
        ox, oy, oyaw = self.opponent_pose()
        rel = np.array([x - ox, y - oy])
        phi_us = wrap(math.atan2(rel[1], rel[0]) - oyaw)
        reach = float(np.linalg.norm(self.opp_half)) + self.our_half_diag + cfg.danger_margin_m
        if not contact and abs(phi_us) < self.arc and np.linalg.norm(rel) < reach:
            r += rw["danger_per_s"] * dt
            s.danger_s += dt
        speed = float(np.hypot(self.data.qvel[0], self.data.qvel[1]))
        if not (wedge and not arc) and speed < cfg.still_speed_mps:
            r += rw["still_per_s"] * dt
            s.still_s += dt
        return r

    def _score_obstacles(self, f: np.ndarray) -> float:
        """Wall events (waived with the wedge on the opponent) and wall-block contacts."""
        rw, s = self.cfg.reward, self.score
        r = 0.0
        wall = f[4] > 0.5
        if wall and not self.prev_wall and not (f[2] > 0.5 and f[0] > 0.5):
            r += rw["wall_event"]
            s.wall_events += 1
        self.prev_wall = wall
        block = f[5] > 0.5 or f[6] > 0.5
        if block and not self.prev_block:
            r += rw["block_event"]
            s.block_contacts += 1
        self.prev_block = block
        return r

    def _score_hazards(self, x: float, y: float, dt: float) -> tuple[float, bool, bool]:
        """Holes, flips, nose lift and solver divergence. Returns (reward, terminated, diverged)."""
        cfg, rw, s = self.cfg, self.cfg.reward, self.score
        r = 0.0
        in_hole = any(
            sl["present"]
            and sl["kind"] == obs_spec.HOLE
            and math.hypot(x - sl["x"], y - sl["y"]) < sl["r"]
            for sl in self.slots
        )
        pitch = self.pitch
        tipped = self.up_z < 0.0 or abs(pitch) > math.radians(80)
        self.flip_s = self.flip_s + dt if tipped else 0.0
        flipped = self.flip_s >= cfg.flip_hold_s
        if pitch < -math.radians(30):
            r += rw["nose_lift_per_s"] * dt
            s.nose_lift_s += dt
        diverged = not np.isfinite(self.data.qpos).all()
        for happened, reward, field_name, outcome in (
            (in_hole, rw["hole"], "holes", "FELL_IN"),
            (flipped, rw["flip"], "flips", "FLIPPED"),
            (diverged, 0.0, "sim_failures", "DIVERGED"),
        ):
            if happened:
                r += reward
                setattr(s, field_name, getattr(s, field_name) + 1)
                s.outcome = outcome
        return r, in_hole or flipped or diverged, diverged
