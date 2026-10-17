"""The toy environment: the viability test for RL navigation (plan step 1).

The grey-box plant (`torch_plant`) drives Mr Stabs Mk2 in a square box against one opponent that
never moves. Robots are discs. Contact is classified once per engagement by where it lands:

- inside the opponent's front arc (60 degrees either side of its heading): a hit taken
- outside it, on our wedge (contact normal within `wedge_half_angle_deg` of our heading): a strike,
  worth more the closer it is to dead rear
- anywhere else: nothing

That is all of the plan's section 7 reward that needs no contact geometry, plus wall events and
command smoothing. With `sensing` on, the policy sees our robot through `OurEkfErrorModel` and the
opponent through `MeasurementModel` and the opponent filter mirror, the same estimators the Warp
environment and the robot use.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import torch

from auto_battlebot.control.plant import PlantParams
from auto_battlebot.perception.opponent_filter import OpponentFilter, OpponentFilterConfig
from auto_battlebot.perception.sensing_model import (
    Context,
    MeasurementModel,
    OurEkfErrorModel,
    SensingModelFile,
    TruePoses,
)
from auto_battlebot.rl.env import SOURCES, EnvSpec, Step
from auto_battlebot.rl.torch_plant import (
    CommandDelay,
    TorchPlantParams,
    integrate,
    saturate,
    steady_state,
)

REPO = Path(__file__).resolve().parents[3]
TOY_OBS_SPEC_VERSION = 0

OBS_NAMES = (
    "our_speed",
    "our_yaw_rate",
    "our_age",
    "our_stale",
    "opp_x",
    "opp_y",
    "opp_distance",
    "opp_bearing_sin",
    "opp_bearing_cos",
    "opp_heading_sin",
    "opp_heading_cos",
    "opp_vx",
    "opp_vy",
    "opp_yaw_rate",
    "opp_position_sigma",
    "opp_heading_sigma",
    "opp_age",
    "opp_stale",
    "opp_known",
    "wall_front",
    "wall_left",
    "wall_back",
    "wall_right",
    "prev_linear",
    "prev_angular",
)
PRIVILEGED_NAMES = (
    "true_opp_x",
    "true_opp_y",
    "true_opp_distance",
    "true_opp_heading_sin",
    "true_opp_heading_cos",
    "true_our_speed",
    "true_our_yaw_rate",
    "true_phi_sin",
    "true_phi_cos",
    "engaged",
    "latency",
    "episode_progress",
)


@dataclass
class ToyConfig:
    worlds: int = 4096
    arena_half_m: float = 1.25
    our_radius_m: float = 0.08
    opp_radius_m: float = 0.12
    wedge_half_angle_deg: float = 45.0
    arc_half_angle_deg: float = 60.0
    danger_margin_m: float = 0.10
    stage_offset_m: float = 0.25
    min_start_separation_m: float = 0.5
    engagement_gap_s: float = 0.5
    latency_s: float = 0.05
    episode_s: float = 20.0
    policy_dt_s: float = 0.02
    substep_s: float = 0.002
    gamma: float = 0.99
    plant_profile: str = "config/mr_stabs_mk2_desktop.toml"
    sensing: bool = False
    sensing_model: str = "simulation/sensing_models/provisional_rl.toml"
    measurement_period_s: float = 0.01
    reward: dict[str, float] = field(
        default_factory=lambda: {
            "strike_base": 3.0,
            "strike_rear": 7.0,
            "push_per_s": 1.0,
            "hit_taken": -10.0,
            "danger_per_s": -1.0,
            "wall_event": -0.2,
            "command_change": -0.01,
            "shaping": 1.0,
        }
    )


def wrap(a: torch.Tensor) -> torch.Tensor:
    return torch.remainder(a + math.pi, 2.0 * math.pi) - math.pi


def ray_to_box(xy: torch.Tensor, angle: torch.Tensor, half: float, cap: float) -> torch.Tensor:
    """Distance from `xy` [W, 2] along `angle` [W, K] to the wall of a box of half size `half`."""
    dx, dy = torch.cos(angle), torch.sin(angle)
    big = torch.full_like(dx, 1e9)
    tx = torch.where(dx.abs() > 1e-9, (half * torch.sign(dx) - xy[:, :1]) / dx, big)
    ty = torch.where(dy.abs() > 1e-9, (half * torch.sign(dy) - xy[:, 1:]) / dy, big)
    return torch.minimum(tx, ty).clamp(0.0, cap)


def smoothstep(u: torch.Tensor) -> torch.Tensor:
    u = u.clamp(0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


@SOURCES.register("toy")
class ToyEnv:
    def __init__(self, cfg: ToyConfig, device: torch.device | str = "cuda", seed: int = 0) -> None:
        self.cfg = cfg
        self.device = torch.device(device)
        self.dtype = torch.float32
        self.num_worlds = cfg.worlds
        self.substeps = int(round(cfg.policy_dt_s / cfg.substep_s))
        self.max_steps = int(round(cfg.episode_s / cfg.policy_dt_s))
        self.spec = EnvSpec(
            obs_names=OBS_NAMES,
            privileged_names=PRIVILEGED_NAMES,
            obs_spec_version=TOY_OBS_SPEC_VERSION,
            policy_dt_s=cfg.policy_dt_s,
            max_episode_steps=self.max_steps,
        )
        self.arc = math.radians(cfg.arc_half_angle_deg)
        self.wedge = math.radians(cfg.wedge_half_angle_deg)
        self.contact_dist = cfg.our_radius_m + cfg.opp_radius_m
        self.gen = torch.Generator(device=self.device)
        self.gen.manual_seed(seed)
        self.seed = seed

        w, dev, dt = cfg.worlds, self.device, self.dtype
        self.base_plant = PlantParams.from_toml(REPO / cfg.plant_profile)
        self.plant = TorchPlantParams.from_params(self.base_plant, w, dev, dt)
        capacity = int(math.ceil(0.2 / cfg.substep_s)) + 2
        self.delay = CommandDelay(w, capacity, dev, dt)
        self.our = torch.zeros(w, 5, device=dev, dtype=dt)
        self.opp_xy = torch.zeros(w, 2, device=dev, dtype=dt)
        self.opp_yaw = torch.zeros(w, device=dev, dtype=dt)
        self.t = torch.zeros(w, device=dev, dtype=dt)
        self.steps = torch.zeros(w, dtype=torch.long, device=dev)
        self.prev_action = torch.zeros(w, 2, device=dev, dtype=dt)
        self.engaged = torch.zeros(w, dtype=torch.bool, device=dev)
        self.struck = torch.zeros(w, dtype=torch.bool, device=dev)
        self.apart_s = torch.zeros(w, device=dev, dtype=dt)
        self.in_wall = torch.zeros(w, dtype=torch.bool, device=dev)
        self.latency = torch.full((w,), cfg.latency_s, device=dev, dtype=dt)
        self.stats = {
            k: torch.zeros(w, device=dev, dtype=dt)
            for k in ("strikes", "rear_strikes", "hits_taken", "danger_s", "wall_events", "reward")
        }
        self.first_strike_s = torch.full((w,), math.inf, device=dev, dtype=dt)
        self.first_hit_s = torch.full((w,), math.inf, device=dev, dtype=dt)

        self.sensing = None
        if cfg.sensing:
            spec = SensingModelFile.load(REPO / cfg.sensing_model)
            self.measure = MeasurementModel(
                spec, ("ours", "opponent"), w, dev, dt, seed=seed + 101, randomize=True
            )
            self.ekf = OurEkfErrorModel(spec, w, "ours", 0.2, dev, dt, seed=seed + 202)
            half = torch.full((w, 2), cfg.arena_half_m, device=dev, dtype=dt)
            self.opp_filter = OpponentFilter(
                w,
                OpponentFilterConfig(
                    opponent_yaw_accel_psd=30.0,
                    opponent_heading_sigma_rad=0.15,
                    opponent_max_coast_s=0.1,
                    render_lead_s=self.base_plant.delay_s,
                ),
                field_half_extent=half,
                device=dev,
                dtype=dt,
            )
            self.measure_every = max(1, int(round(cfg.measurement_period_s / cfg.substep_s)))

    # ------------------------------------------------------------------ randomness

    def _rand(self, *shape: int) -> torch.Tensor:
        return torch.rand(*shape, generator=self.gen, device=self.device, dtype=self.dtype)

    # ------------------------------------------------------------------ episodes

    def reset(self, seed: int | None = None) -> tuple[torch.Tensor, torch.Tensor]:
        if seed is not None:
            self.gen.manual_seed(seed)
        everyone = torch.ones(self.num_worlds, dtype=torch.bool, device=self.device)
        self._reset_worlds(everyone)
        return self._observe()

    def desynchronize(self) -> None:
        """Spread the worlds' truncation steps over one episode, so training batches do not all
        reset on the same step. Evaluation skips this: every scenario plays a full episode."""
        offset = (self._rand(self.num_worlds) * self.max_steps).long()
        self.steps = offset

    def _reset_worlds(self, mask: torch.Tensor) -> None:
        cfg = self.cfg
        n = self.num_worlds
        reach = cfg.arena_half_m - cfg.opp_radius_m - 0.05
        opp_xy = (self._rand(n, 2) * 2.0 - 1.0) * reach
        # Our start: rejection-free, a random direction at a random distance from the opponent,
        # clamped into the box. Clamping can shorten the separation, so retry a few times.
        our_reach = cfg.arena_half_m - cfg.our_radius_m - 0.05
        our_xy = (self._rand(n, 2) * 2.0 - 1.0) * our_reach
        for _ in range(8):
            close = (our_xy - opp_xy).norm(dim=1) < cfg.min_start_separation_m
            if not bool(close.any()):
                break
            fresh = (self._rand(n, 2) * 2.0 - 1.0) * our_reach
            our_xy = torch.where(close[:, None], fresh, our_xy)
        our_yaw = (self._rand(n) * 2.0 - 1.0) * math.pi
        opp_yaw = (self._rand(n) * 2.0 - 1.0) * math.pi

        m1 = mask[:, None]
        state = torch.zeros(n, 5, device=self.device, dtype=self.dtype)
        state[:, :2] = our_xy
        state[:, 2] = our_yaw
        self.our = torch.where(m1, state, self.our)
        self.opp_xy = torch.where(m1, opp_xy, self.opp_xy)
        self.opp_yaw = torch.where(mask, opp_yaw, self.opp_yaw)
        self.t = torch.where(mask, torch.zeros_like(self.t), self.t)
        self.steps = torch.where(mask, torch.zeros_like(self.steps), self.steps)
        self.prev_action = torch.where(m1, torch.zeros_like(self.prev_action), self.prev_action)
        false = torch.zeros_like(self.engaged)
        self.engaged = torch.where(mask, false, self.engaged)
        self.struck = torch.where(mask, false, self.struck)
        self.in_wall = torch.where(mask, false, self.in_wall)
        self.apart_s = torch.where(mask, torch.zeros_like(self.apart_s), self.apart_s)
        for k, v in self.stats.items():
            self.stats[k] = torch.where(mask, torch.zeros_like(v), v)
        inf = torch.full_like(self.first_strike_s, math.inf)
        self.first_strike_s = torch.where(mask, inf, self.first_strike_s)
        self.first_hit_s = torch.where(mask, inf, self.first_hit_s)
        lag = torch.round(self.latency / self.cfg.substep_s).long()
        self.delay.set_lag(mask, lag)
        if self.cfg.sensing:
            self._reset_sensing(mask)

    def _reset_sensing(self, mask: torch.Tensor) -> None:
        now = self.t
        self.measure.reset(mask, now)
        self.ekf.reset(mask, now, self.our[:, 3].abs())
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

    # ------------------------------------------------------------------ physics

    def _resolve(self) -> torch.Tensor:
        """Push our disc out of the opponent and the walls. Returns the wall-contact mask."""
        cfg = self.cfg
        rel = self.our[:, :2] - self.opp_xy
        dist = rel.norm(dim=1).clamp(min=1e-6)
        overlap = (self.contact_dist - dist).clamp(min=0.0)
        pushed = self.our[:, :2] + rel / dist[:, None] * overlap[:, None]
        lim = cfg.arena_half_m - cfg.our_radius_m
        clamped = pushed.clamp(-lim, lim)
        wall: torch.Tensor = (pushed.abs() >= lim).any(dim=1)
        self.our = torch.cat([clamped, self.our[:, 2:]], dim=1)
        return wall

    def _measure(self) -> None:
        n = self.num_worlds
        xy = torch.stack([self.our[:, :2], self.opp_xy], dim=1)
        yaw = torch.stack([self.our[:, 2], self.opp_yaw], dim=1)
        heading = torch.stack([torch.cos(self.our[:, 2]), torch.sin(self.our[:, 2])], dim=1)
        vel = torch.stack([heading * self.our[:, 3:4], torch.zeros(n, 2, device=self.device)], 1)
        yaw_rate = torch.stack([self.our[:, 4], torch.zeros_like(self.our[:, 4])], dim=1)
        truth = TruePoses(xy=xy, yaw=yaw, velocity=vel, yaw_rate=yaw_rate)
        half = torch.full((2,), self.cfg.arena_half_m, device=self.device, dtype=self.dtype)
        ctx = Context.from_rect_arena(xy, 0.1, half)
        out = self.measure.step(self.t, truth, ctx)
        f = self.opp_filter
        before = f.output_stale.clone()
        var = torch.full((n,), 0.012**2, device=self.device, dtype=self.dtype)
        keypoint = torch.ones(n, dtype=torch.bool, device=self.device)
        f.update(out.stamp, out.measured[:, 1], out.xy[:, 1], out.yaw[:, 1], keypoint, var)
        # A world with no frame this tick has had no update in the C++ either.
        f.output_stale = torch.where(out.delivered, f.output_stale, before)

    def step(self, action: torch.Tensor) -> Step:
        cfg = self.cfg
        rw = cfg.reward
        sub = cfg.substep_s
        action = action.to(self.device, self.dtype).clamp(-1.0, 1.0)
        command = saturate(action)
        phi0 = self._potential()
        reward = torch.zeros(self.num_worlds, device=self.device, dtype=self.dtype)
        contact_any = torch.zeros_like(self.engaged)

        for k in range(self.substeps):
            delayed = self.delay.push(command)
            v_t, w_t = steady_state(delayed[:, 0], delayed[:, 1], self.plant)
            self.our = integrate(self.our, v_t, w_t, sub, self.plant)
            wall = self._resolve()
            self.t = self.t + sub

            rel = self.opp_xy - self.our[:, :2]
            dist = rel.norm(dim=1)
            contact = dist <= self.contact_dist + 1e-4
            normal_angle = torch.atan2(rel[:, 1], rel[:, 0])
            beta = wrap(normal_angle - self.our[:, 2])
            on_wedge = beta.abs() < self.wedge
            phi = wrap(normal_angle + math.pi - self.opp_yaw)
            in_arc = phi.abs() < self.arc

            starting = contact & ~self.engaged
            hit_taken = starting & in_arc
            strike = starting & ~in_arc & on_wedge
            u = (phi.abs() - self.arc) / (math.pi - self.arc)
            strike_value = rw["strike_base"] + rw["strike_rear"] * smoothstep(u)
            reward += torch.where(strike, strike_value, torch.zeros_like(reward))
            reward += torch.where(hit_taken, torch.full_like(reward, rw["hit_taken"]), 0.0)
            self.stats["strikes"] += strike.float()
            self.stats["rear_strikes"] += (strike & (phi.abs() > 2.0 * math.pi / 3.0)).float()
            self.stats["hits_taken"] += hit_taken.float()
            self.first_strike_s = torch.where(
                strike & torch.isinf(self.first_strike_s), self.t, self.first_strike_s
            )
            self.first_hit_s = torch.where(
                hit_taken & torch.isinf(self.first_hit_s), self.t, self.first_hit_s
            )
            self.struck = self.struck | strike
            self.engaged = self.engaged | contact
            self.apart_s = torch.where(contact, torch.zeros_like(self.apart_s), self.apart_s + sub)
            ended = self.engaged & (self.apart_s >= cfg.engagement_gap_s)
            self.engaged = self.engaged & ~ended
            self.struck = self.struck & ~ended
            contact_any = contact_any | contact

            pushing = contact & on_wedge & ~in_arc & self.struck
            reward += torch.where(pushing, torch.full_like(reward, rw["push_per_s"] * sub), 0.0)
            danger = ~contact & in_arc & (dist < self.contact_dist + cfg.danger_margin_m)
            reward += torch.where(danger, torch.full_like(reward, rw["danger_per_s"] * sub), 0.0)
            self.stats["danger_s"] += danger.float() * sub

            wall_event = wall & ~self.in_wall & ~(contact & on_wedge)
            reward += torch.where(wall_event, torch.full_like(reward, rw["wall_event"]), 0.0)
            self.stats["wall_events"] += wall_event.float()
            self.in_wall = wall

            if cfg.sensing and (k + 1) % self.measure_every == 0:
                self._measure()

        phi1 = self._potential()
        reward += rw["shaping"] * (cfg.gamma * phi1 - phi0)
        reward += rw["command_change"] * (action - self.prev_action).abs().sum(dim=1)
        self.prev_action = action
        self.stats["reward"] += reward
        self.steps += 1

        truncated = self.steps >= self.max_steps
        terminated = torch.zeros_like(truncated)
        obs, priv = self._observe()
        done = truncated | terminated
        episode: dict[str, torch.Tensor] = {}
        if bool(done.any()):
            episode = {k: v.clone() for k, v in self.stats.items()}
            episode["first_strike_s"] = self.first_strike_s.clone()
            episode["first_hit_s"] = self.first_hit_s.clone()
            final_obs, final_priv = obs, priv
            self._reset_worlds(done)
            obs, priv = self._observe()
            obs = torch.where(done[:, None], obs, final_obs)
            priv = torch.where(done[:, None], priv, final_priv)
        else:
            final_obs, final_priv = obs, priv
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

    def _potential(self) -> torch.Tensor:
        back = torch.stack([torch.cos(self.opp_yaw), torch.sin(self.opp_yaw)], dim=1)
        stage = self.opp_xy - self.cfg.stage_offset_m * back
        potential: torch.Tensor = -(self.our[:, :2] - stage).norm(dim=1)
        return potential

    # ------------------------------------------------------------------ observation

    def _our_last_delivery(self) -> torch.Tensor:
        """When our robot's last measurement reached the filter: its capture stamp plus the
        latency. `OurEkfErrorModel` starts a gap from this, as the real EKF does from its last
        update; the capture stamp alone is always a latency old and never leaves the gap."""
        stamp: torch.Tensor = self.measure.last_measured_stamp[:, 0] + self.measure.params.latency_s
        return stamp

    def _observe(self) -> tuple[torch.Tensor, torch.Tensor]:
        n = self.num_worlds
        zeros = torch.zeros(n, device=self.device, dtype=self.dtype)
        if self.cfg.sensing:
            heading = torch.stack([torch.cos(self.our[:, 2]), torch.sin(self.our[:, 2])], dim=1)
            ours = self.ekf.step(
                self.t,
                self.our[:, :2],
                self.our[:, 2],
                heading * self.our[:, 3:4],
                self.our[:, 4],
                self._our_last_delivery(),
                self.measure.params.frame_period_s,
            )
            our_xy, our_yaw = ours.xy, ours.yaw
            our_v, our_w = ours.forward_speed, ours.yaw_rate
            our_stale = ours.stale.to(self.dtype)
            our_age = (self.t - self.measure.last_measured_stamp[:, 0]).clamp(0.0, 0.5)
            est = self.opp_filter.coast(self.t)
            opp_xy = torch.stack([est.x, est.y], dim=1)
            opp_yaw, opp_v, opp_w = est.yaw, torch.stack([est.vx, est.vy], 1), est.omega
            pos_sigma = (est.position_sigma_m / 0.1).clamp(max=1.0)
            head_sigma = torch.nan_to_num(est.heading_sigma_rad / (math.pi / 2), posinf=1.0)
            head_sigma = head_sigma.clamp(max=1.0)
            opp_age = (self.t - self.opp_filter.last_measured).clamp(0.0, 0.5)
            known = est.active
            opp_stale = (est.is_stale | ~known).to(self.dtype)
        else:
            our_xy, our_yaw, our_v, our_w = (
                self.our[:, :2],
                self.our[:, 2],
                self.our[:, 3],
                self.our[:, 4],
            )
            our_stale, our_age = zeros, zeros
            opp_xy, opp_yaw = self.opp_xy, self.opp_yaw
            opp_v, opp_w = torch.zeros(n, 2, device=self.device), zeros
            pos_sigma, head_sigma, opp_age, opp_stale = zeros, zeros, zeros, zeros
            known = torch.ones(n, dtype=torch.bool, device=self.device)

        c, s = torch.cos(our_yaw), torch.sin(our_yaw)
        rel = opp_xy - our_xy
        bx = c * rel[:, 0] + s * rel[:, 1]
        by = -s * rel[:, 0] + c * rel[:, 1]
        dist = torch.hypot(bx, by)
        bearing = torch.atan2(by, bx)
        rel_heading = wrap(opp_yaw - our_yaw)
        vbx = c * opp_v[:, 0] + s * opp_v[:, 1]
        vby = -s * opp_v[:, 0] + c * opp_v[:, 1]
        kf = known.to(self.dtype)
        angles = our_yaw[:, None] + torch.tensor(
            [0.0, math.pi / 2, math.pi, -math.pi / 2], device=self.device, dtype=self.dtype
        )
        rays = ray_to_box(our_xy, angles, self.cfg.arena_half_m, 1.5) / 1.5
        obs = torch.stack(
            [
                our_v / 2.5,
                our_w / 15.0,
                our_age / 0.5,
                our_stale,
                kf * bx / 2.0,
                kf * by / 2.0,
                kf * dist / 2.0,
                kf * torch.sin(bearing),
                kf * torch.cos(bearing),
                kf * torch.sin(rel_heading),
                kf * torch.cos(rel_heading),
                kf * vbx / 2.5,
                kf * vby / 2.5,
                kf * opp_w / 15.0,
                torch.where(known, pos_sigma, torch.ones_like(pos_sigma)),
                torch.where(known, head_sigma, torch.ones_like(head_sigma)),
                opp_age / 0.5,
                opp_stale,
                kf,
                rays[:, 0],
                rays[:, 1],
                rays[:, 2],
                rays[:, 3],
                self.prev_action[:, 0],
                self.prev_action[:, 1],
            ],
            dim=1,
        )

        tc, ts = torch.cos(self.our[:, 2]), torch.sin(self.our[:, 2])
        trel = self.opp_xy - self.our[:, :2]
        tbx = tc * trel[:, 0] + ts * trel[:, 1]
        tby = -ts * trel[:, 0] + tc * trel[:, 1]
        theading = wrap(self.opp_yaw - self.our[:, 2])
        phi = wrap(torch.atan2(-trel[:, 1], -trel[:, 0]) - self.opp_yaw)
        priv = torch.stack(
            [
                tbx / 2.0,
                tby / 2.0,
                torch.hypot(tbx, tby) / 2.0,
                torch.sin(theading),
                torch.cos(theading),
                self.our[:, 3] / 2.5,
                self.our[:, 4] / 15.0,
                torch.sin(phi),
                torch.cos(phi),
                self.engaged.to(self.dtype),
                self.latency / 0.1,
                self.steps.to(self.dtype) / self.max_steps,
            ],
            dim=1,
        )
        return obs, priv
