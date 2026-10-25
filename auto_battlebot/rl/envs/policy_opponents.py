"""Opponents driven by trained navigation policies, for the RL environment, batched in torch.

A driver is a navigation policy on a robot: Mr Stabs Mk2 running a trained Mk2 bundle, or a copy
of the robot in training (self-play, a frozen snapshot the trainer refreshes). Each policy step it
builds the observation (`obs_spec.build_v1`) from the opponent's side of the arena, from true
state: it sees us exactly, the way `simulation/policy_opponent.py` plays a mirror match. Its
action goes through `DifferentialDriveProcessor`'s saturation, the robot's command delay and its
grey-box plant (`torch_plant`), and comes out as the opponent mocap's velocity. The scripted
opponents' wall and contact rules (`Opponents.command`) still apply.

Driver 0 is the scripted opponents (`opponents.py`); drivers 1 and up are `PolicyDriver`s in the
order given.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import torch

from auto_battlebot.control.plant import PlantParams
from auto_battlebot.rl import obs_spec
from auto_battlebot.rl.env import EnvSpec
from auto_battlebot.rl.policies.mlp import MlpGaussianPolicy
from auto_battlebot.rl.torch_plant import TorchPlantParams, integrate, saturate, steady_state

REPO = Path(__file__).resolve().parents[3]
SCRIPTED = "scripted"
SELF_PLAY = "self_play"
PLANT_SUBSTEPS = 4


@dataclass(frozen=True)
class PolicyDriver:
    name: str
    robot: str  # footprint key, warp_nav.ROBOT_SIZE_M
    plant_file: str  # a TOML with a [plant] table, repo-relative
    checkpoint: str  # a training checkpoint, repo-relative; "" for self-play


class PolicyOpponents:
    def __init__(
        self,
        drivers: tuple[PolicyDriver, ...],
        spec: EnvSpec,
        worlds: int,
        policy_dt: float,
        device: torch.device,
        dtype: torch.dtype,
    ) -> None:
        self.drivers = drivers
        self.names = (SCRIPTED, *(d.name for d in drivers))
        self.spec = spec
        self.w = worlds
        self.dt = policy_dt
        self.device = device
        self.dtype = dtype
        self.driver = torch.zeros(worlds, dtype=torch.long, device=device)
        self.nets: list[MlpGaussianPolicy | None] = []
        self.plants: list[TorchPlantParams] = []
        self.lags: list[int] = []
        for d in drivers:
            net = None
            if d.checkpoint:
                net = MlpGaussianPolicy(spec).to(device).eval()
                state = torch.load(REPO / d.checkpoint, map_location=device)["learner"]["policy"]
                net.load_state_dict(state)
            self.nets.append(net)
            p = PlantParams.from_toml(REPO / d.plant_file)
            self.plants.append(TorchPlantParams.from_params(p, worlds, device, dtype))
            self.lags.append(max(0, round(p.delay_s / policy_dt)))
        lag = max(self.lags, default=0) + 1
        self.ring = torch.zeros(worlds, lag, 2, device=device, dtype=dtype)
        self.v = torch.zeros(worlds, device=device, dtype=dtype)
        self.w_rate = torch.zeros(worlds, device=device, dtype=dtype)
        self.hist_action = torch.zeros(worlds, obs_spec.HISTORY, 2, device=device, dtype=dtype)
        self.hist_speed = torch.zeros(worlds, obs_spec.HISTORY, device=device, dtype=dtype)
        self.hist_rate = torch.zeros(worlds, obs_spec.HISTORY, device=device, dtype=dtype)

    @property
    def available(self) -> bool:
        return len(self.drivers) > 0

    def set_self_play(self, state: dict[str, torch.Tensor]) -> None:
        """Freeze a copy of the policy in training as the self-play driver."""
        for k, d in enumerate(self.drivers):
            if d.checkpoint:
                continue
            net = self.nets[k]
            if net is None:
                net = MlpGaussianPolicy(self.spec).to(self.device).eval()
                self.nets[k] = net
            net.load_state_dict(state)

    def draw(self, mask: torch.Tensor, probability: float, gen: torch.Generator) -> None:
        """Pick each reset world's driver: scripted, or with `probability` one of the policies
        that has a network, uniformly."""
        ready = [k + 1 for k, net in enumerate(self.nets) if net is not None]
        if not ready or probability <= 0.0:
            self.driver = torch.where(mask, torch.zeros_like(self.driver), self.driver)
            return
        r = torch.rand(self.w, generator=gen, device=self.device, dtype=self.dtype)
        pick = torch.randint(0, len(ready), (self.w,), generator=gen, device=self.device)
        chosen = torch.tensor(ready, device=self.device)[pick]
        drawn = torch.where(r < probability, chosen, torch.zeros_like(chosen))
        self.driver = torch.where(mask, drawn, self.driver)
        for t in (self.ring, self.hist_action):
            t[mask] = 0.0
        for t in (self.v, self.w_rate, self.hist_speed, self.hist_rate):
            t[mask] = 0.0

    def robot_of(
        self, driver: torch.Tensor, scripted: torch.Tensor, keys: tuple[str, ...]
    ) -> torch.Tensor:
        """Each world's opponent footprint index into `keys`: the driver's robot, or `scripted`."""
        out = scripted.clone()
        for k, d in enumerate(self.drivers):
            out = torch.where(driver == k + 1, keys.index(d.robot), out)
        return out

    @torch.no_grad()
    def command(
        self, inp: obs_spec.ObsInputs, yaw: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """World-frame (vx, vy, yaw rate) [W, 3] for policy-driven worlds, and their mask.

        `inp` is the observation input from the opponent's side: its pose as ours, ours as the
        opponent's. `hist_*` fields are filled in here."""
        active = self.driver > 0
        out = torch.zeros(self.w, 3, device=self.device, dtype=self.dtype)
        if not bool(active.any()):
            return out, active
        inp.hist_action = self.hist_action
        inp.hist_speed = self.hist_speed
        inp.hist_yaw_rate = self.hist_rate
        obs = obs_spec.build_v1(inp)
        action = torch.zeros(self.w, 2, device=self.device, dtype=self.dtype)
        for k, net in enumerate(self.nets):
            rows = self.driver == k + 1
            if net is None or not bool(rows.any()):
                continue
            action[rows] = net.act(obs[rows], None, deterministic=True)[0]
        self.hist_action = torch.cat([action[:, None], self.hist_action[:, :-1]], 1)
        self.hist_speed = torch.cat([inp.our_speed[:, None], self.hist_speed[:, :-1]], 1)
        self.hist_rate = torch.cat([inp.our_yaw_rate[:, None], self.hist_rate[:, :-1]], 1)

        # Command delay at the policy step: ring slot 0 is this step's command.
        self.ring = torch.cat([saturate(action)[:, None], self.ring[:, :-1]], 1)
        lin = torch.zeros(self.w, device=self.device, dtype=self.dtype)
        ang = torch.zeros_like(lin)
        for k, lag in enumerate(self.lags):
            rows = self.driver == k + 1
            lin = torch.where(rows, self.ring[:, lag, 0], lin)
            ang = torch.where(rows, self.ring[:, lag, 1], ang)
        state = torch.zeros(self.w, 5, device=self.device, dtype=self.dtype)
        state[:, 3], state[:, 4] = self.v, self.w_rate
        h = self.dt / PLANT_SUBSTEPS
        for k, p in enumerate(self.plants):
            rows = self.driver == k + 1
            if not bool(rows.any()):
                continue
            v_t, w_t = steady_state(lin, ang, p)
            s = state.clone()
            for _ in range(PLANT_SUBSTEPS):
                s = integrate(s, v_t, w_t, h, p)
            state = torch.where(rows[:, None], s, state)
        self.v = torch.where(active, state[:, 3], self.v)
        self.w_rate = torch.where(active, state[:, 4], self.w_rate)
        out[:, 0] = self.v * torch.cos(yaw)
        out[:, 1] = self.v * torch.sin(yaw)
        out[:, 2] = self.w_rate
        return out, active
