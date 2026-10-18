"""RL navigation: PPO on a two-state toy, the bundle export round trip, plant and spec checks.

Everything here runs on the CPU in a few seconds; the GPU environments are exercised by
`playground/rl/` and the training runs.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
import torch

from auto_battlebot.compat import tomllib
from auto_battlebot.control.plant import PlantParams, PlantState, integrate_step
from auto_battlebot.control.plant import steady_state as np_steady_state
from auto_battlebot.rl import obs_spec
from auto_battlebot.rl.env import EnvSpec, Step
from auto_battlebot.rl.export.bundle import read_selftest, spec_from_policy, write_bundle
from auto_battlebot.rl.export.rtneural import run_json
from auto_battlebot.rl.learners.ppo import Ppo, PpoConfig
from auto_battlebot.rl.policies.mlp import MlpGaussianConfig, MlpGaussianPolicy
from auto_battlebot.rl.torch_plant import TorchPlantParams, integrate, saturate, steady_state

REPO = Path(__file__).resolve().parents[2]


class TwoStateEnv:
    """Each step shows state 0 or 1 (one-hot); the reward is +1 when the action's linear sign is
    + in state 0 and - in state 1, else -1. One-step episodes."""

    def __init__(self, worlds: int = 512) -> None:
        self.num_worlds = worlds
        self.device = torch.device("cpu")
        self.spec = EnvSpec(("s0", "s1"), ("p",), 0, 0.02, 1)
        self.gen = torch.Generator().manual_seed(0)
        self.state = torch.zeros(worlds, dtype=torch.long)

    def _obs(self) -> tuple[torch.Tensor, torch.Tensor]:
        obs = torch.nn.functional.one_hot(self.state, 2).float()
        return obs, self.state[:, None].float()

    def reset(self, seed: int) -> tuple[torch.Tensor, torch.Tensor]:
        self.state = torch.randint(2, (self.num_worlds,), generator=self.gen)
        return self._obs()

    def step(self, action: torch.Tensor) -> Step:
        want = torch.where(self.state == 0, 1.0, -1.0)
        reward = torch.where(torch.sign(action[:, 0]) == want, 1.0, -1.0)
        final_obs, final_priv = self._obs()
        self.state = torch.randint(2, (self.num_worlds,), generator=self.gen)
        obs, priv = self._obs()
        done = torch.ones(self.num_worlds, dtype=torch.bool)
        return Step(obs, priv, reward, done, torch.zeros_like(done), final_obs, final_priv)


def test_ppo_learns_two_state_toy() -> None:
    torch.manual_seed(0)
    env = TwoStateEnv()
    policy = MlpGaussianPolicy(env.spec, MlpGaussianConfig(hidden=(32, 32)))
    learner = Ppo(policy, env, PpoConfig(steps_per_update=8, critic_hidden=(32, 32)), seed=0)  # type: ignore[arg-type]
    for _ in range(40):
        learner.iterate()
    obs = torch.eye(2)
    action, _ = policy.act(obs, None, deterministic=True)
    assert action[0, 0] > 0.5 and action[1, 0] < -0.5


def test_export_round_trip(tmp_path: Path) -> None:
    torch.manual_seed(3)
    spec = EnvSpec(tuple("abcde"), (), 9, 0.02, 1)
    policy = MlpGaussianPolicy(spec, MlpGaussianConfig(hidden=(16, 8)))
    with torch.no_grad():
        for p in policy.actor.parameters():
            p.copy_(torch.randn_like(p))
        policy.norm.mean.copy_(torch.tensor([1.0, -2.0, 0.5, 0.0, 3.0]))
        policy.norm.var.copy_(torch.tensor([4.0, 1.0, 0.25, 9.0, 1.0]))
    obs = torch.randn(16, 5) * 3
    bundle = spec_from_policy(policy, "test_run", 9, obs)
    out = write_bundle(bundle, tmp_path / "bundle", REPO)
    manifest = tomllib.loads((out / "manifest.toml").read_text())
    assert manifest["obs_spec_version"] == 9
    assert manifest["networks"][0]["inputs"][0]["shape"] == [5]
    assert manifest["decoder"]["kind"] == "SQUASHED"
    o, a = read_selftest(out / "selftest.bin")
    model = __import__("json").loads((out / "actor.rtneural.json").read_text())
    np.testing.assert_allclose(np.tanh(run_json(model, o.astype(float))), a, atol=1e-5)
    expected = torch.tanh(policy.actor(policy.norm(obs))).detach().numpy()
    np.testing.assert_allclose(a, expected, atol=1e-5)


def test_torch_plant_matches_numpy() -> None:
    p = PlantParams.from_toml(REPO / "config/mr_stabs_mk2_desktop.toml")
    rng = np.random.default_rng(0)
    w = 16
    u = rng.uniform(-1, 1, (200, w, 2))
    s_np = PlantState(np.zeros(w), np.zeros(w), np.zeros(w), np.zeros(w), np.zeros(w))
    tp = TorchPlantParams.from_params(p, w, torch.device("cpu"), torch.float64)
    s_t = torch.zeros(w, 5, dtype=torch.float64)
    for k in range(200):
        v, om = np_steady_state(u[k, :, 0], u[k, :, 1], p)
        s_np = integrate_step(s_np, v, om, 0.002, p)
        vt, wt = steady_state(torch.tensor(u[k, :, 0]), torch.tensor(u[k, :, 1]), tp)
        s_t = integrate(s_t, vt, wt, 0.002, tp)
    np.testing.assert_allclose(s_np.as_array().T, s_t.numpy(), atol=1e-9)


def test_saturate_gives_angular_priority_and_wheel_deadzone() -> None:
    out = saturate(torch.tensor([[1.0, 0.6], [0.01, 0.0], [-0.5, 0.0]]))
    # Linear is cut to the 0.4 left after angular, then each wheel loses the deadzone:
    # left 1.0 stays 1.0, right -0.2 becomes -(0.2 - 0.02) / 0.98.
    right = -(0.2 - 0.02) / 0.98
    assert out[0, 0] == pytest.approx(0.5 * (1.0 + right), abs=1e-6)
    assert out[0, 1] == pytest.approx(0.5 * (1.0 - right), abs=1e-6)
    assert out[1].abs().max() == 0.0  # both wheels inside the 2% zero deadzone
    assert out[2, 0] == pytest.approx(-(0.5 - 0.02) / 0.98, abs=1e-6)


def test_firmware_kernel_matches_scalar_mixer() -> None:
    wp = pytest.importorskip("warp")
    from auto_battlebot.mujoco_sim.firmware import FirmwareMixer
    from auto_battlebot.rl.envs.warp_kernels import FW_COLUMNS, firmware_test

    wp.init()
    rng = np.random.default_rng(1)
    t = 400
    a = np.repeat(rng.uniform(-100, 100, 40), 10)
    b = np.repeat(np.where(rng.random(40) < 0.5, 0.0, rng.uniform(-100, 100, 40)), 10)
    heading = np.cumsum(rng.normal(0, 3, t)) % 360
    rate = rng.normal(0, 200, t)
    scalar = np.zeros((t, 2))
    mixer = FirmwareMixer(auto_steer=True)
    for k in range(t):
        scalar[k] = mixer.step(a[k], b[k], heading[k], 0.01, yaw_rate_dps=rate[k])
    dev = "cpu"
    arrs = [
        wp.array(x[None].astype(np.float32), dtype=float, device=dev) for x in (a, b, heading, rate)
    ]
    fw = wp.zeros((1, FW_COLUMNS), dtype=float, device=dev)
    out = wp.zeros((1, t, 2), dtype=float, device=dev)
    wp.launch(firmware_test, dim=1, inputs=[*arrs, 0.01, fw], outputs=[out], device=dev)
    np.testing.assert_allclose(out.numpy()[0], scalar, atol=2e-3)


def _inputs(**over: object) -> obs_spec.ObsInputs:
    def t(*shape: int, value: float = 0.0) -> torch.Tensor:
        return torch.full(shape, value, dtype=torch.float64)

    base: dict[str, object] = dict(
        our_xy=t(1, 2),
        our_yaw=t(1),
        our_speed=t(1),
        our_yaw_rate=t(1),
        our_age_s=t(1),
        our_stale=torch.tensor([False]),
        run_away=torch.tensor([False]),
        hist_action=t(1, 8, 2),
        hist_speed=t(1, 8),
        hist_yaw_rate=t(1, 8),
        opp_present=torch.tensor([True]),
        opp_xy=torch.tensor([[1.0, 0.0]], dtype=torch.float64),
        opp_yaw=t(1, value=math.pi),
        opp_velocity=t(1, 2),
        opp_yaw_rate=t(1),
        opp_position_sigma=t(1),
        opp_heading_sigma=t(1, value=math.inf),
        opp_yaw_rate_sigma=t(1),
        opp_half_diagonal=t(1, value=0.2),
        opp_age_s=t(1),
        opp_stale=torch.tensor([False]),
        field_half=torch.tensor([[1.0, 1.0]], dtype=torch.float64),
        hazard_xy=t(1, 0, 2),
        hazard_radius=t(1, 0),
        hazard_kind=torch.zeros(1, 0, dtype=torch.long),
        hazard_velocity=t(1, 0, 2),
        hazard_present=torch.zeros(1, 0, dtype=torch.bool),
    )
    base.update(over)
    return obs_spec.ObsInputs(**base)  # type: ignore[arg-type]


def test_obs_spec_layout_and_values() -> None:
    names = obs_spec.OBS_NAMES
    o = obs_spec.build_v1(_inputs())[0]
    assert o.shape == (138,)
    assert o[names.index("opp_x")] == pytest.approx(0.5)
    assert o[names.index("opp_heading_cos")] == pytest.approx(-1.0)
    assert o[names.index("opp_heading_sigma")] == 1.0  # unknown heading reads 1
    assert o[names.index("ray_solid_0")] == pytest.approx(1.0 / 1.5)  # wall 1 m ahead
    assert o[names.index("ray_hole_0")] == 1.0
    assert o[names.index("hazard_0_present")] == 0.0
    absent = obs_spec.build_v1(_inputs(opp_present=torch.tensor([False])))[0]
    assert absent[names.index("opp_x")] == 0.0 and absent[names.index("opp_stale")] == 1.0


def test_obs_spec_hazards_sorted_by_edge_distance() -> None:
    names = obs_spec.OBS_NAMES
    hz = torch.tensor([[[0.8, 0.0], [0.0, 0.3], [-0.5, 0.0]]], dtype=torch.float64)
    o = obs_spec.build_v1(
        _inputs(
            hazard_xy=hz,
            hazard_radius=torch.tensor([[0.1, 0.1, 0.3]], dtype=torch.float64),
            hazard_kind=torch.tensor([[obs_spec.HOLE, obs_spec.WALL_BLOCK, obs_spec.TRACKED]]),
            hazard_velocity=torch.zeros(1, 3, 2, dtype=torch.float64),
            hazard_present=torch.ones(1, 3, dtype=torch.bool),
        )
    )[0]
    # Edge distances: 0.7, 0.2, 0.2 -> wall block (input order wins the tie), tracked, hole.
    assert o[names.index("hazard_0_wall_block")] == 1.0
    assert o[names.index("hazard_1_tracked")] == 1.0
    assert o[names.index("hazard_2_hole")] == 1.0
    assert o[names.index("hazard_3_present")] == 0.0
    # The hole at 0.8 m ahead: the forward hole ray hits its near edge at 0.7 m.
    assert o[names.index("ray_hole_0")] == pytest.approx(0.7 / 1.5)


class SmoothTwoStateEnv(TwoStateEnv):
    """Reward 1 - |linear - target|, target +0.6 in state 0 and -0.6 in state 1: a smooth
    reward, which a deterministic policy gradient can climb (the sign reward above is flat)."""

    def step(self, action: torch.Tensor) -> Step:
        want = torch.where(self.state == 0, 0.6, -0.6)
        reward = 1.0 - (action[:, 0] - want).abs()
        final_obs, final_priv = self._obs()
        self.state = torch.randint(2, (self.num_worlds,), generator=self.gen)
        obs, priv = self._obs()
        done = torch.ones(self.num_worlds, dtype=torch.bool)
        return Step(obs, priv, reward, done, torch.zeros_like(done), final_obs, final_priv)


def test_fasttd3_learns_smooth_two_state_toy() -> None:
    from auto_battlebot.rl.learners.fasttd3 import FastTd3, FastTd3Config

    torch.manual_seed(1)
    env = SmoothTwoStateEnv(256)
    policy = MlpGaussianPolicy(env.spec, MlpGaussianConfig(hidden=(32, 32)))
    cfg = FastTd3Config(
        batch_size=1024, critic_hidden=(64, 64), buffer_steps=64, v_min=-2, v_max=2, atoms=51
    )
    learner = FastTd3(policy, env, cfg, seed=1)  # type: ignore[arg-type]
    for _ in range(80):
        learner.iterate()
    action, _ = policy.act(torch.eye(2), None, deterministic=True)
    assert action[0, 0] == pytest.approx(0.6, abs=0.1)
    assert action[1, 0] == pytest.approx(-0.6, abs=0.1)


def test_toy_sensing_ekf_leaves_gaps() -> None:
    """The our-robot EKF error model must see fresh measurements as fresh. Fed the capture stamp,
    which always lags by the latency, it stayed in a gap forever and froze the rendered pose."""
    from auto_battlebot.rl.envs.toy import ToyConfig, ToyEnv

    env = ToyEnv(ToyConfig(worlds=256, sensing=True), "cpu", seed=4)
    env.reset(4)
    coasting = []
    for k in range(150):
        env.step(torch.tensor([[0.3, 0.2]]).expand(256, 2))
        if k >= 50:
            coasting.append(env.ekf.coasting.float().mean())
    assert float(torch.stack(coasting).mean()) < 0.3


def test_opponent_does_not_push_through_us() -> None:
    """A kinematic opponent driving straight at us may close only as far as contact (it would
    otherwise pry our chassis over), and keeps its speed when far away. Contact is the two
    footprint boxes along the line between centres."""
    from auto_battlebot.rl.envs.opponents import CONTACT_PRESS_M, STRAIGHT, Opponents

    opp = Opponents(2, torch.device("cpu"), torch.float64, torch.Generator().manual_seed(0))
    opp.p.behaviour = torch.full((2,), STRAIGHT, dtype=torch.long)
    opp.p.speed = torch.full((2,), 1.0, dtype=torch.float64)
    # Us at the origin facing +x; opponents 0.19 m and 1.0 m away on -x, heading +x. Our tail is
    # 0.0685 m behind our centre and the opponent's nose 0.111 m ahead of its centre.
    zero2 = torch.zeros(2, 2, dtype=torch.float64)
    opp.record_us(zero2, zero2, torch.zeros(2, dtype=torch.float64))
    xy = torch.tensor([[-0.19, 0.0], [-1.0, 0.0]], dtype=torch.float64)
    half = torch.tensor([[0.111, 0.127]], dtype=torch.float64).expand(2, 2)
    v = opp.command(
        xy,
        torch.zeros(2, dtype=torch.float64),
        torch.full((2, 2), 2.0, dtype=torch.float64),
        half.norm(dim=1),
        0.02,
        opp_half=half,
    )
    gap = 0.19 - 0.111 - 0.0685
    assert v[0, 0] == pytest.approx((gap + CONTACT_PRESS_M) / 0.02, abs=1e-9)
    assert v[1, 0] == pytest.approx(1.0, abs=1e-9)


@pytest.mark.skipif(not torch.cuda.is_available(), reason="WarpNavEnv runs on CUDA")
def test_warp_env_broadphase_bounds_follow_resized_geoms() -> None:
    """The environment resizes arena walls, wall blocks and the opponent per world. MuJoCo Warp
    culls contact pairs with geom_rbound and geom_aabb, so those must match what MuJoCo compiles
    for the new sizes, or the outer part of a block larger than the MJCF placeholder is passable.
    """
    import mujoco

    from auto_battlebot.rl.envs.warp_nav import WarpNavConfig, WarpNavEnv

    env = WarpNavEnv(WarpNavConfig(worlds=8, stage="D"), "cuda", seed=3)
    env.reset(3)
    sizes = env.m.geom_size.numpy()
    rbound = env.m.geom_rbound.numpy()
    aabb = env.m.geom_aabb.numpy()
    cases = [(g, mujoco.mjtGeom.mjGEOM_BOX) for g in (*env.wall_geoms, env.opp_geom)]
    cases += [(g, mujoco.mjtGeom.mjGEOM_CYLINDER) for g in env.block_geoms]
    for world in range(8):
        for g, kind in cases:
            spec = mujoco.MjSpec()
            n = 2 if kind == mujoco.mjtGeom.mjGEOM_CYLINDER else 3
            spec.worldbody.add_geom(type=kind, size=list(sizes[world, g][:n]) + [0.0] * (3 - n))
            model = spec.compile()
            assert rbound[world, g] == pytest.approx(model.geom_rbound[0], rel=1e-5)
            np.testing.assert_allclose(aabb[world, g].ravel(), model.geom_aabb[0], atol=1e-6)


@pytest.mark.skipif(not torch.cuda.is_available(), reason="WarpNavEnv runs on CUDA")
def test_standing_still_costs_reward_and_driving_does_not() -> None:
    """A parked robot pays still_per_s every second it is out of contact; a driving one does not."""
    from auto_battlebot.rl.envs.warp_nav import WarpNavConfig, WarpNavEnv

    worlds = 32
    env = WarpNavEnv(WarpNavConfig(worlds=worlds, stage="A"), "cuda", seed=4)
    env.reset(4)
    parked = torch.zeros(worlds, 2, device="cuda")
    for _ in range(50):
        env.step(parked)
    still_parked = env.stats["still_s"].clone()
    assert float(still_parked.median()) > 0.9  # about 1 s of the first 1 s

    env.reset(4)
    spin_and_drive = torch.tensor([[0.4, 0.3]], device="cuda").expand(worlds, 2).contiguous()
    for _ in range(50):
        env.step(spin_and_drive)
    # The first 0.2 s or so is spent getting up to speed from rest.
    assert float(env.stats["still_s"].median()) < 0.5 * float(still_parked.median())


def test_trained_opponent_draws_and_drives_each_behaviour() -> None:
    """Every training behaviour runs as a single sim opponent, inside the arena; pursuit closes
    on us and replay starts on its track."""
    from auto_battlebot.rl.envs.trained_opponent import TRAINED_BEHAVIOURS, TrainedOpponent

    with pytest.raises(ValueError, match="valid"):
        TrainedOpponent("sideways", False, 2.4, 2.4, (0.111, 0.127), 0)
    for name in TRAINED_BEHAVIOURS:
        opp = TrainedOpponent(name, False, 2.4, 2.4, (0.111, 0.127), seed=3)
        start = opp.reset((-0.9, -0.9))
        if name != "any":
            assert opp.behaviour == name
        assert (start is not None) == (opp.behaviour == "replay")
        pose = start or (0.6, 0.6, math.pi)
        opp.observe_us((-0.9, -0.9), (0.0, 0.0), 0.0)
        d0 = math.hypot(pose[0] + 0.9, pose[1] + 0.9)
        for _ in range(100):
            pose = opp.step(pose, 0.02)
            assert abs(pose[0]) <= 1.2 - 0.168 + 1e-9 and abs(pose[1]) <= 1.2 - 0.168 + 1e-9
        if name == "pursuit":
            assert math.hypot(pose[0] + 0.9, pose[1] + 0.9) < d0 - 0.3
        if name == "static":
            assert pose == (0.6, 0.6, math.pi)


def test_trained_opponent_replay_starts_clear_of_us() -> None:
    from auto_battlebot.rl.envs.trained_opponent import TrainedOpponent

    for seed in range(20):
        opp = TrainedOpponent("replay", False, 2.4, 2.4, (0.111, 0.127), seed=seed)
        start = opp.reset((-0.9, -0.9))
        assert start is not None
        assert math.hypot(start[0] + 0.9, start[1] + 0.9) > 0.168 + 0.106 + 0.3


def test_trained_opponent_does_not_push_through_us() -> None:
    from auto_battlebot.rl.envs.trained_opponent import TrainedOpponent

    opp = TrainedOpponent("pursuit", False, 2.4, 2.4, (0.111, 0.127), seed=1)
    opp.reset((0.0, 0.0))
    opp.observe_us((0.0, 0.0), (0.0, 0.0), 0.0)
    pose = (0.6, 0.0, math.pi)
    for _ in range(200):
        pose = opp.step(pose, 0.02)
    # Nose to nose: its half-length plus our nose distance. The old two-circle rule stopped it
    # at 0.33 m, out of contact.
    assert math.hypot(pose[0], pose[1]) == pytest.approx(0.111 + 0.0685, abs=0.01)


def test_pursuit_turns_proportionally_without_chatter() -> None:
    """The pursuer settles on its target instead of reversing its yaw rate every step."""
    from auto_battlebot.rl.envs.trained_opponent import TrainedOpponent

    opp = TrainedOpponent("pursuit", False, 2.4, 2.4, (0.111, 0.127), seed=2)
    opp.reset((0.8, 0.8))
    opp.observe_us((0.8, 0.8), (0.0, 0.0), 0.0)
    pose = (-0.8, -0.8, 0.0)
    yaws = []
    for _ in range(40):
        pose = opp.step(pose, 0.02)
        yaws.append(pose[2])
    rates = np.diff(np.unwrap(yaws[10:]))
    assert np.mean(np.sign(rates[1:]) != np.sign(rates[:-1])) < 0.2
    assert abs(math.atan2(0.8 - pose[1], 0.8 - pose[0]) - pose[2]) < 0.1


@pytest.mark.skipif(not torch.cuda.is_available(), reason="WarpNavEnv runs on CUDA")
def test_pinned_in_the_arc_keeps_costing_hits() -> None:
    """A static opponent parked nose-on against a robot that does nothing charges a hit about
    every arc_rehit_s, and the parked robot also pays the standing-still rate."""
    from auto_battlebot.rl.envs.warp_nav import OPP_MOCAP, WarpNavConfig, WarpNavEnv

    worlds = 16
    env = WarpNavEnv(WarpNavConfig(worlds=worlds, stage="A"), "cuda", seed=6)
    env.reset(6)

    def park_on_nose() -> None:
        # Glued against our nose and facing us, as a pursuer holds contact: its half-length plus
        # our nose (0.069 m from centre) less 1 cm.
        tr = env.truth()
        heading = torch.stack([torch.cos(tr["yaw"]), torch.sin(tr["yaw"])], 1)
        gap = env.opp_half[:, 0] + 0.059
        env.mocap_pos[:, OPP_MOCAP, :2] = tr["xy"] + heading * gap[:, None]
        yaw = tr["yaw"] + math.pi
        zero = torch.zeros_like(yaw)
        env.mocap_quat[:, OPP_MOCAP] = torch.stack(
            [torch.cos(yaw / 2), zero, zero, torch.sin(yaw / 2)], 1
        )

    for _ in range(100):  # 2 s
        park_on_nose()
        env.step(torch.zeros(worlds, 2, device="cuda"))
    touching = env.stats["hits_taken"] > 0
    assert int(touching.sum()) >= worlds // 2
    # 2 s pinned at one hit per 0.5 s; the first touch lands a step or two in.
    assert float(env.stats["hits_taken"][touching].median()) >= 3


def test_still_penalty_exempts_only_wedge_pushing() -> None:
    """Standing still is free only while our wedge is on the opponent from outside its arc;
    pinned inside its arc it costs both a hit every arc_rehit_s and the standing-still rate."""
    import json

    from auto_battlebot.rl.lockstep_world import LockstepWorld

    scenario = json.loads((REPO / "tests/python/data/rl_scenario_static.json").read_text())
    dt = 0.02
    cases = {
        "pushing": np.array([1, 0, 1, 2.5, 0, 0, 0], dtype=float),  # wedge on it, behind
        "pinned": np.array([1, 1, 0, 0.1, 0, 0, 0], dtype=float),  # in its arc
        "apart": np.zeros(7),
    }
    out = {}
    for name, flags in cases.items():
        world = LockstepWorld(scenario, timestep=2e-3)
        for _ in range(50):  # 1 s
            world._score_contacts(flags, dt)
            world.t += dt
        out[name] = (world.score.still_s, world.score.hits_taken)
    assert out["pushing"][0] == 0.0
    assert out["pinned"][0] == pytest.approx(1.0, abs=0.03)
    assert out["pinned"][1] == 2  # the first touch, then 0.5 s later
    assert out["apart"][0] == pytest.approx(1.0, abs=0.03)


def _pursuers(level: float) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    from auto_battlebot.rl.envs.opponents import PURSUIT, Opponents

    gen = torch.Generator().manual_seed(4)
    opp = Opponents(2048, torch.device("cpu"), torch.float32, gen)
    opp.pursuit_level = level
    ones = torch.ones(2048)
    opp.reset(torch.ones(2048, dtype=torch.bool), (PURSUIT,), ones, holdout=False)
    p = opp.p
    return p.speed, p.turn_rate, p.delay_steps.float() * 0.02


def test_pursuit_level_scales_pursuer_draws() -> None:
    """Pursuit level 0 draws only the easiest pursuer; level 1 reaches the hard end of each range,
    with the delay shrinking as the pursuer gets harder."""
    from auto_battlebot.rl.envs.opponents import PURSUIT, PURSUIT_EASY, PURSUIT_HARD, Opponents

    n = 4096
    mask = torch.ones(n, dtype=torch.bool)
    ones = torch.ones(n, dtype=torch.float64)
    opp = Opponents(n, torch.device("cpu"), torch.float64, torch.Generator().manual_seed(0))
    opp.pursuit_level = 0.0
    opp.reset(mask, (PURSUIT,), ones, holdout=False)
    assert torch.allclose(opp.p.speed, torch.full_like(opp.p.speed, PURSUIT_EASY[0]))
    assert torch.allclose(opp.p.turn_rate, torch.full_like(opp.p.turn_rate, PURSUIT_EASY[1]))
    assert bool((opp.p.delay_steps == round(PURSUIT_EASY[2] / 0.02)).all())

    opp.pursuit_level = 1.0
    opp.reset(mask, (PURSUIT,), ones, holdout=False)
    assert float(opp.p.speed.max()) > 0.95 * PURSUIT_HARD[0]
    assert float(opp.p.turn_rate.max()) > 0.95 * PURSUIT_HARD[1]
    assert int(opp.p.delay_steps.min()) <= round(0.07 / 0.02)


def test_flank_potential_rewards_side_and_rear_up_close() -> None:
    """0 in front of the opponent or far away, 1 behind it at its centre, rising with bearing."""
    from auto_battlebot.rl.envs.warp_nav import FLANK_RANGE_M, flank_potential

    arc = math.radians(60.0)
    yaw = torch.zeros(4, dtype=torch.float64)
    rel = torch.tensor(
        [[0.3, 0.0], [0.0, 0.3], [-0.3, 0.0], [-FLANK_RANGE_M - 0.1, 0.0]], dtype=torch.float64
    )
    front, side, rear, far = flank_potential(rel, yaw, arc).tolist()
    assert front == 0.0
    assert far == 0.0
    assert side == pytest.approx(1.0 - 0.3 / FLANK_RANGE_M)
    assert rear == pytest.approx(side)


def test_anchor_loss_counts_only_masked_samples_in_std_units() -> None:
    """Zero where the student matches the anchor, half the squared gap in std units elsewhere,
    and unmasked samples left out of both the sum and the count."""
    from auto_battlebot.rl.learners.ppo import Ppo

    mean = torch.tensor([[0.0, 0.0], [0.2, 0.0], [5.0, 5.0]])
    std = torch.full((3, 2), 0.1)
    flat = {
        "anchor_mean": torch.tensor([[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]),
        "anchor_w": torch.tensor([1.0, 1.0, 0.0]),
    }
    loss = Ppo._anchor_loss(mean, std, flat, torch.arange(3))
    assert float(loss) == pytest.approx(0.5 * (0.2 / 0.1) ** 2 / 2)


def test_pursuer_grip_limits_braking_and_turn_rate_at_speed() -> None:
    """A gripped pursuer at full speed whose target swings behind it cannot stop in one step,
    and turns no faster than grip / speed; without grip it stops and turns at its full rate."""
    from auto_battlebot.rl.envs.opponents import DELAY_SLOTS, PURSUIT, Opponents

    dt = 0.02
    xy = torch.zeros(2, 2, dtype=torch.float64)
    yaw = torch.zeros(2, dtype=torch.float64)
    half = torch.full((2, 2), 5.0, dtype=torch.float64)
    foot = torch.full((2,), 0.1, dtype=torch.float64)
    opp = Opponents(2, torch.device("cpu"), torch.float64, torch.Generator().manual_seed(0))
    ones = torch.ones(2, dtype=torch.float64)
    opp.reset(torch.ones(2, dtype=torch.bool), (PURSUIT,), ones, holdout=False)
    opp.p.speed[:] = 2.5
    opp.p.turn_rate[:] = 10.0
    opp.p.lead[:] = 0.0
    opp.p.grip = torch.tensor([10.0, 10.0], dtype=torch.float64)
    opp.forward[:] = 2.5
    # Our robot directly behind the pursuer: the target is 180 degrees off its heading.
    opp.history[:, :DELAY_SLOTS, 0] = -2.0
    opp.history[:, :DELAY_SLOTS, 1] = 0.01
    v = opp.command(xy, yaw, half, foot, dt)
    assert float(opp.forward[0]) == pytest.approx(2.5 - 10.0 * dt)
    assert float(v[0, 2].abs()) == pytest.approx(10.0 / (2.5 - 10.0 * dt))

    opp.p.grip = None
    opp.forward[:] = 2.5
    v = opp.command(xy, yaw, half, foot, dt)
    assert float(opp.forward[0]) == 0.0
    assert float(v[0, 2].abs()) == pytest.approx(10.0)
