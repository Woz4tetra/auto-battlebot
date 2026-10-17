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
    """A kinematic opponent driving straight at us within contact range loses the closing part of
    its velocity (it would otherwise pry our chassis over), and keeps it when far away."""
    from auto_battlebot.rl.envs.opponents import STRAIGHT, Opponents

    opp = Opponents(2, torch.device("cpu"), torch.float64, torch.Generator().manual_seed(0))
    opp.p.behaviour = torch.full((2,), STRAIGHT, dtype=torch.long)
    opp.p.speed = torch.full((2,), 1.0, dtype=torch.float64)
    # Us at the origin; opponents at 0.3 m and 1.0 m on -x, both heading +x (towards us).
    opp.record_us(torch.zeros(2, 2, dtype=torch.float64), torch.zeros(2, 2, dtype=torch.float64))
    xy = torch.tensor([[-0.3, 0.0], [-1.0, 0.0]], dtype=torch.float64)
    v = opp.command(
        xy,
        torch.zeros(2, dtype=torch.float64),
        torch.full((2, 2), 2.0, dtype=torch.float64),
        torch.full((2,), 0.17, dtype=torch.float64),
        0.02,
    )
    assert v[0, 0] == pytest.approx(0.0, abs=1e-9)
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
