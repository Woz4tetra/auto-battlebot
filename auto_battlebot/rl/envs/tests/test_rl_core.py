from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
import torch

from auto_battlebot.compat import tomllib
from auto_battlebot.control.plant import PlantParams, PlantState, integrate_step
from auto_battlebot.control.plant import steady_state as np_steady_state
from auto_battlebot.paths import REPO_ROOT
from auto_battlebot.rl import obs_spec
from auto_battlebot.rl.env import EnvSpec, Step
from auto_battlebot.rl.export.bundle import read_selftest, spec_from_policy, write_bundle
from auto_battlebot.rl.export.bundle_policy import (
    BundlePolicy,
    Hazard,
    HazardKind,
    PolicyView,
    Pose,
)
from auto_battlebot.rl.export.rtneural import run_json
from auto_battlebot.rl.learners.ppo import Ppo, PpoConfig
from auto_battlebot.rl.policies.mlp import MlpGaussianConfig, MlpGaussianPolicy, RunningNorm
from auto_battlebot.rl.policies.scripted import PursuitConfig, PursuitPolicy
from auto_battlebot.rl.sidestep import Sidestep, SidestepConfig
from auto_battlebot.rl.torch_plant import TorchPlantParams, integrate, saturate, steady_state

DEADZONE = 0.02
BEHIND_PURSUER_XY = (-2.0, 0.01)
NOSE_CONTACT_M = 0.059
PARKED_S = 1.0
STILL_S = 1.0
RAY_RANGE_M = 1.5
UNKNOWN_HEADING_SIGMA = 1.0
REPO = REPO_ROOT


class TwoStateEnv:
    """One-hot state 0 or 1; reward +1 when the linear sign is + in state 0 and - in state 1."""

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


@pytest.mark.slow
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


def test_bundle_policy_matches_torch_and_pushes_history(tmp_path: Path) -> None:
    torch.manual_seed(4)
    spec = EnvSpec(obs_spec.OBS_NAMES, (), obs_spec.OBS_SPEC_VERSION, 0.02, 1)
    policy = MlpGaussianPolicy(spec, MlpGaussianConfig(hidden=(16, 8)))
    sample = torch.rand(16, obs_spec.OBS_DIM)
    out = write_bundle(spec_from_policy(policy, "p", 1, sample), tmp_path / "bundle", REPO)
    runner = BundlePolicy(out)
    view = PolicyView(
        our_pose=Pose(-0.5, 0.2, 0.3),
        our_speed_mps=1.2,
        our_yaw_rate_rad_s=-2.0,
        target_pose=Pose(0.4, -0.1, 2.0),
        target_velocity_mps=(0.5, 0.0),
        target_yaw_rate_rad_s=1.0,
        target_half_diagonal_m=0.106,
        field_half_m=(1.2, 1.2),
        hazards=[Hazard(0.0, 0.85, 0.2083, HazardKind.TRACKED)],
    )
    obs = runner.observe(view)
    assert obs.shape == (1, obs_spec.OBS_DIM)
    linear, angular = runner.act(view)
    expected = torch.tanh(policy.actor(policy.norm(torch.from_numpy(obs).float())))[0]
    np.testing.assert_allclose([linear, angular], expected.detach().numpy(), atol=1e-5)
    pushed = runner.observe(view)[0]
    names = obs_spec.OBS_NAMES
    assert pushed[names.index("hist_action_linear_0")] == pytest.approx(linear)
    assert pushed[names.index("hist_speed_0")] == pytest.approx(1.2 / obs_spec.SPEED_SCALE)
    assert pushed[names.index("hist_action_linear_1")] == 0.0


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
    angular = 0.6
    linear_left = 1.0 - angular
    right_cmd = linear_left - angular
    right = -(abs(right_cmd) - DEADZONE) / (1.0 - DEADZONE)
    assert out[0, 0] == pytest.approx(0.5 * (1.0 + right), abs=1e-6)
    assert out[0, 1] == pytest.approx(0.5 * (1.0 - right), abs=1e-6)
    assert out[1].abs().max() == 0.0
    assert out[2, 0] == pytest.approx(-(0.5 - DEADZONE) / (1.0 - DEADZONE), abs=1e-6)


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
    assert o[names.index("opp_heading_sigma")] == UNKNOWN_HEADING_SIGMA
    wall_ahead_m = 1.0
    assert o[names.index("ray_solid_0")] == pytest.approx(wall_ahead_m / RAY_RANGE_M)
    assert o[names.index("ray_hole_0")] == 1.0
    assert o[names.index("hazard_0_present")] == 0.0
    absent = obs_spec.build_v1(_inputs(opp_present=torch.tensor([False])))[0]
    assert absent[names.index("opp_x")] == 0.0 and absent[names.index("opp_stale")] == 1.0


def test_obs_spec_hazards_sorted_by_edge_distance() -> None:
    names = obs_spec.OBS_NAMES
    hole_ahead_m, hole_radius = 0.8, 0.1
    hz = torch.tensor([[[hole_ahead_m, 0.0], [0.0, 0.3], [-0.5, 0.0]]], dtype=torch.float64)
    o = obs_spec.build_v1(
        _inputs(
            hazard_xy=hz,
            hazard_radius=torch.tensor([[hole_radius, 0.1, 0.3]], dtype=torch.float64),
            hazard_kind=torch.tensor([[obs_spec.HOLE, obs_spec.WALL_BLOCK, obs_spec.TRACKED]]),
            hazard_velocity=torch.zeros(1, 3, 2, dtype=torch.float64),
            hazard_present=torch.ones(1, 3, dtype=torch.bool),
        )
    )[0]
    assert o[names.index("hazard_0_wall_block")] == 1.0, "equal edge distances keep input order"
    assert o[names.index("hazard_1_tracked")] == 1.0
    assert o[names.index("hazard_2_hole")] == 1.0
    assert o[names.index("hazard_3_present")] == 0.0
    hole_near_edge = hole_ahead_m - hole_radius
    assert o[names.index("ray_hole_0")] == pytest.approx(hole_near_edge / RAY_RANGE_M)


class SmoothTwoStateEnv(TwoStateEnv):
    """Reward 1 - |linear - target|, target +0.6 in state 0 and -0.6 in state 1."""

    def step(self, action: torch.Tensor) -> Step:
        want = torch.where(self.state == 0, 0.6, -0.6)
        reward = 1.0 - (action[:, 0] - want).abs()
        final_obs, final_priv = self._obs()
        self.state = torch.randint(2, (self.num_worlds,), generator=self.gen)
        obs, priv = self._obs()
        done = torch.ones(self.num_worlds, dtype=torch.bool)
        return Step(obs, priv, reward, done, torch.zeros_like(done), final_obs, final_priv)


@pytest.mark.slow
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


@pytest.mark.slow
def test_toy_sensing_ekf_leaves_gaps() -> None:
    """The our-robot EKF error model is fed delivery time, so fresh measurements read as fresh."""
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
    """A kinematic opponent driving straight at us closes only to contact, and keeps its speed."""
    from auto_battlebot.rl.envs.opponents import (
        CONTACT_PRESS_M,
        OUR_HALF_EXTENT_M,
        STRAIGHT,
        Opponents,
    )

    our_tail_m = OUR_HALF_EXTENT_M[0]
    opp_nose_m = 0.111
    near_start_m = 0.19
    far_start_m = 1.0

    opp = Opponents(2, torch.device("cpu"), torch.float64, torch.Generator().manual_seed(0))
    opp.p.behaviour = torch.full((2,), STRAIGHT, dtype=torch.long)
    opp.p.speed = torch.full((2,), 1.0, dtype=torch.float64)
    zero2 = torch.zeros(2, 2, dtype=torch.float64)
    opp.record_us(zero2, zero2, torch.zeros(2, dtype=torch.float64))
    xy = torch.tensor([[-near_start_m, 0.0], [-far_start_m, 0.0]], dtype=torch.float64)
    half = torch.tensor([[opp_nose_m, 0.127]], dtype=torch.float64).expand(2, 2)
    v = opp.command(
        xy,
        torch.zeros(2, dtype=torch.float64),
        torch.full((2, 2), 2.0, dtype=torch.float64),
        half.norm(dim=1),
        0.02,
        opp_half_length_width=half,
    )
    gap = near_start_m - opp_nose_m - our_tail_m
    assert v[0, 0] == pytest.approx((gap + CONTACT_PRESS_M) / 0.02, abs=1e-9)
    assert v[1, 0] == pytest.approx(1.0, abs=1e-9)


@pytest.mark.gpu
@pytest.mark.skipif(not torch.cuda.is_available(), reason="WarpNavEnv runs on CUDA")
def test_warp_env_broadphase_bounds_follow_resized_geoms() -> None:
    """geom_rbound and geom_aabb match what MuJoCo compiles for the resized geoms."""
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


@pytest.mark.gpu
@pytest.mark.skipif(not torch.cuda.is_available(), reason="WarpNavEnv runs on CUDA")
def test_standing_still_costs_reward_and_driving_does_not() -> None:
    """A parked robot pays still_per_s every second it is out of contact; a driving one does not."""
    from auto_battlebot.rl.envs.warp_nav import WarpNavConfig, WarpNavEnv

    worlds = 32
    env = WarpNavEnv(WarpNavConfig(worlds=worlds, stage="A"), "cuda", seed=4)
    env.reset(4)
    parked = torch.zeros(worlds, 2, device="cuda")
    for _ in range(int(PARKED_S / env.cfg.policy_dt_s)):
        env.step(parked)
    still_parked = env.stats["still_s"].clone()
    assert float(still_parked.median()) > 0.9 * PARKED_S

    env.reset(4)
    spin_and_drive = torch.tensor([[0.4, 0.3]], device="cuda").expand(worlds, 2).contiguous()
    for _ in range(50):
        env.step(spin_and_drive)
    assert float(env.stats["still_s"].median()) < 0.5 * float(still_parked.median()), (
        "only the start-up from rest counts as still"
    )


def test_trained_opponent_draws_and_drives_each_behaviour() -> None:
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
    opp_half_length_m = 0.111
    our_nose_distance_m = 0.0685
    nose_to_nose_m = opp_half_length_m + our_nose_distance_m
    assert math.hypot(pose[0], pose[1]) == pytest.approx(nose_to_nose_m, abs=0.01)


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


@pytest.mark.gpu
@pytest.mark.skipif(not torch.cuda.is_available(), reason="WarpNavEnv runs on CUDA")
def test_pinned_in_the_arc_keeps_costing_hits() -> None:
    from auto_battlebot.rl.envs.warp_nav import OPP_MOCAP, WarpNavConfig, WarpNavEnv

    worlds = 16
    cfg = WarpNavConfig(worlds=worlds, stage="A")
    env = WarpNavEnv(cfg, "cuda", seed=6)
    env.reset(6)

    def park_on_nose() -> None:
        tr = env.truth()
        heading = torch.stack([torch.cos(tr["yaw"]), torch.sin(tr["yaw"])], 1)
        gap = env.opp_half[:, 0] + NOSE_CONTACT_M
        env.mocap_pos[:, OPP_MOCAP, :2] = tr["xy"] + heading * gap[:, None]
        yaw = tr["yaw"] + math.pi
        zero = torch.zeros_like(yaw)
        env.mocap_quat[:, OPP_MOCAP] = torch.stack(
            [torch.cos(yaw / 2), zero, zero, torch.sin(yaw / 2)], 1
        )

    pinned_s = 2.0
    for _ in range(int(pinned_s / cfg.policy_dt_s)):
        park_on_nose()
        env.step(torch.zeros(worlds, 2, device="cuda"))
    touching = env.stats["hits_taken"] > 0
    assert int(touching.sum()) >= worlds // 2
    expected_hits = int(pinned_s / cfg.arc_rehit_s) - 1
    assert float(env.stats["hits_taken"][touching].median()) >= expected_hits


def _contact_flags(contact: bool, in_arc: bool, wedge_on: bool, angle: float) -> np.ndarray:
    return np.array([contact, in_arc, wedge_on, angle, 0, 0, 0], dtype=float)


def test_still_penalty_exempts_only_wedge_pushing() -> None:
    import json

    from auto_battlebot.rl.lockstep_world import LockstepWorld

    scenario = json.loads(
        (REPO / "auto_battlebot/rl/envs/tests/data/rl_scenario_static.json").read_text()
    )
    dt = 0.02
    cases = {
        "pushing": _contact_flags(contact=True, in_arc=False, wedge_on=True, angle=2.5),
        "pinned": _contact_flags(contact=True, in_arc=True, wedge_on=False, angle=0.1),
        "apart": np.zeros(7),
    }
    out = {}
    for name, flags in cases.items():
        world = LockstepWorld(scenario, timestep=2e-3)
        for _ in range(int(STILL_S / dt)):
            world._score_contacts(flags, dt)
            world.t += dt
        out[name] = (world.score.still_s, world.score.hits_taken)
    assert out["pushing"][0] == 0.0
    assert out["pinned"][0] == pytest.approx(1.0, abs=0.03)
    assert out["pinned"][1] == int(STILL_S / world.cfg.arc_rehit_s)
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
    """Zero where the student matches the anchor, half the squared std-unit gap elsewhere."""
    from auto_battlebot.rl.learners.ppo import Ppo

    mean = torch.tensor([[0.0, 0.0], [0.2, 0.0], [5.0, 5.0]])
    std = torch.full((3, 2), 0.1)
    flat = {
        "anchor_mean": torch.tensor([[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]),
        "anchor_w": torch.tensor([1.0, 1.0, 0.0]),
    }
    loss = Ppo._anchor_loss(mean, std, flat, torch.arange(3))
    masked_count = int(flat["anchor_w"].sum())
    assert float(loss) == pytest.approx(0.5 * (0.2 / 0.1) ** 2 / masked_count)


def test_pursuer_grip_limits_braking_and_turn_rate_at_speed() -> None:
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
    opp.commanded_forward_mps[:] = 2.5
    opp.history[:, :DELAY_SLOTS, 0] = BEHIND_PURSUER_XY[0]
    opp.history[:, :DELAY_SLOTS, 1] = BEHIND_PURSUER_XY[1]
    v = opp.command(xy, yaw, half, foot, dt)
    assert float(opp.commanded_forward_mps[0]) == pytest.approx(2.5 - 10.0 * dt)
    assert float(v[0, 2].abs()) == pytest.approx(10.0 / (2.5 - 10.0 * dt))

    opp.p.grip = None
    opp.commanded_forward_mps[:] = 2.5
    v = opp.command(xy, yaw, half, foot, dt)
    assert float(opp.commanded_forward_mps[0]) == 0.0
    assert float(v[0, 2].abs()) == pytest.approx(10.0)


def test_lockstep_slew_and_guard_stages() -> None:
    import json

    from auto_battlebot.rl.lockstep_world import LockstepWorld

    base = json.loads(
        (REPO / "auto_battlebot/rl/envs/tests/data/rl_scenario_static.json").read_text()
    )
    dt = 0.02

    def run(**stages: float) -> np.ndarray:
        world = LockstepWorld({**base, **stages, "trace": True}, timestep=2e-3)
        world.step(0.0, 0.0, dt)
        for _ in range(30):
            world.step(-0.9, 0.0, dt)
        return np.asarray(world.score.extra["trace"])

    neither = run()
    np.testing.assert_allclose(neither[:, 1], neither[:, 0])
    slew = run(slew_per_s=1.0)
    np.testing.assert_allclose(slew[1:6, 1], [-0.02, -0.04, -0.06, -0.08, -0.10], atol=1e-9)
    guard = run(guard_scale=0.5)
    np.testing.assert_allclose(guard[:, 1], neither[:, 1])
    k = _first_step_tick(neither[:, 2])
    assert guard[k, 2] == pytest.approx(0.5 * neither[k, 2], rel=0.05)
    assert neither[k, 2] < -50.0


def _first_step_tick(wheel_command: np.ndarray) -> int:
    return int(np.argmax(np.abs(wheel_command) > 1.0))


def test_step_response_helpers() -> None:
    from auto_battlebot.mujoco_sim.step_response import StickTape, ideal_step, loosest_slew

    tape = ideal_step(0.6, slew_per_s=2.0, duration_s=0.5)
    np.testing.assert_allclose(tape.linear_frac_fwd[:3], [0.02, 0.04, 0.06])
    assert tape.linear_frac_fwd[-1] == pytest.approx(0.6)
    held = StickTape.resample(np.array([0.0, 0.025]), np.array([0.1, 0.5]), np.zeros(2), 0.05)
    np.testing.assert_allclose(held.linear_frac_fwd, [0.1, 0.1, 0.1, 0.5, 0.5])
    assert loosest_slew([(1.0, 50.0)], 30.0) is None
    assert loosest_slew([(1.0, 2.0), (0.0, 3.0)], 30.0) == 0.0


def test_loosest_slew_stops_at_first_failure_from_slow_end() -> None:
    from auto_battlebot.mujoco_sim.step_response import loosest_slew

    sweep = [(0.0, 95.0), (1.0, 2.0), (4.0, 5.0), (8.0, 99.0), (12.0, 3.0)]
    assert loosest_slew(sweep, 30.0) == 4.0


def test_step_result_final_obs_is_previous_episode() -> None:
    from auto_battlebot.rl.envs.toy import ToyConfig, ToyEnv

    cfg = ToyConfig(worlds=8, episode_s=0.1)
    env = ToyEnv(cfg, "cpu", seed=1)
    env.reset(seed=1)
    steps = env.max_steps
    action = torch.full((cfg.worlds, 2), 0.5)
    for _ in range(steps - 1):
        result = env.step(action)
        assert not bool(result.done.any())
    result = env.step(action)
    assert bool(result.truncated.all())
    assert not torch.equal(result.final_obs, result.obs)
    assert not torch.equal(result.final_privileged, result.privileged)
    assert result.obs.shape == result.final_obs.shape
    assert set(result.episode) >= {"reward"}
    assert result.episode["reward"].shape == (cfg.worlds,)


def test_pursuit_level_one_matches_uncurriculum_draw() -> None:
    from auto_battlebot.rl.envs.opponents import (
        PURSUIT_COL_DELAY,
        PURSUIT_COL_TOP_SPEED,
        PURSUIT_COL_TURN_RATE,
        Opponents,
    )

    opp = Opponents(1, torch.device("cpu"), torch.float64, torch.Generator())
    u = torch.linspace(0.0, 1.0, 11, dtype=torch.float64)
    bounds = {
        PURSUIT_COL_TOP_SPEED: (0.8, 2.5),
        PURSUIT_COL_TURN_RATE: (4.0, 10.0),
        PURSUIT_COL_DELAY: (0.05, 0.25),
    }
    opp.pursuit_level = 1.0
    for column, (lo, hi) in bounds.items():
        expected = lo + (hi - lo) * u
        torch.testing.assert_close(opp._pursuit_draw(u, column), expected)
    opp.pursuit_level = 0.0
    for column, (lo, hi) in bounds.items():
        easy = hi if column == PURSUIT_COL_DELAY else lo
        torch.testing.assert_close(opp._pursuit_draw(u, column), torch.full_like(u, easy))


def _pursuit_corner_draws(holdout: bool) -> tuple[torch.Tensor, torch.Tensor]:
    from auto_battlebot.rl.envs.opponents import PURSUIT, Opponents

    worlds = 8192
    opp = Opponents(worlds, torch.device("cpu"), torch.float64, torch.Generator().manual_seed(5))
    opp.pursuit_level = 1.0
    mask = torch.ones(worlds, dtype=torch.bool)
    opp.reset(mask, (PURSUIT,), torch.ones(worlds, dtype=torch.float64), holdout=holdout)
    return opp.p.speed, opp.p.delay_steps


def test_training_pursuit_never_draws_holdout_corner() -> None:
    from auto_battlebot.rl.envs.opponents import PURSUIT_HOLDOUT_DELAY, PURSUIT_HOLDOUT_SPEED

    last_corner_step = int(PURSUIT_HOLDOUT_DELAY / 0.02)
    speed, steps = _pursuit_corner_draws(holdout=False)
    in_corner = (speed > PURSUIT_HOLDOUT_SPEED) & (steps <= last_corner_step)
    assert not bool(in_corner.any())
    assert bool((speed > PURSUIT_HOLDOUT_SPEED).any())
    speed, steps = _pursuit_corner_draws(holdout=True)
    assert bool(((speed >= PURSUIT_HOLDOUT_SPEED) & (steps <= last_corner_step)).all())


def test_opponent_draws_stable_for_seed() -> None:
    from auto_battlebot.rl.envs.opponents import Opponents

    opp = Opponents(6, torch.device("cpu"), torch.float64, torch.Generator().manual_seed(11))
    opp.reset(torch.ones(6, dtype=torch.bool), tuple(range(6)), torch.ones(6), holdout=False)
    p = opp.p
    assert p.behaviour.tolist() == [3, 3, 0, 5, 5, 3]
    assert p.delay_steps.tolist() == [11, 10, 11, 9, 12, 9]
    assert p.corner.tolist() == [1, 1, 1, 0, 2, 1]
    expected_speed = [0.62834, 1.16723, 0.48564, 0.51012, 0.46446, 0.29739]
    expected_turn_rate = [4.49459, 5.81029, 9.90659, 7.59701, 7.53208, 9.23573]
    assert p.speed.tolist() == pytest.approx(expected_speed, abs=1e-5)
    assert p.turn_rate.tolist() == pytest.approx(expected_turn_rate, abs=1e-5)


def test_opponent_params_reject_bad_columns() -> None:
    from auto_battlebot.rl.envs.opponents import OpponentParams

    def build(**overrides: torch.Tensor) -> OpponentParams:
        z = torch.zeros(2)
        zl = torch.zeros(2, dtype=torch.long)
        fields = dict(
            behaviour=zl,
            speed=z,
            radius=z,
            direction=torch.ones(2),
            noise=z,
            lead=z,
            delay_steps=zl,
            turn_rate=z,
            corner=zl,
        )
        return OpponentParams(**{**fields, **overrides})

    build()
    with pytest.raises(TypeError, match="behaviour"):
        build(behaviour=torch.zeros(2))
    with pytest.raises(ValueError, match="direction"):
        build(direction=torch.tensor([1.0, 0.0]))
    with pytest.raises(ValueError, match="corner"):
        build(corner=torch.tensor([0, 4]))


@pytest.mark.gpu
def test_policy_opponent_input_is_perspective_swapped() -> None:
    from auto_battlebot.rl.envs.warp_nav import WarpNavConfig, WarpNavEnv

    if not torch.cuda.is_available():
        pytest.skip("WarpNavEnv runs on CUDA")
    env = WarpNavEnv(WarpNavConfig(worlds=4, stage="A"), "cuda", seed=3)
    env.reset(3)
    seen = {}

    def capture(inp: object, yaw: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        seen["inp"] = inp
        return torch.zeros(4, 3, device="cuda"), torch.zeros(4, dtype=torch.bool, device="cuda")

    env.policy_opp.command = capture  # type: ignore[method-assign]
    tr = env.truth()
    env._policy_opponent_command(tr)
    inp = seen["inp"]
    assert torch.equal(inp.our_xy, tr["opp_xy"])
    assert torch.equal(inp.opp_xy, tr["xy"])
    assert torch.equal(inp.our_yaw, tr["opp_yaw"])
    assert torch.equal(inp.opp_yaw, tr["yaw"])


@pytest.mark.gpu
@pytest.mark.skipif(not torch.cuda.is_available(), reason="WarpNavEnv runs on CUDA")
def test_warp_command_chain_signs_match_the_app() -> None:
    from auto_battlebot.rl.envs.warp_nav import WarpNavConfig, WarpNavEnv

    worlds = 8
    env = WarpNavEnv(WarpNavConfig(worlds=worlds, stage="A"), "cuda", seed=4)

    def drive(linear: float, angular: float, steps: int) -> tuple[torch.Tensor, torch.Tensor]:
        env.reset(4)
        before = env.truth()
        action = torch.tensor([[linear, angular]], device="cuda").expand(worlds, 2).contiguous()
        for _ in range(steps):
            env.step(action)
        after = env.truth()
        heading = torch.stack([torch.cos(before["yaw"]), torch.sin(before["yaw"])], 1)
        along = ((after["xy"] - before["xy"]) * heading).sum(1)
        turned = torch.remainder(after["yaw"] - before["yaw"] + math.pi, 2 * math.pi) - math.pi
        return along, turned

    along, _ = drive(0.4, 0.0, 25)
    assert float(along.median()) > 0.05
    _, turned = drive(0.0, 0.4, 5)
    assert float(turned.median()) > 0.05


class FinishEveryStepEnv:
    def __init__(self) -> None:
        self.num_worlds = 3
        self.device = torch.device("cpu")
        self.spec = EnvSpec(("o",), (), 0, 0.02, 2)
        self.calls = 0

    def reset(self, seed: int) -> tuple[torch.Tensor, torch.Tensor]:
        return torch.zeros(3, 1), torch.zeros(3, 0)

    def step(self, action: torch.Tensor) -> Step:
        self.calls += 1
        done = torch.tensor([True, self.calls >= 2, True])
        episode = {
            "strikes": torch.full((3,), float(self.calls)),
            "hits_taken": torch.zeros(3),
            "first_strike_s": torch.full((3,), 0.1),
            "first_hit_s": torch.full((3,), float("inf")),
        }
        zeros = torch.zeros(3, 1)
        return Step(
            zeros,
            torch.zeros(3, 0),
            torch.zeros(3),
            done,
            torch.zeros_like(done),
            zeros,
            torch.zeros(3, 0),
            episode,
        )


class ZeroPolicy:
    def act(
        self, obs: torch.Tensor, state: object, deterministic: bool
    ) -> tuple[torch.Tensor, None]:
        return torch.zeros(obs.shape[0], 2), None

    def reset_state(self, state: object, done: torch.Tensor) -> None:
        return None


def test_evaluate_scores_first_episode_per_world_only() -> None:
    from auto_battlebot.rl.evaluate import evaluate

    env = FinishEveryStepEnv()
    metrics = evaluate(ZeroPolicy(), env, seed=0)  # type: ignore[arg-type]
    assert env.calls == 2
    assert metrics.strikes_per_episode == pytest.approx(4 / 3)


def test_run_json_applies_kernel_orientation_and_every_activation() -> None:
    kernel = [[1.0, 2.0, -1.0], [0.5, -1.0, 3.0]]
    bias = [0.1, -0.2, 0.0]
    x = np.array([[2.0, -1.0]])
    pre = x @ np.array(kernel) + np.array(bias)
    expected = {
        "": pre,
        "relu": np.maximum(pre, 0.0),
        "tanh": np.tanh(pre),
        "sigmoid": 1.0 / (1.0 + np.exp(-pre)),
        "elu": np.where(pre > 0, pre, np.expm1(pre)),
    }
    for name, want in expected.items():
        model = {"layers": [{"weights": [kernel, bias], "activation": name}]}
        np.testing.assert_allclose(run_json(model, x), want, atol=1e-12, err_msg=name)
    with pytest.raises(ValueError, match="unknown activation"):
        run_json({"layers": [{"weights": [kernel, bias], "activation": "swish"}]}, x)


def test_ppo_normalizer_updates_after_epochs() -> None:
    torch.manual_seed(0)
    env = TwoStateEnv()
    policy = MlpGaussianPolicy(env.spec, MlpGaussianConfig(hidden=(8, 8)))
    learner = Ppo(policy, env, PpoConfig(steps_per_update=4, critic_hidden=(8, 8)), seed=0)  # type: ignore[arg-type]
    seen = []
    distribution = policy.distribution

    def spy(obs: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        seen.append(float(policy.norm.count))
        return distribution(obs)

    policy.distribution = spy  # type: ignore[method-assign]
    before = float(policy.norm.count)
    learner.iterate()
    assert set(seen) == {before}
    assert float(policy.norm.count) > before


def test_hazard_and_pose_types_name_their_fields() -> None:
    from auto_battlebot.rl.envs.warp_nav import ROBOTS, ArcRule, CommandPath, Weapon

    assert Pose(1.0, 2.0, 3.0).yaw == 3.0
    assert int(HazardKind.TRACKED) == obs_spec.TRACKED
    assert ROBOTS["mr_stabs_mk2"].command is CommandPath.ONBOARD_MIXER
    assert ROBOTS["mrs_buff_mk3"].weapon is Weapon.DISK_SWEEP_CYLINDER
    assert ROBOTS["mrs_buff_mk3"].arc_rule is ArcRule.FRONT_FACE_IN_ARC


def test_bundle_spec_rejects_mismatched_selftest_arrays() -> None:
    from auto_battlebot.rl.export.bundle import BundleSpec

    good = np.zeros((2, 3), dtype=np.float32)
    BundleSpec("r", 1, 3, 3, {}, good, good)
    with pytest.raises(ValueError, match="selftest_obs"):
        BundleSpec("r", 1, 4, 3, {}, good, good)
    with pytest.raises(ValueError, match="selftest_actions"):
        BundleSpec("r", 1, 3, 3, {}, good, good.astype(np.float64))


@pytest.mark.gpu
@pytest.mark.skipif(not torch.cuda.is_available(), reason="WarpNavEnv runs on CUDA")
def test_privileged_layout_stable_for_mr_stabs() -> None:
    from auto_battlebot.rl.envs.warp_nav import PRIVILEGED_NAMES, WarpNavConfig, WarpNavEnv

    stabs = WarpNavEnv(WarpNavConfig(worlds=2, stage="A"), "cuda", seed=1)
    assert stabs.privileged_names == PRIVILEGED_NAMES


@pytest.mark.gpu
@pytest.mark.skipif(not torch.cuda.is_available(), reason="WarpNavEnv runs on CUDA")
def test_set_stage_enabling_sensing_initializes_all_worlds() -> None:
    from auto_battlebot.rl.envs.warp_nav import WarpNavConfig, WarpNavEnv

    env = WarpNavEnv(WarpNavConfig(worlds=4, stage="A"), "cuda", seed=2)
    env.reset(2)
    assert not env.sensing
    env.opp_filter.last_measured.fill_(123.0)
    env.set_stage("C")
    assert env.sensing
    assert bool((env.opp_filter.last_measured == env.t - 1.0).all())


def _episode(done: int, **cols: float) -> dict[str, torch.Tensor]:
    keys = ("strikes", "hits_taken", "holes", "flips", "run_away", "behaviour", "driver")
    return {k: torch.full((done,), cols.get(k, 0.0), device="cuda") for k in keys}


@pytest.mark.gpu
@pytest.mark.skipif(not torch.cuda.is_available(), reason="WarpNavEnv runs on CUDA")
def test_curriculum_counts_only_current_stage_episodes() -> None:
    from auto_battlebot.rl.envs.warp_nav import WarpNavConfig, WarpNavEnv

    env = WarpNavEnv(WarpNavConfig(worlds=4, stage="B"), "cuda", seed=3)
    env.episode_stage = torch.tensor([env.stage_index, env.stage_index - 1] * 2, device="cuda")
    done = torch.ones(4, dtype=torch.bool, device="cuda")
    env.record_success(_episode(4, strikes=1.0), done)
    assert env.success_window == [1.0, 1.0]


@pytest.mark.gpu
@pytest.mark.skipif(not torch.cuda.is_available(), reason="WarpNavEnv runs on CUDA")
def test_pursuit_window_counts_only_current_level() -> None:
    from auto_battlebot.rl.envs.opponents import PURSUIT
    from auto_battlebot.rl.envs.warp_nav import WarpNavConfig, WarpNavEnv

    env = WarpNavEnv(WarpNavConfig(worlds=4, stage="P"), "cuda", seed=3)
    level = env.opponents.pursuit_level
    env.episode_stage = torch.full((4,), env.stage_index, device="cuda")
    env.episode_pursuit_level = torch.tensor([level, level + 0.5] * 2, device="cuda")
    done = torch.ones(4, dtype=torch.bool, device="cuda")
    env.record_success(_episode(4, strikes=1.0, behaviour=float(PURSUIT)), done)
    assert env.pursuit_attack_success_window == [1.0, 1.0]


@pytest.mark.gpu
@pytest.mark.skipif(not torch.cuda.is_available(), reason="WarpNavEnv runs on CUDA")
def test_replay_episode_scene_clears_track_start() -> None:
    from auto_battlebot.rl.envs.opponents import BEHAVIOURS, REPLAY
    from auto_battlebot.rl.envs.warp_nav import OPP_MOCAP, WarpNavConfig, WarpNavEnv

    assert BEHAVIOURS[REPLAY] == "replay"
    env = WarpNavEnv(WarpNavConfig(worlds=64, stage="D"), "cuda", seed=5)
    assert env.opponents.replay is not None
    for seed in (5, 6):
        env.reset(seed)
        replaying = env.opponents.p.behaviour == REPLAY
        assert bool(replaying.any())
        reach = env.arena_half - env.opp_half.norm(dim=1)[:, None] - 0.05
        xy, _ = env.opponents.replay.sample(
            env.opponents.replay_index, env.opponents.replay_t, reach
        )
        placed = env.mocap_pos[:, OPP_MOCAP, :2]
        assert torch.allclose(placed[replaying], xy[replaying], atol=1e-5)


@pytest.mark.gpu
@pytest.mark.skipif(not torch.cuda.is_available(), reason="WarpNavEnv runs on CUDA")
def test_export_scenarios_after_steps_refused() -> None:
    from auto_battlebot.rl.envs.warp_nav import WarpNavConfig, WarpNavEnv

    env = WarpNavEnv(WarpNavConfig(worlds=2, stage="A"), "cuda", seed=7)
    env.reset(7)
    assert len(env.export_scenarios()) == 2
    env.step(torch.zeros(2, 2, device="cuda"))
    with pytest.raises(RuntimeError, match="straight after reset"):
        env.export_scenarios()


@pytest.mark.gpu
@pytest.mark.skipif(not torch.cuda.is_available(), reason="WarpNavEnv runs on CUDA")
def test_diverged_world_ends_without_reward() -> None:
    from auto_battlebot.rl.envs.warp_nav import WarpNavConfig, WarpNavEnv

    env = WarpNavEnv(WarpNavConfig(worlds=4, stage="A"), "cuda", seed=8)
    env.reset(8)
    env.qpos[1, 0] = float("nan")
    step = env.step(torch.zeros(4, 2, device="cuda"))
    assert bool(step.terminated[1])
    assert float(step.reward[1]) == 0.0
    assert bool(torch.isfinite(step.reward).all())


@pytest.mark.gpu
@pytest.mark.skipif(not torch.cuda.is_available(), reason="WarpNavEnv runs on CUDA")
def test_warp_sensing_ekf_leaves_gaps() -> None:
    from auto_battlebot.rl.envs.warp_nav import WarpNavConfig, WarpNavEnv

    env = WarpNavEnv(WarpNavConfig(worlds=64, stage="C"), "cuda", seed=4)
    env.reset(4)
    coasting = []
    for k in range(150):
        env.step(torch.tensor([[0.3, 0.2]], device="cuda").expand(64, 2))
        if k >= 50:
            coasting.append(env.ekf.coasting.float().mean())
    assert float(torch.stack(coasting).mean()) < 0.3


def test_ray_box_zero_outside() -> None:
    origin = torch.tensor([[2.0, 0.0], [0.0, 0.0]], dtype=torch.float64)
    direction = torch.tensor([[[1.0, 0.0]], [[1.0, 0.0]]], dtype=torch.float64)
    half = torch.tensor([[1.0, 1.0], [1.0, 1.0]], dtype=torch.float64)
    out = obs_spec.ray_box(origin, direction, half)
    assert out[0, 0] == 0.0
    assert out[1, 0] == pytest.approx(1.0)


def test_ray_discs_inside_hits_at_zero() -> None:
    origin = torch.zeros(1, 2, dtype=torch.float64)
    direction = torch.tensor([[[1.0, 0.0]]], dtype=torch.float64)
    centre = torch.zeros(1, 1, 2, dtype=torch.float64)
    radius = torch.ones(1, 1, dtype=torch.float64)
    mask = torch.ones(1, 1, dtype=torch.bool)
    assert obs_spec.ray_discs(origin, direction, centre, radius, mask)[0, 0] == 0.0
    far = obs_spec.ray_discs(origin, direction, centre + torch.tensor([3.0, 0.0]), radius, mask)
    assert far[0, 0] == pytest.approx(2.0)


def test_hazard_ties_keep_input_order() -> None:
    names = obs_spec.OBS_NAMES
    hz = torch.tensor([[[0.5, 0.0], [-0.5, 0.0]]], dtype=torch.float64)
    o = obs_spec.build_v1(
        _inputs(
            hazard_xy=hz,
            hazard_radius=torch.tensor([[0.1, 0.1]], dtype=torch.float64),
            hazard_kind=torch.tensor([[obs_spec.HOLE, obs_spec.TRACKED]]),
            hazard_velocity=torch.zeros(1, 2, 2, dtype=torch.float64),
            hazard_present=torch.ones(1, 2, dtype=torch.bool),
        )
    )[0]
    assert o[names.index("hazard_0_hole")] == 1.0
    assert o[names.index("hazard_1_tracked")] == 1.0


def test_running_norm_constant_input_has_unit_std() -> None:
    norm = RunningNorm(2)
    x = torch.stack([torch.zeros(200_000), torch.linspace(-1.0, 1.0, 200_000)], dim=1)
    norm.update(x)
    assert norm.std[0] == 1.0
    assert torch.isfinite(norm(x)).all()


def _pursuit_reference(x: float, y: float, cfg: PursuitConfig) -> tuple[float, float]:
    error = math.atan2(y, x)
    distance = math.hypot(x, y)
    angular = cfg.angular_kp * error
    near, far = cfg.velocity_ramp_near_distance, cfg.velocity_ramp_far_distance
    if distance <= near:
        speed = 1.0
    elif distance >= far:
        speed = cfg.velocity_ramp_min_scale
    else:
        speed = 1.0 - (distance - near) / (far - near) * (1.0 - cfg.velocity_ramp_min_scale)
    turn = 1.0 - abs(error) / cfg.angle_threshold * 0.5
    brake = max(0.0, 1.0 - 0.5 * abs(angular))
    linear = 0.0 if abs(error) >= cfg.angle_threshold else speed * turn * brake
    return min(linear, cfg.max_linear_command), angular


def test_scripted_pursuit_matches_reference_control_law() -> None:
    cfg = PursuitConfig(max_linear_command=1.0)
    spec = EnvSpec(("opp_x", "opp_y", "opp_known"), (), 0, 0.02, 1)
    policy = PursuitPolicy(spec, cfg)
    for x, y in [(1.2, 0.3), (0.6, -0.2), (3.0, 1.0), (-1.0, 0.5)]:
        obs = torch.zeros(1, spec.obs_dim)
        obs[0, spec.index("opp_x")] = x / 2.0
        obs[0, spec.index("opp_y")] = y / 2.0
        obs[0, spec.index("opp_known")] = 1.0
        action, _ = policy.act(obs)
        linear, angular = _pursuit_reference(x, y, cfg)
        assert action[0, 0].item() == pytest.approx(linear, abs=1e-5)
        assert action[0, 1].item() == pytest.approx(angular, abs=1e-5)
    assert action[0, 0] == 0.0


def test_sidestep_keeps_orbit_side_across_centre_line() -> None:
    controller = Sidestep(1, "cpu", SidestepConfig())

    def truth(phi_deg: float) -> dict[str, torch.Tensor]:
        phi = math.radians(phi_deg)
        return {
            "xy": torch.tensor([[math.cos(phi), math.sin(phi)]]),
            "yaw": torch.zeros(1),
            "yaw_rate": torch.zeros(1),
            "opp_xy": torch.zeros(1, 2),
            "opp_yaw": torch.zeros(1),
            "reach": torch.zeros(1),
        }

    controller(truth(40.0))
    assert controller.side.item() == 1.0
    for phi_deg in (10.0, -10.0, -25.0):
        controller(truth(phi_deg))
        assert controller.side.item() == 1.0
    controller(truth(-40.0))
    assert controller.side.item() == -1.0
