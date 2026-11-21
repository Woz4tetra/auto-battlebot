import ast

import numpy as np
import pytest
import torch

from auto_battlebot.paths import REPO_ROOT

REPO = REPO_ROOT
KERNEL_MODULES = (
    "auto_battlebot/rl/envs/warp_kernels.py",
    "auto_battlebot/mujoco_sim/warp_kernels.py",
)
needs_cuda = pytest.mark.skipif(not torch.cuda.is_available(), reason="WarpNavEnv runs on CUDA")


@pytest.mark.parametrize("relative", KERNEL_MODULES)
def test_warp_kernel_modules_keep_annotations_as_live_types(relative: str) -> None:
    tree = ast.parse((REPO / relative).read_text())
    futures = [
        alias.name
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom) and node.module == "__future__"
        for alias in node.names
    ]
    assert "annotations" not in futures


@needs_cuda
def test_mj_quat_reads_mujoco_wxyz_order_for_yaw() -> None:
    import warp as wp

    from auto_battlebot.rl.envs import warp_kernels as k

    @wp.kernel
    def read_yaw(qpos: wp.array2d(dtype=float), out: wp.array(dtype=float)):
        w = wp.tid()
        out[w] = k.yaw_of_warp_quat(k.mj_quat(qpos, w))

    yaw = 0.7
    qpos = np.zeros((1, 7), dtype=np.float32)
    qpos[0, 3:] = [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
    out = wp.zeros(1, dtype=float, device="cuda")
    wp.launch(read_yaw, dim=1, inputs=[wp.array(qpos, dtype=float, device="cuda"), out])
    assert float(out.numpy()[0]) == pytest.approx(yaw, abs=1e-5)


@needs_cuda
def test_truth_snapshot_survives_the_next_step() -> None:
    from auto_battlebot.rl.envs.warp_nav import WarpNavConfig, WarpNavEnv

    env = WarpNavEnv(WarpNavConfig(worlds=4, stage="A"), "cuda", seed=2)
    env.reset(2)
    env.step(torch.zeros(4, 2, device="cuda"))
    before = {key: value.clone() for key, value in env.truth().items()}
    held = env.truth()
    env.step(torch.full((4, 2), 0.8, device="cuda"))
    for key in ("xy", "velocity"):
        assert torch.equal(held[key], before[key])


@pytest.mark.slow
@needs_cuda
def test_env_stays_finite_with_zero_floor_friction() -> None:
    from auto_battlebot.rl.envs.warp_nav import WarpNavConfig, WarpNavEnv

    worlds = 16
    env = WarpNavEnv(WarpNavConfig(worlds=worlds, stage="D"), "cuda", seed=5)
    env.reset(5)
    gen = torch.Generator(device="cuda").manual_seed(1)
    for _ in range(3000):
        env.step(torch.rand(worlds, 2, device="cuda", generator=gen) * 2 - 1)
        assert torch.isfinite(env.qpos).all()


@needs_cuda
def test_cpu_weapon_sweep_matches_the_warp_kernel() -> None:
    import json

    import mujoco
    import warp as wp

    from auto_battlebot.mujoco_sim import mass_properties
    from auto_battlebot.rl.envs import warp_kernels as k
    from auto_battlebot.rl.envs.warp_nav import OPP_MOCAP
    from auto_battlebot.rl.lockstep_world import LockstepWorld

    @wp.kernel
    def sweep_hit(
        qpos: wp.array2d(dtype=float),
        sweep: wp.vec4,
        sweep_y: float,
        centre: wp.vec3,
        yaw: float,
        half: wp.vec3,
        out: wp.array(dtype=int),
    ):
        w = wp.tid()
        out[w] = k.weapon_sweep_hit(qpos, w, sweep, sweep_y, centre, yaw, half)

    scenario = json.loads(
        (REPO / "auto_battlebot/rl/envs/tests/data/rl_scenario_static.json").read_text()
    )
    world = LockstepWorld(scenario, timestep=2e-3)
    mk3 = mass_properties.load(mass_properties.asset_dir("mrs_buff_mk3") / "mass_properties.toml")
    world.mp = mk3
    sweep = mk3.weapon
    half = world.model.geom_size[world.opp_geom]
    sweep_vec = wp.vec4(sweep.center[0], sweep.center[2], sweep.radius, sweep.half_width)
    rng = np.random.default_rng(0)
    hits = 0
    for _ in range(3000):
        yaw = rng.uniform(-np.pi, np.pi)
        pos = np.array([rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3), 0.05])
        quat = np.zeros(4)
        mujoco.mju_axisAngle2Quat(quat, np.array([0.0, 0.0, 1.0]), yaw)
        world.data.qpos[:3] = pos
        world.data.qpos[3:7] = quat
        world.data.mocap_pos[OPP_MOCAP] = [rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3), 0.05]
        opp_yaw = rng.uniform(-np.pi, np.pi)
        world.data.mocap_quat[OPP_MOCAP] = [np.cos(opp_yaw / 2), 0, 0, np.sin(opp_yaw / 2)]
        flat = np.empty(9)
        mujoco.mju_quat2Mat(flat, world.data.qpos[3:7])
        cpu = world._sweep_hits(flat.reshape(3, 3))
        ox, oy, oyaw = world.opponent_pose()
        out = wp.zeros(1, dtype=int, device="cuda")
        wp.launch(
            sweep_hit,
            dim=1,
            inputs=[
                wp.array(world.data.qpos[None, :7].astype(np.float32), dtype=float, device="cuda"),
                sweep_vec,
                float(sweep.center[1]),
                wp.vec3(ox, oy, float(world.data.mocap_pos[OPP_MOCAP][2])),
                float(oyaw),
                wp.vec3(*half),
                out,
            ],
            device="cuda",
        )
        assert bool(out.numpy()[0]) == cpu
        hits += cpu
    assert 100 < hits < 2900
