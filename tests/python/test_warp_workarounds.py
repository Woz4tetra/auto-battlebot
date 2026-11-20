import ast
from pathlib import Path

import numpy as np
import pytest
import torch

REPO = Path(__file__).resolve().parents[2]
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


@needs_cuda
def test_env_stays_finite_with_zero_floor_friction() -> None:
    from auto_battlebot.rl.envs.warp_nav import WarpNavConfig, WarpNavEnv

    worlds = 16
    env = WarpNavEnv(WarpNavConfig(worlds=worlds, stage="D"), "cuda", seed=5)
    env.reset(5)
    gen = torch.Generator(device="cuda").manual_seed(1)
    for _ in range(200):
        env.step(torch.rand(worlds, 2, device="cuda", generator=gen) * 2 - 1)
        assert torch.isfinite(env.qpos).all()
