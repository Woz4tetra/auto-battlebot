"""The export_checkpoint CLI turns a training checkpoint into a bundle the C++ policy loads."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
import torch

from auto_battlebot.compat import tomllib
from auto_battlebot.paths import REPO_ROOT
from auto_battlebot.rl.envs.warp_nav import WarpNavConfig, WarpNavEnv
from auto_battlebot.rl.export.bundle import read_selftest
from auto_battlebot.rl.export.rtneural import run_json
from auto_battlebot.rl.policies.mlp import MlpGaussianPolicy


@pytest.mark.slow
@pytest.mark.skipif(not torch.cuda.is_available(), reason="the exporter rolls the policy on CUDA")
def test_cli_writes_a_bundle_whose_selftest_matches_the_checkpoint(tmp_path: Path) -> None:
    torch.manual_seed(5)
    env = WarpNavEnv(WarpNavConfig(worlds=1, stage="EVAL", robot="mr_stabs_mk2"), "cuda", seed=1)
    policy = MlpGaussianPolicy(env.spec)
    with torch.no_grad():
        for parameter in policy.actor.parameters():
            parameter.copy_(torch.randn_like(parameter) * 0.3)
    checkpoint = tmp_path / "policy.pt"
    torch.save({"robot": "mr_stabs_mk2", "learner": {"policy": policy.state_dict()}}, checkpoint)
    out = tmp_path / "bundle"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "auto_battlebot.rl.export_checkpoint",
            str(checkpoint),
            "--out",
            str(out),
            "--run-id",
            "cli_test",
        ],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
    )
    manifest = tomllib.loads((out / "manifest.toml").read_text())
    assert manifest["obs_spec_version"] == env.spec.obs_spec_version
    obs, actions = read_selftest(out / "selftest.bin")
    assert len(obs) > 0
    expected = torch.tanh(policy.actor(policy.norm(torch.tensor(obs, dtype=torch.float32))))
    np.testing.assert_allclose(actions, expected.detach().numpy(), atol=1e-4)
    model = json.loads((out / "actor.rtneural.json").read_text())
    np.testing.assert_allclose(np.tanh(run_json(model, obs.astype(float))), actions, atol=1e-4)
