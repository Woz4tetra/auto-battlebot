"""Write a model bundle directory for `RlPolicyNavigation` (plan section 12).

    <dir>/manifest.toml        policy kind, observation spec, decoder, execution, networks
    <dir>/actor.rtneural.json  the actor, input normalization folded into its first layer
    <dir>/selftest.bin         observation and decoded-action pairs from PyTorch

`selftest.bin` is little-endian: the bytes `RLST`, then uint32 format version (1), pair count N,
observation size D and action size A, then N x D float32 observations, then N x A float32
actions after the decoder (`scale * tanh(network output)`). The C++ loader runs every
observation through its runtime and decoder and requires agreement within the manifest's
tolerance.

Before writing, the exporter evaluates the JSON it is about to write in numpy and compares it
with PyTorch, so a layout mistake fails here rather than on the robot.
"""

from __future__ import annotations

import json
import struct
import subprocess
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch

from auto_battlebot.rl.export.rtneural import dense_layers, fold_normalization, run_json, to_json
from auto_battlebot.rl.policies.mlp import MlpGaussianPolicy

FORMAT_VERSION = 1
SELFTEST_MAGIC = b"RLST"
SELFTEST_VERSION = 1


@dataclass
class BundleSpec:
    run_id: str
    obs_spec_version: int
    obs_dim: int
    action_dim: int
    model: dict[str, object]
    selftest_obs: np.ndarray  # [N, D] float32
    selftest_actions: np.ndarray  # [N, A] float32
    decoder_scale: tuple[float, ...] = (1.0, 1.0)
    policy_period_ticks: int = 5
    tolerance: float = 1e-4


def git_hash(repo: Path) -> str:
    try:
        return subprocess.run(
            ["git", "-C", str(repo), "rev-parse", "--short", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


@torch.no_grad()
def spec_from_policy(
    policy: MlpGaussianPolicy,
    run_id: str,
    obs_spec_version: int,
    sample_obs: torch.Tensor,
    count: int = 16,
) -> BundleSpec:
    """Fold the frozen normalization into the actor and take `count` self-test pairs from
    `sample_obs` (real observations, so the test exercises the inputs the policy will see)."""
    norm = policy.norm
    layers = fold_normalization(
        dense_layers(policy.actor),
        norm.mean.double().cpu().numpy(),
        norm.std.double().cpu().numpy(),
    )
    model = to_json(layers)
    obs = sample_obs[:count].float().cpu()
    torch_actions = torch.tanh(policy.actor(norm(obs.to(norm.mean.device)))).cpu().double().numpy()
    json_actions = np.tanh(run_json(model, obs.double().numpy()))
    err = float(np.abs(json_actions - torch_actions).max())
    if err > 1e-4:
        raise RuntimeError(f"exported JSON disagrees with PyTorch by {err:.3g}")
    return BundleSpec(
        run_id=run_id,
        obs_spec_version=obs_spec_version,
        obs_dim=int(obs.shape[1]),
        action_dim=int(torch_actions.shape[1]),
        model=model,
        selftest_obs=obs.numpy().astype(np.float32),
        selftest_actions=torch_actions.astype(np.float32),
    )


def write_bundle(spec: BundleSpec, out_dir: Path, repo: Path) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "actor.rtneural.json").write_text(json.dumps(spec.model))
    with open(out_dir / "selftest.bin", "wb") as f:
        n = spec.selftest_obs.shape[0]
        f.write(SELFTEST_MAGIC)
        f.write(struct.pack("<IIII", SELFTEST_VERSION, n, spec.obs_dim, spec.action_dim))
        f.write(spec.selftest_obs.astype("<f4").tobytes())
        f.write(spec.selftest_actions.astype("<f4").tobytes())
    scale = ", ".join(f"{s:.6g}" for s in spec.decoder_scale)
    manifest = f"""format_version = {FORMAT_VERSION}
run_id = "{spec.run_id}"
git_hash = "{git_hash(repo)}"
policy_kind = "NETWORK"
obs_spec_version = {spec.obs_spec_version}
execution = "INLINE"
policy_period_ticks = {spec.policy_period_ticks}

[decoder]
kind = "SQUASHED"
scale = [{scale}]

[[networks]]
name = "actor"
runtime = "RTNEURAL"
file = "actor.rtneural.json"
inputs = [{{ name = "obs.vector", shape = [{spec.obs_dim}] }}]
outputs = [{{ name = "action", shape = [{spec.action_dim}] }}]

[selftest]
file = "selftest.bin"
tolerance = {spec.tolerance:g}
"""
    (out_dir / "manifest.toml").write_text(manifest)
    return out_dir


def read_selftest(path: Path) -> tuple[np.ndarray, np.ndarray]:
    data = path.read_bytes()
    if data[:4] != SELFTEST_MAGIC:
        raise ValueError(f"{path}: not a self-test file")
    version, n, d, a = struct.unpack("<IIII", data[4:20])
    if version != SELFTEST_VERSION:
        raise ValueError(f"{path}: self-test version {version}, expected {SELFTEST_VERSION}")
    obs = np.frombuffer(data, dtype="<f4", count=n * d, offset=20).reshape(n, d)
    act = np.frombuffer(data, dtype="<f4", count=n * a, offset=20 + 4 * n * d).reshape(n, a)
    return obs, act
