"""Write `tests/rl_policy/bundles/tiny_mlp/`, the 4-input bundle the C++ unit tests load."""

from __future__ import annotations

from pathlib import Path

import torch

from auto_battlebot.rl.env import EnvSpec
from auto_battlebot.rl.export.bundle import spec_from_policy, write_bundle
from auto_battlebot.rl.policies.mlp import MlpGaussianConfig, MlpGaussianPolicy

REPO = Path(__file__).resolve().parents[3]


def main() -> None:
    torch.manual_seed(7)
    spec = EnvSpec(
        obs_names=("a", "b", "c", "d"),
        privileged_names=(),
        obs_spec_version=0,
        policy_dt_s=0.02,
        max_episode_steps=1,
    )
    policy = MlpGaussianPolicy(spec, MlpGaussianConfig(hidden=(8,)))
    with torch.no_grad():
        for p in policy.actor.parameters():
            p.copy_(torch.randn_like(p) * 0.7)
        policy.norm.mean.copy_(torch.tensor([0.5, -1.0, 2.0, 0.0]))
        policy.norm.var.copy_(torch.tensor([4.0, 0.25, 1.0, 9.0]))
    obs = torch.randn(16, 4) * 2.0
    bundle = spec_from_policy(policy, "tiny_mlp", 0, obs)
    out = write_bundle(bundle, REPO / "tests/rl_policy/bundles/tiny_mlp", REPO)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
