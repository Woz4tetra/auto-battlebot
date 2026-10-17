"""Export a training checkpoint as a model bundle for `RlPolicyNavigation` (plan step 8).

The self-test pairs are real observations: the checkpoint's policy is run for a second in the
`EVAL` environment and 16 of the observations it saw are kept. The exporter checks the RTNeural
JSON against PyTorch before writing (`auto_battlebot/rl/export/bundle.py`); the C++ side runs the
same pairs again at startup.

    source scripts/activate_python.sh
    python playground/rl/export_checkpoint.py runs/rl_nav/<run>/checkpoint.pt \\
        --out data/models/nav_policies/rl_nav_<run>
"""

from __future__ import annotations

import argparse
from pathlib import Path

import torch

from auto_battlebot.rl.envs.warp_nav import WarpNavConfig, WarpNavEnv
from auto_battlebot.rl.export.bundle import spec_from_policy, write_bundle
from auto_battlebot.rl.policies.mlp import MlpGaussianPolicy

REPO = Path(__file__).resolve().parents[2]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("checkpoint", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--run-id", default=None)
    args = ap.parse_args()
    env = WarpNavEnv(WarpNavConfig(worlds=64, stage="EVAL", holdout=True), "cuda", seed=7)
    policy = MlpGaussianPolicy(env.spec).cuda()
    policy.load_state_dict(torch.load(args.checkpoint, map_location="cuda")["learner"]["policy"])
    obs, _ = env.reset(7)
    seen = []
    for _ in range(50):
        action, _ = policy.act(obs, None, True)
        obs = env.step(action).obs
        seen.append(obs[:1])
    sample = torch.cat(seen)[::3][:16]
    run_id = args.run_id or args.out.name
    bundle = spec_from_policy(policy, run_id, env.spec.obs_spec_version, sample)
    write_bundle(bundle, args.out, REPO)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
