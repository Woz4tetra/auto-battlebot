"""Train the RL navigation policy on the Warp environment through the curriculum (plan step 6, 7).

Stages A to D advance when the rolling success rate (attack: at least one strike; run away: no
hit taken, no fall, no flip) over the last 2000 episodes passes `--advance-at` (70%). Output goes
to `runs/rl_nav/<name>_<timestamp>/`: `metrics.jsonl`, `checkpoint.pt` (rewritten every
`--checkpoint-every-s`), `stage_<X>.pt` at each stage transition, and `bundle/`, the exported
model bundle (plan section 12) written at the end.

    source scripts/activate_python.sh
    python training/rl/train_nav_policy.py --hours 3
    python training/rl/train_nav_policy.py --resume runs/rl_nav/<run>/checkpoint.pt --hours 2
"""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import torch

from auto_battlebot.rl.envs.warp_nav import STAGES, WarpNavConfig, WarpNavEnv
from auto_battlebot.rl.export.bundle import spec_from_policy, write_bundle
from auto_battlebot.rl.learners.ppo import Ppo, PpoConfig
from auto_battlebot.rl.policies.mlp import MlpGaussianPolicy

REPO = Path(__file__).resolve().parents[2]
ORDER = ["A", "B", "C", "D"]
CHOICES = list(STAGES)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--name", default="rl_nav_ppo")
    ap.add_argument("--worlds", type=int, default=4096)
    ap.add_argument("--episode-s", type=float, default=20.0, help="time limit per episode")
    ap.add_argument("--stage", default="A", choices=CHOICES)
    ap.add_argument("--final-stage", default="D", choices=CHOICES)
    ap.add_argument("--hours", type=float, default=3.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--advance-at", type=float, default=0.7)
    ap.add_argument(
        "--max-stage-steps",
        type=float,
        default=250e6,
        help="advance anyway after this many environment steps in a stage (logged as a fallback)",
    )
    ap.add_argument(
        "--stage-steps-so-far", type=float, default=0.0, help="steps already in the resumed stage"
    )
    ap.add_argument("--resume", type=Path, default=None)
    ap.add_argument("--force-stage", action="store_true", help="use --stage, not the resumed one")
    ap.add_argument(
        "--reset-log-std",
        type=float,
        default=None,
        help="after --resume, set the policy's action log-std (exploration) to this; a long run "
        "shrinks it to almost nothing, and fine-tuning on a changed reward needs it back",
    )
    ap.add_argument("--checkpoint-every-s", type=float, default=600.0)
    ap.add_argument("--out", type=Path, default=REPO / "runs" / "rl_nav")
    args = ap.parse_args()

    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    out = args.out / f"{args.name}_s{args.seed}_{stamp}"
    out.mkdir(parents=True, exist_ok=True)
    torch.manual_seed(args.seed)
    stage = args.stage
    resume = torch.load(args.resume, map_location="cuda") if args.resume else None
    if resume is not None and not args.force_stage:
        stage = resume.get("stage", stage)
    env = WarpNavEnv(
        WarpNavConfig(worlds=args.worlds, stage=stage, episode_s=args.episode_s),
        "cuda",
        seed=args.seed,
    )
    policy = MlpGaussianPolicy(env.spec)
    learner = Ppo(policy, env, PpoConfig(), seed=args.seed)
    if resume is not None:
        learner.load_state_dict(resume["learner"])
        if args.reset_log_std is not None:
            with torch.no_grad():
                policy.log_std.fill_(args.reset_log_std)
    (out / "config.json").write_text(
        json.dumps(
            {**vars(args), "resume": str(args.resume), "stage": stage}, default=str, indent=2
        )
    )
    log = open(out / "metrics.jsonl", "w")
    start = time.time()
    last_ckpt = start
    stage_start = learner.env_steps - args.stage_steps_so_far
    it = 0

    def save(path: Path) -> None:
        torch.save(
            {"learner": learner.state_dict(), "stage": env.stage, "env_steps": learner.env_steps},
            path,
        )

    while time.time() - start < args.hours * 3600:
        stats = learner.iterate()
        it += 1
        row = {
            "iteration": it,
            "wall_s": time.time() - start,
            "stage": env.stage,
            "success": env.success_rate,
            **stats,
        }
        log.write(json.dumps(row) + "\n")
        log.flush()
        if it % 20 == 0:
            keys = (
                "env_steps",
                "ep_success",
                "ep_strikes",
                "ep_hits_taken",
                "ep_holes",
                "ep_flips",
                "ep_sim_failures",
                "ep_reward",
                "kl",
                "std",
            )
            print(
                f"[{row['wall_s']:7.0f}s] {env.stage} it {it} success={env.success_rate:.3f} "
                + " ".join(f"{k}={stats.get(k, float('nan')):.4g}" for k in keys),
                flush=True,
            )
        passed = len(env.success_window) >= 2000 and env.success_rate >= args.advance_at
        timed_out = learner.env_steps - stage_start >= args.max_stage_steps
        if env.stage != args.final_stage and (passed or timed_out):
            save(out / f"stage_{env.stage}.pt")
            nxt = ORDER[ORDER.index(env.stage) + 1]
            how = "passed" if passed else f"fallback at success {env.success_rate:.3f}"
            print(
                f"advancing {env.stage} -> {nxt} at {learner.env_steps:,} env steps ({how})",
                flush=True,
            )
            env.set_stage(nxt)
            env.success_window = []
            stage_start = learner.env_steps
        if time.time() - last_ckpt >= args.checkpoint_every_s:
            save(out / "checkpoint.pt")
            last_ckpt = time.time()

    save(out / "checkpoint.pt")
    sample, _ = env.reset(args.seed + 999)
    bundle = spec_from_policy(
        policy, f"{args.name}_s{args.seed}_{stamp}", env.spec.obs_spec_version, sample[:64]
    )
    write_bundle(bundle, out / "bundle", REPO)
    print(f"run directory: {out}")


if __name__ == "__main__":
    main()
