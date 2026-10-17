"""Train or evaluate one arm of the RL toy viability test (plan step 1).

One TOML per arm picks a source, a policy and a learner by registry name. A config with no
`[learner]` table only evaluates its policy (the scripted baselines). Output goes to
`runs/rl_toy/<name>_s<seed>_<timestamp>/`: `metrics.jsonl` (training and evaluation rows),
`checkpoint.pt` and `final_eval.json`.

    source scripts/activate_python.sh
    python playground/rl_toy/train_toy.py playground/rl_toy/configs/ppo_static.toml --seed 0
"""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import torch

from auto_battlebot.rl.config import RunConfig, build
from auto_battlebot.rl.envs.toy import ToyConfig, ToyEnv
from auto_battlebot.rl.evaluate import evaluate
from auto_battlebot.rl.learners.base import LEARNERS
from auto_battlebot.rl.learners.ppo import PpoConfig
from auto_battlebot.rl.policies.base import POLICIES
from auto_battlebot.rl.policies.mlp import MlpGaussianConfig
from auto_battlebot.rl.policies.scripted import PursuitConfig

REPO = Path(__file__).resolve().parents[2]
POLICY_CONFIGS: dict[str, Any] = {"mlp_gaussian": MlpGaussianConfig, "pursuit": PursuitConfig}
LEARNER_CONFIGS: dict[str, Any] = {"ppo": PpoConfig}


def learner_config(kind: str, table: dict[str, Any]) -> Any:
    if kind == "fasttd3":
        from auto_battlebot.rl.learners.fasttd3 import FastTd3Config

        return build(FastTd3Config, table, "learner")
    return build(LEARNER_CONFIGS[kind], table, "learner")


def make_env(source: dict[str, Any], device: str, seed: int, worlds: int | None = None) -> ToyEnv:
    table = {k: v for k, v in source.items() if k != "type"}
    if worlds is not None:
        table["worlds"] = worlds
    return ToyEnv(build(ToyConfig, table, "source"), device, seed=seed)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("config", type=Path)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--out", type=Path, default=REPO / "runs" / "rl_toy")
    ap.add_argument("--wall-clock-s", type=float, default=None)
    ap.add_argument("--env-steps", type=int, default=None)
    ap.add_argument("--sensing", action="store_true", help="evaluate with the sensing model on")
    args = ap.parse_args()

    run = RunConfig.load(args.config)
    if args.seed is not None:
        run.seed = args.seed
    if args.wall_clock_s is not None:
        run.budget.wall_clock_s = args.wall_clock_s
    if args.env_steps is not None:
        run.budget.env_steps = args.env_steps
    if args.sensing:
        run.source["sensing"] = True
    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    out = args.out / f"{run.name}_s{run.seed}_{stamp}"
    out.mkdir(parents=True, exist_ok=True)
    (out / "config.json").write_text(
        json.dumps(run.__dict__, default=lambda o: o.__dict__, indent=2)
    )
    torch.manual_seed(run.seed)

    eval_env = make_env(run.source, args.device, seed=run.eval.seed, worlds=run.eval.episodes)
    ptype = run.policy["type"]
    pcfg = build(
        POLICY_CONFIGS[ptype], {k: v for k, v in run.policy.items() if k != "type"}, "policy"
    )
    policy = POLICIES[ptype](eval_env.spec, pcfg)
    log = open(out / "metrics.jsonl", "w")

    def write(row: dict[str, Any]) -> None:
        log.write(json.dumps(row) + "\n")
        log.flush()

    if not run.learner:
        metrics = evaluate(policy, eval_env, run.eval.seed)
        write({"kind": "eval", "env_steps": 0, "wall_s": 0.0, **metrics.as_dict()})
        (out / "final_eval.json").write_text(json.dumps(metrics.as_dict(), indent=2))
        print(json.dumps(metrics.as_dict(), indent=2))
        return

    env = make_env(run.source, args.device, seed=run.seed)
    lkind = run.learner["type"]
    lcfg = learner_config(lkind, {k: v for k, v in run.learner.items() if k != "type"})
    learner = LEARNERS[lkind](policy, env, lcfg, seed=run.seed)
    start = time.time()
    last_eval = start
    it = 0
    while (
        learner.env_steps < run.budget.env_steps and time.time() - start < run.budget.wall_clock_s
    ):
        stats = learner.iterate()
        it += 1
        now = time.time()
        write({"kind": "train", "iteration": it, "wall_s": now - start, **stats})
        if it % 10 == 0:
            keys = (
                "env_steps",
                "ep_success",
                "ep_strikes",
                "ep_hits_taken",
                "ep_reward",
                "kl",
                "std",
            )
            print(
                f"[{now - start:7.0f}s] it {it} "
                + " ".join(f"{k}={stats.get(k, float('nan')):.4g}" for k in keys),
                flush=True,
            )
        if now - last_eval >= run.budget.eval_every_s:
            metrics = evaluate(policy, eval_env, run.eval.seed)
            write(
                {
                    "kind": "eval",
                    "env_steps": learner.env_steps,
                    "wall_s": now - start,
                    **metrics.as_dict(),
                }
            )
            print(f"  eval: {json.dumps(metrics.as_dict())}", flush=True)
            torch.save(learner.state_dict(), out / "checkpoint.pt")
            last_eval = time.time()

    metrics = evaluate(policy, eval_env, run.eval.seed)
    write(
        {
            "kind": "eval",
            "env_steps": learner.env_steps,
            "wall_s": time.time() - start,
            **metrics.as_dict(),
        }
    )
    torch.save(learner.state_dict(), out / "checkpoint.pt")
    final = {"env_steps": learner.env_steps, "wall_s": time.time() - start, **metrics.as_dict()}
    (out / "final_eval.json").write_text(json.dumps(final, indent=2))
    print(json.dumps(final, indent=2))
    print(f"run directory: {out}")


if __name__ == "__main__":
    main()
