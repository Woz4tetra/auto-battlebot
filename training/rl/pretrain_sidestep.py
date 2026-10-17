"""Warm-start the navigation policy on the scripted sidestep controller before PPO (DAgger).

Runs stage P (half the episodes against a pursuer) from a trained checkpoint. In attack episodes
against a pursuer the label is the sidestep controller's action on true state
(`auto_battlebot/rl/sidestep.py`); everywhere else the label is the starting policy's own
deterministic action, so the other behaviours are kept rather than forgotten. Each round rolls
out a mix of expert and student actions (the expert's share falls from 1 to 0 across rounds),
adds the observations and labels to the data set, and fits the actor mean to the labels. The
critic and observation normalization are left as they are. The output is a checkpoint in the
training format, for `train_nav_policy.py --resume`.

    source scripts/activate_python.sh
    python training/rl/pretrain_sidestep.py runs/rl_nav/<run>/checkpoint.pt \\
        --out runs/rl_nav/sidestep_bc/checkpoint.pt --expert '{"flank_m": 0.4}'
"""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path

import torch

from auto_battlebot.rl.envs.opponents import PURSUIT
from auto_battlebot.rl.envs.warp_nav import WarpNavConfig, WarpNavEnv
from auto_battlebot.rl.learners.ppo import Ppo, PpoConfig
from auto_battlebot.rl.policies.mlp import MlpGaussianPolicy
from auto_battlebot.rl.sidestep import Sidestep, SidestepConfig


def collect(
    env: WarpNavEnv,
    obs: torch.Tensor,
    student: MlpGaussianPolicy,
    teacher: MlpGaussianPolicy,
    expert: Sidestep,
    steps: int,
    beta: float,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    xs, ys, chased_steps = [], [], []
    for _ in range(steps):
        tr = env.expert_view()
        chased = (env.opponents.p.behaviour == PURSUIT) & ~env.run_away
        with torch.no_grad():
            label = teacher.act(obs, None, deterministic=True)[0]
            scripted = expert(tr)
            label = torch.where(chased[:, None], scripted, label)
            mine = student.act(obs, None, deterministic=True)[0]
        use_expert = torch.rand(env.num_worlds, device=obs.device) < beta
        action = torch.where(use_expert[:, None], label, mine)
        xs.append(obs.clone())
        ys.append(label)
        chased_steps.append(chased)
        obs = env.step(action).obs
    return torch.cat(xs), torch.cat(ys), torch.cat(chased_steps), obs


def fit(
    policy: MlpGaussianPolicy, x: torch.Tensor, y: torch.Tensor, epochs: int, batch: int
) -> float:
    opt = torch.optim.Adam(policy.actor.parameters(), lr=3e-4)
    loss_value = 0.0
    for _ in range(epochs):
        perm = torch.randperm(x.shape[0], device=x.device)
        for i in range(0, x.shape[0], batch):
            idx = perm[i : i + batch]
            mean = policy.actor(policy.norm(x[idx]))
            loss = (torch.tanh(mean) - y[idx]).pow(2).sum(1).mean()
            opt.zero_grad()
            loss.backward()
            opt.step()
            loss_value = float(loss.detach())
    return loss_value


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("checkpoint", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--worlds", type=int, default=4096)
    ap.add_argument("--rounds", type=int, default=8)
    ap.add_argument("--steps-per-round", type=int, default=250, help="policy steps, 20 ms each")
    ap.add_argument("--max-samples", type=int, default=8_000_000)
    ap.add_argument("--epochs", type=int, default=2)
    ap.add_argument("--batch", type=int, default=65536)
    ap.add_argument(
        "--expert",
        type=json.loads,
        default={},
        help="SidestepConfig fields as JSON, e.g. '{\"flank_m\": 0.4}' (pursuit_map.py picks them)",
    )
    ap.add_argument("--seed", type=int, default=11)
    args = ap.parse_args()

    env = WarpNavEnv(WarpNavConfig(worlds=args.worlds, stage="P"), "cuda", seed=args.seed)
    resume = torch.load(args.checkpoint, map_location="cuda")
    policy = MlpGaussianPolicy(env.spec)
    learner = Ppo(policy, env, PpoConfig(), seed=args.seed)
    learner.load_state_dict(resume["learner"])
    teacher = copy.deepcopy(policy).eval()
    expert = Sidestep(args.worlds, "cuda", SidestepConfig(**args.expert))
    obs, _ = env.reset(args.seed)
    env.desynchronize()
    data_x: list[torch.Tensor] = []
    data_y: list[torch.Tensor] = []
    log = []
    for r in range(args.rounds):
        beta = 1.0 - r / max(args.rounds - 1, 1)
        x, y, chased, obs = collect(env, obs, policy, teacher, expert, args.steps_per_round, beta)
        data_x.append(x)
        data_y.append(y)
        all_x, all_y = torch.cat(data_x), torch.cat(data_y)
        if all_x.shape[0] > args.max_samples:
            all_x, all_y = all_x[-args.max_samples :], all_y[-args.max_samples :]
            data_x, data_y = [all_x], [all_y]
        loss = fit(policy, all_x, all_y, args.epochs, args.batch)
        row = {
            "round": r,
            "beta": beta,
            "samples": int(all_x.shape[0]),
            "pursuit_share": float(chased.float().mean()),
            "loss": loss,
            "success": env.success_rate,
            "pursuit_success": env.pursuit_success_rate,
        }
        log.append(row)
        print(json.dumps(row), flush=True)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "learner": learner.state_dict(),
            "stage": "P",
            "env_steps": learner.env_steps,
            "pretrain": {"source": str(args.checkpoint), **vars(args), "log": log},
        },
        args.out,
    )
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
