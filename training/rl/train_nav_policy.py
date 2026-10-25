"""Train the RL navigation policy on the Warp environment through the curriculum (plan step 6, 7).

Stages A to D advance when the rolling success rate (attack: at least one strike; run away: no
hit taken, no fall, no flip) over the last 2000 episodes passes `--advance-at` (70%). Output goes
to `runs/rl_nav/<name>_<timestamp>/`: `metrics.jsonl`, `checkpoint.pt` (rewritten every
`--checkpoint-every-s`), `stage_<X>.pt` at each stage transition, and `bundle/`, the exported
model bundle (plan section 12) written at the end.

    source scripts/activate_python.sh
    python training/rl/train_nav_policy.py --hours 3
    python training/rl/train_nav_policy.py --resume runs/rl_nav/<run>/checkpoint.pt --hours 2
    python training/rl/train_nav_policy.py --robot mrs_buff_mk3 --name rl_nav_mk3 --hours 6

`--robot` picks the robot (`warp_nav.ROBOTS`), Mr Stabs Mk2 by default. A checkpoint records its
robot, and resuming it as another robot is refused. A robot with a self-play opponent driver
(`policy_opponents.py`) plays a frozen copy of the policy, refreshed every `--self-play-every`
iterations.

`--pursuit-curriculum` starts the pursuers at `--pursuit-level` (0 is the easiest pursuer,
1 the whole range) and raises the level by 0.1 each time the last 500 or more attack episodes
against a pursuer since the last raise strike in at least `--pursuit-advance-at` of them. Use it
with stage P, which gives pursuit half the episodes. Only episodes whose pursuer was drawn at the
current level count. `--anchor` holds the policy near another checkpoint's actions in every
episode that is not against a pursuer (`Ppo` anchor), so the pursuit fine-tune keeps the rest.
"""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import torch

from auto_battlebot.rl.envs.opponents import PURSUIT
from auto_battlebot.rl.envs.warp_nav import ROBOTS, STAGES, WarpNavConfig, WarpNavEnv
from auto_battlebot.rl.export.bundle import spec_from_policy, write_bundle
from auto_battlebot.rl.learners.ppo import Ppo, PpoConfig
from auto_battlebot.rl.policies.mlp import MlpGaussianPolicy

REPO = Path(__file__).resolve().parents[2]
ORDER = ["A", "B", "C", "D"]
CHOICES = list(STAGES)


def restore(
    learner: Ppo, policy: MlpGaussianPolicy, resume: dict | None, reset_log_std: float | None
) -> None:
    if resume is None:
        return
    learner.load_state_dict(resume["learner"])
    if reset_log_std is not None:
        with torch.no_grad():
            policy.log_std.fill_(reset_log_std)


def configure_pursuit(env: WarpNavEnv, args: argparse.Namespace, resume: dict | None) -> None:
    if args.pursuit_max_speed is not None:
        hard = env.opponents.pursuit_hard
        env.opponents.pursuit_hard = (args.pursuit_max_speed, hard[1], hard[2])
    level = args.pursuit_level
    if level is None:
        level = resume.get("pursuit_level", 1.0) if resume is not None else 1.0
    if args.pursuit_curriculum and args.pursuit_level is None and resume is None:
        level = 0.0
    env.opponents.pursuit_level = level


def set_anchor(learner: Ppo, env: WarpNavEnv, path: Path) -> None:
    anchor = MlpGaussianPolicy(env.spec).cuda().eval()
    anchor.load_state_dict(torch.load(path, map_location="cuda")["learner"]["policy"])
    learner.anchor = anchor
    learner.anchor_mask = lambda: env.opponents.p.behaviour != PURSUIT


def raise_pursuit_level(env: WarpNavEnv, advance_at: float, env_steps: int) -> None:
    if env.opponents.pursuit_level >= 1.0 or len(env.pursuit_window) < 500:
        return
    if env.pursuit_success_rate < advance_at:
        return
    env.opponents.pursuit_level = min(1.0, round(env.opponents.pursuit_level + 0.1, 3))
    print(
        f"pursuit level -> {env.opponents.pursuit_level:.1f} at {env_steps:,} env steps "
        f"(success {env.pursuit_success_rate:.3f})",
        flush=True,
    )
    env.pursuit_window = []


def load_resume(args: argparse.Namespace) -> tuple[dict | None, str]:
    """The checkpoint to resume, if any, and the stage to start in."""
    stage = args.stage
    resume = torch.load(args.resume, map_location="cuda") if args.resume else None
    if resume is None:
        return None, stage
    if not args.force_stage:
        stage = resume.get("stage", stage)
    # Checkpoints from before the robot was recorded are all Mr Stabs Mk2.
    trained = resume.get("robot", "mr_stabs_mk2")
    if trained != args.robot:
        raise SystemExit(f"{args.resume} trained {trained}, not {args.robot}")
    return resume, stage


def reward_weights(overrides: list[str]) -> dict[str, float]:
    weights = WarpNavConfig().reward
    for item in overrides:
        key, sep, value = item.partition("=")
        if not sep or key not in weights:
            raise SystemExit(f"--reward {item}: expected KEY=VALUE with KEY one of {list(weights)}")
        weights[key] = float(value)
    return weights


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--name", default="rl_nav_ppo")
    ap.add_argument("--robot", default="mr_stabs_mk2", choices=list(ROBOTS))
    ap.add_argument("--fit-file", default="", help="MuJoCo plant fit; default: the robot's own")
    # 8,192 runs 91,000 policy steps/s on the laptop 4080 against 66,000 at 4,096.
    ap.add_argument("--worlds", type=int, default=8192)
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
    ap.add_argument(
        "--min-log-std",
        type=float,
        default=None,
        help="floor on the policy's action log-std, so exploration cannot collapse",
    )
    ap.add_argument("--pursuit-curriculum", action="store_true")
    ap.add_argument("--pursuit-level", type=float, default=None, help="start level, 0 to 1")
    ap.add_argument("--pursuit-advance-at", type=float, default=0.5)
    ap.add_argument(
        "--pursuit-max-speed",
        type=float,
        default=None,
        help="top speed of the hardest pursuer drawn (the beatable edge from pursuit_map.py)",
    )
    ap.add_argument(
        "--anchor",
        type=Path,
        default=None,
        help="checkpoint whose actions the policy is held near outside pursuit episodes, so "
        "pursuit training does not overwrite the other behaviours",
    )
    ap.add_argument("--anchor-coef", type=float, default=0.5)
    ap.add_argument("--checkpoint-every-s", type=float, default=600.0)
    ap.add_argument("--self-play-every", type=int, default=25, help="iterations per refresh")
    ap.add_argument("--out", type=Path, default=REPO / "runs" / "rl_nav")
    ap.add_argument(
        "--reward",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="override one reward weight (WarpNavConfig.reward), e.g. hole=-30; repeatable",
    )
    args = ap.parse_args()

    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    out = args.out / f"{args.name}_s{args.seed}_{stamp}"
    out.mkdir(parents=True, exist_ok=True)
    torch.manual_seed(args.seed)
    resume, stage = load_resume(args)
    env = WarpNavEnv(
        WarpNavConfig(
            worlds=args.worlds,
            stage=stage,
            episode_s=args.episode_s,
            robot=args.robot,
            fit_file=args.fit_file,
            reward=reward_weights(args.reward),
        ),
        "cuda",
        seed=args.seed,
    )
    policy = MlpGaussianPolicy(env.spec)
    if args.min_log_std is not None:
        policy.cfg.min_log_std = args.min_log_std
    configure_pursuit(env, args, resume)
    learner = Ppo(policy, env, PpoConfig(anchor_coef=args.anchor_coef), seed=args.seed)
    restore(learner, policy, resume, args.reset_log_std)
    if args.anchor is not None:
        set_anchor(learner, env, args.anchor)
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
            {
                "learner": learner.state_dict(),
                "robot": args.robot,
                "stage": env.stage,
                "env_steps": learner.env_steps,
                "pursuit_level": env.opponents.pursuit_level,
            },
            path,
        )

    while time.time() - start < args.hours * 3600:
        if it % args.self_play_every == 0:
            env.policy_opp.set_self_play(policy.state_dict())
        stats = learner.iterate()
        it += 1
        row = {
            "iteration": it,
            "wall_s": time.time() - start,
            "stage": env.stage,
            "success": env.success_rate,
            "pursuit_success": env.pursuit_success_rate,
            "pursuit_level": env.opponents.pursuit_level,
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
                f"pursuit={env.pursuit_success_rate:.3f}@{env.opponents.pursuit_level:.1f} "
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
        if args.pursuit_curriculum:
            raise_pursuit_level(env, args.pursuit_advance_at, learner.env_steps)
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
