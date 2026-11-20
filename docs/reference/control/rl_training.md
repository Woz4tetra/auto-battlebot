# RL navigation training

`training/rl/train_nav_policy.py` trains the RL navigation policy on the Warp environment through the curriculum.

## Curriculum

- Stages A to D advance when the rolling success rate over the last 2000 episodes passes `--advance-at` (default 0.7).
- Success for attack is at least one strike. For run away it is no hit taken, no fall, no flip.

## Output

Output goes to `runs/rl_nav/<name>_<timestamp>/`:

- `metrics.jsonl`
- `checkpoint.pt`, rewritten every `--checkpoint-every-s`
- `stage_<X>.pt` at each stage transition
- `bundle/`, the exported model bundle written at the end (`docs/reference/control/rl_policy_bundle.md`)

## Commands

```bash
source scripts/activate_python.sh
python training/rl/train_nav_policy.py --hours 3
python training/rl/train_nav_policy.py --resume runs/rl_nav/<run>/checkpoint.pt --hours 2
python training/rl/train_nav_policy.py --robot mrs_buff_mk3 --name rl_nav_mk3 --hours 6
```

## Robot and self-play

- `--robot` picks the robot (`warp_nav.ROBOTS`). The default is Mr Stabs Mk2.
- A checkpoint records its robot. Resuming it as another robot is refused.
- A robot with a self-play opponent driver (`policy_opponents.py`) plays a frozen copy of the policy, refreshed every `--self-play-every` iterations.

## Pursuit curriculum

- `--pursuit-curriculum` starts the pursuers at `--pursuit-level` (0 is the easiest pursuer, 1 the whole range).
- The level rises by 0.1 each time the last 500 or more attack episodes against a pursuer since the last raise strike in at least `--pursuit-advance-at` of them. Only episodes whose pursuer was drawn at the current level count.
- Use it with stage P, which gives pursuit half the episodes.
- `--anchor` holds the policy near another checkpoint's actions in every episode that is not against a pursuer (the `Ppo` anchor), so the pursuit fine-tune keeps the rest.
