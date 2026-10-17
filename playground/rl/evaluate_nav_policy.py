"""Score navigation policies on the held-out evaluation scenarios (plan gates G4 and G5).

`warp` scores a training checkpoint in the Warp environment on the `EVAL` stage with
`holdout=True`: stage D's opponent mix with the held-out pursuit corner and replay tracks, on the
nominal plant. With the same `--count` and `--seed` as `lockstep_eval.py export`, each world's
first episode is the exported scenario of the same index.

`report` puts any mix of Warp results and lockstep result files side by side, overall and per
opponent behaviour, with the G4 agreement (Warp against lockstep for the same policy) and the
G5 comparison (the policy against `MotionProfileNavigation`).

    source scripts/activate_python.sh
    python playground/rl/evaluate_nav_policy.py warp runs/rl_nav/<run>/checkpoint.pt \\
        --count 128 --seed 2026 --out playground/rl/out/warp_rl.json
    python playground/rl/evaluate_nav_policy.py report playground/rl/out/warp_rl.json \\
        playground/rl/out/lockstep_rl.jsonl playground/rl/out/lockstep_motion_profile.jsonl
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import torch

from auto_battlebot.rl.envs.opponents import BEHAVIOURS
from auto_battlebot.rl.metrics import summarize

REPO = Path(__file__).resolve().parents[2]
EPISODE_S = 20.0  # for result files written before rows recorded their episode_s


def warp(checkpoint: Path, count: int, seed: int, out: Path, episode_s: float) -> None:
    from auto_battlebot.rl.envs.warp_nav import WarpNavConfig, WarpNavEnv
    from auto_battlebot.rl.policies.mlp import MlpGaussianPolicy

    cfg = WarpNavConfig(worlds=count, stage="EVAL", holdout=True, episode_s=episode_s)
    env = WarpNavEnv(cfg, "cuda", seed=seed)
    policy = MlpGaussianPolicy(env.spec).cuda()
    state = torch.load(checkpoint, map_location="cuda")
    policy.load_state_dict(state["learner"]["policy"])
    obs, _ = env.reset(seed)
    finished = torch.zeros(count, dtype=torch.bool, device="cuda")
    rows: dict[str, torch.Tensor] = {}
    for _ in range(env.max_steps + 1):
        action, _ = policy.act(obs, None, deterministic=True)
        step = env.step(action)
        first = step.done & ~finished
        if bool(first.any()):
            for key, value in step.episode.items():
                rows.setdefault(key, torch.zeros_like(value))
                rows[key] = torch.where(first, value, rows[key])
        finished |= step.done
        obs = step.obs
        if bool(finished.all()):
            break
    episodes = [
        {k: float(v[i]) for k, v in rows.items()}
        | {"index": i, "label": "warp_rl", "episode_s": episode_s}
        for i in range(count)
    ]
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(episodes))
    print(f"wrote {count} episodes to {out}")


def load(path: Path) -> list[dict[str, Any]]:
    if path.suffix == ".jsonl":
        rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    else:
        rows = json.loads(path.read_text())
    for r in rows:
        for k in ("first_strike_s", "first_hit_s"):
            if r.get(k) is None:
                r[k] = math.inf
    return sorted(rows, key=lambda r: int(r["index"]))


def table(rows: list[dict[str, Any]]) -> dict[str, float]:
    if not rows:
        return {}
    keys = (
        "strikes",
        "rear_strikes",
        "hits_taken",
        "first_strike_s",
        "first_hit_s",
        "danger_s",
        "wall_events",
        "holes",
        "flips",
        "block_contacts",
        "still_s",
    )
    episode = {k: torch.tensor([float(r.get(k, 0.0)) for r in rows]) for k in keys}
    episode_s = max(float(r.get("episode_s", EPISODE_S)) for r in rows)
    result = summarize(episode, episode_s).as_dict()
    # A flip or a hole ends the episode early, so danger seconds per episode reward dying fast.
    # The share of time alive spent in danger compares policies that survive for different times.
    alive = sum(float(r.get("seconds", episode_s)) for r in rows)
    result["danger_share"] = sum(float(r["danger_s"]) for r in rows) / max(alive, 1e-9)
    result["seconds_per_episode"] = alive / len(rows)
    return result


def report(paths: list[Path]) -> None:
    sets = {p.stem: load(p) for p in paths}
    common = set.intersection(*(set(int(r["index"]) for r in rows) for rows in sets.values()))
    print(f"{len(common)} scenarios scored by every source")
    out: dict[str, Any] = {}
    for name, rows in sets.items():
        rows = [r for r in rows if int(r["index"]) in common]
        attack = [r for r in rows if r.get("run_away", 0.0) < 0.5]
        run = [r for r in rows if r.get("run_away", 0.0) >= 0.5]
        out[name] = {
            "attack": table(attack),
            "run_away": table(run),
            "by_behaviour": {
                BEHAVIOURS[b]: table([r for r in attack if int(r.get("behaviour", -1)) == b])
                for b in range(len(BEHAVIOURS))
            },
        }
    cols = (
        "hits_per_hit_taken",
        "strikes_per_episode",
        "hits_taken_per_episode",
        "success_rate",
        "danger_s_per_episode",
        "danger_share",
        "seconds_per_episode",
        "holes_per_episode",
        "flips_per_episode",
        "still_s_per_episode",
    )
    print(f"{'source':34s} " + " ".join(f"{c[:14]:>14s}" for c in cols))
    for name, res in out.items():
        a = res["attack"]
        print(f"{name:34s} " + " ".join(f"{a.get(c, float('nan')):14.3f}" for c in cols))
    print("\nper behaviour, attack episodes: hits per hit taken (strikes, hits taken per episode)")
    for b in BEHAVIOURS:
        cells = []
        for name, res in out.items():
            t = res["by_behaviour"][b]
            if t:
                cells.append(
                    f"{name}: {t['hits_per_hit_taken']:.2f} ({t['strikes_per_episode']:.2f}, "
                    f"{t['hits_taken_per_episode']:.2f}) n={int(t['episodes'])}"
                )
        print(f"  {b:12s} " + " | ".join(cells))
    print("\nrun-away episodes: hits taken, holes, flips per episode")
    for name, res in out.items():
        r = res["run_away"]
        if r:
            print(
                f"  {name:32s} {r['hits_taken_per_episode']:.3f} {r['holes_per_episode']:.3f} "
                f"{r['flips_per_episode']:.3f} n={int(r['episodes'])}"
            )
    Path(paths[0]).with_name("report.json").write_text(json.dumps(out, indent=2))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    w = sub.add_parser("warp")
    w.add_argument("checkpoint", type=Path)
    w.add_argument("--count", type=int, default=128)
    w.add_argument("--seed", type=int, default=2026)
    w.add_argument("--episode-s", type=float, default=20.0, help="match the scenarios' length")
    w.add_argument("--out", type=Path, default=REPO / "playground/rl/out/warp_rl.json")
    r = sub.add_parser("report")
    r.add_argument("paths", type=Path, nargs="+")
    args = ap.parse_args()
    if args.cmd == "warp":
        warp(args.checkpoint, args.count, args.seed, args.out, args.episode_s)
    else:
        report(args.paths)


if __name__ == "__main__":
    main()
