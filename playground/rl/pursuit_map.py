"""Map which pursuers can be beaten: a grid of pursuit top speed, turn rate and reaction delay.

Every world runs one 20 s attack episode against a pursuer pinned to one grid cell, in the Warp
environment's `PURSUIT` stage (pursuit only, open arena, nominal plant, sensing on). `sidestep`
sweeps the scripted controller's settings (`auto_battlebot/rl/sidestep.py`) and keeps the best
per cell; `policy` scores a training checkpoint, deterministic. `show` prints saved maps side by
side. A cell counts as beaten when the episodes there land at least as many strikes as hits taken
and strike in at least half of them.

    source scripts/activate_python.sh
    python playground/rl/pursuit_map.py sidestep --out playground/rl/out/pursuit_map_sidestep.json
    python playground/rl/pursuit_map.py policy runs/rl_nav/<run>/checkpoint.pt \\
        --out playground/rl/out/pursuit_map_policy.json
    python playground/rl/pursuit_map.py show playground/rl/out/pursuit_map_*.json
"""

from __future__ import annotations

import argparse
import itertools
import json
from collections.abc import Callable
from pathlib import Path
from typing import Any

import torch

from auto_battlebot.rl.envs.opponents import PURSUIT_GRID as CELLS

REPO = Path(__file__).resolve().parents[2]
SPEEDS = sorted({c[0] for c in CELLS})
TURNS = sorted({c[1] for c in CELLS})
DELAYS = sorted({c[2] for c in CELLS})


def run(
    act: Callable[[Any, torch.Tensor], torch.Tensor], per_cell: int, seed: int
) -> list[dict[str, float]]:
    """One episode per world; returns per-cell totals."""
    from auto_battlebot.rl.envs.warp_nav import WarpNavConfig, WarpNavEnv

    worlds = len(CELLS) * per_cell
    env = WarpNavEnv(WarpNavConfig(worlds=worlds, stage="PURSUIT"), "cuda", seed=seed)
    grid = torch.tensor(CELLS, device="cuda", dtype=torch.float32)
    env.opponents.pursuit_fixed = grid.repeat_interleave(per_cell, 0)
    obs, _ = env.reset(seed)
    finished = torch.zeros(worlds, dtype=torch.bool, device="cuda")
    strikes = torch.zeros(worlds, device="cuda")
    hits = torch.zeros(worlds, device="cuda")
    flips = torch.zeros(worlds, device="cuda")
    for _ in range(env.max_steps + 1):
        step = env.step(act(env, obs))
        first = step.done & ~finished
        if bool(first.any()):
            strikes = torch.where(first, step.episode["strikes"], strikes)
            hits = torch.where(first, step.episode["hits_taken"], hits)
            flips = torch.where(first, step.episode["flips"], flips)
        finished |= step.done
        obs = step.obs
        if bool(finished.all()):
            break
    rows = []
    for i, (speed, turn, delay) in enumerate(CELLS):
        cell = slice(i * per_cell, (i + 1) * per_cell)
        s, h = strikes[cell], hits[cell]
        rows.append(
            {
                "speed": speed,
                "turn": turn,
                "delay": delay,
                "strikes": float(s.mean()),
                "hits_taken": float(h.mean()),
                "success": float((s > 0).float().mean()),
                "flips": float(flips[cell].mean()),
            }
        )
    return rows


def beaten(row: dict[str, float]) -> bool:
    return row["strikes"] >= row["hits_taken"] and row["success"] >= 0.5


def score(row: dict[str, float]) -> float:
    return row["strikes"] - row["hits_taken"]


def rank(row: dict[str, float]) -> tuple[bool, float, float]:
    """Best variant per cell: beaten first, then strike rate, then strikes less hits taken. On
    the net score alone a variant that never strikes but is rarely hit wins."""
    return beaten(row), row["success"], score(row)


# The controller settings swept per cell; the best per cell is kept.
SWEEP = {"orbit_m": (0.3, 0.4), "max_lateral_mps2": (9.0, 12.0), "attack_deg": (75.0, 100.0)}


def sidestep(per_cell: int, seed: int) -> dict[str, Any]:
    from auto_battlebot.rl.sidestep import Sidestep, SidestepConfig

    best: list[dict[str, Any]] = []
    for values in itertools.product(*SWEEP.values()):
        variant = dict(zip(SWEEP, values, strict=True))
        cfg = SidestepConfig(**variant)
        ctl: Sidestep | None = None

        def act(env: Any, obs: torch.Tensor, cfg: SidestepConfig = cfg) -> torch.Tensor:
            nonlocal ctl
            if ctl is None:
                ctl = Sidestep(env.num_worlds, "cuda", cfg)
            return ctl(env.expert_view())

        rows = run(act, per_cell, seed)
        won = sum(beaten(r) for r in rows)
        print(f"{variant}: {won} of {len(rows)} cells beaten", flush=True)
        for i, r in enumerate(rows):
            r["variant"] = variant
            if len(best) <= i:
                best.append(r)
            elif rank(r) > rank(best[i]):
                best[i] = r
    return {"source": "sidestep", "cells": best}


def policy(checkpoint: Path, per_cell: int, seed: int) -> dict[str, Any]:
    from auto_battlebot.rl.policies.mlp import MlpGaussianPolicy

    net: MlpGaussianPolicy | None = None

    def act(env: Any, obs: torch.Tensor) -> torch.Tensor:
        nonlocal net
        if net is None:
            net = MlpGaussianPolicy(env.spec).cuda()
            state = torch.load(checkpoint, map_location="cuda")
            net.load_state_dict(state["learner"]["policy"])
        action, _ = net.act(obs, None, deterministic=True)
        return action

    return {"source": str(checkpoint), "cells": run(act, per_cell, seed)}


def show(paths: list[Path]) -> None:
    maps = {p.stem: json.loads(p.read_text()) for p in paths}
    for name, m in maps.items():
        cells = m["cells"]
        won = sum(beaten(r) for r in cells)
        print(f"\n{name}: {won} of {len(cells)} cells beaten ({m['source']})")
        print("strikes - hits taken per episode; * = beaten. rows: top speed, columns: turn rate")
        for delay in DELAYS:
            print(f"  delay {delay * 1000:.0f} ms " + "".join(f"{t:>9.0f}" for t in TURNS))
            for speed in SPEEDS:
                line = []
                for turn in TURNS:
                    r = next(
                        c
                        for c in cells
                        if (c["speed"], c["turn"], c["delay"]) == (speed, turn, delay)
                    )
                    line.append(f"{score(r):8.1f}{'*' if beaten(r) else ' '}")
                print(f"    {speed:4.1f} m/s   " + "".join(line))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("sidestep", "policy"):
        p = sub.add_parser(name)
        if name == "policy":
            p.add_argument("checkpoint", type=Path)
        p.add_argument("--per-cell", type=int, default=16)
        p.add_argument("--seed", type=int, default=7)
        p.add_argument("--out", type=Path, required=True)
    s = sub.add_parser("show")
    s.add_argument("paths", type=Path, nargs="+")
    args = ap.parse_args()
    if args.cmd == "show":
        show(args.paths)
        return
    if args.cmd == "sidestep":
        result = sidestep(args.per_cell, args.seed)
    else:
        result = policy(args.checkpoint, args.per_cell, args.seed)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=1))
    show([args.out])


if __name__ == "__main__":
    main()
