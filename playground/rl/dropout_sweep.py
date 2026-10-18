"""Score policies against sensing dropouts: which robot drops out, and how long the gaps run.

Runs one deterministic 20 s episode per world in the Warp environment's EVAL stage for each
setting: dropouts off, then on for our robot, the opponent or both, with the sensing model's
`dropout_stretch` pinned (gaps x stretch, measured runs / stretch; training draws 0.7 to 2.0).
Each setting switches dropouts per robot by wrapping `MeasurementModel._advance_dropout`, so it
ignores whatever the sensing file's dropout `enabled` flag says.

    source scripts/activate_python.sh
    python playground/rl/dropout_sweep.py runs/rl_nav/<run>/checkpoint.pt ... \\
        --out playground/rl/out/dropout_sweep.json
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import torch

from auto_battlebot.rl.envs.opponents import PURSUIT
from auto_battlebot.rl.envs.warp_nav import WarpNavConfig, WarpNavEnv
from auto_battlebot.rl.policies.mlp import MlpGaussianPolicy

ROLES = {"off": (), "ours": (0,), "opponent": (1,), "both": (0, 1)}
KEYS = ("strikes", "hits_taken", "flips", "holes", "behaviour", "run_away")


def run(checkpoint: Path, roles: tuple[int, ...], stretch: float, worlds: int, seed: int) -> dict:
    env = WarpNavEnv(WarpNavConfig(worlds=worlds, stage="EVAL"), "cuda", seed=seed)
    measure = env.measure
    for sampler in measure._samplers:
        if hasattr(sampler, "dropout_enabled"):  # sensing files that can switch dropouts off
            sampler.dropout_enabled = True
    advance = measure._advance_dropout

    def advance_selected(r: int, *rest: Any) -> torch.Tensor:
        if r in roles:
            return advance(r, *rest)
        measure.is_measured[:, r] = True
        return measure.is_measured[:, r]

    measure._advance_dropout = advance_selected  # type: ignore[method-assign]
    net = MlpGaussianPolicy(env.spec).cuda()
    net.load_state_dict(torch.load(checkpoint, map_location="cuda")["learner"]["policy"])
    obs, _ = env.reset(seed)
    env.measure.params.dropout_stretch[:] = stretch
    finished = torch.zeros(worlds, dtype=torch.bool, device="cuda")
    seen = torch.zeros(worlds, 2, device="cuda")
    steps = torch.zeros(worlds, device="cuda")
    ep: dict[str, torch.Tensor] = {k: torch.zeros(worlds, device="cuda") for k in KEYS}
    for _ in range(env.max_steps + 1):
        live = (~finished).float()
        seen += live[:, None] * env.measure.is_measured.float()
        steps += live
        step = env.step(net.act(obs, None, deterministic=True)[0])
        first = step.done & ~finished
        if bool(first.any()):
            for k in KEYS:
                ep[k] = torch.where(first, step.episode[k], ep[k])
        finished |= step.done
        obs = step.obs
        if bool(finished.all()):
            break
    attack = ep["run_away"] < 0.5
    groups = {
        "pursuit": attack & (ep["behaviour"] == PURSUIT),
        "other": attack & (ep["behaviour"] != PURSUIT),
    }
    s, h = ep["strikes"], ep["hits_taken"]
    row: dict[str, Any] = {
        "ours_seen": float((seen[:, 0] / steps).mean()),
        "opponent_seen": float((seen[:, 1] / steps).mean()),
        "strikes_per_hit_taken": float(s[attack].sum() / h[attack].sum().clamp(min=1)),
        "run_away_hits_taken": float(h[~attack].mean()),
        "flips": float(ep["flips"].mean()),
        "holes": float(ep["holes"].mean()),
    }
    for name, m in groups.items():
        row[f"{name}_strikes"] = float(s[m].mean())
        row[f"{name}_hits_taken"] = float(h[m].mean())
        row[f"{name}_success"] = float((s[m] > 0).float().mean())
    return row


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("checkpoints", type=Path, nargs="+")
    ap.add_argument("--stretch", type=float, nargs="+", default=[1.0, 2.0, 4.0, 8.0])
    ap.add_argument("--worlds", type=int, default=4096)
    ap.add_argument("--seed", type=int, default=11)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()

    rows = []
    for checkpoint in args.checkpoints:
        settings = [("off", 1.0)] + [
            (role, s) for role in ("ours", "opponent", "both") for s in args.stretch
        ]
        for role, stretch in settings:
            row = {"checkpoint": str(checkpoint), "dropouts": role, "stretch": stretch}
            row.update(run(checkpoint, ROLES[role], stretch, args.worlds, args.seed))
            rows.append(row)
            shown = {k: round(v, 3) if isinstance(v, float) else v for k, v in row.items()}
            print(json.dumps(shown), flush=True)
            args.out.parent.mkdir(parents=True, exist_ok=True)
            args.out.write_text(json.dumps(rows, indent=1))


if __name__ == "__main__":
    main()
