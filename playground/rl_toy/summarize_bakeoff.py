"""Summarize the toy viability test runs (plan step 1, questions 1 to 4).

Reads every `runs/rl_toy/<arm>_s<seed>_<stamp>/metrics.jsonl` and reports, per arm, the median
and range over seeds (never the best run) of: the final evaluation, and the evaluation reached by
matched budgets of environment steps and wall-clock time, read off each run's periodic
evaluations (the last evaluation at or under the budget).

    source scripts/activate_python.sh
    python playground/rl_toy/summarize_bakeoff.py --steps 25e6 50e6 100e6 --minutes 5 10 15
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
KEYS = (
    "success_rate",
    "clean_success_rate",
    "hits_per_hit_taken",
    "median_time_to_first_strike_s",
    "strikes_per_episode",
    "hits_taken_per_episode",
)


def runs(root: Path) -> dict[str, list[list[dict[str, float]]]]:
    arms: dict[str, list[list[dict[str, float]]]] = {}
    for d in sorted(root.glob("*_s[0-9]*_*")):
        m = re.match(r"(.+)_s(\d+)_\d{4}-", d.name)
        log = d / "metrics.jsonl"
        if not m or not log.exists():
            continue
        evals = [r for r in map(json.loads, log.read_text().splitlines()) if r["kind"] == "eval"]
        if evals:
            arms.setdefault(m.group(1), []).append(evals)
    return arms


def at_budget(evals: list[dict[str, float]], key: str, budget: float) -> dict[str, float] | None:
    under = [e for e in evals if e[key] <= budget]
    return under[-1] if under else None


def stat(rows: list[dict[str, float] | None], key: str) -> str:
    vals = np.array([r[key] for r in rows if r is not None], dtype=float)
    if vals.size == 0:
        return "n/a"
    return f"{np.median(vals):.3g} [{vals.min():.3g}, {vals.max():.3g}] n={vals.size}"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--root", type=Path, default=REPO / "runs/rl_toy")
    ap.add_argument("--steps", type=float, nargs="*", default=[25e6, 50e6, 100e6])
    ap.add_argument("--minutes", type=float, nargs="*", default=[5, 10, 15])
    args = ap.parse_args()
    out: dict[str, dict[str, dict[str, str]]] = {}
    for arm, seeds in runs(args.root).items():
        rows = {"final": [s[-1] for s in seeds]}
        for b in args.steps:
            rows[f"steps<={b:.3g}"] = [at_budget(s, "env_steps", b) for s in seeds]
        for b in args.minutes:
            rows[f"minutes<={b:g}"] = [at_budget(s, "wall_s", b * 60) for s in seeds]
        out[arm] = {
            name: {k: stat(r, k) for k in (*KEYS, "env_steps", "wall_s")}
            for name, r in rows.items()
        }
        print(f"\n== {arm} ({len(seeds)} seeds)")
        for name, r in rows.items():
            print(
                f"  {name:16s} "
                + "  ".join(
                    f"{k}={stat(r, k)}"
                    for k in ("success_rate", "hits_per_hit_taken", "median_time_to_first_strike_s")
                )
            )
    dest = REPO / "playground/rl/out/bakeoff_summary.json"
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
