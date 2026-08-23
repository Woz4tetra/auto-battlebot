"""Print the scored columns from one or more sim_sweep output directories.

Used to compare a controller change against the previous step: run the sweeps into a per-step
directory, then point this at it. Columns are the ones each scenario is judged on, per
docs/experiments/control_improvement/stage4_plant_backed_nav_plan.md.

    python playground/control_stage0/summarize_sweeps.py playground/control_stage0/sweep_out --tag step1
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd

# Scenario -> the columns worth reading for it. Anything absent is skipped rather than erroring,
# so a sweep that scores a subset still prints.
SCENARIO_COLUMNS: dict[str, list[str]] = {
    "stop": ["terminal_pos_err_m", "terminal_vel_mps", "overshoot_m", "t_to_goal_s", "wall_contacts"],
    "ram": ["navigation.attack_terminal_velocity", "impact_speed_mps", "min_dist_m", "t_contact_s"],
    "track": ["track_err_mean_m", "track_err_rms_m", "wall_contacts"],
    "turn": ["terminal_pos_err_m", "overshoot_m", "t_to_goal_s", "mean_abs_ang_deg", "facing_pct"],
    "dropout": ["terminal_pos_err_m", "terminal_vel_mps", "overshoot_m", "wall_contacts"],
}


def scenario_of(name: str) -> str:
    for key in SCENARIO_COLUMNS:
        if key in name:
            return key
    return "stop"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("root", type=Path, help="directory holding one subdirectory per sweep")
    ap.add_argument("--tag", default="", help="label printed in the header")
    args = ap.parse_args()

    pd.set_option("display.width", 250)
    results = sorted(args.root.glob("*/results.csv"))
    if not results:
        raise SystemExit(f"no results.csv under {args.root}")

    for csv in results:
        df = pd.read_csv(csv)
        cols = ["name"] + [c for c in SCENARIO_COLUMNS[scenario_of(csv.parent.name)] if c in df.columns]
        header = f"{csv.parent.name}"
        if args.tag:
            header += f"  [{args.tag}]"
        print(f"===== {header} =====")
        if not df.get("ok", pd.Series([True])).all():
            print(f"!! {(~df['ok']).sum()} of {len(df)} runs failed")
        print(df[cols].to_string(index=False))
        print()


if __name__ == "__main__":
    main()
