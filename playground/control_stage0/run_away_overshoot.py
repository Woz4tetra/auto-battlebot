#!/usr/bin/env python3
"""Diagnose RUN_AWAY overshoot from a recorded fight.

Puts the two halves of the question on one time axis: did the goal move, or was the
brake late? The five leads in docs/hazard_avoidance_plan.md map to columns here:

  1. goal churn            -> goal_changes, goal_hz, median_epoch_s, median_goal_jump_m
  2. filtered speed error  -> v_ref vs v_actual traces, speed_err_p90
  3. open-loop stretches   -> open_loop_pct (speed_is_measured == 0)
  4. heading-gate re-entry -> gate_off_pct, gate_reentries
  5. solver plateau jitter -> solver source histogram, goal jump sizes

Overshoot itself is measured per goal epoch (a stretch with one held goal): the robot
approaches to some minimum distance and then recedes. The recession is the overshoot.

Usage:
    source scripts/activate_python.sh
    python playground/control_stage0/run_away_overshoot.py data/recordings/<file>.mcap
    python playground/control_stage0/run_away_overshoot.py data/recordings/*.mcap --out out_dir
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from mcap.reader import make_reader  # noqa: E402

from auto_battlebot.mcap_io import decode_diagnostic_array  # noqa: E402

DIAGNOSTICS_TOPIC = "/diagnostics"
NAV_HW_ID = "motion_profile_nav"
SELECTOR_HW_ID = "safest_point_target"
RUNNER_HW_ID = "runner"
TRANSMITTER_HW_ID = "opentx_transmitter"

# Goal moves smaller than this are solver noise, not a retarget.
GOAL_EPS_M = 1e-4
# An epoch only counts as an approach-and-recede if the robot got at least this close.
APPROACH_M = 0.35
# ... and then receded by at least this much.
RECEDE_M = 0.05

_NUMERIC = [
    "distance",
    "v_ref",
    "v_actual",
    "speed_is_measured",
    "w_ref",
    "angle_error_deg",
    "facing_target",
    "terminal_velocity",
    "our_x",
    "our_y",
    "our_yaw_deg",
    "target_x",
    "target_y",
    "linear_x",
    "angular_z",
    "goal_x",
    "goal_y",
    "goal_radius",
    "solver_x",
    "solver_y",
    "solver_radius",
    "solver_evals",
    "n_opponents",
    "trainer_enabled",
]


def extract(path: Path) -> pd.DataFrame:
    """One row per diagnostics message, merging the nav, selector, and transmitter views."""
    rows: dict[int, dict[str, Any]] = defaultdict(dict)
    with open(path, "rb") as f:
        reader = make_reader(f)
        for _schema, channel, message in reader.iter_messages(topics=[DIAGNOSTICS_TOPIC]):
            ts = message.log_time
            for status in decode_diagnostic_array(message.data):
                hw, name, kv = status["hardware_id"], status["name"], status["values"]
                if hw == NAV_HW_ID:
                    if name in ("profile", "poses", "command", "stop", "wall_reverse"):
                        rows[ts].update(kv)
                    if name == "wall_reverse":
                        rows[ts]["wall_reverse"] = 1
                elif hw == SELECTOR_HW_ID:
                    if name == "run_away_target":
                        for k in ("x", "y", "radius"):
                            if k in kv:
                                rows[ts][f"goal_{k}"] = kv[k]
                    elif name == "solver":
                        for k in ("x", "y", "radius"):
                            if k in kv:
                                rows[ts][f"solver_{k}"] = kv[k]
                        if "source" in kv:
                            rows[ts]["solver_source"] = kv["source"]
                        if "evaluations" in kv:
                            rows[ts]["solver_evals"] = kv["evaluations"]
                        if "n_opponents" in kv:
                            rows[ts]["n_opponents"] = kv["n_opponents"]
                elif hw == RUNNER_HW_ID and name == "navigation":
                    if "behavior_mode" in kv:
                        rows[ts]["behavior_mode"] = kv["behavior_mode"]
                elif hw == TRANSMITTER_HW_ID and name == "switch_states":
                    if "trainer_enabled" in kv:
                        rows[ts]["trainer_enabled"] = kv["trainer_enabled"]

    if not rows:
        raise SystemExit(f"no /diagnostics in {path}")

    df = pd.DataFrame([{**rows[ts], "timestamp_ns": ts} for ts in sorted(rows)])
    for col in _NUMERIC:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    for col in ("behavior_mode", "solver_source", "trainer_enabled"):
        if col in df.columns:
            df[col] = df[col].ffill()
    df["t"] = (df["timestamp_ns"] - df["timestamp_ns"].iloc[0]) / 1e9
    return df


def run_away_segments(df: pd.DataFrame, min_ticks: int = 15) -> list[pd.DataFrame]:
    """Contiguous stretches where the resolved behavior mode is RUN_AWAY."""
    if "behavior_mode" not in df.columns:
        return []
    mask = df["behavior_mode"].astype(str) == "RUN_AWAY"
    if "trainer_enabled" in df.columns:
        trainer = df["trainer_enabled"].fillna(1)
        mask &= trainer > 0.5
    if not mask.any():
        return []
    group = (mask != mask.shift()).cumsum()
    return [
        seg.reset_index(drop=True)
        for _, seg in df[mask].groupby(group[mask])
        if len(seg) >= min_ticks
    ]


def goal_epochs(seg: pd.DataFrame) -> list[tuple[int, int]]:
    """[start, end) index pairs over which the held goal did not move."""
    if not {"goal_x", "goal_y"}.issubset(seg.columns):
        return []
    gx, gy = seg["goal_x"].ffill(), seg["goal_y"].ffill()
    moved = (gx.diff().abs() > GOAL_EPS_M) | (gy.diff().abs() > GOAL_EPS_M)
    moved.iloc[0] = True
    starts = list(np.flatnonzero(moved.to_numpy()))
    return list(zip(starts, starts[1:] + [len(seg)]))


def _finite(series: pd.Series) -> np.ndarray:
    values = series.to_numpy(dtype=float)
    return values[np.isfinite(values)]


def score_segment(seg: pd.DataFrame, name: str) -> dict[str, Any]:
    dt = float(seg["t"].diff().median())
    duration = float(seg["t"].iloc[-1] - seg["t"].iloc[0])
    out: dict[str, Any] = {"segment": name, "ticks": len(seg), "duration_s": round(duration, 2)}

    epochs = goal_epochs(seg)
    out["goal_changes"] = max(0, len(epochs) - 1)
    out["goal_hz"] = round(out["goal_changes"] / duration, 2) if duration > 0 else None
    if epochs:
        lengths = [(e - s) * dt for s, e in epochs]
        out["median_epoch_s"] = round(float(np.median(lengths)), 3)
        out["max_epoch_s"] = round(float(np.max(lengths)), 2)
        gx, gy = seg["goal_x"].ffill().to_numpy(), seg["goal_y"].ffill().to_numpy()
        jumps = [
            float(np.hypot(gx[s] - gx[s - 1], gy[s] - gy[s - 1])) for s, _ in epochs[1:] if s > 0
        ]
        if jumps:
            out["median_goal_jump_m"] = round(float(np.median(jumps)), 3)
            out["p90_goal_jump_m"] = round(float(np.percentile(jumps, 90)), 3)

    if "speed_is_measured" in seg.columns:
        measured = seg["speed_is_measured"].fillna(0)
        out["open_loop_pct"] = round(100.0 * float((measured < 0.5).mean()), 1)
        gaps = (measured < 0.5).astype(int)
        edges = int(((gaps.diff() == 1)).sum())
        out["open_loop_runs"] = edges
    if "facing_target" in seg.columns:
        facing = seg["facing_target"].fillna(0)
        out["gate_off_pct"] = round(100.0 * float((facing < 0.5).mean()), 1)
        out["gate_reentries"] = int(((facing.diff() == 1)).sum())
    if {"v_ref", "v_actual"}.issubset(seg.columns):
        err = _finite(seg["v_ref"] - seg["v_actual"])
        if err.size:
            out["speed_err_p90"] = round(float(np.percentile(np.abs(err), 90)), 3)
            out["speed_err_med"] = round(float(np.median(err)), 3)
    if "wall_reverse" in seg.columns:
        out["wall_reverse_pct"] = round(100.0 * float(seg["wall_reverse"].fillna(0).mean()), 1)
    if "solver_source" in seg.columns:
        counts = seg["solver_source"].value_counts()
        out["solver_top"] = f"{counts.index[0]}:{int(counts.iloc[0])}" if len(counts) else ""
        out["solver_sources"] = int(seg["solver_source"].nunique())

    # Overshoot: per goal epoch, closest approach then recession while that goal was held.
    events = []
    if {"our_x", "our_y"}.issubset(seg.columns):
        ox, oy = seg["our_x"].ffill().to_numpy(), seg["our_y"].ffill().to_numpy()
        gx, gy = seg["goal_x"].ffill().to_numpy(), seg["goal_y"].ffill().to_numpy()
        for s, e in epochs:
            if e - s < 3:
                continue
            d = np.hypot(ox[s:e] - gx[s], oy[s:e] - gy[s])
            d = d[np.isfinite(d)]
            if d.size < 3:
                continue
            i = int(np.argmin(d))
            recede = float(d[i:].max() - d[i])
            if d[i] <= APPROACH_M and recede >= RECEDE_M:
                events.append((float(d[i]), recede, (e - s) * dt))
    out["overshoot_events"] = len(events)
    if events:
        out["max_overshoot_m"] = round(max(e[1] for e in events), 3)
        out["median_overshoot_m"] = round(float(np.median([e[1] for e in events])), 3)
        out["min_approach_m"] = round(min(e[0] for e in events), 3)
    return out


def plot_segment(seg: pd.DataFrame, title: str, out_path: Path) -> None:
    t = seg["t"].to_numpy() - seg["t"].iloc[0]
    epochs = goal_epochs(seg)
    fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True)
    fig.suptitle(title, fontsize=12)

    ax = axes[0]
    if {"our_x", "our_y", "goal_x", "goal_y"}.issubset(seg.columns):
        ox, oy = seg["our_x"].ffill().to_numpy(), seg["our_y"].ffill().to_numpy()
        gx, gy = seg["goal_x"].ffill().to_numpy(), seg["goal_y"].ffill().to_numpy()
        held = np.full(len(seg), np.nan)
        for s, e in epochs:
            held[s:e] = np.hypot(ox[s:e] - gx[s], oy[s:e] - gy[s])
        ax.plot(t, held, lw=1.0, color="tab:blue", label="distance to held goal")
    if "distance" in seg.columns:
        ax.plot(t, seg["distance"], lw=0.8, color="tab:gray", alpha=0.7, label="nav distance")
    for s, _ in epochs[1:]:
        ax.axvline(t[s], color="crimson", lw=0.6, alpha=0.5)
    ax.set_ylabel("m")
    ax.legend(fontsize=8, loc="upper right")
    ax.set_title("distance to goal; red lines = goal changes")

    ax = axes[1]
    for col, color in (("v_ref", "tab:green"), ("v_actual", "tab:orange")):
        if col in seg.columns:
            ax.plot(t, seg[col], lw=0.9, color=color, label=col)
    if "terminal_velocity" in seg.columns:
        ax.plot(t, seg["terminal_velocity"], lw=0.8, ls=":", color="k", label="terminal")
    ax.set_ylabel("m/s")
    ax.legend(fontsize=8, loc="upper right")
    ax.set_title("brake schedule vs filtered speed")

    ax = axes[2]
    if "speed_is_measured" in seg.columns:
        open_loop = seg["speed_is_measured"].fillna(0) < 0.5
        ax.fill_between(t, 0, 1, where=open_loop, color="tab:red", alpha=0.3, label="open loop")
    if "facing_target" in seg.columns:
        gate_off = seg["facing_target"].fillna(0) < 0.5
        ax.fill_between(t, 1, 2, where=gate_off, color="tab:purple", alpha=0.3, label="gate off")
    ax.set_ylim(0, 2)
    ax.set_yticks([])
    ax.legend(fontsize=8, loc="upper right")
    ax.set_title("feedback availability")

    ax = axes[3]
    for col, color in (("linear_x", "tab:blue"), ("angular_z", "tab:red")):
        if col in seg.columns:
            ax.plot(t, seg[col], lw=0.9, color=color, label=col)
    ax.set_ylabel("command")
    ax.set_xlabel("time in segment (s)")
    ax.legend(fontsize=8, loc="upper right")
    ax.set_title("transmitted command")

    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recordings", type=Path, nargs="+")
    ap.add_argument("--out", type=Path, default=Path("playground/control_stage0/run_away_out"))
    ap.add_argument("--min-ticks", type=int, default=15)
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    rows = []
    for path in args.recordings:
        df = extract(path)
        segments = run_away_segments(df, args.min_ticks)
        if not segments:
            print(f"{path.name}: no RUN_AWAY segments")
            continue
        for i, seg in enumerate(segments):
            name = f"{path.stem}#{i}"
            row = score_segment(seg, name)
            row["recording"] = path.name
            rows.append(row)
            plot_segment(seg, name, args.out / f"{path.stem}_seg{i}.png")
        print(f"{path.name}: {len(segments)} RUN_AWAY segment(s)")

    if not rows:
        raise SystemExit("no RUN_AWAY segments in any recording")
    table = pd.DataFrame(rows)
    csv = args.out / "run_away_segments.csv"
    table.to_csv(csv, index=False)
    with pd.option_context("display.width", 220, "display.max_columns", 50):
        print(table.to_string(index=False))
    print(f"\nwrote {csv}")


if __name__ == "__main__":
    main()
