#!/usr/bin/env python3
"""Stage 1 of the run-away solver experiment: MCAP recordings -> per-tick trace CSVs.

Reads each Jetson fight recording and writes traces/<recording>.csv with one row
per /robot_markers tick: timestamp, field size, our x/y, opponent x/y per track,
opponent count. Everything downstream (compare_solvers.py) reads only these CSVs,
so a bad trace is caught here by inspection instead of silently polluting the
comparison.

Usage:
    python extract_traces.py data/recordings/auto_battlebot_main_2026-05-02_*_repaired*.mcap
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

from auto_battlebot.diag_io import load_field_size, load_robot_positions

MAX_OPPONENTS = 3


def build_trace(recording: Path) -> pd.DataFrame | None:
    field = load_field_size(recording)
    if field is None:
        print(f"  no field markers, skipping: {recording.name}")
        return None
    field_w, field_h = field

    positions = load_robot_positions(recording)
    if positions.empty:
        print(f"  no robot markers, skipping: {recording.name}")
        return None

    records = []
    for ts, tick in positions.groupby("timestamp_ns", sort=True):
        ours = tick[tick["group"] == "OURS"]
        # Deterministic opponent order within a tick: by frame id name.
        theirs = tick[tick["group"] == "THEIRS"].sort_values("frame")
        row: dict[str, float | int] = {
            "timestamp_ns": int(ts),
            "field_w": field_w,
            "field_h": field_h,
            "our_x": float(ours.iloc[0]["x"]) if len(ours) else np.nan,
            "our_y": float(ours.iloc[0]["y"]) if len(ours) else np.nan,
            "n_opp": min(len(theirs), MAX_OPPONENTS),
        }
        for i in range(MAX_OPPONENTS):
            row[f"opp{i}_x"] = float(theirs.iloc[i]["x"]) if i < len(theirs) else np.nan
            row[f"opp{i}_y"] = float(theirs.iloc[i]["y"]) if i < len(theirs) else np.nan
        records.append(row)
    return pd.DataFrame(records)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recordings", nargs="+", type=Path)
    parser.add_argument("--out-dir", type=Path, default=Path(__file__).resolve().parent / "traces")
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    for recording in args.recordings:
        print(f"extracting {recording.name}")
        trace = build_trace(recording)
        if trace is None:
            continue
        out_path = args.out_dir / f"{recording.stem}.csv"
        trace.to_csv(out_path, index=False)
        n_by_count = trace["n_opp"].value_counts().sort_index().to_dict()
        print(
            f"  {len(trace)} ticks, field {trace['field_w'].iloc[0]:.3f} x "
            f"{trace['field_h'].iloc[0]:.3f} m, ticks by opponent count {n_by_count} "
            f"-> {out_path}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
