#!/usr/bin/env python3
"""Stage 2 of the run-away solver experiment: traces -> per-tick results + summary.

Loads the trace CSVs written by extract_traces.py, calls the C++ solvers through
run_away_solver_ext (built with -DBUILD_PYTHON_BINDINGS=ON in build/), and writes:

- results.csv: one row per (recording, tick, method) with center, radius,
  evaluations, elapsed_ns, plus the brute-force reference for that tick.
- summary.csv: metrics per method split by opponent count (0, 1, 2+, all).
- validation.csv: exact-vs-brute agreement per recording; any tick where the
  exact solver falls below the brute-force radius by more than the grid error
  bound is a bug in the exact solver.

The whole Python side is marshal, call, aggregate: no solver logic and no timing
lives here.

Historical: the experiment concluded and the exact solver won, so the nanobind
module, the CMake option, and the losing solvers were removed afterwards. To
reproduce this comparison, check out commit b2f37a0 (the last commit with all
four solvers and the bindings) and rebuild the module there.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]

MAX_OPPONENTS = 3
BRUTE_RESOLUTION_M = 0.001
GRID_RESOLUTION_M = 0.15
BNB_TOLERANCE_M = 0.01
# 1-Lipschitz radius: the best brute-force node is within h / sqrt(2) of the true optimum.
BRUTE_ERROR_BOUND_M = BRUTE_RESOLUTION_M / np.sqrt(2.0)
# 3 lb bot, ~0.16 m square footprint -> half-diagonal ~0.113 m. Drives the "nowhere safe"
# rate: a best radius below this means no spot on the field clears walls and opponents.
ROBOT_HALF_DIAGONAL_M = 0.113

# Solver name -> (attribute on ext.Method, tolerance argument). The enum values themselves
# come from the extension module, which is loaded lazily by load_extension().
METHODS: dict[str, tuple[str, float]] = {
    "brute": ("BRUTE_FORCE", BRUTE_RESOLUTION_M),
    "grid": ("COARSE_GRID", GRID_RESOLUTION_M),
    "exact": ("EXACT", 0.0),
    "bnb": ("BRANCH_AND_BOUND", BNB_TOLERANCE_M),
}


def load_extension():
    """Import the compiled solver from build/python.

    It is a build artifact, not an installed package, so the path is added here rather
    than at module import: importing this file should not depend on a build existing.
    """
    sys.path.append(str(REPO_ROOT / "build" / "python"))
    try:
        import run_away_solver_ext
    except ImportError as error:
        raise SystemExit(
            f"run_away_solver_ext not importable from {REPO_ROOT / 'build' / 'python'}: "
            f"{error}. The bindings were removed at b2f37a0; check that commit out to build them."
        ) from error
    return run_away_solver_ext


def flatten_opponents(trace: pd.DataFrame) -> tuple[np.ndarray, np.ndarray]:
    """Per-tick opponent lists as one flat (M, 2) array plus a per-tick count array,
    so nothing allocates per tick inside the C++ timed region."""
    counts = trace["n_opp"].to_numpy(dtype=np.int64)
    rows = []
    for i in range(MAX_OPPONENTS):
        cols = trace[[f"opp{i}_x", f"opp{i}_y"]].to_numpy(dtype=np.float64)
        valid = i < counts
        rows.append(cols[valid])
    # Re-interleave in tick order: build per-tick slices from the per-slot arrays.
    flat = np.zeros((int(counts.sum()), 2), dtype=np.float64)
    offsets = np.concatenate([[0], np.cumsum(counts)])
    for i in range(MAX_OPPONENTS):
        valid_ticks = np.nonzero(i < counts)[0]
        flat[offsets[valid_ticks] + i] = rows[i]
    return np.ascontiguousarray(flat), counts


def run_trace(trace: pd.DataFrame, recording: str, repeats: int) -> pd.DataFrame:
    ext = load_extension()
    field_w = float(trace["field_w"].iloc[0])
    field_h = float(trace["field_h"].iloc[0])
    opponents, counts = flatten_opponents(trace)

    frames = []
    for name, (method_name, parameter) in METHODS.items():
        method = getattr(ext.Method, method_name)
        out = ext.run_batch(field_w, field_h, opponents, counts, method, parameter, 1)
        frame = pd.DataFrame({key: np.asarray(value) for key, value in out.items()})
        if repeats > 1 and name != "brute":
            warm = ext.run_batch(field_w, field_h, opponents, counts, method, parameter, repeats)
            frame["elapsed_ns_warm"] = np.asarray(warm["elapsed_ns"])
        else:
            frame["elapsed_ns_warm"] = frame["elapsed_ns"]
        frame["method"] = name
        frame["recording"] = recording
        frame["timestamp_ns"] = trace["timestamp_ns"].to_numpy()
        frame["n_opp"] = counts
        frames.append(frame)
    results = pd.concat(frames, ignore_index=True)

    reference = (
        results[results["method"] == "brute"]
        .set_index("timestamp_ns")[["center_x", "center_y", "radius"]]
        .rename(columns={"center_x": "ref_x", "center_y": "ref_y", "radius": "ref_radius"})
    )
    return results.join(reference, on="timestamp_ns")


def opponent_bucket(n_opp: pd.Series) -> pd.Series:
    return pd.cut(n_opp, [-0.5, 0.5, 1.5, np.inf], labels=["0", "1", "2+"]).astype(str)


def summarize(results: pd.DataFrame) -> pd.DataFrame:
    results = results.copy()
    results["radius_err"] = results["ref_radius"] - results["radius"]
    results["center_disp"] = np.hypot(
        results["center_x"] - results["ref_x"], results["center_y"] - results["ref_y"]
    )
    results["bucket"] = opponent_bucket(results["n_opp"])

    rows = []
    for method in METHODS:
        subset = results[results["method"] == method]
        for bucket in ["all", "0", "1", "2+"]:
            part = subset if bucket == "all" else subset[subset["bucket"] == bucket]
            if part.empty:
                continue
            # Frame-to-frame center jitter, per recording so recording boundaries do not
            # count as steps.
            steps = []
            for _, rec in part.groupby("recording"):
                rec = rec.sort_values("timestamp_ns")
                steps.append(np.hypot(rec["center_x"].diff(), rec["center_y"].diff()).dropna())
            jitter = pd.concat(steps).abs() if steps else pd.Series(dtype=float)
            rows.append(
                {
                    "method": method,
                    "opponents": bucket,
                    "ticks": len(part),
                    "radius_err_mean_mm": part["radius_err"].mean() * 1e3,
                    "radius_err_p95_mm": part["radius_err"].quantile(0.95) * 1e3,
                    "radius_err_max_mm": part["radius_err"].max() * 1e3,
                    "center_disp_mean_m": part["center_disp"].mean(),
                    "center_disp_p95_m": part["center_disp"].quantile(0.95),
                    "center_disp_max_m": part["center_disp"].max(),
                    # Stale tracks repeat positions, so most steps are exactly zero and the
                    # median always lands there; the mean, p95, and big-hop rate carry the
                    # signal navigation actually feels.
                    "jitter_mean_mm": jitter.mean() * 1e3 if len(jitter) else np.nan,
                    "jitter_p95_mm": jitter.quantile(0.95) * 1e3 if len(jitter) else np.nan,
                    "jitter_hop_over_10cm_rate": ((jitter > 0.1).mean() if len(jitter) else np.nan),
                    "evaluations_mean": part["evaluations"].mean(),
                    "time_p50_us": part["elapsed_ns"].median() / 1e3,
                    "time_p95_us": part["elapsed_ns"].quantile(0.95) / 1e3,
                    "time_warm_p50_us": part["elapsed_ns_warm"].median() / 1e3,
                    "nowhere_safe_rate": (part["radius"] < ROBOT_HALF_DIAGONAL_M).mean(),
                }
            )
    return pd.DataFrame(rows)


def validate_exact(results: pd.DataFrame) -> pd.DataFrame:
    """Exact must match brute force everywhere: brute can beat exact by at most its own
    grid error bound, and exact must never fall below brute. Violations are exact-solver
    bugs (degenerate constraint triples), the thing this experiment exists to catch."""
    exact = results[results["method"] == "exact"]
    rows = []
    for recording, part in exact.groupby("recording"):
        deficit = part["ref_radius"] - part["radius"]  # > bound -> exact missed the optimum
        excess = part["radius"] - part["ref_radius"]  # exact may exceed brute by the bound
        rows.append(
            {
                "recording": recording,
                "ticks": len(part),
                "max_deficit_mm": deficit.max() * 1e3,
                "max_excess_mm": excess.max() * 1e3,
                "bound_mm": BRUTE_ERROR_BOUND_M * 1e3,
                "ticks_over_bound": int((deficit > BRUTE_ERROR_BOUND_M + 1e-9).sum()),
            }
        )
    return pd.DataFrame(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "traces",
        nargs="*",
        type=Path,
        default=sorted((Path(__file__).resolve().parent / "traces").glob("*.csv")),
    )
    parser.add_argument("--out-dir", type=Path, default=Path(__file__).resolve().parent / "out")
    parser.add_argument(
        "--repeats",
        type=int,
        default=20,
        help="extra cache-warm timing pass for the fast methods; reported next to the "
        "honest repeats=1 numbers, treat as a lower bound",
    )
    args = parser.parse_args()
    if not args.traces:
        print("no trace CSVs found; run extract_traces.py first", file=sys.stderr)
        return 1

    args.out_dir.mkdir(parents=True, exist_ok=True)
    all_results = []
    for trace_path in args.traces:
        trace = pd.read_csv(trace_path)
        print(f"solving {trace_path.stem} ({len(trace)} ticks)")
        all_results.append(run_trace(trace, trace_path.stem, args.repeats))
    results = pd.concat(all_results, ignore_index=True)
    results.to_csv(args.out_dir / "results.csv", index=False)

    summary = summarize(results)
    summary.to_csv(args.out_dir / "summary.csv", index=False)
    validation = validate_exact(results)
    validation.to_csv(args.out_dir / "validation.csv", index=False)

    pd.set_option("display.width", 200)
    print("\n=== exact vs brute force (every tick) ===")
    print(validation.to_string(index=False))
    print("\n=== summary ===")
    print(summary.round(4).to_string(index=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
