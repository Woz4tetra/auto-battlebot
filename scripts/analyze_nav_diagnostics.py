#!/usr/bin/env python3
"""Parse pursuit_nav diagnostics from an MCAP recording and produce
time-series plots + summary statistics for navigation failure diagnosis.

Dependencies: mcap, mcap-ros1-support, matplotlib, pandas, numpy
"""

from __future__ import annotations

import argparse
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from mcap_ros1.decoder import DecoderFactory
from mcap.reader import make_reader


PURSUIT_NAV_HW_ID = "pursuit_nav"
RUNNER_HW_ID = "runner"
DIAGNOSTICS_TOPIC = "/diagnostics"


# ---------------------------------------------------------------------------
# MCAP extraction
# ---------------------------------------------------------------------------


def _kv_to_dict(values) -> dict[str, str]:
    """Convert a list of diagnostic_msgs/KeyValue to a plain dict."""
    return {kv.key: kv.value for kv in values}


def extract_diagnostics(path: Path) -> pd.DataFrame:
    """Read an MCAP file and return a DataFrame of pursuit_nav + runner/pipeline
    diagnostics, one row per tick (keyed on message timestamp)."""

    # Subsection name -> list of {timestamp_ns, key: value, ...}
    rows_by_ts: dict[int, dict[str, str]] = defaultdict(dict)

    with open(path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for schema, channel, message, decoded in reader.iter_decoded_messages(
            topics=[DIAGNOSTICS_TOPIC]
        ):
            ts_ns = message.log_time
            for status in decoded.status:
                hw_id = status.hardware_id
                name = status.name  # subsection

                if hw_id == PURSUIT_NAV_HW_ID:
                    kv = _kv_to_dict(status.values)
                    rows_by_ts[ts_ns].update(kv)
                elif hw_id == RUNNER_HW_ID and name == "pipeline":
                    kv = _kv_to_dict(status.values)
                    for k, v in kv.items():
                        rows_by_ts[ts_ns][f"pipeline/{k}"] = v

    if not rows_by_ts:
        print("No pursuit_nav diagnostics found in the recording.", file=sys.stderr)
        sys.exit(1)

    timestamps_ns = sorted(rows_by_ts.keys())
    records = []
    for ts_ns in timestamps_ns:
        row = rows_by_ts[ts_ns]
        row["timestamp_ns"] = ts_ns
        records.append(row)

    df = pd.DataFrame(records)

    # Convert numeric columns (all values come as strings from DiagnosticStatus)
    numeric_cols = [
        "our_x",
        "our_y",
        "our_yaw_deg",
        "target_x",
        "target_y",
        "distance",
        "angle_to_target_deg",
        "angle_error_deg",
        "threshold_deg",
        "facing_target",
        "turn_commit",
        "linear_x",
        "angular_z",
        "pipeline/latency_ms",
    ]
    for col in numeric_cols:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")

    t0 = df["timestamp_ns"].iloc[0]
    df["t"] = (df["timestamp_ns"] - t0) / 1e9  # seconds from start

    return df


# ---------------------------------------------------------------------------
# Anomaly detection helpers
# ---------------------------------------------------------------------------


def detect_yaw_jumps(df: pd.DataFrame, threshold_deg: float = 90.0) -> pd.Series:
    """Boolean mask where consecutive yaw jumps exceed threshold."""
    if "our_yaw_deg" not in df.columns:
        return pd.Series(False, index=df.index)
    delta = df["our_yaw_deg"].diff().abs()
    # Handle wrap-around: if delta > 180, it's likely a wrap, not a jump
    delta = delta.where(delta <= 180, 360 - delta)
    return delta > threshold_deg


def detect_pos_jumps(df: pd.DataFrame, threshold_m: float = 0.2) -> pd.Series:
    """Boolean mask where consecutive position jumps exceed threshold."""
    if "our_x" not in df.columns or "our_y" not in df.columns:
        return pd.Series(False, index=df.index)
    dx = df["our_x"].diff()
    dy = df["our_y"].diff()
    return np.sqrt(dx**2 + dy**2) > threshold_m


def detect_target_switches(df: pd.DataFrame) -> pd.Series:
    """Boolean mask where the target frame_id changes between ticks."""
    if "target_frame_id" not in df.columns:
        return pd.Series(False, index=df.index)
    return df["target_frame_id"] != df["target_frame_id"].shift(1)


def count_full_spins(df: pd.DataFrame) -> float:
    """Estimate total full rotations from integrated angular command."""
    if "angular_z" not in df.columns or "t" not in df.columns:
        return 0.0
    dt = df["t"].diff().fillna(0)
    cumulative_rad = (df["angular_z"] * dt).cumsum()
    # angular_z is normalized [-1,1]; actual rad/s depends on max_angular_velocity
    # but for counting sign-changes and relative magnitude this suffices
    return float(cumulative_rad.abs().iloc[-1] / (2 * np.pi))


def detect_spin_events(
    df: pd.DataFrame, cmd_thresh: float = 0.8, min_duration_s: float = 0.5
) -> list[tuple[float, float]]:
    """Find contiguous windows where |angular_z| > cmd_thresh for > min_duration_s.
    Returns list of (start_t, end_t) pairs."""
    if "angular_z" not in df.columns:
        return []
    spinning = df["angular_z"].abs() > cmd_thresh
    events = []
    start = None
    for i, (t, is_spin) in enumerate(zip(df["t"], spinning)):
        if is_spin and start is None:
            start = t
        elif not is_spin and start is not None:
            if t - start >= min_duration_s:
                events.append((start, t))
            start = None
    if start is not None and df["t"].iloc[-1] - start >= min_duration_s:
        events.append((start, float(df["t"].iloc[-1])))
    return events


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------


def plot_diagnostics(df: pd.DataFrame, save_path: str | None, no_show: bool) -> None:
    fig, axes = plt.subplots(5, 1, figsize=(14, 18), constrained_layout=True)
    fig.suptitle("Pursuit Navigation Diagnostics", fontsize=14)

    t = df["t"]

    # -- 1. XY trajectory ------------------------------------------------
    ax = axes[0]
    if "our_x" in df.columns and "our_y" in df.columns:
        sc = ax.scatter(
            df["our_x"],
            df["our_y"],
            c=t,
            cmap="viridis",
            s=4,
            label="our robot",
        )
        plt.colorbar(sc, ax=ax, label="time (s)")
    if "target_x" in df.columns and "target_y" in df.columns:
        ax.scatter(
            df["target_x"],
            df["target_y"],
            c=t,
            cmap="Reds",
            s=4,
            marker="x",
            label="target",
        )
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("XY Trajectory")
    ax.set_aspect("equal")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # -- 2. Angle error + turn commit ------------------------------------
    ax = axes[1]
    if "angle_error_deg" in df.columns:
        ax.plot(t, df["angle_error_deg"], linewidth=0.6, label="angle_error")
    if "turn_commit" in df.columns:
        tc = df["turn_commit"].fillna(0)
        ax.fill_between(
            t,
            -180,
            180,
            where=(tc > 0),
            alpha=0.12,
            color="blue",
            label="commit +1",
        )
        ax.fill_between(
            t,
            -180,
            180,
            where=(tc < 0),
            alpha=0.12,
            color="red",
            label="commit -1",
        )
    ax.set_ylabel("angle error (deg)")
    ax.set_title("Angle Error + Turn Commit")
    ax.set_ylim(-200, 200)
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    # -- 3. Commands -----------------------------------------------------
    ax = axes[2]
    if "angular_z" in df.columns:
        ax.plot(t, df["angular_z"], linewidth=0.6, color="tab:red", label="angular_z")
    if "linear_x" in df.columns:
        ax2 = ax.twinx()
        ax2.plot(t, df["linear_x"], linewidth=0.6, color="tab:blue", label="linear_x")
        ax2.set_ylabel("linear_x", color="tab:blue")
        ax2.set_ylim(-0.1, 1.1)
    ax.set_ylabel("angular_z", color="tab:red")
    ax.set_ylim(-1.2, 1.2)
    ax.set_title("Velocity Commands")
    ax.grid(True, alpha=0.3)

    # -- 4. Distance to target -------------------------------------------
    ax = axes[3]
    if "distance" in df.columns:
        ax.plot(t, df["distance"], linewidth=0.6)
    ax.set_ylabel("distance (m)")
    ax.set_title("Distance to Target")
    ax.grid(True, alpha=0.3)

    # -- 5. Yaw over time with jump markers ------------------------------
    ax = axes[4]
    if "our_yaw_deg" in df.columns:
        ax.plot(t, df["our_yaw_deg"], linewidth=0.6)
        jumps = detect_yaw_jumps(df)
        if jumps.any():
            ax.scatter(
                t[jumps],
                df["our_yaw_deg"][jumps],
                color="red",
                s=30,
                zorder=5,
                label=f"yaw jumps ({jumps.sum()})",
            )
            ax.legend()
    ax.set_ylabel("our_yaw (deg)")
    ax.set_xlabel("time (s)")
    ax.set_title("Our Yaw (jump markers = suspected front/back flip)")
    ax.grid(True, alpha=0.3)

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f"Saved figure to {save_path}")
    if not no_show:
        plt.show()
    plt.close(fig)


# ---------------------------------------------------------------------------
# Summary statistics
# ---------------------------------------------------------------------------


def print_summary(df: pd.DataFrame) -> None:
    duration = df["t"].iloc[-1] - df["t"].iloc[0]
    print(f"\n{'=' * 60}")
    print(f"  Navigation Diagnostics Summary")
    print(f"{'=' * 60}")
    print(f"  Recording duration:  {duration:.1f} s  ({len(df)} ticks)")

    if "angle_error_deg" in df.columns:
        ae = df["angle_error_deg"].abs()
        print(f"  |angle_error| mean:  {ae.mean():.1f} deg")
        print(f"  |angle_error| max:   {ae.max():.1f} deg")

    yaw_jumps = detect_yaw_jumps(df)
    print(f"  Yaw jumps (>90 deg): {yaw_jumps.sum()}")

    pos_jumps = detect_pos_jumps(df)
    print(f"  Position jumps (>0.2m): {pos_jumps.sum()}")

    target_sw = detect_target_switches(df)
    n_switches = target_sw.sum()
    if "target_frame_id" in df.columns:
        print(f"  Target switches:     {n_switches}")
    else:
        print(f"  Target switches:     (target_frame_id not logged)")

    spins = count_full_spins(df)
    print(f"  Cumulative rotations (cmd): ~{spins:.1f}")

    spin_events = detect_spin_events(df)
    total_spin_s = sum(e - s for s, e in spin_events)
    print(
        f"  Spin events (|az|>0.8, >0.5s): {len(spin_events)}  ({total_spin_s:.1f} s total)"
    )

    if "pipeline/latency_ms" in df.columns:
        lat = df["pipeline/latency_ms"].dropna()
        if len(lat) > 0:
            print(f"  Pipeline latency mean: {lat.mean():.1f} ms")
            print(f"  Pipeline latency p95:  {lat.quantile(0.95):.1f} ms")
            print(f"  Pipeline latency max:  {lat.max():.1f} ms")

    print(f"{'=' * 60}\n")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Analyze pursuit_nav diagnostics from an MCAP recording."
    )
    parser.add_argument("file", type=Path, help="Path to MCAP recording")
    parser.add_argument(
        "--save", type=str, default=None, help="Save figure to this path (PNG)"
    )
    parser.add_argument(
        "--no-show", action="store_true", help="Don't open interactive plot window"
    )
    args = parser.parse_args()

    if not args.file.exists():
        print(f"File not found: {args.file}", file=sys.stderr)
        sys.exit(1)

    print(f"Reading {args.file} ...")
    df = extract_diagnostics(args.file)
    print(f"Extracted {len(df)} ticks")

    print_summary(df)
    plot_diagnostics(df, save_path=args.save, no_show=args.no_show)


if __name__ == "__main__":
    main()
