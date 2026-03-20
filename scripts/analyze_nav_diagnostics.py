#!/usr/bin/env python3
"""Parse pursuit_nav diagnostics from an MCAP recording and produce
time-series plots + summary statistics for navigation failure diagnosis.

Dependencies: mcap, matplotlib, pandas, numpy
"""

from __future__ import annotations

import argparse
import struct
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from mcap.reader import make_reader


PURSUIT_NAV_HW_ID = "pursuit_nav"
RUNNER_HW_ID = "runner"
SIM_CAMERA_HW_ID = "sim_camera"
DIAGNOSTICS_TOPIC = "/diagnostics"


# ---------------------------------------------------------------------------
# ROS1 binary deserialization for diagnostic_msgs/DiagnosticArray
# ---------------------------------------------------------------------------
# Wire format (all little-endian):
#   Header:  uint32 seq, uint32 stamp_secs, uint32 stamp_nsecs, string frame_id
#   status:  uint32 count, then each DiagnosticStatus:
#     int8 level, string name, string message, string hardware_id,
#     values: uint32 count, then each KeyValue: string key, string value
#   string = uint32 length + raw bytes (no null terminator)
# ---------------------------------------------------------------------------


def _read_string(data: bytes, offset: int) -> tuple[str, int]:
    (length,) = struct.unpack_from("<I", data, offset)
    offset += 4
    s = data[offset : offset + length].decode("utf-8", errors="replace")
    return s, offset + length


def _read_uint32(data: bytes, offset: int) -> tuple[int, int]:
    (v,) = struct.unpack_from("<I", data, offset)
    return v, offset + 4


def _read_int8(data: bytes, offset: int) -> tuple[int, int]:
    (v,) = struct.unpack_from("<b", data, offset)
    return v, offset + 1


def _decode_diagnostic_array(data: bytes) -> list[dict]:
    """Decode a diagnostic_msgs/DiagnosticArray from raw ROS1 bytes.
    Returns a list of dicts, one per DiagnosticStatus."""
    off = 0

    # Header: seq, stamp.secs, stamp.nsecs, frame_id
    _seq, off = _read_uint32(data, off)
    _secs, off = _read_uint32(data, off)
    _nsecs, off = _read_uint32(data, off)
    _frame_id, off = _read_string(data, off)

    # status array
    status_count, off = _read_uint32(data, off)
    statuses = []
    for _ in range(status_count):
        level, off = _read_int8(data, off)
        name, off = _read_string(data, off)
        message, off = _read_string(data, off)
        hardware_id, off = _read_string(data, off)

        values_count, off = _read_uint32(data, off)
        values: dict[str, str] = {}
        for _ in range(values_count):
            key, off = _read_string(data, off)
            value, off = _read_string(data, off)
            values[key] = value

        statuses.append(
            {
                "level": level,
                "name": name,
                "message": message,
                "hardware_id": hardware_id,
                "values": values,
            }
        )
    return statuses


# ---------------------------------------------------------------------------
# MCAP extraction
# ---------------------------------------------------------------------------


def extract_diagnostics(path: Path) -> pd.DataFrame:
    """Read an MCAP file and return a DataFrame of pursuit_nav + runner/pipeline
    diagnostics, one row per tick (keyed on message timestamp)."""

    rows_by_ts: dict[int, dict[str, str]] = defaultdict(dict)

    with open(path, "rb") as f:
        reader = make_reader(f)
        for schema, channel, message in reader.iter_messages():
            if channel.topic != DIAGNOSTICS_TOPIC:
                continue

            ts_ns = message.log_time
            statuses = _decode_diagnostic_array(message.data)

            for status in statuses:
                hw_id = status["hardware_id"]
                name = status["name"]
                kv = status["values"]

                if hw_id == PURSUIT_NAV_HW_ID:
                    rows_by_ts[ts_ns].update(kv)
                elif hw_id == RUNNER_HW_ID and name == "pipeline":
                    for k, v in kv.items():
                        rows_by_ts[ts_ns][f"pipeline/{k}"] = v
                elif hw_id == SIM_CAMERA_HW_ID and name == "ground_truth":
                    for k, v in kv.items():
                        rows_by_ts[ts_ns][f"gt/{k}"] = v

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
        "gt/our_x",
        "gt/our_y",
        "gt/our_yaw_deg",
        "gt/opp1_x",
        "gt/opp1_y",
        "gt/opp1_yaw_deg",
    ]
    for col in numeric_cols:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")

    t0 = df["timestamp_ns"].iloc[0]
    df["t"] = (df["timestamp_ns"] - t0) / 1e9

    return df


# ---------------------------------------------------------------------------
# Ground truth comparison
# ---------------------------------------------------------------------------


def has_ground_truth(df: pd.DataFrame) -> bool:
    return "gt/our_x" in df.columns and "gt/our_yaw_deg" in df.columns


def _wrap_deg(d: pd.Series) -> pd.Series:
    """Wrap angle difference to [-180, 180]."""
    return (d + 180) % 360 - 180


def classify_yaw_jumps(
    df: pd.DataFrame, jump_threshold_deg: float = 90.0
) -> pd.DataFrame:
    """For each detected yaw jump, classify as identity swap (H2) or front/back flip (H4).

    Compares detected our_pose to ground truth poses of our robot and all opponents.
    - If detected yaw is close to an opponent's GT yaw: identity swap (H2).
    - If detected yaw differs from our GT yaw by ~180 deg: front/back flip (H4).
    - Otherwise: unknown.

    Returns a DataFrame with columns: t, detected_yaw, gt_our_yaw, classification.
    """
    if not has_ground_truth(df):
        return pd.DataFrame()

    jumps = detect_yaw_jumps(df, jump_threshold_deg)
    if not jumps.any():
        return pd.DataFrame()

    opp_yaw_cols = [
        c for c in df.columns if c.startswith("gt/opp") and c.endswith("_yaw_deg")
    ]
    opp_pos_cols_x = [
        c for c in df.columns if c.startswith("gt/opp") and c.endswith("_x")
    ]
    opp_pos_cols_y = [
        c for c in df.columns if c.startswith("gt/opp") and c.endswith("_y")
    ]

    records = []
    for idx in df.index[jumps]:
        row = df.loc[idx]
        det_yaw = row.get("our_yaw_deg", np.nan)
        det_x = row.get("our_x", np.nan)
        det_y = row.get("our_y", np.nan)
        gt_yaw = row.get("gt/our_yaw_deg", np.nan)
        gt_x = row.get("gt/our_x", np.nan)
        gt_y = row.get("gt/our_y", np.nan)

        if np.isnan(det_yaw) or np.isnan(gt_yaw):
            records.append({"t": row["t"], "classification": "no_data"})
            continue

        yaw_diff_from_gt = abs(((det_yaw - gt_yaw) + 180) % 360 - 180)

        # Check if detected pose is closer to an opponent's GT position
        best_opp_dist = float("inf")
        best_opp_yaw_diff = float("inf")
        for cx, cy, cyaw in zip(opp_pos_cols_x, opp_pos_cols_y, opp_yaw_cols):
            ox, oy, oyaw = (
                row.get(cx, np.nan),
                row.get(cy, np.nan),
                row.get(cyaw, np.nan),
            )
            if np.isnan(ox):
                continue
            d = np.sqrt((det_x - ox) ** 2 + (det_y - oy) ** 2)
            ydiff = abs(((det_yaw - oyaw) + 180) % 360 - 180)
            if d < best_opp_dist:
                best_opp_dist = d
                best_opp_yaw_diff = ydiff

        dist_to_gt_our = np.sqrt((det_x - gt_x) ** 2 + (det_y - gt_y) ** 2)

        if best_opp_dist < dist_to_gt_our and best_opp_yaw_diff < 45:
            classification = "identity_swap_H2"
        elif yaw_diff_from_gt > 120:
            classification = "front_back_flip_H4"
        elif yaw_diff_from_gt < 45:
            classification = "correct_detection"
        else:
            classification = "ambiguous"

        records.append(
            {
                "t": row["t"],
                "detected_yaw": det_yaw,
                "gt_our_yaw": gt_yaw,
                "yaw_diff_from_gt": yaw_diff_from_gt,
                "dist_to_gt_our": dist_to_gt_our,
                "dist_to_nearest_opp": best_opp_dist,
                "classification": classification,
            }
        )

    return pd.DataFrame(records)


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
    product = df["angular_z"].fillna(0) * dt
    cumulative = product.cumsum()
    last = cumulative.iloc[-1]
    if np.isnan(last):
        return 0.0
    return float(abs(last) / (2 * np.pi))


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


def compute_settling_time(
    df: pd.DataFrame, threshold_deg: float = 10.0, hold_s: float = 1.0
) -> float | None:
    """Time at which |angle_error| first stays below threshold for hold_s seconds.

    Returns seconds from recording start, or None if never settles.
    """
    if "angle_error_deg" not in df.columns:
        return None
    settled_start: float | None = None
    for _, row in df.iterrows():
        if abs(row["angle_error_deg"]) < threshold_deg:
            if settled_start is None:
                settled_start = row["t"]
            elif row["t"] - settled_start >= hold_s:
                return settled_start
        else:
            settled_start = None
    return None


def compute_time_to_target(df: pd.DataFrame, threshold_m: float = 0.15) -> float | None:
    """Time at which distance to target first drops below threshold. None if never reached."""
    if "distance" not in df.columns:
        return None
    close = df[df["distance"] < threshold_m]
    if close.empty:
        return None
    return float(close["t"].iloc[0])


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------


def plot_diagnostics(df: pd.DataFrame, save_path: str | None, no_show: bool) -> None:
    gt = has_ground_truth(df)
    n_panels = 6 if gt else 5
    fig, axes = plt.subplots(
        n_panels, 1, figsize=(14, 3.6 * n_panels), constrained_layout=True
    )
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
        ax.plot(t, df["our_yaw_deg"], linewidth=0.6, label="detected yaw")
        if gt and "gt/our_yaw_deg" in df.columns:
            ax.plot(
                t,
                df["gt/our_yaw_deg"],
                linewidth=0.6,
                alpha=0.7,
                color="green",
                label="GT our yaw",
            )
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
        ax.legend(loc="upper right", fontsize=8)
    ax.set_ylabel("our_yaw (deg)")
    ax.set_title("Detected Yaw vs Ground Truth")
    ax.grid(True, alpha=0.3)

    # -- 6. Ground truth classification (only if GT available) -----------
    if gt:
        ax = axes[5]
        jump_df = classify_yaw_jumps(df)
        if not jump_df.empty:
            colors = {
                "identity_swap_H2": "tab:orange",
                "front_back_flip_H4": "tab:purple",
                "correct_detection": "tab:green",
                "ambiguous": "tab:gray",
                "no_data": "tab:gray",
            }
            for cls, color in colors.items():
                subset = jump_df[jump_df["classification"] == cls]
                if subset.empty:
                    continue
                ax.scatter(
                    subset["t"],
                    subset.get("yaw_diff_from_gt", pd.Series(dtype=float)),
                    color=color,
                    s=25,
                    label=f"{cls} ({len(subset)})",
                    zorder=5,
                )
            ax.axhline(
                y=120, color="tab:purple", linestyle="--", alpha=0.4, linewidth=0.8
            )
            ax.axhline(
                y=45, color="tab:green", linestyle="--", alpha=0.4, linewidth=0.8
            )
            ax.legend(loc="upper right", fontsize=8)
        ax.set_ylabel("|detected - GT our| yaw (deg)")
        ax.set_xlabel("time (s)")
        ax.set_title(
            "Yaw Jump Classification: Identity Swap (H2) vs Front/Back Flip (H4)"
        )
        ax.grid(True, alpha=0.3)
    else:
        axes[-1].set_xlabel("time (s)")

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

    settling = compute_settling_time(df)
    if settling is not None:
        print(f"  Settling time (<10 deg for 1s): {settling:.1f} s")
    else:
        print(f"  Settling time (<10 deg for 1s): NEVER")

    ttt = compute_time_to_target(df)
    if ttt is not None:
        print(f"  Time to target (<0.15m):  {ttt:.1f} s")
    else:
        print(f"  Time to target (<0.15m):  NEVER")

    yaw_jumps = detect_yaw_jumps(df)
    print(f"  Yaw jumps (>90 deg): {yaw_jumps.sum()}")

    if has_ground_truth(df):
        jump_df = classify_yaw_jumps(df)
        if not jump_df.empty:
            counts = jump_df["classification"].value_counts()
            print(f"    -> identity swaps (H2):   {counts.get('identity_swap_H2', 0)}")
            print(
                f"    -> front/back flips (H4): {counts.get('front_back_flip_H4', 0)}"
            )
            print(f"    -> correct detections:    {counts.get('correct_detection', 0)}")
            print(f"    -> ambiguous:             {counts.get('ambiguous', 0)}")

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
