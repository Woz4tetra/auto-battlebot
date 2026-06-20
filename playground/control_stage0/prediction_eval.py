#!/usr/bin/env python3
"""Quantify the robot filter's velocity-based motion prediction via an SVO-playback A/B.

The prediction in RobotTemporalMotionFilter::update_with_prediction advances OUR_ROBOT_1's pose
along the commanded velocity while it is unmeasured (a detection dropout), then emits it stale.
Navigation consumes that pose. This script measures how much that prediction helps by comparing two
recordings of the SAME SVO that differ only in whether the prediction loop runs:

  --on   prediction active (baseline)
  --off  prediction skipped (unmeasured robots hold their last pose)

Both runs require the PlaybackTransmitter fix that streams command feedback continuously; without it
the prediction never fires in playback and the two arms are identical.

Metric (resume error): a dropout "gap" is a run of ticks with perc/our_present_live == 0. The pose
the filter reports on the LAST gap tick is its best estimate of where OUR_ROBOT_1 is just before
re-acquisition. The FIRST measured pose after the gap is ground truth. Their distance is the
prediction error accumulated across the gap. With prediction this should stay small and roughly flat
in gap length; frozen (off) it should grow with gap length as the robot drives away from the stale
pose. Detections are identical between arms, so gaps align and the resume ground truth matches.

Usage:
  python prediction_eval.py --on <on.mcap> --off <off.mcap> --out <dir>

Dependencies: mcap, mcap_ros1, numpy, pandas, matplotlib (same as the rest of control_stage0).
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from mcap_ros1.reader import read_ros1_messages  # noqa: E402

from diag_io import (  # noqa: E402
    ROBOT_MARKERS_TOPIC,
    _log_time_ns,
    frame_name,
    load_diagnostics,
)

PRESENT_COL = "perc/our_present_live"
USING_PREV_COL = "nav/using_previous_robots"
OUR_FRAME = "OUR_ROBOT_1"

# Align /robot_markers to /diagnostics within this tolerance. Both are published once per loop tick
# (~25 ms apart between ticks), microseconds apart within a tick, so a few ms is ample.
_MERGE_TOL_NS = 12_000_000


def _load_our_marker_pose(path: Path) -> pd.DataFrame:
    """Per-/robot_markers-message OUR_ROBOT_1 pose, taken from the pre-cache filter output that
    runner.cpp publishes before robot_descriptions_cache_.resolve(). This is the filter's actual
    per-frame estimate (measured or predicted/frozen), unlike pursuit_nav's our_x which is the
    post-cache, possibly-substituted pose."""
    rows = []
    for m in read_ros1_messages(str(path), topics=[ROBOT_MARKERS_TOPIC]):
        ox = oy = None
        for mk in m.ros_msg.markers:
            if mk.ns != "robot_bounds":
                continue
            if frame_name(mk.id) == OUR_FRAME:
                ox = float(mk.pose.position.x)
                oy = float(mk.pose.position.y)
                break
        rows.append({"timestamp_ns": _log_time_ns(m), "our_x": ox, "our_y": oy})
    return pd.DataFrame(rows).sort_values("timestamp_ns").reset_index(drop=True)


@dataclass
class ArmResult:
    name: str
    ticks: pd.DataFrame  # per-tick: t, our_x, our_y, present, using_prev, jump_m
    gaps: pd.DataFrame  # per-gap: gap_len_ticks, gap_len_ms, resume_error_m, frozen_ref_error_m
    live_fraction: float
    using_prev_fraction: float
    mean_jump_m: float


def _per_tick(path: Path) -> pd.DataFrame:
    df = load_diagnostics(path)
    if PRESENT_COL not in df.columns:
        raise SystemExit(f"{path}: missing required diagnostics column '{PRESENT_COL}'")
    diag = pd.DataFrame(
        {
            "timestamp_ns": df["timestamp_ns"].astype("int64").to_numpy(),
            "t": df["t"].to_numpy(),
            "present": pd.to_numeric(df[PRESENT_COL], errors="coerce").fillna(0).astype(int).to_numpy(),
            "using_prev": (
                pd.to_numeric(df[USING_PREV_COL], errors="coerce").fillna(0).astype(int).to_numpy()
                if USING_PREV_COL in df.columns
                else 0
            ),
        }
    ).sort_values("timestamp_ns").reset_index(drop=True)

    markers = _load_our_marker_pose(path)
    markers["timestamp_ns"] = markers["timestamp_ns"].astype("int64")

    out = pd.merge_asof(
        diag,
        markers,
        on="timestamp_ns",
        direction="nearest",
        tolerance=_MERGE_TOL_NS,
    ).reset_index(drop=True)

    dx = out["our_x"].diff()
    dy = out["our_y"].diff()
    out["jump_m"] = np.sqrt(dx * dx + dy * dy)
    return out


def _gaps(ticks: pd.DataFrame) -> pd.DataFrame:
    """Extract dropout gaps bounded by live ticks. A gap is a maximal run of present==0 that has at
    least one live tick before it (frozen/predicted reference) and after it (resume ground truth)."""
    present = ticks["present"].to_numpy()
    n = len(present)
    rows = []
    i = 0
    while i < n:
        if present[i] == 0:
            j = i
            while j < n and present[j] == 0:
                j += 1
            # gap covers ticks [i, j-1]; live tick before is i-1, live tick after is j
            if i - 1 >= 0 and j < n:
                pre = ticks.iloc[i - 1]  # last live pose before the gap (frozen reference)
                last_gap = ticks.iloc[j - 1]  # filter's estimate just before re-acquisition
                resume = ticks.iloc[j]  # first measured pose after the gap (ground truth)
                if not any(
                    pd.isna(v)
                    for v in (
                        pre["our_x"], pre["our_y"],
                        last_gap["our_x"], last_gap["our_y"],
                        resume["our_x"], resume["our_y"],
                    )
                ):
                    resume_err = float(
                        np.hypot(last_gap["our_x"] - resume["our_x"], last_gap["our_y"] - resume["our_y"])
                    )
                    frozen_err = float(
                        np.hypot(pre["our_x"] - resume["our_x"], pre["our_y"] - resume["our_y"])
                    )
                    rows.append(
                        {
                            "gap_len_ticks": j - i,
                            "gap_len_ms": float((resume["t"] - pre["t"]) * 1000.0),
                            "resume_error_m": resume_err,
                            "frozen_ref_error_m": frozen_err,
                        }
                    )
            i = j
        else:
            i += 1
    return pd.DataFrame(rows)


def analyze(name: str, path: Path) -> ArmResult:
    ticks = _per_tick(path)
    gaps = _gaps(ticks)
    return ArmResult(
        name=name,
        ticks=ticks,
        gaps=gaps,
        live_fraction=float(ticks["present"].mean()),
        using_prev_fraction=float(ticks["using_prev"].mean()),
        mean_jump_m=float(np.nanmean(ticks["jump_m"].to_numpy())),
    )


def _fmt(x: float) -> str:
    return "n/a" if x is None or (isinstance(x, float) and np.isnan(x)) else f"{x:.4f}"


def write_report(on: ArmResult, off: ArmResult, out: Path) -> None:
    out.mkdir(parents=True, exist_ok=True)

    # Per-gap CSV (both arms).
    per_gap = pd.concat(
        [on.gaps.assign(arm="on"), off.gaps.assign(arm="off")], ignore_index=True
    )
    per_gap.to_csv(out / "gaps.csv", index=False)

    # Summary CSV.
    def summarize(r: ArmResult) -> dict:
        g = r.gaps
        return {
            "arm": r.name,
            "live_fraction": r.live_fraction,
            "using_prev_fraction": r.using_prev_fraction,
            "n_ticks": len(r.ticks),
            "n_gaps": len(g),
            "mean_gap_ms": float(g["gap_len_ms"].mean()) if len(g) else float("nan"),
            "median_gap_ms": float(g["gap_len_ms"].median()) if len(g) else float("nan"),
            "mean_resume_error_m": float(g["resume_error_m"].mean()) if len(g) else float("nan"),
            "median_resume_error_m": float(g["resume_error_m"].median()) if len(g) else float("nan"),
            "mean_frozen_ref_error_m": float(g["frozen_ref_error_m"].mean()) if len(g) else float("nan"),
            "mean_tick_jump_m": r.mean_jump_m,
        }

    summary = pd.DataFrame([summarize(on), summarize(off)])
    summary.to_csv(out / "summary.csv", index=False)

    # Markdown summary.
    lines = ["# Velocity-prediction A/B: results", ""]
    lines.append(f"- ON  ticks={len(on.ticks)} gaps={len(on.gaps)} live={on.live_fraction:.3f}")
    lines.append(f"- OFF ticks={len(off.ticks)} gaps={len(off.gaps)} live={off.live_fraction:.3f}")
    lines.append("")
    if len(on.gaps) == 0 and len(off.gaps) == 0:
        lines.append(
            "**No OUR_ROBOT_1 dropout gaps in this recording.** The prediction is only exercised "
            "during our-robot dropouts, so this recording cannot distinguish ON from OFF. Pick a "
            "recording with more our-robot dropouts and rerun."
        )
    else:
        lines.append("| metric | ON | OFF |")
        lines.append("| --- | --- | --- |")
        s_on, s_off = summarize(on), summarize(off)
        for key in (
            "n_gaps",
            "mean_gap_ms",
            "mean_resume_error_m",
            "median_resume_error_m",
            "mean_frozen_ref_error_m",
            "mean_tick_jump_m",
        ):
            lines.append(f"| {key} | {_fmt(s_on[key])} | {_fmt(s_off[key])} |")
        lines.append("")
        if len(on.gaps) and len(off.gaps):
            improv = s_off["mean_resume_error_m"] - s_on["mean_resume_error_m"]
            lines.append(
                f"Mean resume error is {_fmt(improv)} m lower with prediction ON "
                f"({_fmt(s_on['mean_resume_error_m'])} vs {_fmt(s_off['mean_resume_error_m'])} m). "
                "Positive means prediction reduced the post-dropout position error."
            )
            lines.append("")
            lines.append(
                "Because prediction also changes data association (it advances last_position), the two "
                "arms do not have identical gap sets, so the table below bins by gap length to compare "
                "at matched dropout durations."
            )
            lines.append("")
            lines.append("| gap length (ms) | n ON | n OFF | resume err ON (m) | resume err OFF (m) |")
            lines.append("| --- | --- | --- | --- | --- |")
            bins = [0, 40, 80, 120, 200, 400, 2000]
            on_b = on.gaps.assign(bin=pd.cut(on.gaps["gap_len_ms"], bins))
            off_b = off.gaps.assign(bin=pd.cut(off.gaps["gap_len_ms"], bins))
            for b in on_b["bin"].cat.categories:
                o = on_b[on_b["bin"] == b]["resume_error_m"]
                f = off_b[off_b["bin"] == b]["resume_error_m"]
                if len(o) == 0 and len(f) == 0:
                    continue
                lines.append(
                    f"| {int(b.left)}–{int(b.right)} | {len(o)} | {len(f)} | "
                    f"{_fmt(o.mean()) if len(o) else 'n/a'} | {_fmt(f.mean()) if len(f) else 'n/a'} |"
                )
    (out / "summary.md").write_text("\n".join(lines) + "\n")

    # Plot 1: resume error vs gap length, ON vs OFF.
    fig, ax = plt.subplots(figsize=(7, 5))
    for r, color in ((off, "tab:red"), (on, "tab:blue")):
        if len(r.gaps):
            ax.scatter(
                r.gaps["gap_len_ms"], r.gaps["resume_error_m"], s=18, alpha=0.5,
                color=color, label=f"{r.name} (n={len(r.gaps)})",
            )
            # binned mean
            g = r.gaps.copy()
            if len(g) >= 4:
                bins = np.linspace(g["gap_len_ms"].min(), g["gap_len_ms"].max() + 1e-6, 6)
                g["bin"] = pd.cut(g["gap_len_ms"], bins)
                bm = g.groupby("bin", observed=True).agg(
                    x=("gap_len_ms", "mean"), y=("resume_error_m", "mean")
                )
                ax.plot(bm["x"], bm["y"], color=color, lw=2)
    ax.set_xlabel("dropout gap length (ms)")
    ax.set_ylabel("resume error (m)")
    ax.set_title("OUR_ROBOT_1 post-dropout position error vs gap length")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "resume_error_vs_gap.png", dpi=120)
    plt.close(fig)

    # Plot 2: per-tick jump distribution (pose smoothness).
    fig, ax = plt.subplots(figsize=(7, 5))
    for r, color in ((off, "tab:red"), (on, "tab:blue")):
        jumps = r.ticks["jump_m"].dropna().to_numpy()
        jumps = jumps[jumps > 1e-4]
        if len(jumps):
            ax.hist(jumps, bins=60, range=(0, np.percentile(jumps, 99)), histtype="step",
                    color=color, label=f"{r.name} (mean={r.mean_jump_m:.4f} m)")
    ax.set_xlabel("frame-to-frame OUR_ROBOT_1 position jump (m)")
    ax.set_ylabel("count")
    ax.set_title("Pose smoothness (lower/tighter is smoother)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "pose_jump_hist.png", dpi=120)
    plt.close(fig)

    print(f"Wrote {out}/summary.md, summary.csv, gaps.csv, resume_error_vs_gap.png, pose_jump_hist.png")
    print((out / "summary.md").read_text())


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--on", required=True, type=Path, help="MCAP with prediction ON (baseline)")
    ap.add_argument("--off", required=True, type=Path, help="MCAP with prediction OFF (patched)")
    ap.add_argument("--out", required=True, type=Path, help="output directory")
    args = ap.parse_args()

    on = analyze("on", args.on)
    off = analyze("off", args.off)
    write_report(on, off, args.out)


if __name__ == "__main__":
    main()
