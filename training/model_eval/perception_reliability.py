"""Measure opponent-track reliability from a playback MCAP recording.

Stage 0 instrumentation for the control-improvement plan. From a recording (a real Jetson SVO
replayed in playback, or a live run), report how often a live (non-stale) opponent track is
available and how long the track is lost at a time. This sets how hard the predictor must work:
it coasts through the measured dropout gaps, so the gap-length distribution bounds the horizon.

Source data is the `perception` diagnostics subsection the runner emits once per tick
(their_count_total, their_count_live, our_present_live; src/runner.cpp), plus the `jump_reject`
events the FrameId assigner logs. Any recording carrying `/diagnostics` (post 2026-06-19) works
with no re-instrumentation. The message stamp is the camera frame stamp, so dropout durations
are real time even under free-run playback.

Metrics:
  - opponent validity: % of ticks with a live (non-stale) opponent (their_count_live > 0)
  - our-robot validity: % of ticks with a live OUR_ROBOT_1 (our_present_live > 0)
  - opponent dropout gaps: run-lengths of consecutive ticks with no live opponent (frames + ms)
  - our-robot dropout gaps: run-lengths of consecutive ticks with no live OUR_ROBOT_1 (frames + ms).
    These bound the keypoint-override decay `hold_window` in docs/robot_filter_decay_plan.md:
    if our-robot gaps are short, holding the "ours" identity across them is cheap.
  - track churn: jump_reject events per second (the FrameId assigner refusing a far reassignment)

Not yet measured: true track-ID-switch rate. The diagnostics carry opponent counts, not per-slot
identity, so a physical robot swapping FrameId slots is not directly observable. jump_reject rate
is the available proxy. Exact switch tracking needs a small addition to the perception diagnostics
block (log the live THEIRS FrameId set); see docs.

Usage:
    source scripts/activate_python.sh
    python training/model_eval/perception_reliability.py \
        data/recordings/auto_battlebot_experiment_playback_2026-06-20_00-15-08.mcap \
        --csv out/perception.csv --plot out/perception.png
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from auto_battlebot.mcap_io import (
    DIAGNOSTICS_TOPIC,
    decode_diagnostic_array,
    iter_messages,
)


@dataclass
class Frame:
    t: float  # camera frame stamp (s)
    their_total: int  # THEIRS descriptions present (live + coasting)
    their_live: int  # THEIRS descriptions that are not stale this frame
    our_live: int  # 1 if OUR_ROBOT_1 present and not stale
    our_blob_no_keypoint: int  # 1 if our keypoint missed AND a blob coincided with our held pose


@dataclass
class Reliability:
    frames: list[Frame]
    jump_rejects: int  # FrameId-assigner far-reassignment rejections over the run
    has_leak_signal: bool  # whether the recording carries our_blob_present_no_keypoint (new build)


def read_reliability(path: Path) -> Reliability:
    """Read the per-tick perception diagnostics and count FrameId-assigner jump rejects.

    Each /diagnostics message is one runner tick and carries at most one `perception` status. Ticks
    before the filter initializes have no `perception` status and are skipped.
    """
    frames: list[Frame] = []
    jump_rejects = 0
    has_leak_signal = False
    for _topic, log_ns, data in iter_messages(path, [DIAGNOSTICS_TOPIC]):
        perc: dict[str, str] | None = None
        for status in decode_diagnostic_array(data):
            if status["name"] == "perception":
                perc = status["values"]
            elif status["name"] == "jump_reject":
                jump_rejects += 1
        if perc is None:
            continue
        if "our_blob_present_no_keypoint" in perc:
            has_leak_signal = True
        frames.append(
            Frame(
                t=log_ns * 1e-9,
                their_total=int(perc.get("their_count_total", 0)),
                their_live=int(perc.get("their_count_live", 0)),
                our_live=int(perc.get("our_present_live", 0)),
                our_blob_no_keypoint=int(perc.get("our_blob_present_no_keypoint", 0)),
            )
        )
    frames.sort(key=lambda f: f.t)
    return Reliability(frames=frames, jump_rejects=jump_rejects, has_leak_signal=has_leak_signal)


def _false_runs(flags: np.ndarray) -> list[tuple[int, int]]:
    """Contiguous [start, stop) index ranges where flags is False (a dropout run)."""
    idx = np.flatnonzero(~flags)
    if len(idx) == 0:
        return []
    breaks = np.flatnonzero(np.diff(idx) > 1)
    starts = np.concatenate([[idx[0]], idx[breaks + 1]])
    stops = np.concatenate([idx[breaks], [idx[-1]]])
    return list(zip(starts.tolist(), (stops + 1).tolist()))


@dataclass
class Gaps:
    count: int
    frames: np.ndarray  # gap length in frames
    ms: np.ndarray  # gap duration in milliseconds (first-missing to recovery)


def _gaps(frames: list[Frame], live: np.ndarray) -> Gaps:
    """Dropout gaps: maximal runs where `live` is False, in frames and real milliseconds.

    `live` is a per-frame boolean mask aligned to `frames` (True = the track was live this tick).
    """
    t = np.array([f.t for f in frames])
    n = len(frames)
    lens: list[int] = []
    durs: list[float] = []
    for a, b in _false_runs(live):
        lens.append(b - a)
        end_t = t[b] if b < n else t[-1]  # recovery time, or end of recording
        durs.append((end_t - t[a]) * 1e3)
    return Gaps(count=len(lens), frames=np.array(lens), ms=np.array(durs))


def opponent_gaps(frames: list[Frame]) -> Gaps:
    """Dropout gaps: maximal runs with no live opponent, in frames and real milliseconds."""
    return _gaps(frames, np.array([f.their_live > 0 for f in frames]))


def our_robot_gaps(frames: list[Frame]) -> Gaps:
    """Dropout gaps: maximal runs with no live OUR_ROBOT_1, in frames and real milliseconds.

    These are the our-robot keypoint-dropout gaps the robot-filter decay plan bounds: the proposed
    `hold_window` should cover the typical gap (~p90) so the "ours" identity survives it.
    """
    return _gaps(frames, np.array([f.our_live > 0 for f in frames]))


def _print_gap_block(label: str, gaps: Gaps, time_without_pct: float | None = None) -> None:
    """Print a dropout-gap summary for one track (opponent or our-robot).

    `time_without_pct` (% of ticks with no live track) is printed when given; it is omitted for a
    pooled block across recordings, where a single live fraction is not meaningful.
    """
    if gaps.count:
        print(
            f"  {label} dropout gaps: {gaps.count}   frames [median {np.median(gaps.frames):.0f}, "
            f"p90 {np.percentile(gaps.frames, 90):.0f}, max {int(gaps.frames.max())}]"
        )
        print(
            f"      {' ' * len(label)}          duration [median {np.median(gaps.ms):.0f} ms, "
            f"p90 {np.percentile(gaps.ms, 90):.0f} ms, max {gaps.ms.max():.0f} ms]"
        )
        if time_without_pct is not None:
            print(f"  time without a live {label}: {time_without_pct:.1f}%")
    else:
        print(f"  no {label} dropouts: a live {label} was present every frame")


def print_summary(name: str, rel: Reliability) -> None:
    frames = rel.frames
    n = len(frames)
    if n < 2:
        print(f"{name}: no perception frames (recording predates the perception diagnostics?)")
        return
    t = np.array([f.t for f in frames])
    span = float(t[-1] - t[0])
    dt = float(np.median(np.diff(t)))
    live = np.array([f.their_live > 0 for f in frames])
    our = np.array([f.our_live > 0 for f in frames])

    present = 100.0 * float(np.mean([f.their_total > 0 for f in frames]))
    print(f"\n{name}: {n} frames over {span:.1f} s (~{1.0 / dt:.0f} fps)")
    print(
        f"  opponent live: {100.0 * live.mean():.1f}%   "
        f"our-robot live: {100.0 * our.mean():.1f}%   "
        f"opponent present (incl. coasting): {present:.1f}%"
    )

    _print_gap_block("opponent", opponent_gaps(frames), 100.0 * (1.0 - live.mean()))
    _print_gap_block("our-robot", our_robot_gaps(frames), 100.0 * (1.0 - our.mean()))

    if rel.has_leak_signal:
        leak = np.array([f.our_blob_no_keypoint > 0 for f in frames])
        n_leak = int(leak.sum())
        n_dropout = int((~our).sum())
        of_dropout = (
            f", {100.0 * n_leak / n_dropout:.1f}% of our-robot dropout ticks" if n_dropout else ""
        )
        print(
            f"  leak-opportunity ticks: {n_leak} "
            f"({100.0 * leak.mean():.1f}% of ticks, {n_leak / span:.2f}/s{of_dropout}) "
            "[our keypoint missed + coincident blob -> would leak as opponent]"
        )
    else:
        print(
            "  leak-opportunity: n/a (recording predates the our_blob_present_no_keypoint signal)"
        )

    print(
        f"  track churn: {rel.jump_rejects} jump_reject events "
        f"({rel.jump_rejects / span:.2f}/s) [proxy for track-ID instability]"
    )


def write_csv(path: Path, frames: list[Frame]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    t0 = frames[0].t if frames else 0.0
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "their_total", "their_live", "our_live"])
        for fr in frames:
            writer.writerow([f"{fr.t - t0:.4f}", fr.their_total, fr.their_live, fr.our_live])
    print(f"wrote {path}")


def write_gap_csv(path: Path, gaps: Gaps) -> None:
    """Per-gap rows (frame length + ms duration) for a single track's dropout gaps."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["gap_frames", "gap_ms"])
        for n_frames, ms in zip(gaps.frames.tolist(), gaps.ms.tolist()):
            writer.writerow([int(n_frames), f"{ms:.1f}"])
    print(f"wrote {path}")


def save_plot(path: Path, rel: Reliability) -> None:
    try:
        import matplotlib
    except ModuleNotFoundError:
        print("matplotlib not installed; skipping --plot")
        return
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    frames = rel.frames
    if len(frames) < 2:
        return
    t = np.array([f.t for f in frames]) - frames[0].t
    live = np.array([f.their_live > 0 for f in frames])
    our = np.array([f.our_live > 0 for f in frames])
    opp_gaps = opponent_gaps(frames)
    our_gaps = our_robot_gaps(frames)

    def _hist(axis: Any, gaps: Gaps, color: str, title: str) -> None:
        if gaps.count:
            axis.hist(gaps.ms, bins=30, color=color, alpha=0.8)
            axis.axvline(
                float(np.median(gaps.ms)),
                color="k",
                ls="--",
                lw=1,
                label=(
                    f"median {np.median(gaps.ms):.0f} ms, p90 {np.percentile(gaps.ms, 90):.0f} ms"
                ),
            )
            axis.legend(fontsize=8)
        axis.set_xlabel("dropout gap duration (ms)")
        axis.set_ylabel("count")
        axis.set_title(title)

    fig, ax = plt.subplots(3, 1, figsize=(12, 9))
    ax[0].fill_between(
        t, 0, live.astype(float), step="pre", color="C0", alpha=0.5, label="opponent live"
    )
    ax[0].plot(t, our.astype(float) * 0.5, color="C1", lw=0.8, label="our-robot live (x0.5)")
    for a, b in _false_runs(live):
        end = t[b] if b < len(t) else t[-1]
        ax[0].axvspan(t[a], end, color="red", alpha=0.10, lw=0)
    for a, b in _false_runs(our):
        end = t[b] if b < len(t) else t[-1]
        ax[0].axvspan(t[a], end, color="C1", alpha=0.12, lw=0)
    ax[0].set_ylim(-0.05, 1.1)
    ax[0].set_xlabel("time (s)")
    ax[0].set_ylabel("track live")
    ax[0].set_title("track availability (red = opponent dropout, orange = our-robot dropout)")
    ax[0].legend(fontsize=8, loc="upper right")

    _hist(ax[1], opp_gaps, "C3", "how long the opponent track is lost at a time")
    _hist(ax[2], our_gaps, "C1", "how long OUR_ROBOT_1 is lost (bounds decay hold_window)")

    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    print(f"wrote {path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "mcap", type=Path, nargs="+", help="one or more playback/live MCAP recordings"
    )
    parser.add_argument(
        "--csv", type=Path, default=None, help="per-frame CSV (first recording only)"
    )
    parser.add_argument(
        "--plot", type=Path, default=None, help="timeline + gap histogram (first recording)"
    )
    parser.add_argument(
        "--our-gap-csv",
        type=Path,
        default=None,
        help="per-gap our-robot dropout rows (frames, ms), pooled across all recordings",
    )
    args = parser.parse_args()

    pooled_our_frames: list[int] = []
    pooled_our_ms: list[float] = []
    pooled_ticks = 0
    pooled_span = 0.0
    pooled_leak_ticks = 0
    pooled_dropout_ticks = 0
    pooled_has_leak = False
    for i, path in enumerate(args.mcap):
        rel = read_reliability(path)
        print_summary(path.name, rel)
        g = our_robot_gaps(rel.frames)
        pooled_our_frames.extend(g.frames.tolist())
        pooled_our_ms.extend(g.ms.tolist())
        if len(rel.frames) >= 2:
            pooled_ticks += len(rel.frames)
            pooled_span += rel.frames[-1].t - rel.frames[0].t
            pooled_leak_ticks += sum(f.our_blob_no_keypoint > 0 for f in rel.frames)
            pooled_dropout_ticks += sum(f.our_live == 0 for f in rel.frames)
            pooled_has_leak = pooled_has_leak or rel.has_leak_signal
        if i == 0 and args.csv:
            write_csv(args.csv, rel.frames)
        if i == 0 and args.plot:
            save_plot(args.plot, rel)

    if len(args.mcap) > 1 and pooled_our_ms:
        pooled = Gaps(
            count=len(pooled_our_frames),
            frames=np.array(pooled_our_frames),
            ms=np.array(pooled_our_ms),
        )
        print(f"\npooled across {len(args.mcap)} recordings:")
        _print_gap_block("our-robot", pooled)
        if pooled_has_leak and pooled_span > 0:
            of_dropout = (
                f", {100.0 * pooled_leak_ticks / pooled_dropout_ticks:.1f}% of dropout ticks"
                if pooled_dropout_ticks
                else ""
            )
            print(
                f"  leak-opportunity ticks: {pooled_leak_ticks} "
                f"({100.0 * pooled_leak_ticks / pooled_ticks:.1f}% of ticks, "
                f"{pooled_leak_ticks / pooled_span:.2f}/s{of_dropout})"
            )
    if args.our_gap_csv:
        pooled = Gaps(
            count=len(pooled_our_frames),
            frames=np.array(pooled_our_frames),
            ms=np.array(pooled_our_ms),
        )
        write_gap_csv(args.our_gap_csv, pooled)


if __name__ == "__main__":
    main()
