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
  - dropout gaps: run-lengths of consecutive ticks with no live opponent, in frames and milliseconds
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


@dataclass
class Reliability:
    frames: list[Frame]
    jump_rejects: int  # FrameId-assigner far-reassignment rejections over the run


def read_reliability(path: Path) -> Reliability:
    """Read the per-tick perception diagnostics and count FrameId-assigner jump rejects.

    Each /diagnostics message is one runner tick and carries at most one `perception` status. Ticks
    before the filter initializes have no `perception` status and are skipped.
    """
    frames: list[Frame] = []
    jump_rejects = 0
    for _topic, log_ns, data in iter_messages(path, [DIAGNOSTICS_TOPIC]):
        perc: dict[str, str] | None = None
        for status in decode_diagnostic_array(data):
            if status["name"] == "perception":
                perc = status["values"]
            elif status["name"] == "jump_reject":
                jump_rejects += 1
        if perc is None:
            continue
        frames.append(
            Frame(
                t=log_ns * 1e-9,
                their_total=int(perc.get("their_count_total", 0)),
                their_live=int(perc.get("their_count_live", 0)),
                our_live=int(perc.get("our_present_live", 0)),
            )
        )
    frames.sort(key=lambda f: f.t)
    return Reliability(frames=frames, jump_rejects=jump_rejects)


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


def opponent_gaps(frames: list[Frame]) -> Gaps:
    """Dropout gaps: maximal runs with no live opponent, in frames and real milliseconds."""
    t = np.array([f.t for f in frames])
    live = np.array([f.their_live > 0 for f in frames])
    n = len(frames)
    lens: list[int] = []
    durs: list[float] = []
    for a, b in _false_runs(live):
        lens.append(b - a)
        end_t = t[b] if b < n else t[-1]  # recovery time, or end of recording
        durs.append((end_t - t[a]) * 1e3)
    return Gaps(count=len(lens), frames=np.array(lens), ms=np.array(durs))


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

    gaps = opponent_gaps(frames)
    if gaps.count:
        print(
            f"  dropout gaps: {gaps.count}   frames [median {np.median(gaps.frames):.0f}, "
            f"p90 {np.percentile(gaps.frames, 90):.0f}, max {int(gaps.frames.max())}]"
        )
        print(
            f"                duration [median {np.median(gaps.ms):.0f} ms, "
            f"p90 {np.percentile(gaps.ms, 90):.0f} ms, max {gaps.ms.max():.0f} ms]"
        )
        print(f"  time without a live opponent: {100.0 * (1.0 - live.mean()):.1f}%")
    else:
        print("  no dropouts: a live opponent was present every frame")

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
    gaps = opponent_gaps(frames)

    fig, ax = plt.subplots(2, 1, figsize=(12, 6))
    ax[0].fill_between(
        t, 0, live.astype(float), step="pre", color="C0", alpha=0.5, label="opponent live"
    )
    ax[0].plot(t, our.astype(float) * 0.5, color="C1", lw=0.8, label="our-robot live (x0.5)")
    for a, b in _false_runs(live):
        end = t[b] if b < len(t) else t[-1]
        ax[0].axvspan(t[a], end, color="red", alpha=0.12, lw=0)
    ax[0].set_ylim(-0.05, 1.1)
    ax[0].set_xlabel("time (s)")
    ax[0].set_ylabel("track live")
    ax[0].set_title("opponent track availability (red = dropout)")
    ax[0].legend(fontsize=8, loc="upper right")

    if gaps.count:
        ax[1].hist(gaps.ms, bins=30, color="C3", alpha=0.8)
        ax[1].axvline(
            float(np.median(gaps.ms)),
            color="k",
            ls="--",
            lw=1,
            label=f"median {np.median(gaps.ms):.0f} ms",
        )
        ax[1].legend(fontsize=8)
    ax[1].set_xlabel("dropout gap duration (ms)")
    ax[1].set_ylabel("count")
    ax[1].set_title("how long the opponent track is lost at a time")

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
    args = parser.parse_args()

    for i, path in enumerate(args.mcap):
        rel = read_reliability(path)
        print_summary(path.name, rel)
        if i == 0 and args.csv:
            write_csv(args.csv, rel.frames)
        if i == 0 and args.plot:
            save_plot(args.plot, rel)


if __name__ == "__main__":
    main()
