#!/usr/bin/env python3
"""Produce a pipeline latency report from an MCAP recording.

Reads /diagnostics messages: per-stage FunctionTimer timings (elapsed_ms), the
runner's end-to-end pipeline latency (camera frame stamp to command send), and
the loop rate. Prints a per-stage timing table plus an end-to-end summary
checked against the 60 ms latency budget. Writes a markdown report and a plot
to docs/experiments/ by default.

Usage (inside the venv, see scripts/activate_python.sh):
    python scripts/mcap_latency_report.py data/recordings/<file>.mcap

Dependencies: mcap, matplotlib, numpy
"""

from __future__ import annotations

import argparse
import sys
from collections import defaultdict
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np

from auto_battlebot.mcap_io import DIAGNOSTICS_TOPIC, decode_diagnostic_array, iter_messages

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUT_DIR = REPO_ROOT / "docs" / "experiments"

RUNNER_HW_ID = "runner"
PIPELINE_NAME = "pipeline"
PIPELINE_STAGE = "pipeline.latency"
TICK_STAGE = "runner.tick"
BUDGET_MS = 60.0


# ---------------------------------------------------------------------------
# Extraction
# ---------------------------------------------------------------------------


@dataclass
class LatencySamples:
    """Per-stage timing samples pulled from /diagnostics."""

    # stage -> parallel lists of (time since start [s], duration [ms])
    stage_t_s: dict[str, list[float]]
    stage_ms: dict[str, list[float]]
    rate_hz: list[float]
    duration_s: float
    window_note: str | None = None


# First stage of the initialized tick path; its first sample marks successful field init.
FIELD_INIT_STAGE = "runner.field_filter.track_field"


def extract_latency_samples(path: Path, after_field_init: bool = False) -> LatencySamples:
    """Collect elapsed_ms per stage, pipeline latency, and loop rate from an MCAP file."""
    stage_t_ns: dict[str, list[int]] = defaultdict(list)
    stage_ms: dict[str, list[float]] = defaultdict(list)
    rate_hz: list[float] = []
    rate_t_ns: list[int] = []
    first_ns: int | None = None
    last_ns: int | None = None

    for _topic, t_ns, data in iter_messages(path, [DIAGNOSTICS_TOPIC]):
        first_ns = t_ns if first_ns is None else first_ns
        last_ns = t_ns
        for status in decode_diagnostic_array(data):
            hw_id = status["hardware_id"]
            name = status["name"]
            values = status["values"]
            # No subsection: the backend sets name = hardware_id. Avoid "x.x" stages.
            stage = hw_id if name == hw_id else f"{hw_id}.{name}"

            if "elapsed_ms" in values:
                stage_t_ns[stage].append(t_ns)
                stage_ms[stage].append(float(values["elapsed_ms"]))
            elif hw_id == RUNNER_HW_ID and name == PIPELINE_NAME and "latency_ms" in values:
                stage_t_ns[PIPELINE_STAGE].append(t_ns)
                stage_ms[PIPELINE_STAGE].append(float(values["latency_ms"]))
            elif hw_id == RUNNER_HW_ID and "rate" in values:
                rate_hz.append(float(values["rate"]))
                rate_t_ns.append(t_ns)

    if first_ns is None or last_ns is None or not stage_ms:
        print(f"No timing diagnostics found on {DIAGNOSTICS_TOPIC} in {path}", file=sys.stderr)
        sys.exit(1)

    t0 = first_ns
    window_note = None
    if after_field_init:
        if not stage_t_ns.get(FIELD_INIT_STAGE):
            print(
                f"--after-field-init: no {FIELD_INIT_STAGE} samples in {path}; "
                "the field was never initialized",
                file=sys.stderr,
            )
            sys.exit(1)
        t0 = stage_t_ns[FIELD_INIT_STAGE][0]
        for stage in list(stage_t_ns):
            kept = [
                (t, ms) for t, ms in zip(stage_t_ns[stage], stage_ms[stage], strict=True) if t >= t0
            ]
            if kept:
                stage_t_ns[stage] = [t for t, _ in kept]
                stage_ms[stage] = [ms for _, ms in kept]
            else:
                del stage_t_ns[stage]
                del stage_ms[stage]
        rate_hz = [r for t, r in zip(rate_t_ns, rate_hz, strict=True) if t >= t0]
        window_note = f"Window: after field init ({(t0 - first_ns) / 1e9:.1f} s into the recording)"

    stage_t_s = {stage: [(t - t0) / 1e9 for t in t_list] for stage, t_list in stage_t_ns.items()}
    return LatencySamples(
        stage_t_s=stage_t_s,
        stage_ms=dict(stage_ms),
        rate_hz=rate_hz,
        duration_s=(last_ns - t0) / 1e9,
        window_note=window_note,
    )


# ---------------------------------------------------------------------------
# Aggregation
# ---------------------------------------------------------------------------


@dataclass
class StageStats:
    stage: str
    count: int
    mean_ms: float
    median_ms: float
    p95_ms: float
    max_ms: float
    pct_tick: float | None  # share of the runner.tick mean; None where meaningless


def aggregate(samples: LatencySamples, stage_filter: list[str] | None) -> list[StageStats]:
    """Compute per-stage stats, sorted by mean descending. Pipeline latency is its own row."""
    tick_mean: float | None = None
    if TICK_STAGE in samples.stage_ms:
        tick_mean = float(np.mean(samples.stage_ms[TICK_STAGE]))

    stats: list[StageStats] = []
    for stage, values in samples.stage_ms.items():
        if stage_filter and not any(f in stage for f in stage_filter):
            continue
        arr = np.asarray(values, dtype=np.float64)
        pct_tick: float | None = None
        if tick_mean is not None and tick_mean > 0 and stage != PIPELINE_STAGE:
            pct_tick = 100.0 * float(arr.mean()) / tick_mean
        stats.append(
            StageStats(
                stage=stage,
                count=len(arr),
                mean_ms=float(arr.mean()),
                median_ms=float(np.median(arr)),
                p95_ms=float(np.percentile(arr, 95)),
                max_ms=float(arr.max()),
                pct_tick=pct_tick,
            )
        )
    stats.sort(key=lambda s: s.mean_ms, reverse=True)
    return stats


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------


def _table_rows(stats: list[StageStats]) -> list[tuple[str, str, str, str, str, str, str]]:
    rows = []
    for s in stats:
        pct = f"{s.pct_tick:.1f}%" if s.pct_tick is not None else "-"
        rows.append(
            (
                s.stage,
                f"{s.count:,}",
                f"{s.mean_ms:.2f}",
                f"{s.median_ms:.2f}",
                f"{s.p95_ms:.2f}",
                f"{s.max_ms:.2f}",
                pct,
            )
        )
    return rows


def _summary_lines(samples: LatencySamples, stats: list[StageStats]) -> list[str]:
    lines = [f"Duration: {samples.duration_s:.1f} s"]
    if samples.window_note:
        lines.append(samples.window_note)
    if samples.rate_hz:
        lines.append(f"Loop rate: {np.mean(samples.rate_hz):.1f} Hz mean")

    pipeline = next((s for s in stats if s.stage == PIPELINE_STAGE), None)
    if pipeline is None:
        lines.append("End-to-end pipeline latency: not recorded")
        return lines

    verdict = "OVER BUDGET" if pipeline.p95_ms > BUDGET_MS else "within budget"
    lines.append(
        f"End-to-end latency: mean {pipeline.mean_ms:.1f} ms / "
        f"p95 {pipeline.p95_ms:.1f} ms / max {pipeline.max_ms:.1f} ms"
    )
    lines.append(f"Budget: {BUDGET_MS:.0f} ms -> p95 is {verdict}")
    return lines


def print_report(path: Path, samples: LatencySamples, stats: list[StageStats]) -> None:
    col_stage = max(max(len(s.stage) for s in stats), 5)
    header = (
        f"{'Stage':<{col_stage}}  {'n':>8}  {'mean ms':>8}  {'med ms':>8}"
        f"  {'p95 ms':>8}  {'max ms':>8}  {'% tick':>7}"
    )
    print(f"\nFile: {path}")
    print("-" * len(header))
    print(header)
    print("-" * len(header))
    for stage, n, mean, med, p95, mx, pct in _table_rows(stats):
        print(f"{stage:<{col_stage}}  {n:>8}  {mean:>8}  {med:>8}  {p95:>8}  {mx:>8}  {pct:>7}")
    print("-" * len(header))

    width = 60
    print("=" * width)
    for line in _summary_lines(samples, stats):
        print(f"  {line}")
    print("=" * width)


def write_markdown_report(
    report_path: Path,
    mcap_path: Path,
    samples: LatencySamples,
    stats: list[StageStats],
    plot_name: str | None,
) -> None:
    lines = [
        f"# Latency report: {mcap_path.stem}",
        "",
        f"- Source: `{mcap_path}`",
        f"- Generated: {datetime.now().strftime('%Y-%m-%d %H:%M')} by"
        " `scripts/mcap_latency_report.py`",
    ]
    lines += [f"- {line}" for line in _summary_lines(samples, stats)]
    lines += [
        "",
        "| Stage | n | mean (ms) | median (ms) | p95 (ms) | max (ms) | % of tick |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in _table_rows(stats):
        lines.append("| " + " | ".join(row) + " |")
    if plot_name is not None:
        lines += ["", f"![latency plot]({plot_name})"]
    report_path.write_text("\n".join(lines) + "\n")
    print(f"Wrote report to {report_path}")


def write_csv(csv_path: Path, stats: list[StageStats]) -> None:
    lines = ["stage,count,mean_ms,median_ms,p95_ms,max_ms,pct_tick"]
    for s in stats:
        pct = f"{s.pct_tick:.3f}" if s.pct_tick is not None else ""
        lines.append(
            f"{s.stage},{s.count},{s.mean_ms:.3f},{s.median_ms:.3f},"
            f"{s.p95_ms:.3f},{s.max_ms:.3f},{pct}"
        )
    csv_path.write_text("\n".join(lines) + "\n")
    print(f"Wrote CSV to {csv_path}")


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------


def plot_latency(
    samples: LatencySamples,
    stats: list[StageStats],
    save_path: Path,
    show: bool,
    title: str,
) -> None:
    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax_time, ax_box) = plt.subplots(2, 1, figsize=(14, 9), constrained_layout=True)
    fig.suptitle(title, fontsize=13)

    # -- 1. End-to-end latency over time vs budget ------------------------
    if PIPELINE_STAGE in samples.stage_ms:
        t = samples.stage_t_s[PIPELINE_STAGE]
        v = samples.stage_ms[PIPELINE_STAGE]
        ax_time.plot(t, v, linewidth=0.7, color="#3b6fb6", label="pipeline latency")
        ax_time.axhline(
            BUDGET_MS,
            color="#b0413e",
            linestyle="--",
            linewidth=1.2,
            label=f"{BUDGET_MS:.0f} ms budget",
        )
        p95 = float(np.percentile(v, 95))
        ax_time.axhline(
            p95, color="#3b6fb6", linestyle=":", linewidth=1.0, label=f"p95 = {p95:.1f} ms"
        )
        ax_time.legend(loc="upper right", fontsize=9)
    else:
        ax_time.text(0.5, 0.5, "pipeline latency not recorded", ha="center", va="center")
    ax_time.set_xlabel("time (s)")
    ax_time.set_ylabel("latency (ms)")
    ax_time.set_title("End-to-end pipeline latency (camera stamp -> command)")
    ax_time.grid(True, alpha=0.3)

    # -- 2. Per-stage distribution, slowest at the top --------------------
    box_stats = [s for s in stats if s.stage != PIPELINE_STAGE]
    box_stats.reverse()  # horizontal boxplots draw bottom-up
    if box_stats:
        data = [samples.stage_ms[s.stage] for s in box_stats]
        bp = ax_box.boxplot(
            data,
            vert=False,
            labels=[s.stage for s in box_stats],
            showfliers=False,
            patch_artist=True,
            medianprops={"color": "#1f3a5f", "linewidth": 1.4},
        )
        for patch in bp["boxes"]:
            patch.set_facecolor("#9dbde3")
            patch.set_edgecolor("#3b6fb6")
    ax_box.set_xlabel("duration (ms)")
    ax_box.set_title("Per-stage timing distribution (outliers hidden)")
    ax_box.grid(True, axis="x", alpha=0.3)

    fig.savefig(save_path, dpi=150)
    print(f"Saved plot to {save_path}")
    if show:
        plt.show()
    plt.close(fig)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Report per-stage timing and end-to-end latency from an MCAP recording."
    )
    parser.add_argument("file", type=Path, help="Path to MCAP recording")
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=DEFAULT_OUT_DIR,
        help=f"Directory for the report, plot, and CSV (default: {DEFAULT_OUT_DIR})",
    )
    parser.add_argument("--no-plot", action="store_true", help="Skip the plot")
    parser.add_argument("--show", action="store_true", help="Open an interactive plot window")
    parser.add_argument("--csv", action="store_true", help="Also write the per-stage table as CSV")
    parser.add_argument(
        "--stages",
        type=str,
        default=None,
        help="Comma-separated substrings; only report stages matching one of them",
    )
    parser.add_argument(
        "--after-field-init",
        action="store_true",
        help="Only include samples after the first successful field initialization "
        f"(first {FIELD_INIT_STAGE} sample)",
    )
    args = parser.parse_args()

    if not args.file.exists():
        print(f"File not found: {args.file}", file=sys.stderr)
        sys.exit(1)

    print(f"Reading {args.file} ...")
    samples = extract_latency_samples(args.file, after_field_init=args.after_field_init)

    stage_filter = args.stages.split(",") if args.stages else None
    stats = aggregate(samples, stage_filter)
    if not stats:
        print(f"No stages match filter: {args.stages}", file=sys.stderr)
        sys.exit(1)

    print_report(args.file, samples, stats)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    stem = args.file.stem
    plot_name: str | None = None
    if not args.no_plot:
        plot_name = f"{stem}_latency.png"
        plot_latency(samples, stats, args.out_dir / plot_name, args.show, stem)
    write_markdown_report(args.out_dir / f"{stem}_latency.md", args.file, samples, stats, plot_name)
    if args.csv:
        write_csv(args.out_dir / f"{stem}_latency.csv", stats)


if __name__ == "__main__":
    main()
