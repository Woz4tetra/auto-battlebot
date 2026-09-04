#!/usr/bin/env python3
"""Plot precision and recall against re-detection cadence for the tracker gap-fill experiment.

`score.py` treats its candidates as unordered names, so its plots cannot show the one thing
Experiment 3 measures: how accuracy falls as the gap between detections grows. This reads the
summary CSVs it wrote and draws that curve.

Constant-velocity coasting is drawn as a single marker at its own cadence rather than a line,
because it is the control at one setting, not a point on the tracker's curve.

Usage:
    python playground/plot_cadence_curve.py \
        --summary nhrl=<scores>/exp3_final_nhrl3/summary.csv \
        --summary massd=<scores>/exp3_final_massd/summary.csv \
        -o <assets>/exp3_cadence.png
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import pandas as pd

matplotlib.use("Agg")

FPS = 30.0
PRECISION_COLOUR = "#1f77b4"
RECALL_COLOUR = "#d62728"
COAST_COLOUR = "#7f7f7f"


def parse(summary: Path) -> tuple[pd.DataFrame, pd.DataFrame]:
    """(tracker arms ordered by cadence, coasting control rows) at the agnostic level."""
    frame = pd.read_csv(summary)
    frame = frame[frame.level == "agnostic"].copy()
    frame["cadence"] = frame.candidate.map(
        lambda name: int(match.group(1)) if (match := re.search(r"_every_(\d+)$", name)) else None
    )
    coast = frame[frame.candidate.str.startswith("coast")]
    tracker = frame[frame.candidate.str.startswith("detect")].sort_values("cadence")
    return tracker, coast


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument(
        "--summary",
        action="append",
        required=True,
        metavar="LABEL=PATH",
        help="split label and its score.py summary.csv, repeatable",
    )
    parser.add_argument("-o", "--output", type=Path, required=True)
    args = parser.parse_args()

    splits = [entry.split("=", 1) for entry in args.summary]
    figure, axes = plt.subplots(1, len(splits), figsize=(6.2 * len(splits), 4.6), squeeze=False)

    for axis, (label, path) in zip(axes[0], splits):
        tracker, coast = parse(Path(path))
        gaps = tracker.cadence * 1000.0 / FPS

        axis.plot(gaps, tracker.precision, "o-", color=PRECISION_COLOUR, label="precision")
        axis.plot(gaps, tracker.recall, "s-", color=RECALL_COLOUR, label="recall")
        for _, row in coast.iterrows():
            gap = row.cadence * 1000.0 / FPS
            axis.plot(gap, row.precision, "X", color=COAST_COLOUR, markersize=11)
            axis.plot(gap, row.recall, "P", color=COAST_COLOUR, markersize=11)
            axis.annotate(
                "constant-velocity\ncoast (control)",
                (gap, row.recall),
                textcoords="offset points",
                xytext=(6, -26),
                fontsize=8,
                color=COAST_COLOUR,
            )

        axis.set_xscale("log")
        axis.set_xticks(list(gaps))
        axis.set_xticklabels([f"{g:.0f}" for g in gaps])
        axis.minorticks_off()
        axis.set_xlabel("gap between detections (ms, 30 fps)")
        axis.set_ylabel("agnostic precision / recall")
        axis.set_ylim(0, 1.02)
        axis.grid(alpha=0.3)
        axis.set_title(label)
        axis.legend(loc="lower left", fontsize=9)

    figure.suptitle(
        "Experiment 3: accuracy vs re-detection cadence (ViT tracker fills the gaps)",
        fontsize=12,
    )
    figure.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=140)
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
