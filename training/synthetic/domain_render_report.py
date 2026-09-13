"""Gate report for a merged domain-mix render: per-view counts, classes, keypoints, drops, damage.

Step 3 of docs/experiments/perception_performance/synthetic_domain_mix_plan_2026-09-12.md lists the
checks a render passes before it goes to training. `validate_yolo_integrity.py --strict` covers the
dataset's structure; this covers what that cannot see, all split by view because the distorted and
rectified views cut objects at the frame edge and at the border where pinhole does not:

- frames per view and venue from `manifest.jsonl`, against an even three-way split
- instances per class, so our robots are not rare
- keypoints flagged 0 (not visible), against the randomized pool's rate
- frames dropped by the visibility gate, from each run's `render.log` in the parts directory
- fully clean frames, the pool the damage-off arm draws from

    venv/bin/python training/synthetic/domain_render_report.py \\
        training/data/synth_cage_nhrl_2026-09-13 \\
        --baseline training/data/all_robot_keypoints/train

Writes `gate_report.json` beside the dataset's manifest and exits 1 when a gate fails.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path

NAMES = ("mr_stabs_mk2", "mrs_buff_mk3", "nhrl_robot", "house_bot")
# A class-id column, a box, then (x, y, visibility) per keypoint.
KEYPOINT_START = 5
KEYPOINT_STRIDE = 3
# "Run summary: 100/100 images written across 11 scenes (2 frames dropped, 0 short)"
SUMMARY_RE = re.compile(
    r"Run summary: (?P<written>\d+)/\d+ images written across (?P<scenes>\d+) scenes"
    r"(?: \((?P<dropped>\d+) frames dropped)?"
)
# The step 3 gate: past this share of attempted frames dropped, the mat margin or the
# distractor count needs a look before the rest of the budget goes.
MAX_DROP_FRACTION = 0.25
# A view's frame count may sit this far from an exact third: the uneven split and rounding.
VIEW_COUNT_SLACK = 1
REPORT_NAME = "gate_report.json"


def label_stats(label_paths: list[Path]) -> tuple[Counter[str], int, int]:
    """(instances per class name, keypoints flagged 0, keypoints) over *label_paths*."""
    classes: Counter[str] = Counter()
    hidden = total = 0
    for path in label_paths:
        for line in path.read_text(encoding="utf-8").splitlines():
            fields = line.split()
            if not fields:
                continue
            class_id = int(fields[0])
            classes[NAMES[class_id] if class_id < len(NAMES) else str(class_id)] += 1
            flags = fields[KEYPOINT_START + 2 :: KEYPOINT_STRIDE]
            total += len(flags)
            hidden += sum(1 for flag in flags if float(flag) == 0.0)
    return classes, hidden, total


def run_drops(parts_dir: Path) -> dict[str, dict[str, int]]:
    """Frames written and dropped per run, summed over every attempt in its render.log."""
    runs: dict[str, dict[str, int]] = {}
    for log in sorted(parts_dir.glob("*/render.log")):
        written = dropped = scenes = 0
        for match in SUMMARY_RE.finditer(log.read_text(encoding="utf-8", errors="replace")):
            written += int(match["written"])
            scenes += int(match["scenes"])
            dropped += int(match["dropped"] or 0)
        runs[log.parent.name] = {"written": written, "dropped": dropped, "scenes": scenes}
    return runs


def is_clean(row: dict) -> bool:
    """Whether every instance damage was rolled for in this frame came out undamaged."""
    return all(float(instance.get("damage", 0.0)) == 0.0 for instance in row.get("instances", []))


def build_report(dataset: Path, parts_dir: Path, baseline: Path | None) -> dict:
    """Every gate number, split by view."""
    rows = [
        json.loads(line)
        for line in (dataset / "manifest.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    by_view: dict[str, list[dict]] = defaultdict(list)
    for row in rows:
        by_view[str(row.get("view", "unrecorded"))].append(row)

    views: dict[str, dict] = {}
    for view, view_rows in sorted(by_view.items()):
        labels = [dataset / "labels" / f"{Path(row['image']).stem}.txt" for row in view_rows]
        classes, hidden, keypoints = label_stats(labels)
        clean = sum(1 for row in view_rows if is_clean(row))
        views[view] = {
            "frames": len(view_rows),
            "venues": dict(Counter(str(row.get("venue")) for row in view_rows)),
            "classes": dict(sorted(classes.items())),
            "keypoints_hidden_fraction": round(hidden / keypoints, 4) if keypoints else None,
            "clean_frames": clean,
            "clean_fraction": round(clean / len(view_rows), 4),
        }

    drops = run_drops(parts_dir) if parts_dir.is_dir() else {}
    for view, entry in views.items():
        written = sum(run["written"] for name, run in drops.items() if name.endswith(f"_{view}"))
        dropped = sum(run["dropped"] for name, run in drops.items() if name.endswith(f"_{view}"))
        attempted = written + dropped
        entry["dropped_frames"] = dropped
        entry["drop_fraction"] = round(dropped / attempted, 4) if attempted else None

    report = {
        "dataset": str(dataset.resolve()),
        "frames": len(rows),
        "views": views,
        "runs": drops,
    }
    if baseline is not None:
        _, hidden, keypoints = label_stats(sorted((baseline / "labels").glob("synthetic__*.txt")))
        report["baseline_keypoints_hidden_fraction"] = (
            round(hidden / keypoints, 4) if keypoints else None
        )
    report["failures"] = gate_failures(report)
    return report


def gate_failures(report: dict) -> list[str]:
    """The gates a render fails, as sentences."""
    failures = []
    views = report["views"]
    if "unrecorded" in views:
        failures.append(f"{views['unrecorded']['frames']} manifest rows carry no view")
    if views:
        third = report["frames"] / len(views)
        for view, entry in views.items():
            if abs(entry["frames"] - third) > VIEW_COUNT_SLACK:
                failures.append(f"{view} has {entry['frames']} frames, not a third ({third:.0f})")
            drop = entry.get("drop_fraction")
            if drop is not None and drop > MAX_DROP_FRACTION:
                failures.append(f"{view} dropped {drop:.1%} of attempted frames")
            if entry["classes"].get("mrs_buff_mk3", 0) == 0:
                failures.append(f"{view} has no mrs_buff_mk3 instances")
    return failures


def print_report(report: dict) -> None:
    """The report as a markdown table, for pasting into the writeup."""
    print(f"{report['dataset']}: {report['frames']} frames\n")
    print("| View | Frames | Venues | Clean | Hidden kpts | Dropped | Instances |")
    print("| --- | --- | --- | --- | --- | --- | --- |")
    for view, entry in report["views"].items():
        venues = ", ".join(f"{venue} {count}" for venue, count in entry["venues"].items())
        classes = ", ".join(f"{name} {count}" for name, count in entry["classes"].items())
        hidden = entry["keypoints_hidden_fraction"]
        drop = entry.get("drop_fraction")
        print(
            f"| {view} | {entry['frames']} | {venues} | {entry['clean_fraction']:.1%} |"
            f" {'n/a' if hidden is None else f'{hidden:.1%}'} |"
            f" {'n/a' if drop is None else f'{drop:.1%}'} | {classes} |"
        )
    if "baseline_keypoints_hidden_fraction" in report:
        baseline = report["baseline_keypoints_hidden_fraction"]
        print(f"\nrandomized pool hidden keypoints: {baseline:.1%}")
    for failure in report["failures"]:
        print(f"GATE FAILED: {failure}")
    if not report["failures"]:
        print("\nall gates passed")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("dataset", type=Path, help="Merged render with manifest.jsonl")
    parser.add_argument(
        "--parts", type=Path, default=None, help="Per-run directories (default: <dataset>_parts)"
    )
    parser.add_argument(
        "--baseline", type=Path, default=None, help="Randomized pool split with labels/"
    )
    args = parser.parse_args()
    parts = args.parts or args.dataset.parent / f"{args.dataset.name}_parts"
    report = build_report(args.dataset, parts, args.baseline)
    (args.dataset / REPORT_NAME).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print_report(report)
    return 1 if report["failures"] else 0


if __name__ == "__main__":
    sys.exit(main())
