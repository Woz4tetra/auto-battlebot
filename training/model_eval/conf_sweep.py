#!/usr/bin/env python3
"""Find the confidence threshold whose detection count matches what a dataset should hold.

A pre-label run at a fixed confidence over-detects on purpose, so the box count says as much about
the threshold as about the model. This runs each candidate once at a low floor, keeps every
detection's confidence, and then sweeps thresholds offline: one inference pass answers "what
threshold would have produced the expected count", for every threshold at once.

The expectation is a per-frame box rate, measured from a labelled set rather than assumed. For NHRL
cage footage `nhrl_keypoints_eval_test` gives 2.81 boxes per frame over its 590 NHRL frames
(mrs_buff_mk3 0.98, opponent 1.14, house_bot 0.68), which is below the nominal two robots plus a
house bot because the house bot is only in frame 68 percent of the time.

Matching the count is necessary, not sufficient: an arm can hit the right number of boxes with them
in the wrong places, and a class the footage never contains (Mr Stabs, in Mrs Buff's matches) is a
false positive at every threshold. The per-class table is printed for that reason.

Detections are cached per candidate under --cache, so re-sweeping against a different expectation
costs nothing.

Usage:
    venv/bin/python training/model_eval/conf_sweep.py training/data/nhrl_cage_high_eval \\
        --candidate base=data/models/yolo26s-pose_base_2026-09-13_last.pt \\
        --map nhrl_robot=opponent --expected-per-frame 2.81 --device 0
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg"}


def subdatasets(root: Path) -> list[Path]:
    """The dataset itself if it has images/, else every child that does."""
    if (root / "images").is_dir():
        return [root]
    return sorted(child for child in root.iterdir() if (child / "images").is_dir())


def dataset_names(root: Path) -> list[str]:
    """The class names of the first subdataset that declares them."""
    for subdataset in subdatasets(root):
        for name in ("data.yaml", "data.yml"):
            path = subdataset / name
            if path.is_file():
                return list(yaml.safe_load(path.read_text(encoding="utf-8"))["names"])
    raise SystemExit(f"{root} has no data.yaml")


def images(root: Path) -> list[Path]:
    """Every frame in the dataset, over all subdatasets."""
    found = []
    for subdataset in subdatasets(root):
        found += sorted(
            path
            for path in (subdataset / "images").iterdir()
            if path.suffix.lower() in IMAGE_SUFFIXES
        )
    return found


def parse_pairs(pairs: list[str], what: str) -> dict[str, str]:
    """``left=right`` pairs."""
    out = {}
    for pair in pairs:
        left, _, right = pair.partition("=")
        if not right:
            raise SystemExit(f"--{what} wants left=right, got {pair!r}")
        out[left] = right
    return out


def detect(model_path: Path, frames: list[Path], renames: dict[str, str], args) -> list[list]:
    """``[class_name, confidence]`` for every detection at or above the floor."""
    from ultralytics import YOLO  # deferred: slow import, and --help should not pay it

    model = YOLO(str(model_path))
    names = {index: renames.get(name, name) for index, name in model.names.items()}
    rows = []
    for frame in frames:
        result = model.predict(
            str(frame), conf=args.conf_floor, imgsz=args.imgsz, device=args.device, verbose=False
        )[0]
        for index in range(len(result.boxes)):
            name = names[int(result.boxes.cls[index])]
            rows.append([name, float(result.boxes.conf[index])])
    return rows


def sweep(rows: list[list], frames: int, target: float, classes: list[str]) -> list[dict]:
    """Total and per-class counts at each threshold from 0.05 to 0.95."""
    table = []
    for step in range(5, 96):
        threshold = step / 100
        kept = [row for row in rows if row[1] >= threshold]
        table.append(
            {
                "conf": threshold,
                "total": len(kept),
                "per_frame": len(kept) / frames,
                "error": abs(len(kept) - target),
                "by_class": {name: sum(1 for row in kept if row[0] == name) for name in classes},
            }
        )
    return table


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("dataset", type=Path, help="Eval dataset, or a root of subdatasets")
    parser.add_argument(
        "--candidate",
        action="append",
        required=True,
        metavar="NAME=WEIGHTS",
        help="Ultralytics pose .pt to sweep, repeatable",
    )
    parser.add_argument("--map", nargs="*", default=[], metavar="MODEL=DATASET")
    parser.add_argument("--conf-floor", type=float, default=0.05, help="Lowest threshold swept")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0")
    parser.add_argument(
        "--expected-per-frame",
        type=float,
        default=2.81,
        help="Boxes per frame the footage should hold, measured from a labelled set",
    )
    parser.add_argument("--cache", type=Path, help="Directory of cached detections per candidate")
    args = parser.parse_args()

    candidates = parse_pairs(args.candidate, "candidate")
    renames = parse_pairs(args.map, "map")
    classes = dataset_names(args.dataset)
    frames = images(args.dataset)
    target = args.expected_per_frame * len(frames)
    print(
        f"{len(frames)} frames, expecting {args.expected_per_frame:.2f}/frame = {target:.0f} boxes"
    )
    print(f"classes: {classes}\n")

    if args.cache:
        args.cache.mkdir(parents=True, exist_ok=True)
    best_rows = []
    for name, weights in candidates.items():
        cached = args.cache / f"{name}.json" if args.cache else None
        if cached and cached.is_file():
            rows = json.loads(cached.read_text(encoding="utf-8"))
        else:
            rows = detect(Path(weights), frames, renames, args)
            if cached:
                cached.write_text(json.dumps(rows), encoding="utf-8")
        table = sweep(rows, len(frames), target, classes)
        best = min(table, key=lambda entry: entry["error"])
        best_rows.append((name, best))
        counts = "  ".join(f"{c} {best['by_class'].get(c, 0)}" for c in classes)
        print(
            f"{name:<20} conf {best['conf']:.2f}  {best['total']:>5} boxes "
            f"({best['per_frame']:.2f}/frame)  {counts}"
        )

    print(f"\n{'arm':<20}{'best conf':>10}{'boxes':>8}{'/frame':>9}{'off by':>9}")
    for name, best in sorted(best_rows, key=lambda item: item[1]["conf"]):
        print(
            f"{name:<20}{best['conf']:>10.2f}{best['total']:>8}{best['per_frame']:>9.2f}"
            f"{best['total'] - target:>+9.0f}"
        )


if __name__ == "__main__":
    main()
