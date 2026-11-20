"""Build the all-keypoints arm: a domain-mix arm list plus both hand-reviewed eval sets."""

from __future__ import annotations

import argparse
import json
import os
from collections import Counter
from pathlib import Path

import yaml

NAMES = ["mr_stabs_mk2", "mrs_buff_mk3", "nhrl_robot", "house_bot"]
REVIEW_STATE = "validation_state.json"


def fix_row(line: str) -> tuple[str, int]:
    """*line* with every keypoint outside [0, 1] set to unlabelled, and how many were."""
    parts = line.split()
    fixed = 0
    for i in range(5, len(parts), 3):
        x, y = float(parts[i]), float(parts[i + 1])
        if not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0):
            parts[i : i + 3] = ["0.000000", "0.000000", "0"]
            fixed += 1
    return " ".join(parts), fixed


def eval_frames(root: Path, out: Path, dropped: Counter, unlabelled: Counter) -> list[Path]:
    """The pass frames of one eval set, as image paths with sibling labels that fit NAMES."""
    state = json.loads((root / REVIEW_STATE).read_text(encoding="utf-8"))
    frames = []
    for key, verdict in sorted(state.items()):
        if verdict != "pass":
            continue
        image = root / key
        label = image.parent.parent / "labels" / f"{image.stem}.txt"
        if not image.is_file() or not label.is_file():
            raise SystemExit(f"{root}: passed frame {key} is missing its image or label")
        rows = [line for line in label.read_text(encoding="utf-8").splitlines() if line.strip()]
        kept = []
        fixed = 0
        for line in rows:
            if int(line.split()[0]) >= len(NAMES):
                continue
            row, count = fix_row(line)
            kept.append(row)
            fixed += count
        if len(kept) == len(rows) and not fixed:
            frames.append(image.resolve())
            continue
        dropped[root.name] += len(rows) - len(kept)
        unlabelled[root.name] += fixed
        link = out / "eval_labels" / root.name / key
        link.parent.mkdir(parents=True, exist_ok=True)
        if link.is_symlink():
            link.unlink()
        link.symlink_to(image.resolve())
        new_label = link.parent.parent / "labels" / f"{image.stem}.txt"
        new_label.parent.mkdir(parents=True, exist_ok=True)
        new_label.write_text("".join(f"{line}\n" for line in kept), encoding="utf-8")
        frames.append(link.absolute())
    return frames


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("--arm", type=Path, required=True, help="Domain-mix arm image list")
    parser.add_argument("--val", type=Path, required=True, help="Validation image list")
    parser.add_argument("--eval", type=Path, nargs="+", required=True, help="Reviewed eval sets")
    parser.add_argument("--out", type=Path, required=True, help="Directory for the list and yaml")
    parser.add_argument("--name", default="all_keypoints", help="Arm name for <name>.txt/.yml")
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    arm = [Path(line.strip()) for line in args.arm.read_text(encoding="utf-8").splitlines()]
    val = [Path(line.strip()) for line in args.val.read_text(encoding="utf-8").splitlines()]
    dropped: Counter = Counter()
    unlabelled: Counter = Counter()
    extra = {root.name: eval_frames(root, args.out, dropped, unlabelled) for root in args.eval}

    frames = arm + [frame for group in extra.values() for frame in group]
    real = [os.path.realpath(frame) for frame in frames]
    if len(set(real)) != len(real):
        raise SystemExit("an image appears twice once symlinks are resolved")
    if set(real) & {os.path.realpath(frame) for frame in val}:
        raise SystemExit("val overlaps the training list")

    (args.out / "val.txt").write_text("".join(f"{path}\n" for path in val), encoding="utf-8")
    listing = args.out / f"{args.name}.txt"
    listing.write_text("".join(f"{path}\n" for path in sorted(frames)), encoding="utf-8")
    meta = {
        "path": str(args.out.resolve()),
        "train": listing.name,
        "val": "val.txt",
        "nc": len(NAMES),
        "names": NAMES,
        "kpt_shape": [2, 3],
        "flip_idx": [0, 1],
    }
    (args.out / f"{args.name}.yml").write_text(yaml.safe_dump(meta, sort_keys=False), "utf-8")
    manifest = {
        "arm": str(args.arm.resolve()),
        "arm_frames": len(arm),
        "eval": {name: len(group) for name, group in extra.items()},
        "dropped_rows": dict(dropped),
        "unlabelled_keypoints": dict(unlabelled),
        "frames": len(frames),
        "val_frames": len(val),
    }
    (args.out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", "utf-8")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
