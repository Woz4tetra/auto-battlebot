"""Build the all-keypoints arm: a domain-mix arm list plus both hand-reviewed eval sets.

Trains a deployment model on every reviewed keypoint frame there is, so neither eval set can score
it afterwards. Each eval set contributes only the frames its ``validation_state.json`` marks
``pass``. Eval labels in a class the model does not have (``object``, index 4) are dropped: the
frame's labels are rewritten under ``<out>/eval_labels/`` beside symlinked images, because
ultralytics finds a label by swapping ``images`` for ``labels`` in the image path.

Usage:
  venv/bin/python training/yolo/make_all_keypoints_arm.py \\
      --arm training/data/domain_mix_arms_2026-09-19/d50000.txt \\
      --val training/data/domain_mix_arms_2026-09-19/val.txt \\
      --eval training/data/cage_high_x50_conf044 training/data/nhrl_keypoints_eval_test \\
      --out training/data/all_keypoints_2026-09-25 --name all_keypoints
"""

from __future__ import annotations

import argparse
import json
import os
from collections import Counter
from pathlib import Path

import yaml

NAMES = ["mr_stabs_mk2", "mrs_buff_mk3", "nhrl_robot", "house_bot"]
REVIEW_STATE = "validation_state.json"


def eval_frames(root: Path, out: Path, dropped: Counter) -> list[Path]:
    """The `pass` frames of one eval set, as image paths whose sibling labels fit `NAMES`.

    Raises:
        SystemExit: When a passed frame lacks its image or label.
    """
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
        kept = [line for line in rows if int(line.split()[0]) < len(NAMES)]
        if len(kept) == len(rows):
            frames.append(image.resolve())
            continue
        dropped[root.name] += len(rows) - len(kept)
        # <out>/eval_labels/<set>/<key's dirs>/images/<file> links the image, labels/ holds the
        # rewritten rows.
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
    extra = {root.name: eval_frames(root, args.out, dropped) for root in args.eval}

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
        "frames": len(frames),
        "val_frames": len(val),
    }
    (args.out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", "utf-8")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
