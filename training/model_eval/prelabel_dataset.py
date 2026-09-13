#!/usr/bin/env python3
"""Pre-label an eval dataset's empty frames with a pose model, for correction in edit_labels.py.

Step 6 of docs/experiments/perception_performance/synthetic_domain_mix_plan_2026-09-12.md.
``make_eval_dataset.py`` writes empty labels; this fills them with a model's detections so a
labeller deletes spurious boxes instead of drawing every box and both keypoints from scratch.
``export_labels.py`` cannot do this: it reads detection topics out of a label_playback MCAP, and
sampled frames have none.

Three rules keep pre-labels from poisoning the eval:

* **Only empty label files are written.** A frame someone already labelled is never touched, so a
  second round with a better model only reaches the frames nobody has reviewed.
* **Nothing is marked reviewed.** A pre-labelled frame looks finished at a glance, so review state
  stays with ``edit_labels.py``.
* **A blind hold-out stays empty.** ``--holdout`` leaves a fixed fraction of frames unlabelled and
  lists them in ``prelabel_holdout.json``, to be labelled from scratch. Box counts there against
  the pre-labelled population are the measured bias the eval carries.

Run at a low confidence (default 0.15): deleting a spurious box is one keypress, drawing a missed
box plus two keypoints is a dozen actions.

Usage:
    venv/bin/python training/model_eval/prelabel_dataset.py <dataset_root> \\
        --model data/models/yolo26x-pose_all_robot_keypoints_2026-09-05_last.pt \\
        --map nhrl_robot=opponent --conf 0.15 --holdout 0.1 --device cpu
"""

from __future__ import annotations

import argparse
import json
import random
from pathlib import Path

import yaml

IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg"}
HOLDOUT_NAME = "prelabel_holdout.json"
# A keypoint the model is at least this sure of is labelled visible (2), the rest labelled but
# not visible (1), matching how the eval set flags occluded keypoints.
KEYPOINT_VISIBLE_CONF = 0.5


def subdatasets(root: Path) -> list[Path]:
    """The dataset itself if it has images/, else every child that does."""
    if (root / "images").is_dir():
        return [root]
    return sorted(child for child in root.iterdir() if (child / "images").is_dir())


def dataset_names(subdataset: Path) -> list[str]:
    """The subdataset's class names, from its data.yaml."""
    for name in ("data.yaml", "data.yml"):
        path = subdataset / name
        if path.is_file():
            return list(yaml.safe_load(path.read_text(encoding="utf-8"))["names"])
    raise SystemExit(f"{subdataset} has no data.yaml")


def parse_map(pairs: list[str]) -> dict[str, str]:
    """``model_name=dataset_name`` pairs."""
    mapping = {}
    for pair in pairs:
        source, _, target = pair.partition("=")
        if not target:
            raise SystemExit(f"--map wants model_name=dataset_name, got {pair!r}")
        mapping[source] = target
    return mapping


def empty_frames(subdataset: Path) -> list[Path]:
    """Images whose label file is missing or holds no rows."""
    frames = []
    for image in sorted((subdataset / "images").iterdir()):
        if image.suffix.lower() not in IMAGE_SUFFIXES:
            continue
        label = subdataset / "labels" / f"{image.stem}.txt"
        if not label.is_file() or not label.read_text(encoding="utf-8").strip():
            frames.append(image)
    return frames


def label_rows(result, model_names: dict[int, str], class_ids: dict[str, int]) -> list[str]:
    """YOLO pose rows for one ultralytics result, in the dataset's class ids."""
    rows = []
    boxes = result.boxes
    keypoints = result.keypoints
    for index in range(len(boxes)):
        name = model_names[int(boxes.cls[index])]
        if name not in class_ids:
            continue
        cx, cy, w, h = (float(v) for v in boxes.xywhn[index])
        fields = [str(class_ids[name]), f"{cx:.6f}", f"{cy:.6f}", f"{w:.6f}", f"{h:.6f}"]
        if keypoints is not None:
            xyn = keypoints.xyn[index]
            conf = keypoints.conf[index] if keypoints.conf is not None else None
            for k in range(len(xyn)):
                visible = conf is None or float(conf[k]) >= KEYPOINT_VISIBLE_CONF
                x, y = float(xyn[k][0]), float(xyn[k][1])
                fields += [f"{x:.6f}", f"{y:.6f}", "2" if visible else "1"]
        rows.append(" ".join(fields))
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("dataset", type=Path, help="Eval dataset, or a root of subdatasets")
    parser.add_argument("--model", type=Path, required=True, help="Ultralytics pose .pt")
    parser.add_argument(
        "--map",
        nargs="*",
        default=[],
        metavar="MODEL=DATASET",
        help="Rename model classes to dataset classes, e.g. nhrl_robot=opponent",
    )
    parser.add_argument("--conf", type=float, default=0.15, help="Detection confidence floor")
    parser.add_argument("--imgsz", type=int, default=640, help="Inference size")
    parser.add_argument("--device", default="cpu", help="'cpu' or a CUDA index")
    parser.add_argument(
        "--holdout", type=float, default=0.1, help="Fraction of empty frames left for blind audit"
    )
    parser.add_argument("--seed", type=int, default=0, help="Seed for the hold-out draw")
    args = parser.parse_args()

    from ultralytics import YOLO  # deferred: slow import, and --help should not pay it

    model = YOLO(str(args.model))
    renames = parse_map(args.map)
    model_names = {i: renames.get(n, n) for i, n in model.names.items()}
    for subdataset in subdatasets(args.dataset):
        names = dataset_names(subdataset)
        class_ids = {name: index for index, name in enumerate(names)}
        frames = empty_frames(subdataset)
        rng = random.Random(f"{args.seed}:{subdataset.name}")
        holdout = sorted(rng.sample(frames, round(len(frames) * args.holdout)))
        held = set(holdout)
        (subdataset / "labels").mkdir(exist_ok=True)
        written = boxes = 0
        for image in frames:
            if image in held:
                continue
            result = model.predict(
                str(image), conf=args.conf, imgsz=args.imgsz, device=args.device, verbose=False
            )[0]
            rows = label_rows(result, model_names, class_ids)
            label = subdataset / "labels" / f"{image.stem}.txt"
            label.write_text("".join(f"{row}\n" for row in rows), encoding="utf-8")
            written += 1
            boxes += len(rows)
        record = {
            "model": str(args.model),
            "conf": args.conf,
            "map": renames,
            "prelabelled": written,
            "boxes": boxes,
            "holdout": [str(path.relative_to(subdataset)) for path in holdout],
        }
        (subdataset / HOLDOUT_NAME).write_text(
            json.dumps(record, indent=2) + "\n", encoding="utf-8"
        )
        print(
            f"{subdataset.name}: {written} pre-labelled ({boxes} boxes), {len(holdout)} held out"
            f" of {len(frames)} empty frames"
        )


if __name__ == "__main__":
    main()
