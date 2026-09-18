"""Ground-truth datasets: label taxonomy, frame records, and loading from disk.

The GT argument to the scorer is either a single dataset dir (data.yaml + images/ +
labels/) or a root containing such subdatasets. If a validation_state.json is present,
only frames it marks `pass` are loaded; failing that, an edit_labels.py .edit_state.json
is used the same way.

Frames are identified by FrameKey, which pairs the subdataset with the frame stamp. The
stamp alone is not unique across subdatasets: a set sampled from video restarts at 0 in
every recording, so keying by stamp silently collapsed same-numbered frames.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import NamedTuple

import numpy as np
import yaml

LEVELS = ("agnostic", "archetype", "instance")
AGNOSTIC_LABEL = "robot"


class FrameKey(NamedTuple):
    """Which frame this is: the subdataset it came from, and its stamp.

    Keying by stamp alone loses frames whenever two subdatasets number a frame the same.
    That is the normal case for a set built from video, where every recording starts at
    stamp 0: cage_high_x50_conf044 loaded 571 of its 636 frames, the nine recordings'
    frame 0 collapsing into one. Sets sampled from SVO recordings carry stamps unique
    across recordings and were never affected.

    dataset is the subdataset's path relative to the GT root, "." for a single dataset.
    stamp_ns stays available for the consumers that have to align GT against something
    else timestamped, such as compare_cpp_python.py against a C++ MCAP.
    """

    dataset: str
    stamp_ns: int

    def __str__(self) -> str:
        return f"{self.dataset}/{self.stamp_ns}" if self.dataset != "." else str(self.stamp_ns)


@dataclass
class Frame:
    """One aligned frame: GT boxes and candidate predictions, as (N, 4) xyxy arrays.

    gt_keypoints / pred_keypoints hold one (K, 3) array per box: x, y in pixels plus
    visibility (GT) or confidence (prediction). Empty arrays for box-only rows."""

    gt_boxes: np.ndarray
    gt_labels: list[str]
    gt_keypoints: list[np.ndarray]
    pred_boxes: np.ndarray
    pred_labels: list[str]
    pred_scores: np.ndarray
    pred_keypoints: list[np.ndarray]


class Taxonomy:
    """Label mapping for each comparison level."""

    def __init__(self, path: Path | None) -> None:
        data = yaml.safe_load(path.read_text()) if path and path.exists() else {}
        data = data or {}
        self.archetypes: dict[str, str] = dict(data.get("archetypes", {}))
        self.exclude: set[str] = set(data.get("exclude", []))

    def map_label(self, label: str, level: str) -> str:
        if level == "agnostic":
            return AGNOSTIC_LABEL
        if level == "archetype":
            return self.archetypes.get(label, label)
        return label


GtFrame = tuple[np.ndarray, list[str], list[np.ndarray]]


def _dataset_dirs(root: Path) -> list[Path]:
    """The dataset dir itself, or its subdataset children (dirs with a data.yaml)."""
    if (root / "data.yaml").exists():
        return [root]
    subs = sorted(d for d in root.iterdir() if d.is_dir() and (d / "data.yaml").exists())
    if not subs:
        raise SystemExit(f"No data.yaml found in {root} or its subdirectories")
    return subs


VALIDATION_STATE = "validation_state.json"
EDIT_STATE = ".edit_state.json"
VALIDATION_PASS = "pass"


def reviewed_frames(root: Path) -> set[tuple[str, str]] | None:
    """Frames that count as ground truth as (subdataset, stem) pairs, or None when the
    dataset tracks no review state.

    `validation_state.json` is the authority: it maps each image path, relative to the root, to a
    validation verdict, and only `pass` frames are scored. `.edit_state.json` is the older
    edit_labels.py state and is consulted only when no validation state exists. Preferring the
    older file silently drops whole recordings: on `nhrl_keypoints_eval_test` the edit state lists
    429 frames and names none of the 98 MassD ones, all of which validate as `pass`.

    The subdataset is carried alongside the stem for the same reason FrameKey carries it: two
    recordings can name a frame the same, and a verdict on one must not select or reject the
    other.

    Both files live at the dir that was pointed at, so a symlink root with neither scores every
    label file present."""

    def identity(rel: str) -> tuple[str, str]:
        # <subdataset>/images/<stem>.png, or images/<stem>.png for a single dataset.
        path = Path(rel)
        return str(path.parent.parent), path.stem

    validation_path = root / VALIDATION_STATE
    if validation_path.exists():
        state = json.loads(validation_path.read_text())
        return {identity(rel) for rel, verdict in state.items() if verdict == VALIDATION_PASS}

    edit_path = root / EDIT_STATE
    if not edit_path.exists():
        return None
    reviewed = json.loads(edit_path.read_text()).get("reviewed", [])
    return {identity(rel) for rel in reviewed}


def load_gt(root: Path) -> tuple[dict[FrameKey, GtFrame], list[str], dict[FrameKey, Path]]:
    """Read YOLO labels from a dataset dir or a root of subdatasets.

    When a review-state file exists, only the frames it accepts count as ground truth; see
    reviewed_frames for which file wins.
    Returns ({FrameKey: (boxes, labels, keypoints)}, names, {FrameKey: image path})."""
    reviewed = reviewed_frames(root)
    names: list[str] = []
    frames: dict[FrameKey, GtFrame] = {}
    images: dict[FrameKey, Path] = {}
    for dataset in _dataset_dirs(root):
        data = yaml.safe_load((dataset / "data.yaml").read_text())
        dataset_names = list(data["names"])
        # Subdatasets may trail off early (a recording without some class), but class
        # indices must agree.
        if dataset_names[: len(names)] != names[: len(dataset_names)]:
            raise SystemExit(f"Class names in {dataset} conflict with sibling datasets")
        if len(dataset_names) > len(names):
            names = dataset_names
        relative = str(dataset.relative_to(root))
        for label_path in sorted((dataset / "labels").glob("*.txt")):
            if reviewed is not None and (relative, label_path.stem) not in reviewed:
                continue
            image_path = _find_image(dataset / "images", label_path.stem)
            if image_path is None:
                continue
            width, height = _image_size(image_path)
            key = FrameKey(relative, int(label_path.stem))
            frames[key] = _parse_rows(label_path, dataset_names, width, height)
            images[key] = image_path
    if not frames:
        raise SystemExit(f"No scoreable labels found under {root}")
    return frames, names, images


def _find_image(images_dir: Path, stem: str) -> Path | None:
    for ext in (".png", ".jpg", ".jpeg", ".bmp", ".webp"):
        candidate = images_dir / f"{stem}{ext}"
        if candidate.exists():
            return candidate
    return None


def _image_size(image_path: Path) -> tuple[int, int]:
    from PIL import Image

    with Image.open(image_path) as img:
        return img.size


def _parse_rows(label_path: Path, names: list[str], width: int, height: int) -> GtFrame:
    """Parse detect (5 values) and pose (5 + 3k values) rows into pixel coordinates."""
    boxes = []
    labels = []
    keypoints = []
    for line in label_path.read_text().splitlines():
        parts = line.split()
        if len(parts) < 5 or (len(parts) - 5) % 3 != 0:
            continue
        class_id = int(float(parts[0]))
        values = [float(v) for v in parts[1:]]
        cx, cy, w, h = values[:4]
        boxes.append(
            [
                (cx - w / 2) * width,
                (cy - h / 2) * height,
                (cx + w / 2) * width,
                (cy + h / 2) * height,
            ]
        )
        labels.append(names[class_id] if class_id < len(names) else f"class_{class_id}")
        kps = [
            [values[i] * width, values[i + 1] * height, values[i + 2]]
            for i in range(4, len(values), 3)
        ]
        keypoints.append(np.asarray(kps, dtype=np.float64).reshape(-1, 3))
    return np.asarray(boxes, dtype=np.float64).reshape(-1, 4), labels, keypoints
