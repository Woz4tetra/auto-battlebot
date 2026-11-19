"""Ground-truth datasets: label taxonomy, frame records, loading from disk."""

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
    """Which frame this is: the subdataset it came from, and its stamp."""

    dataset: str
    stamp_ns: int

    def __str__(self) -> str:
        return f"{self.dataset}/{self.stamp_ns}" if self.dataset != "." else str(self.stamp_ns)


@dataclass
class Frame:
    """One aligned frame: GT boxes and candidate predictions."""

    gt_boxes: np.ndarray
    gt_labels: list[str]
    gt_keypoints: list[np.ndarray]
    pred_boxes: np.ndarray
    pred_labels: list[str]
    pred_scores: np.ndarray
    pred_keypoints: list[np.ndarray]

    def __post_init__(self) -> None:
        for name, boxes, labels, keypoints in (
            ("gt", self.gt_boxes, self.gt_labels, self.gt_keypoints),
            ("pred", self.pred_boxes, self.pred_labels, self.pred_keypoints),
        ):
            if boxes.ndim != 2 or boxes.shape[1] != 4:
                raise ValueError(f"{name}_boxes has shape {boxes.shape}, expected (N, 4)")
            if len(labels) != len(boxes) or len(keypoints) != len(boxes):
                raise ValueError(f"{name} labels and keypoints need one entry per box")
            for row in keypoints:
                if row.size and (row.ndim != 2 or row.shape[1] != 3):
                    raise ValueError(
                        f"{name}_keypoints entry has shape {row.shape}, expected (K, 3)"
                    )
        if len(self.pred_scores) != len(self.pred_boxes):
            raise ValueError("pred_scores needs one entry per predicted box")


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
    """Frames that count as ground truth, as (subdataset, stem) pairs, or None."""

    def identity(rel: str) -> tuple[str, str]:
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


class LoadedGt(NamedTuple):
    frames: dict[FrameKey, GtFrame]
    names: list[str]
    images: dict[FrameKey, Path]


def load_gt(root: Path) -> LoadedGt:
    """Read YOLO labels from a dataset dir or a root of subdatasets, keeping reviewed frames."""
    reviewed = reviewed_frames(root)
    names: list[str] = []
    frames: dict[FrameKey, GtFrame] = {}
    images: dict[FrameKey, Path] = {}
    for dataset in _dataset_dirs(root):
        data = yaml.safe_load((dataset / "data.yaml").read_text())
        dataset_names = list(data["names"])
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
    return LoadedGt(frames, names, images)


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
