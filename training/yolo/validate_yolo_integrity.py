#!/usr/bin/env python3
"""Check that a YOLO dataset is internally consistent and safe to train on.

Runs a batch of read-only integrity checks and prints a report:

- data.yml fields valid for YOLO: names present and non-empty, nc (if given) matches
  len(names), kpt_shape/flip_idx well formed for pose sets. Extra fields (colors,
  source_mcap, path, ...) are ignored.
- Every annotation's class id is in range 0..len(names)-1 (the core check).
- Every row is a well formed detect box, pose row, or seg polygon, with normalized
  coordinates in [0, 1] and positive box sizes.
- Image/label pairing: labels that reference a missing image are flagged; images with
  no label file are counted as background negatives.
- Health signals: per-class instance counts, classes with zero instances, duplicate
  class names, empty label files.

Layouts supported: split dirs (train/ val/ test/, each with images/ + labels/) or a
single flat images/ + labels/ pair at the dataset root.

Exit code is 0 when no errors are found, 1 otherwise. With --strict, warnings also fail.

Usage:
    python validate_yolo_integrity.py training/data/nhrl_seg/nhrl_robots
    python validate_yolo_integrity.py training/data/synthetic --check-images --strict
"""

from __future__ import annotations

import argparse
import collections
from pathlib import Path

import yaml

IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp")
SPLIT_NAMES = ("train", "val", "test")

# Normalized coordinates should live in [0, 1]; allow a little slack for rounding.
COORD_EPS = 1e-3
# Keypoint visibility flags per the YOLO pose spec.
VISIBILITY_FLAGS = {0, 1, 2}
# Cap stored examples per issue so the report stays readable on large datasets.
MAX_EXAMPLES = 10


class Report:
    """Collects errors and warnings, keeping only the first few examples of each."""

    def __init__(self) -> None:
        self.errors: dict[str, int] = collections.Counter()
        self.warnings: dict[str, int] = collections.Counter()
        self.examples: dict[str, list[str]] = collections.defaultdict(list)

    def error(self, category: str, example: str | None = None) -> None:
        self.errors[category] += 1
        self._add_example(category, example)

    def warning(self, category: str, example: str | None = None) -> None:
        self.warnings[category] += 1
        self._add_example(category, example)

    def _add_example(self, category: str, example: str | None) -> None:
        if example is not None and len(self.examples[category]) < MAX_EXAMPLES:
            self.examples[category].append(example)

    def print_section(self, title: str, counts: dict[str, int]) -> None:
        if not counts:
            return
        print(f"\n{title}:")
        for category, count in counts.items():
            print(f"  [{count}] {category}")
            for example in self.examples[category]:
                print(f"      - {example}")


def load_names(meta: dict) -> list[str]:
    """Extract class names as a list, supporting the list and dict data.yml forms."""
    raw = meta.get("names", [])
    if isinstance(raw, list):
        return [str(v) for v in raw]
    if isinstance(raw, dict):
        parsed: dict[int, str] = {}
        for key, value in raw.items():
            try:
                parsed[int(key)] = str(value)
            except (TypeError, ValueError):
                continue
        return [parsed.get(i, f"class_{i}") for i in range(max(parsed) + 1)] if parsed else []
    return []


def find_data_yaml(root: Path) -> Path:
    for name in ("data.yml", "data.yaml"):
        if (root / name).exists():
            return root / name
    raise SystemExit(f"No data.yml/data.yaml in {root}")


def find_split_dirs(root: Path) -> list[tuple[str, Path, Path]]:
    """Return (split_name, images_dir, labels_dir) triples, or the flat root layout."""
    triples = []
    for split in SPLIT_NAMES:
        images, labels = root / split / "images", root / split / "labels"
        if labels.is_dir():
            triples.append((split, images, labels))
    if triples:
        return triples
    if (root / "labels").is_dir():
        return [(".", root / "images", root / "labels")]
    raise SystemExit(f"No train/val/test or images/labels layout under {root}")


def find_image_for_label(images_dir: Path, stem: str) -> Path | None:
    for ext in IMAGE_EXTENSIONS:
        for candidate in (images_dir / f"{stem}{ext}", images_dir / f"{stem}{ext.upper()}"):
            if candidate.exists():
                return candidate
    return None


def validate_yaml_fields(meta: dict, names: list[str], report: Report) -> None:
    """Validate the YOLO-relevant fields of data.yml; ignore extra fields."""
    if not names:
        report.error("data.yml has no usable 'names' field")
        return

    nc = meta.get("nc")
    if nc is None:
        report.warning("data.yml missing 'nc' field (recommended)")
    elif not isinstance(nc, int) or isinstance(nc, bool):
        report.error(f"data.yml 'nc' is not an integer: {nc!r}")
    elif nc != len(names):
        report.error(f"data.yml 'nc' ({nc}) != len(names) ({len(names)})")

    duplicates = [name for name, count in collections.Counter(names).items() if count > 1]
    if duplicates:
        report.warning(f"duplicate class names in data.yml: {sorted(duplicates)}")

    if not any(split in meta for split in SPLIT_NAMES):
        report.warning("data.yml declares no train/val/test split pointer")

    validate_kpt_shape(meta, report)


def validate_kpt_shape(meta: dict, report: Report) -> None:
    """Validate pose fields kpt_shape/flip_idx if present."""
    kpt_shape = meta.get("kpt_shape")
    if kpt_shape is None:
        if "flip_idx" in meta:
            report.warning("data.yml has 'flip_idx' without 'kpt_shape'")
        return

    if not (isinstance(kpt_shape, list) and len(kpt_shape) == 2):
        report.error(f"data.yml 'kpt_shape' must be [num_keypoints, dims]: {kpt_shape!r}")
        return
    num_kpts, dims = kpt_shape
    if not isinstance(num_kpts, int) or num_kpts < 1:
        report.error(f"data.yml 'kpt_shape' num_keypoints invalid: {num_kpts!r}")
    if dims not in (2, 3):
        report.error(f"data.yml 'kpt_shape' dims must be 2 or 3: {dims!r}")

    flip_idx = meta.get("flip_idx")
    if flip_idx is not None and isinstance(num_kpts, int):
        if not isinstance(flip_idx, list) or len(flip_idx) != num_kpts:
            report.error(f"data.yml 'flip_idx' length must equal num_keypoints ({num_kpts})")
        elif any(not isinstance(i, int) or i < 0 or i >= num_kpts for i in flip_idx):
            report.error(f"data.yml 'flip_idx' has out-of-range entries: {flip_idx!r}")


def coords_in_range(values: list[float]) -> bool:
    return all(-COORD_EPS <= v <= 1.0 + COORD_EPS for v in values)


def validate_row(
    class_id: int,
    coords: list[float],
    num_classes: int,
    kpt_shape: list | None,
    where: str,
    report: Report,
) -> None:
    """Validate one parsed annotation row (class id range, format, coordinate bounds)."""
    if class_id < 0 or class_id >= num_classes:
        report.error(f"class id out of range [0, {num_classes})", f"{where}: id={class_id}")
        return

    ncoord = len(coords)
    if ncoord < 4:
        report.error("row has fewer than 4 geometry values", where)
        return

    if kpt_shape is not None:
        validate_pose_row(coords, kpt_shape, where, report)
    elif ncoord == 4:
        validate_box(coords, where, report)
    elif ncoord >= 6 and ncoord % 2 == 0:
        validate_polygon(coords, where, report)
    else:
        report.error(f"row has {ncoord} geometry values (not a box, pose, or polygon)", where)


def validate_box(box: list[float], where: str, report: Report) -> None:
    if not coords_in_range(box):
        report.error("box coordinate outside [0, 1]", where)
    _, _, w, h = box
    if w <= 0 or h <= 0:
        report.error("box has non-positive width or height", where)


def validate_polygon(coords: list[float], where: str, report: Report) -> None:
    if len(coords) // 2 < 3:
        report.error("polygon has fewer than 3 points", where)
    if not coords_in_range(coords):
        report.error("polygon coordinate outside [0, 1]", where)


def validate_pose_row(coords: list[float], kpt_shape: list, where: str, report: Report) -> None:
    num_kpts, dims = int(kpt_shape[0]), int(kpt_shape[1])
    ncoord = len(coords)
    # A pose row is either a bare box (keypoints omitted) or box + num_kpts * dims.
    if ncoord != 4 and ncoord != 4 + num_kpts * dims:
        report.error(f"pose row has {ncoord} values (expected 4 or {4 + num_kpts * dims})", where)
        return

    validate_box(coords[:4], where, report)
    if ncoord == 4:
        return

    for k in range(num_kpts):
        base = 4 + k * dims
        if not coords_in_range(coords[base : base + 2]):
            report.error("keypoint coordinate outside [0, 1]", where)
        if dims == 3 and coords[base + 2] not in VISIBILITY_FLAGS:
            report.warning("keypoint visibility flag not in {0, 1, 2}", where)


def validate_labels(
    split_dirs: list[tuple[str, Path, Path]],
    num_classes: int,
    kpt_shape: list | None,
    report: Report,
) -> tuple[collections.Counter, int, int]:
    """Parse every label file. Returns (per-class counts, total rows, empty label files)."""
    counts: collections.Counter[int] = collections.Counter()
    total_rows = 0
    empty_files = 0

    for split, _images, labels in split_dirs:
        for label_path in sorted(labels.glob("*.txt")):
            text = label_path.read_text()
            rows = [line for line in text.splitlines() if line.strip()]
            if not rows:
                empty_files += 1
                continue
            for line_no, line in enumerate(rows, start=1):
                where = f"{split}/labels/{label_path.name}:{line_no}"
                parts = line.split()
                try:
                    class_id = int(parts[0])
                    coords = [float(v) for v in parts[1:]]
                except (ValueError, IndexError):
                    report.error("malformed row (non-numeric field)", where)
                    continue
                total_rows += 1
                counts[class_id] += 1
                validate_row(class_id, coords, num_classes, kpt_shape, where, report)

    return counts, total_rows, empty_files


def validate_pairing(
    split_dirs: list[tuple[str, Path, Path]], check_images: bool, report: Report
) -> tuple[int, int]:
    """Cross-check images against labels. Returns (labeled images, background images)."""
    labeled = 0
    background = 0

    for split, images, labels in split_dirs:
        label_stems = {p.stem for p in labels.glob("*.txt")}
        for label_path in sorted(labels.glob("*.txt")):
            image_path = find_image_for_label(images, label_path.stem)
            if image_path is None:
                where = f"{split}/labels/{label_path.name}"
                report.error("label file has no matching image", where)
                continue
            if check_images and image_path.stat().st_size == 0:
                report.error("image file is empty (0 bytes)", str(image_path))

        if not images.is_dir():
            report.warning(f"images dir missing for split '{split}'")
            continue
        for img_path in sorted(images.rglob("*")):
            if img_path.suffix.lower() not in IMAGE_EXTENSIONS:
                continue
            if img_path.stem in label_stems:
                labeled += 1
            else:
                background += 1

    return labeled, background


def report_class_health(counts: collections.Counter, names: list[str], report: Report) -> None:
    unused = [names[i] for i in range(len(names)) if counts[i] == 0]
    if unused:
        report.warning(f"classes with zero instances: {unused}")


def print_summary(
    root: Path,
    names: list[str],
    counts: collections.Counter,
    total_rows: int,
    empty_files: int,
    labeled: int,
    background: int,
) -> None:
    print(f"dataset: {root}")
    print(f"classes: {len(names)}")
    print(f"images:  {labeled} labeled, {background} background (no label)")
    print(f"labels:  {total_rows} annotations across labeled frames, {empty_files} empty files")
    print("instances per class:")
    for i, name in enumerate(names):
        print(f"  {i:>3} {name:<24} {counts[i]}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("dataset", type=Path, help="dataset root (has data.yml + splits)")
    parser.add_argument(
        "--check-images",
        action="store_true",
        help="also verify image files are non-empty (slower)",
    )
    parser.add_argument("--strict", action="store_true", help="treat warnings as failures too")
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    root: Path = args.dataset
    if not root.is_dir():
        raise SystemExit(f"Not a directory: {root}")

    meta = yaml.safe_load(find_data_yaml(root).read_text()) or {}
    names = load_names(meta)
    report = Report()

    validate_yaml_fields(meta, names, report)
    num_classes = len(names)
    kpt_shape = meta.get("kpt_shape") if isinstance(meta.get("kpt_shape"), list) else None

    split_dirs = find_split_dirs(root)
    counts, total_rows, empty_files = validate_labels(split_dirs, num_classes, kpt_shape, report)
    labeled, background = validate_pairing(split_dirs, args.check_images, report)
    report_class_health(counts, names, report)

    print_summary(root, names, counts, total_rows, empty_files, labeled, background)
    report.print_section("ERRORS", report.errors)
    report.print_section("WARNINGS", report.warnings)

    num_errors = sum(report.errors.values())
    num_warnings = sum(report.warnings.values())
    print(f"\nresult: {num_errors} error(s), {num_warnings} warning(s)")
    if num_errors or (args.strict and num_warnings):
        raise SystemExit(1)
    print("dataset looks healthy")


if __name__ == "__main__":
    main()
