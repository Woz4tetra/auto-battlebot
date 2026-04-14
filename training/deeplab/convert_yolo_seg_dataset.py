#!/usr/bin/env python3
"""Convert a YOLO-seg dataset into a Deeplab-ready segmask dataset.

This script reads polygon annotations from YOLO-seg label files and writes
pixel-wise masks where each pixel contains a class ID.

Expected YOLO-seg layout:
    <input>/
        images/
            train/...
            val/...
            test/...   # optional
        labels/
            train/...
            val/...
            test/...   # optional
        data.yaml      # optional

Output layout (flat per split, compatible with semantic_train.py):
    <output>/
        train/
            sample_0001.jpg
            sample_0001_mask.png
            ...
        val/
            ...
        test/
            ...
        data.yaml      # optional class metadata
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import cv2
import numpy as np
import yaml
from tqdm import tqdm


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp"}
DEFAULT_SPLIT = "train"
VALID_SPLITS = {"train", "val", "test"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert a YOLO-seg dataset into a Deeplab segmask dataset"
    )
    parser.add_argument("input", type=Path, help="Path to YOLO-seg dataset root")
    parser.add_argument("output", type=Path, help="Path for output segmask dataset root")
    parser.add_argument(
        "--class-offset",
        type=int,
        default=1,
        help=(
            "Value added to each YOLO class ID when writing masks "
            "(default: 1, reserves 0 for background)"
        ),
    )
    parser.add_argument(
        "--binary",
        action="store_true",
        help="Write all foreground polygons as class 1 (binary segmentation)",
    )
    parser.add_argument(
        "--include-test",
        action="store_true",
        help="Also convert the test split if present",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Delete output directory before writing",
    )
    return parser.parse_args()


def load_data_yaml(dataset_root: Path) -> dict:
    for name in ("data.yaml", "data.yml"):
        path = dataset_root / name
        if path.exists():
            with open(path, "r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
    return {}


def parse_yolo_seg_rows(label_path: Path) -> list[tuple[int, list[tuple[float, float]]]]:
    rows: list[tuple[int, list[tuple[float, float]]]] = []
    text = label_path.read_text(encoding="utf-8")
    for line in text.splitlines():
        parts = line.strip().split()
        if not parts:
            continue
        try:
            class_id = int(float(parts[0]))
            values = [float(v) for v in parts[1:]]
        except ValueError:
            continue

        if len(values) < 6 or len(values) % 2 != 0:
            continue

        polygon = [(values[i], values[i + 1]) for i in range(0, len(values), 2)]
        if len(polygon) < 3:
            continue
        rows.append((class_id, polygon))
    return rows


def polygon_area(points: np.ndarray) -> float:
    if len(points) < 3:
        return 0.0
    return float(abs(cv2.contourArea(points.astype(np.float32))))


def polygon_interior_point(points: np.ndarray) -> tuple[float, float]:
    moments = cv2.moments(points.astype(np.float32))
    if abs(moments["m00"]) > 1e-8:
        return (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])
    p = points[0]
    return (float(p[0]), float(p[1]))


def compute_nesting_depths(polygons: list[np.ndarray]) -> list[int]:
    if not polygons:
        return []

    areas = [polygon_area(poly) for poly in polygons]
    parents: list[int] = [-1] * len(polygons)

    for idx, poly in enumerate(polygons):
        point = polygon_interior_point(poly)
        parent_idx = -1
        parent_area = float("inf")
        for candidate_idx, candidate in enumerate(polygons):
            if candidate_idx == idx:
                continue
            if areas[candidate_idx] <= areas[idx]:
                continue
            test = cv2.pointPolygonTest(
                candidate.reshape((-1, 1, 2)).astype(np.float32), point, False
            )
            if test < 0:
                continue
            if areas[candidate_idx] < parent_area:
                parent_idx = candidate_idx
                parent_area = areas[candidate_idx]
        parents[idx] = parent_idx

    depths: list[int] = [0] * len(polygons)
    for idx in range(len(polygons)):
        depth = 0
        current = parents[idx]
        visited: set[int] = set()
        while current != -1 and current not in visited:
            visited.add(current)
            depth += 1
            current = parents[current]
        depths[idx] = depth
    return depths


def rasterize_mask(
    image_shape: tuple[int, int],
    rows: list[tuple[int, list[tuple[float, float]]]],
    class_offset: int,
    binary: bool,
) -> np.ndarray:
    h, w = image_shape
    mask = np.zeros((h, w), dtype=np.uint8)

    polygons_by_class: dict[int, list[np.ndarray]] = {}
    for class_id, polygon in rows:
        pts = np.array(
            [
                (
                    int(round(np.clip(x_norm, 0.0, 1.0) * (w - 1))),
                    int(round(np.clip(y_norm, 0.0, 1.0) * (h - 1))),
                )
                for x_norm, y_norm in polygon
            ],
            dtype=np.int32,
        )
        if len(pts) < 3:
            continue
        polygons_by_class.setdefault(class_id, []).append(pts)

    for class_id, polygons in polygons_by_class.items():
        if not polygons:
            continue
        depths = compute_nesting_depths(polygons)
        fill_value = 1 if binary else class_id + class_offset
        for polygon, depth in zip(polygons, depths):
            value = 0 if depth % 2 == 1 else int(fill_value)
            cv2.fillPoly(mask, [polygon], value)
    return mask


def build_output_stem(split_rel: Path, seen: set[str]) -> str:
    stem = str(split_rel.with_suffix("")).replace("/", "__")
    if stem not in seen:
        seen.add(stem)
        return stem

    idx = 1
    while f"{stem}_{idx}" in seen:
        idx += 1
    candidate = f"{stem}_{idx}"
    seen.add(candidate)
    return candidate


def image_label_pairs(dataset_root: Path) -> list[tuple[str, Path, Path, Path]]:
    images_dir = dataset_root / "images"
    labels_dir = dataset_root / "labels"
    if not images_dir.exists() or not labels_dir.exists():
        raise FileNotFoundError(f"Expected {images_dir} and {labels_dir} to exist")

    pairs: list[tuple[str, Path, Path, Path]] = []
    for img_path in sorted(images_dir.rglob("*")):
        if img_path.suffix.lower() not in IMAGE_EXTENSIONS:
            continue

        rel = img_path.relative_to(images_dir)
        label_path = (labels_dir / rel).with_suffix(".txt")
        if not label_path.exists():
            continue

        split = DEFAULT_SPLIT
        split_rel = rel
        if rel.parts and rel.parts[0] in VALID_SPLITS:
            split = rel.parts[0]
            split_rel = Path(*rel.parts[1:]) if len(rel.parts) > 1 else Path(rel.name)

        pairs.append((split, split_rel, img_path, label_path))
    return pairs


def write_output_data_yaml(
    output_root: Path, input_yaml: dict, class_offset: int, binary: bool
) -> None:
    names = input_yaml.get("names", [])
    colors = input_yaml.get("colors", [])
    if not isinstance(names, list):
        return
    if not isinstance(colors, list):
        colors = []

    if binary:
        out_yaml = {
            "nc": 2,
            "names": ["background", "foreground"],
            "colors": ["black", "red"],
        }
    elif class_offset == 1:
        shifted_names = ["background", *[str(n) for n in names]]
        shifted_colors = ["black", *[str(c) for c in colors]]
        out_yaml = {"nc": len(shifted_names), "names": shifted_names}
        if shifted_colors:
            out_yaml["colors"] = shifted_colors
    else:
        out_yaml = {"nc": len(names), "names": [str(n) for n in names]}
        if colors:
            out_yaml["colors"] = [str(c) for c in colors]

    yaml_path = output_root / "data.yaml"
    with open(yaml_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(out_yaml, f, sort_keys=False)


def main() -> None:
    args = parse_args()
    input_root: Path = args.input
    output_root: Path = args.output

    if not input_root.exists():
        raise FileNotFoundError(f"Input dataset not found: {input_root}")

    if output_root.exists() and args.overwrite:
        shutil.rmtree(output_root)
    output_root.mkdir(parents=True, exist_ok=True)

    pairs = image_label_pairs(input_root)
    if not pairs:
        raise RuntimeError("No image/label pairs found in input dataset")

    selected_splits = {"train", "val"}
    if args.include_test:
        selected_splits.add("test")

    seen_by_split: dict[str, set[str]] = {}
    converted = 0
    skipped_missing = 0
    skipped_split = 0

    progress = tqdm(pairs, desc="Converting", unit="image", dynamic_ncols=True)
    for split, split_rel, img_path, label_path in progress:
        if split not in selected_splits:
            skipped_split += 1
            continue

        image = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
        if image is None:
            skipped_missing += 1
            continue
        h, w = image.shape[:2]

        rows = parse_yolo_seg_rows(label_path)
        mask = rasterize_mask(
            (h, w),
            rows=rows,
            class_offset=args.class_offset,
            binary=args.binary,
        )

        split_out = output_root / split
        split_out.mkdir(parents=True, exist_ok=True)

        if split not in seen_by_split:
            seen_by_split[split] = set()
        out_stem = build_output_stem(split_rel, seen_by_split[split])

        out_image_path = split_out / f"{out_stem}{img_path.suffix.lower()}"
        out_mask_path = split_out / f"{out_stem}_mask.png"
        shutil.copy2(img_path, out_image_path)
        cv2.imwrite(str(out_mask_path), mask)
        converted += 1
        progress.set_postfix(split=split, converted=converted)

    input_yaml = load_data_yaml(input_root)
    if input_yaml:
        write_output_data_yaml(
            output_root, input_yaml, class_offset=args.class_offset, binary=args.binary
        )

    print(f"Converted: {converted}")
    print(f"Skipped (unselected split): {skipped_split}")
    print(f"Skipped (missing image): {skipped_missing}")
    print(f"Output written to: {output_root}")


if __name__ == "__main__":
    main()
