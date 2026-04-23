#!/usr/bin/env python3
"""
Convert a Roboflow COCO export into a YOLO keypoints dataset.

Expected COCO structure:
  - <root>/train/_annotations.coco.json (and optionally valid/test)
  - image files inside each split directory

Usage:
  python3 playground/roboflow_coco_to_yolo_keypoints.py \
    --input /home/ben/Downloads/auto-battlebots.coco \
    --output /home/ben/Downloads/auto-battlebots.yolo
"""

from __future__ import annotations

import argparse
import json
import shutil
from collections import defaultdict
from pathlib import Path


def _to_float(value: object) -> float:
    """Convert JSON numeric values (including stringified floats) to float."""
    return float(value)  # type: ignore[arg-type]


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def discover_splits(input_dir: Path, requested_splits: list[str]) -> dict[str, tuple[Path, Path]]:
    split_paths: dict[str, tuple[Path, Path]] = {}
    for split in requested_splits:
        ann_path = input_dir / split / "_annotations.coco.json"
        if ann_path.exists():
            split_paths[split] = (ann_path, input_dir / split)

    if split_paths:
        return split_paths

    # Allow passing a single split folder directly (e.g. .../train).
    root_ann = input_dir / "_annotations.coco.json"
    if root_ann.exists():
        split_name = input_dir.name.lower()
        if split_name not in {"train", "valid", "test"}:
            split_name = "train"
        split_paths[split_name] = (root_ann, input_dir)
        return split_paths

    raise FileNotFoundError(
        f"Could not find _annotations.coco.json under {input_dir}. "
        "Pass either a COCO root dir (with train/valid/test) or a split dir."
    )


def build_class_mapping(categories: list[dict], keep_classes: list[str] | None) -> tuple[dict[int, int], list[str]]:
    # Roboflow COCO exports category ids; keep deterministic order by category id.
    categories_sorted = sorted(categories, key=lambda c: int(c["id"]))

    if keep_classes:
        keep = set(keep_classes)
        filtered = [c for c in categories_sorted if c["name"] in keep]
        missing = [name for name in keep_classes if name not in {c["name"] for c in filtered}]
        if missing:
            available = [c["name"] for c in categories_sorted]
            raise ValueError(
                f"Requested classes not found: {missing}. Available classes: {available}"
            )
        class_names = keep_classes
        old_to_new = {
            int(c["id"]): class_names.index(c["name"])
            for c in filtered
        }
        return old_to_new, class_names

    class_names = [c["name"] for c in categories_sorted]
    old_to_new = {int(c["id"]): idx for idx, c in enumerate(categories_sorted)}
    return old_to_new, class_names


def infer_num_keypoints(annotations: list[dict], fallback: int | None) -> int:
    for ann in annotations:
        keypoints = ann.get("keypoints")
        if isinstance(keypoints, list) and keypoints:
            if len(keypoints) % 3 != 0:
                raise ValueError(
                    f"Invalid keypoints length {len(keypoints)} in annotation id={ann.get('id')}."
                )
            return len(keypoints) // 3

    if fallback is not None:
        return fallback

    raise ValueError(
        "Could not infer keypoint count (no annotation had keypoints). "
        "Pass --num-keypoints explicitly."
    )


def convert_split(
    ann_path: Path,
    images_src_dir: Path,
    images_dst_dir: Path,
    labels_dst_dir: Path,
    old_to_new_class: dict[int, int],
    num_keypoints: int,
) -> tuple[int, int, int]:
    data = json.loads(ann_path.read_text())

    images_by_id = {int(img["id"]): img for img in data["images"]}
    anns_by_image: dict[int, list[dict]] = defaultdict(list)

    for ann in data["annotations"]:
        cat_id = int(ann["category_id"])
        if cat_id in old_to_new_class:
            anns_by_image[int(ann["image_id"])].append(ann)

    images_dst_dir.mkdir(parents=True, exist_ok=True)
    labels_dst_dir.mkdir(parents=True, exist_ok=True)

    processed_images = 0
    skipped_images = 0
    written_labels = 0

    for image_id, anns in anns_by_image.items():
        image = images_by_id.get(image_id)
        if image is None:
            continue

        image_name = image["file_name"]
        src_image_path = images_src_dir / image_name
        if not src_image_path.exists():
            skipped_images += 1
            continue

        image_w = _to_float(image["width"])
        image_h = _to_float(image["height"])

        lines: list[str] = []
        expected_kpt_values = num_keypoints * 3

        for ann in anns:
            class_id = old_to_new_class[int(ann["category_id"])]

            # COCO bbox format: x_min, y_min, width, height
            x_min, y_min, box_w, box_h = (_to_float(v) for v in ann["bbox"])
            x_center = _clamp01((x_min + (box_w / 2.0)) / image_w)
            y_center = _clamp01((y_min + (box_h / 2.0)) / image_h)
            box_w_n = _clamp01(box_w / image_w)
            box_h_n = _clamp01(box_h / image_h)

            values: list[str] = [
                str(class_id),
                f"{x_center:.6f}",
                f"{y_center:.6f}",
                f"{box_w_n:.6f}",
                f"{box_h_n:.6f}",
            ]

            kpts = ann.get("keypoints", [])
            if not isinstance(kpts, list):
                kpts = []

            if len(kpts) < expected_kpt_values:
                kpts = list(kpts) + [0.0] * (expected_kpt_values - len(kpts))
            elif len(kpts) > expected_kpt_values:
                kpts = kpts[:expected_kpt_values]

            for i in range(0, expected_kpt_values, 3):
                kx = _clamp01(_to_float(kpts[i]) / image_w)
                ky = _clamp01(_to_float(kpts[i + 1]) / image_h)
                kv = int(_to_float(kpts[i + 2]))
                values.extend((f"{kx:.6f}", f"{ky:.6f}", str(kv)))

            lines.append(" ".join(values))

        if not lines:
            continue

        shutil.copy2(src_image_path, images_dst_dir / image_name)
        label_path = labels_dst_dir / f"{Path(image_name).stem}.txt"
        label_path.write_text("\n".join(lines) + "\n")

        processed_images += 1
        written_labels += len(lines)

    return processed_images, written_labels, skipped_images


def write_data_yaml(output_dir: Path, class_names: list[str], num_keypoints: int, split_map: dict[str, str]) -> None:
    # YOLO expects "val" key in yaml; map Roboflow's "valid" split to it.
    yaml_lines = [f"path: {output_dir.resolve()}"]
    if "train" in split_map:
        yaml_lines.append(f"train: {split_map['train']}")
    if "valid" in split_map:
        yaml_lines.append(f"val: {split_map['valid']}")
    elif "val" in split_map:
        yaml_lines.append(f"val: {split_map['val']}")
    if "test" in split_map:
        yaml_lines.append(f"test: {split_map['test']}")
    yaml_lines.append(f"nc: {len(class_names)}")
    yaml_lines.append(f"names: {class_names}")
    yaml_lines.append(f"kpt_shape: [{num_keypoints}, 3]")
    (output_dir / "data.yaml").write_text("\n".join(yaml_lines) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert Roboflow COCO export to YOLO keypoints dataset."
    )
    parser.add_argument("--input", required=True, help="COCO export root or split directory")
    parser.add_argument("--output", required=True, help="Output YOLO dataset directory")
    parser.add_argument(
        "--splits",
        nargs="+",
        default=["train", "valid", "test"],
        help="Split names to look for under --input (ignored for single split input)",
    )
    parser.add_argument(
        "--classes",
        nargs="+",
        default=None,
        help="Optional class names to keep, in desired YOLO class-id order",
    )
    parser.add_argument(
        "--num-keypoints",
        type=int,
        default=None,
        help="Expected keypoints per object. If omitted, inferred from annotations.",
    )
    args = parser.parse_args()

    input_dir = Path(args.input).expanduser().resolve()
    output_dir = Path(args.output).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    split_paths = discover_splits(input_dir, args.splits)
    first_ann = json.loads(next(iter(split_paths.values()))[0].read_text())
    old_to_new_class, class_names = build_class_mapping(first_ann["categories"], args.classes)
    num_keypoints = infer_num_keypoints(first_ann["annotations"], args.num_keypoints)

    print(f"Input:          {input_dir}")
    print(f"Output:         {output_dir}")
    print(f"Classes ({len(class_names)}): {class_names}")
    print(f"Num keypoints:  {num_keypoints}")
    print(f"Splits:         {list(split_paths.keys())}")
    print()

    yaml_split_map: dict[str, str] = {}
    total_images = 0
    total_labels = 0
    total_skipped = 0

    for split_name, (ann_path, images_src_dir) in split_paths.items():
        print(f"Converting split '{split_name}' from {ann_path}")
        images_dst = output_dir / "images" / split_name
        labels_dst = output_dir / "labels" / split_name

        n_images, n_labels, n_skipped = convert_split(
            ann_path=ann_path,
            images_src_dir=images_src_dir,
            images_dst_dir=images_dst,
            labels_dst_dir=labels_dst,
            old_to_new_class=old_to_new_class,
            num_keypoints=num_keypoints,
        )
        print(f"  -> images: {n_images}, labels: {n_labels}, missing images: {n_skipped}")

        if n_images > 0:
            yaml_split_map[split_name] = f"images/{split_name}"
        total_images += n_images
        total_labels += n_labels
        total_skipped += n_skipped

    write_data_yaml(output_dir, class_names, num_keypoints, yaml_split_map)
    print()
    print(f"Wrote {output_dir / 'data.yaml'}")
    print(f"Total images: {total_images}, labels: {total_labels}, missing images: {total_skipped}")


if __name__ == "__main__":
    main()
