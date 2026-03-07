#!/usr/bin/env python3
"""
Convert a COCO keypoint dataset to YOLO format, filtering to specific categories.

Usage:
    python3 coco_to_yolo.py \
        --input-dir "/home/ben/Downloads/True Battlebots Keypoints.coco" \
        --output-dir "/home/ben/Downloads/battlebots_yolo" \
        --categories mr_stabs_mk2 referee

Output YOLO label format (with keypoints):
    class_id cx cy w h kp1_x kp1_y kp1_v kp2_x kp2_y kp2_v
All values are normalized to [0, 1] except visibility flags.
"""

import argparse
import json
import os
import shutil
from collections import defaultdict
from pathlib import Path


def convert_split(
    ann_path: Path,
    images_src_dir: Path,
    images_dst_dir: Path,
    labels_dst_dir: Path,
    keep_categories: list[str],
    num_keypoints: int,
):
    with open(ann_path) as f:
        data = json.load(f)

    # Build category name -> new class id mapping (only kept categories)
    keep_set = set(keep_categories)
    old_id_to_new = {}
    for cat in data["categories"]:
        if cat["name"] in keep_set:
            old_id_to_new[cat["id"]] = keep_categories.index(cat["name"])

    if not old_id_to_new:
        raise ValueError(
            f"None of the requested categories {keep_categories} were found in the dataset. "
            f"Available: {[c['name'] for c in data['categories']]}"
        )

    # Build image_id -> image info
    images_by_id = {img["id"]: img for img in data["images"]}

    # Group annotations by image_id
    anns_by_image = defaultdict(list)
    for ann in data["annotations"]:
        if ann["category_id"] in old_id_to_new:
            anns_by_image[ann["image_id"]].append(ann)

    images_dst_dir.mkdir(parents=True, exist_ok=True)
    labels_dst_dir.mkdir(parents=True, exist_ok=True)

    skipped_images = 0
    processed_images = 0
    total_labels = 0

    for image_id, anns in anns_by_image.items():
        img_info = images_by_id[image_id]
        img_w = float(img_info["width"])
        img_h = float(img_info["height"])
        filename = img_info["file_name"]

        src_image = images_src_dir / filename
        if not src_image.exists():
            skipped_images += 1
            continue

        label_lines = []
        for ann in anns:
            new_class_id = old_id_to_new[ann["category_id"]]

            # COCO bbox: [x_min, y_min, width, height]
            bx, by, bw, bh = (float(v) for v in ann["bbox"])
            cx = (bx + bw / 2) / img_w
            cy = (by + bh / 2) / img_h
            nw = bw / img_w
            nh = bh / img_h

            parts = [new_class_id, cx, cy, nw, nh]

            # Keypoints: [x1, y1, v1, x2, y2, v2, ...]
            kps = ann.get("keypoints", [])
            expected_kp_values = num_keypoints * 3
            if kps:
                # Pad or truncate to the expected length
                if len(kps) < expected_kp_values:
                    kps = list(kps) + [0.0] * (expected_kp_values - len(kps))
                else:
                    kps = kps[:expected_kp_values]

                # Swap keypoint 0 and keypoint 1 for mr_stabs_mk2 only
                if (
                    num_keypoints >= 2
                    and keep_categories[new_class_id] == "mr_stabs_mk2"
                ):
                    kps[0:3], kps[3:6] = list(kps[3:6]), list(kps[0:3])

                for i in range(0, expected_kp_values, 3):
                    kx = float(kps[i]) / img_w
                    ky = float(kps[i + 1]) / img_h
                    kv = int(kps[i + 2])
                    parts += [kx, ky, kv]

            label_lines.append(
                " ".join(f"{v:.6f}" if isinstance(v, float) else str(v) for v in parts)
            )

        if not label_lines:
            continue

        # Copy image
        dst_image = images_dst_dir / filename
        if not dst_image.exists():
            shutil.copy2(src_image, dst_image)

        # Write label file
        stem = Path(filename).stem
        label_file = labels_dst_dir / f"{stem}.txt"
        label_file.write_text("\n".join(label_lines) + "\n")

        processed_images += 1
        total_labels += len(label_lines)

    print(f"  Processed: {processed_images} images, {total_labels} labels")
    if skipped_images:
        print(f"  Skipped (image not found): {skipped_images}")

    return processed_images


def main():
    parser = argparse.ArgumentParser(
        description="Convert COCO keypoint dataset to YOLO format"
    )
    parser.add_argument(
        "--input-dir",
        default="/home/ben/Downloads/True Battlebots Keypoints.coco",
        help="Root directory of the COCO dataset",
    )
    parser.add_argument(
        "--output-dir",
        default="/home/ben/Downloads/battlebots_yolo",
        help="Output directory for the YOLO dataset",
    )
    parser.add_argument(
        "--categories",
        nargs="+",
        default=["mr_stabs_mk2", "referee"],
        help="Category names to keep, in desired class-id order",
    )
    parser.add_argument(
        "--num-keypoints",
        type=int,
        default=2,
        help="Number of keypoints per annotation",
    )
    args = parser.parse_args()

    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)
    keep_categories = args.categories

    print(f"Input:      {input_dir}")
    print(f"Output:     {output_dir}")
    print(f"Categories: {keep_categories}")
    print(f"Keypoints:  {args.num_keypoints}")
    print()

    splits = ["train", "valid", "test"]
    found_splits = []
    for split in splits:
        ann_path = input_dir / split / "_annotations.coco.json"
        if ann_path.exists():
            found_splits.append(split)

    if not found_splits:
        # Try root-level annotations
        ann_path = input_dir / "_annotations.coco.json"
        if ann_path.exists():
            found_splits = ["train"]
        else:
            raise FileNotFoundError(f"No annotation files found under {input_dir}")

    split_paths = {}
    for split in found_splits:
        ann = input_dir / split / "_annotations.coco.json"
        if ann.exists():
            split_paths[split] = (ann, input_dir / split)
        else:
            split_paths[split] = (input_dir / "_annotations.coco.json", input_dir)

    yaml_splits = {}
    for split, (ann_path, images_src_dir) in split_paths.items():
        print(f"Converting split: {split}")
        images_dst = output_dir / "images" / split
        labels_dst = output_dir / "labels" / split
        n = convert_split(
            ann_path,
            images_src_dir,
            images_dst,
            labels_dst,
            keep_categories,
            args.num_keypoints,
        )
        if n > 0:
            yaml_splits[split] = f"images/{split}"

    # Write data.yaml
    yaml_path = output_dir / "data.yaml"
    kp_shape = f"[{args.num_keypoints}, 3]"
    lines = []
    lines.append(f"path: {output_dir.resolve()}")
    for split in ["train", "valid", "test"]:
        if split in yaml_splits:
            lines.append(f"{split}: {yaml_splits[split]}")
    lines.append(f"nc: {len(keep_categories)}")
    lines.append(f"names: {keep_categories}")
    lines.append(f"kpt_shape: {kp_shape}")
    yaml_path.write_text("\n".join(lines) + "\n")
    print(f"\nWrote data.yaml -> {yaml_path}")
    print("Done.")


if __name__ == "__main__":
    main()
