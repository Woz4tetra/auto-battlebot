"""Remap label indices in a segmentation mask dataset.

Reads a TOML config that maps source label indices to target label indices,
then rewrites every *_mask.png in the dataset. Useful for converting a
multi-class segmask dataset (e.g. background=0, robot=1, distractor=2)
into a binary floor segmentation dataset (background=0, floor=1).

Usage:
    python remap_masks.py remap_config.toml /path/to/dataset --output /path/to/output

Example TOML config:
    # Source label -> target label.  Any source label not listed is
    # mapped to `default_label`.
    default_label = 0

    # Delete image+mask pairs that contain ANY of these labels.
    delete_labels = [3, 4]

    [label_map]
    0 = 1    # background -> floor
    1 = 0    # robot -> background
    2 = 0    # distractor -> background
"""

import argparse
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np
from tqdm import tqdm

import tomllib


def load_config(config_path: Path) -> tuple[dict[int, int], int, set[int]]:
    with open(config_path, "rb") as f:
        config = tomllib.load(f)

    default_label = config.get("default_label", 0)
    raw_map = config.get("label_map", {})
    delete_labels = set(int(v) for v in config.get("delete_labels", []))

    label_map: dict[int, int] = {}
    for src, dst in raw_map.items():
        label_map[int(src)] = int(dst)

    return label_map, default_label, delete_labels


def remap_mask(
    mask: np.ndarray, label_map: dict[int, int], default_label: int
) -> np.ndarray:
    remapped = np.full_like(mask, default_label)
    for src, dst in label_map.items():
        remapped[mask == src] = dst
    return remapped


def find_mask_files(dataset_dir: Path) -> list[Path]:
    masks = sorted(dataset_dir.rglob("*_mask.png"))
    return masks


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Remap label indices in segmentation mask PNGs"
    )
    parser.add_argument(
        "config",
        type=str,
        help="Path to TOML config file with label_map",
    )
    parser.add_argument(
        "dataset",
        type=str,
        help="Path to dataset directory (searches recursively for *_mask.png)",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default=None,
        help="Output directory (default: overwrite in place)",
    )
    parser.add_argument(
        "--copy-images",
        action="store_true",
        help="When using --output, also copy the corresponding .jpg images",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print what would be done without writing files",
    )
    args = parser.parse_args()

    config_path = Path(args.config)
    dataset_dir = Path(args.dataset)
    output_dir = Path(args.output) if args.output else None

    if not config_path.exists():
        print(f"Error: config file not found: {config_path}", file=sys.stderr)
        sys.exit(1)
    if not dataset_dir.is_dir():
        print(f"Error: dataset directory not found: {dataset_dir}", file=sys.stderr)
        sys.exit(1)

    label_map, default_label, delete_labels = load_config(config_path)
    print(f"Label map: {label_map}")
    print(f"Default label (unmapped sources): {default_label}")
    if delete_labels:
        print(f"Delete labels: {sorted(delete_labels)}")

    mask_files = find_mask_files(dataset_dir)
    if not mask_files:
        print(f"No *_mask.png files found in {dataset_dir}", file=sys.stderr)
        sys.exit(1)
    print(f"Found {len(mask_files)} mask files")

    deleted_count = 0
    written_count = 0

    for mask_path in tqdm(mask_files, desc="Remapping masks", disable=args.dry_run):
        mask = cv2.imread(str(mask_path), cv2.IMREAD_UNCHANGED)
        if mask is None:
            print(f"  WARNING: could not read {mask_path}, skipping")
            continue

        channel = mask[:, :, 0] if mask.ndim == 3 else mask
        unique_before = np.unique(channel)

        should_delete = delete_labels and bool(
            set(unique_before.tolist()) & delete_labels
        )

        if output_dir is not None:
            rel = mask_path.relative_to(dataset_dir)
            dest = output_dir / rel
        else:
            dest = mask_path

        if args.dry_run:
            if should_delete:
                action = "DELETE"
            else:
                remapped = remap_mask(channel, label_map, default_label)
                unique_after = np.unique(remapped)
                action = (
                    f"{unique_before.tolist()} -> {unique_after.tolist()} => {dest}"
                )
            print(f"  {mask_path.name}: {action}")
            continue

        if should_delete:
            deleted_count += 1
            if output_dir is None:
                mask_path.unlink()
                img_path = mask_path.parent / mask_path.name.replace(
                    "_mask.png", ".jpg"
                )
                if img_path.exists():
                    img_path.unlink()
            continue

        remapped = remap_mask(channel, label_map, default_label)
        dest.parent.mkdir(parents=True, exist_ok=True)

        if mask.ndim == 3:
            out = np.zeros_like(mask)
            out[:, :, 0] = remapped
            out[:, :, 1] = remapped
            out[:, :, 2] = remapped
        else:
            out = remapped

        cv2.imwrite(str(dest), out)
        written_count += 1

        if args.copy_images and output_dir is not None:
            img_name = mask_path.name.replace("_mask.png", ".jpg")
            img_path = mask_path.parent / img_name
            if img_path.exists():
                img_dest = dest.parent / img_name
                shutil.copy2(img_path, img_dest)

    if not args.dry_run:
        target = output_dir if output_dir else dataset_dir
        print(f"Done. Wrote {written_count} masks to {target}")
        if deleted_count:
            print(f"Deleted {deleted_count} image+mask pairs")


if __name__ == "__main__":
    main()
