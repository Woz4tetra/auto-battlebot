#!/usr/bin/env python3
"""
Merge multiple segmentation mask datasets into a single dataset.

Discovers all segmask datasets under a root directory (identified by a
validation_state.json alongside *_mask.png files), and copies only
image/mask pairs that pass validation (or are unvalidated).

If per-dataset data.yaml files are present (with 'names' and 'colors' keys),
their class lists are merged and mask pixel values are remapped to unified
IDs. Otherwise all datasets are assumed to share the same class IDs and
masks are copied without remapping.

An optional --classes YAML can be provided to supply class info for datasets
that don't have their own data.yaml, or to write a classes file to the output.

Output layout mirrors the flat format used by split_segmask_dataset.py:

    output_dir/
        <split>/
            image_001.jpg
            image_001_mask.png
            ...
        data.yaml   (written only when class info is available)

Usage:
    python3 merge_segmask_datasets.py \
        --search-dir /path/to/datasets \
        --output-dir /path/to/merged \
        [--split train]                  # default: train
        [--classes /path/to/classes.yaml]
        [--exclude-unvalidated]          # default: include unvalidated images
"""

import argparse
import json
import shutil
import yaml
import cv2
import numpy as np
from pathlib import Path


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp"}

DEFAULT_COLORS = [
    "red",
    "green",
    "blue",
    "yellow",
    "magenta",
    "cyan",
    "orange",
    "purple",
    "lime",
    "pink",
    "teal",
    "navy",
]


def find_segmask_datasets(search_dir: Path) -> list[Path]:
    """
    Recursively find all segmask dataset roots under search_dir.
    A dataset root is a directory that contains a validation_state.json AND
    at least one *_mask.png file anywhere beneath it.
    """
    seen: set[Path] = set()
    datasets = []
    for state_file in sorted(search_dir.rglob("validation_state.json")):
        root = state_file.parent
        if root in seen:
            continue
        if any(root.rglob("*_mask.png")):
            seen.add(root)
            datasets.append(root)
    return sorted(datasets)


def load_dataset_yaml(dataset_root: Path) -> dict:
    """Load data.yaml or data.yml from a dataset root, if present."""
    for name in ("data.yaml", "data.yml"):
        p = dataset_root / name
        if p.exists():
            with open(p) as f:
                return yaml.safe_load(f) or {}
    return {}


def find_image_mask_pairs(dataset_root: Path) -> list[tuple[Path, Path]]:
    """Find all (image, mask) pairs within a dataset, searching recursively."""
    pairs = []
    for img_path in sorted(dataset_root.rglob("*")):
        if img_path.suffix.lower() not in IMAGE_EXTENSIONS:
            continue
        if "_mask" in img_path.stem:
            continue
        mask_path = img_path.with_name(img_path.stem + "_mask.png")
        if mask_path.exists():
            pairs.append((img_path, mask_path))
    return pairs


def load_validation_state(dataset_root: Path) -> dict[str, str] | None:
    """
    Load validation_state.json if present. Keys are relative to dataset root.
    Returns None if the file does not exist.
    """
    state_file = dataset_root / "validation_state.json"
    if state_file.exists():
        with open(state_file) as f:
            return json.load(f)
    return None


def build_merged_class_list(datasets_info: list[dict]) -> tuple[list[str], list[str]]:
    """
    Build a merged (names, colors) pair from all datasets.
    Starts with the dataset that has the most classes, then appends any new
    names found in subsequent datasets (preserving insertion order).
    Colors are carried over from the source; new classes without a color get
    a default color.
    """
    sorted_info = sorted(datasets_info, key=lambda d: len(d["names"]), reverse=True)

    merged_names: list[str] = []
    merged_colors: list[str] = []
    seen: set[str] = set()
    color_pool_idx = 0

    for info in sorted_info:
        for name, color in zip(info["names"], info["colors"]):
            if name not in seen:
                merged_names.append(name)
                merged_colors.append(
                    color
                    if color
                    else DEFAULT_COLORS[color_pool_idx % len(DEFAULT_COLORS)]
                )
                if not color:
                    color_pool_idx += 1
                seen.add(name)

    return merged_names, merged_colors


def remap_mask(mask_arr: np.ndarray, old_id_to_new: dict[int, int]) -> np.ndarray:
    """Remap pixel values in a 2-D mask array according to old_id_to_new."""
    remapped = mask_arr.copy()
    for old_id, new_id in old_id_to_new.items():
        remapped[mask_arr == old_id] = new_id
    return remapped


def is_image_passing(
    img_path: Path,
    dataset_root: Path,
    validation_state: dict[str, str] | None,
    include_unvalidated: bool,
) -> bool:
    """Return True if an image should be included in the merged dataset."""
    if validation_state is None:
        return False

    rel_key = str(img_path.relative_to(dataset_root))
    status = validation_state.get(rel_key)
    if status == "fail":
        return False
    if status == "pass":
        return True
    return include_unvalidated


def safe_stem(dataset_root: Path, img_path: Path) -> str:
    """Build a collision-safe filename stem: <dataset_folder>__<original_stem>"""
    return f"{dataset_root.name}__{img_path.stem}"


def read_mask_channel(mask_path: Path) -> np.ndarray | None:
    """Read a *_mask.png and return the label channel as a 2-D uint8 array."""
    raw = cv2.imread(str(mask_path), cv2.IMREAD_UNCHANGED)
    if raw is None:
        return None
    return raw[:, :, 0] if raw.ndim == 3 else raw


def write_mask(arr: np.ndarray, dest: Path) -> None:
    """Write a 2-D label array as a grayscale PNG."""
    dest.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(dest), arr)


def main():
    parser = argparse.ArgumentParser(description="Merge multiple segmask datasets")
    parser.add_argument(
        "--search-dir",
        required=True,
        type=Path,
        help="Root directory to recursively search for segmask datasets",
    )
    parser.add_argument(
        "--output-dir",
        required=True,
        type=Path,
        help="Output directory for the merged dataset",
    )
    parser.add_argument(
        "--split",
        default="train",
        help="Output split name (default: train)",
    )
    parser.add_argument(
        "--classes",
        type=Path,
        default=None,
        help="Path to a YAML file with 'names' and 'colors' keys. Used as the "
        "class definition for datasets that have no data.yaml, and written "
        "to the output directory.",
    )
    parser.add_argument(
        "--exclude-unvalidated",
        action="store_true",
        help="Exclude images that have no entry in validation_state.json",
    )
    args = parser.parse_args()

    include_unvalidated = not args.exclude_unvalidated
    out_split = args.output_dir / args.split
    out_split.mkdir(parents=True, exist_ok=True)

    # Load global classes override if provided
    global_classes: dict = {}
    if args.classes:
        with open(args.classes) as f:
            global_classes = yaml.safe_load(f) or {}

    # ── 1. Discover datasets ──────────────────────────────────────────────────
    print(f"Searching for segmask datasets under: {args.search_dir}")
    dataset_roots = find_segmask_datasets(args.search_dir)
    print(f"Found {len(dataset_roots)} dataset(s)\n")

    if not dataset_roots:
        print("No datasets found. Exiting.")
        return

    # ── 2. Load metadata from each dataset ───────────────────────────────────
    datasets_info = []
    for root in dataset_roots:
        meta = load_dataset_yaml(root) or global_classes
        names = meta.get("names", [])
        colors = meta.get("colors", [])
        colors = list(colors) + [None] * max(0, len(names) - len(colors))
        datasets_info.append(
            {
                "root": root,
                "names": names,
                "colors": colors,
                "has_classes": bool(names),
            }
        )
        class_str = f"{len(names)} classes" if names else "no class info"
        print(f"  {root.name}: {class_str}")

    print()

    # ── 3. Build merged class list (only if any dataset has class info) ───────
    have_class_info = any(d["has_classes"] for d in datasets_info)
    merged_names: list[str] = []
    merged_colors: list[str] = []
    merged_name_to_id: dict[str, int] = {}

    if have_class_info:
        info_with_classes = [d for d in datasets_info if d["has_classes"]]
        merged_names, merged_colors = build_merged_class_list(info_with_classes)
        merged_name_to_id = {name: i for i, name in enumerate(merged_names)}
        print(f"Merged class list ({len(merged_names)} classes):")
        for i, (name, color) in enumerate(zip(merged_names, merged_colors)):
            print(f"  {i:3d}  {name}  ({color})")
    else:
        print("No class info found — masks will be copied without remapping.")
    print()

    # ── 4. Process each dataset ───────────────────────────────────────────────
    total_copied = 0
    total_skipped_fail = 0
    total_skipped_bad_mask = 0
    collision_count = 0

    for info in datasets_info:
        root = info["root"]
        names = info["names"]

        old_id_to_new: dict[int, int] | None = (
            {i: merged_name_to_id[name] for i, name in enumerate(names)}
            if (names and merged_name_to_id)
            else None
        )

        validation_state = load_validation_state(root)
        if validation_state is None:
            print(
                f"  WARNING: {root.name}: no validation_state.json — skipping all images"
            )

        pairs = find_image_mask_pairs(root)
        ds_copied = 0
        ds_fail = 0

        for img_path, mask_path in pairs:
            if not is_image_passing(
                img_path, root, validation_state, include_unvalidated
            ):
                ds_fail += 1
                continue

            stem = safe_stem(root, img_path)
            out_img = out_split / (stem + img_path.suffix)
            out_mask = out_split / (stem + "_mask.png")

            if out_img.exists():
                collision_count += 1
                stem = f"{stem}_{collision_count}"
                out_img = out_split / (stem + img_path.suffix)
                out_mask = out_split / (stem + "_mask.png")

            if old_id_to_new is not None:
                mask_arr = read_mask_channel(mask_path)
                if mask_arr is None:
                    print(f"  WARNING: could not read mask {mask_path}, skipping")
                    total_skipped_bad_mask += 1
                    continue
                write_mask(remap_mask(mask_arr, old_id_to_new), out_mask)
            else:
                shutil.copy2(mask_path, out_mask)

            shutil.copy2(img_path, out_img)
            ds_copied += 1

        print(f"  {root.name}: copied {ds_copied}, skipped (fail/no state) {ds_fail}")
        total_copied += ds_copied
        total_skipped_fail += ds_fail

    # ── 5. Write data.yaml if we have class info ──────────────────────────────
    if merged_names or global_classes:
        names_out = merged_names or global_classes.get("names", [])
        colors_out = merged_colors or global_classes.get("colors", [])
        yaml_data = {
            "nc": len(names_out),
            "names": names_out,
            "colors": colors_out,
        }
        yaml_path = args.output_dir / "data.yaml"
        with open(yaml_path, "w") as f:
            yaml.dump(yaml_data, f, default_flow_style=False, sort_keys=False)
        print(f"Wrote data.yaml -> {yaml_path}")

    print(f"\nTotal copied:              {total_copied}")
    print(f"Total skipped (fail):      {total_skipped_fail}")
    if total_skipped_bad_mask:
        print(f"Total skipped (bad mask):  {total_skipped_bad_mask}")
    if collision_count:
        print(f"Filename collisions resolved: {collision_count}")
    print("Done.")


if __name__ == "__main__":
    main()
