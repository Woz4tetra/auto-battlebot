#!/usr/bin/env python3
"""
Merge multiple YOLO datasets into a single dataset.

Discovers all YOLO datasets under a root directory (identified by a data.yaml/data.yml
alongside images/ and labels/ directories), merges their class lists, remaps annotation
class IDs, and copies only images/labels that pass validation (or are unvalidated).

Usage:
    python3 merge_yolo_datasets.py \
        --search-dir /home/ben/auto-battlebot/training/data/exported_bounding_boxes \
        --output-dir /home/ben/auto-battlebot/training/data/merged \
        [--include-unvalidated]   # default: include unvalidated images
        [--split train]           # output split name (default: train)
"""

import argparse
import json
import multiprocessing as mp
import shutil
import yaml
from pathlib import Path

from tqdm import tqdm


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp"}


def find_yolo_datasets(search_dir: Path) -> list[Path]:
    """Recursively find all YOLO dataset roots under search_dir."""
    seen: set[Path] = set()
    datasets = []
    for pattern in ("data.yaml", "data.yml"):
        for yaml_path in sorted(search_dir.rglob(pattern)):
            root = yaml_path.parent
            if root in seen:
                continue
            if (root / "images").exists() and (root / "labels").exists():
                seen.add(root)
                datasets.append(root)
    return sorted(datasets)


def load_dataset_yaml(dataset_root: Path) -> dict:
    """Load data.yaml or data.yml from a dataset root."""
    for name in ("data.yaml", "data.yml"):
        p = dataset_root / name
        if p.exists():
            with open(p) as f:
                return yaml.safe_load(f) or {}
    return {}


def extract_names(meta: dict) -> list[str]:
    """
    Extract class names from dataset metadata.

    Supports common YOLO forms:
      - names: [class_a, class_b, ...]
      - names: {0: class_a, 1: class_b, ...}
      - names: {"0": class_a, "1": class_b, ...}
    """
    raw_names = meta.get("names", [])

    if isinstance(raw_names, list):
        return [str(v) for v in raw_names]

    if isinstance(raw_names, dict):
        parsed: dict[int, str] = {}
        for k, v in raw_names.items():
            try:
                idx = int(k)
            except (TypeError, ValueError):
                continue
            parsed[idx] = str(v)
        if not parsed:
            return []
        max_idx = max(parsed)
        return [parsed.get(i, f"class_{i}") for i in range(max_idx + 1)]

    return []


def find_image_label_pairs(dataset_root: Path) -> list[tuple[Path, Path]]:
    """Find all (image, label) pairs within a dataset, searching recursively."""
    images_dir = dataset_root / "images"
    labels_dir = dataset_root / "labels"
    pairs = []

    for img_path in sorted(images_dir.rglob("*")):
        if img_path.suffix.lower() not in IMAGE_EXTENSIONS:
            continue
        # Mirror the sub-path from images/ into labels/
        rel = img_path.relative_to(images_dir)
        label_path = (labels_dir / rel).with_suffix(".txt")
        if label_path.exists():
            pairs.append((img_path, label_path))

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


def build_merged_class_list(datasets_info: list[dict]) -> list[str]:
    """
    Build a merged class list from all datasets.
    Starts with the dataset that has the most classes, then appends
    any new names found in subsequent datasets (preserving insertion order).
    """
    # Sort datasets by number of classes descending so the largest set comes first
    sorted_info = sorted(datasets_info, key=lambda d: len(d["names"]), reverse=True)

    merged: list[str] = []
    seen: set[str] = set()
    for info in sorted_info:
        for name in info["names"]:
            if name not in seen:
                merged.append(name)
                seen.add(name)
    return merged


def remap_label_line(line: str, old_id_to_new: dict[int, int]) -> str | None:
    """
    Remap the class ID in a YOLO label line.
    Returns None if the class is not in old_id_to_new (should not happen in normal use).
    """
    parts = line.strip().split()
    if not parts:
        return None
    try:
        old_id = int(parts[0])
    except ValueError:
        return None
    new_id = old_id_to_new.get(old_id)
    if new_id is None:
        return None
    parts[0] = str(new_id)
    return " ".join(parts)


def process_pair_task(task: dict) -> tuple[bool, bool]:
    """
    Process one (image, label) pair task.

    Returns:
      (copied, skipped_no_label)
    """
    img_path = Path(task["img_path"])
    label_path = Path(task["label_path"])
    out_img = Path(task["out_img"])
    out_lbl = Path(task["out_lbl"])
    old_id_to_new = task["old_id_to_new"]

    remapped_lines = []
    with open(label_path) as f:
        for line in f:
            remapped = remap_label_line(line, old_id_to_new)
            if remapped is not None:
                remapped_lines.append(remapped)

    if not remapped_lines:
        return False, True

    out_img.parent.mkdir(parents=True, exist_ok=True)
    out_lbl.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(img_path, out_img)
    out_lbl.write_text("\n".join(remapped_lines) + "\n")
    return True, False


def is_image_passing(
    img_path: Path,
    dataset_root: Path,
    validation_state: dict[str, str] | None,
    include_unvalidated: bool,
) -> bool:
    """Return True if an image should be included in the merged dataset."""
    # None means no validation_state.json exists — skip everything
    if validation_state is None:
        return False

    # Key format used by validate_yolo_dataset.py: relative to dataset root
    rel_key = str(img_path.relative_to(dataset_root))

    status = validation_state.get(rel_key)
    if status == "fail":
        return False
    if status == "pass":
        return True
    # Not in state file = unvalidated
    return include_unvalidated


def safe_stem(dataset_root: Path, img_path: Path) -> str:
    """
    Build a collision-safe filename stem: <dataset_folder>__<original_stem>
    """
    return f"{dataset_root.name}__{img_path.stem}"


def main():
    parser = argparse.ArgumentParser(description="Merge multiple YOLO datasets")
    parser.add_argument(
        "--search-dir",
        required=True,
        type=Path,
        help="Root directory to recursively search for YOLO datasets",
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
        "--exclude-unvalidated",
        action="store_true",
        help="Exclude images that have no entry in validation_state.json",
    )
    parser.add_argument(
        "-j",
        "--jobs",
        type=int,
        default=0,
        help="Worker processes for pair processing (default: CPU count)",
    )
    args = parser.parse_args()

    include_unvalidated = not args.exclude_unvalidated
    jobs = args.jobs if args.jobs > 0 else (mp.cpu_count() or 1)
    jobs = max(1, jobs)
    out_images = args.output_dir / "images" / args.split
    out_labels = args.output_dir / "labels" / args.split
    out_images.mkdir(parents=True, exist_ok=True)
    out_labels.mkdir(parents=True, exist_ok=True)

    # ── 1. Discover datasets ──────────────────────────────────────────────────
    print(f"Searching for YOLO datasets under: {args.search_dir}")
    dataset_roots = find_yolo_datasets(args.search_dir)
    print(f"Found {len(dataset_roots)} dataset(s)\n")

    if not dataset_roots:
        print("No datasets found. Exiting.")
        return

    # ── 2. Load metadata from each dataset ───────────────────────────────────
    datasets_info = []
    for root in dataset_roots:
        meta = load_dataset_yaml(root)
        names = extract_names(meta)
        if not names:
            print(f"  [SKIP] {root.name}: no 'names' field in data yaml")
            continue
        datasets_info.append(
            {
                "root": root,
                "names": names,
                "nc": len(names),
                "kpt_shape": meta.get("kpt_shape"),
            }
        )
        print(f"  {root.name}: {len(names)} classes, kpt_shape={meta.get('kpt_shape')}")

    print()

    # ── 3. Build merged class list ────────────────────────────────────────────
    merged_names = build_merged_class_list(datasets_info)
    merged_name_to_id = {name: i for i, name in enumerate(merged_names)}
    print(f"Merged class list ({len(merged_names)} classes):")
    for i, name in enumerate(merged_names):
        print(f"  {i:3d}  {name}")
    print()

    # Determine output kpt_shape (use the first non-None one found)
    out_kpt_shape = None
    for info in datasets_info:
        if info["kpt_shape"] is not None:
            out_kpt_shape = info["kpt_shape"]
            break

    # ── 4. Process each dataset ───────────────────────────────────────────────
    total_copied = 0
    total_skipped_fail = 0
    total_skipped_no_label = 0
    collision_count = 0

    print(f"Using {jobs} worker process(es)\n")

    for info in datasets_info:
        root = info["root"]
        names = info["names"]

        # Build class ID remap for this dataset
        old_id_to_new = {i: merged_name_to_id[name] for i, name in enumerate(names)}

        validation_state = load_validation_state(root)
        if validation_state is None:
            print(
                f"  WARNING: {root.name}: no validation_state.json found — skipping all images"
            )
        pairs = find_image_label_pairs(root)

        ds_fail = 0
        candidate_pairs: list[tuple[Path, Path]] = []

        for img_path, label_path in pairs:
            # Filter by validation state
            if not is_image_passing(
                img_path, root, validation_state, include_unvalidated
            ):
                ds_fail += 1
                continue
            candidate_pairs.append((img_path, label_path))

        stem_counts: dict[str, int] = {}
        tasks: list[dict] = []
        for img_path, label_path in candidate_pairs:
            base_stem = safe_stem(root, img_path)
            count = stem_counts.get(base_stem, 0)
            stem_counts[base_stem] = count + 1
            stem = base_stem if count == 0 else f"{base_stem}_{count}"
            if count > 0:
                collision_count += 1

            out_img = out_images / (stem + img_path.suffix)
            out_lbl = out_labels / (stem + ".txt")
            tasks.append(
                {
                    "img_path": str(img_path),
                    "label_path": str(label_path),
                    "out_img": str(out_img),
                    "out_lbl": str(out_lbl),
                    "old_id_to_new": old_id_to_new,
                }
            )

        ds_copied = 0
        ds_no_label = 0
        if jobs == 1:
            results_iter = map(process_pair_task, tasks)
            progress_iter = tqdm(
                results_iter,
                total=len(tasks),
                desc=f"Merging {root.name}",
                leave=False,
            )
            for copied, skipped_no_label in progress_iter:
                if copied:
                    ds_copied += 1
                if skipped_no_label:
                    ds_no_label += 1
        else:
            with mp.Pool(processes=jobs) as pool:
                results_iter = pool.imap_unordered(
                    process_pair_task, tasks, chunksize=64
                )
                progress_iter = tqdm(
                    results_iter,
                    total=len(tasks),
                    desc=f"Merging {root.name}",
                    leave=False,
                )
                for copied, skipped_no_label in progress_iter:
                    if copied:
                        ds_copied += 1
                    if skipped_no_label:
                        ds_no_label += 1
                progress_iter.close()

        total_skipped_no_label += ds_no_label
        print(
            f"  {root.name}: copied {ds_copied}, skipped (fail) {ds_fail}, skipped (no lbl) {ds_no_label}"
        )
        total_copied += ds_copied
        total_skipped_fail += ds_fail

    # ── 5. Write data.yml ─────────────────────────────────────────────────────
    yaml_lines = [
        f"path: {args.output_dir.resolve()}",
        f"{args.split}: images/{args.split}",
        f"nc: {len(merged_names)}",
        f"names: {merged_names}",
    ]
    if out_kpt_shape is not None:
        yaml_lines.append(f"kpt_shape: {out_kpt_shape}")

    yaml_path = args.output_dir / "data.yml"
    yaml_path.write_text("\n".join(yaml_lines) + "\n")

    print(f"\nTotal copied:           {total_copied}")
    print(f"Total skipped (fail):   {total_skipped_fail}")
    print(f"Total skipped (no lbl): {total_skipped_no_label}")
    if collision_count:
        print(f"Filename collisions resolved: {collision_count}")
    print(f"\nWrote data.yml -> {yaml_path}")
    print("Done.")


if __name__ == "__main__":
    main()
