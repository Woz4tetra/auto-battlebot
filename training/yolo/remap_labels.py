#!/usr/bin/env python3
"""Remap class IDs in a YOLO dataset.

Reads a TOML config that maps source class IDs to target class IDs, then
rewrites YOLO label files (*.txt). Useful for collapsing multi-class datasets
into fewer classes or converting to binary tasks.

Usage:
    python remap_labels.py remap_config.toml /path/to/dataset --output /path/to/output

Example TOML config:
    # Source class -> target class. Any source class not listed is mapped
    # to `default_label`.
    default_label = 0

    # Delete image+label pairs that contain ANY of these classes.
    delete_labels = [3, 4]

    [label_map]
    0 = 1
    1 = 0
    2 = 0
"""

import argparse
import shutil
import sys
from pathlib import Path

import tomllib
from tqdm import tqdm

IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp")


def load_config(config_path: Path) -> tuple[dict[int, int], int, set[int]]:
    with open(config_path, "rb") as f:
        config = tomllib.load(f)

    default_label = int(config.get("default_label", 0))
    raw_map = config.get("label_map", {})
    delete_labels = set(int(v) for v in config.get("delete_labels", []))

    label_map: dict[int, int] = {}
    for src, dst in raw_map.items():
        label_map[int(src)] = int(dst)

    return label_map, default_label, delete_labels


def find_label_files(dataset_dir: Path) -> list[Path]:
    """Find YOLO label files under dataset_dir (prefers dataset_dir/labels)."""
    search_root = dataset_dir / "labels" if (dataset_dir / "labels").is_dir() else dataset_dir
    return sorted(search_root.rglob("*.txt"))


def find_paired_image(label_path: Path, dataset_dir: Path) -> Path | None:
    """Best-effort lookup of the image paired with a label file."""
    candidates: list[Path] = []

    # 1) Same directory as the label file.
    candidates.extend(label_path.with_suffix(ext) for ext in IMAGE_EXTENSIONS)

    # 2) If path includes /labels/, mirror to /images/.
    parts = label_path.parts
    if "labels" in parts:
        idx = parts.index("labels")
        prefix = Path(*parts[:idx])
        rel_no_suffix = Path(*parts[idx + 1 :]).with_suffix("")
        image_prefix = prefix / "images" / rel_no_suffix
        candidates.extend(image_prefix.with_suffix(ext) for ext in IMAGE_EXTENSIONS)

    # 3) Also try mirroring relative to the provided dataset root.
    try:
        rel = label_path.relative_to(dataset_dir)
        rel_parts = list(rel.parts)
        if "labels" in rel_parts:
            idx = rel_parts.index("labels")
            rel_no_suffix = Path(*rel_parts[idx + 1 :]).with_suffix("")
            image_prefix = dataset_dir / "images" / rel_no_suffix
            candidates.extend(image_prefix.with_suffix(ext) for ext in IMAGE_EXTENSIONS)
    except ValueError:
        pass

    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def remap_label_lines(
    lines: list[str], label_map: dict[int, int], default_label: int
) -> tuple[list[str], set[int], set[int], int]:
    """Remap YOLO label lines.

    Returns:
      - remapped lines
      - classes seen before remap
      - classes seen after remap
      - malformed line count
    """
    remapped_lines: list[str] = []
    classes_before: set[int] = set()
    classes_after: set[int] = set()
    malformed_count = 0

    for raw_line in lines:
        stripped = raw_line.strip()
        if not stripped:
            continue

        parts = stripped.split()
        try:
            src_cls = int(parts[0])
        except (ValueError, IndexError):
            malformed_count += 1
            continue

        dst_cls = label_map.get(src_cls, default_label)
        classes_before.add(src_cls)
        classes_after.add(dst_cls)
        parts[0] = str(dst_cls)
        remapped_lines.append(" ".join(parts))

    return remapped_lines, classes_before, classes_after, malformed_count


def main() -> None:
    parser = argparse.ArgumentParser(description="Remap class IDs in YOLO label files")
    parser.add_argument(
        "config",
        type=str,
        help="Path to TOML config file with label_map",
    )
    parser.add_argument(
        "dataset",
        type=str,
        help="Path to dataset directory (searches recursively for .txt labels)",
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
        help="When using --output, also copy paired images",
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

    label_files = find_label_files(dataset_dir)
    if not label_files:
        print(f"No .txt label files found in {dataset_dir}", file=sys.stderr)
        sys.exit(1)
    print(f"Found {len(label_files)} label files")

    deleted_count = 0
    written_count = 0
    skipped_malformed_total = 0

    for label_path in tqdm(label_files, desc="Remapping labels", disable=args.dry_run):
        lines = label_path.read_text().splitlines()
        remapped, classes_before, classes_after, malformed_count = remap_label_lines(
            lines, label_map, default_label
        )
        skipped_malformed_total += malformed_count

        # If a .txt file has no valid YOLO lines, skip it (e.g. classes.txt).
        if not classes_before:
            continue

        should_delete = delete_labels and bool(classes_before & delete_labels)

        if output_dir is not None:
            rel = label_path.relative_to(dataset_dir)
            dest = output_dir / rel
        else:
            dest = label_path

        if args.dry_run:
            if should_delete:
                action = "DELETE"
            else:
                action = f"{sorted(classes_before)} -> {sorted(classes_after)} => {dest}"
            print(f"  {label_path.name}: {action}")
            continue

        paired_image = find_paired_image(label_path, dataset_dir)

        if should_delete:
            deleted_count += 1
            if output_dir is None:
                label_path.unlink(missing_ok=True)
                if paired_image and paired_image.exists():
                    paired_image.unlink()
            continue

        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_text("\n".join(remapped) + "\n")
        written_count += 1

        if args.copy_images and output_dir is not None and paired_image and paired_image.exists():
            img_rel = paired_image.relative_to(dataset_dir)
            img_dest = output_dir / img_rel
            img_dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(paired_image, img_dest)

    if skipped_malformed_total:
        print(f"Skipped {skipped_malformed_total} malformed label lines")
    if not args.dry_run:
        target = output_dir if output_dir else dataset_dir
        print(f"Done. Wrote {written_count} label files to {target}")
        if deleted_count:
            print(f"Deleted {deleted_count} image+label pairs")


if __name__ == "__main__":
    main()
