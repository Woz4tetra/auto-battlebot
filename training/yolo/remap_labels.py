#!/usr/bin/env python3
"""Remap class IDs in a YOLO dataset.

Reads a TOML config that maps source class IDs to target class IDs, then
rewrites YOLO label files (*.txt). Useful for collapsing multi-class datasets
into fewer classes or converting to binary tasks.

Usage:
    python remap_labels.py remap_config.toml /path/to/dataset --output /path/to/output

Example TOML config:
    [label_map]
    0 = 1
    1 = 0
    2 = 0

Source classes not listed in `label_map` are left unchanged.
"""

import argparse
import ast
import multiprocessing as mp
import shutil
import sys
from pathlib import Path

import tomllib
from tqdm import tqdm

IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp")
SPLIT_NAMES = ("train", "val", "test")
WORKER_CONTEXT: dict[str, object] = {}


def load_config(config_path: Path) -> dict[int, int]:
    with open(config_path, "rb") as f:
        config = tomllib.load(f)

    raw_map = config.get("label_map", {})

    label_map: dict[int, int] = {}
    for src, dst in raw_map.items():
        label_map[int(src)] = int(dst)

    return label_map


def find_label_files(dataset_dir: Path) -> tuple[list[Path], Path]:
    """Find YOLO label files under dataset_dir.

    Returns (label_files, labels_root). If dataset_dir/labels exists, that is
    used as labels_root; otherwise dataset_dir is used.
    """
    labels_root = (
        dataset_dir / "labels" if (dataset_dir / "labels").is_dir() else dataset_dir
    )
    return sorted(labels_root.rglob("*.txt")), labels_root


def find_existing_image_for_prefix(prefix_no_suffix: Path) -> Path | None:
    """Find an existing image file for a stem, extension case-insensitive."""
    parent = prefix_no_suffix.parent
    stem = prefix_no_suffix.name
    if not parent.is_dir():
        return None

    for candidate in parent.glob(f"{stem}.*"):
        if candidate.is_file() and candidate.suffix.lower() in IMAGE_EXTENSIONS:
            return candidate
    return None


def find_paired_image(
    label_path: Path, dataset_dir: Path, labels_root: Path
) -> Path | None:
    """Best-effort lookup of the image paired with a label file."""
    candidate_prefixes: list[Path] = []

    # 1) Canonical YOLO layout: labels/<...>/<name>.txt -> images/<...>/<name>.<ext>
    images_root = dataset_dir / "images"
    if images_root.is_dir():
        try:
            rel_no_suffix = label_path.relative_to(labels_root).with_suffix("")
            candidate_prefixes.append(images_root / rel_no_suffix)
        except ValueError:
            pass

    # 2) Same directory as the label file.
    candidate_prefixes.append(label_path.with_suffix(""))

    # 3) If path includes /labels/, mirror to /images/.
    parts = label_path.parts
    if "labels" in parts:
        idx = parts.index("labels")
        prefix = Path(*parts[:idx])
        rel_no_suffix = Path(*parts[idx + 1 :]).with_suffix("")
        candidate_prefixes.append(prefix / "images" / rel_no_suffix)

    # 4) Also try mirroring relative to the provided dataset root.
    try:
        rel = label_path.relative_to(dataset_dir)
        rel_parts = list(rel.parts)
        if "labels" in rel_parts:
            idx = rel_parts.index("labels")
            rel_no_suffix = Path(*rel_parts[idx + 1 :]).with_suffix("")
            candidate_prefixes.append(dataset_dir / "images" / rel_no_suffix)
    except ValueError:
        pass

    for prefix in candidate_prefixes:
        candidate = find_existing_image_for_prefix(prefix)
        if candidate is not None:
            return candidate
    return None


def remap_label_lines(
    lines: list[str],
    label_map: dict[int, int],
    unmapped_classes: set[int],
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

        classes_before.add(src_cls)
        dst_cls = label_map.get(src_cls, src_cls)
        if src_cls not in label_map:
            unmapped_classes.add(src_cls)
        classes_after.add(dst_cls)
        parts[0] = str(dst_cls)
        remapped_lines.append(" ".join(parts))

    return remapped_lines, classes_before, classes_after, malformed_count


def init_worker(
    dataset_dir: str,
    labels_root: str,
    output_dir: str | None,
    label_map: dict[int, int],
    skip_copy_images: bool,
    dry_run: bool,
) -> None:
    WORKER_CONTEXT["dataset_dir"] = Path(dataset_dir)
    WORKER_CONTEXT["labels_root"] = Path(labels_root)
    WORKER_CONTEXT["output_dir"] = Path(output_dir) if output_dir else None
    WORKER_CONTEXT["label_map"] = label_map
    WORKER_CONTEXT["skip_copy_images"] = skip_copy_images
    WORKER_CONTEXT["dry_run"] = dry_run


def process_label_file(label_path_str: str) -> dict[str, object]:
    label_path = Path(label_path_str)
    dataset_dir = WORKER_CONTEXT["dataset_dir"]
    labels_root = WORKER_CONTEXT["labels_root"]
    output_dir = WORKER_CONTEXT["output_dir"]
    label_map = WORKER_CONTEXT["label_map"]
    skip_copy_images = WORKER_CONTEXT["skip_copy_images"]
    dry_run = WORKER_CONTEXT["dry_run"]
    unmapped_classes: set[int] = set()

    lines = label_path.read_text().splitlines()
    (
        remapped,
        classes_before,
        classes_after,
        malformed_count,
    ) = remap_label_lines(lines, label_map, unmapped_classes)

    result: dict[str, object] = {
        "label_path": str(label_path),
        "malformed_count": malformed_count,
        "classes_before": classes_before,
        "classes_after": classes_after,
        "unmapped_classes": unmapped_classes,
        "written": False,
        "image_copied": False,
        "missing_image_for_label": None,
        "split_name": None,
        "has_valid_labels": bool(classes_before),
        "dry_run_action": None,
    }

    # If a .txt file has no valid YOLO lines, skip it (e.g. classes.txt).
    if not classes_before:
        return result

    if output_dir is not None:
        rel = label_path.relative_to(dataset_dir)
        dest = output_dir / rel
    else:
        dest = label_path

    if dry_run:
        result["dry_run_action"] = (
            f"{sorted(classes_before)} -> {sorted(classes_after)} => {dest}"
        )
        return result

    paired_image = find_paired_image(label_path, dataset_dir, labels_root)

    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text("\n".join(remapped) + "\n")

    result["written"] = True
    split_name = infer_split_name(label_path, labels_root)
    if split_name:
        result["split_name"] = split_name

    if (
        not skip_copy_images
        and output_dir is not None
        and paired_image
        and paired_image.exists()
    ):
        img_rel = paired_image.relative_to(dataset_dir)
        img_dest = output_dir / img_rel
        img_dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(paired_image, img_dest)
        result["image_copied"] = True
    elif not skip_copy_images and output_dir is not None:
        result["missing_image_for_label"] = str(label_path)

    return result


def parse_yaml_names_list(data_yaml_path: Path) -> list[str] | None:
    """Best-effort parse of YOLO `names` from data.yaml/data.yml."""
    try:
        lines = data_yaml_path.read_text().splitlines()
    except OSError:
        return None

    for idx, line in enumerate(lines):
        stripped = line.strip()
        if not stripped.startswith("names:"):
            continue

        value = stripped.partition(":")[2].strip()
        if value:
            # Common YOLO form: names: ["a", "b"] or names: [a, b]
            try:
                parsed = ast.literal_eval(value)
            except (ValueError, SyntaxError):
                parsed = None
            if isinstance(parsed, list):
                return [str(v) for v in parsed]

        # Block forms:
        # names:
        #   - class_a
        #   - class_b
        # or:
        # names:
        #   0: class_a
        #   1: class_b
        base_indent = len(line) - len(line.lstrip())
        list_names: list[str] = []
        indexed_names: dict[int, str] = {}

        for block_line in lines[idx + 1 :]:
            if not block_line.strip():
                continue
            indent = len(block_line) - len(block_line.lstrip())
            if indent <= base_indent:
                break

            s = block_line.strip()
            if s.startswith("- "):
                list_names.append(s[2:].strip().strip("\"'"))
                continue

            key, sep, raw_val = s.partition(":")
            if sep and key.strip().isdigit():
                indexed_names[int(key.strip())] = raw_val.strip().strip("\"'")

        if list_names:
            return list_names
        if indexed_names:
            max_idx = max(indexed_names)
            return [indexed_names.get(i, f"class_{i}") for i in range(max_idx + 1)]

    return None


def find_dataset_yaml(dataset_dir: Path) -> Path | None:
    for name in ("data.yaml", "data.yml"):
        p = dataset_dir / name
        if p.exists():
            return p
    return None


def infer_split_name(label_path: Path, labels_root: Path) -> str | None:
    """Infer split name from labels/<split>/... layout."""
    try:
        rel = label_path.relative_to(labels_root)
    except ValueError:
        return None
    if not rel.parts:
        return None
    first = rel.parts[0]
    if first in SPLIT_NAMES:
        return first
    return None


def build_output_names(
    label_map: dict[int, int],
    observed_output_classes: set[int],
    source_names: list[str] | None,
) -> list[str]:
    class_ids = set(observed_output_classes)
    class_ids.update(label_map.values())

    if not class_ids:
        class_ids = {0}

    max_id = max(class_ids)
    names = [f"class_{i}" for i in range(max_id + 1)]

    if source_names:
        for src_idx, src_name in enumerate(source_names):
            dst_idx = label_map.get(src_idx, src_idx)
            if 0 <= dst_idx < len(names) and names[dst_idx].startswith("class_"):
                names[dst_idx] = src_name

    return names


def write_data_yml(dataset_root: Path, names: list[str], splits: set[str]) -> Path:
    yaml_lines = [f"path: {dataset_root.resolve()}"]
    if splits:
        for split in SPLIT_NAMES:
            if split in splits:
                yaml_lines.append(f"{split}: images/{split}")
    else:
        yaml_lines.append("train: images")
    yaml_lines.append(f"nc: {len(names)}")
    yaml_lines.append(f"names: {names}")

    out_path = dataset_root / "data.yml"
    out_path.write_text("\n".join(yaml_lines) + "\n")
    return out_path


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
        "--skip-copy-images",
        action="store_true",
        help="When using --output, do not copy paired images",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print what would be done without writing files",
    )
    parser.add_argument(
        "-j",
        "--jobs",
        type=int,
        default=0,
        help="Worker processes (default: CPU count)",
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

    label_map = load_config(config_path)
    source_yaml = find_dataset_yaml(dataset_dir)
    source_names = parse_yaml_names_list(source_yaml) if source_yaml else None
    print(f"Label map: {label_map}")
    if source_yaml:
        print(f"Source dataset yaml: {source_yaml}")

    label_files, labels_root = find_label_files(dataset_dir)
    if not label_files:
        print(f"No .txt label files found in {dataset_dir}", file=sys.stderr)
        sys.exit(1)
    print(f"Found {len(label_files)} label files")

    unmapped_classes_seen: set[int] = set()
    written_count = 0
    image_copy_count = 0
    missing_paired_image_count = 0
    missing_paired_image_examples: list[str] = []
    skipped_malformed_total = 0
    remapped_output_classes: set[int] = set()
    observed_splits: set[str] = set()

    jobs = args.jobs if args.jobs > 0 else (mp.cpu_count() or 1)
    jobs = max(1, jobs)
    print(f"Using {jobs} worker process(es)")

    worker_inputs = [str(p) for p in label_files]
    if jobs == 1:
        init_worker(
            str(dataset_dir),
            str(labels_root),
            str(output_dir) if output_dir else None,
            label_map,
            args.skip_copy_images,
            args.dry_run,
        )
        results_iter = map(process_label_file, worker_inputs)
        progress_iter = tqdm(results_iter, total=len(worker_inputs), desc="Remapping labels")
    else:
        with mp.Pool(
            processes=jobs,
            initializer=init_worker,
            initargs=(
                str(dataset_dir),
                str(labels_root),
                str(output_dir) if output_dir else None,
                label_map,
                args.skip_copy_images,
                args.dry_run,
            ),
        ) as pool:
            results_iter = pool.imap_unordered(process_label_file, worker_inputs, chunksize=64)
            progress_iter = tqdm(
                results_iter, total=len(worker_inputs), desc="Remapping labels"
            )

            for result in progress_iter:
                skipped_malformed_total += int(result["malformed_count"])
                unmapped_classes_seen.update(result["unmapped_classes"])

                if args.dry_run and result["has_valid_labels"]:
                    label_name = Path(str(result["label_path"])).name
                    print(f"  {label_name}: {result['dry_run_action']}")

                if bool(result["written"]):
                    written_count += 1
                    remapped_output_classes.update(result["classes_after"])
                    split_name = result["split_name"]
                    if split_name:
                        observed_splits.add(str(split_name))

                if bool(result["image_copied"]):
                    image_copy_count += 1
                elif result["missing_image_for_label"] is not None:
                    missing_paired_image_count += 1
                    if len(missing_paired_image_examples) < 5:
                        missing_paired_image_examples.append(
                            str(result["missing_image_for_label"])
                        )

        progress_iter.close()

    if jobs == 1:
        for result in progress_iter:
            skipped_malformed_total += int(result["malformed_count"])
            unmapped_classes_seen.update(result["unmapped_classes"])

            if args.dry_run and result["has_valid_labels"]:
                label_name = Path(str(result["label_path"])).name
                print(f"  {label_name}: {result['dry_run_action']}")

            if bool(result["written"]):
                written_count += 1
                remapped_output_classes.update(result["classes_after"])
                split_name = result["split_name"]
                if split_name:
                    observed_splits.add(str(split_name))

            if bool(result["image_copied"]):
                image_copy_count += 1
            elif result["missing_image_for_label"] is not None:
                missing_paired_image_count += 1
                if len(missing_paired_image_examples) < 5:
                    missing_paired_image_examples.append(
                        str(result["missing_image_for_label"])
                    )

    if skipped_malformed_total:
        print(f"Skipped {skipped_malformed_total} malformed label lines")
    if unmapped_classes_seen:
        if source_names:
            unmapped_labels_with_names = []
            for cls_id in sorted(unmapped_classes_seen):
                if 0 <= cls_id < len(source_names):
                    unmapped_labels_with_names.append(f"{cls_id} ({source_names[cls_id]})")
                else:
                    unmapped_labels_with_names.append(str(cls_id))
            print(
                "Warning: source classes not present in label_map (left unchanged): "
                + ", ".join(unmapped_labels_with_names)
            )
        else:
            print(
                "Warning: source classes not present in label_map (left unchanged): "
                + ", ".join(str(v) for v in sorted(unmapped_classes_seen))
            )
    if not args.dry_run:
        target = output_dir if output_dir else dataset_dir
        output_names = build_output_names(
            label_map, remapped_output_classes, source_names
        )
        data_yml_path = write_data_yml(target, output_names, observed_splits)
        print(
            f"Done. Wrote {written_count} label files and {image_copy_count} images to {target}"
        )
        print(f"Wrote data.yml -> {data_yml_path}")
        if missing_paired_image_count:
            print(
                f"Could not find paired images for {missing_paired_image_count} label files"
            )
            for sample in missing_paired_image_examples:
                print(f"  missing image for label: {sample}")
    else:
        target = output_dir if output_dir else dataset_dir
        output_names = build_output_names(
            label_map, remapped_output_classes, source_names
        )
        print(
            f"Dry run: would write {target / 'data.yml'} with nc={len(output_names)} and names={output_names}"
        )


if __name__ == "__main__":
    main()
