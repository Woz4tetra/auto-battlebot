#!/usr/bin/env python3
"""Remove class IDs from YOLO label files.

This tool removes annotation lines whose class ID matches any provided
`label_ids`. It does not remap remaining classes.

Usage:
    python delete_labels.py /path/to/dataset 39 40 --output /path/to/output
"""

import argparse
import multiprocessing as mp
import shutil
import sys
from pathlib import Path

from tqdm import tqdm

IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp")
WORKER_CONTEXT: dict[str, object] = {}


def find_label_files(dataset_dir: Path) -> tuple[list[Path], Path]:
    labels_root = (
        dataset_dir / "labels" if (dataset_dir / "labels").is_dir() else dataset_dir
    )
    return sorted(labels_root.rglob("*.txt")), labels_root


def find_existing_image_for_prefix(prefix_no_suffix: Path) -> Path | None:
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
    candidate_prefixes: list[Path] = []

    images_root = dataset_dir / "images"
    if images_root.is_dir():
        try:
            rel_no_suffix = label_path.relative_to(labels_root).with_suffix("")
            candidate_prefixes.append(images_root / rel_no_suffix)
        except ValueError:
            pass

    candidate_prefixes.append(label_path.with_suffix(""))

    parts = label_path.parts
    if "labels" in parts:
        idx = parts.index("labels")
        prefix = Path(*parts[:idx])
        rel_no_suffix = Path(*parts[idx + 1 :]).with_suffix("")
        candidate_prefixes.append(prefix / "images" / rel_no_suffix)

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


def remove_label_lines(
    lines: list[str], labels_to_remove: set[int]
) -> tuple[list[str], set[int], int, int]:
    filtered_lines: list[str] = []
    classes_before: set[int] = set()
    malformed_count = 0
    removed_count = 0

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
        if src_cls in labels_to_remove:
            removed_count += 1
            continue
        filtered_lines.append(" ".join(parts))

    return filtered_lines, classes_before, malformed_count, removed_count


def init_worker(
    dataset_dir: str,
    labels_root: str,
    output_dir: str | None,
    labels_to_remove: set[int],
    skip_copy_images: bool,
    dry_run: bool,
) -> None:
    WORKER_CONTEXT["dataset_dir"] = Path(dataset_dir)
    WORKER_CONTEXT["labels_root"] = Path(labels_root)
    WORKER_CONTEXT["output_dir"] = Path(output_dir) if output_dir else None
    WORKER_CONTEXT["labels_to_remove"] = labels_to_remove
    WORKER_CONTEXT["skip_copy_images"] = skip_copy_images
    WORKER_CONTEXT["dry_run"] = dry_run


def process_label_file(label_path_str: str) -> dict[str, object]:
    label_path = Path(label_path_str)
    dataset_dir = WORKER_CONTEXT["dataset_dir"]
    labels_root = WORKER_CONTEXT["labels_root"]
    output_dir = WORKER_CONTEXT["output_dir"]
    labels_to_remove = WORKER_CONTEXT["labels_to_remove"]
    skip_copy_images = WORKER_CONTEXT["skip_copy_images"]
    dry_run = WORKER_CONTEXT["dry_run"]

    lines = label_path.read_text().splitlines()
    filtered, classes_before, malformed_count, removed_count = remove_label_lines(
        lines, labels_to_remove
    )

    result: dict[str, object] = {
        "label_path": str(label_path),
        "malformed_count": malformed_count,
        "removed_count": removed_count,
        "classes_before": classes_before,
        "hit_labels": classes_before & labels_to_remove,
        "written": False,
        "image_copied": False,
        "missing_image_for_label": None,
        "has_valid_labels": bool(classes_before),
        "dry_run_action": None,
    }

    if not classes_before:
        return result

    if output_dir is not None:
        rel = label_path.relative_to(dataset_dir)
        dest = output_dir / rel
    else:
        dest = label_path

    if dry_run:
        result["dry_run_action"] = (
            f"removed={removed_count} from classes={sorted(classes_before)} => {dest}"
        )
        return result

    paired_image = find_paired_image(label_path, dataset_dir, labels_root)
    dest.parent.mkdir(parents=True, exist_ok=True)
    if filtered:
        dest.write_text("\n".join(filtered) + "\n")
    else:
        # Keep empty labels so paired images become negative examples.
        dest.write_text("")
    result["written"] = True

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


def copy_dataset_yaml_if_present(dataset_dir: Path, output_dir: Path) -> Path | None:
    for name in ("data.yaml", "data.yml"):
        src = dataset_dir / name
        if src.exists():
            dst = output_dir / name
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
            return dst
    return None


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Remove class IDs from YOLO label files"
    )
    parser.add_argument("dataset", type=str, help="Path to dataset directory")
    parser.add_argument(
        "label_ids",
        type=int,
        nargs="+",
        help="Class IDs to remove from annotation lines",
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

    dataset_dir = Path(args.dataset)
    output_dir = Path(args.output) if args.output else None
    labels_to_remove = set(args.label_ids)

    if not dataset_dir.is_dir():
        print(f"Error: dataset directory not found: {dataset_dir}", file=sys.stderr)
        sys.exit(1)
    if not labels_to_remove:
        print("Error: provide at least one label ID", file=sys.stderr)
        sys.exit(1)

    print(f"Remove labels: {sorted(labels_to_remove)}")
    label_files, labels_root = find_label_files(dataset_dir)
    if not label_files:
        print(f"No .txt label files found in {dataset_dir}", file=sys.stderr)
        sys.exit(1)
    print(f"Found {len(label_files)} label files")

    written_count = 0
    image_copy_count = 0
    missing_paired_image_count = 0
    missing_paired_image_examples: list[str] = []
    skipped_malformed_total = 0
    removed_annotations_total = 0
    files_with_removals = 0
    hit_labels: set[int] = set()

    jobs = args.jobs if args.jobs > 0 else (mp.cpu_count() or 1)
    jobs = max(1, jobs)
    print(f"Using {jobs} worker process(es)")

    worker_inputs = [str(p) for p in label_files]
    if jobs == 1:
        init_worker(
            str(dataset_dir),
            str(labels_root),
            str(output_dir) if output_dir else None,
            labels_to_remove,
            args.skip_copy_images,
            args.dry_run,
        )
        results_iter = map(process_label_file, worker_inputs)
        progress_iter = tqdm(
            results_iter, total=len(worker_inputs), desc="Removing labels"
        )
    else:
        with mp.Pool(
            processes=jobs,
            initializer=init_worker,
            initargs=(
                str(dataset_dir),
                str(labels_root),
                str(output_dir) if output_dir else None,
                labels_to_remove,
                args.skip_copy_images,
                args.dry_run,
            ),
        ) as pool:
            results_iter = pool.imap_unordered(
                process_label_file, worker_inputs, chunksize=64
            )
            progress_iter = tqdm(
                results_iter, total=len(worker_inputs), desc="Removing labels"
            )

            for result in progress_iter:
                skipped_malformed_total += int(result["malformed_count"])
                removed_annotations_total += int(result["removed_count"])
                hit_labels.update(result["hit_labels"])
                if int(result["removed_count"]) > 0:
                    files_with_removals += 1

                if args.dry_run and result["has_valid_labels"]:
                    label_name = Path(str(result["label_path"])).name
                    print(f"  {label_name}: {result['dry_run_action']}")

                if bool(result["written"]):
                    written_count += 1

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
            removed_annotations_total += int(result["removed_count"])
            hit_labels.update(result["hit_labels"])
            if int(result["removed_count"]) > 0:
                files_with_removals += 1

            if args.dry_run and result["has_valid_labels"]:
                label_name = Path(str(result["label_path"])).name
                print(f"  {label_name}: {result['dry_run_action']}")

            if bool(result["written"]):
                written_count += 1

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

    missing_labels = sorted(labels_to_remove - hit_labels)
    if missing_labels:
        print(
            "Warning: requested label IDs not found in parsed annotations: "
            + ", ".join(str(v) for v in missing_labels)
        )

    if not args.dry_run:
        target = output_dir if output_dir else dataset_dir
        print(
            f"Done. Wrote {written_count} label files and {image_copy_count} images to {target}"
        )
        print(
            f"Removed {removed_annotations_total} label annotations across {files_with_removals} files"
        )
        if output_dir is not None:
            copied_yaml = copy_dataset_yaml_if_present(dataset_dir, output_dir)
            if copied_yaml:
                print(f"Copied dataset yaml -> {copied_yaml}")
        if missing_paired_image_count:
            print(
                f"Could not find paired images for {missing_paired_image_count} label files"
            )
            for sample in missing_paired_image_examples:
                print(f"  missing image for label: {sample}")
    else:
        print("Dry run complete.")


if __name__ == "__main__":
    main()
