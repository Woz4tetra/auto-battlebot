"""Convert opponent segmentation labels into box-only, masked-keypoint pose labels.

Builds an opponent-only YOLO *pose* dataset from a segmentation-polygon dataset (e.g.
``training/data/nhrl_seg/nhrl_robots``). For every frame:

  * If it contains ANY ``--exclude-frame-classes`` label (e.g. our own robots), the whole
    frame is skipped -- so those robots never appear unlabeled in the opponent data. Our
    robots are meant to come only from the keypoint dataset, with real keypoints.
  * Each ``--keep-class`` polygon is converted to its bounding box and written as a pose row
    ``<out-class> cx cy w h  0 0 0 ...`` with ``--num-keypoints`` visibility-0 (masked)
    keypoints. Ultralytics ignores vis-0 keypoints in the location loss, so these rows are
    box-only supervision while staying schema-compatible with real keypoint rows.

Frames with no kept opponent boxes are skipped. Paired images are copied (hardlinked when possible).

Used for Exp 2: mix real opponent boxes (single class, box-only) into a keypoint dataset.

Usage:
  python training/yolo/seg_to_boxonly_pose.py training/data/nhrl_seg/nhrl_robots \
      -o training/data/opponent_pose \
      --keep-class 1 --exclude-frame-classes 3 4 --out-class 2 --num-keypoints 2
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp")


def find_label_files(dataset_dir: Path) -> tuple[list[Path], Path]:
    """Return (all label .txt files, labels_root). Handles flat or nested (train/labels) layouts."""
    labels_root = dataset_dir / "labels" if (dataset_dir / "labels").is_dir() else dataset_dir
    return sorted(labels_root.rglob("*.txt")), labels_root


def find_paired_image(label_path: Path) -> Path | None:
    """Find the paired image (mirror the label's nearest ``labels`` path segment to ``images``)."""
    parts = label_path.parts
    if "labels" in parts:
        idx = len(parts) - 1 - parts[::-1].index("labels")  # nearest 'labels' segment
        stem = Path(*parts[idx + 1 :]).name
        stem = stem[: -len(".txt")] if stem.endswith(".txt") else stem
        base_dir = Path(*parts[:idx]) / "images" / Path(*parts[idx + 1 : -1])
        for ext in IMAGE_EXTENSIONS:
            candidate = base_dir / (stem + ext)  # append ext; names contain dots, so no with_suffix
            if candidate.is_file():
                return candidate
    for ext in IMAGE_EXTENSIONS:
        candidate = label_path.with_suffix(ext)
        if candidate.is_file():
            return candidate
    return None


def polygon_to_bbox(coords: list[float]) -> tuple[float, float, float, float] | None:
    """Axis-aligned normalized bbox (cx, cy, w, h) from a flat [x, y, x, y, ...] polygon."""
    xs, ys = coords[0::2], coords[1::2]
    if len(xs) < 3 or len(ys) < 3:
        return None
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    w, h = x1 - x0, y1 - y0
    if w <= 0 or h <= 0:
        return None
    cx = min(max(x0 + w / 2, 0.0), 1.0)
    cy = min(max(y0 + h / 2, 0.0), 1.0)
    return cx, cy, min(w, 1.0), min(h, 1.0)


def convert_label(
    text: str, keep_class: int, exclude: set[int], out_class: int, kp_suffix: str
) -> list[str] | None:
    """Convert one label file. Returns None if the frame is skipped (has an excluded class)."""
    out_lines: list[str] = []
    for raw in text.splitlines():
        parts = raw.split()
        if not parts:
            continue
        try:
            cls = int(float(parts[0]))
        except ValueError:
            continue
        if cls in exclude:
            return None  # excluded class present -> drop the whole frame
        if cls != keep_class:
            continue
        try:
            coords = [float(v) for v in parts[1:]]
        except ValueError:
            continue
        box = (
            (coords[0], coords[1], coords[2], coords[3])
            if len(coords) == 4
            else polygon_to_bbox(coords)
        )
        if box is None:
            continue
        cx, cy, w, h = box
        out_lines.append(f"{out_class} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}{kp_suffix}")
    return out_lines


def build_arg_parser() -> argparse.ArgumentParser:
    """Construct the command line argument parser."""
    parser = argparse.ArgumentParser(
        description="Seg-polygon opponents -> box-only masked-keypoint pose labels"
    )
    parser.add_argument("dataset", type=Path, help="Segmentation-polygon YOLO dataset dir")
    parser.add_argument("-o", "--output", type=Path, required=True, help="Output dataset dir")
    parser.add_argument(
        "--keep-class", type=int, default=1, help="Class id kept as opponent (default: 1)"
    )
    parser.add_argument(
        "--exclude-frame-classes",
        type=int,
        nargs="*",
        default=[3, 4],
        help="Skip any frame containing these class ids (default: 3 4 = our robots)",
    )
    parser.add_argument(
        "--out-class", type=int, default=2, help="Output class id for opponents (default: 2)"
    )
    parser.add_argument(
        "--num-keypoints",
        type=int,
        default=2,
        help="Masked keypoints appended per row (default: 2)",
    )
    parser.add_argument("--skip-copy-images", action="store_true", help="Write labels only")
    return parser


def main() -> None:
    """Convert a seg-polygon dataset into a flat opponent-only box-only pose dataset."""
    args = build_arg_parser().parse_args()
    if not args.dataset.is_dir():
        raise SystemExit(f"dataset not found: {args.dataset}")
    label_files, labels_root = find_label_files(args.dataset)
    if not label_files:
        raise SystemExit(f"no label .txt files under {args.dataset}")

    exclude = set(args.exclude_frame_classes)
    kp_suffix = " 0 0 0" * args.num_keypoints
    out_images = args.output / "images"
    out_labels = args.output / "labels"
    out_images.mkdir(parents=True, exist_ok=True)
    out_labels.mkdir(parents=True, exist_ok=True)

    kept = skipped_excluded = no_opponent = no_image = 0
    for lbl in label_files:
        lines = convert_label(lbl.read_text(), args.keep_class, exclude, args.out_class, kp_suffix)
        if lines is None:
            skipped_excluded += 1
            continue
        if not lines:
            no_opponent += 1
            continue
        image = find_paired_image(lbl)
        if image is None:
            no_image += 1
            continue
        rel = lbl.relative_to(labels_root).with_suffix("")
        stem = "__".join(p for p in rel.parts if p != "labels")
        (out_labels / f"{stem}.txt").write_text("\n".join(lines) + "\n")
        if not args.skip_copy_images:
            dst = out_images / f"{stem}{image.suffix.lower()}"
            try:
                dst.hardlink_to(image)
            except (OSError, FileExistsError):
                shutil.copyfile(image, dst)
        kept += 1

    print(
        f"kept {kept} opponent frames | skipped {skipped_excluded} (contain excluded classes) | "
        f"{no_opponent} had no opponent boxes | {no_image} missing image"
    )
    print(f"output: {args.output} (images/, labels/)")


if __name__ == "__main__":
    main()
