"""Convert a YOLO segmentation-polygon dataset into a detection (bbox-only) dataset.

Preserves class ids and the train/val/test split structure exactly. Each polygon
label row ``class x1 y1 x2 y2 ...`` becomes a box row ``class cx cy w h`` from the
polygon's axis-aligned bounds. Rows already in box form (4 coords) pass through.
Images are hardlinked (fast, no copy) when the output is on the same filesystem,
otherwise copied.

Built for the seg-vs-bbox controlled comparison: identical images, classes, and
splits as the seg dataset, box labels only, so any box-AP difference is attributable
to the seg-vs-detect head and nothing else.

Usage:
  python training/yolo/seg_to_bbox.py training/data/nhrl_seg/nhrl_robots \
      -o training/data/nhrl_robots_bbox
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import yaml

IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp")
SPLITS = ("train", "val", "test")


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


def convert_label(text: str) -> list[str]:
    """Convert one polygon label file's rows to box rows, preserving class ids."""
    out_lines: list[str] = []
    for raw in text.splitlines():
        parts = raw.split()
        if not parts:
            continue
        try:
            cls = int(float(parts[0]))
            coords = [float(v) for v in parts[1:]]
        except ValueError:
            continue
        if len(coords) == 4:
            cx, cy, w, h = coords  # already a box row
        else:
            box = polygon_to_bbox(coords)
            if box is None:
                continue
            cx, cy, w, h = box
        out_lines.append(f"{cls} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
    return out_lines


def link_or_copy(src: Path, dst: Path) -> None:
    """Hardlink src to dst, falling back to a copy across filesystems."""
    try:
        dst.hardlink_to(src)
    except (OSError, FileExistsError):
        shutil.copyfile(src, dst)


def find_image(label_path: Path, images_dir: Path) -> Path | None:
    """Find the image paired with a label (same stem; names may contain dots)."""
    stem = label_path.name[: -len(".txt")] if label_path.name.endswith(".txt") else label_path.stem
    for ext in IMAGE_EXTENSIONS:
        candidate = images_dir / (stem + ext)
        if candidate.is_file():
            return candidate
    return None


def convert_split(
    src_split: Path, out_split: Path, skip_images: bool
) -> tuple[int, int, dict[int, int]]:
    """Convert one split. Returns (frames written, images missing, per-class instance counts)."""
    src_labels = src_split / "labels"
    src_images = src_split / "images"
    if not src_labels.is_dir():
        return 0, 0, {}
    (out_split / "labels").mkdir(parents=True, exist_ok=True)
    if not skip_images:
        (out_split / "images").mkdir(parents=True, exist_ok=True)

    frames = missing = 0
    counts: dict[int, int] = {}
    for lbl in sorted(src_labels.glob("*.txt")):
        lines = convert_label(lbl.read_text())
        (out_split / "labels" / lbl.name).write_text("\n".join(lines) + ("\n" if lines else ""))
        for ln in lines:
            cls = int(ln.split()[0])
            counts[cls] = counts.get(cls, 0) + 1
        if not skip_images:
            img = find_image(lbl, src_images)
            if img is None:
                missing += 1
            else:
                link_or_copy(img, out_split / "images" / img.name)
        frames += 1
    return frames, missing, counts


def build_arg_parser() -> argparse.ArgumentParser:
    """Construct the command line argument parser."""
    parser = argparse.ArgumentParser(description="Seg-polygon dataset -> bbox detection dataset")
    parser.add_argument("dataset", type=Path, help="Source seg dataset dir (data.yml + splits)")
    parser.add_argument("-o", "--output", type=Path, required=True, help="Output dataset dir")
    parser.add_argument("--skip-images", action="store_true", help="Write labels + data.yml only")
    return parser


def write_data_yml(src: Path, out: Path) -> list[str]:
    """Write a detection data.yml mirroring the source classes; return class names."""
    src_yml = yaml.safe_load((src / "data.yml").read_text())
    names: list[str] = list(src_yml["names"])
    data = {
        "path": str(out.resolve()),
        "train": "train/images",
        "val": "val/images",
        "test": "test/images",
        "nc": len(names),
        "names": names,
    }
    if "colors" in src_yml:
        data["colors"] = src_yml["colors"]
    (out / "data.yml").write_text(yaml.safe_dump(data, sort_keys=False))
    return names


def main() -> None:
    """Convert a seg-polygon dataset into a box-only detection dataset."""
    args = build_arg_parser().parse_args()
    if not (args.dataset / "data.yml").is_file():
        raise SystemExit(f"no data.yml under {args.dataset}")
    args.output.mkdir(parents=True, exist_ok=True)

    names = write_data_yml(args.dataset, args.output)
    total_counts: dict[int, int] = {}
    for split in SPLITS:
        frames, missing, counts = convert_split(
            args.dataset / split, args.output / split, args.skip_images
        )
        for cls, n in counts.items():
            total_counts[cls] = total_counts.get(cls, 0) + n
        miss = f", {missing} missing images" if missing else ""
        print(f"{split}: {frames} frames{miss}")

    print("instances per class:")
    for cls, name in enumerate(names):
        print(f"  {cls} {name:16s} {total_counts.get(cls, 0)}")
    print(f"output: {args.output}")


if __name__ == "__main__":
    main()
