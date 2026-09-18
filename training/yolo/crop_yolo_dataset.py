"""Crop a YOLO dataset's images and annotations to a pixel rectangle, in place.

Broadcast footage carries banners the camera never sees: the MassDestruction streams put
a scoreboard above the cage view and a ticker below it. Training or scoring on those rows
teaches the detector about graphics. This crops the images and moves the labels with them,
so a hand-labeled set stays usable without relabeling.

Boxes are shifted into the crop and clipped to it; a box left thinner than --min-box-px on
either axis is dropped. Keypoints that land outside the crop are written as
``0.000000 0.000000 0`` (visibility 0 = out-of-frame, the convention in
training/synthetic/synthgen/annotations.py); the rest keep their visibility flag. Rows that
are neither detect (5 values) nor pose (5 + 3k) abort the run before anything is written,
since a polygon cannot be clipped this way.

Each recording's data.yaml gains a ``crop``/``source_size`` record, which also makes the
pass idempotent: an image already at the crop size is left alone rather than cropped twice.

Usage:
  # one recording, or a root of subdatasets filtered by directory name
  python training/yolo/crop_yolo_dataset.py training/data/cage_high_x50_conf044 \
      --rect 0 94 1920 895 --match 'r[123]_*' --dry-run
"""

from __future__ import annotations

import argparse
import fnmatch
from pathlib import Path

from PIL import Image

IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp")
KPT_OUT_OF_FRAME = "0.000000 0.000000 0"


class UnsupportedRowError(ValueError):
    """A label row that is neither a detect nor a pose row (e.g. a seg polygon)."""


def crop_row(
    parts: list[str], src_w: int, src_h: int, rect: tuple[int, int, int, int], min_box_px: float
) -> tuple[str | None, bool, int]:
    """Crop one detect/pose row. Returns (row or None if dropped, was_clipped, kpts_zeroed).

    Coordinates come in normalized to the source image and go out normalized to the crop.
    """
    if len(parts) < 5 or (len(parts) - 5) % 3 != 0:
        raise UnsupportedRowError(" ".join(parts))
    try:
        class_id = int(float(parts[0]))
        values = [float(v) for v in parts[1:]]
    except ValueError as exc:
        raise UnsupportedRowError(" ".join(parts)) from exc

    x0, y0, crop_w, crop_h = rect
    cx, cy, w, h = values[:4]
    x1 = (cx - w / 2) * src_w - x0
    x2 = (cx + w / 2) * src_w - x0
    y1 = (cy - h / 2) * src_h - y0
    y2 = (cy + h / 2) * src_h - y0
    clipped_x1, clipped_x2 = max(x1, 0.0), min(x2, float(crop_w))
    clipped_y1, clipped_y2 = max(y1, 0.0), min(y2, float(crop_h))
    if clipped_x2 - clipped_x1 < min_box_px or clipped_y2 - clipped_y1 < min_box_px:
        return None, False, 0
    was_clipped = (clipped_x1, clipped_x2, clipped_y1, clipped_y2) != (x1, x2, y1, y2)

    row = (
        f"{class_id} "
        f"{(clipped_x1 + clipped_x2) / 2 / crop_w:.6f} "
        f"{(clipped_y1 + clipped_y2) / 2 / crop_h:.6f} "
        f"{(clipped_x2 - clipped_x1) / crop_w:.6f} "
        f"{(clipped_y2 - clipped_y1) / crop_h:.6f}"
    )
    zeroed = 0
    for i in range(4, len(values), 3):
        kx = values[i] * src_w - x0
        ky = values[i + 1] * src_h - y0
        visibility = values[i + 2]
        if visibility == 0 or not (0 <= kx < crop_w and 0 <= ky < crop_h):
            row += f" {KPT_OUT_OF_FRAME}"
            zeroed += visibility != 0
        else:
            row += f" {kx / crop_w:.6f} {ky / crop_h:.6f} {visibility:g}"
    return row, was_clipped, zeroed


def crop_label_text(
    text: str, src_w: int, src_h: int, rect: tuple[int, int, int, int], min_box_px: float
) -> tuple[str, int, int, int]:
    """Crop every row of one label file. Returns (text, dropped, clipped, kpts_zeroed)."""
    rows: list[str] = []
    dropped = clipped = zeroed = 0
    for line in text.splitlines():
        parts = line.split()
        if not parts:
            continue
        row, was_clipped, kpts = crop_row(parts, src_w, src_h, rect, min_box_px)
        zeroed += kpts
        if row is None:
            dropped += 1
            continue
        rows.append(row)
        clipped += was_clipped
    return ("\n".join(rows) + "\n" if rows else ""), dropped, clipped, zeroed


def find_recordings(roots: list[Path], match: str | None) -> list[Path]:
    """Directories holding an images/ dir, at or under each root, optionally name-filtered."""
    found: list[Path] = []
    for root in roots:
        candidates = [root] if (root / "images").is_dir() else sorted(root.rglob("images"))
        for candidate in candidates:
            recording = candidate if candidate.name != "images" else candidate.parent
            if match and not fnmatch.fnmatch(recording.name, match):
                continue
            if (recording / "images").is_dir() and recording not in found:
                found.append(recording)
    return found


def label_path_for(img_path: Path) -> Path:
    """dataset/images/x.png -> dataset/labels/x.txt (nearest images/ component)."""
    parts = img_path.parts
    for i in range(len(parts) - 1, -1, -1):
        if parts[i] == "images":
            return Path(*parts[:i], "labels", *parts[i + 1 :]).with_suffix(".txt")
    return img_path.with_suffix(".txt")


def record_crop(
    recording: Path, rect: tuple[int, int, int, int], src_size: tuple[int, int]
) -> None:
    """Append the crop provenance to data.yaml, leaving the existing text untouched."""
    for name in ("data.yaml", "data.yml"):
        data_yaml = recording / name
        if not data_yaml.exists():
            continue
        text = data_yaml.read_text()
        if "\ncrop:" in text or text.startswith("crop:"):
            return
        prefix = "" if text.endswith("\n") else "\n"
        data_yaml.write_text(
            f"{text}{prefix}crop: [{rect[0]}, {rect[1]}, {rect[2]}, {rect[3]}]\n"
            f"source_size: [{src_size[0]}, {src_size[1]}]\n"
        )
        return


def plan_crop(
    recordings: list[Path], rect: tuple[int, int, int, int], min_box_px: float
) -> tuple[list[tuple[Path, list[Path]]], int]:
    """Check every image and label before anything is written, since a crop that fails
    halfway leaves a dataset of mixed sizes. Returns (work, images already cropped)."""
    x0, y0, crop_w, crop_h = rect
    work: list[tuple[Path, list[Path]]] = []
    skipped = 0
    for recording in recordings:
        todo: list[Path] = []
        images = sorted(
            p for p in (recording / "images").rglob("*") if p.suffix.lower() in IMAGE_EXTENSIONS
        )
        for img_path in images:
            with Image.open(img_path) as img:
                size = img.size
            if size == (crop_w, crop_h):
                skipped += 1
                continue
            if size[0] < x0 + crop_w or size[1] < y0 + crop_h:
                raise SystemExit(f"{img_path} is {size[0]}x{size[1]}, too small for the crop")
            label_path = label_path_for(img_path)
            if label_path.exists():
                try:
                    crop_label_text(label_path.read_text(), *size, rect, min_box_px)
                except UnsupportedRowError as exc:
                    raise SystemExit(f"{label_path}: cannot crop row: {exc}") from exc
            todo.append(img_path)
        work.append((recording, todo))
    return work, skipped


def apply_crop(
    work: list[tuple[Path, list[Path]]],
    rect: tuple[int, int, int, int],
    min_box_px: float,
    dry_run: bool,
) -> tuple[int, int, int, int]:
    """Crop images and labels. Returns (images, boxes dropped, boxes clipped, kpts zeroed)."""
    x0, y0, crop_w, crop_h = rect
    images_done = dropped_total = clipped_total = zeroed_total = 0
    for recording, images in work:
        src_size: tuple[int, int] | None = None
        for img_path in images:
            with Image.open(img_path) as img:
                src_size = img.size
                cropped = img.crop((x0, y0, x0 + crop_w, y0 + crop_h))
                if not dry_run:
                    cropped.save(img_path)
            images_done += 1
            label_path = label_path_for(img_path)
            if not label_path.exists():
                continue
            text, dropped, clipped, zeroed = crop_label_text(
                label_path.read_text(), *src_size, rect, min_box_px
            )
            dropped_total += dropped
            clipped_total += clipped
            zeroed_total += zeroed
            if not dry_run:
                label_path.write_text(text)
        if src_size is not None and not dry_run:
            record_crop(recording, rect, src_size)
        print(f"{recording}: {len(images)} images")
    return images_done, dropped_total, clipped_total, zeroed_total


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("dataset", type=Path, nargs="+", help="recording dir or a root of them")
    parser.add_argument(
        "--rect",
        type=int,
        nargs=4,
        required=True,
        metavar=("X0", "Y0", "WIDTH", "HEIGHT"),
        help="crop in source pixels",
    )
    parser.add_argument("--match", help="only recordings whose directory name matches this glob")
    parser.add_argument(
        "--min-box-px",
        type=float,
        default=2.0,
        help="drop boxes clipped below this width or height (default: 2)",
    )
    parser.add_argument("--dry-run", action="store_true", help="report without writing")
    args = parser.parse_args()

    x0, y0, crop_w, crop_h = args.rect
    rect = (x0, y0, crop_w, crop_h)
    recordings = find_recordings(args.dataset, args.match)
    if not recordings:
        raise SystemExit("No recordings matched")

    work, skipped = plan_crop(recordings, rect, args.min_box_px)
    images, dropped, clipped, zeroed = apply_crop(work, rect, args.min_box_px, args.dry_run)

    verb = "would crop" if args.dry_run else "cropped"
    print(f"\n{verb} {images} images to {crop_w}x{crop_h} at ({x0}, {y0})")
    print(f"boxes dropped (outside crop): {dropped}")
    print(f"boxes clipped to the edge:    {clipped}")
    print(f"keypoints marked out-of-frame: {zeroed}")
    if skipped:
        print(f"already at crop size, skipped: {skipped}")


if __name__ == "__main__":
    main()
