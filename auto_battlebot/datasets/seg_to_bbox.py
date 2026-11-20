"""Convert a YOLO segmentation-polygon dataset into a detection (bbox-only) dataset."""

from __future__ import annotations

import argparse
import shutil
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import yaml

IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp")
SPLITS = ("train", "val", "test")


CONTOUR_RASTER_SIZE_PX = (
    720,
    1280,
)


def polygon_to_bbox(coords: list[float]) -> tuple[float, float, float, float] | None:
    """Axis-aligned normalized bbox (cx, cy, w, h) of a flat polygon."""
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


SUBCONTOUR_JUMP = 0.03


def split_subcontours(pts: np.ndarray, jump: float = SUBCONTOUR_JUMP) -> list[np.ndarray]:
    """Split one label row's point list where it leaps to a disjoint contour."""
    if len(pts) < 2:
        return [pts]
    steps = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    breaks = np.flatnonzero(steps > jump) + 1
    if breaks.size == 0:
        return [pts]
    return [piece for piece in np.split(pts, breaks) if len(piece) >= 3]


TOP_CONTOURS = 3


def top_contours_bbox(
    polygons: list[list[float]], top_n: int = TOP_CONTOURS
) -> tuple[float, float, float, float] | None:
    """One box for one label: enclosing the label's **``top_n`` largest connected contours**."""
    h, w = CONTOUR_RASTER_SIZE_PX
    mask = np.zeros((h, w), dtype=np.uint8)
    drawn = False
    for coords in polygons:
        if len(coords) < 6:
            continue
        pts = np.asarray(coords, dtype=np.float64).reshape(-1, 2)
        for piece in split_subcontours(pts):
            if len(piece) < 3:
                continue
            scaled = piece * [w, h]
            cv2.fillPoly(mask, [scaled.astype(np.int32)], (1,))
            drawn = True
    if not drawn:
        return None

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    keep = sorted(contours, key=cv2.contourArea, reverse=True)[:top_n]
    x, y, bw, bh = cv2.boundingRect(np.vstack(keep))
    if bw <= 0 or bh <= 0:
        return None
    return (x + bw / 2) / w, (y + bh / 2) / h, bw / w, bh / h


def convert_label(text: str) -> list[str]:
    """Convert one polygon label file to box rows, one box per class."""
    by_class: dict[int, list[list[float]]] = defaultdict(list)
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
            out_lines.append(f"{cls} " + " ".join(f"{v:.6f}" for v in coords))
            continue
        by_class[cls].append(coords)

    for cls in sorted(by_class):
        box = top_contours_bbox(by_class[cls])
        if box is None:
            continue
        out_lines.append(f"{cls} " + " ".join(f"{v:.6f}" for v in box))
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
