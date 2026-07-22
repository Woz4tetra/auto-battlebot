"""Convert a YOLO pose (keypoint) dataset into a detection (bbox-only) dataset.

Preserves the train/val/test split structure exactly. Each pose label row
``class cx cy w h  kx ky kv  kx ky kv ...`` becomes a box row ``class cx cy w h``:
the keypoint columns are dropped, the box columns pass through verbatim. Rows already
in box form (4 coords) pass through unchanged. Optionally remaps class ids so the
output lands in a different class vocabulary (e.g. fold a synthetic pose dataset into
a real detection dataset's classes). Images are hardlinked (fast, no copy) when the
output is on the same filesystem, otherwise copied; ``.npy`` training caches are skipped.

Unlike seg_to_bbox.py, this must NOT treat the extra columns as polygon coordinates:
a pose row's keypoints are not box geometry. Only the first four coords are the box.

Built for the synthetic+bbox experiment: turn the synthetic keypoint dataset
``all_robot_keypoints`` into box-only labels remapped into the ``nhrl_robots_bbox``
vocabulary, so it can be pooled with the real bbox set. See
``docs/experiments/perception_performance/synthetic_plus_bbox_plan.md``.

Usage:
  python training/yolo/pose_to_bbox.py training/data/all_robot_keypoints \
      -o training/data/synth_bbox_from_keypoints \
      --class-map 2:1,0:3,1:4 \
      --names-from training/data/nhrl_robots_bbox/data.yml
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import yaml

IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp")
SPLITS = ("train", "val", "test")


def parse_class_map(spec: str | None) -> dict[int, int]:
    """Parse ``2:1,0:3,1:4`` into {2: 1, 0: 3, 1: 4}. Empty/None -> identity map."""
    if not spec:
        return {}
    out: dict[int, int] = {}
    for pair in spec.split(","):
        pair = pair.strip()
        if not pair:
            continue
        src, dst = pair.split(":")
        out[int(src)] = int(dst)
    return out


def convert_label(text: str, class_map: dict[int, int]) -> tuple[list[str], int]:
    """Convert one pose label file's rows to box rows. Returns (rows, malformed count).

    Keeps ``class cx cy w h`` (first four coords), drops any keypoint columns, and
    applies ``class_map`` (classes absent from the map are left unchanged). A row with
    fewer than four coords is malformed (not a valid box) and is dropped and counted.
    """
    out_lines: list[str] = []
    malformed = 0
    for raw in text.splitlines():
        parts = raw.split()
        if not parts:
            continue
        try:
            cls = int(float(parts[0]))
            coords = [float(v) for v in parts[1:]]
        except ValueError:
            malformed += 1
            continue
        if len(coords) < 4:
            malformed += 1
            continue
        cx, cy, w, h = coords[:4]  # box only; drop keypoint columns
        if w <= 0 or h <= 0:
            malformed += 1
            continue
        cls = class_map.get(cls, cls)
        out_lines.append(f"{cls} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
    return out_lines, malformed


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
    src_split: Path, out_split: Path, class_map: dict[int, int], skip_images: bool
) -> tuple[int, int, int, dict[int, int]]:
    """Convert one split. Returns (frames, missing images, malformed rows, per-class counts)."""
    src_labels = src_split / "labels"
    src_images = src_split / "images"
    if not src_labels.is_dir():
        return 0, 0, 0, {}
    (out_split / "labels").mkdir(parents=True, exist_ok=True)
    if not skip_images:
        (out_split / "images").mkdir(parents=True, exist_ok=True)

    frames = missing = malformed = 0
    counts: dict[int, int] = {}
    for lbl in sorted(src_labels.glob("*.txt")):
        lines, bad = convert_label(lbl.read_text(), class_map)
        malformed += bad
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
    return frames, missing, malformed, counts


def build_arg_parser() -> argparse.ArgumentParser:
    """Construct the command line argument parser."""
    parser = argparse.ArgumentParser(
        description="Pose (keypoint) dataset -> bbox detection dataset"
    )
    parser.add_argument("dataset", type=Path, help="Source pose dataset dir (data.yml + splits)")
    parser.add_argument("-o", "--output", type=Path, required=True, help="Output dataset dir")
    parser.add_argument(
        "--class-map",
        help="Remap class ids, e.g. '2:1,0:3,1:4'. Classes not listed are unchanged.",
    )
    parser.add_argument(
        "--names-from",
        type=Path,
        help="data.yml whose names/colors define the output vocabulary (e.g. the target "
        "detection dataset). Defaults to the source dataset's names.",
    )
    parser.add_argument("--skip-images", action="store_true", help="Write labels + data.yml only")
    return parser


def write_data_yml(src: Path, out: Path, names_from: Path | None) -> list[str]:
    """Write a detection data.yml (no kpt_shape/flip_idx); return class names."""
    vocab_src = names_from if names_from is not None else (src / "data.yml")
    vocab = yaml.safe_load(vocab_src.read_text())
    names: list[str] = list(vocab["names"])
    data = {
        "path": str(out.resolve()),
        "train": "train/images",
        "val": "val/images",
        "test": "test/images",
        "nc": len(names),
        "names": names,
    }
    if "colors" in vocab:
        data["colors"] = vocab["colors"]
    (out / "data.yml").write_text(yaml.safe_dump(data, sort_keys=False))
    return names


def main() -> None:
    """Convert a pose dataset into a box-only detection dataset."""
    args = build_arg_parser().parse_args()
    if not (args.dataset / "data.yml").is_file():
        raise SystemExit(f"no data.yml under {args.dataset}")
    class_map = parse_class_map(args.class_map)
    args.output.mkdir(parents=True, exist_ok=True)

    names = write_data_yml(args.dataset, args.output, args.names_from)
    if class_map:
        bad_targets = [d for d in class_map.values() if d >= len(names)]
        if bad_targets:
            raise SystemExit(f"class-map targets {bad_targets} exceed vocab size {len(names)}")

    total_counts: dict[int, int] = {}
    total_malformed = 0
    for split in SPLITS:
        frames, missing, malformed, counts = convert_split(
            args.dataset / split, args.output / split, class_map, args.skip_images
        )
        for cls, n in counts.items():
            total_counts[cls] = total_counts.get(cls, 0) + n
        total_malformed += malformed
        miss = f", {missing} missing images" if missing else ""
        bad = f", {malformed} malformed rows" if malformed else ""
        print(f"{split}: {frames} frames{miss}{bad}")

    print("instances per class:")
    for cls, name in enumerate(names):
        print(f"  {cls} {name:16s} {total_counts.get(cls, 0)}")
    if total_malformed:
        print(f"WARNING: {total_malformed} malformed rows dropped")
    print(f"output: {args.output}")


if __name__ == "__main__":
    main()
