"""Pool a full real dataset with a fraction of a synthetic dataset into one flat YOLO tree.

Recipe C (see docs/experiments/perception_performance/experiment_runbook.md) has no committed
script; this is it, written for the data_epoch_min experiment's Phase B synthetic scene-variation
probe. Every real frame is always included; synthetic frames are subsampled by --synth-fraction,
selected either in image-numbered (filename) order or randomly. Filenames carry a
"...T17-01-18-000050..." timestamp + frame index, so filename order IS render-sequence order:
--synth-order sequential is scene-correlated, --synth-order random spreads scene coverage. The gap
between the two accuracy-vs-fraction curves at a fixed real count answers "how much synthetic scene
variation do I need".

Output is a flat images/+labels/ tree ready for:
    split_yolo_dataset.py  ->  validate_yolo_integrity.py --strict

Keep synthetic out of val: pool into the train split only, and build a real-only val split
separately (grade always on the external eval, never same-corpus val).

Only the image (.jpg/.png/...) and its stem-matched .txt label are linked. Any .npy / labels.cache
next to the images are ultralytics cache="disk" artifacts (derived) and are skipped; training
regenerates them.

Example:
    python pool_datasets.py \
        --real ../data/real_our_robots \
        --synth ../data/all_robot_keypoints/train \
        --out ../data/phaseB_r100_s25_seq \
        --synth-fraction 0.25 --synth-order sequential
"""

import argparse
import os
import shutil
from pathlib import Path
from random import Random

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}
# Derived files that sit next to images (ultralytics cache="disk"); never pooled.
DERIVED_EXTS = {".npy", ".cache"}


def resolve_dirs(source: Path) -> tuple[Path, Path]:
    """Return (images_dir, labels_dir): an images/+labels/ root, else a flat single directory."""
    images = source / "images"
    labels = source / "labels"
    if images.is_dir() and labels.is_dir():
        return images, labels
    return source, source


def index_by_stem(directory: Path) -> dict[str, list[Path]]:
    """Map each file stem to the files sharing it (an image and its sidecars/label)."""
    index: dict[str, list[Path]] = {}
    for path in directory.iterdir():
        if path.is_file():
            index.setdefault(path.stem, []).append(path)
    return index


def list_frames(images_dir: Path) -> list[Path]:
    """Image files in images_dir, sorted by filename (== render-sequence order)."""
    frames = [p for p in images_dir.iterdir() if p.is_file() and p.suffix.lower() in IMAGE_EXTS]
    return sorted(frames, key=lambda p: p.name)


def select_synth(frames: list[Path], fraction: float, order: str, seed: int) -> list[Path]:
    """Choose round(fraction * N) synthetic frames, sequentially or with a seeded shuffle."""
    if not 0.0 <= fraction <= 1.0:
        raise ValueError(f"--synth-fraction must be in [0, 1], got {fraction}")
    keep = round(fraction * len(frames))
    if order == "sequential":
        return frames[:keep]
    # random: seeded, reproducible, comparable to the sequential arm at the same fraction.
    shuffled = list(frames)
    Random(seed).shuffle(shuffled)
    return shuffled[:keep]


def prepare_output(out: Path, overwrite: bool) -> tuple[Path, Path]:
    """Create (out/images, out/labels), clearing them first only when --overwrite is set."""
    out_images = out / "images"
    out_labels = out / "labels"
    for directory in (out_images, out_labels):
        if directory.exists() and any(directory.iterdir()):
            if not overwrite:
                raise SystemExit(
                    f"{directory} is not empty. Pass --overwrite to replace it, or choose a fresh "
                    "--out (pooling into a dirty tree would mix arms)."
                )
            shutil.rmtree(directory)
        directory.mkdir(parents=True, exist_ok=True)
    return out_images, out_labels


def link_pairs(
    frames: list[Path],
    images_dir: Path,
    labels_dir: Path,
    label_index: dict[str, list[Path]],
    out_images: Path,
    out_labels: Path,
    prefix: str,
) -> tuple[int, int]:
    """Hardlink each frame's image + .txt label into the output tree with a source prefix.

    Returns (linked, skipped_no_label).
    """
    flat = images_dir == labels_dir
    linked = 0
    skipped = 0
    for frame in frames:
        labels = [
            p
            for p in label_index.get(frame.stem, [])
            if p.suffix.lower() not in DERIVED_EXTS
            and (not flat or p.suffix.lower() not in IMAGE_EXTS)
        ]
        if not labels:
            print(f"Skipping {frame.name}: no label found.")
            skipped += 1
            continue
        hardlink(frame, out_images / f"{prefix}__{frame.name}")
        for label in labels:
            hardlink(label, out_labels / f"{prefix}__{label.name}")
        linked += 1
    return linked, skipped


def hardlink(src: Path, dst: Path) -> None:
    """os.link src -> dst, with a clear message if the two are on different filesystems."""
    try:
        os.link(src, dst)
    except FileExistsError:
        raise SystemExit(f"Refusing to overwrite existing {dst} (filename collision).")
    except OSError as error:
        raise SystemExit(
            f"Hardlink {src} -> {dst} failed ({error}). Recipe C requires the sources and --out to "
            "be on the same filesystem; move --out onto the same volume as the datasets."
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Pool a full real dataset with a fraction of a synthetic dataset (Recipe C) for the "
            "data_epoch_min synthetic scene-variation probe. Hardlinks; same filesystem required."
        ),
    )
    parser.add_argument(
        "--real", required=True, type=Path, help="Real source (images/+labels/ or flat)"
    )
    parser.add_argument(
        "--synth", required=True, type=Path, help="Synthetic source (images/+labels/ or flat)"
    )
    parser.add_argument(
        "--out", required=True, type=Path, help="Output pool dir (gets images/+labels/)"
    )
    parser.add_argument(
        "--synth-fraction",
        required=True,
        type=float,
        help="Fraction of synthetic frames to include, in [0, 1]. Real is always included in full.",
    )
    parser.add_argument(
        "--synth-order",
        required=True,
        choices=["sequential", "random"],
        help="sequential = filename/render-sequence order (scene-correlated); random = seeded.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=0,
        help="Seed for --synth-order random (printed for reproducibility). Ignored for sequential.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Clear a non-empty --out/images and --out/labels before pooling.",
    )
    args = parser.parse_args()

    real_images_dir, real_labels_dir = resolve_dirs(args.real)
    synth_images_dir, synth_labels_dir = resolve_dirs(args.synth)

    real_frames = list_frames(real_images_dir)
    synth_frames_all = list_frames(synth_images_dir)
    synth_frames = select_synth(synth_frames_all, args.synth_fraction, args.synth_order, args.seed)

    out_images, out_labels = prepare_output(args.out, args.overwrite)

    real_label_index = index_by_stem(real_labels_dir)
    synth_label_index = index_by_stem(synth_labels_dir)

    real_linked, real_skipped = link_pairs(
        real_frames,
        real_images_dir,
        real_labels_dir,
        real_label_index,
        out_images,
        out_labels,
        "real",
    )
    synth_linked, synth_skipped = link_pairs(
        synth_frames,
        synth_images_dir,
        synth_labels_dir,
        synth_label_index,
        out_images,
        out_labels,
        "synth",
    )

    print(
        f"Pooled into {args.out}: real {real_linked} (skipped {real_skipped}), "
        f"synth {synth_linked}/{len(synth_frames_all)} "
        f"(f={args.synth_fraction}, order={args.synth_order}, seed={args.seed}; "
        f"skipped {synth_skipped})."
    )
    print(
        "Next: split_yolo_dataset.py <out>/images <out>/labels <out> -t 0.9 -v 0.1 "
        "&& validate_yolo_integrity.py <out> --strict  (keep val real-only)"
    )


if __name__ == "__main__":
    main()
