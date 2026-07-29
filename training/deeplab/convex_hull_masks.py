"""Regenerate a segmask dataset with every mask blob replaced by its convex hull.

Each `*_mask.png` is read as class IDs (0 = background). For every non-background
class, each connected blob is replaced by the convex hull of that blob, so a
mask with three separate blobs of one class comes out as three separate hulls.

Speck blobs are dropped before hulling: any blob whose pixel area is at or below
`--min-blob-fraction` of the largest blob of its class in that image is deleted
outright rather than hulled. Otherwise a stray handful of pixels would survive as
its own hull.

That relative test alone deletes real regions when an occluder splits the field
into uneven pieces: a robot across the middle of the frame can leave one side at
9% of the other, which is a genuine half of the field, not a speck. So a blob also
survives if it covers at least `--min-blob-image-fraction` of the whole image
regardless of how it compares to the largest blob. Measured on the field corpus,
runner-up blobs that survive the relative test start around 0.36% of the image,
while the median dropped one is 0.08%, so the absolute floor separates the two
cleanly.

Hulls are painted largest-area first, so a small hull that lands inside a big one
stays visible. Pixels from the surviving original blobs are stamped back on top
afterwards, so hull expansion can only ever claim background pixels, never
overwrite an existing annotation.

Everything else in the dataset is reproduced as-is: source images are hardlinked
(copied if the output lands on another filesystem), metadata files are copied,
symlinks are recreated as symlinks.

Usage:
    python convex_hull_masks.py /path/to/dataset -o /path/to/output
    python convex_hull_masks.py /path/to/dataset -o /path/to/output --samples 500

Samples are before/after comparison renders written outside the output dataset
(default `<output>_samples`) so they do not pollute the dataset tree.
"""

from __future__ import annotations

import argparse
import json
import os
import random
import shutil
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from tqdm import tqdm

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp"}
MASK_SUFFIX = "_mask.png"
STATE_FILENAME = "validation_state.json"

PANEL_WIDTH = 960
OVERLAY_ALPHA = 0.45
ORIGINAL_COLOR = (0, 255, 0)  # BGR: green, pixels labelled in the source mask
ADDED_COLOR = (0, 0, 255)  # BGR: red, pixels the hull added
DROPPED_COLOR = (255, 0, 0)  # BGR: blue, pixels of blobs removed as specks
HULL_OUTLINE_COLOR = (0, 255, 255)  # BGR: yellow


@dataclass
class MaskStats:
    """Per-mask summary used for the aggregate report."""

    blobs_kept: int
    blobs_dropped: int
    blobs_rescued: int
    pixels_before: int
    pixels_after: int
    pixels_dropped: int
    sample_written: bool = False


def is_mask(path: Path) -> bool:
    return path.name.endswith(MASK_SUFFIX)


def load_validated_keys(output: Path) -> set[str]:
    """Return the image keys that already carry a verdict in the output dataset."""
    state_path = output / STATE_FILENAME
    if not state_path.exists():
        return set()
    with open(state_path, "r") as handle:
        state = json.load(handle)
    return {key for key, verdict in state.items() if verdict in ("pass", "fail")}


def mask_state_key(dataset: Path, mask_rel: Path) -> str | None:
    """Map a mask's relative path to the validator's key for its image, if it exists."""
    image_path = find_source_image(dataset / mask_rel)
    if image_path is None:
        return None
    return str(image_path.relative_to(dataset))


def drop_validated_masks(masks: list[Path], dataset: Path, output: Path) -> list[Path]:
    """Keep only the masks whose image has no verdict yet in the output dataset."""
    validated = load_validated_keys(output)
    if not validated:
        print(f"No verdicts found in {output / STATE_FILENAME}; regenerating everything")
        return masks

    remaining = [rel for rel in masks if mask_state_key(dataset, rel) not in validated]
    print(
        f"Skipping {len(masks) - len(remaining)} masks already validated in "
        f"{output / STATE_FILENAME}; regenerating {len(remaining)}"
    )
    return remaining


def prepare_samples(
    args: argparse.Namespace, output: Path, mask_count: int
) -> tuple[Path | None, set[int]]:
    """Resolve the sample directory and pick the random sample indices."""
    if args.samples <= 0 and not args.sample_rescued:
        return None, set()

    sample_dir = (
        args.sample_dir.expanduser()
        if args.sample_dir
        else output.with_name(output.name + "_samples")
    )
    if sample_dir.resolve() == output.resolve() or output in sample_dir.resolve().parents:
        print("Error: sample directory must live outside the output dataset", file=sys.stderr)
        sys.exit(1)

    rng = random.Random(args.seed)
    sampled = set(rng.sample(range(mask_count), min(max(args.samples, 0), mask_count)))
    sample_dir.mkdir(parents=True, exist_ok=True)
    return sample_dir, sampled


def convex_hull_mask(
    mask: np.ndarray, min_blob_fraction: float, min_blob_image_fraction: float
) -> tuple[np.ndarray, MaskStats]:
    """Drop speck blobs, then replace each surviving blob with its convex hull.

    A blob is dropped when its pixel area is at or below `min_blob_fraction` of the
    largest blob of the same class in this mask AND it covers less than
    `min_blob_image_fraction` of the image. The second test keeps the smaller side
    of a field that an occluder has split unevenly.
    """
    hulls: list[tuple[float, int, np.ndarray]] = []
    kept = np.zeros(mask.shape, dtype=bool)
    blobs_dropped = 0
    blobs_rescued = 0
    area_floor = mask.shape[0] * mask.shape[1] * min_blob_image_fraction

    for class_id in np.unique(mask):
        if class_id == 0:
            continue
        binary = (mask == class_id).astype(np.uint8)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
        if count < 2:
            continue

        areas = stats[1:, cv2.CC_STAT_AREA]
        threshold = areas.max() * min_blob_fraction
        for index, area in enumerate(areas, start=1):
            if area <= threshold:
                if area < area_floor:
                    blobs_dropped += 1
                    continue
                blobs_rescued += 1

            # Crop to the blob's bounding box before extracting points; comparing
            # the full label image per blob is the slow way to do this.
            x = stats[index, cv2.CC_STAT_LEFT]
            y = stats[index, cv2.CC_STAT_TOP]
            w = stats[index, cv2.CC_STAT_WIDTH]
            h = stats[index, cv2.CC_STAT_HEIGHT]
            blob = labels[y : y + h, x : x + w] == index
            kept[y : y + h, x : x + w] |= blob

            points = cv2.findNonZero(blob.astype(np.uint8))
            hull = cv2.convexHull(points) + np.array([x, y], dtype=points.dtype)
            hulls.append((cv2.contourArea(hull), int(class_id), hull))

    out = np.zeros_like(mask)
    for _, class_id, hull in sorted(hulls, key=lambda entry: -entry[0]):
        cv2.fillConvexPoly(out, hull, class_id)

    out[kept] = mask[kept]
    labelled = mask != 0
    return out, MaskStats(
        blobs_kept=len(hulls),
        blobs_dropped=blobs_dropped,
        blobs_rescued=blobs_rescued,
        pixels_before=int(np.count_nonzero(labelled)),
        pixels_after=int(np.count_nonzero(out)),
        pixels_dropped=int(np.count_nonzero(labelled & ~kept)),
    )


def read_mask(path: Path) -> np.ndarray | None:
    mask = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if mask is None:
        return None
    return mask[:, :, 0] if mask.ndim == 3 else mask


def overlay(image: np.ndarray, selection: np.ndarray, color: tuple[int, int, int]) -> None:
    """Blend a flat color into `image` wherever `selection` is set, in place."""
    if not selection.any():
        return
    tinted = image[selection].astype(np.float32)
    blended = tinted * (1.0 - OVERLAY_ALPHA) + np.array(color, dtype=np.float32) * OVERLAY_ALPHA
    image[selection] = blended.astype(np.uint8)


def label_panel(panel: np.ndarray, text: str) -> None:
    cv2.rectangle(panel, (0, 0), (panel.shape[1], 34), (0, 0, 0), -1)
    cv2.putText(panel, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)


def render_sample(image_path: Path, original: np.ndarray, hulled: np.ndarray, dest: Path) -> None:
    """Write a side-by-side before/after render for eyeballing the transform."""
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        return
    if image.shape[:2] != original.shape[:2]:
        image = cv2.resize(
            image, (original.shape[1], original.shape[0]), interpolation=cv2.INTER_LINEAR
        )

    before = image.copy()
    overlay(before, original != 0, ORIGINAL_COLOR)

    after = image.copy()
    overlay(after, (hulled != 0) & (original != 0), ORIGINAL_COLOR)
    overlay(after, (hulled != 0) & (original == 0), ADDED_COLOR)
    overlay(after, (hulled == 0) & (original != 0), DROPPED_COLOR)
    for class_id in np.unique(hulled):
        if class_id == 0:
            continue
        binary = (hulled == class_id).astype(np.uint8)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(after, contours, -1, HULL_OUTLINE_COLOR, 2)

    scale = PANEL_WIDTH / before.shape[1]
    size = (PANEL_WIDTH, max(1, int(round(before.shape[0] * scale))))
    before = cv2.resize(before, size, interpolation=cv2.INTER_AREA)
    after = cv2.resize(after, size, interpolation=cv2.INTER_AREA)

    label_panel(before, "original mask (green)")
    label_panel(
        after, "convex hulls (green = kept, red = added, blue = dropped, yellow = hull edge)"
    )

    separator = np.full((size[1], 4, 3), 255, dtype=np.uint8)
    dest.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(dest), np.hstack([before, separator, after]), [cv2.IMWRITE_JPEG_QUALITY, 90])


@dataclass
class MaskJob:
    source: Path
    dest: Path
    sample_dest: Path | None
    is_random_sample: bool
    sample_rescued: bool
    min_blob_fraction: float
    min_blob_image_fraction: float


def process_mask(job: MaskJob) -> MaskStats | None:
    """Hull one mask, write it to `dest`, and optionally render a sample."""
    cv2.setNumThreads(1)

    original = read_mask(job.source)
    if original is None:
        print(f"  WARNING: could not read {job.source}, skipping", file=sys.stderr)
        return None

    hulled, stats = convex_hull_mask(original, job.min_blob_fraction, job.min_blob_image_fraction)
    job.dest.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(job.dest), hulled)

    wanted = job.is_random_sample or (job.sample_rescued and stats.blobs_rescued > 0)
    if job.sample_dest is not None and wanted:
        image_path = find_source_image(job.source)
        if image_path is not None:
            render_sample(image_path, original, hulled, job.sample_dest)
            stats.sample_written = True

    return stats


def find_source_image(mask_path: Path) -> Path | None:
    stem = mask_path.name[: -len(MASK_SUFFIX)]
    for suffix in (".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp"):
        candidate = mask_path.with_name(stem + suffix)
        if candidate.exists():
            return candidate
    return None


def replicate_entry(source: Path, dest: Path) -> None:
    """Reproduce a non-mask file: symlinks as symlinks, images hardlinked, rest copied."""
    dest.parent.mkdir(parents=True, exist_ok=True)
    if dest.exists() or dest.is_symlink():
        return

    if source.is_symlink():
        os.symlink(os.readlink(source), dest)
        return

    # Images are never rewritten in place by the tooling, so sharing an inode is
    # safe and free. Metadata files (validation_state.json and friends) do get
    # rewritten, so they must be real copies.
    if source.suffix.lower() in IMAGE_SUFFIXES:
        try:
            os.link(source, dest)
            return
        except OSError:
            pass
    shutil.copy2(source, dest)


def walk_dataset(root: Path) -> tuple[list[Path], list[Path]]:
    """Return (mask files, everything else) as paths relative to `root`."""
    masks: list[Path] = []
    others: list[Path] = []

    def visit(directory: Path) -> None:
        for entry in sorted(directory.iterdir()):
            if entry.is_symlink():
                others.append(entry.relative_to(root))
            elif entry.is_dir():
                visit(entry)
            elif is_mask(entry):
                masks.append(entry.relative_to(root))
            else:
                others.append(entry.relative_to(root))

    visit(root)
    return masks, others


def report(stats: list[MaskStats], sample_dir: Path | None, sample_count: int) -> None:
    if not stats:
        return
    kept = np.array([s.blobs_kept for s in stats], dtype=np.float64)
    dropped = np.array([s.blobs_dropped for s in stats], dtype=np.float64)
    rescued = np.array([s.blobs_rescued for s in stats], dtype=np.float64)
    before = np.array([s.pixels_before for s in stats], dtype=np.float64)
    after = np.array([s.pixels_after for s in stats], dtype=np.float64)
    dropped_pixels = np.array([s.pixels_dropped for s in stats], dtype=np.float64)
    growth = np.divide(after, before, out=np.ones_like(after), where=before > 0)

    print(
        "Blobs kept per mask:  "
        f"mean {kept.mean():.2f}  median {np.median(kept):.0f}  max {kept.max():.0f}"
    )
    print(
        f"Blobs dropped as specks:  {int(dropped.sum())} total across "
        f"{int((dropped > 0).sum())} / {len(stats)} masks  "
        f"({dropped_pixels.sum() / max(before.sum(), 1) * 100:.3f}% of labelled pixels)"
    )
    print(
        "Labelled area growth:  "
        f"mean {growth.mean():.3f}x  median {np.median(growth):.3f}x  max {growth.max():.3f}x"
    )
    print(
        f"Blobs rescued by area floor: {int(rescued.sum())} total across "
        f"{int((rescued > 0).sum())} / {len(stats)} masks"
    )
    print(f"Masks that shrank: {int((growth < 0.9999).sum())} / {len(stats)}")
    if sample_dir is not None:
        print(f"Wrote {sample_count} comparison samples to {sample_dir}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Regenerate a segmask dataset with each mask blob replaced by its convex hull"
    )
    parser.add_argument("dataset", type=Path, help="Source dataset directory")
    parser.add_argument("-o", "--output", type=Path, required=True, help="Output dataset directory")
    parser.add_argument(
        "--min-blob-fraction",
        type=float,
        default=0.10,
        help=(
            "Drop blobs whose pixel area is at or below this fraction of the largest blob "
            "of the same class in that mask (default: 0.10)"
        ),
    )
    parser.add_argument(
        "--min-blob-image-fraction",
        type=float,
        default=0.01,
        help=(
            "Keep a blob regardless of the relative test when it covers at least this "
            "fraction of the whole image. Preserves the smaller side of a field that an "
            "occluder split unevenly (default: 0.01)"
        ),
    )
    parser.add_argument(
        "--skip-validated",
        action="store_true",
        help=(
            "Leave masks alone when their image already has a verdict in "
            "<output>/validation_state.json. Only unvalidated masks are regenerated."
        ),
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=500,
        help="Number of random before/after comparison renders to write (0 disables)",
    )
    parser.add_argument(
        "--sample-rescued",
        action="store_true",
        help="Also render a comparison for every mask where the area floor rescued a blob",
    )
    parser.add_argument(
        "--sample-dir",
        type=Path,
        default=None,
        help="Where to write comparison renders (default: <output>_hull_samples)",
    )
    parser.add_argument("--seed", type=int, default=0, help="Seed for sample selection")
    parser.add_argument(
        "-j",
        "--jobs",
        type=int,
        default=os.cpu_count() or 1,
        help="Worker processes",
    )
    args = parser.parse_args()

    dataset: Path = args.dataset.expanduser()
    output: Path = args.output.expanduser()

    if not dataset.is_dir():
        print(f"Error: dataset directory not found: {dataset}", file=sys.stderr)
        sys.exit(1)
    if output.resolve() == dataset.resolve():
        print("Error: output must differ from the source dataset", file=sys.stderr)
        sys.exit(1)

    masks, others = walk_dataset(dataset)
    if not masks:
        print(f"No *{MASK_SUFFIX} files found in {dataset}", file=sys.stderr)
        sys.exit(1)
    print(f"Found {len(masks)} masks and {len(others)} other files in {dataset}")

    if args.skip_validated:
        masks = drop_validated_masks(masks, dataset, output)
        if not masks:
            print("Nothing left to regenerate.")
            return

    sample_dir, sampled = prepare_samples(args, output, len(masks))

    for rel in tqdm(others, desc="Replicating images and metadata"):
        replicate_entry(dataset / rel, output / rel)

    jobs = [
        MaskJob(
            source=dataset / rel,
            dest=output / rel,
            sample_dest=(sample_dir / f"{rel.name[: -len(MASK_SUFFIX)]}_hullcheck.jpg")
            if sample_dir is not None
            else None,
            is_random_sample=index in sampled,
            sample_rescued=args.sample_rescued,
            min_blob_fraction=args.min_blob_fraction,
            min_blob_image_fraction=args.min_blob_image_fraction,
        )
        for index, rel in enumerate(masks)
    ]

    stats: list[MaskStats] = []
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        for result in tqdm(
            pool.map(process_mask, jobs, chunksize=16), total=len(jobs), desc="Hulling masks"
        ):
            if result is not None:
                stats.append(result)

    print(f"Done. Wrote {len(stats)} masks to {output}")
    report(stats, sample_dir, sum(1 for s in stats if s.sample_written))


if __name__ == "__main__":
    main()
