"""Split a flat images/ + labels/ pair into train/val/test directories.

Note this is a **frame-level random** split. For anything graded on generalization, prefer
``split_by_scene.py``: consecutive frames of one recording are near-duplicates, so a random split
puts near-copies of the training frames into val and val stops measuring anything useful.
"""

import argparse
import random
import shutil
from pathlib import Path

SPLITS = ("train", "val", "test")


def make_split_structure(dataset_root: Path, splits: tuple[str, ...]) -> None:
    """Create empty images/ + labels/ under each requested split, clearing any previous contents."""
    for subdir in splits:
        for subsubdir in ("images", "labels"):
            subdir_path = dataset_root / subdir / subsubdir
            shutil.rmtree(subdir_path, ignore_errors=True)
            subdir_path.mkdir(parents=True)


def split_counts(total: int, train: float, val: float, test: float) -> dict[str, int]:
    """Frame counts per split that sum to exactly ``total``.

    ``int()`` truncates, so ``int(0.9 * N) + int(0.1 * N)`` can fall one short of ``N`` (it does for
    N = 31,465). The old code handed that remainder to whatever was left over, which put a stray
    image in test even when no test split was asked for. Here the remainder goes to **train** --
    never silently into a split the caller sized at zero.
    """
    counts = {
        "train": int(train * total),
        "val": int(val * total),
        "test": int(test * total),
    }
    counts["train"] += total - sum(counts.values())
    return counts


def build_arg_parser() -> argparse.ArgumentParser:
    """Construct the command line argument parser."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("images", help="Path to images")
    parser.add_argument("labels", help="Path to labels")
    parser.add_argument("output", help="Output path")
    parser.add_argument("-t", "--train", type=float, help="Train fraction", default=0.9)
    parser.add_argument("-v", "--val", type=float, help="Validation fraction", default=0.1)
    parser.add_argument(
        "--test",
        type=float,
        default=0.0,
        help="Test fraction (default 0.0 = no test split, and the directory is not created)",
    )
    parser.add_argument(
        "-n",
        "--num-images",
        type=int,
        help="Number of images to include in the dataset",
        default=None,
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=0,
        help="Shuffle seed. Fixed by default so a split is reproducible; the previous behaviour "
        "was unseeded, so re-running produced a different split every time.",
    )
    return parser


def main() -> None:
    """Split a flat image/label pair into train/val/test directories."""
    args = build_arg_parser().parse_args()

    total_fraction = args.train + args.val + args.test
    if abs(total_fraction - 1.0) > 1e-6:
        raise SystemExit(
            f"train + val + test must sum to 1.0, got {total_fraction:.6f} "
            f"({args.train} + {args.val} + {args.test})"
        )

    output_path = Path(args.output)
    all_image_paths = sorted(Path(args.images).glob("*.jpg"))
    stem_to_annotation_map = {path.stem: path for path in Path(args.labels).glob("*.txt")}

    selected = len(all_image_paths) if args.num_images is None else args.num_images
    selected = min(selected, len(all_image_paths))

    random.Random(args.seed).shuffle(all_image_paths)
    all_image_paths = all_image_paths[:selected]

    counts = split_counts(selected, args.train, args.val, args.test)
    active = tuple(name for name in SPLITS if counts[name] > 0)
    make_split_structure(output_path, active)

    print(
        f"Splitting {selected} images (seed {args.seed}): "
        + ", ".join(f"{name}={counts[name]}" for name in SPLITS)
    )

    start = 0
    missing = 0
    for name in SPLITS:
        chunk = all_image_paths[start : start + counts[name]]
        start += counts[name]
        for image_path in chunk:
            annotation_path = stem_to_annotation_map.get(image_path.stem)
            if annotation_path is None:
                print(f"Skipping {image_path.stem}. No annotation.")
                missing += 1
                continue
            shutil.copyfile(image_path, output_path / name / "images" / image_path.name)
            shutil.copyfile(annotation_path, output_path / name / "labels" / annotation_path.name)

    if missing:
        print(f"{missing} image(s) had no annotation and were skipped")
    print(f"Wrote {', '.join(active)} to {output_path}")


if __name__ == "__main__":
    main()
