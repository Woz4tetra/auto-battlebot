"""Split a segmask dataset into train/val/test splits.

Recursively searches for .jpg and .png images with matching _mask.png files.
Outputs the standard flat directory layout expected by semantic_train.py:

    output/
        train/
            image_001.jpg
            image_001_mask.png
            ...
        val/
            ...
        test/
            ...

Usage:
    python split_segmask_dataset.py /path/to/dataset /path/to/output
    python split_segmask_dataset.py /path/to/dataset /path/to/output -t 0.8 -v 0.1
"""

import argparse
import random
import shutil
from pathlib import Path


def make_split_structure(output_root: Path) -> None:
    for subdir in ("train", "val", "test"):
        subdir_path = output_root / subdir
        shutil.rmtree(subdir_path, ignore_errors=True)
        subdir_path.mkdir(parents=True)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Split a segmask dataset into train/val/test splits"
    )
    parser.add_argument(
        "input",
        help="Path to directory (searched recursively) with images and _mask.png files",
    )
    parser.add_argument("output", help="Output path for split dataset")
    parser.add_argument(
        "-t", "--train", type=float, help="Train fraction (default: 0.9)", default=0.9
    )
    parser.add_argument(
        "-v",
        "--val",
        type=float,
        help="Validation fraction (default: 0.1)",
        default=0.1,
    )
    parser.add_argument(
        "-n",
        "--num-images",
        type=int,
        help="Limit the number of images to include",
        default=None,
    )
    parser.add_argument(
        "--seed", type=int, help="Random seed for reproducibility", default=None
    )
    args = parser.parse_args()

    input_path = Path(args.input)
    output_path = Path(args.output)
    train_frac = args.train
    val_frac = args.val

    if args.seed is not None:
        random.seed(args.seed)

    all_images = sorted(
        p
        for p in input_path.rglob("*")
        if p.suffix.lower() in (".jpg", ".png") and "_mask" not in p.stem
    )
    paired = []
    for img_path in all_images:
        mask_path = img_path.with_name(img_path.stem + "_mask.png")
        if not mask_path.exists():
            print(f"Skipping {img_path.name}: no matching mask")
            continue
        paired.append((img_path, mask_path))

    random.shuffle(paired)

    selected = paired[: args.num_images] if args.num_images is not None else paired
    total = len(selected)
    num_train = int(train_frac * total)
    num_val = int(val_frac * total)

    splits = {
        "train": selected[:num_train],
        "val": selected[num_train : num_train + num_val],
        "test": selected[num_train + num_val :],
    }

    make_split_structure(output_path)

    print(f"Total paired images: {total}")
    for name, items in splits.items():
        print(f"  {name}: {len(items)}")
        dest = output_path / name
        for img_path, mask_path in items:
            shutil.copy2(img_path, dest / img_path.name)
            shutil.copy2(mask_path, dest / mask_path.name)

    print(f"Done. Output written to {output_path}")


if __name__ == "__main__":
    main()
