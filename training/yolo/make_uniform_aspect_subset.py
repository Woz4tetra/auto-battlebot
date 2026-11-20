"""Build a uniform-aspect-ratio view of a YOLO dataset, for rectangular training."""

import argparse
from pathlib import Path

from PIL import Image

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png"}


def link_split(src: Path, dst: Path, split: str, target_ar: float | None) -> tuple[int, int]:
    img_src, lbl_src = src / split / "images", src / split / "labels"
    img_dst, lbl_dst = dst / split / "images", dst / split / "labels"
    for d in (img_dst, lbl_dst):
        d.mkdir(parents=True, exist_ok=True)

    kept = dropped = 0
    for img in sorted(img_src.iterdir()):
        if img.suffix.lower() not in IMAGE_SUFFIXES:
            continue
        if target_ar is not None:
            with Image.open(img) as im:
                w, h = im.size
            if abs(w / h - target_ar) > 1e-3:
                dropped += 1
                continue
        label = lbl_src / f"{img.stem}.txt"
        for source, link in ((img, img_dst / img.name), (label, lbl_dst / label.name)):
            if not source.exists():
                continue
            link.unlink(missing_ok=True)
            link.symlink_to(source.resolve())
        kept += 1
    return kept, dropped


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("src", type=Path, help="source dataset directory")
    parser.add_argument("dst", type=Path, help="directory to create the filtered view in")
    parser.add_argument(
        "--aspect", type=float, default=16 / 9, help="width/height to keep (default 16:9)"
    )
    parser.add_argument(
        "--filter-val",
        action="store_true",
        help="also drop odd-aspect val frames. Off by default so val metrics stay comparable "
        "with a square-trained baseline; val never shuffles, so it does not need this.",
    )
    parser.add_argument("--names", nargs="+", default=["robot", "house_bot"])
    args = parser.parse_args()

    for split, filt in (("train", args.aspect), ("val", args.aspect if args.filter_val else None)):
        kept, dropped = link_split(args.src.resolve(), args.dst.resolve(), split, filt)
        print(f"{split}: linked {kept}, dropped {dropped}")

    names = "\n".join(f"- {n}" for n in args.names)
    (args.dst / "data.yml").write_text(
        f"path: {args.dst.resolve()}\ntrain: train/images\nval: val/images\n"
        f"nc: {len(args.names)}\nnames:\n{names}\n"
    )


if __name__ == "__main__":
    main()
