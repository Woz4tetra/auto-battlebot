"""Build an anisotropically-resized copy of a YOLO dataset, for the stretch arm."""

import argparse
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import cv2
from tqdm import tqdm

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png"}
SPLITS = ("train", "val")


def resize_one(job: tuple[str, str, int, int]) -> bool:
    src, dst, width, height = job
    image = cv2.imread(src)
    if image is None:
        return False
    out = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    return bool(cv2.imwrite(dst, out, [cv2.IMWRITE_JPEG_QUALITY, 95]))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("src", type=Path)
    parser.add_argument("dst", type=Path)
    parser.add_argument("--size", type=int, nargs=2, default=[640, 640], metavar=("H", "W"))
    parser.add_argument("--workers", type=int, default=16)
    parser.add_argument("--names", nargs="+", default=["robot", "house_bot"])
    args = parser.parse_args()

    height, width = args.size
    src, dst = args.src.resolve(), args.dst.resolve()
    jobs = []
    for split in SPLITS:
        img_dst, lbl_dst = dst / split / "images", dst / split / "labels"
        for d in (img_dst, lbl_dst):
            d.mkdir(parents=True, exist_ok=True)
        for img in sorted((src / split / "images").iterdir()):
            if img.suffix.lower() not in IMAGE_SUFFIXES:
                continue
            label = src / split / "labels" / f"{img.stem}.txt"
            if label.exists():
                link = lbl_dst / label.name
                link.unlink(missing_ok=True)
                link.symlink_to(label)
            jobs.append((str(img), str(img_dst / f"{img.stem}.jpg"), width, height))

    failed = 0
    with ProcessPoolExecutor(args.workers) as ex:
        for ok in tqdm(ex.map(resize_one, jobs, chunksize=64), total=len(jobs), desc="resize"):
            failed += not ok
    print(f"{len(jobs)} images resized to {width}x{height}, {failed} failed")

    names = "\n".join(f"- {n}" for n in args.names)
    (dst / "data.yml").write_text(
        f"path: {dst}\ntrain: train/images\nval: val/images\n"
        f"nc: {len(args.names)}\nnames:\n{names}\n"
    )


if __name__ == "__main__":
    main()
