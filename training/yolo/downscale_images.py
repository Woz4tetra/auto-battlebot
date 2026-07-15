"""Downscale a dataset's images so the longest side <= --max-dim, in place.

Training runs at imgsz=640, so source images larger than ~720px waste decode time
and inflate Ultralytics' RAM-cache size estimate (check_cache_ram uses full-res
bytes), which silently disables the train RAM cache -> GPUs starve on JPEG decode.
Bounding-box/keypoint labels are normalized, so they are unaffected by resizing.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from PIL import Image
from tqdm import tqdm

IMG_EXTS = {".jpg", ".jpeg", ".png"}


def resize_one(args: tuple[Path, int, int]) -> int:
    path, max_dim, quality = args
    try:
        with Image.open(path) as im:
            w, h = im.size
            if max(w, h) <= max_dim:
                return 0
            scale = max_dim / max(w, h)
            out = im.convert("RGB").resize(
                (round(w * scale), round(h * scale)), Image.Resampling.LANCZOS
            )
        out.save(path, "JPEG", quality=quality)  # overwrite (breaks any hardlink -> real file)
        return 1
    except Exception as e:  # noqa: BLE001
        print(f"  skip {path.name}: {e}")
        return 0


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("root", type=Path, help="Dataset root (recurses into */images)")
    p.add_argument("--max-dim", type=int, default=720, help="Longest side after resize")
    p.add_argument("--quality", type=int, default=92, help="JPEG quality")
    p.add_argument("--workers", type=int, default=32)
    args = p.parse_args()

    imgs = [f for f in args.root.rglob("*") if f.suffix.lower() in IMG_EXTS]
    print(f"scanning {len(imgs)} images under {args.root} (max-dim {args.max_dim})")
    work = [(f, args.max_dim, args.quality) for f in imgs]
    resized = 0
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        with tqdm(total=len(imgs), unit="img", desc="downscaling") as bar:
            for r in ex.map(resize_one, work, chunksize=64):
                resized += r
                bar.update(1)
                bar.set_postfix(resized=resized)
    print(f"done: {resized}/{len(imgs)} resized (rest were already <= {args.max_dim})")


if __name__ == "__main__":
    main()
