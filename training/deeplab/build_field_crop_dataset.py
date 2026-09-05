"""Build the field-cropped training corpus for arm E of input_resolution_plan.md.

Arm E asks whether spending the input tensor on the field alone beats spending it on the
whole frame. At deployment the field is already available -- `field_filter.track_field`
runs before perception and is computed once at init, not per frame -- so cropping to it
costs arithmetic. The training corpus has no depth, so the field has to come from the
DeepLab RGB model instead.

Two stages, so the expensive one runs once:

  masks  Run DeepLab over every image and record the field bounding box in normalized
         frame coordinates. GPU-bound, a few minutes over 32k images. Cached to JSON.
  crop   Read that JSON, crop each image to the box plus a margin, and rewrite the labels.
         CPU-bound and cheap, so the margin can be re-tuned without touching the GPU.

Unlike the anisotropic-stretch arm, a crop *does* move normalized YOLO coordinates: every
box is re-expressed against the cropped frame and clipped to it.

    python training/deeplab/build_field_crop_dataset.py masks \
        --model ../../data/models/field_deeplabv3p_r50_2026-07-29.pth \
        --src ../data/nhrl_robots_bbox_2class --out field_boxes.json
    python training/deeplab/build_field_crop_dataset.py crop \
        --src ../data/nhrl_robots_bbox_2class --dst ../data/nhrl_robots_bbox_2class_fieldcrop \
        --boxes field_boxes.json --margin 0.10
"""

import argparse
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import cv2
import numpy as np
import torch
from load_deeplabv3 import common_transforms, load_model
from tqdm import tqdm

FLOOR_CLASS = 1
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png"}
SPLITS = ("train", "val")
# A clipped box keeping less than this share of its original area is dropped rather than
# taught as a sliver: a robot mostly outside the crop is not what the box was annotating.
MIN_VISIBLE_AREA = 0.25


def list_images(root: Path) -> list[Path]:
    return [p for p in sorted(root.iterdir()) if p.suffix.lower() in IMAGE_SUFFIXES]


def field_box(mask: np.ndarray) -> tuple[float, float, float, float] | None:
    """Field bounding box as (x0, y0, x1, y1) fractions of the frame, or None if empty."""
    if not mask.any():
        return None
    ys, xs = np.nonzero(mask)
    h, w = mask.shape
    return (
        float(xs.min()) / w,
        float(ys.min()) / h,
        float(xs.max() + 1) / w,
        float(ys.max() + 1) / h,
    )


def stage_masks(args: argparse.Namespace) -> None:
    device = torch.device(args.device or ("cuda" if torch.cuda.is_available() else "cpu"))
    model, cfg = load_model(args.model, device)
    transform = common_transforms(pad_size=cfg.pad_size)
    boxes: dict[str, list[float] | None] = {}

    for split in SPLITS:
        images = list_images(args.src / split / "images")
        for start in tqdm(range(0, len(images), args.batch), desc=split):
            chunk = images[start : start + args.batch]
            tensors, kept = [], []
            for path in chunk:
                frame = cv2.imread(str(path))
                if frame is None:
                    boxes[f"{split}/{path.name}"] = None
                    continue
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                resized = cv2.resize(
                    rgb, (cfg.image_size, cfg.image_size), interpolation=cv2.INTER_LINEAR
                )
                tensors.append(transform(resized))
                kept.append(path)
            if not tensors:
                continue
            batch = torch.stack(tensors).to(device)
            with torch.no_grad():
                pred = torch.argmax(model(batch), dim=1).cpu().numpy()
            if cfg.pad_size > 0:
                pred = pred[:, cfg.pad_size : -cfg.pad_size, cfg.pad_size : -cfg.pad_size]
            for path, single in zip(kept, pred):
                box = field_box((single == FLOOR_CLASS).astype(np.uint8))
                boxes[f"{split}/{path.name}"] = list(box) if box else None

    args.out.write_text(json.dumps(boxes))
    missing = sum(1 for v in boxes.values() if v is None)
    print(f"wrote {args.out}: {len(boxes)} frames, {missing} with no field detected")


def transform_labels(lines: list[str], x0: float, y0: float, cw: float, ch: float) -> list[str]:
    """Re-express normalized YOLO boxes against a crop, clipping and dropping slivers."""
    out = []
    for line in lines:
        parts = line.split()
        if len(parts) < 5:
            continue
        cls, cx, cy, bw, bh = parts[0], *(float(v) for v in parts[1:5])
        left, right = cx - bw / 2, cx + bw / 2
        top, bottom = cy - bh / 2, cy + bh / 2
        # Into crop-relative fractions, then clip to the crop.
        left, right = (left - x0) / cw, (right - x0) / cw
        top, bottom = (top - y0) / ch, (bottom - y0) / ch
        clipped = (max(left, 0.0), max(top, 0.0), min(right, 1.0), min(bottom, 1.0))
        nw, nh = clipped[2] - clipped[0], clipped[3] - clipped[1]
        if nw <= 0 or nh <= 0:
            continue
        if (nw * nh) / ((right - left) * (bottom - top)) < MIN_VISIBLE_AREA:
            continue
        out.append(f"{cls} {clipped[0] + nw / 2:.6f} {clipped[1] + nh / 2:.6f} {nw:.6f} {nh:.6f}")
    return out


def crop_one(job: tuple[str, str, str, str, list[float] | None, float]) -> tuple[int, int]:
    img_path, lbl_path, img_out, lbl_out, box, margin = job
    frame = cv2.imread(img_path)
    if frame is None:
        return 0, 0
    h, w = frame.shape[:2]

    if box is None:
        # No field found: keep the whole frame rather than inventing a crop.
        x0, y0, x1, y1 = 0.0, 0.0, 1.0, 1.0
    else:
        x0, y0, x1, y1 = box
        mx, my = (x1 - x0) * margin, (y1 - y0) * margin
        x0, y0 = max(x0 - mx, 0.0), max(y0 - my, 0.0)
        x1, y1 = min(x1 + mx, 1.0), min(y1 + my, 1.0)

    px0, py0 = int(round(x0 * w)), int(round(y0 * h))
    px1, py1 = max(int(round(x1 * w)), px0 + 1), max(int(round(y1 * h)), py0 + 1)
    cv2.imwrite(img_out, frame[py0:py1, px0:px1], [cv2.IMWRITE_JPEG_QUALITY, 95])

    lines = Path(lbl_path).read_text().splitlines() if Path(lbl_path).exists() else []
    kept = transform_labels(lines, px0 / w, py0 / h, (px1 - px0) / w, (py1 - py0) / h)
    Path(lbl_out).write_text("\n".join(kept) + ("\n" if kept else ""))
    return len(lines), len(kept)


def stage_crop(args: argparse.Namespace) -> None:
    boxes = json.loads(args.boxes.read_text())
    jobs = []
    for split in SPLITS:
        img_dst, lbl_dst = args.dst / split / "images", args.dst / split / "labels"
        for d in (img_dst, lbl_dst):
            d.mkdir(parents=True, exist_ok=True)
        for img in list_images(args.src / split / "images"):
            jobs.append(
                (
                    str(img),
                    str(args.src / split / "labels" / f"{img.stem}.txt"),
                    str(img_dst / f"{img.stem}.jpg"),
                    str(lbl_dst / f"{img.stem}.txt"),
                    boxes.get(f"{split}/{img.name}"),
                    args.margin,
                )
            )

    before = after = 0
    with ProcessPoolExecutor(args.workers) as ex:
        for b, a in tqdm(ex.map(crop_one, jobs, chunksize=64), total=len(jobs), desc="crop"):
            before += b
            after += a
    print(f"{len(jobs)} frames, labels {before} -> {after} ({before - after} dropped by the crop)")

    (args.dst / "data.yml").write_text(
        f"path: {args.dst.resolve()}\ntrain: train/images\nval: val/images\n"
        "nc: 2\nnames:\n- robot\n- house_bot\n"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="stage", required=True)

    masks = sub.add_parser("masks", help="run DeepLab and cache field boxes")
    masks.add_argument("--model", type=Path, required=True)
    masks.add_argument("--src", type=Path, required=True)
    masks.add_argument("--out", type=Path, required=True)
    masks.add_argument("--batch", type=int, default=32)
    masks.add_argument("--device", default="")
    masks.set_defaults(func=stage_masks)

    crop = sub.add_parser("crop", help="crop images and rewrite labels")
    crop.add_argument("--src", type=Path, required=True)
    crop.add_argument("--dst", type=Path, required=True)
    crop.add_argument("--boxes", type=Path, required=True)
    crop.add_argument("--margin", type=float, default=0.10, help="fraction of the box, per side")
    crop.add_argument("--workers", type=int, default=16)
    crop.set_defaults(func=stage_crop)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
