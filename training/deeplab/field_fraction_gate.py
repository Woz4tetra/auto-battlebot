"""Go / no-go gate for the field-crop arm (arm E of input_resolution_plan.md).

Arm E crops each frame to the field before the detector sees it, buying resolution where
the robots are. That only closes the train/deploy domain gap if the field occupies a
*similar* fraction of the frame in both corpora. NHRL's overhead cage cameras are framed on
the cage, so the field may already fill the frame there, while the ZED shoots from inside
the cage and sees much less. If the two fractions differ sharply, cropping teaches the
detector a scale the deployment camera never delivers and arm E should be cut.

Runs the trained DeepLab field model over a sample of both corpora and reports, per corpus,
the fraction of the frame covered by the field mask and by its bounding box (the bbox is
what a crop would actually use).

With `--labels`, it also answers the question that decides the arm: how much margin the crop
needs before it stops clipping annotated robots, and how much zoom survives that margin. A
crop tight enough to zoom is a crop that cuts robots in half; a crop loose enough to keep
them is a crop that covers the frame.

    python training/deeplab/field_fraction_gate.py \
        --model data/models/field_deeplabv3p_r50_2026-07-29.pth \
        --train-images training/data/nhrl_robots_bbox_2class/train/images \
        --eval-root training/data/nhrl_keypoints_eval_test \
        --sample 300 --device cpu
"""

import argparse
import json
import random
from pathlib import Path

import cv2
import numpy as np
import torch
from load_deeplabv3 import common_transforms, load_model

FLOOR_CLASS = 1
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png"}


def predict_field(
    model: torch.nn.Module,
    frame_bgr: np.ndarray,
    transform: torch.nn.Module,
    device: torch.device,
    image_size: int,
    pad_size: int,
) -> np.ndarray:
    """Return the binary field mask at the model's own output resolution."""
    rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    resized = cv2.resize(rgb, (image_size, image_size), interpolation=cv2.INTER_LINEAR)
    tensor = transform(resized).unsqueeze(0).to(device)
    with torch.no_grad():
        pred = torch.argmax(model(tensor), dim=1).squeeze(0).cpu().numpy()
    if pad_size > 0:
        pred = pred[pad_size:-pad_size, pad_size:-pad_size]
    return (pred == FLOOR_CLASS).astype(np.uint8)


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


def measure(mask: np.ndarray) -> tuple[float, float, float]:
    """Return (mask fraction, bbox fraction, bbox aspect ratio) of the frame.

    The mask is square because the model resizes anisotropically to a square input, so
    fractions are scale-free and transfer back to the source frame. The aspect ratio is
    reported in *source* terms by the caller, which knows the frame shape.
    """
    h, w = mask.shape
    area = float(h * w)
    covered = float(mask.sum())
    if covered == 0:
        return 0.0, 0.0, 0.0
    ys, xs = np.nonzero(mask)
    bh = float(ys.max() - ys.min() + 1)
    bw = float(xs.max() - xs.min() + 1)
    return covered / area, (bh * bw) / area, bw / bh


def read_boxes(label_path: Path) -> list[tuple[float, float, float, float]]:
    """Normalized (x0, y0, x1, y1) label boxes, or empty if the file is missing."""
    if not label_path.exists():
        return []
    boxes = []
    for line in label_path.read_text().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        cx, cy, bw, bh = (float(v) for v in parts[1:5])
        boxes.append((cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2))
    return boxes


def margin_retention(
    rows: list[dict[str, float]], margins: tuple[float, ...]
) -> list[dict[str, float]]:
    """For each margin: share of labels surviving the crop, and the zoom the crop still buys.

    `kept` counts boxes retaining at least a quarter of their area (what the dataset builder
    keeps); `whole` counts boxes entirely inside the crop. `zoom` is the linear scale gain on
    the median frame, after the margin is clamped at the frame edge.
    """
    out = []
    for margin in margins:
        total = kept = whole = 0
        areas = []
        for row in rows:
            box = row.get("field_box")
            if box is None:
                continue
            fx0, fy0, fx1, fy1 = box  # type: ignore[misc]
            mx, my = (fx1 - fx0) * margin, (fy1 - fy0) * margin
            cx0, cy0 = max(fx0 - mx, 0.0), max(fy0 - my, 0.0)
            cx1, cy1 = min(fx1 + mx, 1.0), min(fy1 + my, 1.0)
            areas.append((cx1 - cx0) * (cy1 - cy0))
            for bx0, by0, bx1, by1 in row.get("boxes", []):  # type: ignore[union-attr]
                iw = max(0.0, min(bx1, cx1) - max(bx0, cx0))
                ih = max(0.0, min(by1, cy1) - max(by0, cy0))
                visible = (iw * ih) / max((bx1 - bx0) * (by1 - by0), 1e-9)
                total += 1
                kept += visible >= 0.25
                whole += visible >= 0.999
        if not total:
            continue
        out.append(
            {
                "margin": margin,
                "kept": kept / total,
                "whole": whole / total,
                "zoom": 1.0 / float(np.sqrt(np.median(areas))),
                "n_labels": total,
            }
        )
    return out


def sample_images(root: Path, n: int, rng: random.Random) -> list[Path]:
    files = [p for p in sorted(root.rglob("*")) if p.suffix.lower() in IMAGE_SUFFIXES]
    if len(files) <= n:
        return files
    return rng.sample(files, n)


def summarize(name: str, rows: list[dict[str, float]]) -> dict[str, object]:
    if not rows:
        return {"corpus": name, "n": 0}
    mask = np.array([r["mask_frac"] for r in rows])
    bbox = np.array([r["bbox_frac"] for r in rows])
    return {
        "corpus": name,
        "n": len(rows),
        "mask_frac_median": float(np.median(mask)),
        "mask_frac_p10": float(np.percentile(mask, 10)),
        "mask_frac_p90": float(np.percentile(mask, 90)),
        "bbox_frac_median": float(np.median(bbox)),
        "bbox_frac_p10": float(np.percentile(bbox, 10)),
        "bbox_frac_p90": float(np.percentile(bbox, 90)),
        "empty_mask_frames": int((mask == 0).sum()),
    }


def run_corpus(
    name: str,
    images: list[Path],
    model: torch.nn.Module,
    transform: torch.nn.Module,
    device: torch.device,
    image_size: int,
    pad_size: int,
    with_labels: bool = False,
) -> tuple[dict[str, object], list[dict[str, float]]]:
    rows: list[dict[str, float]] = []
    for i, path in enumerate(images):
        frame = cv2.imread(str(path))
        if frame is None:
            continue
        mask = predict_field(model, frame, transform, device, image_size, pad_size)
        mask_frac, bbox_frac, _ = measure(mask)
        row: dict[str, object] = {
            "path": str(path),
            "mask_frac": mask_frac,
            "bbox_frac": bbox_frac,
            "field_box": field_box(mask),
        }
        if with_labels:
            label = path.parent.parent / "labels" / f"{path.stem}.txt"
            row["boxes"] = read_boxes(label)
        rows.append(row)  # type: ignore[arg-type]
        if (i + 1) % 50 == 0:
            print(f"  {name}: {i + 1}/{len(images)}", flush=True)
    return summarize(name, rows), rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--train-images", type=Path, required=True)
    parser.add_argument("--eval-root", type=Path, required=True)
    parser.add_argument("--sample", type=int, default=300, help="frames per corpus")
    parser.add_argument("--device", default="", help="cpu / cuda; default auto")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--labels",
        action="store_true",
        help="also report, per margin, what share of annotated robots survives the crop and how "
        "much zoom is left once that margin is applied. This is what decides arm E.",
    )
    parser.add_argument(
        "--margins",
        type=float,
        nargs="+",
        default=[0.0, 0.1, 0.2, 0.35, 0.5],
        help="crop margins to evaluate, as a fraction of the field box per side",
    )
    parser.add_argument("--output", type=Path, default=None, help="write JSON here")
    args = parser.parse_args()

    device = torch.device(args.device or ("cuda" if torch.cuda.is_available() else "cpu"))
    model, cfg = load_model(args.model, device)
    transform = common_transforms(pad_size=cfg.pad_size)
    rng = random.Random(args.seed)

    results = []
    per_frame: dict[str, list[dict[str, float]]] = {}

    train_images = sample_images(args.train_images, args.sample, rng)
    summary, rows = run_corpus(
        "train", train_images, model, transform, device, cfg.image_size, cfg.pad_size, args.labels
    )
    results.append(summary)
    per_frame["train"] = rows

    # The eval set is several recordings with different framing; keep them separate so a
    # single outlier recording cannot decide the gate.
    for rec in sorted(p for p in args.eval_root.iterdir() if (p / "images").is_dir()):
        images = sample_images(rec / "images", args.sample, rng)
        summary, rows = run_corpus(
            f"eval:{rec.name}",
            images,
            model,
            transform,
            device,
            cfg.image_size,
            cfg.pad_size,
            args.labels,
        )
        results.append(summary)
        per_frame[rec.name] = rows

    print(f"\n{'corpus':<52} {'n':>5} {'mask%':>8} {'bbox%':>8} {'bbox p10-p90':>16}")
    for r in results:
        if not r.get("n"):
            continue
        print(
            f"{str(r['corpus']):<52} {r['n']:>5} "
            f"{float(r['mask_frac_median']) * 100:>7.1f}% "
            f"{float(r['bbox_frac_median']) * 100:>7.1f}% "
            f"{float(r['bbox_frac_p10']) * 100:>6.1f}-{float(r['bbox_frac_p90']) * 100:<.1f}%"
        )

    if args.labels:
        for name, rows in (
            ("train", per_frame["train"]),
            ("eval", [r for k, v in per_frame.items() if k != "train" for r in v]),
        ):
            print(f"\n{name}: crop margin vs. labels kept and zoom remaining")
            print(f"  {'margin':>7} {'kept':>7} {'whole':>7} {'zoom':>7}")
            for r in margin_retention(rows, tuple(args.margins)):
                print(
                    f"  {r['margin']:>7.2f} {r['kept'] * 100:>6.1f}% {r['whole'] * 100:>6.1f}% "
                    f"{r['zoom']:>6.2f}x"
                )

    if args.output:
        args.output.write_text(json.dumps({"summary": results, "frames": per_frame}, indent=2))
        print(f"\nwrote {args.output}")


if __name__ == "__main__":
    main()
