#!/usr/bin/env python3
"""Render the images behind the cut-paste context-swap result as a figure.

`interpret_context_vs_appearance.py` reports the cut-paste probe as four means over
the eval frames; this shows the actual inputs those means are read from. One row per
sample frame, one column per condition, each tile captioned with the opponent score
both models give the same target box (max opponent-class probability among head
anchors inside it) - the exact quantity the probe averages.

Conditions, in the order the report's table lists them:

  original            the letterboxed eval frame
  crop on other arena the context swap: same crop, same coordinates, a *different*
                      eval frame's arena underneath (see NOTE below)
  crop on gray        `crop_on_gray`: same crop on a neutral 114 canvas (appearance only)
  robot removed       `robot_removed`: box contents replaced by a blur of the frame

NOTE: the probe's own `crop_on_arena` condition is not shown, because it does not
swap context. It indexes its background list with the frame's own index
(`backgrounds[fi % len(backgrounds)]` where `backgrounds` is the same frame list), so
the crop is pasted back at its own coordinates on its own frame and the result is
bit-identical to the original — its reported equality with `original` is an identity,
not a finding. "crop on other arena" here is the swap that condition was meant to be.

Usage:
    PYTHONPATH=. venv/bin/python training/model_eval/make_cutpaste_mosaic.py \
        --eval training/data/nhrl_keypoints_eval_test \
        --real data/eval_models/yolo26n_nhrl_robots_bbox_2026-07-16.pt \
        --mix  data/eval_models/yolo26n_mix_all_2026-07-22.pt \
        -o docs/experiments/perception_performance/assets/cutpaste_mosaic.png
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import torch
from interpret_context_vs_appearance import IMGSZ, Model, load_eval

TILE_W = 400
PAD = 6
HEADER_H = 34
CAPTION_H = 26
MIN_INDEX_GAP = 2  # frames are ordered by recording then timestamp; skip near-neighbors
FONT = cv2.FONT_HERSHEY_SIMPLEX
BOX_COLOR = (60, 220, 60)  # BGR: target box the score is read from

CONDITIONS = ("original", "crop on other arena", "crop on gray", "robot removed")


def build_conditions(
    frames: list[dict], index: int, other_index: int
) -> tuple[dict[str, np.ndarray], list[float]] | None:
    """The condition images for one frame, built exactly as the probe builds them."""
    img, box = frames[index]["img"], frames[index]["boxes"][0]
    x0, y0, x1, y1 = (int(round(v)) for v in box)
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(IMGSZ, x1), min(IMGSZ, y1)
    if x1 - x0 < 6 or y1 - y0 < 6:
        return None
    crop = img[y0:y1, x0:x1]

    other = frames[other_index]["img"].copy()
    other[y0:y1, x0:x1] = crop

    gray = np.full((IMGSZ, IMGSZ, 3), 114, np.uint8)
    gray[y0:y1, x0:x1] = crop

    removed = img.copy()
    removed[y0:y1, x0:x1] = cv2.GaussianBlur(img, (0, 0), 15)[y0:y1, x0:x1]

    images = dict(zip(CONDITIONS, (img, other, gray, removed)))
    return images, [float(x0), float(y0), float(x1), float(y1)]


def pick_indices(frames: list[dict], eligible: list[int], count: int) -> list[int]:
    """The `count` eligible frames with the largest target box, kept apart in the list.

    The probe averages over every frame, but a figure only reads if the robot is big
    enough to see, so the tiles are the closest-in samples rather than a spread.
    """
    chosen: list[int] = []
    for index in sorted(eligible, key=lambda i: -box_area(frames[i]["boxes"][0])):
        if all(abs(index - c) >= MIN_INDEX_GAP for c in chosen):
            chosen.append(index)
        if len(chosen) == count:
            break
    return sorted(chosen)


def box_area(box: list[float]) -> float:
    return max(0.0, box[2] - box[0]) * max(0.0, box[3] - box[1])


def content_band(image_rgb: np.ndarray) -> tuple[int, int]:
    """Rows of a letterboxed frame that hold image, not the 114 padding bars."""
    filled = np.flatnonzero((image_rgb != 114).any(axis=(1, 2)))
    if filled.size == 0:
        return 0, IMGSZ
    return int(filled[0]), int(filled[-1]) + 1


def draw_tile(
    image_rgb: np.ndarray,
    box: list[float],
    band: tuple[int, int],
    tile_h: int,
    caption: str,
    outline: bool,
) -> np.ndarray:
    """One condition tile: letterbox bars cropped away, score below.

    Only the `original` tile outlines the target box — the box is the same in every
    condition of a row, so repeating it just covers the pasted pixels being compared.
    """
    top, bottom = band
    cropped = image_rgb[top:bottom]
    tile = cv2.cvtColor(
        cv2.resize(cropped, (TILE_W, tile_h), interpolation=cv2.INTER_AREA), cv2.COLOR_RGB2BGR
    )
    if outline:
        sx, sy = TILE_W / IMGSZ, tile_h / (bottom - top)
        cv2.rectangle(
            tile,
            (int(box[0] * sx), int((box[1] - top) * sy)),
            (int(box[2] * sx), int((box[3] - top) * sy)),
            BOX_COLOR,
            2,
        )
    panel = np.full((tile_h + CAPTION_H, TILE_W, 3), 18, np.uint8)
    panel[:tile_h] = tile
    cv2.putText(panel, caption, (4, tile_h + 18), FONT, 0.45, (235, 235, 235), 1, cv2.LINE_AA)
    return panel


def build_mosaic(
    frames: list[dict], indices: list[int], scores: dict[tuple[int, str], tuple[float, float]]
) -> np.ndarray:
    cols, rows_n = len(CONDITIONS), len(indices)
    # Every eval frame letterboxes to the same band, so one tile height fits the grid.
    top, bottom = content_band(frames[indices[0]]["img"])
    tile_h = round(TILE_W * (bottom - top) / IMGSZ)
    cell_h = tile_h + CAPTION_H
    width = cols * TILE_W + (cols + 1) * PAD
    height = HEADER_H + rows_n * cell_h + (rows_n + 1) * PAD
    canvas = np.full((height, width, 3), 18, np.uint8)

    for c, name in enumerate(CONDITIONS):
        tw = cv2.getTextSize(name, FONT, 0.55, 1)[0][0]
        x = PAD + c * (TILE_W + PAD) + max(0, (TILE_W - tw) // 2)
        cv2.putText(canvas, name, (x, HEADER_H - 12), FONT, 0.55, (235, 235, 235), 1, cv2.LINE_AA)

    for r, index in enumerate(indices):
        built = build_conditions(frames, index, (index + len(frames) // 2) % len(frames))
        assert built is not None  # filtered in main()
        images, box = built
        band = content_band(frames[index]["img"])
        for c, name in enumerate(CONDITIONS):
            real, mix = scores[(index, name)]
            caption = f"real {real:.3f}   mix {mix:.3f}"
            panel = draw_tile(images[name], box, band, tile_h, caption, name == CONDITIONS[0])
            y = HEADER_H + PAD + r * (cell_h + PAD)
            x = PAD + c * (TILE_W + PAD)
            canvas[y : y + cell_h, x : x + TILE_W] = panel
    return canvas


def score_all(
    model_path: str, frames: list[dict], indices: list[int]
) -> dict[tuple[int, str], float]:
    model = Model(model_path)
    out: dict[tuple[int, str], float] = {}
    for index in indices:
        built = build_conditions(frames, index, (index + len(frames) // 2) % len(frames))
        assert built is not None
        images, box = built
        for name in CONDITIONS:
            out[(index, name)] = float(model.box_score([images[name]], box)[0])
    del model
    torch.cuda.empty_cache()
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description="Cut-paste context-swap condition mosaic")
    ap.add_argument("--eval", type=Path, required=True, help="eval dataset root")
    ap.add_argument("--real", required=True, help="real_bbox baseline .pt")
    ap.add_argument("--mix", required=True, help="mix_all .pt")
    ap.add_argument("-o", "--output", type=Path, required=True, help="output PNG path")
    ap.add_argument("--samples", type=int, default=3, help="frames to show (default 3)")
    ap.add_argument(
        "--frames", type=int, default=60, help="eval frames loaded, matching the probe (default 60)"
    )
    args = ap.parse_args()

    frames = load_eval(args.eval, args.frames)
    print(f"loaded {len(frames)} eval frames with opponent boxes")
    # Only frames whose target box survives the probe's minimum-size gate are eligible.
    eligible = [i for i in range(len(frames)) if build_conditions(frames, i, (i + 1) % len(frames))]
    indices = pick_indices(frames, eligible, args.samples)
    areas = [box_area(frames[i]["boxes"][0]) for i in indices]
    print(f"{len(indices)} samples: {indices} (box areas {[round(a) for a in areas]} px^2)")

    real_scores = score_all(args.real, frames, indices)
    mix_scores = score_all(args.mix, frames, indices)
    scores = {k: (real_scores[k], mix_scores[k]) for k in real_scores}

    for name in CONDITIONS:
        means = np.mean([[real_scores[(i, name)], mix_scores[(i, name)]] for i in indices], axis=0)
        print(f"  {name:<20} real {means[0]:.3f}   mix {means[1]:.3f}")

    canvas = build_mosaic(frames, indices, scores)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), canvas)
    print(f"wrote {args.output} ({canvas.shape[1]}x{canvas.shape[0]})")


if __name__ == "__main__":
    main()
