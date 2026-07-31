#!/usr/bin/env python3
"""Render the images behind the cut-paste context-swap result as a figure.

`interpret_context_vs_appearance.py` reports the cut-paste probe as four means over
the eval frames; this shows the actual inputs those means are read from. One row per
sample frame, one column per condition, each tile captioned with the opponent score
both models give the same target box (max opponent-class probability among head
anchors inside it) - the exact quantity the probe averages.

Conditions, in the order the report's table lists them, plus one diagnostic column:

  original            the letterboxed eval frame
  crop on own arena   the probe's `crop_on_arena`: the crop pasted back at its own
                      coordinates on its own frame, so the image is bit-identical to
                      the original (see NOTE below)
  crop on other arena the context swap that column is meant to be: same crop, same
                      coordinates, a *different* eval frame's arena underneath
  crop on gray        `crop_on_gray`: same crop on a neutral 114 canvas (appearance only)
  robot removed       `robot_removed`: box contents replaced by a blur of the frame

NOTE: `crop_on_arena` in the probe indexes its background list with the frame's own
index (`backgrounds[fi % len(backgrounds)]` where `backgrounds` is the same frame
list), so it reconstructs the original image rather than swapping context. Its
reported equality with `original` is therefore an identity, not a finding. The
"crop on other arena" column here runs the intended swap for comparison.

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

TILE = 320
PAD = 6
HEADER_H = 34
CAPTION_H = 26
FONT = cv2.FONT_HERSHEY_SIMPLEX
BOX_COLOR = (60, 220, 60)  # BGR: target box the score is read from

CONDITIONS = (
    "original",
    "crop on own arena",
    "crop on other arena",
    "crop on gray",
    "robot removed",
)


def build_conditions(
    frames: list[dict], index: int, other_index: int
) -> tuple[dict[str, np.ndarray], list[float]] | None:
    """The five condition images for one frame, built exactly as the probe builds them."""
    img, box = frames[index]["img"], frames[index]["boxes"][0]
    x0, y0, x1, y1 = (int(round(v)) for v in box)
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(IMGSZ, x1), min(IMGSZ, y1)
    if x1 - x0 < 6 or y1 - y0 < 6:
        return None
    crop = img[y0:y1, x0:x1]

    own = img.copy()
    own[y0:y1, x0:x1] = crop  # the probe's crop_on_arena: a no-op paste

    other = frames[other_index]["img"].copy()
    other[y0:y1, x0:x1] = crop

    gray = np.full((IMGSZ, IMGSZ, 3), 114, np.uint8)
    gray[y0:y1, x0:x1] = crop

    removed = img.copy()
    removed[y0:y1, x0:x1] = cv2.GaussianBlur(img, (0, 0), 15)[y0:y1, x0:x1]

    images = dict(zip(CONDITIONS, (img, own, other, gray, removed)))
    return images, [float(x0), float(y0), float(x1), float(y1)]


def pick_indices(n_frames: int, count: int) -> list[int]:
    """`count` frame indices spread evenly over the probe's frame list."""
    if n_frames <= count:
        return list(range(n_frames))
    return [round(i * (n_frames - 1) / (count - 1)) for i in range(count)]


def draw_tile(image_rgb: np.ndarray, box: list[float], caption: str) -> np.ndarray:
    tile = cv2.cvtColor(
        cv2.resize(image_rgb, (TILE, TILE), interpolation=cv2.INTER_AREA), cv2.COLOR_RGB2BGR
    )
    s = TILE / IMGSZ
    cv2.rectangle(
        tile,
        (int(box[0] * s), int(box[1] * s)),
        (int(box[2] * s), int(box[3] * s)),
        BOX_COLOR,
        1,
    )
    panel = np.full((TILE + CAPTION_H, TILE, 3), 18, np.uint8)
    panel[:TILE] = tile
    cv2.putText(panel, caption, (4, TILE + 18), FONT, 0.45, (235, 235, 235), 1, cv2.LINE_AA)
    return panel


def build_mosaic(
    frames: list[dict], indices: list[int], scores: dict[tuple[int, str], tuple[float, float]]
) -> np.ndarray:
    cols, rows_n = len(CONDITIONS), len(indices)
    cell_h = TILE + CAPTION_H
    width = cols * TILE + (cols + 1) * PAD
    height = HEADER_H + rows_n * cell_h + (rows_n + 1) * PAD
    canvas = np.full((height, width, 3), 18, np.uint8)

    for c, name in enumerate(CONDITIONS):
        tw = cv2.getTextSize(name, FONT, 0.55, 1)[0][0]
        x = PAD + c * (TILE + PAD) + max(0, (TILE - tw) // 2)
        cv2.putText(canvas, name, (x, HEADER_H - 12), FONT, 0.55, (235, 235, 235), 1, cv2.LINE_AA)

    for r, index in enumerate(indices):
        built = build_conditions(frames, index, (index + len(frames) // 2) % len(frames))
        assert built is not None  # filtered in main()
        images, box = built
        for c, name in enumerate(CONDITIONS):
            real, mix = scores[(index, name)]
            panel = draw_tile(images[name], box, f"real {real:.3f}   mix {mix:.3f}")
            y = HEADER_H + PAD + r * (cell_h + PAD)
            x = PAD + c * (TILE + PAD)
            canvas[y : y + cell_h, x : x + TILE] = panel
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
    ap.add_argument("--samples", type=int, default=9, help="frames to show (default 9)")
    ap.add_argument(
        "--frames", type=int, default=60, help="eval frames loaded, matching the probe (default 60)"
    )
    args = ap.parse_args()

    frames = load_eval(args.eval, args.frames)
    print(f"loaded {len(frames)} eval frames with opponent boxes")
    # Only frames whose target box survives the probe's minimum-size gate are eligible.
    eligible = [i for i in range(len(frames)) if build_conditions(frames, i, (i + 1) % len(frames))]
    indices = [eligible[i] for i in pick_indices(len(eligible), args.samples)]
    print(f"{len(indices)} samples: {indices}")

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
