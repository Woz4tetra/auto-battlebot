#!/usr/bin/env python3
"""Render the images behind the cut-paste context-swap result as a figure.

`interpret_context_vs_appearance.py` reports the cut-paste probe as four means over
the eval frames; this shows the actual inputs those means are read from. One row per
sample frame, one column per condition, each tile captioned with the opponent score
every candidate gives the same target box (max opponent-class probability among head
anchors inside it) - the exact quantity the probe averages.

Conditions, in the order the report's table lists them:

  original            the letterboxed eval frame
  crop on other arena `crop_on_arena`: same crop, same coordinates, a different eval
                      frame's arena underneath (donor picked by the probe's
                      `donor_index`, so the tile is the image the probe scored)
  crop on gray        `crop_on_gray`: same crop on a neutral 114 canvas (appearance only)
  robot removed       `robot_removed`: box contents replaced by a blur of the frame

Pass `--opp-channels 0` for 2-class models: the probe's default (0,1) is the 5-class
`object`+`robot` pair, and on a 2-class head channel 1 is `house_bot`, which would score
"found a house bot" as "found an opponent".

Usage:
    PYTHONPATH=. venv/bin/python training/model_eval/make_cutpaste_mosaic.py \
        --eval training/data/nhrl_keypoints_eval_test \
        --candidate real_only=data/eval_models/yolo26n_arm_real_only_2026-07-31.pt \
        --candidate mixed=data/eval_models/yolo26n_arm_mixed_2026-07-31.pt \
        --candidate synth_only=data/eval_models/yolo26n_arm_synth_only_2026-07-31.pt \
        --opp-channels 0 --frames 100 \
        -o docs/experiments/perception_performance/assets/cutpaste_mosaic.png
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import interpret_context_vs_appearance as probe
import numpy as np
import torch

TILE_W = 400
PAD = 6
HEADER_H = 34
CAPTION_LINE_H = 18
MIN_INDEX_GAP = 2  # frames are ordered by recording then timestamp; skip near-neighbors
FONT = cv2.FONT_HERSHEY_SIMPLEX
BOX_COLOR = (60, 220, 60)  # BGR: target box the score is read from

CONDITIONS = ("original", "crop on other arena", "crop on gray", "robot removed")


def build_conditions(
    frames: list[dict], index: int
) -> tuple[dict[str, np.ndarray], list[float]] | None:
    """The condition images for one frame, built exactly as the probe builds them."""
    imgsz = probe.IMGSZ
    img, box = frames[index]["img"], frames[index]["boxes"][0]
    x0, y0, x1, y1 = (int(round(v)) for v in box)
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(imgsz, x1), min(imgsz, y1)
    if x1 - x0 < 6 or y1 - y0 < 6:
        return None
    crop = img[y0:y1, x0:x1]
    rect = [float(x0), float(y0), float(x1), float(y1)]

    other = frames[probe.donor_index(frames, index, rect)]["img"].copy()
    other[y0:y1, x0:x1] = crop

    gray = np.full((imgsz, imgsz, 3), 114, np.uint8)
    gray[y0:y1, x0:x1] = crop

    removed = img.copy()
    removed[y0:y1, x0:x1] = cv2.GaussianBlur(img, (0, 0), 15)[y0:y1, x0:x1]

    return dict(zip(CONDITIONS, (img, other, gray, removed))), rect


def box_area(box: list[float]) -> float:
    return max(0.0, box[2] - box[0]) * max(0.0, box[3] - box[1])


def pick_indices(frames: list[dict], eligible: list[int], count: int, mode: str) -> list[int]:
    """Which frames become rows.

    `largest` picks the biggest target boxes - the most legible tiles, but a biased
    sample: gray retention depends strongly on box size, and the largest boxes are the
    one stratum where the arms barely differ. `spread` (the default) splits the eligible
    frames into `count` equal box-area bins and takes the median-area frame of each, so
    the figure shows the size dependence instead of hiding it.
    """
    by_area = sorted(eligible, key=lambda i: box_area(frames[i]["boxes"][0]))
    chosen: list[int] = []
    if mode == "largest":
        for index in reversed(by_area):
            if all(abs(index - c) >= MIN_INDEX_GAP for c in chosen):
                chosen.append(index)
            if len(chosen) == count:
                break
        return sorted(chosen)

    edges = np.linspace(0, len(by_area), count + 1).astype(int)
    for lo, hi in zip(edges[:-1], edges[1:]):
        binned = by_area[lo:hi]
        if not binned:
            continue
        for index in sorted(binned, key=lambda i: abs(i - binned[len(binned) // 2])):
            if all(abs(index - c) >= MIN_INDEX_GAP for c in chosen):
                chosen.append(index)
                break
    return sorted(chosen, key=lambda i: box_area(frames[i]["boxes"][0]))


def content_band(image_rgb: np.ndarray) -> tuple[int, int]:
    """Rows of a letterboxed frame that hold image, not the 114 padding bars."""
    filled = np.flatnonzero((image_rgb != 114).any(axis=(1, 2)))
    if filled.size == 0:
        return 0, probe.IMGSZ
    return int(filled[0]), int(filled[-1]) + 1


def draw_tile(
    image_rgb: np.ndarray,
    box: list[float],
    band: tuple[int, int],
    tile_h: int,
    captions: list[str],
    outline: bool,
    n_caption_lines: int,
) -> np.ndarray:
    """One condition tile: letterbox bars cropped away, one caption line per candidate.

    Only the `original` tile outlines the target box - the box is the same in every
    condition of a row, so repeating it just covers the pasted pixels being compared.
    """
    top, bottom = band
    tile = cv2.cvtColor(
        cv2.resize(image_rgb[top:bottom], (TILE_W, tile_h), interpolation=cv2.INTER_AREA),
        cv2.COLOR_RGB2BGR,
    )
    if outline:
        sx, sy = TILE_W / probe.IMGSZ, tile_h / (bottom - top)
        cv2.rectangle(
            tile,
            (int(box[0] * sx), int((box[1] - top) * sy)),
            (int(box[2] * sx), int((box[3] - top) * sy)),
            BOX_COLOR,
            2,
        )
    caption_h = n_caption_lines * CAPTION_LINE_H + 8
    panel = np.full((tile_h + caption_h, TILE_W, 3), 18, np.uint8)
    panel[:tile_h] = tile
    for i, line in enumerate(captions):
        y = tile_h + 14 + i * CAPTION_LINE_H
        cv2.putText(panel, line, (4, y), FONT, 0.42, (235, 235, 235), 1, cv2.LINE_AA)
    return panel


def score_all(model_path: str, frames: list[dict], indices: list[int]) -> dict[tuple, float]:
    """Target-box opponent score for every (frame, condition) under one model."""
    model = probe.Model(model_path)
    out: dict[tuple, float] = {}
    for index in indices:
        built = build_conditions(frames, index)
        assert built is not None  # filtered in main()
        images, box = built
        for name in CONDITIONS:
            out[(index, name)] = float(model.box_score([images[name]], box)[0])
    del model
    torch.cuda.empty_cache()
    return out


def build_mosaic(
    frames: list[dict],
    indices: list[int],
    names: list[str],
    scores: dict[str, dict[tuple, float]],
) -> np.ndarray:
    cols, rows_n = len(CONDITIONS), len(indices)
    # Every eval frame letterboxes to the same band, so one tile height fits the grid.
    top, bottom = content_band(frames[indices[0]]["img"])
    tile_h = round(TILE_W * (bottom - top) / probe.IMGSZ)
    # +1 caption line: the `original` tile also prints the box area, since retention
    # depends strongly on it and a reader has to know which stratum a row is from.
    cell_h = tile_h + (len(names) + 1) * CAPTION_LINE_H + 8
    width = cols * TILE_W + (cols + 1) * PAD
    height = HEADER_H + rows_n * cell_h + (rows_n + 1) * PAD
    canvas = np.full((height, width, 3), 18, np.uint8)

    for c, name in enumerate(CONDITIONS):
        tw = cv2.getTextSize(name, FONT, 0.55, 1)[0][0]
        x = PAD + c * (TILE_W + PAD) + max(0, (TILE_W - tw) // 2)
        cv2.putText(canvas, name, (x, HEADER_H - 12), FONT, 0.55, (235, 235, 235), 1, cv2.LINE_AA)

    label_w = max(len(n) for n in names)
    for r, index in enumerate(indices):
        area = round(box_area(frames[index]["boxes"][0]))
        built = build_conditions(frames, index)
        assert built is not None
        images, box = built
        band = content_band(frames[index]["img"])
        for c, cond in enumerate(CONDITIONS):
            captions = [f"{n:<{label_w}}  {scores[n][(index, cond)]:.3f}" for n in names]
            if cond == CONDITIONS[0]:
                captions = [f"box {area} px^2"] + captions
            panel = draw_tile(
                images[cond], box, band, tile_h, captions, cond == CONDITIONS[0], len(names) + 1
            )
            y = HEADER_H + PAD + r * (cell_h + PAD)
            x = PAD + c * (TILE_W + PAD)
            canvas[y : y + cell_h, x : x + TILE_W] = panel
    return canvas


def main() -> None:
    ap = argparse.ArgumentParser(description="Cut-paste context-swap condition mosaic")
    ap.add_argument("--eval", type=Path, required=True, help="eval dataset root")
    ap.add_argument(
        "--candidate",
        action="append",
        required=True,
        metavar="NAME=PT",
        help="Model to score each tile with, repeatable; one caption line per candidate",
    )
    ap.add_argument("-o", "--output", type=Path, required=True, help="output PNG path")
    ap.add_argument("--samples", type=int, default=3, help="frames to show (default 3)")
    ap.add_argument(
        "--frames", type=int, default=60, help="eval frames loaded, matching the probe (default 60)"
    )
    ap.add_argument(
        "--select",
        choices=["spread", "largest"],
        default="spread",
        help="Row sampling: 'spread' = one frame per box-area bin (default), 'largest' = biggest",
    )
    ap.add_argument(
        "--min-score",
        type=float,
        default=0.15,
        help="Only show frames the first candidate detects at this score (0 disables)",
    )
    ap.add_argument(
        "--opp-channels",
        default="0,1",
        help="Head channels meaning 'opponent'. 5-class: 0,1 (object+robot). 2-class: 0 (robot).",
    )
    args = ap.parse_args()

    probe.OPP_CHANNELS = tuple(int(c) for c in args.opp_channels.split(","))
    candidates: list[tuple[str, str]] = []
    for spec in args.candidate:
        name, _, path = spec.partition("=")
        if not path:
            raise SystemExit(f"--candidate expects NAME=PT, got {spec!r}")
        candidates.append((name, path))

    frames = probe.load_eval(args.eval, args.frames)
    print(f"loaded {len(frames)} eval frames; opponent channels {probe.OPP_CHANNELS}")
    # Only frames whose target box survives the probe's minimum-size gate are eligible.
    eligible = [i for i in range(len(frames)) if build_conditions(frames, i)]
    # ...and that the first candidate actually detects. A cut-paste ratio on a box the
    # model never found (0.004 -> 0.001) illustrates nothing, so those make useless rows.
    # Same 0.15 threshold the RISE probe uses to decide which boxes are worth explaining.
    # This selects which frames are *shown*; every reported statistic is over all frames.
    if args.min_score > 0:
        gate = probe.Model(candidates[0][1])
        kept = [
            i
            for i in eligible
            if float(gate.box_score([frames[i]["img"]], frames[i]["boxes"][0])[0]) >= args.min_score
        ]
        del gate
        torch.cuda.empty_cache()
        print(f"{len(kept)}/{len(eligible)} frames detected by {candidates[0][0]}")
        eligible = kept or eligible
    indices = pick_indices(frames, eligible, args.samples, args.select)
    areas = [box_area(frames[i]["boxes"][0]) for i in indices]
    print(f"{len(indices)} samples: {indices} (box areas {[round(a) for a in areas]} px^2)")

    names = [n for n, _ in candidates]
    scores = {name: score_all(pt, frames, indices) for name, pt in candidates}
    for cond in CONDITIONS:
        means = {n: np.mean([scores[n][(i, cond)] for i in indices]) for n in names}
        print(f"  {cond:<20} " + "  ".join(f"{n} {m:.3f}" for n, m in means.items()))

    canvas = build_mosaic(frames, indices, names, scores)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), canvas)
    print(f"wrote {args.output} ({canvas.shape[1]}x{canvas.shape[0]})")


if __name__ == "__main__":
    main()
