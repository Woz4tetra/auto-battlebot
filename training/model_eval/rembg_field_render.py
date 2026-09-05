#!/usr/bin/env python3
"""Draw the rembg experiment: one eval-frame flip-book per arm, plus failure contact sheets.

A table of recall and precision does not show *how* a method fails, and the failure modes
expected here (merged blobs, whole-floor responses, the parallax smear) are obvious on sight
and invisible in a number. Every clip uses the same 2x2 panel:

    frame, GT and predicted boxes     | raw rembg matte, before any mask
    matte after the field mask,       | arm-specific: the floor raster for rembg_warped,
    mask outline drawn                |   the field mask overlay for the post-hoc arms

The raw-matte panel is the one that earns the format: it is the only way to tell "the mask
deleted it" from "rembg never found it", which the score cannot separate.

Reads the cache and params written by rembg_field_predict.py; nothing here runs a network.

Usage:
    venv/bin/python training/model_eval/rembg_field_render.py \
        training/data/nhrl_keypoints_eval_test -o training/data/eval_results/rembg \
        --assets docs/experiments/perception_performance/assets/2026-09-04_rembg
"""

from __future__ import annotations

import argparse
import subprocess
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from rembg_field import (
    ARMS,
    Component,
    FrameRef,
    MatteCache,
    enumerate_frames,
    load_params,
    make_raster,
    scored_frames,
    warp_to_raster,
)
from rembg_field_predict import FrameContext, read_frame, score_frame
from score import Frame, load_gt, match_indices
from tqdm import tqdm

from auto_battlebot.floor_background import FloorRaster

CLIP_FPS = 4
TITLE_SECONDS = 2
PANEL_W, PANEL_H = 640, 360
IOU = 0.5

GT_BGR = (0, 200, 0)
TP_BGR = (60, 60, 235)
FP_BGR = (0, 170, 255)
MASK_BGR = (52, 104, 235)
INK = (245, 245, 245)
SHADOW = (20, 20, 20)
FONT = cv2.FONT_HERSHEY_SIMPLEX

TILE_H = 240


def label_text(image: np.ndarray, text: str, origin: tuple[int, int], color=INK, scale=0.5):
    cv2.putText(image, text, origin, FONT, scale, SHADOW, 3, cv2.LINE_AA)
    cv2.putText(image, text, origin, FONT, scale, color, 1, cv2.LINE_AA)


def fit(image: np.ndarray, width: int = PANEL_W, height: int = PANEL_H) -> np.ndarray:
    """Letterbox an image into a fixed panel."""
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    scale = min(width / image.shape[1], height / image.shape[0])
    resized = cv2.resize(image, (int(image.shape[1] * scale), int(image.shape[0] * scale)))
    panel = np.zeros((height, width, 3), np.uint8)
    y = (height - resized.shape[0]) // 2
    x = (width - resized.shape[1]) // 2
    panel[y : y + resized.shape[0], x : x + resized.shape[1]] = resized
    return panel


def draw_box(image: np.ndarray, box, color, thickness: int = 2) -> None:
    x1, y1, x2, y2 = (int(v) for v in box)
    cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)


def draw_outline(image: np.ndarray, mask: np.ndarray, color=MASK_BGR) -> None:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(image, contours, -1, color, 2)


@dataclass
class Outcome:
    """One frame's detections split by the IoU match against GT."""

    frame: Frame
    detections: list[Component]
    matched_pred: set[int]
    matched_gt: set[int]

    @property
    def tp(self) -> int:
        return len(self.matched_pred)

    @property
    def fp(self) -> int:
        return len(self.detections) - self.tp

    @property
    def fn(self) -> int:
        return len(self.frame.gt_labels) - len(self.matched_gt)


def outcome_for(gt: tuple, detections: list[Component], iou: float = IOU) -> Outcome:
    frame = score_frame(gt, detections)
    matched_pred: set[int] = set()
    matched_gt: set[int] = set()
    for g, p in match_indices(frame, iou):
        if g is not None and p is not None:
            matched_pred.add(p)
            matched_gt.add(g)
    return Outcome(frame, detections, matched_pred, matched_gt)


def boxes_panel(image: np.ndarray, outcome: Outcome, caption: str) -> np.ndarray:
    canvas = image.copy()
    for box in outcome.frame.gt_boxes:
        draw_box(canvas, box, GT_BGR)
    for index, det in enumerate(outcome.detections):
        draw_box(canvas, det.box, TP_BGR if index in outcome.matched_pred else FP_BGR)
    panel = fit(canvas)
    label_text(panel, caption, (8, 18), INK, 0.5)
    label_text(
        panel,
        f"{len(outcome.detections)} det  {len(outcome.frame.gt_labels)} GT   "
        f"TP {outcome.tp}  FP {outcome.fp}  FN {outcome.fn}",
        (8, 38),
        INK,
        0.5,
    )
    label_text(panel, "green GT  red TP  orange FP", (8, PANEL_H - 10), INK, 0.45)
    return panel


def matte_panel(matte: np.ndarray, title: str) -> np.ndarray:
    panel = fit(matte)
    label_text(panel, title, (8, 18))
    return panel


def masked_panel(matte: np.ndarray, mask: np.ndarray, title: str) -> np.ndarray:
    masked = cv2.cvtColor(cv2.bitwise_and(matte, mask), cv2.COLOR_GRAY2BGR)
    draw_outline(masked, mask)
    panel = fit(masked)
    label_text(panel, title, (8, 18))
    return panel


def overlay_panel(image: np.ndarray, mask: np.ndarray, title: str) -> np.ndarray:
    tint = image.copy()
    tint[mask > 0] = (0.55 * tint[mask > 0] + 0.45 * np.array(MASK_BGR)).astype(np.uint8)
    draw_outline(tint, mask)
    panel = fit(tint)
    label_text(panel, title, (8, 18))
    return panel


class ArmRenderer:
    """Builds the 2x2 panel for one arm on one frame from the shared context."""

    def __init__(self, arm: str, raster: FloorRaster) -> None:
        self.arm = arm
        self.raster = raster

    def panel(self, ctx: FrameContext, image: np.ndarray, outcome: Outcome, caption: str):
        top_left = boxes_panel(image, outcome, caption)
        if self.arm == "rembg_warped":
            assert ctx.homography is not None and ctx.raster_matte is not None
            warped, valid = warp_to_raster(image, self.raster, ctx.homography)
            top_right = matte_panel(ctx.raster_matte, "rembg matte on the floor raster")
            bottom_left = masked_panel(ctx.raster_matte, valid, "matte inside the floor square")
            bottom_right = fit(warped)
            label_text(bottom_right, "975 px floor raster (rembg input)", (8, 18))
        else:
            mask_name = "geom" if self.arm == "rembg_geom" else "deeplab"
            mask = ctx.masks[mask_name]
            top_right = matte_panel(ctx.raw, "raw rembg matte, before any mask")
            bottom_left = masked_panel(ctx.raw, mask, f"matte after the {mask_name} mask")
            bottom_right = overlay_panel(
                image,
                mask,
                "geometric field polygon" if mask_name == "geom" else "DeepLab floor, hulled",
            )
        return np.vstack([np.hstack([top_left, top_right]), np.hstack([bottom_left, bottom_right])])


def title_card(lines: list[str]) -> np.ndarray:
    card = np.full((PANEL_H * 2, PANEL_W * 2, 3), 18, np.uint8)
    for index, line in enumerate(lines):
        label_text(card, line, (60, 300 + 50 * index), INK, 1.0)
    return card


class H264Writer:
    """Pipe raw BGR frames into ffmpeg; libx264 at CRF 28 keeps a 600-frame clip under a few MB."""

    def __init__(self, path: Path, size: tuple[int, int], fps: int) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self._process = subprocess.Popen(
            [
                "ffmpeg",
                "-y",
                "-loglevel",
                "error",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "bgr24",
                "-s",
                f"{size[0]}x{size[1]}",
                "-r",
                str(fps),
                "-i",
                "-",
                "-c:v",
                "libx264",
                "-preset",
                "medium",
                "-crf",
                "28",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(path),
            ],
            stdin=subprocess.PIPE,
        )

    def write(self, frame: np.ndarray) -> None:
        assert self._process.stdin is not None
        self._process.stdin.write(frame.tobytes())

    def close(self) -> None:
        assert self._process.stdin is not None
        self._process.stdin.close()
        self._process.wait()


def render_flipbooks(
    frames: list[FrameRef], cache: MatteCache, raster: FloorRaster, params, gts, assets: Path
) -> dict[str, list[tuple[FrameRef, Outcome]]]:
    """One clip per arm over every scored frame; returns each arm's outcomes for the sheets."""
    writers = {
        arm: H264Writer(assets / f"flipbook_{arm}.mp4", (PANEL_W * 2, PANEL_H * 2), CLIP_FPS)
        for arm in ARMS
    }
    renderers = {arm: ArmRenderer(arm, raster) for arm in ARMS}
    outcomes: dict[str, list[tuple[FrameRef, Outcome]]] = {arm: [] for arm in ARMS}
    current = None
    for frame in tqdm(frames, desc="flipbooks", unit="frame"):
        if frame.recording != current:
            current = frame.recording
            count = sum(1 for f in frames if f.recording == current)
            card = title_card([current, f"{count} scored frames, {frame.size[0]}x{frame.size[1]}"])
            for writer in writers.values():
                for _ in range(CLIP_FPS * TITLE_SECONDS):
                    writer.write(card)
        image = read_frame(frame)
        ctx = FrameContext(frame, cache, raster)
        caption = f"{frame.recording}  {frame.stem}"
        for arm in ARMS:
            outcome = outcome_for(gts[frame.stamp_ns], ctx.detections(arm, params[arm]))
            outcomes[arm].append((frame, outcome))
            writers[arm].write(renderers[arm].panel(ctx, image, outcome, f"{arm}  {caption}"))
    for writer in writers.values():
        writer.close()
    return outcomes


# --- contact sheets ----------------------------------------------------------------------


def crop(image: np.ndarray, box, margin: float = 0.6) -> np.ndarray:
    x1, y1, x2, y2 = box
    w, h = max(x2 - x1, 32), max(y2 - y1, 32)
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    half = max(w, h) * (0.5 + margin)
    xa, ya = int(max(0, cx - half)), int(max(0, cy - half))
    xb, yb = int(min(image.shape[1], cx + half)), int(min(image.shape[0], cy + half))
    return image[ya:yb, xa:xb]


def tile(image: np.ndarray, height: int = TILE_H) -> np.ndarray:
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if image.size == 0:
        return np.zeros((height, height, 3), np.uint8)
    scale = height / image.shape[0]
    return cv2.resize(image, (max(1, int(image.shape[1] * scale)), height))


def sheet(rows: list[np.ndarray], path: Path, columns: int, title: str) -> None:
    """Grid of equal-height tiles, padded to the widest, with a title strip."""
    if not rows:
        return
    width = max(r.shape[1] for r in rows)
    padded = []
    for r in rows:
        canvas = np.zeros((r.shape[0], width, 3), np.uint8)
        canvas[:, : r.shape[1]] = r
        padded.append(canvas)
    grid = []
    for start in range(0, len(padded), columns):
        row = padded[start : start + columns]
        while len(row) < columns:
            row.append(np.zeros_like(padded[0]))
        grid.append(np.hstack(row))
    body = np.vstack(grid)
    strip = np.full((36, body.shape[1], 3), 18, np.uint8)
    label_text(strip, title, (10, 24), INK, 0.65)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), np.vstack([strip, body]), [cv2.IMWRITE_JPEG_QUALITY, 85])


def annotated(image: np.ndarray, outcome: Outcome) -> np.ndarray:
    canvas = image.copy()
    for box in outcome.frame.gt_boxes:
        draw_box(canvas, box, GT_BGR)
    for index, det in enumerate(outcome.detections):
        draw_box(canvas, det.box, TP_BGR if index in outcome.matched_pred else FP_BGR)
    return canvas


def raw_matte_sheet(frames, cache: MatteCache, assets: Path, per_recording: int = 3) -> None:
    """Frame beside its raw matte, a few per recording, before any mask touches it."""
    rows = []
    for recording in sorted({f.recording for f in frames}):
        subset = [f for f in frames if f.recording == recording]
        step = max(1, len(subset) // per_recording)
        for frame in subset[::step][:per_recording]:
            matte = cache.get("raw", frame)
            assert matte is not None
            pair = np.hstack([tile(read_frame(frame)), tile(matte)])
            label_text(pair, f"{recording[:28]} {frame.stem}", (6, 16), INK, 0.45)
            rows.append(pair)
    sheet(
        rows, assets / "sheet_raw_mattes.jpg", 3, "raw rembg mattes before masking (frame | matte)"
    )


def merged_blob_sheet(outcomes, assets: Path, limit: int = 12) -> None:
    """One detection lying over two or more GT boxes: the multi-object failure."""
    rows = []
    for frame, outcome in outcomes:
        gt = outcome.frame.gt_boxes
        for det in outcome.detections:
            covered = [
                g
                for g in gt
                if g[0] >= det.box[0] - 8
                and g[1] >= det.box[1] - 8
                and g[2] <= det.box[2] + 8
                and g[3] <= det.box[3] + 8
            ]
            if len(covered) >= 2:
                image = annotated(read_frame(frame), outcome)
                item = tile(crop(image, det.box, 0.3))
                label_text(item, f"{len(covered)} GT in one blob", (6, 16), INK, 0.45)
                rows.append(item)
                break
        if len(rows) >= limit:
            break
    sheet(
        rows, assets / "sheet_merged_blobs.jpg", 4, "merged blobs: one detection over 2+ GT boxes"
    )


def missed_sheet(outcomes, cache: MatteCache, assets: Path, limit: int = 16) -> None:
    """GT boxes nothing matched at IoU 0.5, with the raw matte crop beside them."""
    rows = []
    for frame, outcome in outcomes:
        matte = cache.get("raw", frame)
        assert matte is not None
        image = annotated(read_frame(frame), outcome)
        for g, box in enumerate(outcome.frame.gt_boxes):
            if g in outcome.matched_gt:
                continue
            pair = np.hstack([tile(crop(image, box)), tile(crop(matte, box))])
            label_text(pair, f"{outcome.frame.gt_labels[g]}  {frame.stem}", (6, 16), INK, 0.45)
            rows.append(pair)
            break
        if len(rows) >= limit:
            break
    sheet(
        rows, assets / "sheet_missed_robots.jpg", 4, "missed robots (frame crop | raw matte crop)"
    )


def parallax_sheet(outcomes, assets: Path, limit: int = 8) -> None:
    """rembg_warped detections that match at IoU 0.3 but not 0.5: the smear made visible."""
    rows = []
    for frame, outcome in outcomes:
        loose = outcome_for(
            (outcome.frame.gt_boxes, outcome.frame.gt_labels, outcome.frame.gt_keypoints),
            outcome.detections,
            0.3,
        )
        smeared = loose.matched_pred - outcome.matched_pred
        if not smeared:
            continue
        image = annotated(read_frame(frame), outcome)
        det = outcome.detections[next(iter(smeared))]
        item = tile(crop(image, det.box, 0.4))
        label_text(item, f"IoU in [0.3, 0.5)  {frame.stem}", (6, 16), INK, 0.45)
        rows.append(item)
        if len(rows) >= limit:
            break
    sheet(
        rows,
        assets / "sheet_parallax_smear.jpg",
        4,
        "rembg_warped: boxes matched at 0.3 but not 0.5",
    )


def disagreement_sheet(geom, deeplab, cache: MatteCache, contexts, assets: Path, limit=12):
    """Detections that exactly one of the two post-hoc masks kept, with both outlines drawn."""
    rows = []
    for (frame, geom_out), (_, deeplab_out) in zip(geom, deeplab):
        geom_boxes = {tuple(d.box) for d in geom_out.detections}
        deeplab_boxes = {tuple(d.box) for d in deeplab_out.detections}
        only = geom_boxes ^ deeplab_boxes
        if not only:
            continue
        ctx = contexts(frame)
        image = read_frame(frame)
        draw_outline(image, ctx.masks["geom"], MASK_BGR)
        draw_outline(image, ctx.masks["deeplab"], (255, 200, 0))
        for box in geom_out.frame.gt_boxes:
            draw_box(image, box, GT_BGR)
        box = next(iter(only))
        draw_box(image, box, TP_BGR if box in geom_boxes else FP_BGR)
        item = tile(crop(image, box, 0.8))
        label_text(
            item,
            "kept by " + ("geom" if box in geom_boxes else "deeplab") + " only",
            (6, 16),
            INK,
            0.45,
        )
        rows.append(item)
        if len(rows) >= limit:
            break
    sheet(
        rows,
        assets / "sheet_mask_disagreements.jpg",
        4,
        "detections one mask kept and the other dropped (orange outline: geom, cyan: deeplab hull)",
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("gt", type=Path)
    parser.add_argument(
        "-o", "--output", type=Path, required=True, help="results dir from rembg_field_predict.py"
    )
    parser.add_argument("--assets", type=Path, required=True, help="where clips and sheets go")
    parser.add_argument("--no-clips", action="store_true")
    args = parser.parse_args()

    cache = MatteCache(args.output / "cache")
    raster = make_raster()
    params = load_params(args.output / "params.json")
    frames = scored_frames(enumerate_frames(args.gt))
    gts, _, _ = load_gt(args.gt)
    args.assets.mkdir(parents=True, exist_ok=True)

    if args.no_clips:
        outcomes = {arm: [] for arm in ARMS}
        for frame in tqdm(frames, desc="outcomes", unit="frame"):
            ctx = FrameContext(frame, cache, raster)
            for arm in ARMS:
                outcomes[arm].append(
                    (frame, outcome_for(gts[frame.stamp_ns], ctx.detections(arm, params[arm])))
                )
    else:
        outcomes = render_flipbooks(frames, cache, raster, params, gts, args.assets)

    raw_matte_sheet(frames, cache, args.assets)
    merged_blob_sheet(outcomes["rembg_deeplab"], args.assets)
    missed_sheet(outcomes["rembg_deeplab"], cache, args.assets)
    parallax_sheet(outcomes["rembg_warped"], args.assets)
    disagreement_sheet(
        outcomes["rembg_geom"],
        outcomes["rembg_deeplab"],
        cache,
        lambda frame: FrameContext(frame, cache, raster),
        args.assets,
    )
    print(f"wrote clips and sheets to {args.assets}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
