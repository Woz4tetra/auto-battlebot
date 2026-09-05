#!/usr/bin/env python3
"""Run rembg on every frame of a short SVO window to see whether detections strobe.

The eval frames are seconds apart, so the flip-books cannot show flicker, and flicker
matters: rembg runs per frame with no temporal state and anything feeding the filter has to
be stable from one frame to the next. This decodes a contiguous window from a source SVO at
native rate, runs the shared raw-frame pass and the `rembg_deeplab` arm on each frame, and
writes a clip plus per-frame counts.

Only `rembg_deeplab` can run here: the other two arms need a camera pose per frame, and
poses exist only for the exported eval frames. The raw matte panel still shows every arm's
shared input, so matte flicker is visible for all three.

Usage:
    venv/bin/python training/model_eval/rembg_field_stability.py data/svo/2026-05-02T11-45-08.svo2 \
        --params training/data/eval_results/rembg/params.json \
        --start-s 200 --seconds 10 -o docs/experiments/perception_performance/assets/<date>_rembg
"""

from __future__ import annotations

import argparse
import contextlib
import json
from pathlib import Path

import cv2
import numpy as np
from rembg_field import (
    FieldSegmenter,
    Saliency,
    largest_component_hull,
    matte_components,
    post_hoc_select,
)
from rembg_field_render import (
    PANEL_H,
    PANEL_W,
    TP_BGR,
    H264Writer,
    draw_box,
    draw_outline,
    fit,
    label_text,
)
from score import iou_matrix
from tqdm import tqdm

from auto_battlebot.svo2 import find_side_by_side_topic, iter_left_jpegs, sample_fps

TRACK_IOU = 0.5


def window_frames(svo: Path, start_s: float, seconds: float):
    """Decode the SVO from the start and yield (index, bgr) for the requested window.

    The stream is intra-refresh H.264 with no seek points, so everything before the window
    is decoded and discarded."""
    topic, count = find_side_by_side_topic(svo)
    fps = sample_fps(svo, topic)
    first = int(start_s * fps)
    last = min(count, first + int(seconds * fps))
    with contextlib.closing(iter_left_jpegs(svo, topic, fps)) as jpegs:
        for index, jpeg in enumerate(jpegs):
            if index >= last:
                break
            if index < first:
                continue
            yield index, cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    return fps


def persistence(previous: list, current: list) -> tuple[int, int]:
    """(detections carried over from the previous frame, detections that are new)."""
    if not previous or not current:
        return 0, len(current)
    ious = iou_matrix(np.asarray([c.box for c in current]), np.asarray([p.box for p in previous]))
    carried = int((ious.max(axis=1) >= TRACK_IOU).sum())
    return carried, len(current) - carried


def panel(image, matte, hull, detections, caption) -> np.ndarray:
    boxed = image.copy()
    for det in detections:
        draw_box(boxed, det.box, TP_BGR)
    left = fit(boxed)
    label_text(left, caption, (8, 18))
    middle = fit(matte)
    label_text(middle, "raw rembg matte", (8, 18))
    masked = cv2.cvtColor(cv2.bitwise_and(matte, hull), cv2.COLOR_GRAY2BGR)
    draw_outline(masked, hull)
    right = fit(masked)
    label_text(right, "after DeepLab hull", (8, 18))
    return np.hstack([left, middle, right])


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("svo", type=Path)
    parser.add_argument("--params", type=Path, required=True, help="params.json from tune")
    parser.add_argument("--start-s", type=float, required=True, help="window start, seconds")
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("-o", "--output", type=Path, required=True, help="assets directory")
    args = parser.parse_args()

    chosen = json.loads(args.params.read_text())["rembg_deeplab"]["chosen"]
    threshold, min_area, overlap = chosen["threshold"], chosen["min_area"], chosen["overlap"]
    saliency = Saliency()
    segmenter = FieldSegmenter()

    topic, _ = find_side_by_side_topic(args.svo)
    fps = sample_fps(args.svo, topic)
    name = f"stability_{args.svo.stem}"
    writer = H264Writer(args.output / f"{name}.mp4", (PANEL_W * 3, PANEL_H), int(round(fps)))
    previous: list = []
    counts = []
    carried_total = 0
    new_total = 0
    for index, image in tqdm(
        window_frames(args.svo, args.start_s, args.seconds), desc=name, unit="frame"
    ):
        matte = saliency.matte(image)
        hull = largest_component_hull(segmenter.raw_mask(image))
        components = matte_components(matte, threshold, {"deeplab": hull})
        detections = post_hoc_select(components, "deeplab", min_area, overlap)
        carried, new = persistence(previous, detections)
        carried_total += carried
        new_total += new
        counts.append(len(detections))
        previous = detections
        writer.write(
            panel(
                image,
                matte,
                hull,
                detections,
                f"{args.svo.stem} frame {index}  {len(detections)} det  "
                f"carried {carried}  new {new}",
            )
        )
    writer.close()

    counts_array = np.asarray(counts)
    changes = int((np.diff(counts_array) != 0).sum()) if len(counts) > 1 else 0
    report = {
        "svo": str(args.svo),
        "start_s": args.start_s,
        "seconds": args.seconds,
        "fps": fps,
        "frames": len(counts),
        "frame_size": [image.shape[1], image.shape[0]],
        "params": chosen,
        "detections_total": int(counts_array.sum()),
        "detections_per_frame": {
            "mean": float(counts_array.mean()),
            "min": int(counts_array.min()),
            "max": int(counts_array.max()),
        },
        "frames_where_count_changed": changes,
        "count_change_rate": changes / max(1, len(counts) - 1),
        "carried_over_from_previous_frame": carried_total,
        "new_this_frame": new_total,
        "persistence_rate": carried_total / max(1, carried_total + new_total),
        "rembg_timing": saliency.timing_summary(),
    }
    (args.output / f"{name}.json").write_text(json.dumps(report, indent=1))
    print(json.dumps(report, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
