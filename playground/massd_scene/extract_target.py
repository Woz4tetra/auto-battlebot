#!/usr/bin/env python3
"""Robot-free target frame from a MassDestruction broadcast clip, banners cropped off.

The Omega broadcast lays a title bar over the top of the picture and a sponsor bar over
the bottom. Both are cropped here (the crop used by
`data/downloads/mass_destruction/MANIFEST.md`: rows 96 to 982 of a 1080p stream), so the
target, the hull mask and every K written downstream describe the same 1920x886 picture.

Unlike the NHRL cage-high flow this takes one hand-checked frame rather than a per-pixel
median: the fights fill the whole clip, so the median still carries robot ghosts, while a
between-round frame is genuinely empty. `--scan` ranks candidate frames by how little of
the floor differs from the clip median and writes a contact sheet to choose from; the
chosen index is then passed with `--frame`.

Output layout (matching playground/cage_scene so the cage-high tools read it unchanged):

    <out>/targets/<clip>/target.png      1920x886, banners cropped
    <out>/targets/<clip>/hull_mask.png   255 inside the cached floor hull
    <out>/targets/<clip>/meta.json       clip, event, source frame, crop

Usage:
    venv/bin/python playground/massd_scene/extract_target.py \\
        data/downloads/massd_resurgence6_mrsbuff/r1_beeroll_vs_mrsbuff.mp4 \\
        --frame 8140 --out runs/massd_scene
    venv/bin/python playground/massd_scene/extract_target.py <video> --scan 12
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from auto_battlebot.segmentation.field_hull import hull_mask_from_polygon, load_hull

BANNER_TOP_ROW = 96
BANNER_BOTTOM_ROW = 982
EVENT = "massd_resurgence6"


def read_frame(video: Path, index: int) -> np.ndarray:
    capture = cv2.VideoCapture(str(video))
    capture.set(cv2.CAP_PROP_POS_FRAMES, index)
    ok, frame = capture.read()
    capture.release()
    if not ok:
        raise SystemExit(f"{video}: frame {index} did not decode")
    return frame


def scan_for_empty(
    video: Path, polygon: np.ndarray, stride: int, count: int
) -> list[tuple[int, float]]:
    """Rank sampled frames by the floor fraction that differs from the clip median.

    Robots, brooms and hands all read as floor that moved. Thin objects score low, so the
    ranking narrows the search rather than deciding it; look at the contact sheet.
    """
    capture = cv2.VideoCapture(str(video))
    thumbs: list[np.ndarray] = []
    indices: list[int] = []
    index = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if index % stride == 0:
            thumbs.append(cv2.resize(frame, (480, 270)))
            indices.append(index)
        index += 1
    capture.release()
    stack = np.array(thumbs, np.uint8)
    median = np.median(stack[::4].astype(np.float32), axis=0)
    mask = np.zeros(stack.shape[1:3], np.uint8)
    cv2.fillPoly(mask, [np.round(polygon * (480 / 1920)).astype(np.int32)], 255)
    inside = cv2.erode(mask, np.ones((9, 9), np.uint8)) > 127
    difference = np.abs(stack.astype(np.float32) - median).mean(axis=3)
    score = (difference[:, inside] > 18).mean(axis=1)
    order = np.argsort(score)[:count]
    return [(indices[i], float(score[i])) for i in order]


def contact_sheet(video: Path, ranked: list[tuple[int, float]], out: Path) -> None:
    tiles = []
    for index, score in ranked:
        tile = cv2.resize(read_frame(video, index), (480, 270))
        cv2.putText(
            tile, f"{index} {score:.4f}", (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2
        )
        tiles.append(tile)
    while len(tiles) % 4:
        tiles.append(np.zeros_like(tiles[0]))
    rows = [np.hstack(tiles[i : i + 4]) for i in range(0, len(tiles), 4)]
    cv2.imwrite(str(out), np.vstack(rows))
    print(f"contact sheet: {out}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("video", type=Path, help="broadcast clip; <video>.hull.json beside it")
    parser.add_argument("--out", type=Path, default=Path("runs/massd_scene"))
    parser.add_argument("--frame", type=int, default=None, help="frame index to keep")
    parser.add_argument(
        "--scan", type=int, default=0, metavar="N", help="rank N candidates and stop"
    )
    parser.add_argument("--scan-stride", type=int, default=5)
    parser.add_argument("--banner-top", type=int, default=BANNER_TOP_ROW)
    parser.add_argument("--banner-bottom", type=int, default=BANNER_BOTTOM_ROW)
    parser.add_argument("--event", default=EVENT)
    args = parser.parse_args()

    hull = load_hull(args.video.with_suffix(".hull.json"))
    polygon = np.array(hull["polygon"], dtype=np.float64)

    if args.scan:
        ranked = scan_for_empty(args.video, polygon, args.scan_stride, args.scan)
        for index, score in ranked:
            print(f"  frame {index:6d}  moved floor {score:.4f}")
        args.out.mkdir(parents=True, exist_ok=True)
        contact_sheet(args.video, ranked, args.out / f"{args.video.stem}_candidates.png")
        return

    if args.frame is None:
        raise SystemExit("pass --frame (use --scan first to find an empty one)")

    frame = read_frame(args.video, args.frame)
    top, bottom = args.banner_top, args.banner_bottom
    cropped = frame[top:bottom]
    target_dir = args.out / "targets" / args.video.stem
    target_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(target_dir / "target.png"), cropped)

    shifted = polygon.copy()
    shifted[:, 1] -= top
    mask = hull_mask_from_polygon(
        np.round(shifted).astype(int).tolist(), (cropped.shape[0], cropped.shape[1])
    )
    cv2.imwrite(str(target_dir / "hull_mask.png"), mask)

    meta = {
        "clip": args.video.stem,
        "video": str(args.video),
        "event": args.event,
        "event_group": args.event,
        "cage": "massd",
        "source_frame": args.frame,
        "samples": 1,
        "banner_crop": [top, bottom],
        "full_size": [frame.shape[1], frame.shape[0]],
        "size": [cropped.shape[1], cropped.shape[0]],
        "hull_fraction": hull.get("hull_fraction"),
        "hull_polygon_cropped": np.round(shifted, 2).tolist(),
    }
    (target_dir / "meta.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(f"{args.video.stem}: frame {args.frame} -> {target_dir}")


if __name__ == "__main__":
    main()
