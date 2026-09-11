#!/usr/bin/env python3
"""Mosaic of the sharpest real captures of each robot in an eval dataset.

Supporting figure for the Meshy fidelity grades (`meshy_grade_*.md`). Those experiments
ask whether a Meshy mesh built from a stale thumbnail still looks like the robot the
camera sees at fight time, so the reader needs the clearest real views of each robot to
compare against. One row per robot, the N best crops from its ground-truth boxes.

Our robots are read from their own GT label. The eval GT labels every opponent as a
generic `opponent`, so each opponent is named by the recording it fought in
(`--opponent name=recording_prefix`), the same map the grade uses.

"Best" is the variance of the Laplacian over the crop after it is resized to a fixed
edge, which prefers sharp, large boxes and penalizes motion blur and small robots alike
(a small crop is upsampled and reads as soft). Boxes touching the frame edge are skipped
since they are partly out of view. Picks from one recording are kept at least
`--min-gap` seconds apart, otherwise a robot that sits still for ten seconds fills its
whole row with the same view.

Usage:
    python training/model_eval/make_robot_capture_mosaic.py \
        training/data/nhrl_keypoints_eval_test \
        -o docs/experiments/perception_performance/assets/<date>/robot_captures_mosaic.png
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from auto_battlebot.eval.dataset import load_gt

TILE = 224
CAPTION_H = 22
ROW_LABEL_W = 170
PAD = 6
CONTEXT = 0.2  # fraction of the box added around it on each side
SHARPNESS_EDGE = 160  # crop's longer side when sharpness is measured
MIN_SIDE_PX = 24
EDGE_MARGIN_PX = 2
FONT = cv2.FONT_HERSHEY_SIMPLEX
SURFACE = (250, 250, 250)
INK = (30, 30, 30)
MUTED = (120, 120, 120)

DEFAULT_OPPONENTS = (
    "clyde=main_2026-05-02_10-06",
    "sphinx=main_2026-05-02_11-45",
    "wreckcreation=main_2026-05-02_14-12",
    "ironwarrior=main_2026-05-02_15-35",
)
OUR_ROBOTS = ("mr_stabs_mk2", "mrs_buff_mk3")


@dataclass
class Capture:
    robot: str
    stamp: int
    recording: str
    image_path: Path
    box: np.ndarray  # xyxy pixels
    sharpness: float
    seconds: float  # since the recording's first GT frame


def sharpness_of(crop: np.ndarray) -> float:
    height, width = crop.shape[:2]
    scale = SHARPNESS_EDGE / max(height, width)
    interp = cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR
    resized = cv2.resize(
        crop, (max(1, int(width * scale)), max(1, int(height * scale))), interpolation=interp
    )
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def recording_of(image_path: Path) -> str:
    """Subdataset directory name: <root>/<recording>/images/<stamp>.png."""
    return image_path.parent.parent.name


def short_recording(name: str) -> str:
    """`main_2026-05-02_10-06-02_repaired__...` -> `05-02_10-06`; MassD keeps its own tag."""
    if name.startswith("main_"):
        return name[len("main_2026-") : len("main_2026-") + 11]
    return name.split("_")[3] if name.count("_") >= 3 else name


def collect(
    root: Path, opponents: dict[str, str]
) -> tuple[dict[str, list[Capture]], dict[str, int]]:
    frames, _, images = load_gt(root)
    picks: dict[str, list[Capture]] = {name: [] for name in (*OUR_ROBOTS, *opponents)}
    seen: dict[str, int] = {name: 0 for name in picks}
    cache: dict[Path, np.ndarray] = {}
    starts: dict[str, int] = {}
    for stamp, image_path in images.items():
        recording = recording_of(image_path)
        starts[recording] = min(starts.get(recording, stamp), stamp)
    for stamp, (boxes, labels, _) in frames.items():
        image_path = images[stamp]
        recording = recording_of(image_path)
        opponent_here = next(
            (name for name, prefix in opponents.items() if recording.startswith(prefix)), None
        )
        for box, label in zip(boxes, labels):
            if label in OUR_ROBOTS:
                robot = label
            elif label == "opponent" and opponent_here is not None:
                robot = opponent_here
            else:
                continue
            seen[robot] += 1
            if image_path not in cache:
                cache.clear()
                cache[image_path] = cv2.imread(str(image_path))
            image = cache[image_path]
            height, width = image.shape[:2]
            x1, y1, x2, y2 = box
            if (
                x1 < EDGE_MARGIN_PX
                or y1 < EDGE_MARGIN_PX
                or x2 > width - EDGE_MARGIN_PX
                or y2 > height - EDGE_MARGIN_PX
            ):
                continue
            if min(x2 - x1, y2 - y1) < MIN_SIDE_PX:
                continue
            crop = image[int(y1) : int(y2), int(x1) : int(x2)]
            picks[robot].append(
                Capture(
                    robot,
                    stamp,
                    recording,
                    image_path,
                    np.asarray(box),
                    sharpness_of(crop),
                    (stamp - starts[recording]) / 1e9,
                )
            )
    for name in picks:
        picks[name].sort(key=lambda c: c.sharpness, reverse=True)
    return picks, seen


def tile_of(capture: Capture, image: np.ndarray) -> np.ndarray:
    height, width = image.shape[:2]
    x1, y1, x2, y2 = capture.box
    side = max(x2 - x1, y2 - y1) * (1 + 2 * CONTEXT)
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    left, top = int(round(cx - side / 2)), int(round(cy - side / 2))
    right, bottom = int(round(cx + side / 2)), int(round(cy + side / 2))
    canvas = np.full((bottom - top, right - left, 3), SURFACE, dtype=np.uint8)
    sx1, sy1, sx2, sy2 = max(left, 0), max(top, 0), min(right, width), min(bottom, height)
    canvas[sy1 - top : sy2 - top, sx1 - left : sx2 - left] = image[sy1:sy2, sx1:sx2]
    interp = cv2.INTER_AREA if canvas.shape[0] > TILE else cv2.INTER_LINEAR
    tile = np.full((TILE + CAPTION_H, TILE, 3), SURFACE, dtype=np.uint8)
    tile[:TILE] = cv2.resize(canvas, (TILE, TILE), interpolation=interp)
    caption = f"{short_recording(capture.recording)} +{capture.seconds:.0f}s  {int(x2 - x1)}px"
    cv2.putText(tile, caption, (4, TILE + 15), FONT, 0.42, MUTED, 1, cv2.LINE_AA)
    return tile


def spread(captures: list[Capture], count: int, min_gap_s: float) -> list[Capture]:
    """Greedy pick in sharpness order, refusing a capture within min_gap of a picked one."""
    chosen: list[Capture] = []
    for capture in captures:
        if all(
            c.recording != capture.recording or abs(c.stamp - capture.stamp) / 1e9 >= min_gap_s
            for c in chosen
        ):
            chosen.append(capture)
        if len(chosen) >= count:
            break
    return chosen


def row_of(robot: str, captures: list[Capture], count: int, seen: int) -> np.ndarray:
    row_h = TILE + CAPTION_H
    row = np.full((row_h + PAD, ROW_LABEL_W + count * (TILE + PAD), 3), SURFACE, dtype=np.uint8)
    cv2.putText(row, robot, (PAD, 28), FONT, 0.62, INK, 1, cv2.LINE_AA)
    cv2.putText(row, f"{seen} GT boxes", (PAD, 52), FONT, 0.45, MUTED, 1, cv2.LINE_AA)
    for col, capture in enumerate(captures[:count]):
        image = cv2.imread(str(capture.image_path))
        x = ROW_LABEL_W + col * (TILE + PAD)
        row[0:row_h, x : x + TILE] = tile_of(capture, image)
    return row


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("gt", type=Path, help="eval dataset root (dir of subdatasets)")
    parser.add_argument("-o", "--output", type=Path, required=True)
    parser.add_argument("-n", "--count", type=int, default=8, help="captures per robot")
    parser.add_argument(
        "--opponent",
        action="append",
        metavar="NAME=RECORDING_PREFIX",
        help="opponent name and the recording it fought in; repeatable. "
        f"Default: {' '.join(DEFAULT_OPPONENTS)}",
    )
    parser.add_argument(
        "--min-gap", type=float, default=4.0, help="seconds between picks from one recording"
    )
    args = parser.parse_args()
    opponents = dict(spec.split("=", 1) for spec in (args.opponent or DEFAULT_OPPONENTS))

    ranked, seen = collect(args.gt, opponents)
    picks = {name: spread(captures, args.count, args.min_gap) for name, captures in ranked.items()}
    for name in [name for name, captures in picks.items() if not captures]:
        print(f"{name}: no usable captures in {args.gt}, row omitted")
        del picks[name]
    rows = [row_of(name, captures, args.count, seen[name]) for name, captures in picks.items()]
    mosaic = np.vstack(rows)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), mosaic)

    print(f"{'robot':<14}{'GT':>5}{'usable':>8}  shown (recording sharpness/box px)")
    for name, captures in picks.items():
        shown = ", ".join(
            f"{short_recording(c.recording)} {c.sharpness:.0f}/{int(c.box[2] - c.box[0])}"
            for c in captures
        )
        print(f"{name:<14}{seen[name]:>5}{len(ranked[name]):>8}  {shown}")
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
