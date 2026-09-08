#!/usr/bin/env python3
"""Render the domain gap the pose corpus is training across, one band per source.

The supporting figure for `pose_model_size_<date>.md`. That experiment found a bigger
pose backbone buys box detection but not keypoint accuracy, which points at the corpus
rather than the model: `all_robot_keypoints` is 97.8% synthetic renders plus 497 real
frames of one robot, while the eval set is the robot's own ZED looking across a cage.

Three bands, each a row of frames with the hand-labeled heading drawn, so the reader can
see what the model trains on and what it is then asked to generalize to.

Usage:
    python training/model_eval/make_pose_corpus_mosaic.py \
        --train training/data/all_robot_keypoints \
        --eval training/data/nhrl_keypoints_eval_test \
        -o docs/experiments/perception_performance/assets/<date>_pose_size/corpus_mosaic.png
"""

from __future__ import annotations

import argparse
import random
from pathlib import Path

import cv2
import numpy as np
from make_pose_arms_mosaic import GT_BGR, INK, MUTED, SURFACE, draw_arrow
from score import BACK_IDX, FRONT_IDX, Taxonomy, load_gt

TILE_W, TILE_H = 320, 180
PAD = 6
BAND_LABEL_H = 34
FONT = cv2.FONT_HERSHEY_SIMPLEX
MIN_BOX_FRAC = 0.045  # skip frames whose biggest robot is too small to read once shrunk


def draw_frame_tile(
    image: np.ndarray, boxes: list[np.ndarray], kps: list[np.ndarray]
) -> np.ndarray:
    """Whole frame scaled to a tile, with each labeled robot's heading drawn on it.

    Whole frames, not crops: the point of this figure is the background, lighting and
    camera angle around the robot, which a tight crop would remove."""
    height, width = image.shape[:2]
    scale = min(TILE_W / width, TILE_H / height)
    resized = cv2.resize(
        image, (int(width * scale), int(height * scale)), interpolation=cv2.INTER_AREA
    )
    tile = np.full((TILE_H, TILE_W, 3), SURFACE, dtype=np.uint8)
    y0 = (TILE_H - resized.shape[0]) // 2
    x0 = (TILE_W - resized.shape[1]) // 2
    tile[y0 : y0 + resized.shape[0], x0 : x0 + resized.shape[1]] = resized

    origin = np.array([-x0 / scale, -y0 / scale])
    for box, kp in zip(boxes, kps):
        if len(kp) <= max(FRONT_IDX, BACK_IDX):
            continue
        if any(kp[i, 2] <= 0 for i in (FRONT_IDX, BACK_IDX)):
            continue
        x1, y1 = (box[:2] - origin) * scale
        x2, y2 = (box[2:] - origin) * scale
        cv2.rectangle(tile, (int(x1), int(y1)), (int(x2), int(y2)), MUTED, 1, cv2.LINE_AA)
        draw_arrow(
            tile,
            (kp[FRONT_IDX, :2] - origin) * scale,
            (kp[BACK_IDX, :2] - origin) * scale,
            GT_BGR,
            dashed=False,
        )
    return tile


def band(title: str, tiles: list[np.ndarray], width: int) -> np.ndarray:
    """A labeled row of tiles, padded out to the mosaic width."""
    label = np.full((BAND_LABEL_H, width, 3), SURFACE, dtype=np.uint8)
    cv2.putText(label, title, (PAD + 2, 23), FONT, 0.58, INK, 1, cv2.LINE_AA)
    row = np.full((TILE_H + PAD, width, 3), SURFACE, dtype=np.uint8)
    for col, tile in enumerate(tiles):
        x = PAD + col * (TILE_W + PAD)
        if x + TILE_W > width:
            break
        row[0:TILE_H, x : x + TILE_W] = tile
    return np.vstack([label, row])


def read_yolo_labels(
    path: Path, width: int, height: int
) -> tuple[list[np.ndarray], list[np.ndarray]]:
    """Parse one YOLO pose label file into pixel-space boxes and keypoints."""
    boxes, kps = [], []
    for line in path.read_text().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        cx, cy, w, h = (float(v) for v in parts[1:5])
        boxes.append(
            np.array(
                [
                    (cx - w / 2) * width,
                    (cy - h / 2) * height,
                    (cx + w / 2) * width,
                    (cy + h / 2) * height,
                ]
            )
        )
        rest = [float(v) for v in parts[5:]]
        kp = np.array(rest, dtype=np.float64).reshape(-1, 3) if rest else np.zeros((0, 3))
        if len(kp):
            kp[:, 0] *= width
            kp[:, 1] *= height
        kps.append(kp)
    return boxes, kps


# `all_robot_keypoints` names its renders `synthetic__NNNNNN` and `our_robot_keypoints` names
# them `synthetic_keypoints__NNNNNN`. Matching only the first silently puts every render in the
# real-footage band and leaves the synthetic band empty.
SYNTHETIC_PREFIXES = ("synthetic__", "synthetic_keypoints__")


def sample_training(
    root: Path, synthetic: bool, count: int, rng: random.Random
) -> list[np.ndarray]:
    """Tiles from the training corpus, split by whether the frame is a synthetic render."""
    images_dir = root / "train" / "images"
    names = sorted(
        f
        for f in images_dir.iterdir()
        if f.suffix.lower() in (".jpg", ".jpeg", ".png")
        and f.name.startswith(SYNTHETIC_PREFIXES) == synthetic
    )
    rng.shuffle(names)
    tiles = []
    for image_path in names:
        label_path = root / "train" / "labels" / f"{image_path.stem}.txt"
        if not label_path.exists():
            continue
        image = cv2.imread(str(image_path))
        if image is None:
            continue
        height, width = image.shape[:2]
        boxes, kps = read_yolo_labels(label_path, width, height)
        if not boxes:
            continue
        biggest = max((b[2] - b[0]) / width for b in boxes)
        if biggest < MIN_BOX_FRAC:
            continue
        tiles.append(draw_frame_tile(image, boxes, kps))
        if len(tiles) >= count:
            break
    return tiles


def count_training(root: Path) -> tuple[int, int]:
    """(synthetic, real) train-frame counts, so the band captions describe the corpus passed."""
    images_dir = root / "train" / "images"
    synth = real = 0
    for f in images_dir.iterdir():
        if f.suffix.lower() not in (".jpg", ".jpeg", ".png"):
            continue
        if f.name.startswith(SYNTHETIC_PREFIXES):
            synth += 1
        else:
            real += 1
    return synth, real


def sample_eval(root: Path, taxonomy: Taxonomy, count: int) -> list[np.ndarray]:
    """Tiles from the eval set, one per recording so the row spans scenes, not moments."""
    gt_frames, _, images = load_gt(root)
    by_recording: dict[str, list[int]] = {}
    for stamp in gt_frames:
        by_recording.setdefault(images[stamp].parent.parent.name, []).append(stamp)

    tiles = []
    for recording in sorted(by_recording):
        stamps = sorted(by_recording[recording])
        stamp = stamps[len(stamps) // 2]  # mid-recording, past any start-of-fight idle
        boxes_all, labels_all, kps_all = gt_frames[stamp]
        keep = [i for i, lbl in enumerate(labels_all) if lbl not in taxonomy.exclude]
        if not keep:
            continue
        image = cv2.imread(str(images[stamp]))
        if image is None:
            continue
        tiles.append(
            draw_frame_tile(image, [boxes_all[i] for i in keep], [kps_all[i] for i in keep])
        )
        if len(tiles) >= count:
            break
    return tiles


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("--train", type=Path, required=True, help="training corpus root")
    parser.add_argument(
        "--eval", dest="eval_root", type=Path, required=True, help="eval dataset root"
    )
    parser.add_argument("--taxonomy", type=Path, help="label -> archetype mapping yaml")
    parser.add_argument("-n", "--per-band", type=int, default=5, help="tiles per band")
    parser.add_argument("--seed", type=int, default=0, help="sampling seed")
    parser.add_argument("-o", "--output", type=Path, required=True, help="output PNG path")
    args = parser.parse_args()

    rng = random.Random(args.seed)
    taxonomy = Taxonomy(args.taxonomy)
    n_synth, n_real = count_training(args.train)
    total = n_synth + n_real
    bands = [
        (
            f"training corpus - synthetic renders ({n_synth:,} of {total:,} train frames)",
            sample_training(args.train, True, args.per_band, rng),
        ),
        (
            f"training corpus - real footage ({n_real:,} frames, all mrs_buff_mk3 sessions)",
            sample_training(args.train, False, args.per_band, rng),
        ),
        (
            "eval set - the robot's own ZED, one frame per recording",
            sample_eval(args.eval_root, taxonomy, args.per_band),
        ),
    ]
    for title, tiles in bands:
        print(f"{len(tiles)} tiles: {title}")

    width = args.per_band * TILE_W + (args.per_band + 1) * PAD
    mosaic = np.vstack([band(title, tiles, width) for title, tiles in bands if tiles])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), mosaic)
    print(f"Wrote {args.output} ({mosaic.shape[1]}x{mosaic.shape[0]})")


if __name__ == "__main__":
    main()
