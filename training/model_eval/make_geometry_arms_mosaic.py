#!/usr/bin/env python3
"""Render a grid comparing what each input-geometry arm detects on the same robots.

The examples figure for `input_geometry_<date>.md`. That experiment is decided by agnostic
recall, and a recall number cannot say whether two geometries disagree about a handful of
hard robots or about a broad slice of the eval set. One row per example robot, one column
per arm, every column cropped identically so the eye compares arms directly.

Each tile draws the hand-labeled box (dashed, aqua) against that arm's matched prediction
(solid, orange) and captions the IoU and confidence, or "missed" when the arm found
nothing. The column header names the arm, so no arm is identified by color alone.

Rows are chosen to show disagreement first -- robots some arms found and others missed are
the only frames where geometry changed an outcome -- then filled in across the GT size
range, since object scale is what the geometries move. Rows are spread over recordings.

Usage:
    python training/model_eval/make_geometry_arms_mosaic.py training/data/nhrl_keypoints_eval_test \
        --candidate A_n640sq=data/models/<engine>.engine \
        --candidate A2_n384x640=data/models/<engine>.engine \
        --candidate C_n576x1024=data/models/<engine>.engine \
        --labels "opponent,house_bot" \
        --taxonomy training/model_eval/taxonomy_merged.yaml --conf 0.5 \
        -o docs/experiments/perception_performance/assets/<date>_input_geometry/detections.png
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
from score import (
    EngineDetector,
    Taxonomy,
    infer_frames,
    iou_matrix,
    load_gt,
    match_indices,
    parse_candidates,
)

from auto_battlebot.trt_yolo import TrtYoloModel

TILE = 300
PAD = 6
HEADER_H = 30
LEGEND_H = 44
CAPTION_H = 40
FONT = cv2.FONT_HERSHEY_SIMPLEX

# Same two categorical slots as make_pose_arms_mosaic.py: aqua is hand-labeled, orange is a
# prediction. Dashed vs solid separates them again so the pair survives a greyscale print.
GT_BGR = (122, 175, 27)  # #1baf7a aqua
PRED_BGR = (52, 104, 235)  # #eb6834 orange
MISS_BGR = (70, 70, 210)
INK = (238, 238, 238)
MUTED = (150, 150, 150)
SURFACE = (26, 26, 26)

CROP_MARGIN = 1.9  # crop side, as a multiple of the GT box's longer side
MIN_BOX_PX = 24  # below this the robot is unreadable once cropped, whatever the arm did
EDGE_PX = 3.0  # a GT box within this of the frame border counts as truncated
BUCKETS = 6  # GT size strata the agreement rows are sampled across


def draw_box(tile: np.ndarray, box: np.ndarray, color, dashed: bool) -> None:
    """Rectangle, dashed by hand when it marks ground truth."""
    x1, y1, x2, y2 = (int(round(v)) for v in box)
    if not dashed:
        cv2.rectangle(tile, (x1, y1), (x2, y2), SURFACE, 5, cv2.LINE_AA)
        cv2.rectangle(tile, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)
        return
    step = 9
    corners = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
    for i in range(4):
        a, b = corners[i], corners[(i + 1) % 4]
        length = float(np.hypot(b[0] - a[0], b[1] - a[1]))
        if length < 1.0:
            continue
        for start in range(0, int(length), step * 2):
            end = min(start + step, int(length))
            sa = (
                int(a[0] + (b[0] - a[0]) * start / length),
                int(a[1] + (b[1] - a[1]) * start / length),
            )
            sb = (
                int(a[0] + (b[0] - a[0]) * end / length),
                int(a[1] + (b[1] - a[1]) * end / length),
            )
            cv2.line(tile, sa, sb, SURFACE, 5, cv2.LINE_AA)
            cv2.line(tile, sa, sb, color, 2, cv2.LINE_AA)


def draw_tile(image: np.ndarray, gt_box: np.ndarray, pred_box: np.ndarray | None) -> np.ndarray:
    """Crop around one GT robot and overlay the labeled box and the arm's prediction."""
    side = max(gt_box[2] - gt_box[0], gt_box[3] - gt_box[1]) * CROP_MARGIN
    cx, cy = (gt_box[0] + gt_box[2]) / 2, (gt_box[1] + gt_box[3]) / 2
    x1, y1 = int(round(cx - side / 2)), int(round(cy - side / 2))
    size = max(1, int(round(side)))
    height, width = image.shape[:2]

    # Pad rather than clamp, so the robot stays centered when it sits near an edge.
    crop = np.full((size, size, 3), SURFACE, dtype=np.uint8)
    sx1, sy1 = max(0, x1), max(0, y1)
    sx2, sy2 = min(width, x1 + size), min(height, y1 + size)
    if sx2 > sx1 and sy2 > sy1:
        crop[sy1 - y1 : sy2 - y1, sx1 - x1 : sx2 - x1] = image[sy1:sy2, sx1:sx2]

    scale = TILE / size
    tile = cv2.resize(crop, (TILE, TILE), interpolation=cv2.INTER_CUBIC)
    origin = np.array([x1, y1, x1, y1], dtype=np.float64)

    draw_box(tile, (gt_box - origin) * scale, GT_BGR, dashed=True)
    if pred_box is not None:
        draw_box(tile, (pred_box - origin) * scale, PRED_BGR, dashed=False)
    else:
        cv2.putText(tile, "missed", (10, TILE - 14), FONT, 0.72, SURFACE, 4, cv2.LINE_AA)
        cv2.putText(tile, "missed", (10, TILE - 14), FONT, 0.72, MISS_BGR, 2, cv2.LINE_AA)
    return tile


def caption_tile(tile: np.ndarray, text: str, emphasis: bool) -> np.ndarray:
    strip = np.full((CAPTION_H, TILE, 3), SURFACE, dtype=np.uint8)
    cv2.putText(strip, text, (6, 25), FONT, 0.56, INK if emphasis else MUTED, 1, cv2.LINE_AA)
    return np.vstack([tile, strip])


def header_strip(width: int, text: str) -> np.ndarray:
    strip = np.full((HEADER_H, width, 3), SURFACE, dtype=np.uint8)
    size = cv2.getTextSize(text, FONT, 0.62, 2)[0]
    cv2.putText(strip, text, ((width - size[0]) // 2, 22), FONT, 0.62, INK, 2, cv2.LINE_AA)
    return strip


def legend_strip(width: int) -> np.ndarray:
    strip = np.full((LEGEND_H, width, 3), SURFACE, dtype=np.uint8)
    entries = (("hand-labeled box", GT_BGR, True), ("arm's matched detection", PRED_BGR, False))
    swatch, gap, margin = 54, 12, 16

    def total(scale: float) -> int:
        text_w = sum(cv2.getTextSize(t, FONT, scale, 1)[0][0] for t, _, _ in entries)
        return 2 * margin + len(entries) * (swatch + gap) + text_w + margin * (len(entries) - 1)

    scale = 0.5
    while scale > 0.3 and total(scale) > width:
        scale -= 0.02

    y, x = LEGEND_H // 2, margin
    for text, color, dashed in entries:
        draw_box(strip, np.array([x, y - 9, x + swatch, y + 9]), color, dashed)
        cv2.putText(strip, text, (x + swatch + gap, y + 5), FONT, scale, INK, 1, cv2.LINE_AA)
        x += swatch + gap + cv2.getTextSize(text, FONT, scale, 1)[0][0] + margin
    return strip


def is_truncated(box: np.ndarray, width: int, height: int) -> bool:
    """A robot running off the frame is a misleading illustration, whoever detected it."""
    return bool(
        box[0] <= EDGE_PX
        or box[1] <= EDGE_PX
        or box[2] >= width - EDGE_PX
        or box[3] >= height - EDGE_PX
    )


def infer_all_arms(args: argparse.Namespace, gt_frames: dict, images: dict, taxonomy: Taxonomy):
    class_labels = [label.strip() for label in args.labels.split(",")]
    candidates = parse_candidates(args.candidate)
    per_arm: dict[str, list] = {}
    for name, engine_path in candidates.items():
        if not engine_path.exists():
            raise SystemExit(f"Candidate not found: {engine_path}")
        print(f"Inferring {name}: {engine_path}")
        detector = EngineDetector(
            TrtYoloModel(
                str(engine_path),
                conf_threshold=args.conf,
                nms_iou_threshold=args.nms_iou,
                num_classes=len(class_labels),
            )
        )
        print(f"  {detector.describe()}")
        per_arm[name] = infer_frames(gt_frames, images, detector, class_labels, taxonomy)
    return list(candidates), per_arm


def gather(args: argparse.Namespace) -> tuple[list[str], list[dict]]:
    """Every readable GT robot, with each arm's matched box, IoU and confidence."""
    gt_frames, names, images = load_gt(args.gt)
    taxonomy = Taxonomy(args.taxonomy)
    print(f"GT: {len(gt_frames)} frames, classes: {names}")
    arms, per_arm = infer_all_arms(args, gt_frames, images, taxonomy)

    rows: list[dict] = []
    for fi, stamp in enumerate(gt_frames):
        frames = {arm: per_arm[arm][fi] for arm in arms}
        base = frames[arms[0]]
        matched: dict[str, dict[int, int]] = {}
        for arm in arms:
            pairs = match_indices(frames[arm], args.iou)
            matched[arm] = {g: p for g, p in pairs if g is not None and p is not None}

        image = cv2.imread(str(images[stamp]))
        if image is None:
            raise SystemExit(f"Failed to read image {images[stamp]}")
        height, width = image.shape[:2]

        for g in range(len(base.gt_boxes)):
            gt_box = base.gt_boxes[g]
            side = float(np.sqrt(max(gt_box[2] - gt_box[0], 1) * max(gt_box[3] - gt_box[1], 1)))
            if side < MIN_BOX_PX or is_truncated(gt_box, width, height):
                continue

            found: dict[str, dict[str, float] | None] = {}
            for arm in arms:
                p = matched[arm].get(g)
                if p is None:
                    found[arm] = None
                    continue
                frame = frames[arm]
                iou = float(iou_matrix(gt_box[None, :], frame.pred_boxes[p][None, :])[0, 0])
                found[arm] = {
                    "iou": iou,
                    "score": float(frame.pred_scores[p]),
                    "box": frame.pred_boxes[p],
                }
            rows.append(
                {
                    "stamp": stamp,
                    "recording": images[stamp].parent.parent.name,
                    "image_path": images[stamp],
                    "label": base.gt_labels[g],
                    "gt_box": gt_box,
                    "gt_side": side,
                    "found": found,
                    "n_missed": sum(1 for v in found.values() if v is None),
                }
            )
    return arms, rows


def select(rows: list[dict], arms: list[str], count: int) -> list[dict]:
    """Disagreements first, then agreement rows sampled across the GT size range.

    Only a robot some arms found and others missed shows geometry changing an outcome, so
    those lead. Filling the rest across sizes rather than at random keeps the figure honest
    when the arms tie everywhere -- which is the outcome the plan expects."""
    if not rows:
        return []
    picked: list[dict] = []
    seen: dict[str, int] = {}

    def take(row: dict) -> None:
        picked.append(row)
        seen[row["recording"]] = seen.get(row["recording"], 0) + 1

    disagree = [r for r in rows if 0 < r["n_missed"] < len(arms)]
    # Smallest robots first: a disagreement on a large robot is the more surprising
    # example, but the small end is where the geometries are expected to differ.
    for row in sorted(disagree, key=lambda r: (seen.get(r["recording"], 0), r["gt_side"])):
        if len(picked) >= max(1, count // 2):
            break
        if any(p["stamp"] == row["stamp"] for p in picked):
            continue
        take(row)

    agree = [r for r in rows if r["n_missed"] == 0]
    ordered = sorted(agree, key=lambda r: r["gt_side"])
    for bucket in range(BUCKETS):
        if len(picked) >= count:
            break
        lo = bucket * len(ordered) // BUCKETS
        hi = max(lo + 1, (bucket + 1) * len(ordered) // BUCKETS)
        pool = sorted(ordered[lo:hi], key=lambda r: seen.get(r["recording"], 0))
        for row in pool:
            if any(p["stamp"] == row["stamp"] for p in picked):
                continue
            take(row)
            break
    return picked[:count]


def render(arms: list[str], picked: list[dict]) -> np.ndarray:
    grid_w = len(arms) * TILE + (len(arms) + 1) * PAD
    header_row = np.full((HEADER_H, grid_w, 3), SURFACE, dtype=np.uint8)
    for col, arm in enumerate(arms):
        x = PAD + col * (TILE + PAD)
        header_row[:, x : x + TILE] = header_strip(TILE, arm)

    body_rows = [header_row]
    for row in picked:
        image = cv2.imread(str(row["image_path"]))
        best = max((v["iou"] for v in row["found"].values() if v), default=0.0)
        tiles = []
        for arm in arms:
            hit = row["found"][arm]
            tile = draw_tile(image, row["gt_box"], hit["box"] if hit else None)
            if hit is None:
                caption, emphasis = f"{arm}: missed", False
            else:
                caption = f"{arm}: IoU {hit['iou']:.2f}  conf {hit['score']:.2f}"
                emphasis = abs(hit["iou"] - best) < 1e-9
            tiles.append(caption_tile(tile, caption, emphasis))
        strip = np.full((tiles[0].shape[0] + PAD, grid_w, 3), SURFACE, dtype=np.uint8)
        for col, tile in enumerate(tiles):
            x = PAD + col * (TILE + PAD)
            strip[PAD : PAD + tile.shape[0], x : x + TILE] = tile
        body_rows.append(strip)

    body_rows.append(legend_strip(grid_w))
    return np.vstack(body_rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("gt", type=Path, help="GT dataset dir, or a root of subdatasets")
    parser.add_argument(
        "--candidate",
        action="append",
        required=True,
        metavar="NAME=ENGINE",
        help="candidate TensorRT engine, repeatable; the first is the baseline column",
    )
    parser.add_argument("--labels", required=True, help="GT label per engine class index")
    parser.add_argument("--taxonomy", type=Path, help="label -> archetype mapping yaml")
    parser.add_argument("--iou", type=float, default=0.5, help="IoU match threshold")
    parser.add_argument("--conf", type=float, default=0.5, help="inference confidence threshold")
    parser.add_argument("--nms-iou", type=float, default=0.45, help="inference NMS IoU threshold")
    parser.add_argument("-n", "--tiles", type=int, default=BUCKETS, help="example rows to draw")
    parser.add_argument("-o", "--output", type=Path, required=True, help="output PNG path")
    args = parser.parse_args()

    arms, rows = gather(args)
    picked = select(rows, arms, args.tiles)
    if not picked:
        raise SystemExit("no eligible robots to draw")
    disagreements = sum(1 for r in rows if 0 < r["n_missed"] < len(arms))
    print(f"{len(rows)} eligible robots, {disagreements} where the arms disagree")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), render(arms, picked))
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
