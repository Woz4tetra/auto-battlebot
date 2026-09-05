#!/usr/bin/env python3
"""Render a grid comparing what each pose arm's heading looks like on the same robots.

The figure for `pose_model_size_<date>.md`. `kp_heading_err_deg` is the number that
decides that experiment, and a mean in degrees does not show whether a 15 deg model is
uniformly sloppy or usually right and occasionally reversed. One row per example robot,
one column per arm, every column cropped identically so the eye compares arms directly.

Each tile draws the hand-labeled heading (dashed, aqua) against that arm's predicted
heading (solid, orange) and captions the angle between them. Only two series share a
tile, so no arm is identified by color alone -- the column header names it.

Rows are chosen to span the baseline's heading-error distribution rather than to flatter
any arm: candidates are bucketed by the first arm's error and sampled across buckets,
spread over recordings and classes. Only robots every arm detected are eligible, so a
row is a like-for-like comparison and never a missed detection dressed up as bad heading.

Usage:
    python training/model_eval/make_pose_arms_mosaic.py training/data/nhrl_keypoints_eval_test \
        --candidate n=data/models/yolo26n-pose_all_robot_keypoints_<date>_x86_64_sm86.engine \
        --candidate s=data/models/yolo26s-pose_all_robot_keypoints_<date>_x86_64_sm86.engine \
        --candidate x=data/models/yolo26x-pose_all_robot_keypoints_<date>_x86_64_sm86.engine \
        --labels "mr_stabs_mk2,mrs_buff_mk3,opponent" \
        --taxonomy training/model_eval/taxonomy_keypoint.yaml --conf 0.5 \
        -o docs/experiments/perception_performance/assets/<date>_pose_size/heading_mosaic.png
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
from score import (
    BACK_IDX,
    FRONT_IDX,
    EngineDetector,
    Taxonomy,
    _heading_error_deg,
    infer_frames,
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

# Two categorical slots of the validated default palette, as BGR, matching
# make_centroid_mosaic.py: aqua is the hand-labeled keypoints, orange is a prediction.
# Dashed vs solid separates them a second time so the pair survives a greyscale print.
GT_BGR = (122, 175, 27)  # #1baf7a aqua
PRED_BGR = (52, 104, 235)  # #eb6834 orange
INK = (238, 238, 238)
MUTED = (150, 150, 150)
SURFACE = (26, 26, 26)

CROP_MARGIN = 1.7  # crop side, as a multiple of the GT box's longer side
MIN_BOX_PX = 60  # skip robots too small to read once cropped
EDGE_PX = 3.0  # a GT box within this of the frame border counts as truncated
BUCKETS = 6  # heading-error strata the rows are sampled across


def draw_arrow(tile: np.ndarray, front: np.ndarray, back: np.ndarray, color, dashed: bool) -> None:
    """Back-to-front arrow, so the tile shows the sign of the heading, not just its axis.

    A reversed prediction is the failure mode aim assist cares about most and is
    invisible if the heading is drawn as an undirected line."""
    if not (np.all(np.isfinite(front)) and np.all(np.isfinite(back))):
        return
    a = (int(round(back[0])), int(round(back[1])))
    b = (int(round(front[0])), int(round(front[1])))
    length = float(np.hypot(b[0] - a[0], b[1] - a[1]))
    if length < 1.0:
        return
    tip = min(0.34, 14.0 / length)
    if not dashed:
        for outline, thickness in ((SURFACE, 6), (color, 3)):
            cv2.arrowedLine(tile, a, b, outline, thickness, cv2.LINE_AA, tipLength=tip)
        cv2.circle(tile, a, 5, SURFACE, -1, cv2.LINE_AA)
        cv2.circle(tile, a, 3, color, -1, cv2.LINE_AA)
        return

    # Dashed shaft plus a solid head, drawn by hand: cv2.arrowedLine would lay a solid
    # shaft back over the dashes and the two arrows would then differ only by color.
    step = 9
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
    head = tip * length
    angle = np.arctan2(b[1] - a[1], b[0] - a[0])
    for sign in (-1, 1):
        theta = angle + sign * np.pi / 6
        wing = (int(round(b[0] - head * np.cos(theta))), int(round(b[1] - head * np.sin(theta))))
        cv2.line(tile, b, wing, SURFACE, 5, cv2.LINE_AA)
        cv2.line(tile, b, wing, color, 2, cv2.LINE_AA)


def draw_tile(
    image: np.ndarray, gt_box: np.ndarray, gt_kps: np.ndarray, pred_kps: np.ndarray | None
) -> np.ndarray:
    """Crop around one GT robot and overlay the labeled and predicted headings."""
    side = max(gt_box[2] - gt_box[0], gt_box[3] - gt_box[1]) * CROP_MARGIN
    cx, cy = (gt_box[0] + gt_box[2]) / 2, (gt_box[1] + gt_box[3]) / 2
    x1 = int(round(cx - side / 2))
    y1 = int(round(cy - side / 2))
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
    origin = np.array([x1, y1], dtype=np.float64)

    def to_tile(point: np.ndarray) -> np.ndarray:
        return (point - origin) * scale

    draw_arrow(
        tile, to_tile(gt_kps[FRONT_IDX, :2]), to_tile(gt_kps[BACK_IDX, :2]), GT_BGR, dashed=True
    )
    if pred_kps is not None and len(pred_kps) > max(FRONT_IDX, BACK_IDX):
        draw_arrow(
            tile,
            to_tile(pred_kps[FRONT_IDX, :2]),
            to_tile(pred_kps[BACK_IDX, :2]),
            PRED_BGR,
            dashed=False,
        )
    return tile


def caption_tile(tile: np.ndarray, text: str, emphasis: bool) -> np.ndarray:
    """Stack a one-line caption under a tile."""
    strip = np.full((CAPTION_H, TILE, 3), SURFACE, dtype=np.uint8)
    color = INK if emphasis else MUTED
    cv2.putText(strip, text, (6, 25), FONT, 0.56, color, 1, cv2.LINE_AA)
    return np.vstack([tile, strip])


def header_strip(width: int, text: str) -> np.ndarray:
    strip = np.full((HEADER_H, width, 3), SURFACE, dtype=np.uint8)
    size = cv2.getTextSize(text, FONT, 0.62, 2)[0]
    cv2.putText(strip, text, ((width - size[0]) // 2, 22), FONT, 0.62, INK, 2, cv2.LINE_AA)
    return strip


def legend_strip(width: int) -> np.ndarray:
    """Line key, drawn with the same styles the tiles use.

    Shrinks the text until the whole key fits: a one-column mosaic is narrower than the
    legend's natural width, and a legend clipped mid-word is worse than a small one."""
    strip = np.full((LEGEND_H, width, 3), SURFACE, dtype=np.uint8)
    entries = (
        ("hand-labeled heading", GT_BGR, True),
        ("predicted heading", PRED_BGR, False),
    )
    arrow, gap, margin = 54, 12, 16

    def total(scale: float) -> int:
        text_w = sum(cv2.getTextSize(t, FONT, scale, 1)[0][0] for t, _, _ in entries)
        return 2 * margin + len(entries) * (arrow + gap) + text_w + margin * (len(entries) - 1)

    scale = 0.5
    while scale > 0.3 and total(scale) > width:
        scale -= 0.02

    y = LEGEND_H // 2
    x = margin
    for text, color, dashed in entries:
        draw_arrow(
            strip, np.array([x + float(arrow), float(y)]), np.array([x, float(y)]), color, dashed
        )
        cv2.putText(strip, text, (x + arrow + gap, y + 5), FONT, scale, INK, 1, cv2.LINE_AA)
        x += arrow + gap + cv2.getTextSize(text, FONT, scale, 1)[0][0] + margin
    return strip


def is_truncated(box: np.ndarray, keypoints: np.ndarray, width: int, height: int) -> bool:
    """Whether the robot runs off the frame, which makes it a misleading illustration.

    Same exclusion as make_centroid_mosaic: the labeler clips the box at the border but
    still places keypoints at the chassis ends, so a truncated robot shows a heading
    drawn partly outside its own crop."""
    if (
        box[0] <= EDGE_PX
        or box[1] <= EDGE_PX
        or box[2] >= width - EDGE_PX
        or box[3] >= height - EDGE_PX
    ):
        return True
    return any(
        not (0 <= keypoints[i, 0] <= width and 0 <= keypoints[i, 1] <= height)
        for i in (FRONT_IDX, BACK_IDX)
        if len(keypoints) > i
    )


def infer_all_arms(args: argparse.Namespace, gt_frames: dict, images: dict, taxonomy: Taxonomy):
    """Run every candidate engine over the whole GT set. Returns (arm names, per-arm frames)."""
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


def is_eligible(base, g: int, width: int, height: int) -> bool:
    """Whether GT robot `g` makes a readable, honest tile: labeled, big enough, whole."""
    gt_kps = base.gt_keypoints[g]
    if len(gt_kps) <= max(FRONT_IDX, BACK_IDX):
        return False
    if any(gt_kps[i, 2] <= 0 for i in (FRONT_IDX, BACK_IDX)):
        return False
    gt_box = base.gt_boxes[g]
    if max(gt_box[2] - gt_box[0], gt_box[3] - gt_box[1]) < MIN_BOX_PX:
        return False
    return not is_truncated(gt_box, gt_kps, width, height)


def gather(args: argparse.Namespace) -> tuple[list[str], list[dict]]:
    """Every GT robot that all arms detected, with each arm's predicted keypoints."""
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
            if not all(g in matched[arm] for arm in arms):
                continue
            if not is_eligible(base, g, width, height):
                continue
            gt_box, gt_kps = base.gt_boxes[g], base.gt_keypoints[g]

            preds, errors = {}, {}
            for arm in arms:
                pred_kps = frames[arm].pred_keypoints[matched[arm][g]]
                preds[arm] = pred_kps
                errors[arm] = _heading_error_deg(gt_kps, pred_kps)
            if any(errors[arm] is None for arm in arms):
                continue
            rows.append(
                {
                    "stamp": stamp,
                    "recording": images[stamp].parent.parent.name,
                    "image_path": images[stamp],
                    "label": base.gt_labels[g],
                    "gt_box": gt_box,
                    "gt_kps": gt_kps,
                    "preds": preds,
                    "errors": errors,
                }
            )
    return arms, rows


def select(rows: list[dict], arms: list[str], count: int) -> list[dict]:
    """Sample rows across the baseline arm's heading-error range, spread over recordings.

    Taking the worst N would show only failures and taking a random N would show only
    the easy majority; stratifying on the baseline error shows the range the mean in the
    report is averaging over."""
    if not rows:
        return []
    baseline = arms[0]
    ordered = sorted(rows, key=lambda r: r["errors"][baseline])
    picked: list[dict] = []
    seen_recordings: dict[str, int] = {}
    for bucket in range(BUCKETS):
        lo = bucket * len(ordered) // BUCKETS
        hi = max(lo + 1, (bucket + 1) * len(ordered) // BUCKETS)
        pool = ordered[lo:hi]
        # Prefer a recording (and then a class) the figure has not used yet.
        pool = sorted(
            pool,
            key=lambda r: (seen_recordings.get(r["recording"], 0), -max(r["errors"].values())),
        )
        for row in pool:
            if row["stamp"] in {p["stamp"] for p in picked}:
                continue
            picked.append(row)
            seen_recordings[row["recording"]] = seen_recordings.get(row["recording"], 0) + 1
            break
        if len(picked) >= count:
            break
    return picked[:count]


def render(arms: list[str], picked: list[dict]) -> np.ndarray:
    """Rows of examples, columns of arms, plus a header row and a legend."""
    grid_w = len(arms) * TILE + (len(arms) + 1) * PAD
    headers = [header_strip(TILE, arm) for arm in arms]
    header_row = np.full((HEADER_H, grid_w, 3), SURFACE, dtype=np.uint8)
    for col, strip in enumerate(headers):
        x = PAD + col * (TILE + PAD)
        header_row[:, x : x + TILE] = strip

    body_rows = [header_row]
    for row in picked:
        image = cv2.imread(str(row["image_path"]))
        best = min(row["errors"].values())
        tiles = []
        for arm in arms:
            tile = draw_tile(image, row["gt_box"], row["gt_kps"], row["preds"][arm])
            err = row["errors"][arm]
            tiles.append(caption_tile(tile, f"{arm}: {err:.1f} deg", abs(err - best) < 1e-9))
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
    print(f"{len(rows)} robots detected by all {len(arms)} arms")
    picked = select(rows, arms, args.tiles)
    if not picked:
        raise SystemExit("No robot was detected by every arm; nothing to draw")
    for row in picked:
        errs = ", ".join(f"{arm} {row['errors'][arm]:.1f}" for arm in arms)
        print(f"  {row['recording']} {row['stamp']} {row['label']}: {errs}")

    mosaic = render(arms, picked)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), mosaic)
    print(f"Wrote {args.output} ({mosaic.shape[1]}x{mosaic.shape[0]})")


if __name__ == "__main__":
    main()
