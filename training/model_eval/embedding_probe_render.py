#!/usr/bin/env python3
"""Stage 5 of the embedding prototype probe: overlay videos from the stage-4 logs.

Re-streams each recording's fight mcap and draws, per frame:

- every low-conf proposal, colored by similarity to the seed gallery (blue 0 -> red 1),
  labeled "sim/conf"
- the accepted proposal (top similarity above the operating point), thick green
- GT boxes on validated eval frames (white: opponent, gray: everything else),
  held on screen for a few frames so they are visible at playback speed
- a HUD with frame index, time, and the operating point

Output: training/data/embedding_probe/<recording>/overlay.mp4 at the recording's native
frame rate.

Usage:
    cd training/model_eval && python embedding_probe_render.py [--recordings 10-06]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import embedding_probe_common as common
import numpy as np
import pandas as pd
from embedding_probe_playback import source_mcap
from make_eval_dataset import left_eye
from tqdm import tqdm

from auto_battlebot.mcap_io import CAMERA_IMAGE_TOPIC, decode_compressed_image, iter_messages

GT_HOLD_FRAMES = 8


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recordings", type=str, default="")
    parser.add_argument("--eval-root", type=Path, default=common.DEFAULT_EVAL_ROOT)
    return parser.parse_args()


def sim_color(sim: float) -> tuple[int, int, int]:
    """Blue (0) to red (1) through purple, BGR."""
    sim = float(np.clip(sim, 0.0, 1.0))
    return (int(255 * (1 - sim)), 40, int(255 * sim))


def draw_box(image, box, color, thickness, label=""):
    x1, y1, x2, y2 = (int(round(v)) for v in box)
    cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)
    if label:
        origin = (x1, max(12, y1 - 4))
        cv2.putText(image, label, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)


def draw_gt(image, gt_hold: list) -> None:
    for entry in gt_hold:
        entry[0] -= 1
        for _, row in entry[1].iterrows():
            color = (255, 255, 255) if row.label == common.OPPONENT else (160, 160, 160)
            draw_box(image, (row.x1, row.y1, row.x2, row.y2), color, 1, f"GT {row.label}")


def draw_proposals(image, group) -> None:
    if group is None:
        return
    for _, row in group.iterrows():
        if row.accepted:
            continue
        box = (row.x1, row.y1, row.x2, row.y2)
        draw_box(image, box, sim_color(row.sim), 1, f"{row.sim:.2f}/{row.conf:.2f}")
    for _, row in group[group.accepted].iterrows():
        box = (row.x1, row.y1, row.x2, row.y2)
        draw_box(image, box, (0, 220, 0), 3, f"ACCEPT {row.sim:.2f}")


def render_recording(recording: str, args: argparse.Namespace) -> Path:
    short = common.short_name(recording)
    out_dir = common.recording_dir(recording)
    log_path = out_dir / "playback_log.csv.gz"
    if not log_path.exists():
        raise SystemExit(f"{short}: no playback log; run embedding_probe_playback.py first")
    log = pd.read_csv(log_path)
    by_frame = dict(tuple(log.groupby("frame_idx")))
    import json

    metrics = json.loads((out_dir / "eval_c_metrics.json").read_text())
    threshold = metrics["threshold"]

    candidates = common.load_candidates(recording)
    gt = candidates[candidates.source == "gt"]
    gt_by_stamp = dict(tuple(gt.groupby("stamp_ns")))

    mcap_path = source_mcap(recording, args.eval_root)
    stamps = log.drop_duplicates("frame_idx").sort_values("frame_idx").stamp_ns.to_numpy()
    fps = 1e9 / float(np.median(np.diff(stamps))) if len(stamps) > 2 else 30.0

    writer = None
    out_path = out_dir / "overlay.mp4"
    frame_idx = -1
    t0 = None
    gt_hold: list = []  # (frames_left, rows)
    for _topic, _log_time, data in tqdm(
        iter_messages(mcap_path, [CAMERA_IMAGE_TOPIC]), desc=short, unit="frame"
    ):
        frame_idx += 1
        message = decode_compressed_image(data)
        image = left_eye(message.image).copy()
        if t0 is None:
            t0 = message.stamp_ns
        if writer is None:
            writer = cv2.VideoWriter(
                str(out_path),
                cv2.VideoWriter_fourcc(*"mp4v"),
                min(max(fps, 5.0), 60.0),
                (image.shape[1], image.shape[0]),
            )

        if message.stamp_ns in gt_by_stamp:
            gt_hold.append([GT_HOLD_FRAMES, gt_by_stamp[message.stamp_ns]])
        gt_hold = [[left, rows] for left, rows in gt_hold if left > 0]
        draw_gt(image, gt_hold)
        draw_proposals(image, by_frame.get(frame_idx))

        elapsed = (message.stamp_ns - t0) / 1e9
        hud = (
            f"{short}  frame {frame_idx}  t={elapsed:7.2f}s  "
            f"{metrics['embedder']} pad{int(metrics['pad'] * 100):02d}  thr={threshold:.3f}"
        )
        cv2.putText(image, hud, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(
            image, hud, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (240, 240, 240), 1, cv2.LINE_AA
        )
        writer.write(image)

    if writer is not None:
        writer.release()
    return out_path


def main() -> None:
    args = parse_args()
    wanted = {name.strip() for name in args.recordings.split(",") if name.strip()}
    for recording in sorted(common.SHORT_NAMES):
        short = common.short_name(recording)
        if wanted and short not in wanted:
            continue
        out_path = render_recording(recording, args)
        print(f"{short}: {out_path}")


if __name__ == "__main__":
    main()
