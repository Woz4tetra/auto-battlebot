#!/usr/bin/env python3
"""Stage 4 of the embedding prototype probe: full-playback propagation (Eval C).

Streams every /camera/image frame of each recording's source fight mcap, runs the
deployed bbox engine at low confidence, embeds every proposal with the chosen frozen
embedder, and accepts the most similar proposal above the stage-3 operating point.

The plan's agreement-with-recorded-pipeline metric is dropped: the fight mcaps carry no
/blob_detections topic (only marker topics), so there is no recorded detection stream to
compare against. Remaining Eval C outputs per recording:

- playback_log.csv.gz: every proposal per frame with conf, similarity, accepted flag
- eval_c_metrics.json: accepted-frame fraction, jump rate, static-FP pool stats
  (the stage-3 Eval D static half: proposals parked in place 100+ consecutive frames)

Usage:
    cd training/model_eval && python embedding_probe_playback.py [--recordings 10-06]
        [--embedder clip] [--pad 0.0]
"""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

import embedding_probe_common as common
import numpy as np
import pandas as pd
import yaml
from embedding_probe_embed import Embedder
from embedding_probe_score import PAD_KEYS, aligned_features, gallery_vectors
from make_eval_dataset import left_eye
from tqdm import tqdm

from auto_battlebot.mcap_io import CAMERA_IMAGE_TOPIC, decode_compressed_image, iter_messages

STATIC_CELL_PX = 8
STATIC_MIN_FRAMES = 100
JUMP_FRACTION = 0.2  # of the frame diagonal, between consecutive accepted frames


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recordings", type=str, default="")
    parser.add_argument("--embedder", type=str, default="clip")
    parser.add_argument("--pad", type=float, default=0.0)
    parser.add_argument("--engine", type=Path, default=common.DEFAULT_ENGINE)
    parser.add_argument("--conf", type=float, default=common.LOW_CONF)
    parser.add_argument("--eval-root", type=Path, default=common.DEFAULT_EVAL_ROOT)
    return parser.parse_args()


def source_mcap(recording: str, eval_root: Path) -> Path:
    data = yaml.safe_load((eval_root / recording / "data.yaml").read_text())
    src = Path(data["source_mcap"])
    if src.exists():
        return src
    saved = common.OUTPUT_ROOT.parents[2] / "data" / "saved_recordings"
    hits = sorted(saved.glob(f"*/{src.name}"))
    if not hits:
        raise SystemExit(f"Source mcap for {recording} not found ({src.name})")
    return hits[0]


def pad_key_for(pad: float) -> str:
    for key, value in PAD_KEYS.items():
        if abs(value - pad) < 1e-9:
            return key
    raise SystemExit(f"pad {pad} has no cached embeddings (choices: {list(PAD_KEYS.values())})")


def seed_gallery(recording: str, embedder_name: str, pad: float) -> np.ndarray:
    candidates = common.load_candidates(recording)
    features = aligned_features(recording, embedder_name, pad_key_for(pad), len(candidates))
    return gallery_vectors(candidates, features, "gallery")


def operating_threshold(embedder_name: str, pad: float) -> float:
    metrics = pd.read_csv(common.OUTPUT_ROOT / "metrics.csv")
    row = metrics[
        (metrics.embedder == embedder_name)
        & (metrics.pad == pad_key_for(pad))
        & (metrics.variant == "gallery")
    ]
    if row.empty:
        raise SystemExit("No stage-3 metrics for this embedder/pad; run embedding_probe_score.py")
    return float(row.threshold.iloc[0])


def process_recording(recording: str, args: argparse.Namespace, model, embedder: Embedder) -> dict:
    short = common.short_name(recording)
    mcap_path = source_mcap(recording, args.eval_root)
    gallery = seed_gallery(recording, args.embedder, args.pad)
    threshold = operating_threshold(args.embedder, args.pad)

    rows = []
    accepted_centers: list[tuple[int, float, float]] = []  # frame_idx, cx, cy
    static_counts: dict[tuple[int, int, int, int], int] = defaultdict(int)
    static_sims: dict[tuple[int, int, int, int], list[float]] = defaultdict(list)
    last_seen: dict[tuple[int, int, int, int], int] = {}
    diag = None
    frame_idx = -1
    for _topic, _log_time, data in tqdm(
        iter_messages(mcap_path, [CAMERA_IMAGE_TOPIC]), desc=short, unit="frame"
    ):
        frame_idx += 1
        message = decode_compressed_image(data)
        image = left_eye(message.image)
        if diag is None:
            diag = float(np.hypot(image.shape[1], image.shape[0]))
        detections = model.infer(image)
        if not detections:
            continue
        crops = [common.crop_box(image, det[0], args.pad) for det in detections]
        sims = (embedder(crops) @ gallery.T).max(axis=1)
        best = int(np.argmax(sims))
        for index, (xyxy, conf, class_id, _kps) in enumerate(detections):
            accepted = index == best and sims[index] >= threshold
            cell = (
                int(xyxy[0] // STATIC_CELL_PX),
                int(xyxy[1] // STATIC_CELL_PX),
                int(xyxy[2] // STATIC_CELL_PX),
                int(xyxy[3] // STATIC_CELL_PX),
            )
            if last_seen.get(cell, -2) == frame_idx - 1:
                static_counts[cell] += 1
            else:
                static_counts[cell] = 1
            last_seen[cell] = frame_idx
            if static_counts[cell] >= STATIC_MIN_FRAMES:
                static_sims[cell].append(float(sims[index]))
            rows.append(
                {
                    "frame_idx": frame_idx,
                    "stamp_ns": message.stamp_ns,
                    "x1": float(xyxy[0]),
                    "y1": float(xyxy[1]),
                    "x2": float(xyxy[2]),
                    "y2": float(xyxy[3]),
                    "conf": float(conf),
                    "label": common.ENGINE_LABELS[class_id]
                    if class_id < len(common.ENGINE_LABELS)
                    else f"class_{class_id}",
                    "sim": float(sims[index]),
                    "accepted": accepted,
                }
            )
            if accepted:
                accepted_centers.append(
                    (frame_idx, float((xyxy[0] + xyxy[2]) / 2), float((xyxy[1] + xyxy[3]) / 2))
                )

    n_frames = frame_idx + 1
    log = pd.DataFrame(rows)
    out_dir = common.recording_dir(recording)
    log.to_csv(out_dir / "playback_log.csv.gz", index=False)

    jumps = 0
    for (fa, xa, ya), (fb, xb, yb) in zip(accepted_centers, accepted_centers[1:]):
        if fb - fa <= 3 and np.hypot(xb - xa, yb - ya) > JUMP_FRACTION * (diag or 1.0):
            jumps += 1
    all_static = [sim for sims_list in static_sims.values() for sim in sims_list]
    metrics = {
        "recording": recording,
        "short_name": short,
        "mcap": str(mcap_path),
        "embedder": args.embedder,
        "pad": args.pad,
        "threshold": threshold,
        "n_frames": n_frames,
        "n_proposals": len(log),
        "accepted_frames": int(log.accepted.sum()),
        "accepted_fraction": float(log.accepted.sum() / n_frames) if n_frames else 0.0,
        "jump_count": jumps,
        "jump_rate_per_accepted": jumps / max(int(log.accepted.sum()), 1),
        "static_pool_size": len(all_static),
        "static_false_accepts_at_threshold": int(np.sum(np.asarray(all_static) >= threshold))
        if all_static
        else 0,
    }
    (out_dir / "eval_c_metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    return metrics


def main() -> None:
    args = parse_args()
    import torch

    from auto_battlebot.trt_yolo import TrtYoloModel

    model = TrtYoloModel(
        str(args.engine),
        conf_threshold=args.conf,
        nms_iou_threshold=common.NMS_IOU,
        num_classes=len(common.ENGINE_LABELS),
    )
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    embedder = Embedder(args.embedder, device)
    wanted = {name.strip() for name in args.recordings.split(",") if name.strip()}
    for recording in sorted(common.SHORT_NAMES):
        short = common.short_name(recording)
        if wanted and short not in wanted:
            continue
        metrics = process_recording(recording, args, model, embedder)
        print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
