#!/usr/bin/env python3
"""Generate SAM3 seed masks per video chunk for floor segmentation."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from common import (
    LOGGER,
    append_jsonl,
    configure_logging,
    json_dump,
    json_load,
    load_pipeline_config,
    mkdir,
)
from sam3_adapter import SegmentationSeed, build_adapter


def frame_at(video_path: Path, frame_idx: int) -> np.ndarray:
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"Failed opening video: {video_path}")
    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
    ok, frame = cap.read()
    cap.release()
    if not ok or frame is None:
        raise RuntimeError(f"Failed reading frame {frame_idx} from {video_path}")
    return frame


def infer_arena_box(frame_bgr: np.ndarray) -> tuple[int, int, int, int]:
    """Estimate arena interior bounding box using conservative inset."""
    h, w = frame_bgr.shape[:2]
    inset_x = int(0.06 * w)
    inset_y = int(0.06 * h)
    return (inset_x, inset_y, w - inset_x, h - inset_y)


def _sample_inside_box(
    box: tuple[int, int, int, int],
    count: int,
    margin: int = 8,
) -> list[tuple[int, int]]:
    x1, y1, x2, y2 = box
    xs = np.random.randint(x1 + margin, max(x1 + margin + 1, x2 - margin), size=count)
    ys = np.random.randint(y1 + margin, max(y1 + margin + 1, y2 - margin), size=count)
    return [(int(x), int(y)) for x, y in zip(xs, ys)]


def _sample_outside_box(
    shape: tuple[int, int, int],
    box: tuple[int, int, int, int],
    count: int,
) -> list[tuple[int, int]]:
    h, w = shape[:2]
    x1, y1, x2, y2 = box
    points: list[tuple[int, int]] = []
    while len(points) < count:
        x = int(np.random.randint(0, w))
        y = int(np.random.randint(0, h))
        if x < x1 or x >= x2 or y < y1 or y >= y2:
            points.append((x, y))
    return points


def generate_seed_points(
    frame_bgr: np.ndarray,
    arena_box: tuple[int, int, int, int],
    pos_count: int,
    neg_count: int,
) -> tuple[list[tuple[int, int]], list[tuple[int, int]]]:
    """Generate automatic positive/negative points for fallback prompting."""
    # Positive points are sampled inside the estimated arena interior.
    positive = _sample_inside_box(arena_box, pos_count)
    # Negative points sampled outside arena boundaries.
    negative = _sample_outside_box(frame_bgr.shape, arena_box, neg_count)
    return positive, negative


def chunk_seed_frame(chunk: dict, fps: float) -> int:
    # Seed from early in each chunk to preserve temporal continuity.
    return int(chunk["start_seconds"] * fps) + 3


def parse_fps(value: str | float | int) -> float:
    if isinstance(value, (int, float)):
        return float(value)
    if "/" in value:
        num, den = value.split("/", maxsplit=1)
        return float(num) / float(den)
    return float(value)


def main() -> None:
    parser = argparse.ArgumentParser(description="Create SAM3 seed masks for each video chunk")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).parent / "config" / "pipeline.toml",
    )
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    configure_logging(args.verbose)
    cfg = load_pipeline_config(args.config)
    mkdir(cfg.paths.masks_dir)
    mkdir(cfg.paths.metadata_dir)

    transcode_index = json_load(cfg.paths.metadata_dir / "transcode_index.json", default=[])
    if not transcode_index:
        raise RuntimeError("No transcode index found. Run transcode_videos.py first.")

    adapter = build_adapter(
        adapter_module=cfg.sam3.adapter_module,
        checkpoint=cfg.sam3.checkpoint,
        device=cfg.sam3.device,
        amp=cfg.sam3.amp,
        model_cfg=cfg.sam3.model_cfg,
    )

    seed_manifest_rows: list[dict] = []
    for video_meta in transcode_index:
        video_path = Path(video_meta["output_video"])
        stream_info = video_meta.get("stream_info", {})
        fps_text = stream_info.get("avg_frame_rate") or stream_info.get("r_frame_rate") or "30/1"
        fps = parse_fps(fps_text)
        video_out_dir = mkdir(cfg.paths.masks_dir / video_path.stem / "seeds")

        LOGGER.info("Generating seed masks for %s", video_path.name)
        for chunk in video_meta["chunks"]:
            seed_idx = chunk_seed_frame(chunk, fps)
            frame = frame_at(video_path, seed_idx)
            arena_box = infer_arena_box(frame)
            pos_points, neg_points = generate_seed_points(
                frame,
                arena_box,
                cfg.sam3.fallback_positive_samples,
                cfg.sam3.fallback_negative_samples,
            )
            seed = SegmentationSeed(
                frame_bgr=frame,
                prompt_tags=list(cfg.sam3.prompt_tags),
                negative_tags=list(cfg.sam3.negative_tags),
                text_threshold=cfg.sam3.text_threshold,
                box_threshold=cfg.sam3.box_threshold,
                positive_points_xy=pos_points,
                negative_points_xy=neg_points,
                arena_box_xyxy=arena_box,
            )
            mask = adapter.segment_seed(seed)
            mask = (mask > 0).astype(np.uint8)
            mask_path = video_out_dir / f"chunk_{chunk['chunk_index']:05d}_seed_mask.png"
            cv2.imwrite(str(mask_path), mask * 255)

            row = {
                "video_path": str(video_path),
                "chunk_index": chunk["chunk_index"],
                "seed_frame_idx": seed_idx,
                "arena_box_xyxy": list(arena_box),
                "seed_mask_path": str(mask_path),
                "start_seconds": chunk["start_seconds"],
                "end_seconds": chunk["end_seconds"],
                "prompt_tags": cfg.sam3.prompt_tags,
                "negative_tags": cfg.sam3.negative_tags,
            }
            seed_manifest_rows.append(row)

    seed_manifest = cfg.paths.metadata_dir / "sam3_seed_manifest.jsonl"
    append_jsonl(seed_manifest, seed_manifest_rows)
    json_dump(
        cfg.paths.metadata_dir / "sam3_seed_summary.json",
        {"num_seeds": len(seed_manifest_rows), "manifest": str(seed_manifest)},
    )
    LOGGER.info("Done. Wrote %d chunk seed masks.", len(seed_manifest_rows))


if __name__ == "__main__":
    main()
