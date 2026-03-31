#!/usr/bin/env python3
"""Propagate SAM3 seed masks across full videos in chunked windows."""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
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
from sam3_adapter import build_adapter


def parse_fps(fps_text: str | float | int) -> float:
    if isinstance(fps_text, (int, float)):
        return float(fps_text)
    if "/" in fps_text:
        num, den = fps_text.split("/", maxsplit=1)
        return float(num) / float(den)
    return float(fps_text)


def frame_range_from_chunk(chunk: dict, fps: float, overlap_frames: int) -> tuple[int, int]:
    start_idx = max(0, int(chunk["start_seconds"] * fps) - overlap_frames)
    end_idx = max(start_idx, int(chunk["end_seconds"] * fps) + overlap_frames)
    return start_idx, end_idx


def read_seed_mask(mask_path: Path) -> np.ndarray:
    mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
    if mask is None:
        raise RuntimeError(f"Failed to read seed mask: {mask_path}")
    return (mask > 0).astype(np.uint8)


def main() -> None:
    parser = argparse.ArgumentParser(description="Propagate floor masks with SAM3")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).parent / "config" / "pipeline.toml",
    )
    parser.add_argument("--overlap-frames", type=int, default=12)
    parser.add_argument("--disable-cache", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    configure_logging(args.verbose)
    cfg = load_pipeline_config(args.config)
    mkdir(cfg.paths.masks_dir)
    mkdir(cfg.paths.metadata_dir)

    transcode_index = json_load(cfg.paths.metadata_dir / "transcode_index.json", default=[])
    seed_manifest_path = cfg.paths.metadata_dir / "sam3_seed_manifest.jsonl"
    if not seed_manifest_path.exists():
        raise RuntimeError("No seed manifest found. Run sam3_floor_segment.py first.")

    by_video_chunks: dict[str, list[dict]] = defaultdict(list)
    with seed_manifest_path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            by_video_chunks[row["video_path"]].append(row)

    transcode_map = {str(v["output_video"]): v for v in transcode_index}
    adapter = build_adapter(
        adapter_module=cfg.sam3.adapter_module,
        checkpoint=cfg.sam3.checkpoint,
        device=cfg.sam3.device,
        amp=cfg.sam3.amp,
        model_cfg=cfg.sam3.model_cfg,
    )

    propagation_rows: list[dict] = []
    for video_path_str, chunk_rows in by_video_chunks.items():
        video_path = Path(video_path_str)
        video_meta = transcode_map.get(video_path_str)
        if video_meta is None:
            LOGGER.warning("Skipping unknown video from seed manifest: %s", video_path_str)
            continue
        stream_info = video_meta.get("stream_info", {})
        fps = parse_fps(stream_info.get("avg_frame_rate", "30/1"))

        mask_out_dir = mkdir(cfg.paths.masks_dir / video_path.stem / "propagated")
        LOGGER.info("Propagating masks for %s", video_path.name)
        for row in sorted(chunk_rows, key=lambda x: x["chunk_index"]):
            chunk_meta = video_meta["chunks"][row["chunk_index"]]
            start_idx, end_idx = frame_range_from_chunk(chunk_meta, fps, args.overlap_frames)
            seed_mask = read_seed_mask(Path(row["seed_mask_path"]))
            outputs = adapter.propagate_video(
                video_path=video_path,
                initial_mask=seed_mask,
                start_frame_idx=start_idx,
                end_frame_idx=end_idx,
                async_prefetch=cfg.cluster.async_prefetch,
                disable_cache=args.disable_cache,
            )
            for frame_idx, mask in outputs.items():
                mask_u8 = (mask > 0).astype(np.uint8) * 255
                mask_path = mask_out_dir / f"frame_{frame_idx:08d}_mask.png"
                cv2.imwrite(str(mask_path), mask_u8)
                propagation_rows.append(
                    {
                        "video_path": str(video_path),
                        "chunk_index": row["chunk_index"],
                        "frame_idx": frame_idx,
                        "mask_path": str(mask_path),
                    }
                )

    manifest_path = cfg.paths.metadata_dir / "propagation_manifest.jsonl"
    append_jsonl(manifest_path, propagation_rows)
    json_dump(
        cfg.paths.metadata_dir / "propagation_summary.json",
        {"num_frames": len(propagation_rows), "manifest": str(manifest_path)},
    )
    LOGGER.info("Done. Propagated %d frame masks.", len(propagation_rows))


if __name__ == "__main__":
    main()
