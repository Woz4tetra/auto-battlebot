#!/usr/bin/env python3
"""Propagate SAM3 seed prompts across full videos.

Reads the seed manifest produced by sam3_floor_segment.py (which contains the
prompt points and seed frame index per video), starts a SAM3 session on the
real video, adds the prompt, and propagates to produce per-frame masks.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from common import (
    LOGGER,
    configure_logging,
    json_dump,
    load_pipeline_config,
    mkdir,
)
from sam3_adapter import build_adapter


def load_jsonl(path: Path) -> list[dict]:
    rows: list[dict] = []
    if not path.exists():
        return rows
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def write_jsonl(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(row, sort_keys=False) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Propagate floor masks with SAM3")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).parent / "config" / "pipeline.toml",
    )
    parser.add_argument("--disable-cache", action="store_true")
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Skip videos that already have propagation results",
    )
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    configure_logging(args.verbose)
    cfg = load_pipeline_config(args.config)
    mkdir(cfg.paths.masks_dir)
    mkdir(cfg.paths.metadata_dir)

    seed_manifest_path = cfg.paths.metadata_dir / "sam3_seed_manifest.jsonl"
    if not seed_manifest_path.exists():
        raise RuntimeError("No seed manifest found. Run sam3_floor_segment.py first.")
    seed_rows = load_jsonl(seed_manifest_path)

    adapter = build_adapter(
        adapter_module=cfg.sam3.adapter_module,
        checkpoint=cfg.sam3.checkpoint,
        device=cfg.sam3.device,
        amp=cfg.sam3.amp,
        model_cfg=cfg.sam3.model_cfg,
    )

    manifest_path = cfg.paths.metadata_dir / "propagation_manifest.jsonl"
    failures_path = cfg.paths.metadata_dir / "propagation_failures.jsonl"
    existing_rows = load_jsonl(manifest_path) if args.resume else []
    done_videos: set[str] = {r["video_path"] for r in existing_rows if "video_path" in r}
    all_rows: list[dict] = list(existing_rows)
    failure_rows: list[dict] = []

    for seed in seed_rows:
        video_path = seed["video_path"]
        if args.resume and video_path in done_videos:
            LOGGER.info("Skipping already-propagated %s", Path(video_path).name)
            continue

        seed_frame_idx = int(seed["seed_frame_idx"])
        pos_pts = np.array(seed["positive_points"], dtype=np.float32)
        neg_pts = np.array(seed["negative_points"], dtype=np.float32)
        points = np.concatenate([pos_pts, neg_pts], axis=0) if len(neg_pts) > 0 else pos_pts
        labels = np.array(
            [1] * len(pos_pts) + [0] * len(neg_pts), dtype=np.int32
        )

        mask_out_dir = mkdir(cfg.paths.masks_dir / Path(video_path).stem / "propagated")
        LOGGER.info("Propagating masks for %s (prompt frame %d)", Path(video_path).name, seed_frame_idx)

        try:
            outputs = adapter.propagate_video(
                video_path=video_path,
                prompt_frame_idx=seed_frame_idx,
                points=points,
                point_labels=labels,
            )
        except Exception as exc:  # noqa: BLE001
            LOGGER.exception("Propagation failed for %s: %s", Path(video_path).name, exc)
            failure_rows.append({
                "video_path": video_path,
                "seed_frame_idx": seed_frame_idx,
                "error": str(exc),
            })
            continue

        LOGGER.info("Got %d frame masks for %s", len(outputs), Path(video_path).name)
        for frame_idx in sorted(outputs.keys()):
            mask = outputs[frame_idx]
            mask_path = mask_out_dir / f"frame_{frame_idx:08d}_mask.png"
            cv2.imwrite(str(mask_path), mask * 255)
            all_rows.append({
                "video_path": video_path,
                "frame_idx": frame_idx,
                "mask_path": str(mask_path),
            })

    write_jsonl(manifest_path, all_rows)
    if failure_rows:
        write_jsonl(failures_path, failure_rows)
    elif failures_path.exists():
        failures_path.unlink(missing_ok=True)

    json_dump(
        cfg.paths.metadata_dir / "propagation_summary.json",
        {
            "num_frames": len(all_rows),
            "manifest": str(manifest_path),
            "failed_videos": len(failure_rows),
            "failures_manifest": str(failures_path) if failure_rows else "",
            "resume_mode": bool(args.resume),
        },
    )
    LOGGER.info(
        "Done. Propagated %d total frame masks (%d failed videos).",
        len(all_rows),
        len(failure_rows),
    )


if __name__ == "__main__":
    main()
