#!/usr/bin/env python3
"""Propagate SAM3 seed prompts across full videos.

Reads the seed manifest produced by sam3_floor_segment.py (which contains the
prompt points and seed frame index per video), starts a SAM3 session on the
real video, adds the prompt, and propagates to produce per-frame masks.

Supports multi-GPU: videos are round-robin assigned to GPUs listed in
``[cluster] gpu_ids`` and processed in parallel via ``multiprocessing`` (spawn).
"""

from __future__ import annotations

import argparse
import json
import logging
import multiprocessing as mp
import os
import time
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


# ---------------------------------------------------------------------------
# Per-video propagation (called from worker or main process)
# ---------------------------------------------------------------------------


def _propagate_one_video(
    adapter,
    seed: dict,
    masks_dir: Path,
    gpu_tag: str,
) -> tuple[list[dict], dict | None]:
    """Propagate one video, write masks to disk.

    Returns ``(manifest_rows, failure_or_none)``.
    """
    log = logging.getLogger("floor_pipeline")
    video_path = seed["video_path"]
    vname = Path(video_path).name
    seed_frame_idx = int(seed["seed_frame_idx"])
    pos_pts = np.array(seed["positive_points"], dtype=np.float32)
    neg_pts = np.array(seed["negative_points"], dtype=np.float32)
    points = (
        np.concatenate([pos_pts, neg_pts], axis=0) if len(neg_pts) > 0 else pos_pts
    )
    labels = np.array([1] * len(pos_pts) + [0] * len(neg_pts), dtype=np.int32)

    mask_out_dir = mkdir(masks_dir / Path(video_path).stem / "propagated")
    log.info("%sPropagating %s (prompt frame %d)", gpu_tag, vname, seed_frame_idx)

    try:
        outputs = adapter.propagate_video(
            video_path=video_path,
            prompt_frame_idx=seed_frame_idx,
            points=points,
            point_labels=labels,
        )
    except Exception as exc:  # noqa: BLE001
        log.exception("%sPropagation failed for %s: %s", gpu_tag, vname, exc)
        return [], {
            "video_path": video_path,
            "seed_frame_idx": seed_frame_idx,
            "error": str(exc),
        }

    log.info("%sGot %d frame masks for %s", gpu_tag, len(outputs), vname)
    rows: list[dict] = []
    for frame_idx in sorted(outputs.keys()):
        mask = outputs[frame_idx]
        mask_path = mask_out_dir / f"frame_{frame_idx:08d}_mask.png"
        cv2.imwrite(str(mask_path), mask * 255)
        rows.append(
            {"video_path": video_path, "frame_idx": frame_idx, "mask_path": str(mask_path)}
        )
    return rows, None


# ---------------------------------------------------------------------------
# GPU worker (runs in a spawned subprocess)
# ---------------------------------------------------------------------------


def _gpu_worker(
    gpu_id: int,
    seeds: list[dict],
    config_path: str,
    masks_dir: str,
) -> tuple[list[dict], list[dict]]:
    """Load model on *gpu_id*, propagate the assigned videos."""
    os.environ["CUDA_VISIBLE_DEVICES"] = str(gpu_id)
    os.environ.setdefault("TORCHDYNAMO_DISABLE", "1")

    configure_logging(False)
    log = logging.getLogger("floor_pipeline")
    tag = f"[GPU {gpu_id}] "

    cfg = load_pipeline_config(Path(config_path))

    log.info("%sLoading SAM3 model …", tag)
    t0 = time.monotonic()
    from sam3_adapter import build_adapter

    adapter = build_adapter(
        adapter_module=cfg.sam3.adapter_module,
        checkpoint=cfg.sam3.checkpoint,
        device="cuda",
        amp=cfg.sam3.amp,
        model_cfg=cfg.sam3.model_cfg,
    )
    log.info("%sModel loaded in %.1fs — processing %d videos", tag, time.monotonic() - t0, len(seeds))

    all_rows: list[dict] = []
    failure_rows: list[dict] = []
    masks_path = Path(masks_dir)

    for i, seed in enumerate(seeds):
        log.info("%s[%d/%d] %s", tag, i + 1, len(seeds), Path(seed["video_path"]).name)
        rows, failure = _propagate_one_video(adapter, seed, masks_path, tag)
        all_rows.extend(rows)
        if failure is not None:
            failure_rows.append(failure)

    log.info(
        "%sDone — %d masks, %d failures",
        tag,
        len(all_rows),
        len(failure_rows),
    )
    return all_rows, failure_rows


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


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

    manifest_path = cfg.paths.metadata_dir / "propagation_manifest.jsonl"
    failures_path = cfg.paths.metadata_dir / "propagation_failures.jsonl"
    existing_rows = load_jsonl(manifest_path) if args.resume else []
    done_videos: set[str] = {r["video_path"] for r in existing_rows if "video_path" in r}

    pending = [s for s in seed_rows if s["video_path"] not in done_videos]
    if not pending:
        LOGGER.info("All %d videos already propagated — nothing to do.", len(seed_rows))
        return

    gpu_ids: list[int] = getattr(cfg.cluster, "gpu_ids", [0])
    LOGGER.info(
        "%d pending videos across %d GPU(s): %s",
        len(pending),
        len(gpu_ids),
        gpu_ids,
    )

    all_rows: list[dict] = list(existing_rows)
    failure_rows: list[dict] = []

    if len(gpu_ids) <= 1:
        # Single-GPU fast path: no subprocess overhead.
        rows, failures = _gpu_worker(
            gpu_ids[0], pending, str(args.config), str(cfg.paths.masks_dir),
        )
        all_rows.extend(rows)
        failure_rows.extend(failures)
    else:
        # Round-robin assign videos to GPUs.
        chunks: list[list[dict]] = [[] for _ in gpu_ids]
        for i, seed in enumerate(pending):
            chunks[i % len(gpu_ids)].append(seed)

        ctx = mp.get_context("spawn")
        worker_args = [
            (gid, chunk, str(args.config), str(cfg.paths.masks_dir))
            for gid, chunk in zip(gpu_ids, chunks)
            if chunk
        ]
        with ctx.Pool(processes=len(worker_args)) as pool:
            results = pool.starmap(_gpu_worker, worker_args)

        for rows, failures in results:
            all_rows.extend(rows)
            failure_rows.extend(failures)

    # Persist manifests.
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
