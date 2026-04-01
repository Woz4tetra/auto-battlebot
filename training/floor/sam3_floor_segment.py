#!/usr/bin/env python3
"""Generate SAM3 seed masks for each transcoded video.

For each video in the transcode index, pick a seed frame early in the video,
generate positive points inside the estimated arena box and negative points
outside it, then call the SAM3 adapter on the REAL video to get a mask.
"""

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
from sam3_adapter import build_adapter


def parse_fps(value: str | float | int) -> float:
    if isinstance(value, (int, float)):
        return float(value)
    if "/" in value:
        num, den = value.split("/", maxsplit=1)
        return float(num) / float(den)
    return float(value)


def infer_arena_box(h: int, w: int) -> tuple[int, int, int, int]:
    """Conservative inset to estimate the arena interior."""
    ix, iy = int(0.06 * w), int(0.06 * h)
    return (ix, iy, w - ix, h - iy)


def sample_points(
    h: int,
    w: int,
    arena_box: tuple[int, int, int, int],
    n_pos: int,
    n_neg: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Sample positive points inside the arena box, negative points outside."""
    x1, y1, x2, y2 = arena_box
    margin = 8
    pos_xs = np.random.randint(x1 + margin, max(x1 + margin + 1, x2 - margin), size=n_pos)
    pos_ys = np.random.randint(y1 + margin, max(y1 + margin + 1, y2 - margin), size=n_pos)

    neg_pts: list[tuple[int, int]] = []
    while len(neg_pts) < n_neg:
        x, y = int(np.random.randint(0, w)), int(np.random.randint(0, h))
        if x < x1 or x >= x2 or y < y1 or y >= y2:
            neg_pts.append((x, y))

    all_x = list(pos_xs) + [p[0] for p in neg_pts]
    all_y = list(pos_ys) + [p[1] for p in neg_pts]
    points = np.array(list(zip(all_x, all_y)), dtype=np.float32)
    labels = np.array([1] * n_pos + [0] * n_neg, dtype=np.int32)
    return points, labels


def pick_seed_frame(video_path: Path, fps: float) -> int:
    """Pick a frame a few seconds into the video (after intros/transitions)."""
    cap = cv2.VideoCapture(str(video_path))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    cap.release()
    target = int(3.0 * fps)  # 3 seconds in
    return min(target, max(0, total - 1))


def get_frame_size(video_path: Path) -> tuple[int, int]:
    cap = cv2.VideoCapture(str(video_path))
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cap.release()
    return h, w


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate SAM3 seed masks")
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

    seed_rows: list[dict] = []
    skipped = 0
    for i, video_meta in enumerate(transcode_index, 1):
        video_path = Path(video_meta["output_video"])
        stream_info = video_meta.get("stream_info", {})
        fps = parse_fps(
            stream_info.get("avg_frame_rate") or stream_info.get("r_frame_rate") or "30/1"
        )

        h, w = get_frame_size(video_path)
        seed_idx = pick_seed_frame(video_path, fps)
        arena_box = infer_arena_box(h, w)
        points, labels = sample_points(
            h, w, arena_box,
            n_pos=cfg.sam3.fallback_positive_samples,
            n_neg=cfg.sam3.fallback_negative_samples,
        )

        LOGGER.info("[%d/%d] Generating seed mask for %s (frame %d)",
                     i, len(transcode_index), video_path.name, seed_idx)
        try:
            mask = adapter.segment_frame(
                video_path=video_path,
                frame_idx=seed_idx,
                points=points,
                point_labels=labels,
            )
        except Exception as exc:  # noqa: BLE001
            LOGGER.error("Failed on %s: %s — skipping", video_path.name, exc)
            skipped += 1
            continue

        if mask is None:
            LOGGER.warning("No mask produced for %s — skipping", video_path.name)
            skipped += 1
            continue

        mask_dir = mkdir(cfg.paths.masks_dir / video_path.stem / "seeds")
        mask_path = mask_dir / f"seed_frame_{seed_idx:08d}_mask.png"
        cv2.imwrite(str(mask_path), mask * 255)

        seed_rows.append({
            "video_path": str(video_path),
            "seed_frame_idx": seed_idx,
            "arena_box_xyxy": list(arena_box),
            "positive_points": points[labels == 1].tolist(),
            "negative_points": points[labels == 0].tolist(),
            "seed_mask_path": str(mask_path),
            "fps": fps,
        })

    seed_manifest = cfg.paths.metadata_dir / "sam3_seed_manifest.jsonl"
    append_jsonl(seed_manifest, seed_rows)
    json_dump(
        cfg.paths.metadata_dir / "sam3_seed_summary.json",
        {"num_seeds": len(seed_rows), "manifest": str(seed_manifest)},
    )
    LOGGER.info("Done. Wrote %d seed masks (%d skipped).", len(seed_rows), skipped)


if __name__ == "__main__":
    main()
