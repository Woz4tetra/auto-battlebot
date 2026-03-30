#!/usr/bin/env python3
"""Quality filtering for propagated floor masks."""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

from common import (
    LOGGER,
    append_jsonl,
    configure_logging,
    json_dump,
    load_pipeline_config,
    mkdir,
)


def load_manifest_rows(path: Path) -> list[dict]:
    rows: list[dict] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            rows.append(json.loads(line))
    return rows


def cleanup_mask(mask_u8: np.ndarray, open_kernel: int, close_kernel: int) -> np.ndarray:
    open_k = np.ones((open_kernel, open_kernel), dtype=np.uint8)
    close_k = np.ones((close_kernel, close_kernel), dtype=np.uint8)
    cleaned = cv2.morphologyEx(mask_u8, cv2.MORPH_OPEN, open_k)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, close_k)
    return (cleaned > 0).astype(np.uint8)


def iou(a: np.ndarray, b: np.ndarray) -> float:
    inter = np.logical_and(a > 0, b > 0).sum()
    union = np.logical_or(a > 0, b > 0).sum()
    if union == 0:
        return 1.0
    return float(inter / union)


def main() -> None:
    parser = argparse.ArgumentParser(description="Apply quality gates to propagated masks")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).parent / "config" / "pipeline.toml",
    )
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    configure_logging(args.verbose)
    cfg = load_pipeline_config(args.config)
    manifest_in = cfg.paths.metadata_dir / "propagation_manifest.jsonl"
    if not manifest_in.exists():
        raise RuntimeError("No propagation manifest found. Run propagate_masks.py first.")

    mkdir(cfg.paths.filtered_masks_dir)
    rows = load_manifest_rows(manifest_in)
    by_video: dict[str, list[dict]] = defaultdict(list)
    for row in rows:
        by_video[row["video_path"]].append(row)

    passed_rows: list[dict] = []
    all_eval_rows: list[dict] = []
    for video_path, video_rows in by_video.items():
        prev_mask: np.ndarray | None = None
        prev_fraction: float | None = None
        for row in sorted(video_rows, key=lambda x: x["frame_idx"]):
            raw_mask = cv2.imread(row["mask_path"], cv2.IMREAD_GRAYSCALE)
            if raw_mask is None:
                continue
            cleaned = cleanup_mask(
                raw_mask,
                open_kernel=cfg.quality.opening_kernel,
                close_kernel=cfg.quality.closing_kernel,
            )
            fraction = float(cleaned.mean())
            ok = True
            reason = "pass"
            confidence = 1.0

            if fraction < cfg.quality.min_floor_fraction:
                ok = False
                reason = "below_min_floor_fraction"
                confidence = max(0.0, fraction / max(cfg.quality.min_floor_fraction, 1e-6))
            elif fraction > cfg.quality.max_floor_fraction:
                ok = False
                reason = "above_max_floor_fraction"
                confidence = max(
                    0.0,
                    cfg.quality.max_floor_fraction / max(fraction, 1e-6),
                )

            if ok and prev_mask is not None and prev_fraction is not None:
                area_delta = abs(fraction - prev_fraction)
                mask_iou = iou(cleaned, prev_mask)
                if area_delta > cfg.quality.max_area_delta:
                    ok = False
                    reason = "area_delta_spike"
                    confidence = max(0.0, 1.0 - (area_delta / max(cfg.quality.max_area_delta, 1e-6)))
                elif mask_iou < cfg.quality.min_iou_with_previous:
                    ok = False
                    reason = "low_temporal_iou"
                    confidence = max(
                        0.0,
                        mask_iou / max(cfg.quality.min_iou_with_previous, 1e-6),
                    )

            eval_row = {
                **row,
                "video_path": video_path,
                "floor_fraction": fraction,
                "confidence": confidence,
                "status": "pass" if ok else "fail",
                "reason": reason,
            }
            all_eval_rows.append(eval_row)

            if ok:
                out_dir = mkdir(cfg.paths.filtered_masks_dir / Path(video_path).stem)
                out_mask = out_dir / Path(row["mask_path"]).name
                cv2.imwrite(str(out_mask), cleaned.astype(np.uint8) * 255)
                pass_row = {**eval_row, "filtered_mask_path": str(out_mask)}
                passed_rows.append(pass_row)
                prev_mask = cleaned
                prev_fraction = fraction

    pass_manifest = cfg.paths.metadata_dir / "quality_pass_manifest.jsonl"
    eval_manifest = cfg.paths.metadata_dir / "quality_eval_manifest.jsonl"
    append_jsonl(pass_manifest, passed_rows)
    append_jsonl(eval_manifest, all_eval_rows)
    json_dump(
        cfg.paths.metadata_dir / "quality_summary.json",
        {
            "total_frames": len(all_eval_rows),
            "passed_frames": len(passed_rows),
            "pass_ratio": (len(passed_rows) / len(all_eval_rows)) if all_eval_rows else 0.0,
            "pass_manifest": str(pass_manifest),
            "eval_manifest": str(eval_manifest),
        },
    )
    LOGGER.info("Done. Kept %d/%d masks.", len(passed_rows), len(all_eval_rows))


if __name__ == "__main__":
    main()
