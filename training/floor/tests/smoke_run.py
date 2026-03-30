#!/usr/bin/env python3
"""Small smoke run for quality + export stages (SAM3 not required)."""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

TEST_DIR = Path(__file__).resolve().parent
FLOOR_DIR = TEST_DIR.parent
CONFIG_DIR = FLOOR_DIR / "config"


def write_synthetic_video(video_path: Path, frames: int = 60, width: int = 320, height: int = 180) -> None:
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(video_path), fourcc, 30.0, (width, height))
    for i in range(frames):
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        frame[:, :] = (80, 80, 90)
        cv2.putText(frame, f"f{i}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        writer.write(frame)
    writer.release()


def main() -> None:
    root = FLOOR_DIR / "data" / "smoke"
    if root.exists():
        shutil.rmtree(root)
    (root / "processed/videos").mkdir(parents=True, exist_ok=True)
    (root / "masks/raw").mkdir(parents=True, exist_ok=True)
    (root / "metadata").mkdir(parents=True, exist_ok=True)

    video_path = root / "processed/videos" / "smoke_video.mp4"
    write_synthetic_video(video_path)

    mask_dir = root / "masks/raw" / "smoke_video" / "propagated"
    mask_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for idx in range(60):
        mask = np.zeros((180, 320), dtype=np.uint8)
        mask[20:160, 20:300] = 255
        if idx % 17 == 0:
            mask[0:10, 0:10] = 255  # noise
        mask_path = mask_dir / f"frame_{idx:08d}_mask.png"
        cv2.imwrite(str(mask_path), mask)
        rows.append(
            {
                "video_path": str(video_path),
                "chunk_index": 0,
                "frame_idx": idx,
                "mask_path": str(mask_path),
            }
        )

    manifest = root / "metadata" / "propagation_manifest.jsonl"
    with manifest.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(row) + "\n")

    config_path = root / "smoke_pipeline.toml"
    config_text = (CONFIG_DIR / "pipeline.toml").read_text(encoding="utf-8")
    config_text = config_text.replace("training/floor/data", str(root).replace("\\", "/"))
    config_path.write_text(config_text, encoding="utf-8")

    subprocess.run(
        [sys.executable, str(FLOOR_DIR / "quality_filter.py"), "--config", str(config_path)],
        check=True,
    )
    subprocess.run(
        [sys.executable, str(FLOOR_DIR / "export_dataset.py"), "--config", str(config_path), "--stride", "10"],
        check=True,
    )

    summary_path = root / "metadata" / "export_summary.json"
    if not summary_path.exists():
        raise RuntimeError("Smoke run failed: missing export summary")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    if summary.get("exported_frames", 0) <= 0:
        raise RuntimeError("Smoke run failed: no exported frames")
    print("Smoke run succeeded:", summary)


if __name__ == "__main__":
    main()
