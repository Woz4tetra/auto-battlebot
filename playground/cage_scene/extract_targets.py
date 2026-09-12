#!/usr/bin/env python3
"""Robot-free, rectified target frames from the fixed cage-high clips.

One target per clip: the per-pixel median over frames spread across the fight (robots move,
so each pixel shows bare floor in most samples), remapped through the same rectification the
C++ camera path applies, so a pinhole render at the rectified K lines up with it. The clip's
cached hull polygon is undistorted the same way and rasterised beside it.

Output layout:

    <out>/camera_rect.json                 rectified K for every later stage
    <out>/targets/<clip>/target.png        1920x1080 rectified median frame
    <out>/targets/<clip>/hull_mask.png     255 inside the mat hull
    <out>/targets/<clip>/meta.json         event, cage, game, source url, sample count
    <out>/targets/<clip>/target_4k.png     with --fetch-4k: rectified median of N remote 4K frames

Usage:
    venv/bin/python playground/cage_scene/extract_targets.py \\
        data/downloads/brettzone_cage_high --cage 2 --weight-class 3lb --out runs/cage_scene
    venv/bin/python playground/cage_scene/extract_targets.py \\
        data/downloads/mrsbuff_may26 --cage 2 --out runs/cage_scene
"""

from __future__ import annotations

import argparse
import json
import subprocess
import tempfile
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from auto_battlebot.perception.camera_calibration import (
    CameraCalibration,
    load_camera_calibration,
    rectify_maps,
    undistort_points,
)
from auto_battlebot.segmentation.field_hull import hull_mask_from_polygon, load_hull, median_frame

DEFAULT_CALIBRATION = Path("config/cameras/brettzone_cage_high.toml")
SOURCE_SIZE = (3840, 2160)


def load_manifest(clip_dir: Path) -> dict[str, dict[str, Any]]:
    entries = json.loads((clip_dir / "manifest.json").read_text())
    return {entry["filename"]: entry for entry in entries}


def fetch_4k_frames(entry: dict[str, Any], count: int, workdir: Path) -> list[np.ndarray]:
    """Remote-seek `count` frames spread over the fight from the 4K source object."""
    start = float(entry["clip_start_s"]) + 10.0
    end = float(entry["clip_start_s"]) + float(entry["clip_duration_s"]) - 10.0
    frames = []
    for i, stamp in enumerate(np.linspace(start, max(start, end), count)):
        out = workdir / f"frame_{i:03d}.png"
        command = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-nostdin",
            "-ss",
            f"{stamp:.3f}",
            "-i",
            entry["source_url"],
            "-frames:v",
            "1",
            "-y",
            str(out),
        ]
        result = subprocess.run(command, capture_output=True, text=True, check=False)
        if result.returncode != 0 or not out.exists():
            print(f"    4K fetch at {stamp:.1f}s failed: {result.stderr.strip()[:200]}")
            continue
        frame = cv2.imread(str(out))
        if frame is not None and frame.shape[1] == SOURCE_SIZE[0]:
            frames.append(frame)
    return frames


def rectify(image: np.ndarray, calibration: CameraCalibration) -> tuple[np.ndarray, np.ndarray]:
    height, width = image.shape[:2]
    map_x, map_y, k_rect = rectify_maps(calibration, (width, height))
    return cv2.remap(image, map_x, map_y, cv2.INTER_LINEAR), k_rect


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "clip_dir", type=Path, help="directory with BZ-*.mp4, *.hull.json, manifest.json"
    )
    parser.add_argument("--out", type=Path, default=Path("runs/cage_scene"))
    parser.add_argument("--cage", default="2", help="cage number to keep; 'all' for every cage")
    parser.add_argument(
        "--weight-class",
        default="3lb",
        help="tournament_id suffix prefix to keep (3lb also keeps 3lbxp); 'all' for every class",
    )
    parser.add_argument("--camera-suffix", default="Overhead-High")
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--samples", type=int, default=60, help="frames in the median")
    parser.add_argument(
        "--fetch-4k", type=int, default=0, metavar="N", help="also median N remote 4K frames"
    )
    parser.add_argument("--force", action="store_true", help="recompute targets that already exist")
    args = parser.parse_args()

    calibration = load_camera_calibration(args.calibration)
    cage = None if args.cage == "all" else args.cage
    clips = []
    manifest = load_manifest(args.clip_dir)
    for video in sorted(args.clip_dir.glob("*.mp4")):
        entry = manifest.get(video.name)
        if entry is None or not str(entry.get("camera", "")).endswith(args.camera_suffix):
            continue
        if cage is not None and str(entry.get("cage")) != cage:
            continue
        weight = str(entry.get("tournament_id", "")).rsplit("_", 1)[-1]
        if args.weight_class != "all" and not weight.startswith(args.weight_class):
            continue
        if not video.with_suffix(".hull.json").exists():
            print(f"skip {video.name}: no .hull.json beside it")
            continue
        clips.append((video, entry))
    if not clips:
        raise SystemExit(f"no {args.weight_class} clips for cage {args.cage} under {args.clip_dir}")

    map_x, map_y, k_rect = rectify_maps(calibration, (calibration.width, calibration.height))
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "camera_rect.json").write_text(
        json.dumps(
            {
                "calibration_id": calibration.calibration_id,
                "source_calibration": str(args.calibration),
                "width": calibration.width,
                "height": calibration.height,
                "fx": float(k_rect[0, 0]),
                "fy": float(k_rect[1, 1]),
                "cx": float(k_rect[0, 2]),
                "cy": float(k_rect[1, 2]),
                "alpha": 1.0,
            },
            indent=2,
        )
        + "\n"
    )

    for video, entry in clips:
        target_dir = args.out / "targets" / video.stem
        target_dir.mkdir(parents=True, exist_ok=True)
        target_path = target_dir / "target.png"
        if target_path.exists() and not args.force:
            print(f"keep  {video.stem}")
        else:
            print(f"median {video.stem} ({args.samples} samples)")
            background = median_frame(video, args.samples)
            if background.shape[1] != calibration.width:
                raise SystemExit(
                    f"{video.name} is {background.shape[1]} px wide, "
                    f"calibration is {calibration.width}"
                )
            rectified = cv2.remap(background, map_x, map_y, cv2.INTER_LINEAR)
            cv2.imwrite(str(target_path), rectified)

        hull = load_hull(video.with_suffix(".hull.json"))
        polygon = np.array(hull["polygon"], dtype=np.float64)
        polygon_rect = undistort_points(polygon, calibration, k_rect)
        mask = hull_mask_from_polygon(
            np.round(polygon_rect).astype(int).tolist(), (calibration.height, calibration.width)
        )
        cv2.imwrite(str(target_dir / "hull_mask.png"), mask)

        meta = {
            "clip": video.stem,
            "video": str(video),
            "event": entry.get("tournament_id"),
            # Weight classes share a weekend and a camera mount, so they share an event group.
            "event_group": str(entry.get("tournament_id", "")).rsplit("_", 1)[0],
            "game_id": entry.get("game_id"),
            "cage": entry.get("cage"),
            "camera": entry.get("camera"),
            "source_url": entry.get("source_url"),
            "clip_start_s": entry.get("clip_start_s"),
            "clip_duration_s": entry.get("clip_duration_s"),
            "samples": args.samples,
            "hull_fraction": hull.get("hull_fraction"),
            "hull_polygon_rectified": np.round(polygon_rect, 2).tolist(),
        }

        if args.fetch_4k > 0 and (args.force or not (target_dir / "target_4k.png").exists()):
            with tempfile.TemporaryDirectory() as tmp:
                frames = fetch_4k_frames(entry, args.fetch_4k, Path(tmp))
            if frames:
                median_4k = np.median(np.stack(frames), axis=0).astype(np.uint8)
                rectified_4k, k_rect_4k = rectify(median_4k, calibration)
                cv2.imwrite(str(target_dir / "target_4k.png"), rectified_4k)
                meta["frames_4k"] = len(frames)
                meta["k_rect_4k"] = k_rect_4k.tolist()
                print(f"    4K median from {len(frames)} frames")
            else:
                print("    no 4K frames fetched")

        (target_dir / "meta.json").write_text(json.dumps(meta, indent=2) + "\n")

    print(f"{len(clips)} targets under {args.out / 'targets'}")
    print(f"rectified K in {args.out / 'camera_rect.json'}")


if __name__ == "__main__":
    main()
