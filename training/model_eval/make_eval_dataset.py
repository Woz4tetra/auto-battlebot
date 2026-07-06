#!/usr/bin/env python3
"""Build a hand-labeling eval dataset from /camera/image frames in MCAP recordings.

Unlike export_labels.py (which needs detection topics to write model pre-labels), this
samples raw frames from recordings that carry no detections, writing empty label files
for labeling from scratch in edit_labels.py. One subdataset per recording:

    <output-dir>/<recording>/images/<stamp_ns>.png   (left-eye frame)
    <output-dir>/<recording>/labels/<stamp_ns>.txt    (empty, ready to label)
    <output-dir>/<recording>/data.yaml                (names/colors/kpt_shape/flip_idx)

Frames are named by the /camera/image header stamp_ns, which is the SVO frame stamp. That
is how score.py aligns this ground truth to a later candidate playback run over the same
SVO, so the dataset is scorable without any AVI/frame-index bookkeeping.

Frames before each video's svo_start_frame (from the playback config) are skipped, then a
fixed number are sampled evenly across the rest (decorrelated, not consecutive). The MCAP
/camera/image stream is the SVO frames in order, so svo_start_frame applies as an index
into that stream. Class names/colors and the keypoint schema are seeded from a source
dataset (default: our_robot_keypoints) and can be extended with --extra-classes for robots
absent from the training set (opponents, house).

Usage:
    python training/model_eval/make_eval_dataset.py data/recordings/*.mcap \
        [--output-dir <dir>] [--per-video 100] \
        [--playback-config config/playback/_playback.toml] \
        [--extra-classes opponent house_bot]
"""

from __future__ import annotations

import argparse
import glob
import re
from pathlib import Path

import cv2
import numpy as np
import yaml
from mcap.reader import make_reader
from tqdm import tqdm

from auto_battlebot.mcap_io import (
    CAMERA_IMAGE_TOPIC,
    decode_compressed_image,
    iter_messages,
)

_HERE = Path(__file__).parent
_REPO = _HERE.parent.parent
DEFAULT_OUTPUT_DIR = _HERE.parent / "data" / "nhrl_keypoints_eval_test"
DEFAULT_CLASS_SOURCE = _HERE.parent / "data" / "our_robot_keypoints" / "data.yml"
DEFAULT_PLAYBACK_CONFIG = _REPO / "config" / "playback" / "_playback.toml"
# Aspect ratio above which a frame is treated as a ZED side-by-side pair (keep left eye).
# 16:9 single eye is 1.78; a side-by-side pair is 3.56.
SIDE_BY_SIDE_ASPECT = 2.6
# Distinct editor colors for classes added via --extra-classes, in order.
EXTRA_COLORS = ["#22c55e", "#f59e0b", "#a855f7", "#14b8a6", "#f43f5e"]


def recording_name(mcap_path: Path) -> str:
    """Output subdir for a recording: its stem without the auto_battlebot_ prefix."""
    stem = mcap_path.stem
    prefix = "auto_battlebot_"
    return stem[len(prefix) :] if stem.startswith(prefix) else stem


def resolve_mcaps(patterns: list[str]) -> list[Path]:
    """Expand paths and globs into a sorted, de-duplicated list of existing MCAP files."""
    resolved: dict[Path, None] = {}
    for pattern in patterns:
        matches = glob.glob(pattern) if any(c in pattern for c in "*?[") else [pattern]
        if not matches:
            print(f"Warning: no files matched {pattern!r}")
        for match in matches:
            path = Path(match)
            if path.suffix != ".mcap":
                print(f"Warning: skipping non-mcap {path}")
                continue
            if not path.is_file():
                print(f"Warning: skipping missing file {path}")
                continue
            resolved[path.resolve()] = None
    return sorted(resolved)


def image_frame_count(mcap_path: Path) -> int:
    """Number of /camera/image messages, read cheaply from the MCAP summary."""
    with open(mcap_path, "rb") as handle:
        summary = make_reader(handle).get_summary()
    if summary is None or summary.statistics is None:
        return 0
    topic_to_id = {ch.topic: cid for cid, ch in summary.channels.items()}
    channel_id = topic_to_id.get(CAMERA_IMAGE_TOPIC)
    if channel_id is None:
        return 0
    return summary.statistics.channel_message_counts.get(channel_id, 0)


def parse_start_frames(config_path: Path) -> dict[str, int]:
    """Map SVO basename (no dir, no .svo2) -> svo_start_frame from the playback config.

    Entries are commented out in the config; a svo_start_frame binds to the svo_file_path
    line above it. Videos with no svo_start_frame default to 0."""
    starts: dict[str, int] = {}
    current: str | None = None
    if not config_path.exists():
        print(f"Warning: playback config {config_path} not found; using start frame 0 for all")
        return starts
    for raw in config_path.read_text().splitlines():
        line = raw.lstrip("#").strip()
        path_match = re.match(r'svo_file_path\s*=\s*"([^"]+)"', line)
        if path_match:
            current = Path(path_match.group(1)).stem
            starts.setdefault(current, 0)
            continue
        frame_match = re.match(r"svo_start_frame\s*=\s*(\d+)", line)
        if frame_match and current is not None:
            starts[current] = int(frame_match.group(1))
    return starts


def svo_key(mcap_path: Path) -> str:
    """SVO basename a recording maps to: the segment after the last '__' in its stem."""
    return mcap_path.stem.split("__")[-1]


def selected_indices(start: int, count: int, n_pick: int) -> list[int]:
    """`n_pick` evenly spaced frame indices across [start, count), both ends inclusive."""
    start = max(0, min(start, count))
    available = count - start
    if n_pick <= 0 or available <= 0:
        return []
    if n_pick >= available:
        return list(range(start, count))
    return sorted({int(round(i)) for i in np.linspace(start, count - 1, n_pick)})


def left_eye(image: np.ndarray) -> np.ndarray:
    """Keep the left half when the frame is a ZED side-by-side pair; else pass through."""
    height, width = image.shape[:2]
    if width > height * SIDE_BY_SIDE_ASPECT:
        return image[:, : width // 2]
    return image


def load_class_schema(source: Path, extra_classes: list[str]) -> dict:
    """Names/colors/kpt schema from the source dataset, extended with extra classes."""
    data = yaml.safe_load(source.read_text()) if source.exists() else {}
    data = data or {}
    names = list(data.get("names", []))
    colors = list(data.get("colors", []))
    for offset, name in enumerate(extra_classes):
        if name in names:
            continue
        names.append(name)
        colors.append(EXTRA_COLORS[offset % len(EXTRA_COLORS)])
    schema = {
        "names": names,
        "colors": colors,
        "nc": len(names),
    }
    if "kpt_shape" in data:
        schema["kpt_shape"] = data["kpt_shape"]
    if "flip_idx" in data:
        schema["flip_idx"] = data["flip_idx"]
    return schema


def write_recording(
    mcap_path: Path, output_dir: Path, n_pick: int, start_frame: int, schema: dict
) -> int:
    """Sample n_pick frames after start_frame into an edit_labels-ready subdataset."""
    count = image_frame_count(mcap_path)
    keep = set(selected_indices(start_frame, count, n_pick))
    if not keep:
        print(f"  {mcap_path.name}: no frames to sample")
        return 0
    if len(keep) < n_pick:
        print(f"  {recording_name(mcap_path)}: only {len(keep)} frames after start {start_frame}")

    dataset_dir = output_dir / recording_name(mcap_path)
    images_dir = dataset_dir / "images"
    labels_dir = dataset_dir / "labels"
    # Clear any prior extraction so re-runs don't mix old and new samples.
    for stale in (images_dir, labels_dir):
        if stale.is_dir():
            for old in stale.iterdir():
                old.unlink()
    images_dir.mkdir(parents=True, exist_ok=True)
    labels_dir.mkdir(parents=True, exist_ok=True)

    written = 0
    frames = enumerate(iter_messages(mcap_path, [CAMERA_IMAGE_TOPIC]))
    for index, (_topic, _log_time_ns, data) in tqdm(
        frames, total=count, desc=f"  {recording_name(mcap_path)}", unit="frame", leave=False
    ):
        if index not in keep:
            continue
        frame = decode_compressed_image(data)
        stem = f"{frame.stamp_ns:019d}"
        cv2.imwrite(str(images_dir / f"{stem}.png"), left_eye(frame.image))
        (labels_dir / f"{stem}.txt").write_text("")  # empty: label from scratch
        written += 1

    (dataset_dir / "data.yaml").write_text(
        yaml.safe_dump(
            {**schema, "source_mcap": str(mcap_path), "svo_start_frame": start_frame},
            sort_keys=False,
        )
    )
    print(f"  {recording_name(mcap_path)}: {written} frames (of {count}, from frame {start_frame})")
    return written


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("mcaps", nargs="+", help="one or more MCAP files or globs")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=f"base output directory (default: {DEFAULT_OUTPUT_DIR})",
    )
    parser.add_argument(
        "--per-video", type=int, default=100, help="frames to sample from each recording"
    )
    parser.add_argument(
        "--playback-config",
        type=Path,
        default=DEFAULT_PLAYBACK_CONFIG,
        help=f"config with svo_start_frame per video (default: {DEFAULT_PLAYBACK_CONFIG})",
    )
    parser.add_argument(
        "--class-source",
        type=Path,
        default=DEFAULT_CLASS_SOURCE,
        help=f"dataset data.yaml to seed names/colors/kpt schema (default: {DEFAULT_CLASS_SOURCE})",
    )
    parser.add_argument(
        "--extra-classes",
        nargs="*",
        default=["opponent", "house_bot"],
        help="classes to append beyond the source (default: opponent house_bot)",
    )
    args = parser.parse_args()

    mcaps = resolve_mcaps(args.mcaps)
    if not mcaps:
        raise SystemExit("No MCAP files to process.")

    schema = load_class_schema(args.class_source, args.extra_classes)
    print(f"Classes: {schema['names']}")

    start_frames = parse_start_frames(args.playback_config)

    total = 0
    for mcap_path in mcaps:
        key = svo_key(mcap_path)
        start = start_frames.get(key, 0)
        if key not in start_frames:
            print(f"  {recording_name(mcap_path)}: no start frame for {key!r}; using 0")
        total += write_recording(mcap_path, args.output_dir, args.per_video, start, schema)

    print(f"\nWrote {total} frames across {len(mcaps)} subdataset(s) under {args.output_dir}")
    first = args.output_dir / recording_name(mcaps[0])
    print(f"Next: python training/model_eval/edit_labels.py {first}")


if __name__ == "__main__":
    main()
