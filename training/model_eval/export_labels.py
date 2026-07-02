#!/usr/bin/env python3
"""Export YOLO datasets with model pre-labels from label_playback MCAP recordings.

Reads /camera/image, /blob_detections and /keypoint_detections from one or more MCAPs
produced by `./scripts/build_and_run.sh -c config/label_playback.toml`, pairs them by
frame stamp, and writes one dataset per model per recording. Images are written once at
the recording top level and symlinked into each subdataset:

    <output-dir>/<recording>/images/<stamp_ns>.png       (shared, real files)
    <output-dir>/<recording>/blob/images   -> ../images  (symlink)
    <output-dir>/<recording>/blob/labels/<stamp_ns>.txt  (class cx cy w h)
    <output-dir>/<recording>/blob/data.yaml
    <output-dir>/<recording>/keypoint/images -> ../images
    <output-dir>/<recording>/keypoint/labels/<stamp_ns>.txt  (class cx cy w h kx ky v ...)
    <output-dir>/<recording>/keypoint/data.yaml

Accepts multiple MCAP paths or globs and writes each under --output-dir automatically,
so `export_labels.py data/recordings/*.mcap` processes a whole batch. Both the blob
detector (detect rows) and keypoint model (pose rows) are exported by default; restrict
with --topics.

Correct the pre-labels with edit_labels.py, then score candidate runs with score.py.

Usage:
    python training/model_eval/export_labels.py data/recordings/*.mcap \
        [--output-dir <dir>] [--topics blob keypoint]
"""

from __future__ import annotations

import argparse
import glob
import shutil
from pathlib import Path

import cv2
import yaml
from tqdm import tqdm

from auto_battlebot.mcap_io import (
    BLOB_DETECTIONS_TOPIC,
    CAMERA_IMAGE_TOPIC,
    KEYPOINT_DETECTIONS_TOPIC,
    Detections,
    decode_compressed_image,
    decode_detections,
    decode_image_stamp_ns,
    iter_messages,
    match_stamps,
)

TOPICS = {"blob": BLOB_DETECTIONS_TOPIC, "keypoint": KEYPOINT_DETECTIONS_TOPIC}
# Anchored to this script (training/model_eval/), so it resolves to training/data/model_eval
# regardless of the working directory.
DEFAULT_OUTPUT_DIR = Path(__file__).parent.parent / "data" / "model_eval"

# Distinct editor colors, indexed by class id (wraps around).
DEFAULT_COLORS = [
    "#FF0000",
    "#00FF00",
    "#0000FF",
    "#FFFF00",
    "#FF00FF",
    "#00FFFF",
    "#FFA500",
    "#800080",
    "#008000",
    "#FFC0CB",
]


def load_recording(
    mcap_path: Path, topic_keys: list[str]
) -> tuple[dict[str, dict[int, Detections]], dict[int, bytes]]:
    """Read the requested detection topics and /camera/image in a single pass.

    Returns ({topic_key: {stamp_ns: Detections}}, {stamp_ns: raw image bytes})."""
    detections: dict[str, dict[int, Detections]] = {key: {} for key in topic_keys}
    images: dict[int, bytes] = {}
    topic_lookup = {TOPICS[key]: key for key in topic_keys}
    messages = tqdm(
        iter_messages(mcap_path, [*topic_lookup, CAMERA_IMAGE_TOPIC]),
        desc=f"  reading {mcap_path.name}",
        unit="msg",
        leave=False,
    )
    for topic, _log_time_ns, data in messages:
        key = topic_lookup.get(topic)
        if key is not None:
            dets = decode_detections(data)
            detections[key][dets.stamp_ns] = dets
        elif topic == CAMERA_IMAGE_TOPIC:
            # Key by the header (frame) stamp: log_time is wall clock in playback.
            # Defer image decoding until we know which frames have detections to pair with.
            images[decode_image_stamp_ns(data)] = data
    return detections, images


def class_names(detections: dict[int, Detections]) -> list[str]:
    """Class-id -> name list from the labels observed in the recording."""
    id_to_label: dict[int, str] = {}
    for dets in detections.values():
        for det in dets.detections:
            id_to_label.setdefault(det.class_id, det.label)
    if not id_to_label:
        return []
    count = max(id_to_label) + 1
    return [id_to_label.get(i, f"class_{i}") for i in range(count)]


def yolo_rows(dets: Detections) -> list[str]:
    """Convert one frame's detections to normalized YOLO rows (pose rows when keypoints exist)."""
    rows = []
    for det in dets.detections:
        cx = (det.x1 + det.x2) / 2.0 / dets.image_width
        cy = (det.y1 + det.y2) / 2.0 / dets.image_height
        w = (det.x2 - det.x1) / dets.image_width
        h = (det.y2 - det.y1) / dets.image_height
        row = f"{det.class_id} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}"
        for kp in det.keypoints:
            kx = kp.x / dets.image_width
            ky = kp.y / dets.image_height
            row += f" {kx:.6f} {ky:.6f} 2"
        rows.append(row)
    return rows


def write_image(image_data: bytes, image_width: int, out_path: Path) -> None:
    image = decode_compressed_image(image_data).image
    # ZED side-by-side exports are twice the pipeline width; keep the left eye.
    if image.shape[1] == 2 * image_width:
        image = image[:, :image_width]
    cv2.imwrite(str(out_path), image)


def link_images(link: Path) -> None:
    """Point <dataset>/images at the shared ../images via a relative symlink."""
    if link.is_symlink() or link.is_file():
        link.unlink()
    elif link.is_dir():
        shutil.rmtree(link)  # replace a real images/ dir left by an earlier layout
    link.symlink_to(Path("..") / "images", target_is_directory=True)


def recording_name(mcap_path: Path) -> str:
    """Auto output subdir for a recording: its filename stem without the config prefix."""
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


def write_subdataset(
    dataset_dir: Path,
    topic_key: str,
    frame_detections: dict[int, Detections],
    matches: dict[int, int],
    mcap_path: Path,
) -> list[str]:
    """Write one model's labels/ + data.yaml, symlinking images to the shared ../images."""
    labels_dir = dataset_dir / "labels"
    labels_dir.mkdir(parents=True, exist_ok=True)
    link_images(dataset_dir / "images")

    for det_stamp in matches:
        dets = frame_detections[det_stamp]
        (labels_dir / f"{det_stamp:019d}.txt").write_text("\n".join(yolo_rows(dets)) + "\n")

    names = class_names(frame_detections)
    data_yaml = {
        "names": names,
        "colors": [DEFAULT_COLORS[i % len(DEFAULT_COLORS)] for i in range(len(names))],
        "source_mcap": str(mcap_path),
        "source_topic": TOPICS[topic_key],
    }
    (dataset_dir / "data.yaml").write_text(yaml.safe_dump(data_yaml, sort_keys=False))
    return names


def export_recording(mcap_path: Path, output_dir: Path, topic_keys: list[str]) -> list[Path]:
    """Export a recording's shared images + one subdataset per model with detections."""
    detections, images = load_recording(mcap_path, topic_keys)
    if not images:
        raise SystemExit(
            f"No /camera/image messages in {mcap_path}. Record with config/label_playback.toml "
            "(it keeps images); experiment/eval profiles exclude them."
        )
    present = [key for key in topic_keys if detections[key]]
    if not present:
        wanted = ", ".join(TOPICS[key] for key in topic_keys)
        raise SystemExit(f"No {wanted} messages in {mcap_path}.")

    recording_dir = output_dir / recording_name(mcap_path)
    images_dir = recording_dir / "images"
    images_dir.mkdir(parents=True, exist_ok=True)

    image_stamps = sorted(images)
    matches = {key: match_stamps(sorted(detections[key]), image_stamps) for key in present}

    # Shared images: the union of frames any present model labels, written once.
    needed: dict[int, int] = {}
    for key in present:
        needed.update(matches[key])
    for det_stamp, image_stamp in tqdm(
        needed.items(), desc="  writing images", unit="frame", leave=False
    ):
        width = next(
            detections[k][det_stamp].image_width for k in present if det_stamp in matches[k]
        )
        write_image(images[image_stamp], width, images_dir / f"{det_stamp:019d}.png")

    datasets = []
    for key in present:
        dataset_dir = recording_dir / key
        names = write_subdataset(dataset_dir, key, detections[key], matches[key], mcap_path)
        print(f"  {key}: {len(matches[key])} labels, classes {names}")
        datasets.append(dataset_dir)

    print(f"Exported {recording_dir} ({len(needed)} shared images)")
    return datasets


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument(
        "mcaps", nargs="+", help="one or more MCAP files or globs from label_playback runs"
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=f"base output directory (default: {DEFAULT_OUTPUT_DIR})",
    )
    parser.add_argument(
        "--topics",
        nargs="+",
        choices=sorted(TOPICS),
        default=sorted(TOPICS),
        help="which models to export (default: blob keypoint)",
    )
    args = parser.parse_args()

    mcaps = resolve_mcaps(args.mcaps)
    if not mcaps:
        raise SystemExit("No MCAP files to process.")

    datasets: list[Path] = []
    for mcap_path in tqdm(mcaps, desc="recordings", unit="mcap", disable=len(mcaps) == 1):
        datasets.extend(export_recording(mcap_path, args.output_dir, args.topics))

    print(f"\nExported {len(datasets)} dataset(s) under {args.output_dir}")
    print(f"Next: python training/model_eval/edit_labels.py {datasets[0]}")


if __name__ == "__main__":
    main()
