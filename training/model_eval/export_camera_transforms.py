#!/usr/bin/env python3
"""Export camera intrinsics and per-image camera->field transforms for the eval dataset.

Reads each sub dataset's `data.yaml` for its `source_mcap`, then writes alongside the
existing `images/` and `labels/` (neither is modified):

    <sub-dataset>/camera_info.json                  intrinsics, constant over the recording
    <sub-dataset>/camera_transforms/<stamp_ns>.json  one per image

The transform is `tf_field_from_camera`: multiply a point in the camera frame by it to get
that point in the field frame. It is the composition of two recorded transforms:

    tf_field_from_camera = tf_field_from_cameraworld @ tf_cameraworld_from_camera

Both edges come from `/tf`, published once per frame the perception loop actually processed.
`field -> camera_world` only changes on a field re-init but is restamped every cycle, so the
applicable one is the latest at or before the image. Recordings made before that edge moved
carry it once on `/tf_static`; those are read the same way.

Frame association
-----------------
Dataset images are named by `/camera/image` header stamp, which the SVO recorder wrote.
`/tf` and `/camera/camera_info` carry the perception pipeline's own stamps, and the two
differ: a pipeline stamp lands roughly 0.4 to 0.9 of the way through the SVO frame interval
it belongs to, a stable sub-frame capture lag rather than clock skew. It never crosses an
interval boundary, so "the pipeline stamp inside [svo_stamp, next_svo_stamp)" identifies the
frame exactly. Each image and each `/tf` sample is resolved to a position in the
`/camera/image` stream that way, and the two are matched on that position. It is a position
within this mcap, not a .svo2 frame index; ZED_SVO_Editor drops a few frames per file, so the
two differ by a small offset. That offset is the same for both sides here, so it cancels.

The pipeline ran slower than the camera (~25 fps against 30 fps), so it skipped frames. For a
skipped frame no live transform was ever computed and the result is interpolated from the
neighbouring processed frames. Every image gets a file; the `method` field says which case it
was, so a consumer can filter on it:

    exact         one processed frame owns this image's frame index
    interpolated  frame was skipped; slerp/lerp between the nearest processed frames
    ambiguous     two processed frames claim this index (only the 05-01 recording)
    unavailable   no field frame yet, or no processed frame to interpolate from

Usage:
    python training/model_eval/export_camera_transforms.py [--dataset-dir <dir>] [--dry-run]
"""

from __future__ import annotations

import argparse
import json
from bisect import bisect_right
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from mcap.reader import make_reader
from mcap_ros1.decoder import DecoderFactory
from scipy.spatial.transform import Rotation, Slerp

_HERE = Path(__file__).parent
DEFAULT_DATASET_DIR = _HERE.parent / "data" / "nhrl_keypoints_eval_test"

IMAGE_TOPIC = "/camera/image"
CAMERA_INFO_TOPIC = "/camera/camera_info"
TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"
FIELD_MARKERS_TOPIC = "/field_markers"

FIELD_FRAME = "field"
CAMERA_WORLD_FRAME = "camera_world"

# Interpolating across a long dropout says more about the gap than the frame. Beyond this many
# camera frames between the bracketing processed frames, report unavailable instead.
MAX_INTERP_GAP_FRAMES = 8


def transform_matrix(translation: Any, rotation: Any) -> np.ndarray:
    """Build a 4x4 homogeneous matrix from a ROS translation and xyzw quaternion."""
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(
        [rotation.x, rotation.y, rotation.z, rotation.w]
    ).as_matrix()
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


def decompose(matrix: np.ndarray) -> dict[str, Any]:
    """Split a 4x4 matrix into the JSON form: matrix, translation, xyzw quaternion."""
    quat = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    return {
        "matrix": [[float(v) for v in row] for row in matrix],
        "translation": {
            "x": float(matrix[0, 3]),
            "y": float(matrix[1, 3]),
            "z": float(matrix[2, 3]),
        },
        "rotation": {
            "x": float(quat[0]),
            "y": float(quat[1]),
            "z": float(quat[2]),
            "w": float(quat[3]),
        },
    }


def read_recording(mcap_path: Path) -> dict[str, Any]:
    """Pull image stamps, intrinsics, and both transform streams out of one recording."""
    image_stamps: list[int] = []
    camera_infos: list[dict[str, Any]] = []
    dynamic: list[tuple[int, np.ndarray]] = []
    field_from_cameraworld: list[tuple[int, np.ndarray]] = []
    field_sizes: list[tuple[int, float, float]] = []

    with open(mcap_path, "rb") as handle:
        reader = make_reader(handle, decoder_factories=[DecoderFactory()])
        topics = [
            IMAGE_TOPIC,
            CAMERA_INFO_TOPIC,
            TF_TOPIC,
            TF_STATIC_TOPIC,
            FIELD_MARKERS_TOPIC,
        ]
        for _schema, channel, _message, decoded in reader.iter_decoded_messages(topics=topics):
            if channel.topic == IMAGE_TOPIC:
                image_stamps.append(decoded.header.stamp.to_nsec())
            elif channel.topic == CAMERA_INFO_TOPIC:
                camera_infos.append(
                    {
                        "width": int(decoded.width),
                        "height": int(decoded.height),
                        "distortion_model": decoded.distortion_model,
                        "D": [float(v) for v in decoded.D],
                        "K": [float(v) for v in decoded.K],
                        "R": [float(v) for v in decoded.R],
                        "P": [float(v) for v in decoded.P],
                    }
                )
            elif channel.topic == FIELD_MARKERS_TOPIC:
                for marker in decoded.markers:
                    if marker.ns != "field" or len(marker.points) < 4:
                        continue
                    # The border is a closed LINE_STRIP whose first four points are the
                    # corners. Opposite edges give the fitted extents.
                    corners = np.array([[p.x, p.y, p.z] for p in marker.points[:4]])
                    edges = [
                        float(np.linalg.norm(corners[(i + 1) % 4] - corners[i])) for i in range(4)
                    ]
                    field_sizes.append(
                        (
                            marker.header.stamp.to_nsec(),
                            (edges[0] + edges[2]) / 2.0,
                            (edges[1] + edges[3]) / 2.0,
                        )
                    )
            else:
                for tf in decoded.transforms:
                    stamp = tf.header.stamp.to_nsec()
                    matrix = transform_matrix(tf.transform.translation, tf.transform.rotation)
                    if tf.header.frame_id == FIELD_FRAME:
                        # Recordings made before the edge moved carry it once on /tf_static;
                        # newer ones repeat it on /tf every cycle. Both land here.
                        field_from_cameraworld.append((stamp, matrix))
                    elif channel.topic == TF_TOPIC and tf.header.frame_id == CAMERA_WORLD_FRAME:
                        dynamic.append((stamp, matrix))

    image_stamps.sort()
    dynamic.sort(key=lambda item: item[0])
    field_from_cameraworld.sort(key=lambda item: item[0])
    field_sizes.sort(key=lambda item: item[0])
    return {
        "image_stamps": np.array(image_stamps, dtype=np.int64),
        "camera_infos": camera_infos,
        "dynamic": dynamic,
        "field_from_cameraworld": field_from_cameraworld,
        "field_sizes": field_sizes,
    }


def frame_indices(image_stamps: np.ndarray, stamps: np.ndarray) -> np.ndarray:
    """Resolve pipeline stamps to the SVO frame interval that contains each one."""
    return np.searchsorted(image_stamps, stamps, side="right") - 1


def summarize_intrinsics(camera_infos: list[dict[str, Any]]) -> dict[str, Any]:
    """Collapse the camera_info stream to one entry, recording any variation seen."""
    first = dict(camera_infos[0])
    varying = sorted(
        key
        for key in ("width", "height", "distortion_model", "D", "K", "R", "P")
        if any(info[key] != first[key] for info in camera_infos)
    )
    first["message_count"] = len(camera_infos)
    first["varying_fields"] = varying
    return first


def interpolate(
    lower: tuple[int, np.ndarray], upper: tuple[int, np.ndarray], position: float
) -> np.ndarray:
    """Blend two poses: lerp the translation, slerp the rotation."""
    low_matrix, high_matrix = lower[1], upper[1]
    rotations = Rotation.from_matrix(np.stack([low_matrix[:3, :3], high_matrix[:3, :3]]))
    matrix = np.eye(4)
    matrix[:3, :3] = Slerp([0.0, 1.0], rotations)([position])[0].as_matrix()
    matrix[:3, 3] = (1.0 - position) * low_matrix[:3, 3] + position * high_matrix[:3, 3]
    return matrix


def resolve_dynamic(
    frame_index: int,
    dynamic: list[tuple[int, np.ndarray]],
    dynamic_frames: np.ndarray,
) -> tuple[str, np.ndarray | None, dict[str, Any]]:
    """Find or interpolate camera_world -> camera for one SVO frame index."""
    left = int(np.searchsorted(dynamic_frames, frame_index, side="left"))
    right = int(np.searchsorted(dynamic_frames, frame_index, side="right"))
    matches = right - left

    if matches == 1:
        return "exact", dynamic[left][1], {"tf_stamp_ns": int(dynamic[left][0])}
    if matches > 1:
        # Two processed frames claim this interval. Take the first and say so.
        return (
            "ambiguous",
            dynamic[left][1],
            {
                "tf_stamp_ns": int(dynamic[left][0]),
                "candidate_count": matches,
                "candidate_stamps_ns": [int(dynamic[i][0]) for i in range(left, right)],
            },
        )

    if left == 0 or left >= len(dynamic):
        return "unavailable", None, {"reason": "no bracketing processed frame"}

    low_index, high_index = int(dynamic_frames[left - 1]), int(dynamic_frames[left])
    gap = high_index - low_index
    if gap > MAX_INTERP_GAP_FRAMES:
        return (
            "unavailable",
            None,
            {"reason": "gap between processed frames too large", "gap_frames": gap},
        )

    position = (frame_index - low_index) / gap
    matrix = interpolate(dynamic[left - 1], dynamic[left], position)
    return (
        "interpolated",
        matrix,
        {
            "gap_frames": gap,
            "position": round(float(position), 6),
            "bracket_frame_indices": [low_index, high_index],
            "bracket_stamps_ns": [int(dynamic[left - 1][0]), int(dynamic[left][0])],
        },
    )


def export_sub_dataset(sub_dir: Path, dry_run: bool) -> dict[str, int]:
    """Write camera_info.json and one transform file per image for one sub dataset."""
    config = yaml.safe_load((sub_dir / "data.yaml").read_text())
    mcap_path = Path(config["source_mcap"])
    recording = read_recording(mcap_path)

    image_stamps = recording["image_stamps"]
    dynamic = recording["dynamic"]
    field_from_cameraworld = recording["field_from_cameraworld"]
    dynamic_frames = frame_indices(image_stamps, np.array([s for s, _ in dynamic], dtype=np.int64))
    field_tf_stamps = [stamp for stamp, _ in field_from_cameraworld]
    # /field_markers is published once per field init, but the field -> camera_world edge now
    # repeats every cycle, so the two no longer share a stamp. Take the latest markers at or
    # before the image instead of keying the size off an exact stamp match.
    field_sizes = recording["field_sizes"]
    field_size_stamps = [stamp for stamp, _, _ in field_sizes]

    intrinsics = summarize_intrinsics(recording["camera_infos"])
    intrinsics.update(
        {
            "frame_id": "camera",
            "source_mcap": str(mcap_path),
            "source_topic": CAMERA_INFO_TOPIC,
            "note": (
                "Constant over the recording unless varying_fields is non-empty. Applies to "
                "every image in images/."
            ),
        }
    )

    transforms_dir = sub_dir / "camera_transforms"
    if not dry_run:
        transforms_dir.mkdir(exist_ok=True)
        (sub_dir / "camera_info.json").write_text(json.dumps(intrinsics, indent=2) + "\n")

    counts: dict[str, int] = {}
    for image_path in sorted((sub_dir / "images").glob("*.png")):
        stamp = int(image_path.stem)
        position = int(np.searchsorted(image_stamps, stamp))
        if position >= len(image_stamps) or int(image_stamps[position]) != stamp:
            raise RuntimeError(f"{image_path.name} has no matching {IMAGE_TOPIC} message")

        method, cameraworld_from_camera, provenance = resolve_dynamic(
            position, dynamic, dynamic_frames
        )

        field_tf_index = bisect_right(field_tf_stamps, stamp) - 1
        if field_tf_index < 0:
            method, cameraworld_from_camera = "unavailable", None
            provenance = {"reason": "image precedes the first field initialization"}

        record: dict[str, Any] = {
            "image": image_path.name,
            "stamp_ns": stamp,
            # Position in this mcap's /camera/image stream, used to match images against
            # transforms. Not the .svo2 frame index: ZED_SVO_Editor drops a few frames per
            # file, so the two differ by a small offset.
            "image_stream_index": position,
            "method": method,
            "source_mcap": str(mcap_path),
            "provenance": provenance,
        }
        if cameraworld_from_camera is not None:
            field_from_cameraworld_matrix = field_from_cameraworld[field_tf_index][1]
            field_from_camera = field_from_cameraworld_matrix @ cameraworld_from_camera
            record["tf_field_from_camera"] = decompose(field_from_camera)
            record["tf_field_from_cameraworld"] = decompose(field_from_cameraworld_matrix)
            record["tf_cameraworld_from_camera"] = decompose(cameraworld_from_camera)
            record["provenance"]["field_tf_stamp_ns"] = int(
                field_from_cameraworld[field_tf_index][0]
            )
            # Extents the RANSAC plane fit measured at this field init, centred on the field
            # origin. Needed to know where the arena edges are: the transform alone does not
            # say. Estimates vary per init and some are plainly bad fits, so the value is
            # reported as measured rather than snapped to a nominal arena size.
            size_index = bisect_right(field_size_stamps, stamp) - 1
            size = None if size_index < 0 else field_sizes[size_index][1:]
            record["field_size_m"] = (
                {"x": round(size[0], 6), "y": round(size[1], 6)} if size else None
            )

        counts[method] = counts.get(method, 0) + 1
        if not dry_run:
            (transforms_dir / f"{stamp}.json").write_text(json.dumps(record, indent=2) + "\n")

    return counts


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-dir", type=Path, default=DEFAULT_DATASET_DIR)
    parser.add_argument(
        "--dry-run", action="store_true", help="report coverage without writing files"
    )
    args = parser.parse_args()

    sub_dirs = sorted(path for path in args.dataset_dir.iterdir() if path.is_dir())
    totals: dict[str, int] = {}
    for sub_dir in sub_dirs:
        counts = export_sub_dataset(sub_dir, args.dry_run)
        for method, count in counts.items():
            totals[method] = totals.get(method, 0) + count
        breakdown = "  ".join(f"{method}={count}" for method, count in sorted(counts.items()))
        print(f"{sub_dir.name:45s} {breakdown}")
    print(f"{'TOTAL':45s} " + "  ".join(f"{m}={c}" for m, c in sorted(totals.items())))


if __name__ == "__main__":
    main()
