"""Shared MCAP reading and ROS1 wire decoding utilities.

The C++ stack records ROS1-encoded messages via McapRecorder. These helpers decode the
subset of message types the analysis tools need, without a mcap_ros1 dependency.

ROS1 wire format notes (all little-endian):
  Header: uint32 seq, uint32 stamp_secs, uint32 stamp_nsecs, string frame_id
  string: uint32 length + raw bytes (no null terminator)
"""

from __future__ import annotations

import json
import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterator

import cv2
import numpy as np
from mcap.reader import make_reader

DIAGNOSTICS_TOPIC = "/diagnostics"
BLOB_DETECTIONS_TOPIC = "/blob_detections"
KEYPOINT_DETECTIONS_TOPIC = "/keypoint_detections"
CAMERA_IMAGE_TOPIC = "/camera/image"
ROBOT_MARKERS_TOPIC = "/robot_markers"


def iter_messages(
    path: Path | str, topics: list[str] | None = None
) -> Iterator[tuple[str, int, bytes]]:
    """Yield (topic, log_time_ns, raw_ros1_bytes) from an MCAP recording."""
    with open(path, "rb") as file:
        reader = make_reader(file)
        for _schema, channel, message in reader.iter_messages(topics=topics):
            yield channel.topic, message.log_time, message.data


# ---------------------------------------------------------------------------
# Primitive readers
# ---------------------------------------------------------------------------


def _read_string(data: bytes, offset: int) -> tuple[str, int]:
    (length,) = struct.unpack_from("<I", data, offset)
    offset += 4
    s = data[offset : offset + length].decode("utf-8", errors="replace")
    return s, offset + length


def _read_uint32(data: bytes, offset: int) -> tuple[int, int]:
    (v,) = struct.unpack_from("<I", data, offset)
    return v, offset + 4


def _read_int8(data: bytes, offset: int) -> tuple[int, int]:
    (v,) = struct.unpack_from("<b", data, offset)
    return v, offset + 1


def _read_header(data: bytes, offset: int) -> tuple[int, str, int]:
    """Read a std_msgs/Header; returns (stamp_ns, frame_id, new_offset)."""
    _seq, offset = _read_uint32(data, offset)
    secs, offset = _read_uint32(data, offset)
    nsecs, offset = _read_uint32(data, offset)
    frame_id, offset = _read_string(data, offset)
    return secs * 1_000_000_000 + nsecs, frame_id, offset


# ---------------------------------------------------------------------------
# std_msgs/String
# ---------------------------------------------------------------------------


def decode_string(data: bytes) -> str:
    """Decode a std_msgs/String message."""
    s, _ = _read_string(data, 0)
    return s


# ---------------------------------------------------------------------------
# diagnostic_msgs/DiagnosticArray
# ---------------------------------------------------------------------------


def decode_diagnostic_array(data: bytes) -> list[dict]:
    """Decode a diagnostic_msgs/DiagnosticArray from raw ROS1 bytes.

    Returns a list of dicts, one per DiagnosticStatus:
    {level, name, message, hardware_id, values: {key: value}}
    """
    off = 0
    _stamp_ns, _frame_id, off = _read_header(data, off)

    status_count, off = _read_uint32(data, off)
    statuses = []
    for _ in range(status_count):
        level, off = _read_int8(data, off)
        name, off = _read_string(data, off)
        message, off = _read_string(data, off)
        hardware_id, off = _read_string(data, off)

        values_count, off = _read_uint32(data, off)
        values: dict[str, str] = {}
        for _ in range(values_count):
            key, off = _read_string(data, off)
            value, off = _read_string(data, off)
            values[key] = value

        statuses.append(
            {
                "level": level,
                "name": name,
                "message": message,
                "hardware_id": hardware_id,
                "values": values,
            }
        )
    return statuses


# ---------------------------------------------------------------------------
# sensor_msgs/CompressedImage
# ---------------------------------------------------------------------------


@dataclass
class CompressedImage:
    stamp_ns: int
    frame_id: str
    format: str
    image: np.ndarray


def decode_image_stamp_ns(data: bytes) -> int:
    """Read only the header stamp of a sensor_msgs/CompressedImage (no image decode).

    Recorded log_time is wall clock, but the header stamp is the frame stamp; use this
    to align images with /detections without decoding every frame.
    """
    stamp_ns, _frame_id, _off = _read_header(data, 0)
    return stamp_ns


def decode_compressed_image(data: bytes) -> CompressedImage:
    """Decode a sensor_msgs/CompressedImage message into a BGR numpy image."""
    stamp_ns, frame_id, off = _read_header(data, 0)
    fmt, off = _read_string(data, off)
    length, off = _read_uint32(data, off)
    payload = np.frombuffer(data, dtype=np.uint8, count=length, offset=off)
    image = cv2.imdecode(payload, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Failed to decode compressed image (format={fmt!r})")
    return CompressedImage(stamp_ns=stamp_ns, frame_id=frame_id, format=fmt, image=image)


# ---------------------------------------------------------------------------
# /blob_detections and /keypoint_detections
# (std_msgs/String with a JSON payload from ros_detections.cpp)
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# sensor_msgs/CameraInfo
# ---------------------------------------------------------------------------


@dataclass
class CameraInfo:
    stamp_ns: int
    frame_id: str
    width: int
    height: int
    distortion_model: str
    distortion: np.ndarray
    intrinsics: np.ndarray  # 3x3 K
    projection: np.ndarray  # 3x4 P


def decode_camera_info(data: bytes) -> CameraInfo:
    """Decode a sensor_msgs/CameraInfo message from raw ROS1 bytes."""
    stamp_ns, frame_id, off = _read_header(data, 0)
    height, off = _read_uint32(data, off)
    width, off = _read_uint32(data, off)
    distortion_model, off = _read_string(data, off)

    distortion_count, off = _read_uint32(data, off)
    distortion = np.frombuffer(data, dtype="<f8", count=distortion_count, offset=off)
    off += 8 * distortion_count

    intrinsics = np.frombuffer(data, dtype="<f8", count=9, offset=off).reshape(3, 3)
    off += 72
    off += 72  # rectification matrix R, unused here
    projection = np.frombuffer(data, dtype="<f8", count=12, offset=off).reshape(3, 4)
    off += 96

    return CameraInfo(
        stamp_ns=stamp_ns,
        frame_id=frame_id,
        width=width,
        height=height,
        distortion_model=distortion_model,
        distortion=distortion.copy(),
        intrinsics=intrinsics.copy(),
        projection=projection.copy(),
    )


# ---------------------------------------------------------------------------
# tf2_msgs/TFMessage
# ---------------------------------------------------------------------------


@dataclass
class Transform:
    stamp_ns: int
    parent_frame_id: str
    child_frame_id: str
    matrix: np.ndarray  # 4x4 homogeneous, maps points in child into parent

    @property
    def key(self) -> tuple[str, str]:
        return (self.parent_frame_id, self.child_frame_id)


def _quaternion_to_rotation(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def decode_tf_message(data: bytes) -> list[Transform]:
    """Decode a tf2_msgs/TFMessage into 4x4 parent-from-child matrices."""
    count, off = _read_uint32(data, 0)
    transforms = []
    for _ in range(count):
        stamp_ns, parent_frame_id, off = _read_header(data, off)
        child_frame_id, off = _read_string(data, off)
        tx, ty, tz, qx, qy, qz, qw = struct.unpack_from("<7d", data, off)
        off += 56
        matrix = np.eye(4)
        matrix[:3, :3] = _quaternion_to_rotation(qx, qy, qz, qw)
        matrix[:3, 3] = (tx, ty, tz)
        transforms.append(
            Transform(
                stamp_ns=stamp_ns,
                parent_frame_id=parent_frame_id,
                child_frame_id=child_frame_id,
                matrix=matrix,
            )
        )
    return transforms


@dataclass
class DetectionKeypoint:
    x: float
    y: float
    confidence: float


@dataclass
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float
    class_id: int
    label: str
    keypoints: list[DetectionKeypoint] = field(default_factory=list)


@dataclass
class Detections:
    stamp: float
    image_width: int
    image_height: int
    detections: list[Detection] = field(default_factory=list)

    @property
    def stamp_ns(self) -> int:
        return int(round(self.stamp * 1e9))


def decode_detections(data: bytes) -> Detections:
    """Decode a detections message (JSON payload in a std_msgs/String)."""
    payload = json.loads(decode_string(data))
    return Detections(
        stamp=float(payload["stamp"]),
        image_width=int(payload["w"]),
        image_height=int(payload["h"]),
        detections=[
            Detection(
                x1=float(det["x1"]),
                y1=float(det["y1"]),
                x2=float(det["x2"]),
                y2=float(det["y2"]),
                confidence=float(det["conf"]),
                class_id=int(det["class_id"]),
                label=str(det["label"]),
                keypoints=[
                    DetectionKeypoint(x=float(kp[0]), y=float(kp[1]), confidence=float(kp[2]))
                    for kp in det.get("kps", [])
                ],
            )
            for det in payload["dets"]
        ],
    )


def read_detections(path: Path | str, topic: str = BLOB_DETECTIONS_TOPIC) -> list[Detections]:
    """Read every detections message on a topic from an MCAP recording, in log-time order."""
    return [decode_detections(data) for _topic, _ts, data in iter_messages(path, [topic])]


# ---------------------------------------------------------------------------
# Stamp alignment
# ---------------------------------------------------------------------------


def match_stamps(
    reference_ns: list[int], candidate_ns: list[int], tolerance_ns: int = 1_000_000
) -> dict[int, int]:
    """Match each reference stamp to the nearest candidate stamp within tolerance.

    Playback replays of the same SVO produce identical frame stamps, but the stamp reaches
    different topics through different double->ns conversions, so allow a small tolerance
    (default 1 ms; frames are >15 ms apart). Returns {reference_stamp: candidate_stamp}.
    """
    matches: dict[int, int] = {}
    if not reference_ns or not candidate_ns:
        return matches
    sorted_candidates = sorted(candidate_ns)
    arr = np.asarray(sorted_candidates, dtype=np.int64)
    for ref in reference_ns:
        idx = int(np.searchsorted(arr, ref))
        best: int | None = None
        for j in (idx - 1, idx):
            if 0 <= j < len(arr):
                if best is None or abs(int(arr[j]) - ref) < abs(best - ref):
                    best = int(arr[j])
        if best is not None and abs(best - ref) <= tolerance_ns:
            matches[ref] = best
    return matches
