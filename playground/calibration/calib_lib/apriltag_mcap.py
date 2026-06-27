"""MCAP layout for the AprilTag ground-truth pipeline: raw camera images in, poses out (offline).

apriltag_track.py records the camera images here; analyze_apriltag_mcap.py replays them, re-runs the
AprilTag detection, and solves the field-plane pose. Recording images instead of detections means the
floor lock, the detector tuning, the intrinsics, and the yaw offset can all be corrected and re-run
offline without re-driving the robot.

A recording holds:
  - /calibration/metadata : one JSON message (intrinsics, image size, robot tag + floor board params).
  - /floor/image          : the frames captured during the one-time floor-board lock (board visible).
  - /camera/image         : the driving frames to track the robot tag in.

Images are stored either as ROS1 sensor_msgs/CompressedImage (JPEG, the default) or sensor_msgs/Image
(uncompressed bgr8, lossless) depending on the capture format; both open in Foxglove. JPEG is lossy at the
marker edges the subpixel corner refinement keys on, so its stored corners differ slightly from the live
frame; raw is bit-identical to what the camera produced. The MCAP chunks are ZSTD-compressed regardless,
so raw frames still shrink losslessly on disk rather than landing at full BGR size.

Timestamps are CLOCK_MONOTONIC seconds (the same clock calibrate_drive.py logs against); each frame's
monotonic time is the MCAP log_time, and analysis reads the pose timestamps straight back from it.
"""

from __future__ import annotations

import json
import struct
from pathlib import Path
from typing import Iterator

import cv2
import numpy as np
from mcap.reader import make_reader
from mcap.writer import Writer

TOPIC_METADATA = "/calibration/metadata"
TOPIC_FLOOR_IMAGE = "/floor/image"
TOPIC_CAMERA_IMAGE = "/camera/image"

# JPEG quality for the stored frames. High enough that the lossy edges do not meaningfully move the
# subpixel-refined marker corners, while keeping 1080p frames a few hundred KB so 60 fps capture sustains.
JPEG_QUALITY = 95

METADATA_SCHEMA = {
    "type": "object",
    "properties": {
        "t": {"type": "number", "description": "CLOCK_MONOTONIC seconds at recording start"},
        "camera_matrix": {"type": "array", "items": {"type": "number"},
                          "description": "3x3 K, row-major (9 values)"},
        "dist_coeffs": {"type": "array", "items": {"type": "number"}},
        "image_width": {"type": "integer"},
        "image_height": {"type": "integer"},
        "tag_size": {"type": "number", "description": "robot tag edge length (m)"},
        "tag_id": {"type": "integer"},
        "yaw_offset_deg": {"type": "number"},
        "floor": {"type": "object", "description": "floor grid board parameters for the lock"},
        "image_format": {"type": "string", "description": "jpeg (lossy) or raw (lossless bgr8)"},
        "clock": {"type": "string"},
    },
}

# ros1msg schema for sensor_msgs/CompressedImage (Foxglove renders this natively for image overlays).
COMPRESSED_IMAGE_SCHEMA = (
    b"std_msgs/Header header\n"
    b"string format\n"
    b"uint8[] data\n"
    b"\n"
    b"================================================================================\n"
    b"MSG: std_msgs/Header\n"
    b"uint32 seq\n"
    b"time stamp\n"
    b"string frame_id\n"
)

# ros1msg schema for sensor_msgs/Image (uncompressed bgr8 for the lossless raw format).
RAW_IMAGE_SCHEMA = (
    b"std_msgs/Header header\n"
    b"uint32 height\n"
    b"uint32 width\n"
    b"string encoding\n"
    b"uint8 is_bigendian\n"
    b"uint32 step\n"
    b"uint8[] data\n"
    b"\n"
    b"================================================================================\n"
    b"MSG: std_msgs/Header\n"
    b"uint32 seq\n"
    b"time stamp\n"
    b"string frame_id\n"
)

IMAGE_FORMATS = ("jpeg", "raw")


def _ns(t: float) -> int:
    """Monotonic seconds to integer nanoseconds for the MCAP log_time."""
    return int(round(t * 1e9))


def _serialize_compressed_image(t: float, jpeg: bytes, frame_id: str = "camera") -> bytes:
    """ROS1 sensor_msgs/CompressedImage wire bytes (little-endian, strings are uint32 len + raw bytes)."""
    sec = int(t)
    nsec = int(round((t - sec) * 1e9))
    if nsec >= 1_000_000_000:  # rounding can tip nsec over; carry into sec
        sec += 1
        nsec -= 1_000_000_000
    fid = frame_id.encode("utf-8")
    fmt = b"jpeg"
    return b"".join([
        struct.pack("<I", 0),               # header.seq
        struct.pack("<II", sec, nsec),      # header.stamp
        struct.pack("<I", len(fid)) + fid,  # header.frame_id
        struct.pack("<I", len(fmt)) + fmt,  # format
        struct.pack("<I", len(jpeg)) + jpeg,  # data
    ])


def _decode_compressed_image(data: bytes) -> np.ndarray:
    """Inverse of _serialize_compressed_image: pull the JPEG payload out and decode to a BGR frame."""
    off = 12  # header.seq (4) + header.stamp (8)
    (fid_len,) = struct.unpack_from("<I", data, off); off += 4 + fid_len
    (fmt_len,) = struct.unpack_from("<I", data, off); off += 4 + fmt_len
    (data_len,) = struct.unpack_from("<I", data, off); off += 4
    jpeg = np.frombuffer(data, dtype=np.uint8, count=data_len, offset=off)
    return cv2.imdecode(jpeg, cv2.IMREAD_COLOR)


def _serialize_raw_image(t: float, frame: np.ndarray, frame_id: str = "camera") -> bytes:
    """ROS1 sensor_msgs/Image wire bytes for an uncompressed bgr8 frame (lossless)."""
    sec = int(t)
    nsec = int(round((t - sec) * 1e9))
    if nsec >= 1_000_000_000:
        sec += 1
        nsec -= 1_000_000_000
    h, w = frame.shape[:2]
    fid = frame_id.encode("utf-8")
    enc = b"bgr8"
    payload = np.ascontiguousarray(frame).tobytes()
    return b"".join([
        struct.pack("<I", 0),               # header.seq
        struct.pack("<II", sec, nsec),      # header.stamp
        struct.pack("<I", len(fid)) + fid,  # header.frame_id
        struct.pack("<I", h),               # height
        struct.pack("<I", w),               # width
        struct.pack("<I", len(enc)) + enc,  # encoding
        struct.pack("<B", 0),               # is_bigendian
        struct.pack("<I", w * 3),           # step
        struct.pack("<I", len(payload)) + payload,  # data
    ])


def _decode_raw_image(data: bytes) -> np.ndarray:
    """Inverse of _serialize_raw_image: reshape the uncompressed bgr8 bytes into a BGR frame."""
    off = 12  # header.seq (4) + header.stamp (8)
    (fid_len,) = struct.unpack_from("<I", data, off); off += 4 + fid_len
    (h,) = struct.unpack_from("<I", data, off); off += 4
    (w,) = struct.unpack_from("<I", data, off); off += 4
    (enc_len,) = struct.unpack_from("<I", data, off); off += 4 + enc_len  # encoding (bgr8)
    off += 1  # is_bigendian
    off += 4  # step
    (data_len,) = struct.unpack_from("<I", data, off); off += 4
    return np.frombuffer(data, dtype=np.uint8, count=data_len, offset=off).reshape(h, w, 3)


def _decode_image(schema, data: bytes) -> np.ndarray:
    """Decode an image message by its schema: sensor_msgs/Image is raw bgr8, else JPEG CompressedImage."""
    if schema is not None and schema.name == "sensor_msgs/Image":
        return _decode_raw_image(data)
    return _decode_compressed_image(data)


class CaptureWriter:
    """Writes the calibration metadata and the floor / camera image streams to an MCAP file.

    Metadata is written lazily, just before the first image, so its image_width/image_height come straight
    from a real frame rather than from possibly-unreliable capture properties. Call set_metadata() once
    before recording any frames.
    """

    def __init__(self, path: Path, image_format: str = "jpeg") -> None:
        if image_format not in IMAGE_FORMATS:
            raise ValueError(f"image_format must be one of {IMAGE_FORMATS}, got {image_format!r}")
        self._image_format = image_format
        self._file = open(path, "wb")
        self._writer = Writer(self._file)
        self._writer.start(profile="apriltag_track")
        self._meta_chan = self._writer.register_channel(
            topic=TOPIC_METADATA, message_encoding="json",
            schema_id=self._writer.register_schema(
                name="apriltag_calibration_metadata", encoding="jsonschema",
                data=json.dumps(METADATA_SCHEMA).encode()),
        )
        if image_format == "raw":
            img_schema = self._writer.register_schema(
                name="sensor_msgs/Image", encoding="ros1msg", data=RAW_IMAGE_SCHEMA)
        else:
            img_schema = self._writer.register_schema(
                name="sensor_msgs/CompressedImage", encoding="ros1msg", data=COMPRESSED_IMAGE_SCHEMA)
        self._floor_chan = self._writer.register_channel(
            topic=TOPIC_FLOOR_IMAGE, message_encoding="ros1", schema_id=img_schema)
        self._cam_chan = self._writer.register_channel(
            topic=TOPIC_CAMERA_IMAGE, message_encoding="ros1", schema_id=img_schema)
        self._meta: dict | None = None
        self._meta_written = False
        self._seq = 0

    def set_metadata(self, meta: dict) -> None:
        """Stash the metadata (everything except image_width/image_height, which are filled per frame)."""
        self._meta = dict(meta)

    def _ensure_metadata(self, t: float, frame: np.ndarray) -> None:
        if self._meta_written:
            return
        if self._meta is None:
            raise RuntimeError("set_metadata() must be called before recording frames")
        h, w = frame.shape[:2]
        payload = {**self._meta, "t": t, "image_width": int(w), "image_height": int(h),
                   "image_format": self._image_format}
        self._writer.add_message(channel_id=self._meta_chan, log_time=_ns(t), publish_time=_ns(t),
                                 data=json.dumps(payload).encode(), sequence=0)
        self._meta_written = True

    def _serialize(self, t: float, frame: np.ndarray) -> bytes:
        if self._image_format == "raw":
            return _serialize_raw_image(t, frame)
        ok, enc = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            raise RuntimeError("cv2.imencode failed to JPEG-encode a frame")
        return _serialize_compressed_image(t, enc.tobytes())

    def _write_image(self, channel_id: int, t: float, frame: np.ndarray) -> None:
        self._ensure_metadata(t, frame)
        self._seq += 1
        self._writer.add_message(channel_id=channel_id, log_time=_ns(t), publish_time=_ns(t),
                                 data=self._serialize(t, frame), sequence=self._seq)

    def write_floor_image(self, t: float, frame: np.ndarray) -> None:
        self._write_image(self._floor_chan, t, frame)

    def write_image(self, t: float, frame: np.ndarray) -> None:
        self._write_image(self._cam_chan, t, frame)

    def close(self) -> None:
        self._writer.finish()
        self._file.close()


def read_metadata(path: Path) -> dict:
    """Return the calibration metadata dict from a recording."""
    with open(path, "rb") as f:
        for _schema, channel, message in make_reader(f).iter_messages(topics=[TOPIC_METADATA]):
            return json.loads(message.data)
    raise SystemExit(f"{path}: no {TOPIC_METADATA} message; not an apriltag_track recording.")


def iter_images(path: Path, topic: str) -> Iterator[tuple[float, np.ndarray]]:
    """Yield (t_seconds, bgr_frame) for each image on `topic`, in recorded order.

    Handles both formats: the frame decodes from JPEG or raw bgr8 based on the message's schema, so analysis
    never needs to know how the recording was captured.
    """
    with open(path, "rb") as f:
        for schema, _channel, message in make_reader(f).iter_messages(topics=[topic]):
            yield message.log_time / 1e9, _decode_image(schema, message.data)


def read_floor_frames(path: Path) -> list[np.ndarray]:
    """Load every floor-lock frame (the board-visible burst) into memory for the one-time extrinsic solve."""
    return [frame for _t, frame in iter_images(path, TOPIC_FLOOR_IMAGE)]
