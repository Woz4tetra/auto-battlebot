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
frame; raw is bit-identical to what the camera produced. MCAP chunk compression is chosen per format: JPEG
uses ZSTD (frames are already small, so it is cheap and shrinks them further), while raw uses no compression.
Compressing 6 MB raw frames per frame is the capture bottleneck (ZSTD ~26 ms, LZ4 ~13 ms, both cap fps below
60 and barely shrink noisy sensor data), so raw stores uncompressed (~3.6 ms/frame) and relies on the NVMe.

Timestamps are CLOCK_MONOTONIC seconds (the same clock calibrate_drive.py logs against); each frame's
monotonic time is the MCAP log_time, and analysis reads the pose timestamps straight back from it.
"""

from __future__ import annotations

import json
import math
import struct
from pathlib import Path
from typing import Iterator

import cv2
import numpy as np
from mcap.reader import make_reader
from mcap.writer import CompressionType, Writer

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
        # JPEG frames are tiny and already compressed, so ZSTD is cheap. Raw frames are 6 MB each
        # and ZSTD-ing each is the capture wall (~26 ms; LZ4 ~13 ms), both under 60 fps. Raw uses
        # NONE: ~3.6 ms/frame, lossless; NVMe carries the ~370 MB/s (~3.6 GB per ~10 s capture).
        compression = CompressionType.ZSTD if image_format == "jpeg" else CompressionType.NONE
        self._writer = Writer(self._file, compression=compression)
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


# --------------------------------------------------------------------------------------------------
# Foxglove overlay recording
#
# analyze_apriltag_mcap.py can emit a second MCAP that overlays the solved poses on the camera frames so
# the geometry can be eyeballed in Foxglove. It carries everything Foxglove needs to project the 3D scene
# onto the image in the 3D panel: the camera frames (copied verbatim from the source), a CameraInfo so the
# projection is calibrated, a TF tree (field -> camera fixed, field -> robot per frame), and marker
# geometry (a body cube + heading arrow at the robot, plus the full trajectory line).
#
# Frames (all ROS optical convention, which is exactly OpenCV's camera frame, so no extra optical rotation):
#   field  : the floor GridBoard frame, z = 0 on the floor. The fixed world frame.
#   camera : the camera optical frame (x right, y down, z forward).
#   robot  : the robot tag projected onto the field plane (x, y, 0) rotated by yaw about field z.
#
# Everything is ROS1-typed (tf2_msgs/TFMessage, sensor_msgs/CameraInfo, visualization_msgs/MarkerArray) to
# match the sensor_msgs images already in the recording; Foxglove renders all of these natively.
# --------------------------------------------------------------------------------------------------

TOPIC_CAMERA_INFO = "/camera/camera_info"
TOPIC_TF = "/tf"
TOPIC_TF_STATIC = "/tf_static"
TOPIC_MARKERS = "/overlay/markers"

# The calibration field frame (the GridBoard frame) can have +z pointing either way depending on board
# orientation; when it points toward the floor the camera renders below the field in Foxglove's z-up world.
# WORLD is a display-only root that flips the field (identity or 180deg about x, chosen from the camera's
# z) so the rig shows upright. It does not affect the image overlay, which depends only on the
# field -> camera relative transform.
WORLD_FRAME = "world"
FIELD_FRAME = "field"
CAMERA_FRAME = "camera"
ROBOT_FRAME = "robot"

# visualization_msgs/Marker type and action constants.
_MARKER_ARROW = 0
_MARKER_CUBE = 1
_MARKER_LINE_STRIP = 4
_ACTION_ADD = 0
_ACTION_DELETE = 2

TF_SCHEMA = (
    b"geometry_msgs/TransformStamped[] transforms\n"
    b"================================================================================\n"
    b"MSG: geometry_msgs/TransformStamped\n"
    b"std_msgs/Header header\n"
    b"string child_frame_id\n"
    b"geometry_msgs/Transform transform\n"
    b"================================================================================\n"
    b"MSG: std_msgs/Header\n"
    b"uint32 seq\n"
    b"time stamp\n"
    b"string frame_id\n"
    b"================================================================================\n"
    b"MSG: geometry_msgs/Transform\n"
    b"geometry_msgs/Vector3 translation\n"
    b"geometry_msgs/Quaternion rotation\n"
    b"================================================================================\n"
    b"MSG: geometry_msgs/Vector3\n"
    b"float64 x\n"
    b"float64 y\n"
    b"float64 z\n"
    b"================================================================================\n"
    b"MSG: geometry_msgs/Quaternion\n"
    b"float64 x\n"
    b"float64 y\n"
    b"float64 z\n"
    b"float64 w\n"
)

CAMERA_INFO_SCHEMA = (
    b"std_msgs/Header header\n"
    b"uint32 height\n"
    b"uint32 width\n"
    b"string distortion_model\n"
    b"float64[] D\n"
    b"float64[9] K\n"
    b"float64[9] R\n"
    b"float64[12] P\n"
    b"uint32 binning_x\n"
    b"uint32 binning_y\n"
    b"sensor_msgs/RegionOfInterest roi\n"
    b"================================================================================\n"
    b"MSG: std_msgs/Header\n"
    b"uint32 seq\n"
    b"time stamp\n"
    b"string frame_id\n"
    b"================================================================================\n"
    b"MSG: sensor_msgs/RegionOfInterest\n"
    b"uint32 x_offset\n"
    b"uint32 y_offset\n"
    b"uint32 height\n"
    b"uint32 width\n"
    b"bool do_rectify\n"
)

MARKER_ARRAY_SCHEMA = (
    b"visualization_msgs/Marker[] markers\n"
    b"================================================================================\n"
    b"MSG: visualization_msgs/Marker\n"
    b"std_msgs/Header header\n"
    b"string ns\n"
    b"int32 id\n"
    b"int32 type\n"
    b"int32 action\n"
    b"geometry_msgs/Pose pose\n"
    b"geometry_msgs/Vector3 scale\n"
    b"std_msgs/ColorRGBA color\n"
    b"duration lifetime\n"
    b"bool frame_locked\n"
    b"geometry_msgs/Point[] points\n"
    b"std_msgs/ColorRGBA[] colors\n"
    b"string text\n"
    b"string mesh_resource\n"
    b"bool mesh_use_embedded_materials\n"
    b"================================================================================\n"
    b"MSG: std_msgs/Header\n"
    b"uint32 seq\n"
    b"time stamp\n"
    b"string frame_id\n"
    b"================================================================================\n"
    b"MSG: geometry_msgs/Pose\n"
    b"geometry_msgs/Point position\n"
    b"geometry_msgs/Quaternion orientation\n"
    b"================================================================================\n"
    b"MSG: geometry_msgs/Point\n"
    b"float64 x\n"
    b"float64 y\n"
    b"float64 z\n"
    b"================================================================================\n"
    b"MSG: geometry_msgs/Quaternion\n"
    b"float64 x\n"
    b"float64 y\n"
    b"float64 z\n"
    b"float64 w\n"
    b"================================================================================\n"
    b"MSG: geometry_msgs/Vector3\n"
    b"float64 x\n"
    b"float64 y\n"
    b"float64 z\n"
    b"================================================================================\n"
    b"MSG: std_msgs/ColorRGBA\n"
    b"float32 r\n"
    b"float32 g\n"
    b"float32 b\n"
    b"float32 a\n"
)

def _pack_time(t: float) -> bytes:
    """ROS1 time (uint32 sec, uint32 nsec) for a monotonic-seconds timestamp."""
    sec = int(t)
    nsec = int(round((t - sec) * 1e9))
    if nsec >= 1_000_000_000:  # rounding can tip nsec over; carry into sec
        sec += 1
        nsec -= 1_000_000_000
    return struct.pack("<II", sec, nsec)


def _pack_string(s: str) -> bytes:
    b = s.encode("utf-8")
    return struct.pack("<I", len(b)) + b


def _pack_header(t: float, frame_id: str) -> bytes:
    return struct.pack("<I", 0) + _pack_time(t) + _pack_string(frame_id)


def _quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """Quaternion (x, y, z, w) for a rotation of `yaw` radians about z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _quat_from_matrix(r: np.ndarray) -> tuple[float, float, float, float]:
    """Quaternion (x, y, z, w) from a 3x3 rotation matrix (Shepperd's method)."""
    m = np.asarray(r, dtype=np.float64)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def _serialize_tf(t: float, transforms: list[tuple]) -> bytes:
    """tf2_msgs/TFMessage from a list of (parent, child, translation(3), quaternion(xyzw))."""
    out = [struct.pack("<I", len(transforms))]
    for parent, child, trans, quat in transforms:
        out.append(_pack_header(t, parent))
        out.append(_pack_string(child))
        out.append(struct.pack("<3d", *trans))
        out.append(struct.pack("<4d", *quat))
    return b"".join(out)


def _serialize_camera_info(t: float, frame_id: str, w: int, h: int,
                           k: list[float], d: list[float]) -> bytes:
    """sensor_msgs/CameraInfo for the (already-rectified-to-itself) pinhole+plumb_bob model used here."""
    k = [float(v) for v in k]
    d = [float(v) for v in d]
    # P is K with a zero fourth column (no stereo baseline): [fx 0 cx 0; 0 fy cy 0; 0 0 1 0].
    p = [k[0], k[1], k[2], 0.0, k[3], k[4], k[5], 0.0, k[6], k[7], k[8], 0.0]
    out = [
        _pack_header(t, frame_id),
        struct.pack("<II", h, w),
        _pack_string("plumb_bob"),
        struct.pack("<I", len(d)) + struct.pack("<%dd" % len(d), *d) if d else struct.pack("<I", 0),
        struct.pack("<9d", *k),
        struct.pack("<9d", 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),  # R = identity
        struct.pack("<12d", *p),
        struct.pack("<II", 0, 0),                    # binning_x, binning_y
        struct.pack("<IIII", 0, 0, 0, 0) + struct.pack("<B", 0),  # roi (+ do_rectify)
    ]
    return b"".join(out)


def _serialize_marker(t: float, frame_id: str, ns: str, mid: int, mtype: int, action: int,
                      pose: tuple, scale: tuple, color: tuple,
                      points: list[tuple] | None = None, frame_locked: bool = True) -> bytes:
    """One visualization_msgs/Marker (lifetime 0 = forever, no per-vertex colors / mesh)."""
    out = [
        _pack_header(t, frame_id),
        _pack_string(ns),
        struct.pack("<iii", mid, mtype, action),
        struct.pack("<7d", *pose),                   # position(3) + orientation(4)
        struct.pack("<3d", *scale),
        struct.pack("<4f", *color),
        struct.pack("<ii", 0, 0),                    # lifetime
        struct.pack("<B", 1 if frame_locked else 0),
    ]
    pts = points or []
    out.append(struct.pack("<I", len(pts)))
    for px, py, pz in pts:
        out.append(struct.pack("<3d", px, py, pz))
    out.append(struct.pack("<I", 0))                 # colors[]
    out.append(_pack_string(""))                     # text
    out.append(_pack_string(""))                     # mesh_resource
    out.append(struct.pack("<B", 0))                 # mesh_use_embedded_materials
    return b"".join(out)


def _serialize_marker_array(markers: list[bytes]) -> bytes:
    return struct.pack("<I", len(markers)) + b"".join(markers)


class OverlayWriter:
    """Writes a Foxglove-viewable overlay MCAP: camera frames + CameraInfo + TF + pose markers.

    Geometry inputs use the same convention as apriltag_detect.solve_floor_extrinsic: (r_fc, t_fc) maps a
    field point into the camera frame (X_c = r_fc @ X_f + t_fc), and each pose row is (x, y, yaw) on the
    field plane.
    """

    # Robot body box (m) and heading arrow (m); tuned for a Mrs-Buff-sized bot, purely cosmetic.
    # Two sets are drawn: a "3d" set at the tag's true height (sits on the tag in the image) and a "floor"
    # set flattened to z=0 (the ground track, registered to the plywood). The floor set is dimmer.
    _BODY = (0.16, 0.16, 0.05)
    _ARROW = (0.22, 0.02, 0.04)
    _BODY_COLOR = (0.10, 0.80, 0.30, 0.6)
    _ARROW_COLOR = (1.00, 0.55, 0.05, 0.95)
    _TRAJ_COLOR = (0.20, 0.55, 1.00, 0.9)
    _BODY_FLOOR_COLOR = (0.45, 0.45, 0.50, 0.35)
    _ARROW_FLOOR_COLOR = (0.70, 0.55, 0.35, 0.5)
    _TRAJ_FLOOR_COLOR = (0.40, 0.45, 0.55, 0.55)

    def __init__(self, path: Path, k: list[float], d: list[float],
                 width: int, height: int) -> None:
        self._k = [float(v) for v in k]
        self._w = int(width)
        self._h = int(height)
        # Frames are undistorted before writing and the published CameraInfo carries zero distortion, so
        # Foxglove's pinhole projection of the 3D markers lands exactly on the (now rectified) image. The
        # ZED's full distortion model has too many coefficients for Foxglove to apply, so we bake it out
        # here instead. Keeping the same K as the new camera matrix preserves focal length and centre.
        k3 = np.asarray(k, dtype=np.float64).reshape(3, 3)
        dv = np.asarray(d, dtype=np.float64).reshape(-1)
        self._map1, self._map2 = cv2.initUndistortRectifyMap(
            k3, dv, None, k3, (self._w, self._h), cv2.CV_16SC2)
        self._file = open(path, "wb")
        self._writer = Writer(self._file)
        self._writer.start(profile="apriltag_overlay")

        img_schema = self._writer.register_schema(
            name="sensor_msgs/CompressedImage", encoding="ros1msg", data=COMPRESSED_IMAGE_SCHEMA)
        self._img_chan = self._writer.register_channel(
            topic=TOPIC_CAMERA_IMAGE, message_encoding="ros1", schema_id=img_schema)

        info_schema = self._writer.register_schema(
            name="sensor_msgs/CameraInfo", encoding="ros1msg", data=CAMERA_INFO_SCHEMA)
        self._info_chan = self._writer.register_channel(
            topic=TOPIC_CAMERA_INFO, message_encoding="ros1", schema_id=info_schema)

        tf_schema = self._writer.register_schema(
            name="tf2_msgs/TFMessage", encoding="ros1msg", data=TF_SCHEMA)
        self._tf_chan = self._writer.register_channel(
            topic=TOPIC_TF, message_encoding="ros1", schema_id=tf_schema)
        self._tf_static_chan = self._writer.register_channel(
            topic=TOPIC_TF_STATIC, message_encoding="ros1", schema_id=tf_schema)

        marker_schema = self._writer.register_schema(
            name="visualization_msgs/MarkerArray", encoding="ros1msg", data=MARKER_ARRAY_SCHEMA)
        self._marker_chan = self._writer.register_channel(
            topic=TOPIC_MARKERS, message_encoding="ros1", schema_id=marker_schema)
        self._seq = 0

    def _add(self, channel_id: int, t: float, data: bytes) -> None:
        self._seq += 1
        self._writer.add_message(channel_id=channel_id, log_time=_ns(t), publish_time=_ns(t),
                                 data=data, sequence=self._seq)

    def write_static_tf(self, t: float, r_fc: np.ndarray, t_fc: np.ndarray) -> None:
        """The two fixed transforms, on /tf_static (Foxglove latches them forever):

        world -> field : orients the field so the camera renders above the floor (Foxglove is z-up). The
                         calibration field frame's z can point either way depending on board orientation,
                         so this is identity or a 180deg-about-x flip chosen from the camera's z.
        field -> camera: the locked extrinsic (camera pose in the field frame).
        """
        r_fc = np.asarray(r_fc, dtype=np.float64).reshape(3, 3)
        t_fc = np.asarray(t_fc, dtype=np.float64).reshape(3)
        # Pose of the camera in the field frame: rotation r_fc^T, origin -r_fc^T @ t_fc.
        r_cf = r_fc.T
        cam_in_field = -r_cf @ t_fc
        quat = _quat_from_matrix(r_cf)
        # If the camera sits on the -z side of the field, flip 180deg about x so it shows above the floor.
        flip = (0.0, 0.0, 0.0, 1.0) if cam_in_field[2] >= 0.0 else (1.0, 0.0, 0.0, 0.0)
        self._add(self._tf_static_chan, t, _serialize_tf(t, [
            (WORLD_FRAME, FIELD_FRAME, (0.0, 0.0, 0.0), flip),
            (FIELD_FRAME, CAMERA_FRAME, cam_in_field.tolist(), quat),
        ]))

    def write_trajectory(self, t: float, rows: list[dict]) -> None:
        """Full solved path, drawn for the whole timeline as two field-frame LINE_STRIPs:

        trajectory_3d   : tag's true field height (z), so it overlays the tag in the image.
        trajectory_floor: flattened to z=0, the ground track registered to the floor plane.
        """
        pts_3d = [(float(r["x"]), float(r["y"]), float(r.get("z") or 0.0))
                  for r in rows if r["visible"]]
        if len(pts_3d) < 2:
            return
        pts_floor = [(p[0], p[1], 0.0) for p in pts_3d]
        identity = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        markers = [
            _serialize_marker(t, FIELD_FRAME, "trajectory_3d", 0, _MARKER_LINE_STRIP, _ACTION_ADD,
                              identity, (0.01, 0.0, 0.0), self._TRAJ_COLOR, points=pts_3d),
            _serialize_marker(t, FIELD_FRAME, "trajectory_floor", 0, _MARKER_LINE_STRIP, _ACTION_ADD,
                              identity, (0.01, 0.0, 0.0), self._TRAJ_FLOOR_COLOR, points=pts_floor),
        ]
        self._add(self._marker_chan, t, _serialize_marker_array(markers))

    def write_image(self, t: float, frame: np.ndarray) -> None:
        """Undistort a source camera frame and write it as JPEG so the pinhole overlay aligns."""
        rectified = cv2.remap(frame, self._map1, self._map2, cv2.INTER_LINEAR)
        ok, encoded = cv2.imencode(".jpg", rectified, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            raise RuntimeError("cv2.imencode failed for overlay frame")
        self._add(self._img_chan, t, _serialize_compressed_image(t, encoded.tobytes(), CAMERA_FRAME))

    def write_camera_info(self, t: float) -> None:
        # Zero distortion: the frames are already rectified, so the overlay is a pure pinhole projection.
        self._add(self._info_chan, t,
                  _serialize_camera_info(t, CAMERA_FRAME, self._w, self._h, self._k, [0.0] * 5))

    def _robot_markers(self, t: float, ns: str, x: float, y: float, z: float, quat: tuple,
                       body_color: tuple, arrow_color: tuple) -> list[bytes]:
        """A body CUBE (id 0) + heading ARROW (id 1) at (x, y, z) with the given yaw quaternion."""
        pose = (x, y, z, *quat)
        return [
            _serialize_marker(t, FIELD_FRAME, ns, 0, _MARKER_CUBE, _ACTION_ADD,
                              pose, self._BODY, body_color),
            _serialize_marker(t, FIELD_FRAME, ns, 1, _MARKER_ARROW, _ACTION_ADD,
                              pose, self._ARROW, arrow_color),
        ]

    def write_pose(self, t: float, row: dict) -> None:
        """field -> robot TF and the robot markers for one frame: a "robot_3d" set at the tag's true
        height and a "robot_floor" set flattened to z=0. Both are DELETEd when the tag is not visible."""
        if row["visible"]:
            x, y, yaw = float(row["x"]), float(row["y"]), float(row["yaw"])
            z = float(row.get("z") or 0.0)  # tag's true field height, so the 3d marker sits on the tag
            quat = _quat_from_yaw(yaw)
            self._add(self._tf_chan, t,
                      _serialize_tf(t, [(FIELD_FRAME, ROBOT_FRAME, (x, y, z), quat)]))
            markers = self._robot_markers(t, "robot_3d", x, y, z, quat,
                                          self._BODY_COLOR, self._ARROW_COLOR)
            markers += self._robot_markers(t, "robot_floor", x, y, 0.0, quat,
                                           self._BODY_FLOOR_COLOR, self._ARROW_FLOOR_COLOR)
            self._add(self._marker_chan, t, _serialize_marker_array(markers))
        else:
            empty, unit, clear = (0.0,) * 7, (1.0, 1.0, 1.0), (0.0, 0.0, 0.0, 0.0)
            markers = []
            for ns in ("robot_3d", "robot_floor"):
                markers.append(_serialize_marker(t, FIELD_FRAME, ns, 0, _MARKER_CUBE, _ACTION_DELETE,
                                                 empty, unit, clear))
                markers.append(_serialize_marker(t, FIELD_FRAME, ns, 1, _MARKER_ARROW, _ACTION_DELETE,
                                                 empty, unit, clear))
            self._add(self._marker_chan, t, _serialize_marker_array(markers))

    def close(self) -> None:
        self._writer.finish()
        self._file.close()


def write_overlay(out_path: Path, src_path: Path, metadata: dict, rows: list[dict],
                  r_fc: np.ndarray, t_fc: np.ndarray) -> None:
    """Write a Foxglove overlay MCAP next to the solved poses.

    Undistorts the source /camera/image frames and adds, per frame, a CameraInfo, the field->robot TF, and
    the robot markers (a true-3d set at the tag height and a floor set at z=0); plus the one-time static
    TFs (world->field, field->camera) and the trajectory (also 3d + floor). rows must be in the same order
    as the source frames (analyze_apriltag_mcap.solve_poses produces exactly that, one per frame).
    """
    out_path.parent.mkdir(parents=True, exist_ok=True)
    writer = OverlayWriter(
        out_path, metadata["camera_matrix"], metadata["dist_coeffs"],
        int(metadata["image_width"]), int(metadata["image_height"]))
    if rows:
        writer.write_static_tf(rows[0]["t"], r_fc, t_fc)
        writer.write_trajectory(rows[0]["t"], rows)
    for row, (_t, frame) in zip(rows, iter_images(src_path, TOPIC_CAMERA_IMAGE)):
        ts = row["t"]
        writer.write_image(ts, frame)
        writer.write_camera_info(ts)
        writer.write_pose(ts, row)
    writer.close()
