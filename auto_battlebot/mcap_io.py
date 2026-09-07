"""Shared MCAP reading utilities.

Recordings follow ``docs/foxglove_recording_format.md``: ``protobuf`` channels carrying Foxglove
schemas and ``json`` channels carrying the project's own payloads. The C++ stack writes them with
the Foxglove SDK; ``auto_battlebot.mcap_write`` writes them from Python.

``iter_messages`` tags every payload it yields with the channel's message encoding, schema name
and topic, and every ``decode_*`` function dispatches on that tag. Untagged bytes are treated as
the legacy ROS 1 wire format, which older recordings still hold until they go through
``scripts/convert_ros1_mcap.py``. The legacy decoders sit together at the bottom of this file so
they can be deleted as one block once the corpus is converted.
"""

from __future__ import annotations

import json
import math
import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Iterator

import cv2
import numpy as np
from mcap.reader import make_reader
from mcap.records import Schema
from mcap_protobuf.decoder import DecoderFactory as _ProtobufDecoderFactory

DIAGNOSTICS_TOPIC = "/diagnostics"
DIAGNOSTICS_TOPIC_PREFIX = "/diagnostics/"
BLOB_DETECTIONS_TOPIC = "/blob_detections"
KEYPOINT_DETECTIONS_TOPIC = "/keypoint_detections"
CAMERA_IMAGE_TOPIC = "/camera/image"
CAMERA_INFO_TOPIC = "/camera/camera_info"
FRAME_META_TOPIC = "/camera/frame_meta"
TF_TOPIC = "/tf"
FIELD_MARKERS_TOPIC = "/field_markers"
FIELD_POINTS_TOPIC = "/field_points"
ROBOT_MARKERS_TOPIC = "/robot_markers"
LOG_TOPIC = "/log"
LEGACY_LOG_TOPIC = "/rosout"

ENCODING_ROS1 = "ros1"
ENCODING_PROTOBUF = "protobuf"
ENCODING_JSON = "json"

_NS = 1_000_000_000


class MessageBytes(bytes):
    """Raw message payload tagged with how to decode it.

    ``iter_messages`` yields these; the tag is what lets one ``decode_*`` call serve every
    recording format. Plain ``bytes`` decode as legacy ROS 1.
    """

    encoding: str
    schema_name: str
    topic: str

    def __new__(cls, data: bytes, encoding: str, schema_name: str, topic: str) -> MessageBytes:
        obj = super().__new__(cls, data)
        obj.encoding = encoding
        obj.schema_name = schema_name
        obj.topic = topic
        return obj


def encoding_of(data: bytes) -> str:
    return getattr(data, "encoding", ENCODING_ROS1)


def topic_of(data: bytes) -> str:
    return getattr(data, "topic", "")


# Protobuf decoders keyed by schema name, filled from the schema records of every file read
# through ``iter_messages``. Foxglove schema names are stable, so a decoder learned from one
# recording serves any other.
_PROTO_DECODERS: dict[str, Callable[[bytes], Any]] = {}


def _register_proto_schema(schema: Schema | None) -> None:
    if schema is None or schema.encoding != "protobuf" or schema.name in _PROTO_DECODERS:
        return
    # One factory per schema: the factory caches decoders by the file-local schema id, which
    # collides across files (id 1 is CompressedImage in one recording and RawImage in another).
    decoder = _ProtobufDecoderFactory().decoder_for("protobuf", schema)
    if decoder is not None:
        _PROTO_DECODERS[schema.name] = decoder


def _proto(data: bytes) -> Any:
    name = getattr(data, "schema_name", "")
    try:
        decoder = _PROTO_DECODERS[name]
    except KeyError:
        raise ValueError(
            f"No protobuf decoder for schema {name!r}; read the message through iter_messages"
        ) from None
    return decoder(bytes(data))


def _expand_topics(reader: Any, topics: list[str] | None) -> list[str] | None:
    """Map requested topics onto the channels a file has.

    ``/diagnostics`` selects every per-module ``/diagnostics/<module>`` channel as well as the
    legacy single topic, and ``/rosout`` also selects ``/log``, so callers keep one name.
    """
    if topics is None:
        return None
    summary = reader.get_summary()
    if summary is None:
        return topics
    present = {channel.topic for channel in summary.channels.values()}
    selected: list[str] = []
    for topic in topics:
        if topic in present:
            selected.append(topic)
        if topic == DIAGNOSTICS_TOPIC:
            selected.extend(t for t in present if t.startswith(DIAGNOSTICS_TOPIC_PREFIX))
        if topic == LEGACY_LOG_TOPIC and LOG_TOPIC in present:
            selected.append(LOG_TOPIC)
    return selected


def iter_messages(
    path: Path | str, topics: list[str] | None = None
) -> Iterator[tuple[str, int, MessageBytes]]:
    """Yield (topic, log_time_ns, tagged payload) from an MCAP recording, in log-time order."""
    with open(path, "rb") as file:
        reader = make_reader(file)
        selected = _expand_topics(reader, topics)
        if selected is not None and not selected:
            return
        for schema, channel, message in reader.iter_messages(topics=selected):
            _register_proto_schema(schema)
            yield (
                channel.topic,
                message.log_time,
                MessageBytes(
                    message.data,
                    channel.message_encoding,
                    schema.name if schema is not None else "",
                    channel.topic,
                ),
            )


def read_active_profile(path: Path | str) -> str | None:
    """The config profile id stored in the ``auto_battlebot`` metadata record, if any."""
    with open(path, "rb") as file:
        for record in make_reader(file).iter_metadata():
            if record.name == "auto_battlebot":
                return record.metadata.get("active_profile")
    return None


# ---------------------------------------------------------------------------
# Timestamps
# ---------------------------------------------------------------------------


def _proto_stamp_ns(stamp: Any) -> int:
    return int(stamp.seconds) * _NS + int(stamp.nanos)


def stamp_to_ns(stamp_seconds: float) -> int:
    """``Header.stamp`` (double seconds) to integer nanoseconds, matching the C++ split."""
    sec = math.floor(stamp_seconds)
    nsec = int(round((stamp_seconds - sec) * 1e9))
    nsec = min(max(nsec, 0), _NS - 1)
    return sec * _NS + nsec


# ---------------------------------------------------------------------------
# Strings and JSON payloads
# ---------------------------------------------------------------------------


def decode_string(data: bytes) -> str:
    """The text of a JSON channel message (or a legacy ``std_msgs/String``)."""
    if encoding_of(data) == ENCODING_ROS1:
        return _ros1_decode_string(data)
    return bytes(data).decode("utf-8")


def decode_json(data: bytes) -> Any:
    return json.loads(decode_string(data))


@dataclass
class FrameMeta:
    image_stamp_ns: int
    svo_frame_index: int
    svo_path: str


def decode_frame_meta(data: bytes) -> FrameMeta:
    """``/camera/frame_meta``. ``image_stamp_ns`` is a decimal string on the wire (it is above
    2^53); the legacy bare-number form is accepted too."""
    payload = decode_json(data)
    return FrameMeta(
        image_stamp_ns=int(payload["image_stamp_ns"]),
        svo_frame_index=int(payload.get("svo_frame_index", -1)),
        svo_path=str(payload.get("svo_path", "")),
    )


# ---------------------------------------------------------------------------
# Diagnostics
# ---------------------------------------------------------------------------


def decode_diagnostic_array(data: bytes) -> list[dict]:
    """One dict per diagnostic status: ``{level, name, message, hardware_id, values}``.

    For a ``/diagnostics/<module>`` JSON message ``hardware_id`` is the module (from the topic),
    ``name`` is the section key and ``values`` are typed. For a legacy ``DiagnosticArray`` every
    value is a string, as it was on the wire.
    """
    if encoding_of(data) == ENCODING_ROS1:
        return _ros1_decode_diagnostic_array(data)
    topic = topic_of(data)
    module = (
        topic[len(DIAGNOSTICS_TOPIC_PREFIX) :] if topic.startswith(DIAGNOSTICS_TOPIC_PREFIX) else ""
    )
    payload = decode_json(data)
    return [
        {
            "level": int(section.get("level", 0)),
            "name": name,
            "message": str(section.get("message", "")),
            "hardware_id": module,
            "values": dict(section.get("values", {})),
        }
        for name, section in payload.items()
    ]


# ---------------------------------------------------------------------------
# Compressed images
# ---------------------------------------------------------------------------


@dataclass
class CompressedImage:
    stamp_ns: int
    frame_id: str
    format: str
    image: np.ndarray


def decode_image_stamp_ns(data: bytes) -> int:
    """Only the header stamp of a compressed image (no JPEG decode).

    The recorded log_time is wall clock; the header stamp is the frame stamp. Use this to align
    images with detections without decoding every frame.
    """
    if encoding_of(data) == ENCODING_ROS1:
        stamp_ns, _frame_id, _off = _ros1_read_header(data, 0)
        return stamp_ns
    return _proto_stamp_ns(_proto(data).timestamp)


def decode_compressed_image_bytes(data: bytes) -> tuple[int, str, str, bytes]:
    """(stamp_ns, frame_id, format, encoded bytes) without decoding the image."""
    if encoding_of(data) == ENCODING_ROS1:
        return _ros1_decode_compressed_image_bytes(data)
    msg = _proto(data)
    return _proto_stamp_ns(msg.timestamp), str(msg.frame_id), str(msg.format), bytes(msg.data)


def decode_raw_image_bytes(data: bytes) -> tuple[int, str, np.ndarray, str]:
    """(stamp_ns, frame_id, HxWxC uint8 array, encoding) of a ``foxglove.RawImage`` (or legacy
    ``sensor_msgs/Image``). Only 8-bit encodings are supported."""
    if encoding_of(data) == ENCODING_ROS1:
        return _ros1_decode_raw_image_bytes(data)
    msg = _proto(data)
    height, width, step = int(msg.height), int(msg.width), int(msg.step)
    encoding = str(msg.encoding)
    channels = max(step // max(width, 1), 1)
    frame = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(height, step)[
        :, : width * channels
    ]
    return (
        _proto_stamp_ns(msg.timestamp),
        str(msg.frame_id),
        frame.reshape(height, width, channels),
        encoding,
    )


def decode_compressed_image(data: bytes) -> CompressedImage:
    """Decode a compressed image message into a BGR numpy image."""
    stamp_ns, frame_id, fmt, payload = decode_compressed_image_bytes(data)
    image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Failed to decode compressed image (format={fmt!r})")
    return CompressedImage(stamp_ns=stamp_ns, frame_id=frame_id, format=fmt, image=image)


# ---------------------------------------------------------------------------
# Camera calibration
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
    rectification: np.ndarray = field(default_factory=lambda: np.eye(3))  # 3x3 R


def decode_camera_info(data: bytes) -> CameraInfo:
    if encoding_of(data) == ENCODING_ROS1:
        return _ros1_decode_camera_info(data)
    msg = _proto(data)
    k = np.asarray(list(msg.K), dtype=np.float64)
    p = np.asarray(list(msg.P), dtype=np.float64)
    r = np.asarray(list(msg.R), dtype=np.float64)
    return CameraInfo(
        stamp_ns=_proto_stamp_ns(msg.timestamp),
        frame_id=str(msg.frame_id),
        width=int(msg.width),
        height=int(msg.height),
        distortion_model=str(msg.distortion_model),
        distortion=np.asarray(list(msg.D), dtype=np.float64),
        intrinsics=k.reshape(3, 3) if k.size == 9 else np.eye(3),
        projection=p.reshape(3, 4) if p.size == 12 else np.zeros((3, 4)),
        rectification=r.reshape(3, 3) if r.size == 9 else np.eye(3),
    )


# ---------------------------------------------------------------------------
# Frame transforms
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


def quaternion_to_rotation(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _transform_matrix(
    tx: float, ty: float, tz: float, qx: float, qy: float, qz: float, qw: float
) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_to_rotation(qx, qy, qz, qw)
    matrix[:3, 3] = (tx, ty, tz)
    return matrix


def decode_tf_message(data: bytes) -> list[Transform]:
    """Decode a ``FrameTransforms`` (or legacy ``tf2_msgs/TFMessage``) into 4x4 matrices."""
    if encoding_of(data) == ENCODING_ROS1:
        return _ros1_decode_tf_message(data)
    msg = _proto(data)
    return [
        Transform(
            stamp_ns=_proto_stamp_ns(tf.timestamp),
            parent_frame_id=str(tf.parent_frame_id),
            child_frame_id=str(tf.child_frame_id),
            matrix=_transform_matrix(
                tf.translation.x,
                tf.translation.y,
                tf.translation.z,
                tf.rotation.x,
                tf.rotation.y,
                tf.rotation.z,
                tf.rotation.w,
            ),
        )
        for tf in msg.transforms
    ]


# ---------------------------------------------------------------------------
# Detections
# ---------------------------------------------------------------------------


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
    """Decode a detections message (JSON payload, ``auto_battlebot.Detections``)."""
    payload = decode_json(data)
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
# Point clouds
# ---------------------------------------------------------------------------

_POINT_FIELD_DTYPES = {
    1: np.uint8,
    2: np.int8,
    3: np.uint16,
    4: np.int16,
    5: np.uint32,
    6: np.int32,
    7: np.float32,
    8: np.float64,
}


def decode_point_cloud(data: bytes) -> np.ndarray:
    """``foxglove.PointCloud`` to an (N, 3) float32 array of x, y, z."""
    msg = _proto(data)
    stride = int(msg.point_stride)
    buffer = bytes(msg.data)
    if stride == 0 or not buffer:
        return np.zeros((0, 3), dtype=np.float32)
    count = len(buffer) // stride
    columns = []
    for name in ("x", "y", "z"):
        matching = [f for f in msg.fields if f.name == name]
        if not matching:
            raise ValueError(f"PointCloud has no {name!r} field")
        f = matching[0]
        dtype = np.dtype(_POINT_FIELD_DTYPES[int(f.type)]).newbyteorder("<")
        view = np.ndarray(
            (count,), dtype=dtype, buffer=buffer, offset=int(f.offset), strides=(stride,)
        )
        columns.append(view.astype(np.float32))
    return np.stack(columns, axis=1)


# ---------------------------------------------------------------------------
# Scene updates (and legacy marker arrays mapped onto the same shape)
# ---------------------------------------------------------------------------


@dataclass
class Pose:
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)  # x, y, z, w


@dataclass
class Color:
    r: float = 0.0
    g: float = 0.0
    b: float = 0.0
    a: float = 1.0


@dataclass
class LinePrimitive:
    type: str  # LINE_STRIP, LINE_LOOP, LINE_LIST
    pose: Pose
    thickness: float
    scale_invariant: bool
    points: list[tuple[float, float, float]]
    color: Color


@dataclass
class CubePrimitive:
    pose: Pose
    size: tuple[float, float, float]
    color: Color


@dataclass
class SpherePrimitive:
    pose: Pose
    size: tuple[float, float, float]
    color: Color


@dataclass
class ArrowPrimitive:
    pose: Pose
    shaft_length: float
    shaft_diameter: float
    head_length: float
    head_diameter: float
    color: Color


@dataclass
class TextPrimitive:
    pose: Pose
    billboard: bool
    font_size: float
    scale_invariant: bool
    color: Color
    text: str


@dataclass
class SceneEntity:
    id: str
    frame_id: str
    stamp_ns: int
    lifetime_ns: int = 0
    frame_locked: bool = False
    lines: list[LinePrimitive] = field(default_factory=list)
    cubes: list[CubePrimitive] = field(default_factory=list)
    spheres: list[SpherePrimitive] = field(default_factory=list)
    arrows: list[ArrowPrimitive] = field(default_factory=list)
    texts: list[TextPrimitive] = field(default_factory=list)

    @property
    def namespace(self) -> str:
        """The legacy marker ``ns`` (everything before the last ``/`` in the id)."""
        return self.id.rsplit("/", 1)[0] if "/" in self.id else self.id

    @property
    def index(self) -> int:
        """The legacy marker ``id`` (the integer after the last ``/`` in the id), or -1."""
        tail = self.id.rsplit("/", 1)[-1] if "/" in self.id else ""
        try:
            return int(tail)
        except ValueError:
            return -1


@dataclass
class SceneEntityDeletion:
    type: str  # MATCHING_ID or ALL
    id: str
    stamp_ns: int


@dataclass
class SceneUpdate:
    entities: list[SceneEntity] = field(default_factory=list)
    deletions: list[SceneEntityDeletion] = field(default_factory=list)


_LINE_TYPES = {0: "LINE_STRIP", 1: "LINE_LOOP", 2: "LINE_LIST"}
_DELETION_TYPES = {0: "MATCHING_ID", 1: "ALL"}


def _proto_pose(pose: Any) -> Pose:
    return Pose(
        position=(pose.position.x, pose.position.y, pose.position.z),
        orientation=(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
    )


def _proto_color(color: Any) -> Color:
    return Color(r=color.r, g=color.g, b=color.b, a=color.a)


def _proto_entity(e: Any) -> SceneEntity:
    entity = SceneEntity(
        id=str(e.id),
        frame_id=str(e.frame_id),
        stamp_ns=_proto_stamp_ns(e.timestamp),
        lifetime_ns=int(e.lifetime.seconds) * _NS + int(e.lifetime.nanos),
        frame_locked=bool(e.frame_locked),
    )
    for line in e.lines:
        entity.lines.append(
            LinePrimitive(
                type=_LINE_TYPES.get(int(line.type), "LINE_STRIP"),
                pose=_proto_pose(line.pose),
                thickness=float(line.thickness),
                scale_invariant=bool(line.scale_invariant),
                points=[(p.x, p.y, p.z) for p in line.points],
                color=_proto_color(line.color),
            )
        )
    for cube in e.cubes:
        entity.cubes.append(
            CubePrimitive(
                pose=_proto_pose(cube.pose),
                size=(cube.size.x, cube.size.y, cube.size.z),
                color=_proto_color(cube.color),
            )
        )
    for sphere in e.spheres:
        entity.spheres.append(
            SpherePrimitive(
                pose=_proto_pose(sphere.pose),
                size=(sphere.size.x, sphere.size.y, sphere.size.z),
                color=_proto_color(sphere.color),
            )
        )
    for arrow in e.arrows:
        entity.arrows.append(
            ArrowPrimitive(
                pose=_proto_pose(arrow.pose),
                shaft_length=float(arrow.shaft_length),
                shaft_diameter=float(arrow.shaft_diameter),
                head_length=float(arrow.head_length),
                head_diameter=float(arrow.head_diameter),
                color=_proto_color(arrow.color),
            )
        )
    for text in e.texts:
        entity.texts.append(
            TextPrimitive(
                pose=_proto_pose(text.pose),
                billboard=bool(text.billboard),
                font_size=float(text.font_size),
                scale_invariant=bool(text.scale_invariant),
                color=_proto_color(text.color),
                text=str(text.text),
            )
        )
    return entity


def decode_scene_update(data: bytes) -> SceneUpdate:
    """``foxglove.SceneUpdate`` to entities and deletions.

    Legacy ``visualization_msgs/MarkerArray`` messages are mapped onto the same shape with the
    marker-to-entity rules from ``docs/foxglove_recording_format.md`` (``POINTS`` markers are
    skipped; use ``decode_marker_array`` to reach them).
    """
    if encoding_of(data) == ENCODING_ROS1:
        update = SceneUpdate()
        for marker in _ros1_decode_marker_array(data):
            mapped = marker_to_scene_entity(marker)
            if isinstance(mapped, SceneEntity):
                update.entities.append(mapped)
            elif isinstance(mapped, SceneEntityDeletion):
                update.deletions.append(mapped)
        return update

    msg = _proto(data)
    update = SceneUpdate()
    for e in msg.entities:
        update.entities.append(_proto_entity(e))
    for d in msg.deletions:
        update.deletions.append(
            SceneEntityDeletion(
                type=_DELETION_TYPES.get(int(d.type), "MATCHING_ID"),
                id=str(d.id),
                stamp_ns=_proto_stamp_ns(d.timestamp),
            )
        )
    return update


# ---------------------------------------------------------------------------
# Logs
# ---------------------------------------------------------------------------


@dataclass
class LogMessage:
    stamp_ns: int
    level: str  # DEBUG, INFO, WARN, ERROR, FATAL
    name: str
    message: str
    file: str
    line: int


_FOXGLOVE_LOG_LEVELS = {0: "UNKNOWN", 1: "DEBUG", 2: "INFO", 3: "WARN", 4: "ERROR", 5: "FATAL"}
_ROS1_LOG_LEVELS = {1: "DEBUG", 2: "INFO", 4: "WARN", 8: "ERROR", 16: "FATAL"}


def decode_log(data: bytes) -> LogMessage:
    """``foxglove.Log`` (or legacy ``rosgraph_msgs/Log``)."""
    if encoding_of(data) == ENCODING_ROS1:
        return _ros1_decode_log(data)
    msg = _proto(data)
    return LogMessage(
        stamp_ns=_proto_stamp_ns(msg.timestamp),
        level=_FOXGLOVE_LOG_LEVELS.get(int(msg.level), "UNKNOWN"),
        name=str(msg.name),
        message=str(msg.message),
        file=str(msg.file),
        line=int(msg.line),
    )


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


# ===========================================================================
# Legacy ROS 1 wire format
#
# Everything below decodes the recordings the C++ stack wrote before the Foxglove migration.
# ``scripts/convert_ros1_mcap.py`` is the last consumer that needs it on purpose; once the corpus
# is converted this whole section goes.
#
# All little-endian. Header: uint32 seq, uint32 stamp_secs, uint32 stamp_nsecs, string frame_id.
# string: uint32 length + raw bytes (no null terminator).
# ===========================================================================


@dataclass
class Marker:
    """One ``visualization_msgs/Marker``, as the legacy recordings hold it."""

    stamp_ns: int
    frame_id: str
    ns: str
    id: int
    type: int
    action: int
    pose: Pose
    scale: tuple[float, float, float]
    color: Color
    lifetime_ns: int
    frame_locked: bool
    points: list[tuple[float, float, float]]
    colors: list[Color]
    text: str


MARKER_ARROW = 0
MARKER_CUBE = 1
MARKER_SPHERE = 2
MARKER_LINE_STRIP = 4
MARKER_LINE_LIST = 5
MARKER_POINTS = 8
MARKER_TEXT_VIEW_FACING = 9
MARKER_ACTION_ADD = 0
MARKER_ACTION_DELETE = 2
MARKER_ACTION_DELETEALL = 3


def _rotation_x_onto(direction: np.ndarray) -> tuple[float, float, float, float]:
    """Quaternion (x, y, z, w) rotating +x onto ``direction``."""
    norm = float(np.linalg.norm(direction))
    if norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    d = direction / norm
    x_axis = np.array([1.0, 0.0, 0.0])
    dot = float(np.dot(x_axis, d))
    if dot < -1.0 + 1e-9:
        return (0.0, 0.0, 1.0, 0.0)  # 180 degrees about z
    axis = np.cross(x_axis, d)
    w = 1.0 + dot
    q = np.array([axis[0], axis[1], axis[2], w])
    q /= np.linalg.norm(q)
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def marker_to_scene_entity(marker: Marker) -> SceneEntity | SceneEntityDeletion | None:
    """The marker-to-entity rules from ``docs/foxglove_recording_format.md``.

    Returns None for ``POINTS`` markers, which have no SceneEntity form.
    """
    entity_id = f"{marker.ns}/{marker.id}"
    if marker.action == MARKER_ACTION_DELETEALL:
        return SceneEntityDeletion(type="ALL", id="", stamp_ns=marker.stamp_ns)
    if marker.action == MARKER_ACTION_DELETE:
        return SceneEntityDeletion(type="MATCHING_ID", id=entity_id, stamp_ns=marker.stamp_ns)
    if marker.type == MARKER_POINTS:
        return None

    entity = SceneEntity(
        id=entity_id,
        frame_id=marker.frame_id,
        stamp_ns=marker.stamp_ns,
        lifetime_ns=marker.lifetime_ns,
        frame_locked=marker.frame_locked,
    )
    sx, sy, sz = marker.scale
    if marker.type in (MARKER_LINE_STRIP, MARKER_LINE_LIST):
        entity.lines.append(
            LinePrimitive(
                type="LINE_STRIP" if marker.type == MARKER_LINE_STRIP else "LINE_LIST",
                pose=marker.pose,
                thickness=sx,
                scale_invariant=False,
                points=list(marker.points),
                color=marker.color,
            )
        )
    elif marker.type == MARKER_CUBE:
        entity.cubes.append(CubePrimitive(pose=marker.pose, size=marker.scale, color=marker.color))
    elif marker.type == MARKER_SPHERE:
        entity.spheres.append(
            SpherePrimitive(pose=marker.pose, size=marker.scale, color=marker.color)
        )
    elif marker.type == MARKER_ARROW:
        if len(marker.points) >= 2:
            start = np.asarray(marker.points[0], dtype=np.float64)
            end = np.asarray(marker.points[1], dtype=np.float64)
            length = float(np.linalg.norm(end - start))
            head_length = sz if sz > 0.0 else 0.23 * length
            entity.arrows.append(
                ArrowPrimitive(
                    pose=Pose(
                        position=(float(start[0]), float(start[1]), float(start[2])),
                        orientation=_rotation_x_onto(end - start),
                    ),
                    shaft_length=max(length - head_length, 0.0),
                    shaft_diameter=sx,
                    head_length=head_length,
                    head_diameter=sy,
                    color=marker.color,
                )
            )
        else:
            entity.arrows.append(
                ArrowPrimitive(
                    pose=marker.pose,
                    shaft_length=0.77 * sx,
                    shaft_diameter=sy,
                    head_length=0.23 * sx,
                    head_diameter=sz,
                    color=marker.color,
                )
            )
    elif marker.type == MARKER_TEXT_VIEW_FACING:
        entity.texts.append(
            TextPrimitive(
                pose=marker.pose,
                billboard=True,
                font_size=sz,
                scale_invariant=False,
                color=marker.color,
                text=marker.text,
            )
        )
    else:
        raise ValueError(f"Unsupported legacy marker type {marker.type} in ns {marker.ns!r}")
    return entity


def decode_marker_array(data: bytes) -> list[Marker]:
    """Every marker of a legacy ``visualization_msgs/MarkerArray`` message."""
    if encoding_of(data) != ENCODING_ROS1:
        raise ValueError("decode_marker_array only reads legacy ros1 MarkerArray messages")
    return _ros1_decode_marker_array(data)


def _ros1_read_string(data: bytes, offset: int) -> tuple[str, int]:
    (length,) = struct.unpack_from("<I", data, offset)
    offset += 4
    s = data[offset : offset + length].decode("utf-8", errors="replace")
    return s, offset + length


def _ros1_read_uint32(data: bytes, offset: int) -> tuple[int, int]:
    (v,) = struct.unpack_from("<I", data, offset)
    return v, offset + 4


def _ros1_read_int8(data: bytes, offset: int) -> tuple[int, int]:
    (v,) = struct.unpack_from("<b", data, offset)
    return v, offset + 1


def _ros1_read_header(data: bytes, offset: int) -> tuple[int, str, int]:
    """Read a std_msgs/Header; returns (stamp_ns, frame_id, new_offset)."""
    _seq, offset = _ros1_read_uint32(data, offset)
    secs, offset = _ros1_read_uint32(data, offset)
    nsecs, offset = _ros1_read_uint32(data, offset)
    frame_id, offset = _ros1_read_string(data, offset)
    return secs * _NS + nsecs, frame_id, offset


def _ros1_decode_string(data: bytes) -> str:
    s, _ = _ros1_read_string(data, 0)
    return s


def _ros1_decode_diagnostic_array(data: bytes) -> list[dict]:
    off = 0
    _stamp_ns, _frame_id, off = _ros1_read_header(data, off)
    status_count, off = _ros1_read_uint32(data, off)
    statuses = []
    for _ in range(status_count):
        level, off = _ros1_read_int8(data, off)
        name, off = _ros1_read_string(data, off)
        message, off = _ros1_read_string(data, off)
        hardware_id, off = _ros1_read_string(data, off)
        values_count, off = _ros1_read_uint32(data, off)
        values: dict[str, str] = {}
        for _ in range(values_count):
            key, off = _ros1_read_string(data, off)
            value, off = _ros1_read_string(data, off)
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


def _ros1_decode_compressed_image_bytes(data: bytes) -> tuple[int, str, str, bytes]:
    stamp_ns, frame_id, off = _ros1_read_header(data, 0)
    fmt, off = _ros1_read_string(data, off)
    length, off = _ros1_read_uint32(data, off)
    return stamp_ns, frame_id, fmt, bytes(data[off : off + length])


def _ros1_decode_raw_image_bytes(data: bytes) -> tuple[int, str, np.ndarray, str]:
    stamp_ns, frame_id, off = _ros1_read_header(data, 0)
    height, off = _ros1_read_uint32(data, off)
    width, off = _ros1_read_uint32(data, off)
    encoding, off = _ros1_read_string(data, off)
    off += 1  # is_bigendian
    step, off = _ros1_read_uint32(data, off)
    length, off = _ros1_read_uint32(data, off)
    channels = max(step // max(width, 1), 1)
    frame = np.frombuffer(data, dtype=np.uint8, count=length, offset=off).reshape(height, step)
    return (
        stamp_ns,
        frame_id,
        frame[:, : width * channels].reshape(height, width, channels),
        encoding,
    )


def _ros1_decode_camera_info(data: bytes) -> CameraInfo:
    stamp_ns, frame_id, off = _ros1_read_header(data, 0)
    height, off = _ros1_read_uint32(data, off)
    width, off = _ros1_read_uint32(data, off)
    distortion_model, off = _ros1_read_string(data, off)

    distortion_count, off = _ros1_read_uint32(data, off)
    distortion = np.frombuffer(data, dtype="<f8", count=distortion_count, offset=off)
    off += 8 * distortion_count

    intrinsics = np.frombuffer(data, dtype="<f8", count=9, offset=off).reshape(3, 3)
    off += 72
    rectification = np.frombuffer(data, dtype="<f8", count=9, offset=off).reshape(3, 3)
    off += 72
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
        rectification=rectification.copy(),
    )


def _ros1_decode_tf_message(data: bytes) -> list[Transform]:
    count, off = _ros1_read_uint32(data, 0)
    transforms = []
    for _ in range(count):
        stamp_ns, parent_frame_id, off = _ros1_read_header(data, off)
        child_frame_id, off = _ros1_read_string(data, off)
        tx, ty, tz, qx, qy, qz, qw = struct.unpack_from("<7d", data, off)
        off += 56
        transforms.append(
            Transform(
                stamp_ns=stamp_ns,
                parent_frame_id=parent_frame_id,
                child_frame_id=child_frame_id,
                matrix=_transform_matrix(tx, ty, tz, qx, qy, qz, qw),
            )
        )
    return transforms


def _ros1_decode_tf_raw(data: bytes) -> list[tuple[int, str, str, tuple[float, ...]]]:
    """(stamp_ns, parent, child, (tx, ty, tz, qx, qy, qz, qw)) per transform, no matrix math."""
    count, off = _ros1_read_uint32(data, 0)
    out = []
    for _ in range(count):
        stamp_ns, parent_frame_id, off = _ros1_read_header(data, off)
        child_frame_id, off = _ros1_read_string(data, off)
        values = struct.unpack_from("<7d", data, off)
        off += 56
        out.append((stamp_ns, parent_frame_id, child_frame_id, values))
    return out


def _ros1_decode_marker_array(data: bytes) -> list[Marker]:
    count, off = _ros1_read_uint32(data, 0)
    markers = []
    for _ in range(count):
        stamp_ns, frame_id, off = _ros1_read_header(data, off)
        ns, off = _ros1_read_string(data, off)
        marker_id, marker_type, action = struct.unpack_from("<iii", data, off)
        off += 12
        px, py, pz, qx, qy, qz, qw = struct.unpack_from("<7d", data, off)
        off += 56
        sx, sy, sz = struct.unpack_from("<3d", data, off)
        off += 24
        cr, cg, cb, ca = struct.unpack_from("<4f", data, off)
        off += 16
        life_sec, life_nsec = struct.unpack_from("<ii", data, off)
        off += 8
        frame_locked = data[off] != 0
        off += 1
        point_count, off = _ros1_read_uint32(data, off)
        points_arr = np.frombuffer(data, dtype="<f8", count=3 * point_count, offset=off)
        off += 24 * point_count
        color_count, off = _ros1_read_uint32(data, off)
        colors_arr = np.frombuffer(data, dtype="<f4", count=4 * color_count, offset=off)
        off += 16 * color_count
        text, off = _ros1_read_string(data, off)
        _mesh_resource, off = _ros1_read_string(data, off)
        off += 1  # mesh_use_embedded_materials
        markers.append(
            Marker(
                stamp_ns=stamp_ns,
                frame_id=frame_id,
                ns=ns,
                id=marker_id,
                type=marker_type,
                action=action,
                pose=Pose(position=(px, py, pz), orientation=(qx, qy, qz, qw)),
                scale=(sx, sy, sz),
                color=Color(r=cr, g=cg, b=cb, a=ca),
                lifetime_ns=life_sec * _NS + life_nsec,
                frame_locked=frame_locked,
                points=[tuple(p) for p in points_arr.reshape(-1, 3).tolist()],
                colors=[Color(*c) for c in colors_arr.reshape(-1, 4).tolist()],
                text=text,
            )
        )
    return markers


def _ros1_decode_log(data: bytes) -> LogMessage:
    stamp_ns, _frame_id, off = _ros1_read_header(data, 0)
    level, off = _ros1_read_int8(data, off)
    name, off = _ros1_read_string(data, off)
    message, off = _ros1_read_string(data, off)
    file, off = _ros1_read_string(data, off)
    _function, off = _ros1_read_string(data, off)
    line, off = _ros1_read_uint32(data, off)
    return LogMessage(
        stamp_ns=stamp_ns,
        level=_ROS1_LOG_LEVELS.get(level & 0xFF, "UNKNOWN"),
        name=name,
        message=message,
        file=file,
        line=line,
    )
