"""Shared MCAP reading utilities.

Recordings follow ``docs/foxglove_recording_format.md``: ``protobuf`` channels carrying Foxglove
schemas and ``json`` channels carrying the project's own payloads. The C++ stack writes them with
the Foxglove SDK; ``auto_battlebot.recording.mcap_write`` writes them from Python.

``iter_messages`` tags every payload it yields with the channel's message encoding, schema name
and topic, and every ``decode_*`` function dispatches on that tag. Recordings from before the
Foxglove migration are refused; ``scripts/convert_ros1_mcap.py`` rewrites them into this layout.
"""

from __future__ import annotations

import json
import math
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
CAMERA_VIDEO_TOPIC = "/camera/video"
CAMERA_INFO_TOPIC = "/camera/camera_info"
FRAME_META_TOPIC = "/camera/frame_meta"
TF_TOPIC = "/tf"
FIELD_MARKERS_TOPIC = "/field_markers"
FIELD_POINTS_TOPIC = "/field_points"
ROBOT_MARKERS_TOPIC = "/robot_markers"
LOG_TOPIC = "/log"

ENCODING_PROTOBUF = "protobuf"
ENCODING_JSON = "json"

_NS = 1_000_000_000


class MessageBytes(bytes):
    """Raw message payload tagged with how to decode it.

    ``iter_messages`` yields these; the tag is what lets one ``decode_*`` call serve both the
    protobuf and the JSON channels.
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
    return getattr(data, "encoding", "")


def _require_current_format(data: bytes) -> None:
    """Refuse payloads that are untagged or carry a pre-migration encoding."""
    encoding = encoding_of(data)
    if encoding in (ENCODING_PROTOBUF, ENCODING_JSON):
        return
    raise ValueError(
        f"Message encoding {encoding or 'untagged'!r} is not readable: only the layout in "
        "docs/foxglove_recording_format.md is supported. A recording from before the Foxglove "
        "migration must be rewritten with scripts/convert_ros1_mcap.py first."
    )


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

    ``/diagnostics`` selects every per-module ``/diagnostics/<module>`` channel, so callers keep
    one name.
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


def recording_topics(path: Path | str) -> set[str]:
    """Every topic the recording carries, from its summary."""
    with open(path, "rb") as file:
        summary = make_reader(file).get_summary()
        if summary is None:
            return set()
        return {channel.topic for channel in summary.channels.values()}


def message_counts(path: Path | str) -> dict[str, int]:
    """Messages per topic, from the summary. Does not read the message data."""
    with open(path, "rb") as file:
        summary = make_reader(file).get_summary()
        if summary is None or summary.statistics is None:
            return {}
        counts = summary.statistics.channel_message_counts
        return {
            channel.topic: int(counts.get(channel_id, 0))
            for channel_id, channel in summary.channels.items()
        }


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
    """The text of a JSON channel message."""
    _require_current_format(data)
    return bytes(data).decode("utf-8")


def decode_json(data: bytes) -> Any:
    return json.loads(decode_string(data))


@dataclass
class FrameMeta:
    image_stamp_ns: int
    video_frame_index: int


def decode_frame_meta(data: bytes) -> FrameMeta:
    """``/camera/frame_meta``. ``image_stamp_ns`` is a decimal string on the wire (it is above
    2^53); the legacy bare-number form is accepted too.

    Recordings written before video moved into the MCAP carry ``svo_frame_index`` and ``svo_path``
    instead. The index reads back as ``video_frame_index``, which is what it always was: the
    ordinal of the frame in whatever video stream the run was writing. The path is dropped, since
    with one file there is no second file to join to.
    """
    payload = decode_json(data)
    index = payload.get("video_frame_index", payload.get("svo_frame_index", -1))
    return FrameMeta(
        image_stamp_ns=int(payload["image_stamp_ns"]),
        video_frame_index=int(index),
    )


# ---------------------------------------------------------------------------
# Diagnostics
# ---------------------------------------------------------------------------


def decode_diagnostic_array(data: bytes) -> list[dict]:
    """One dict per diagnostic status: ``{level, name, message, hardware_id, values}``.

    For a ``/diagnostics/<module>`` JSON message ``hardware_id`` is the module (from the topic),
    ``name`` is the section key and ``values`` are typed.
    """
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
    _require_current_format(data)
    return _proto_stamp_ns(_proto(data).timestamp)


def decode_compressed_image_bytes(data: bytes) -> tuple[int, str, str, bytes]:
    """(stamp_ns, frame_id, format, encoded bytes) without decoding the image."""
    _require_current_format(data)
    msg = _proto(data)
    return _proto_stamp_ns(msg.timestamp), str(msg.frame_id), str(msg.format), bytes(msg.data)


def decode_raw_image_bytes(data: bytes) -> tuple[int, str, np.ndarray, str]:
    """(stamp_ns, frame_id, HxWxC uint8 array, encoding) of a ``foxglove.RawImage``. Only 8-bit
    encodings are supported."""
    _require_current_format(data)
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
# Compressed video
# ---------------------------------------------------------------------------


@dataclass
class CompressedVideo:
    stamp_ns: int
    frame_id: str
    format: str
    image: np.ndarray


def decode_compressed_video_bytes(data: bytes) -> tuple[int, str, str, bytes]:
    """(stamp_ns, frame_id, format, Annex-B access unit) without decoding the frame."""
    _require_current_format(data)
    msg = _proto(data)
    return _proto_stamp_ns(msg.timestamp), str(msg.frame_id), str(msg.format), bytes(msg.data)


class VideoStreamDecoder:
    """Decodes ``/camera/video`` into BGR frames.

    Stateful where ``cv2.imdecode`` was not: H.264 frames reference the ones before them, so the
    decoder holds one context per file and has to be fed in stream order. Every caller already
    iterates a whole recording in order. Seeking means starting from a keyframe, never from the
    middle of a GOP.

    The C++ encoder writes with ``max_b_frames = 0``, so decode order equals capture order and the
    nth frame out is the nth frame captured. ``export_camera_transforms.py`` matches dataset images
    by position in the stream and depends on that.
    """

    def __init__(self) -> None:
        import av

        self._codec = av.CodecContext.create("h264", "r")

    def decode(self, packet_bytes: bytes) -> list[np.ndarray]:
        """BGR frames produced by one access unit. Usually one; empty while the decoder primes."""
        import av

        packets = [av.packet.Packet(packet_bytes)]
        frames = []
        for packet in packets:
            for frame in self._codec.decode(packet):
                frames.append(frame.to_ndarray(format="bgr24"))
        return frames

    def flush(self) -> list[np.ndarray]:
        frames = []
        for frame in self._codec.decode(None):
            frames.append(frame.to_ndarray(format="bgr24"))
        return frames


def iter_video_frames(path: Path | str) -> Iterator[tuple[int, np.ndarray]]:
    """(stamp_ns, BGR frame) for every frame on ``/camera/video``, in capture order."""
    decoder = VideoStreamDecoder()
    for _topic, _log_time, payload in iter_messages(path, [CAMERA_VIDEO_TOPIC]):
        stamp_ns, _frame_id, _fmt, access_unit = decode_compressed_video_bytes(payload)
        for image in decoder.decode(access_unit):
            yield stamp_ns, image
    for image in decoder.flush():
        yield 0, image


def iter_camera_frames(
    path: Path | str, prefer_video: bool = True
) -> Iterator[tuple[int, np.ndarray]]:
    """Frames from whichever camera topic the recording carries.

    A label run records ``/camera/image`` because its config clears ``ignored_topics``; an ordinary
    match recording carries ``/camera/video`` and no JPEG at all. Tools that used to need
    ``scripts/combine_mcap_svo.py`` to reach match frames can point straight at the recording.
    """
    topics = recording_topics(path)
    if prefer_video and CAMERA_VIDEO_TOPIC in topics:
        yield from iter_video_frames(path)
        return
    if CAMERA_IMAGE_TOPIC not in topics:
        raise ValueError(
            f"{path} carries neither {CAMERA_VIDEO_TOPIC} nor {CAMERA_IMAGE_TOPIC}; there are no "
            "camera frames in it"
        )
    for _topic, _log_time, payload in iter_messages(path, [CAMERA_IMAGE_TOPIC]):
        image = decode_compressed_image(payload)
        yield image.stamp_ns, image.image


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
    _require_current_format(data)
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
    """Decode a ``foxglove.FrameTransforms`` message into 4x4 matrices."""
    _require_current_format(data)
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
    """``foxglove.SceneUpdate`` to entities and deletions."""
    _require_current_format(data)
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


def decode_log(data: bytes) -> LogMessage:
    """``foxglove.Log``."""
    _require_current_format(data)
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
