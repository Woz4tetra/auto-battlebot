"""Write recordings in the layout of ``docs/foxglove_recording_format.md`` from Python.

A thin wrapper over the Foxglove SDK writer. ``McapWriter`` opens one file in its own logging
context (so the same topic can be written to several files in one process), writes the
``auto_battlebot`` metadata record, and hands out channels by topic. Every ``log_*`` method takes
an explicit ``log_time_ns``; nothing here reads the wall clock.

The JSON schemas below are the canonical texts. ``include/foxglove_adapters/json_schemas.hpp``
holds the same bytes for the C++ writer.
"""

from __future__ import annotations

import json
import math
import struct
from pathlib import Path
from typing import Any, Callable, Sequence

import foxglove
import foxglove.channels as fg_channels
import foxglove.messages as fg
import numpy as np
from foxglove.mcap import MCAPCompression, MCAPWriteOptions

from auto_battlebot import mcap_io

_NS = 1_000_000_000

FRAME_META_SCHEMA_NAME = "auto_battlebot.FrameMeta"
DETECTIONS_SCHEMA_NAME = "auto_battlebot.Detections"
DIAGNOSTICS_SCHEMA_NAME = "auto_battlebot.Diagnostics"

FRAME_META_JSON_SCHEMA = """{"type":"object","title":"auto_battlebot.FrameMeta","properties":{"image_stamp_ns":{"type":"string","description":"Raw camera image stamp in nanoseconds as a decimal string; above 2^53 so not a JSON number"},"svo_frame_index":{"type":"integer","description":"Frame index within svo_path, -1 when SVO recording is off"},"svo_path":{"type":"string","description":"Active SVO file, empty when SVO recording is off"}},"required":["image_stamp_ns","svo_frame_index","svo_path"]}"""  # noqa: E501

DETECTIONS_JSON_SCHEMA = """{"type":"object","title":"auto_battlebot.Detections","properties":{"stamp":{"type":"number","description":"Frame stamp in seconds"},"w":{"type":"integer","description":"Image width in pixels"},"h":{"type":"integer","description":"Image height in pixels"},"dets":{"type":"array","items":{"type":"object","properties":{"x1":{"type":"number"},"y1":{"type":"number"},"x2":{"type":"number"},"y2":{"type":"number"},"conf":{"type":"number"},"class_id":{"type":"integer"},"label":{"type":"string"},"kps":{"type":"array","description":"Keypoints as [x, y, confidence] in image pixels","items":{"type":"array","items":{"type":"number"},"minItems":3,"maxItems":3}}},"required":["x1","y1","x2","y2","conf","class_id","label"]}}},"required":["stamp","w","h","dets"]}"""  # noqa: E501

DIAGNOSTICS_JSON_SCHEMA = """{"type":"object","title":"auto_battlebot.Diagnostics","description":"One key per subsection of a diagnostics module; the empty subsection is keyed by the module name","additionalProperties":{"type":"object","properties":{"level":{"type":"integer","description":"0 OK, 1 WARN, 2 ERROR, 3 STALE"},"message":{"type":"string"},"values":{"type":"object","additionalProperties":{"type":["number","string","null"]}}},"required":["level","message","values"]}}"""  # noqa: E501

# Topics whose payloads are JSON, with their schema. Everything else is a Foxglove protobuf.
JSON_TOPIC_SCHEMAS: dict[str, tuple[str, str]] = {
    mcap_io.FRAME_META_TOPIC: (FRAME_META_SCHEMA_NAME, FRAME_META_JSON_SCHEMA),
    mcap_io.BLOB_DETECTIONS_TOPIC: (DETECTIONS_SCHEMA_NAME, DETECTIONS_JSON_SCHEMA),
    mcap_io.KEYPOINT_DETECTIONS_TOPIC: (DETECTIONS_SCHEMA_NAME, DETECTIONS_JSON_SCHEMA),
}

LATCHED_TOPICS = frozenset(
    {
        "/field_mask",
        "/field_mask/camera_info",
        "/field_markers",
        "/field_points",
        "/hazard_markers",
        "/robot_markers",
        "/nav_markers",
    }
)


# Chunk compression names as they appear in an MCAP summary's chunk indexes.
_COMPRESSION = {"": None, "none": None, "zstd": MCAPCompression.Zstd, "lz4": MCAPCompression.Lz4}


def chunk_compression(name: str | None) -> Any:
    """Writer option for a chunk compression name (``none``, ``zstd``, ``lz4``)."""
    key = (name or "").lower()
    if key not in _COMPRESSION:
        raise ValueError(f"unknown chunk compression {name!r}")
    return _COMPRESSION[key]


def read_chunk_compression(path: Path | str) -> str:
    """The chunk compression a file was written with (``""`` for none), from its summary."""
    from mcap.reader import make_reader

    with open(path, "rb") as handle:
        summary = make_reader(handle).get_summary()
    if summary is None or not summary.chunk_indexes:
        return ""
    return str(summary.chunk_indexes[0].compression or "")


# ---------------------------------------------------------------------------
# Value conversions
# ---------------------------------------------------------------------------


def timestamp_from_ns(stamp_ns: int) -> fg.Timestamp:
    sec, nsec = divmod(int(stamp_ns), _NS)
    return fg.Timestamp(sec=sec, nsec=nsec)


def timestamp_from_seconds(stamp: float) -> fg.Timestamp:
    """``Header.stamp`` (double seconds) to ``Timestamp``, the same split the C++ writer uses."""
    return timestamp_from_ns(mcap_io.stamp_to_ns(stamp))


def duration_from_ns(duration_ns: int) -> fg.Duration | None:
    if duration_ns <= 0:
        return None
    sec, nsec = divmod(int(duration_ns), _NS)
    return fg.Duration(sec=sec, nsec=nsec)


def vector3(v: Sequence[float]) -> fg.Vector3:
    return fg.Vector3(x=float(v[0]), y=float(v[1]), z=float(v[2]))


def point3(p: Sequence[float]) -> fg.Point3:
    return fg.Point3(x=float(p[0]), y=float(p[1]), z=float(p[2]))


def quaternion(q: Sequence[float]) -> fg.Quaternion:
    """(x, y, z, w) to ``Quaternion``."""
    return fg.Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))


def pose(p: mcap_io.Pose) -> fg.Pose:
    return fg.Pose(position=vector3(p.position), orientation=quaternion(p.orientation))


def identity_pose() -> fg.Pose:
    return pose(mcap_io.Pose())


def color(c: mcap_io.Color) -> fg.Color:
    return fg.Color(r=float(c.r), g=float(c.g), b=float(c.b), a=float(c.a))


def rotation_to_quaternion(r: np.ndarray) -> tuple[float, float, float, float]:
    """3x3 rotation matrix to (x, y, z, w)."""
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
    return (float(x), float(y), float(z), float(w))


def frame_transform(
    stamp_ns: int, parent: str, child: str, matrix: np.ndarray
) -> fg.FrameTransform:
    """A 4x4 parent-from-child matrix to ``FrameTransform``."""
    m = np.asarray(matrix, dtype=np.float64)
    return fg.FrameTransform(
        timestamp=timestamp_from_ns(stamp_ns),
        parent_frame_id=parent,
        child_frame_id=child,
        translation=vector3(m[:3, 3].tolist()),
        rotation=quaternion(rotation_to_quaternion(m[:3, :3])),
    )


def frame_transform_from_parts(
    stamp_ns: int,
    parent: str,
    child: str,
    translation: Sequence[float],
    rotation_xyzw: Sequence[float],
) -> fg.FrameTransform:
    return fg.FrameTransform(
        timestamp=timestamp_from_ns(stamp_ns),
        parent_frame_id=parent,
        child_frame_id=child,
        translation=vector3(translation),
        rotation=quaternion(rotation_xyzw),
    )


def camera_calibration(
    stamp_ns: int,
    frame_id: str,
    width: int,
    height: int,
    k: Sequence[float] | np.ndarray,
    d: Sequence[float] | np.ndarray,
    distortion_model: str = "plumb_bob",
    r: Sequence[float] | np.ndarray | None = None,
    p: Sequence[float] | np.ndarray | None = None,
) -> fg.CameraCalibration:
    k9 = [float(v) for v in np.asarray(k, dtype=np.float64).ravel().tolist()]
    d_list = [float(v) for v in np.asarray(d, dtype=np.float64).ravel().tolist()]
    if r is None:
        r = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    if p is None:
        p = (k9[0], k9[1], k9[2], 0.0, k9[3], k9[4], k9[5], 0.0, k9[6], k9[7], k9[8], 0.0)
    return fg.CameraCalibration(
        timestamp=timestamp_from_ns(stamp_ns),
        frame_id=frame_id,
        width=int(width),
        height=int(height),
        distortion_model=distortion_model if d_list else "",
        D=d_list,
        K=k9,
        R=[float(v) for v in np.asarray(r, dtype=np.float64).ravel().tolist()],
        P=[float(v) for v in np.asarray(p, dtype=np.float64).ravel().tolist()],
    )


def compressed_image(
    stamp_ns: int, frame_id: str, data: bytes, fmt: str = "jpeg"
) -> fg.CompressedImage:
    return fg.CompressedImage(
        timestamp=timestamp_from_ns(stamp_ns), frame_id=frame_id, data=bytes(data), format=fmt
    )


def raw_image(
    stamp_ns: int, frame_id: str, frame: np.ndarray, encoding: str = "bgr8"
) -> fg.RawImage:
    """An uncompressed image (``bgr8`` by default) as ``foxglove.RawImage``."""
    frame = np.ascontiguousarray(frame)
    height, width = frame.shape[:2]
    return fg.RawImage(
        timestamp=timestamp_from_ns(stamp_ns),
        frame_id=frame_id,
        width=int(width),
        height=int(height),
        encoding=encoding,
        step=int(frame.strides[0]),
        data=frame.tobytes(),
    )


def point_cloud_xyz(stamp_ns: int, frame_id: str, points: np.ndarray) -> fg.PointCloud:
    """``/field_points``: float32 xyz, 12-byte stride, no color."""
    xyz = np.ascontiguousarray(np.asarray(points, dtype=np.float32).reshape(-1, 3))
    return fg.PointCloud(
        timestamp=timestamp_from_ns(stamp_ns),
        frame_id=frame_id,
        pose=identity_pose(),
        point_stride=12,
        fields=[
            fg.PackedElementField(
                name="x", offset=0, type=fg.PackedElementFieldNumericType.Float32
            ),
            fg.PackedElementField(
                name="y", offset=4, type=fg.PackedElementFieldNumericType.Float32
            ),
            fg.PackedElementField(
                name="z", offset=8, type=fg.PackedElementFieldNumericType.Float32
            ),
        ],
        data=xyz.astype("<f4").tobytes(),
    )


_LINE_TYPES = {
    "LINE_STRIP": fg.LinePrimitiveLineType.LineStrip,
    "LINE_LOOP": fg.LinePrimitiveLineType.LineLoop,
    "LINE_LIST": fg.LinePrimitiveLineType.LineList,
}

_LOG_LEVELS = {
    "UNKNOWN": fg.LogLevel.Unknown,
    "DEBUG": fg.LogLevel.Debug,
    "INFO": fg.LogLevel.Info,
    "WARN": fg.LogLevel.Warning,
    "WARNING": fg.LogLevel.Warning,
    "ERROR": fg.LogLevel.Error,
    "FATAL": fg.LogLevel.Fatal,
}


def scene_entity(entity: mcap_io.SceneEntity) -> fg.SceneEntity:
    return fg.SceneEntity(
        timestamp=timestamp_from_ns(entity.stamp_ns),
        frame_id=entity.frame_id,
        id=entity.id,
        lifetime=duration_from_ns(entity.lifetime_ns),
        frame_locked=entity.frame_locked,
        lines=[
            fg.LinePrimitive(
                type=_LINE_TYPES[line.type],
                pose=pose(line.pose),
                thickness=float(line.thickness),
                scale_invariant=bool(line.scale_invariant),
                points=[point3(p) for p in line.points],
                color=color(line.color),
            )
            for line in entity.lines
        ],
        cubes=[
            fg.CubePrimitive(pose=pose(c.pose), size=vector3(c.size), color=color(c.color))
            for c in entity.cubes
        ],
        spheres=[
            fg.SpherePrimitive(pose=pose(s.pose), size=vector3(s.size), color=color(s.color))
            for s in entity.spheres
        ],
        arrows=[
            fg.ArrowPrimitive(
                pose=pose(a.pose),
                shaft_length=float(a.shaft_length),
                shaft_diameter=float(a.shaft_diameter),
                head_length=float(a.head_length),
                head_diameter=float(a.head_diameter),
                color=color(a.color),
            )
            for a in entity.arrows
        ],
        texts=[
            fg.TextPrimitive(
                pose=pose(t.pose),
                billboard=bool(t.billboard),
                font_size=float(t.font_size),
                scale_invariant=bool(t.scale_invariant),
                color=color(t.color),
                text=t.text,
            )
            for t in entity.texts
        ],
    )


def scene_entity_deletion(deletion: mcap_io.SceneEntityDeletion) -> fg.SceneEntityDeletion:
    return fg.SceneEntityDeletion(
        timestamp=timestamp_from_ns(deletion.stamp_ns),
        type=(
            fg.SceneEntityDeletionType.All
            if deletion.type == "ALL"
            else fg.SceneEntityDeletionType.MatchingId
        ),
        id=deletion.id,
    )


def scene_update(update: mcap_io.SceneUpdate) -> fg.SceneUpdate:
    return fg.SceneUpdate(
        deletions=[scene_entity_deletion(d) for d in update.deletions],
        entities=[scene_entity(e) for e in update.entities],
    )



def log_message(entry: mcap_io.LogMessage) -> fg.Log:
    return fg.Log(
        timestamp=timestamp_from_ns(entry.stamp_ns),
        level=_LOG_LEVELS.get(entry.level, fg.LogLevel.Unknown),
        message=entry.message,
        name=entry.name,
        file=entry.file,
        line=int(entry.line),
    )


def json_number(value: Any) -> Any:
    """JSON-safe scalar: NaN and infinities become null, numpy scalars become Python ones."""
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (float, np.floating)):
        f = float(value)
        return None if math.isnan(f) or math.isinf(f) else f
    return value


# ---------------------------------------------------------------------------
# Writer
# ---------------------------------------------------------------------------

_PROTOBUF_CHANNEL_CLASSES: dict[str, Callable[..., Any]] = {
    "foxglove.CompressedImage": fg_channels.CompressedImageChannel,
    "foxglove.CameraCalibration": fg_channels.CameraCalibrationChannel,
    "foxglove.FrameTransforms": fg_channels.FrameTransformsChannel,
    "foxglove.SceneUpdate": fg_channels.SceneUpdateChannel,
    "foxglove.PointCloud": fg_channels.PointCloudChannel,
    "foxglove.Log": fg_channels.LogChannel,
    "foxglove.ImageAnnotations": fg_channels.ImageAnnotationsChannel,
    "foxglove.RawImage": fg_channels.RawImageChannel,
}

_MESSAGE_SCHEMA_NAMES: dict[type, str] = {
    fg.CompressedImage: "foxglove.CompressedImage",
    fg.CameraCalibration: "foxglove.CameraCalibration",
    fg.FrameTransforms: "foxglove.FrameTransforms",
    fg.SceneUpdate: "foxglove.SceneUpdate",
    fg.PointCloud: "foxglove.PointCloud",
    fg.Log: "foxglove.Log",
    fg.ImageAnnotations: "foxglove.ImageAnnotations",
    fg.RawImage: "foxglove.RawImage",
}


class McapWriter:
    """One output file. Channels are created on first use, keyed by topic."""

    def __init__(
        self,
        path: Path | str,
        active_profile: str | None = None,
        *,
        allow_overwrite: bool = False,
        compression: str | None = None,
        metadata: dict[str, dict[str, str]] | None = None,
    ) -> None:
        """``compression`` is the chunk compression: ``None``/``"none"``, ``"zstd"`` or ``"lz4"``.
        The C++ recorder writes none (the contract default); JPEG-heavy derived files use zstd."""
        self.path = Path(path)
        self._context = foxglove.Context()
        options = MCAPWriteOptions(compression=chunk_compression(compression), profile="")
        self._writer = foxglove.open_mcap(
            str(self.path),
            allow_overwrite=allow_overwrite,
            context=self._context,
            writer_options=options,
        )
        if active_profile is not None:
            self._writer.write_metadata("auto_battlebot", {"active_profile": active_profile})
        for name, values in (metadata or {}).items():
            self._writer.write_metadata(name, dict(values))
        self._channels: dict[str, Any] = {}
        self._closed = False

    def __enter__(self) -> McapWriter:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def write_metadata(self, name: str, values: dict[str, str]) -> None:
        self._writer.write_metadata(name, dict(values))

    def protobuf_channel(self, topic: str, schema_name: str) -> Any:
        channel = self._channels.get(topic)
        if channel is None:
            channel = _PROTOBUF_CHANNEL_CLASSES[schema_name](topic, context=self._context)
            self._channels[topic] = channel
        return channel

    def json_channel(self, topic: str, schema_name: str, json_schema: str) -> Any:
        channel = self._channels.get(topic)
        if channel is None:
            channel = foxglove.Channel(
                topic,
                message_encoding="json",
                schema=foxglove.Schema(
                    name=schema_name, encoding="jsonschema", data=json_schema.encode("utf-8")
                ),
                context=self._context,
            )
            self._channels[topic] = channel
        return channel

    def raw_channel(
        self,
        topic: str,
        message_encoding: str,
        schema_name: str | None,
        schema_encoding: str | None,
        schema_data: bytes | None,
    ) -> Any:
        """A channel that takes already-encoded bytes, for copying messages between files."""
        channel = self._channels.get(topic)
        if channel is None:
            schema = None
            if schema_name:
                schema = foxglove.Schema(
                    name=schema_name, encoding=schema_encoding or "", data=schema_data or b""
                )
            channel = foxglove.Channel(
                topic, message_encoding=message_encoding, schema=schema, context=self._context
            )
            self._channels[topic] = channel
        return channel

    def log_raw(
        self,
        topic: str,
        message_encoding: str,
        schema_name: str | None,
        schema_encoding: str | None,
        schema_data: bytes | None,
        data: bytes,
        log_time_ns: int,
    ) -> None:
        """Copy one already-encoded message through."""
        self.raw_channel(topic, message_encoding, schema_name, schema_encoding, schema_data).log(
            bytes(data), log_time=int(log_time_ns)
        )

    def log(self, topic: str, message: Any, log_time_ns: int) -> None:
        """Log a Foxglove message object on ``topic``."""
        schema_name = _MESSAGE_SCHEMA_NAMES[type(message)]
        self.protobuf_channel(topic, schema_name).log(message, log_time=int(log_time_ns))

    def log_json(
        self,
        topic: str,
        payload: dict[str, Any] | bytes | str,
        log_time_ns: int,
        schema: tuple[str, str] | None = None,
    ) -> None:
        """Log a JSON payload. ``schema`` defaults to the contract schema for known topics."""
        if schema is None:
            schema = JSON_TOPIC_SCHEMAS[topic]
        channel = self.json_channel(topic, schema[0], schema[1])
        if isinstance(payload, str):
            payload = payload.encode("utf-8")
        if isinstance(payload, dict):
            payload = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        channel.log(payload, log_time=int(log_time_ns))

    def log_diagnostics(self, module: str, sections: dict[str, Any], log_time_ns: int) -> None:
        self.log_json(
            f"{mcap_io.DIAGNOSTICS_TOPIC_PREFIX}{module}",
            sections,
            log_time_ns,
            schema=(DIAGNOSTICS_SCHEMA_NAME, DIAGNOSTICS_JSON_SCHEMA),
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        for channel in self._channels.values():
            channel.close()
        self._writer.close()


def pack_xyz_float32(points: np.ndarray) -> bytes:
    """Helper for tests and tools that build point cloud payloads by hand."""
    xyz = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    return b"".join(struct.pack("<3f", *row) for row in xyz.tolist())
