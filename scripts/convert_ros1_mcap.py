#!/usr/bin/env python3
"""Convert a legacy ROS 1 recording into the Foxglove layout of docs/foxglove_recording_format.md.

    python scripts/convert_ros1_mcap.py <input.mcap> <output.mcap> [--check]
    python scripts/convert_ros1_mcap.py <input_dir> <output_dir> [--check]

Never writes in place and never deletes anything. With a directory pair every ``*.mcap`` under
the input tree is converted to the mirrored path under the output tree; outputs that already
exist are skipped unless ``--overwrite`` is given. Files that are not ``profile: ros1`` are
reported and skipped.

Payload bytes that are already the right thing (JPEGs, the detections JSON) pass through
untouched. ``log_time`` and ``publish_time`` are preserved, and the ``auto_battlebot`` metadata
record (with ``active_profile``) is copied. Four topics change shape on the way through, as the
C++ stack's own output did:

- ``/field_markers`` splits into the border ``SceneUpdate`` and a ``/field_points`` float32
  ``PointCloud``. Every coordinate is asserted to round-trip through float32 unchanged.
- ``/diagnostics`` splits into one JSON channel per module with typed values, using the same
  string-to-number rule ``diag_io`` applied to legacy recordings.
- ``/camera/frame_meta`` re-emits ``image_stamp_ns`` as a decimal string (asserted equal).
- ``/blob_detections`` and ``/keypoint_detections`` lose the ``std_msgs/String`` wrapper.

``--check`` re-reads input and output and compares per-topic message counts (accounting for
the splits) and first/last log times.
"""

from __future__ import annotations

import argparse
import json
import re
import struct
import sys
import time
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from mcap.reader import make_reader

from auto_battlebot.recording import mcap_io, mcap_write
from auto_battlebot.recording.mcap_io import (
    ArrowPrimitive,
    CameraInfo,
    Color,
    CubePrimitive,
    LinePrimitive,
    LogMessage,
    Pose,
    SceneEntity,
    SceneEntityDeletion,
    SpherePrimitive,
    TextPrimitive,
)

ENCODING_ROS1 = "ros1"
LEGACY_LOG_TOPIC = "/rosout"
_NS = 1_000_000_000

SVO_FRAME_TOPIC = "/camera/svo_frame"
SVO_FRAME_SCHEMA = (
    "auto_battlebot.SvoFrame",
    '{"type":"object","title":"auto_battlebot.SvoFrame","properties":{"svo_file":{"type":"string"},'
    '"svo_frame_index":{"type":"integer"},"svo_stamp_ns":{"type":"integer"}}}',
)
GENERIC_JSON_SCHEMA = ("auto_battlebot.Json", '{"type":"object"}')


class ConversionError(RuntimeError):
    pass


@dataclass
class Stats:
    messages_in: int = 0
    messages_out: int = 0
    field_points: int = 0
    topics_in: Counter = field(default_factory=Counter)
    topics_out: Counter = field(default_factory=Counter)
    skipped_schemas: Counter = field(default_factory=Counter)


# ---------------------------------------------------------------------------
# Per-message transforms
# ---------------------------------------------------------------------------


def _convert_field_markers(
    writer: mcap_write.McapWriter, topic: str, data: bytes, log_time: int, stats: Stats
) -> None:
    markers = _ros1_decode_marker_array(data)
    scene = [m for m in markers if m.type != MARKER_POINTS]
    clouds = [m for m in markers if m.type == MARKER_POINTS]
    writer.log(topic, _scene_update_from_markers(scene), log_time)
    stats.topics_out[topic] += 1
    stats.messages_out += 1
    for cloud in clouds:
        if cloud.ns != "field_inliers":
            raise ConversionError(f"{topic}: POINTS marker in unexpected ns {cloud.ns!r}")
        points64 = np.asarray(cloud.points, dtype=np.float64).reshape(-1, 3)
        points32 = points64.astype(np.float32)
        if not np.array_equal(points32.astype(np.float64), points64):
            worst = np.abs(points32.astype(np.float64) - points64).max()
            raise ConversionError(
                f"{topic}: {points64.shape[0]} inlier points do not round-trip through float32 "
                f"(max error {worst:g}); the source cloud was not float32"
            )
        writer.log(
            mcap_io.FIELD_POINTS_TOPIC,
            mcap_write.point_cloud_xyz(cloud.stamp_ns, cloud.frame_id, points32),
            log_time,
        )
        stats.topics_out[mcap_io.FIELD_POINTS_TOPIC] += 1
        stats.messages_out += 1
        stats.field_points += 1


def _convert_diagnostics(
    writer: mcap_write.McapWriter, data: bytes, log_time: int, stats: Stats
) -> None:
    by_module: dict[str, dict[str, Any]] = defaultdict(dict)
    for status in _ros1_decode_diagnostic_array(data):
        module = status["hardware_id"] or "unknown"
        name = status["name"] or module
        section = by_module[module].setdefault(
            name, {"level": int(status["level"]), "message": status["message"], "values": {}}
        )
        section["level"] = max(section["level"], int(status["level"]))
        if status["message"] and status["message"] not in section["message"]:
            section["message"] = (
                f"{section['message']}; {status['message']}"
                if section["message"]
                else status["message"]
            )
        for key, value in status["values"].items():
            section["values"][key] = mcap_write.json_number(coerce_value(value))
    for module, sections in by_module.items():
        writer.log_diagnostics(module, sections, log_time)
        stats.topics_out[f"{mcap_io.DIAGNOSTICS_TOPIC_PREFIX}{module}"] += 1
        stats.messages_out += 1


def _convert_frame_meta(
    writer: mcap_write.McapWriter, topic: str, data: bytes, log_time: int
) -> None:
    payload = json.loads(_ros1_decode_string(data))
    before = int(payload["image_stamp_ns"])
    payload["image_stamp_ns"] = str(before)
    # ROS1 bags predate both the SVO join and the video channel, so the index is -1 either way.
    index = payload.pop("svo_frame_index", payload.get("video_frame_index", -1))
    payload.pop("svo_path", None)
    payload["video_frame_index"] = int(index)
    if int(payload["image_stamp_ns"]) != before:
        raise ConversionError(f"{topic}: image_stamp_ns changed value on re-encode")
    writer.log_json(topic, payload, log_time)


def _convert_string(writer: mcap_write.McapWriter, topic: str, data: bytes, log_time: int) -> None:
    text = _ros1_decode_string(data)
    if topic in mcap_write.JSON_TOPIC_SCHEMAS:
        json.loads(text)  # the payload must already be the contract JSON
        writer.log_json(topic, text, log_time)
        return
    schema = SVO_FRAME_SCHEMA if topic == SVO_FRAME_TOPIC else GENERIC_JSON_SCHEMA
    try:
        json.loads(text)
        writer.log_json(topic, text, log_time, schema=schema)
    except json.JSONDecodeError:
        writer.log_json(topic, {"data": text}, log_time, schema=GENERIC_JSON_SCHEMA)


def _convert_camera_info(
    writer: mcap_write.McapWriter, topic: str, data: bytes, log_time: int
) -> None:
    info = _ros1_decode_camera_info(data)
    writer.log(
        topic,
        mcap_write.camera_calibration(
            info.stamp_ns,
            info.frame_id,
            info.width,
            info.height,
            info.intrinsics,
            info.distortion,
            distortion_model=info.distortion_model,
            r=info.rectification,
            p=info.projection,
        ),
        log_time,
    )


def _convert_tf(writer: mcap_write.McapWriter, topic: str, data: bytes, log_time: int) -> None:
    transforms = [
        mcap_write.frame_transform_from_parts(stamp_ns, parent, child, values[:3], values[3:])
        for stamp_ns, parent, child, values in _ros1_decode_tf_raw(data)
    ]
    writer.log(topic, mcap_write.fg.FrameTransforms(transforms=transforms), log_time)


def _convert_message(  # noqa: C901
    writer: mcap_write.McapWriter,
    schema_name: str,
    topic: str,
    data: bytes,
    log_time: int,
    stats: Stats,
) -> None:
    stats.messages_in += 1
    stats.topics_in[topic] += 1
    tagged = mcap_io.MessageBytes(data, ENCODING_ROS1, schema_name, topic)

    if schema_name == "visualization_msgs/MarkerArray":
        if topic == mcap_io.FIELD_MARKERS_TOPIC:
            _convert_field_markers(writer, topic, tagged, log_time, stats)
            return
        markers = _ros1_decode_marker_array(tagged)
        if any(m.type == MARKER_POINTS for m in markers):
            raise ConversionError(f"{topic}: POINTS marker outside /field_markers")
        writer.log(topic, _scene_update_from_markers(markers), log_time)
    elif schema_name == "diagnostic_msgs/DiagnosticArray":
        _convert_diagnostics(writer, tagged, log_time, stats)
        return
    elif schema_name == "sensor_msgs/CompressedImage":
        stamp_ns, frame_id, fmt, payload = _ros1_decode_compressed_image_bytes(tagged)
        writer.log(topic, mcap_write.compressed_image(stamp_ns, frame_id, payload, fmt), log_time)
    elif schema_name == "sensor_msgs/Image":
        writer.log(topic, mcap_write.raw_image(*_ros1_decode_raw_image_bytes(tagged)), log_time)
    elif schema_name == "sensor_msgs/CameraInfo":
        _convert_camera_info(writer, topic, tagged, log_time)
    elif schema_name == "tf2_msgs/TFMessage":
        _convert_tf(writer, topic, tagged, log_time)
    elif schema_name == "std_msgs/String":
        if topic == mcap_io.FRAME_META_TOPIC:
            _convert_frame_meta(writer, topic, tagged, log_time)
        else:
            _convert_string(writer, topic, tagged, log_time)
    elif schema_name == "rosgraph_msgs/Log":
        topic = mcap_io.LOG_TOPIC
        writer.log(topic, mcap_write.log_message(_ros1_decode_log(tagged)), log_time)
    else:
        raise ConversionError(f"{topic}: no conversion for schema {schema_name!r}")
    stats.topics_out[topic] += 1
    stats.messages_out += 1


# ---------------------------------------------------------------------------
# Files
# ---------------------------------------------------------------------------


def input_profile(path: Path) -> str:
    with open(path, "rb") as handle:
        return make_reader(handle).get_header().profile


def convert_file(src: Path, dst: Path, *, overwrite: bool = False) -> Stats:
    if dst.exists() and not overwrite:
        raise FileExistsError(dst)
    if src.resolve() == dst.resolve():
        raise ConversionError("input and output are the same file; never converts in place")
    dst.parent.mkdir(parents=True, exist_ok=True)
    stats = Stats()
    partial = dst.with_name(dst.name + ".partial")
    partial.unlink(missing_ok=True)
    with open(src, "rb") as handle:
        reader = make_reader(handle)
        metadata = {record.name: dict(record.metadata) for record in reader.iter_metadata()}
        # Keep the source's chunk compression: the C++ recorder writes none, the JPEG-heavy
        # Python-written captures use zstd chunks, and rewriting those uncompressed would grow them.
        compression = mcap_write.read_chunk_compression(src)
        with mcap_write.McapWriter(partial, compression=compression, metadata=metadata) as writer:
            for schema, channel, message in reader.iter_messages(log_time_order=True):
                if message.publish_time != message.log_time:
                    raise ConversionError(
                        f"{channel.topic}: publish_time differs from log_time, which the "
                        "Foxglove writer cannot reproduce"
                    )
                if channel.message_encoding != "ros1":
                    # Already a Foxglove-readable channel (the calibration recordings mix JSON
                    # channels with ros1 images): copy it through unchanged.
                    writer.log_raw(
                        channel.topic,
                        channel.message_encoding,
                        schema.name if schema else None,
                        schema.encoding if schema else None,
                        schema.data if schema else None,
                        message.data,
                        message.log_time,
                    )
                    stats.messages_in += 1
                    stats.messages_out += 1
                    stats.topics_in[channel.topic] += 1
                    stats.topics_out[channel.topic] += 1
                    continue
                if schema is None:
                    stats.skipped_schemas["<none>"] += 1
                    continue
                _convert_message(
                    writer, schema.name, channel.topic, message.data, message.log_time, stats
                )
    partial.replace(dst)
    return stats


@dataclass
class TopicSpan:
    count: int = 0
    first: int | None = None
    last: int | None = None

    def add(self, log_time: int) -> None:
        self.count += 1
        self.first = log_time if self.first is None else min(self.first, log_time)
        self.last = log_time if self.last is None else max(self.last, log_time)


def _spans(path: Path) -> dict[str, TopicSpan]:
    spans: dict[str, TopicSpan] = defaultdict(TopicSpan)
    with open(path, "rb") as handle:
        for _schema, channel, message in make_reader(handle).iter_messages():
            spans[channel.topic].add(message.log_time)
    return spans


def _expected_field_points(src: Path) -> int:
    count = 0
    for _topic, _ts, data in mcap_io.iter_messages(src, [mcap_io.FIELD_MARKERS_TOPIC]):
        count += sum(1 for m in _ros1_decode_marker_array(data) if m.type == MARKER_POINTS)
    return count


def check_conversion(src: Path, dst: Path) -> list[str]:  # noqa: C901
    """Compare per-topic counts and first/last log times. Returns a list of problems."""
    problems: list[str] = []
    src_spans = _spans(src)
    dst_spans = _spans(dst)

    def compare(name: str, expected: TopicSpan, actual: TopicSpan) -> None:
        if expected.count != actual.count:
            problems.append(f"{name}: {expected.count} messages in, {actual.count} out")
        if (expected.first, expected.last) != (actual.first, actual.last):
            problems.append(
                f"{name}: log time span {expected.first}..{expected.last} in, "
                f"{actual.first}..{actual.last} out"
            )

    for topic, span in src_spans.items():
        if topic == mcap_io.DIAGNOSTICS_TOPIC:
            merged = TopicSpan()
            for out_topic, out_span in dst_spans.items():
                if out_topic.startswith(mcap_io.DIAGNOSTICS_TOPIC_PREFIX):
                    merged.count = max(merged.count, out_span.count)
                    merged.first = (
                        out_span.first
                        if merged.first is None
                        else min(merged.first, out_span.first or merged.first)
                    )
                    merged.last = (
                        out_span.last
                        if merged.last is None
                        else max(merged.last, out_span.last or merged.last)
                    )
            # Per-module channels each carry at most one message per source array, so the
            # busiest module bounds the count from below and the union of spans must match.
            if merged.count > span.count:
                problems.append(
                    f"{topic}: {span.count} arrays in, {merged.count} on one module out"
                )
            if (merged.first, merged.last) != (span.first, span.last):
                problems.append(
                    f"{topic}: log time span {span.first}..{span.last} in, "
                    f"{merged.first}..{merged.last} out"
                )
            continue
        if topic == LEGACY_LOG_TOPIC:
            compare(topic, span, dst_spans.get(mcap_io.LOG_TOPIC, TopicSpan()))
            continue
        compare(topic, span, dst_spans.get(topic, TopicSpan()))

    if mcap_io.FIELD_MARKERS_TOPIC in src_spans:
        expected = _expected_field_points(src)
        actual = dst_spans.get(mcap_io.FIELD_POINTS_TOPIC, TopicSpan()).count
        if expected != actual:
            problems.append(f"/field_points: expected {expected} clouds, wrote {actual}")

    stray = set(dst_spans) - set(src_spans) - {mcap_io.FIELD_POINTS_TOPIC, mcap_io.LOG_TOPIC}
    stray = {t for t in stray if not t.startswith(mcap_io.DIAGNOSTICS_TOPIC_PREFIX)}
    if stray:
        problems.append(f"unexpected output topics: {sorted(stray)}")

    if mcap_io.read_active_profile(src) != mcap_io.read_active_profile(dst):
        problems.append("active_profile metadata differs")
    return problems


def _human(size: float) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if size < 1024 or unit == "GB":
            return f"{size:.1f} {unit}"
        size /= 1024
    return f"{size:.1f} GB"


def convert_one(src: Path, dst: Path, *, check: bool, overwrite: bool) -> tuple[str, str]:
    """Returns (status, detail). status is one of ok, skipped, failed."""
    if src.stat().st_size == 0:
        return "skipped", "empty file"
    try:
        profile = input_profile(src)
    except Exception as exc:  # noqa: BLE001 - a torn file is a per-file outcome, not a crash
        return "failed", f"unreadable header: {exc}"
    if profile != "ros1":
        return "skipped", f"profile {profile!r} is not ros1"
    if dst.exists() and not overwrite:
        return "skipped", "output exists"
    started = time.monotonic()
    try:
        stats = convert_file(src, dst, overwrite=overwrite)
    except Exception as exc:  # noqa: BLE001
        dst.with_name(dst.name + ".partial").unlink(missing_ok=True)
        return "failed", f"{type(exc).__name__}: {exc}"
    detail = (
        f"{mcap_write.read_chunk_compression(dst) or 'none'} chunks, "
        f"{stats.messages_in} msgs -> {stats.messages_out}, "
        f"{_human(src.stat().st_size)} -> {_human(dst.stat().st_size)} "
        f"({100.0 * dst.stat().st_size / max(src.stat().st_size, 1):.0f}%), "
        f"{time.monotonic() - started:.1f}s"
    )
    if stats.skipped_schemas:
        detail += f", skipped {dict(stats.skipped_schemas)}"
    if check:
        problems = check_conversion(src, dst)
        if problems:
            return "failed", detail + "; check: " + "; ".join(problems)
        detail += ", check ok"
    return "ok", detail


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("input", type=Path, help="legacy .mcap file or a directory of them")
    parser.add_argument("output", type=Path, help="output .mcap file or mirrored output directory")
    parser.add_argument("--check", action="store_true", help="re-read both files and compare")
    parser.add_argument("--overwrite", action="store_true", help="replace existing outputs")
    args = parser.parse_args()

    if args.input.is_dir():
        pairs = [
            (src, args.output / src.relative_to(args.input))
            for src in sorted(args.input.rglob("*.mcap"))
        ]
    else:
        pairs = [(args.input, args.output)]

    counts: Counter = Counter()
    bytes_in = bytes_out = 0
    for src, dst in pairs:
        status, detail = convert_one(src, dst, check=args.check, overwrite=args.overwrite)
        counts[status] += 1
        if status == "ok":
            bytes_in += src.stat().st_size
            bytes_out += dst.stat().st_size
        print(f"[{status}] {src} -> {dst}: {detail}", flush=True)

    print(
        f"\n{counts['ok']} converted, {counts['skipped']} skipped, {counts['failed']} failed; "
        f"{_human(bytes_in)} -> {_human(bytes_out)}"
    )
    return 1 if counts["failed"] else 0


# ===========================================================================
# Legacy ROS 1 wire format
#
# Everything below decodes the recordings the C++ stack wrote before the Foxglove migration.
# This script is the only consumer left; the readers in ``auto_battlebot`` refuse legacy input.
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
    if mcap_io.encoding_of(data) != ENCODING_ROS1:
        raise ValueError("decode_marker_array only reads legacy MarkerArray messages")
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


_ROS1_LOG_LEVELS = {1: "DEBUG", 2: "INFO", 4: "WARN", 8: "ERROR", 16: "FATAL"}


_INT_PATTERN = re.compile(r"^[+-]?\d+$")


def coerce_value(value: Any) -> Any:
    """The legacy string-to-number rule, one value at a time.

    Legacy recordings stringified every diagnostic value (``std::to_string``), so an integer
    became ``"1"`` and a double ``"12.500000"``. The C++ stack now emits typed values; this is
    the same judgement applied to old bytes: an integer literal becomes ``int``, anything
    ``float()`` accepts becomes ``float`` (``nan``/``inf`` included), and everything else stays
    a string. Already-typed values pass through untouched.
    """
    if not isinstance(value, str):
        return value


def _scene_update_from_markers(markers: list[Marker]) -> Any:
    """Legacy markers to a ``foxglove.SceneUpdate`` through the contract's marker rules."""
    update = mcap_io.SceneUpdate()
    for marker in markers:
        mapped = marker_to_scene_entity(marker)
        if isinstance(mapped, SceneEntity):
            update.entities.append(mapped)
        elif isinstance(mapped, SceneEntityDeletion):
            update.deletions.append(mapped)
    return mcap_write.scene_update(update)


if __name__ == "__main__":
    sys.exit(main())
