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
import sys
import time
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from mcap.reader import make_reader

from auto_battlebot import mcap_io, mcap_write
from auto_battlebot.diag_io import coerce_value

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
    markers = mcap_io.decode_marker_array(data)
    scene = [m for m in markers if m.type != mcap_io.MARKER_POINTS]
    clouds = [m for m in markers if m.type == mcap_io.MARKER_POINTS]
    writer.log(topic, mcap_write.scene_update_from_markers(scene), log_time)
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
    for status in mcap_io.decode_diagnostic_array(data):
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
    payload = json.loads(mcap_io.decode_string(data))
    before = int(payload["image_stamp_ns"])
    payload["image_stamp_ns"] = str(before)
    payload.setdefault("svo_frame_index", -1)
    payload.setdefault("svo_path", "")
    payload["svo_frame_index"] = int(payload["svo_frame_index"])
    payload["svo_path"] = str(payload["svo_path"])
    if int(payload["image_stamp_ns"]) != before:
        raise ConversionError(f"{topic}: image_stamp_ns changed value on re-encode")
    writer.log_json(topic, payload, log_time)


def _convert_string(writer: mcap_write.McapWriter, topic: str, data: bytes, log_time: int) -> None:
    text = mcap_io.decode_string(data)
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
    info = mcap_io.decode_camera_info(data)
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
        for stamp_ns, parent, child, values in mcap_io._ros1_decode_tf_raw(data)
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
    tagged = mcap_io.MessageBytes(data, mcap_io.ENCODING_ROS1, schema_name, topic)

    if schema_name == "visualization_msgs/MarkerArray":
        if topic == mcap_io.FIELD_MARKERS_TOPIC:
            _convert_field_markers(writer, topic, tagged, log_time, stats)
            return
        markers = mcap_io.decode_marker_array(tagged)
        if any(m.type == mcap_io.MARKER_POINTS for m in markers):
            raise ConversionError(f"{topic}: POINTS marker outside /field_markers")
        writer.log(topic, mcap_write.scene_update_from_markers(markers), log_time)
    elif schema_name == "diagnostic_msgs/DiagnosticArray":
        _convert_diagnostics(writer, tagged, log_time, stats)
        return
    elif schema_name == "sensor_msgs/CompressedImage":
        stamp_ns, frame_id, fmt, payload = mcap_io.decode_compressed_image_bytes(tagged)
        writer.log(topic, mcap_write.compressed_image(stamp_ns, frame_id, payload, fmt), log_time)
    elif schema_name == "sensor_msgs/Image":
        writer.log(topic, mcap_write.raw_image(*mcap_io.decode_raw_image_bytes(tagged)), log_time)
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
        writer.log(topic, mcap_write.log_message(mcap_io.decode_log(tagged)), log_time)
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
        # combine_mcap_svo outputs write zstd, and rewriting those uncompressed would grow them.
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
        count += sum(
            1 for m in mcap_io.decode_marker_array(data) if m.type == mcap_io.MARKER_POINTS
        )
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
        if topic == mcap_io.LEGACY_LOG_TOPIC:
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


if __name__ == "__main__":
    sys.exit(main())
