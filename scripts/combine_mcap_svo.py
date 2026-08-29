#!/usr/bin/env python3
"""Combine an MCAP recording with the SVO2 files it references.

For every SVO2 file an input MCAP references (in /rosout or /camera/frame_meta),
produce one combined MCAP <input_mcap_stem>__<svo_stem>.mcap that contains:

  * One cropped left-eye image per SVO frame, decoded straight out of the
    .svo2 (which is itself an MCAP container of H.264 frames).
  * The slice of the original MCAP messages whose log_time falls inside that
    SVO's [start, end] time range (intersect mode).

Requires `ffmpeg` on PATH and the Python `mcap` package. Neither the ZED SDK
nor `pyzed` is used.

The SVO's IMU data is dropped. It lives on the `sensors` /
`integrated_sensors` topics as an opaque base64 blob that only the SDK
decodes, and nothing downstream of these combined MCAPs reads it: the
model_eval and model_compare tools use /camera/image and /camera/camera_info
only. Use `ZED_SVO_Editor -export-to-mcap` if you ever need those channels.
"""

from __future__ import annotations

import argparse
import bisect
import heapq
import json
import logging
import re
import shutil
import struct
import sys
from contextlib import closing
from pathlib import Path
from typing import Any, Iterator, List, Optional, Tuple

from mcap.exceptions import McapError
from mcap.reader import make_reader
from mcap.writer import Writer
from tqdm import tqdm

from auto_battlebot import svo2

logger = logging.getLogger("combine_mcap_svo")

# The SVO stores a horizontally concatenated [left | right] H.264 frame. We
# decode it, crop to the left half, attach a proper ROS1 header, and publish
# as sensor_msgs/CompressedImage on TARGET_LEFT_IMAGE_TOPIC.
TARGET_LEFT_IMAGE_TOPIC = "/camera/image"
# One message per frame, alongside the image it describes, carrying where that image came
# from. It is the export-side companion to /camera/frame_meta, which the pipeline itself
# writes with the SVO frame index straight from the SDK.
#
# How well stamps alone identify a frame depends on when the recording was made. On the
# 2026-08-23 and 2026-08-28 Jetson recordings the pipeline's stamps equal the SVO frame
# stamp exactly (0.00 ms over 9521 frames, both /camera/frame_meta.image_stamp_ns and
# /camera/camera_info), so stamp and index joins agree. On the 2026-05-02 NHRL recordings
# the pipeline stamp lands 0.45 to 0.79 of the way through the SVO frame interval, so
# nearest-timestamp matching silently costs up to a frame there; join on the index, or on
# "pipeline stamp within [svo_stamp, next_svo_stamp)".
SVO_FRAME_TOPIC = "/camera/svo_frame"
# Topic in the original MCAP whose header.frame_id we copy onto the cropped
# image messages so all camera-frame data lines up.
CAMERA_INFO_TOPIC = "/camera/camera_info"
# Written per frame by the pipeline with the SVO file and index it came from.
FRAME_META_TOPIC = "/camera/frame_meta"

# ros1msg schema for sensor_msgs/CompressedImage. Foxglove handles this
# natively for image overlays / projections.
SENSOR_MSGS_COMPRESSED_IMAGE_SCHEMA = (
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

# ros1msg schema for std_msgs/String, matching how the pipeline publishes JSON payloads.
STD_MSGS_STRING_SCHEMA = b"string data\n"

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_SEARCH_DIRS = [
    PROJECT_ROOT / "data" / "svo",
    PROJECT_ROOT / "data" / "temp_svo",
]

# spdlog::info("Resolved SVO path: {}", ...) and "SVO recording started: {}".
# Path is plain ASCII; the next ROS1 string field's length prefix is
# non-printable, so a greedy \S+ capture is naturally bounded.
SVO_PATH_REGEX = re.compile(rb"(?:Resolved SVO path|SVO recording started):\s+(\S+\.svo2)")


def _frame_meta_svo_path(data: bytes) -> Optional[Path]:
    """Return the svo_path from a /camera/frame_meta std_msgs/String payload."""
    if len(data) < 4:
        return None
    (length,) = struct.unpack_from("<I", data, 0)
    try:
        meta = json.loads(data[4 : 4 + length])
    except (json.JSONDecodeError, UnicodeDecodeError):
        return None
    raw = meta.get("svo_path")
    if not raw:  # live-camera frames carry an empty path
        return None
    return Path(raw)


def extract_svo_paths(mcap_path: Path) -> List[Path]:
    """Return the SVO paths this recording used, in first-seen order.

    Both /rosout and /camera/frame_meta are read, because neither is complete
    on its own. /rosout logs the SVO the recorder opened but not the ones it
    rotates to mid-run: auto_battlebot_..._2026-08-28_20-57-33 logs only
    2026-08-28T20-57-35.svo2 and never mentions the 2026-08-28T21-03-09.svo2
    that holds all but the first 123 of its frames. /camera/frame_meta names
    the file per frame, so it catches the rotations, but only recordings from
    2026-08 on publish it.
    """
    seen: "dict[Path, None]" = {}
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for _schema, channel, message in reader.iter_messages(
            topics=["/rosout", FRAME_META_TOPIC],
            log_time_order=True,
        ):
            if channel.topic == FRAME_META_TOPIC:
                path = _frame_meta_svo_path(message.data)
                if path is not None:
                    seen.setdefault(path, None)
                continue
            for match in SVO_PATH_REGEX.finditer(message.data):
                seen.setdefault(Path(match.group(1).decode("utf-8", errors="ignore")), None)
    return list(seen.keys())


def _has_data(path: Path) -> bool:
    """True when `path` is a file with content.

    A recorder that opens an SVO and rotates away from it can leave a 0-byte
    file behind (data/svo/2026-08-28/2026-08-28T20-57-35.svo2). It looks like
    a hit but holds no MCAP magic, so it must not win the search.
    """
    if not path.is_file():
        return False
    if path.stat().st_size > 0:
        return True
    logger.warning("Ignoring empty SVO file: %s", path)
    return False


def resolve_svo_path(referenced: Path, search_dirs: List[Path]) -> Optional[Path]:
    """Locate the SVO file on disk, falling back to basename matches.

    Search dirs are walked recursively, because the recording logs the path
    the Jetson wrote to while the files are filed here into per-date
    subdirectories (data/svo/2026-08-23/...). Directories are tried in order,
    each one shallow-first, so an earlier search dir always wins.
    """
    if referenced.is_absolute():
        if _has_data(referenced):
            return referenced.resolve()
    else:
        relative = (PROJECT_ROOT / referenced).resolve()
        if _has_data(relative):
            return relative

    for directory in search_dirs:
        candidate = directory / referenced.name
        if _has_data(candidate):
            return candidate.resolve()

        matches = sorted(path for path in directory.rglob(referenced.name) if _has_data(path))
        if matches:
            if len(matches) > 1:
                logger.warning(
                    "%d files named %s under %s; using %s",
                    len(matches),
                    referenced.name,
                    directory,
                    matches[0],
                )
            return matches[0].resolve()
    return None


def read_svo_time_range(svo_path: Path, topic: str) -> Tuple[int, int]:
    """Return (start_ns, end_ns) spanned by the SVO's frames.

    Taken from the frame timestamps, never from the MCAP summary statistics:
    svo_header and svo_footer carry log_times unrelated to the recording's own
    timeline, which stretches the summary range by months.
    """
    stamps = svo2.read_frame_stamps(svo_path, topic)
    if not stamps:
        raise RuntimeError(f"SVO has no frames on {topic}: {svo_path}")
    return min(stamps), max(stamps)


def _iter_with_source(
    mcap_path: Path,
    source_idx: int,
    *,
    topics: Optional[List[str]] = None,
    start_time: Optional[int] = None,
    end_time: Optional[int] = None,
) -> Iterator[Tuple[int, int, Any, Any, Any]]:
    """Yield (log_time, source_idx, schema, channel, message) for heapq.merge."""
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for schema, channel, message in reader.iter_messages(
            topics=topics,
            start_time=start_time,
            end_time=end_time,
            log_time_order=True,
        ):
            yield (message.log_time, source_idx, schema, channel, message)


def _make_ros1_compressed_image(jpeg: bytes, stamp_ns: int, frame_id: Optional[str]) -> bytes:
    """Wrap a JPEG as a ROS1 sensor_msgs/CompressedImage.

    The std_msgs/Header is properly formed so Foxglove can do TF-aware image
    annotations. `stamp_ns` is the SVO frame's own timestamp.
    """
    sec = stamp_ns // 1_000_000_000
    nsec = stamp_ns - sec * 1_000_000_000

    fid = (frame_id or "").encode("utf-8")
    fmt_bytes = b"jpeg"

    parts = [
        struct.pack("<I", 0),  # header.seq
        struct.pack("<II", sec, nsec),  # header.stamp
        struct.pack("<I", len(fid)) + fid,  # header.frame_id
        struct.pack("<I", len(fmt_bytes)) + fmt_bytes,  # format
        struct.pack("<I", len(jpeg)) + jpeg,  # data
    ]
    return b"".join(parts)


def _make_ros1_string(payload: str) -> bytes:
    """Serialize a std_msgs/String: a length-prefixed UTF-8 blob."""
    encoded = payload.encode("utf-8")
    return struct.pack("<I", len(encoded)) + encoded


def _make_svo_frame_message(svo_name: str, frame_index: int, stamp_ns: int) -> bytes:
    """Describe one frame: which SVO it came from, its index, and its timestamp.

    `svo_frame_index` counts frames from zero in file order, which is the index the SDK reports
    as the SVO position, because every frame in the recording is read and emitted. `svo_stamp_ns`
    matches the image header stamp exactly. /camera/frame_meta from the pipeline carries the same
    index and is the cross-check.
    """
    payload = json.dumps(
        {"svo_file": svo_name, "svo_frame_index": frame_index, "svo_stamp_ns": stamp_ns},
        separators=(",", ":"),
    )
    return _make_ros1_string(payload)


def read_first_header_frame_id(mcap_path: Path, topic: str) -> Optional[str]:
    """Return the std_msgs/Header.frame_id of the first message on `topic`.

    Assumes the message starts with a ROS1-serialized std_msgs/Header
    (uint32 seq + time stamp + string frame_id). All ROS1 messages whose
    schemas start with `Header header` have this layout.
    """
    header_fixed_bytes = 12  # 4 (seq) + 4 (sec) + 4 (nsec)
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for _schema, _channel, message in reader.iter_messages(topics=[topic]):
            data = message.data
            if len(data) < header_fixed_bytes + 4:
                return None
            (length,) = struct.unpack_from("<I", data, header_fixed_bytes)
            end = header_fixed_bytes + 4 + length
            if length == 0 or end > len(data):
                return None
            return data[header_fixed_bytes + 4 : end].decode("utf-8", errors="replace")
    return None


def read_header_stamp_samples(
    mcap_path: Path, topic: str = CAMERA_INFO_TOPIC
) -> List[Tuple[int, int]]:
    """Return sorted list of (log_time_ns, header_stamp_ns) for `topic`.

    Used to map the original MCAP's wall-clock-stamped log_times back to the
    camera-frame-stamped timeline that the SVO MCAP uses, fixing the ~50 ms
    lag between capture and `mcap_recorder->write()` in
    src/publisher/ros_publisher.cpp.
    """
    samples: List[Tuple[int, int]] = []
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for _schema, _channel, message in reader.iter_messages(topics=[topic]):
            data = message.data
            # Layout: uint32 seq, uint32 sec, uint32 nsec, ...
            if len(data) < 12:
                continue
            sec, nsec = struct.unpack_from("<II", data, 4)
            stamp_ns = sec * 1_000_000_000 + nsec
            samples.append((message.log_time, stamp_ns))
    samples.sort()
    return samples


class HeaderStampRetimer:
    """Linear-interpolate a `log_time -> header.stamp` mapping from samples.

    For log_times outside the sample range, the boundary offset is
    extrapolated as a constant (i.e. the first/last (log_time - stamp)
    delta is reused). Returns the original log_time unchanged when no
    samples are available.
    """

    def __init__(self, samples: List[Tuple[int, int]]):
        self._log_times: List[int] = [s[0] for s in samples]
        self._stamps: List[int] = [s[1] for s in samples]

    def __bool__(self) -> bool:
        return bool(self._log_times)

    @property
    def sample_count(self) -> int:
        return len(self._log_times)

    @property
    def offset_stats_ns(self) -> Tuple[float, int]:
        """Return (avg, max_abs) of (log_time - stamp) across samples."""
        if not self._log_times:
            return 0.0, 0
        offsets = [lt - st for lt, st in zip(self._log_times, self._stamps)]
        avg = sum(offsets) / len(offsets)
        max_abs = max(abs(min(offsets)), abs(max(offsets)))
        return avg, max_abs

    def retime(self, log_time_ns: int) -> int:
        if not self._log_times:
            return log_time_ns
        idx = bisect.bisect_left(self._log_times, log_time_ns)
        if idx == 0:
            offset = self._log_times[0] - self._stamps[0]
            return log_time_ns - offset
        if idx >= len(self._log_times):
            offset = self._log_times[-1] - self._stamps[-1]
            return log_time_ns - offset
        a_lt, a_st = self._log_times[idx - 1], self._stamps[idx - 1]
        b_lt, b_st = self._log_times[idx], self._stamps[idx]
        if b_lt == a_lt:
            return a_st
        t = (log_time_ns - a_lt) / (b_lt - a_lt)
        return int(a_st + t * (b_st - a_st))


def _summary_message_count(mcap_path: Path) -> int:
    """Best-effort message count from an MCAP's summary statistics."""
    try:
        with open(mcap_path, "rb") as f:
            reader = make_reader(f)
            summary = reader.get_summary()
            if summary is not None and summary.statistics is not None:
                return int(summary.statistics.message_count)
    except Exception:
        pass
    return 0


def _log_frame_id_resolution(camera_frame_id: Optional[str], original_path: Path) -> None:
    """Log whether a camera frame_id was found for /camera/image."""
    if camera_frame_id is not None:
        logger.info(
            "Using camera frame_id %r from %s for /camera/image",
            camera_frame_id,
            CAMERA_INFO_TOPIC,
        )
    else:
        logger.warning(
            "Could not find frame_id on %s in %s; leaving SVO frame_id intact",
            CAMERA_INFO_TOPIC,
            original_path,
        )


def _log_retimer_stats(retimer: HeaderStampRetimer, original_path: Path) -> None:
    """Log retiming statistics derived from the original MCAP's header stamps."""
    if retimer:
        avg_offset_ns, max_abs_offset_ns = retimer.offset_stats_ns
        logger.info(
            "Retiming original MCAP via %d %s samples"
            " (avg log_time-stamp offset %.2f ms, max abs %.2f ms)",
            retimer.sample_count,
            CAMERA_INFO_TOPIC,
            avg_offset_ns / 1e6,
            max_abs_offset_ns / 1e6,
        )
    else:
        logger.warning(
            "No %s samples in %s; original log_times will not be retimed",
            CAMERA_INFO_TOPIC,
            original_path,
        )


def _load_retimed_originals(
    original_path: Path,
    retimer: HeaderStampRetimer,
    start_ns: int,
    end_ns: int,
) -> List[Tuple[int, int, object, object, object]]:
    """Load original messages, retime their log_times, and filter to [start, end].

    We over-fetch by max_abs_offset so we don't drop messages whose retimed time
    lands inside the window, then re-sort before the merge.
    """
    if retimer:
        margin_ns = retimer.offset_stats_ns[1]
    else:
        margin_ns = 0
    fetch_start = max(0, start_ns - margin_ns)
    fetch_end = end_ns + margin_ns

    original_buffer: List[Tuple[int, int, object, object, object]] = []
    with open(original_path, "rb") as f:
        reader = make_reader(f)
        for schema, channel, message in reader.iter_messages(
            start_time=fetch_start,
            end_time=fetch_end,
            log_time_order=True,
        ):
            new_log_time = retimer.retime(message.log_time)
            if start_ns <= new_log_time <= end_ns:
                original_buffer.append((new_log_time, 0, schema, channel, message))
    original_buffer.sort(key=lambda x: (x[0], x[1]))
    return original_buffer


def merge_mcaps(
    original_path: Path,
    svo_path: Path,
    svo_topic: str,
    frame_count: int,
    output_path: Path,
    time_range_ns: Tuple[int, int],
    profile: str,
    ffmpeg_bin: str,
) -> None:
    """Stream-merge `original` (sliced) and the SVO's frames into `output_path`."""
    start_ns, end_ns = time_range_ns
    output_path.parent.mkdir(parents=True, exist_ok=True)

    camera_frame_id = read_first_header_frame_id(original_path, CAMERA_INFO_TOPIC)
    _log_frame_id_resolution(camera_frame_id, original_path)

    retimer = HeaderStampRetimer(read_header_stamp_samples(original_path))
    _log_retimer_stats(retimer, original_path)

    # Total is an upper bound: the original count is pre-time-slice, so the
    # bar may finish slightly before 100% on recordings that extend past the
    # SVO's time range. tqdm handles that gracefully.
    total_messages = _summary_message_count(original_path) + frame_count

    # Per-source maps: source_schema_id -> writer_schema_id, same for channels.
    # Source idx 0 = original, 1 = svo.
    schema_remap: List["dict[int, int]"] = [{}, {}]
    channel_remap: List["dict[int, int]"] = [{}, {}]

    with open(output_path, "wb") as out_f:
        writer = Writer(out_f)
        writer.start(profile=profile, library="combine_mcap_svo")

        # Pre-register a single sensor_msgs/CompressedImage channel for the
        # cropped left frames so they get a proper ROS1 schema and header.
        left_image_schema_id = writer.register_schema(
            name="sensor_msgs/CompressedImage",
            encoding="ros1msg",
            data=SENSOR_MSGS_COMPRESSED_IMAGE_SCHEMA,
        )
        left_image_channel_id = writer.register_channel(
            topic=TARGET_LEFT_IMAGE_TOPIC,
            message_encoding="ros1",
            schema_id=left_image_schema_id,
        )

        svo_frame_schema_id = writer.register_schema(
            name="std_msgs/String",
            encoding="ros1msg",
            data=STD_MSGS_STRING_SCHEMA,
        )
        svo_frame_channel_id = writer.register_channel(
            topic=SVO_FRAME_TOPIC,
            message_encoding="ros1",
            schema_id=svo_frame_schema_id,
        )
        svo_name = svo_path.stem
        frame_index = 0

        def get_writer_channel_id(source_idx: int, schema: Any, channel: Any) -> int:
            cmap = channel_remap[source_idx]
            if channel.id in cmap:
                return cmap[channel.id]

            writer_schema_id = 0
            if schema is not None and schema.id != 0:
                smap = schema_remap[source_idx]
                if schema.id in smap:
                    writer_schema_id = smap[schema.id]
                else:
                    writer_schema_id = writer.register_schema(
                        name=schema.name,
                        encoding=schema.encoding,
                        data=schema.data,
                    )
                    smap[schema.id] = writer_schema_id

            writer_channel_id = writer.register_channel(
                topic=channel.topic,
                message_encoding=channel.message_encoding,
                schema_id=writer_schema_id,
                metadata=dict(channel.metadata or {}),
            )
            cmap[channel.id] = writer_channel_id
            return writer_channel_id

        # When retiming, we have to load the original messages, shift their
        # log_times to camera-frame time, filter to the SVO range, and re-sort
        # before the merge.
        original_buffer = _load_retimed_originals(original_path, retimer, start_ns, end_ns)

        original_iter = iter(original_buffer)
        svo_iter = _iter_with_source(svo_path, 1, topics=[svo_topic])

        # ffmpeg emits one JPEG per message we feed it, in order, so the Nth
        # image pairs with the Nth side_by_side message. closing() tears the
        # decoder down if the merge aborts, instead of leaving it blocked on a
        # full pipe.
        fps = svo2.sample_fps(svo_path, svo_topic)
        jpegs = svo2.iter_left_jpegs(svo_path, svo_topic, fps, ffmpeg_bin=ffmpeg_bin)

        with (
            closing(jpegs),
            tqdm(
                total=total_messages if total_messages > 0 else None,
                desc=f"Merge {output_path.name}",
                unit="msg",
                unit_scale=True,
                leave=False,
                dynamic_ncols=True,
            ) as merge_bar,
        ):
            for log_time, source_idx, schema, channel, message in heapq.merge(
                original_iter, svo_iter, key=lambda x: (x[0], x[1])
            ):
                if source_idx == 1:
                    jpeg = next(jpegs, None)
                    if jpeg is None:
                        raise RuntimeError(
                            f"ffmpeg produced fewer frames than the {frame_count} in "
                            f"{svo_path}: ran out at frame {frame_index}"
                        )
                    data = _make_ros1_compressed_image(jpeg, log_time, camera_frame_id)
                    writer.add_message(
                        channel_id=left_image_channel_id,
                        log_time=log_time,
                        data=data,
                        publish_time=log_time,
                        sequence=message.sequence,
                    )
                    writer.add_message(
                        channel_id=svo_frame_channel_id,
                        log_time=log_time,
                        data=_make_svo_frame_message(svo_name, frame_index, log_time),
                        publish_time=log_time,
                        sequence=message.sequence,
                    )
                    frame_index += 1
                    merge_bar.update(1)
                    continue

                writer_channel_id = get_writer_channel_id(source_idx, schema, channel)
                writer.add_message(
                    channel_id=writer_channel_id,
                    log_time=log_time,
                    data=message.data,
                    publish_time=log_time,
                    sequence=message.sequence,
                )
                merge_bar.update(1)

            if next(jpegs, None) is not None:
                raise RuntimeError(
                    f"ffmpeg produced more frames than the {frame_count} in {svo_path}"
                )

        writer.add_metadata(
            "svo_export",
            {"svo_file": svo_name, "frame_count": str(frame_index)},
        )
        logger.info("Wrote %d SVO frames to %s", frame_index, SVO_FRAME_TOPIC)

        writer.finish()


def get_input_profile(mcap_path: Path) -> str:
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        return reader.get_header().profile or ""


def process_mcap(
    input_mcap: Path,
    output_dir: Optional[Path],
    search_dirs: List[Path],
    *,
    overwrite: bool,
    dry_run: bool,
    ffmpeg_bin: str,
    ignore_missing_svo: bool,
) -> Tuple[int, int]:
    """Process one input MCAP.

    Returns (number of combined MCAPs written, number of referenced SVOs unusable).
    """
    if not input_mcap.exists():
        logger.error("Input MCAP not found: %s", input_mcap)
        return 0, 0

    referenced = extract_svo_paths(input_mcap)
    if not referenced:
        logger.warning("No SVO paths found in %s", input_mcap)
        return 0, 0

    out_dir = output_dir if output_dir is not None else input_mcap.parent
    profile = get_input_profile(input_mcap)
    written = 0
    missing = 0
    # Two references can name one file (/rosout and /camera/frame_meta spell the
    # same path, or two spellings resolve to the same file by basename), and a
    # second pass over it would just rewrite the same output.
    done: "dict[Path, None]" = {}

    svo_iter = tqdm(
        referenced,
        desc=f"SVOs in {input_mcap.name}",
        unit="svo",
        leave=False,
        dynamic_ncols=True,
        disable=len(referenced) <= 1 or dry_run,
    )
    for svo_ref in svo_iter:
        svo_path = resolve_svo_path(svo_ref, search_dirs)
        if svo_path is None:
            missing += 1
            log = logger.warning if ignore_missing_svo else logger.error
            log(
                "Referenced SVO not found on disk: %s (also searched by basename under %s)",
                svo_ref,
                ", ".join(str(d) for d in search_dirs),
            )
            continue

        if svo_path in done:
            logger.debug("Already handled %s; skipping duplicate reference", svo_path)
            continue
        done[svo_path] = None

        combined_name = f"{input_mcap.stem}__{svo_path.stem}.mcap"
        combined_path = out_dir / combined_name

        if combined_path.exists() and not overwrite:
            logger.info(
                "Combined output already exists, skipping (use --overwrite): %s",
                combined_path,
            )
            continue

        if dry_run:
            logger.info(
                "DRY-RUN: would combine %s + %s -> %s",
                input_mcap,
                svo_path,
                combined_path,
            )
            continue

        try:
            svo_topic, frame_count = svo2.find_side_by_side_topic(svo_path)
            start_ns, end_ns = read_svo_time_range(svo_path, svo_topic)
        except (McapError, RuntimeError, ValueError, OSError) as exc:
            # A truncated or half-written SVO reads as a broken MCAP.
            logger.error("Skipping unreadable SVO %s: %s", svo_path, exc)
            missing += 1
            continue
        logger.info(
            "SVO time range: [%d, %d] ns (%.3f s span, %d frames)",
            start_ns,
            end_ns,
            (end_ns - start_ns) / 1e9,
            frame_count,
        )

        merge_mcaps(
            input_mcap,
            svo_path,
            svo_topic,
            frame_count,
            combined_path,
            (start_ns, end_ns),
            profile,
            ffmpeg_bin,
        )
        logger.info("Wrote %s", combined_path)
        written += 1

    return written, missing


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Combine an MCAP recording with the SVO2 files it references "
            "(via /rosout). Produces one combined MCAP per SVO."
        )
    )
    parser.add_argument(
        "mcap_files",
        nargs="+",
        type=Path,
        help="Input MCAP files (e.g. data/recordings/auto_battlebot_*.mcap)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Directory for combined outputs. Defaults to each input MCAP's directory.",
    )
    parser.add_argument(
        "--svo-search-dir",
        action="append",
        type=Path,
        default=None,
        help=(
            "Additional directory to search recursively for SVOs by basename if the "
            "path logged in /rosout is missing. Repeatable. Defaults: data/svo, "
            "data/temp_svo (under the project root)."
        ),
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite existing combined MCAPs.",
    )
    parser.add_argument(
        "--ignore-missing-svo",
        action="store_true",
        help=(
            "Treat referenced SVOs that are missing or unreadable as a warning and keep "
            "going. "
            "By default they are an error and the exit code is nonzero."
        ),
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Just list referenced SVOs and the would-be outputs, no work.",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Enable debug logging.",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="[%(asctime)s] %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )

    ffmpeg_bin = shutil.which("ffmpeg")
    if ffmpeg_bin is None and not args.dry_run:
        logger.error("ffmpeg not found on PATH. Install ffmpeg or pass --dry-run.")
        return 2

    search_dirs = list(args.svo_search_dir) if args.svo_search_dir else list(DEFAULT_SEARCH_DIRS)

    total_written = 0
    total_missing = 0
    mcap_iter = tqdm(
        args.mcap_files,
        desc="Input MCAPs",
        unit="mcap",
        leave=False,
        dynamic_ncols=True,
        disable=len(args.mcap_files) <= 1 or args.dry_run,
    )
    for mcap_path in mcap_iter:
        logger.info("Processing %s", mcap_path)
        written, missing = process_mcap(
            mcap_path.resolve() if mcap_path.exists() else mcap_path,
            args.output_dir.resolve() if args.output_dir else None,
            search_dirs,
            overwrite=args.overwrite,
            dry_run=args.dry_run,
            ffmpeg_bin=ffmpeg_bin or "",
            ignore_missing_svo=args.ignore_missing_svo,
        )
        total_written += written
        total_missing += missing

    logger.info("Done. Wrote %d combined MCAP(s).", total_written)
    if total_missing:
        if args.ignore_missing_svo:
            logger.warning("Skipped %d unusable referenced SVO(s).", total_missing)
        else:
            logger.error(
                "%d referenced SVO(s) missing or unreadable. Pass --ignore-missing-svo to "
                "skip them.",
                total_missing,
            )
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
