#!/usr/bin/env python3
"""Combine an MCAP recording with the SVO2 files it references.

For every SVO2 file referenced in an input MCAP's /rosout log, produce one
combined MCAP <input_mcap_stem>__<svo_stem>.mcap that contains:

  * All messages from `ZED_SVO_Editor -export-to-mcap <svo>`
    (image/imu/gnss/custom data).
  * The slice of the original MCAP messages whose log_time falls inside that
    SVO's [start, end] time range (intersect mode).

Requires `ZED_SVO_Editor` on PATH (part of the ZED SDK) and the Python `mcap`
package. `pyzed` is NOT used.
"""

from __future__ import annotations

import argparse
import base64
import bisect
import heapq
import json
import logging
import re
import shutil
import struct
import subprocess
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
from mcap.reader import make_reader
from mcap.writer import Writer
from tqdm import tqdm

logger = logging.getLogger("combine_mcap_svo")

# Format emitted by ZED_SVO_Editor -export-to-mcap, one line per frame.
EXPORT_FRAME_REGEX = re.compile(r"Export frame (\d+) on (\d+)")

# ZED_SVO_Editor emits a horizontally concatenated [left | right] image on a
# topic literally named "side_by_side" with a non-standard
# foxglove.CompressedImage JSON schema (timestamp is a bare int instead of
# {sec, nsec}, which breaks Foxglove TF lookups for image annotations).
# We crop to the left half, attach a proper ROS1 header, and re-publish as
# sensor_msgs/CompressedImage on TARGET_LEFT_IMAGE_TOPIC.
SVO_SIDE_BY_SIDE_TOPIC = "side_by_side"
TARGET_LEFT_IMAGE_TOPIC = "/camera/image"
# Topic in the original MCAP whose header.frame_id we copy onto the cropped
# image messages so all camera-frame data lines up.
CAMERA_INFO_TOPIC = "/camera/camera_info"

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

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_SEARCH_DIRS = [
    PROJECT_ROOT / "data" / "svo",
    PROJECT_ROOT / "data" / "temp_svo",
]

# spdlog::info("Resolved SVO path: {}", ...) and "SVO recording started: {}".
# Path is plain ASCII; the next ROS1 string field's length prefix is
# non-printable, so a greedy \S+ capture is naturally bounded.
SVO_PATH_REGEX = re.compile(
    rb"(?:Resolved SVO path|SVO recording started):\s+(\S+\.svo2)"
)


def extract_svo_paths(mcap_path: Path) -> List[Path]:
    """Return SVO paths referenced by /rosout messages, in first-seen order."""
    seen: "dict[Path, None]" = {}
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for _schema, _channel, message in reader.iter_messages(topics=["/rosout"]):
            for match in SVO_PATH_REGEX.finditer(message.data):
                path = Path(match.group(1).decode("utf-8", errors="ignore"))
                if path not in seen:
                    seen[path] = None
    return list(seen.keys())


def resolve_svo_path(referenced: Path, search_dirs: List[Path]) -> Optional[Path]:
    """Locate the SVO file on disk, falling back to basename matches."""
    if referenced.is_absolute():
        if referenced.exists():
            return referenced.resolve()
    else:
        relative = (PROJECT_ROOT / referenced).resolve()
        if relative.exists():
            return relative

    for directory in search_dirs:
        candidate = directory / referenced.name
        if candidate.exists():
            return candidate.resolve()
    return None


def convert_svo_to_mcap(
    svo_path: Path, intermediate_path: Path, editor_bin: str
) -> Path:
    """Run ZED_SVO_Editor -export-to-mcap and move output to intermediate_path.

    The tool is interactive: it prompts "Export SVO into readable MCAP [Y/n]"
    and "Enter file name [default: <svo_stem>.mcap]". Both have defaults, so
    we just feed two newlines on stdin to accept them. The default output
    location is next to the SVO file.
    """
    # The tool defaults to writing <svo_stem>.mcap next to the SVO. We rename
    # any pre-existing sidecar out of the way so we can unambiguously claim
    # the freshly-produced file.
    default_out = svo_path.with_suffix(".mcap")
    stash_path: Optional[Path] = None
    if default_out.exists():
        stash_path = default_out.with_suffix(".mcap.combine_stash")
        default_out.rename(stash_path)

    logger.info("Converting %s -> %s", svo_path, intermediate_path)
    try:
        captured: List[str] = []
        proc = subprocess.Popen(
            [editor_bin, "-export-to-mcap", str(svo_path)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        try:
            assert proc.stdin is not None and proc.stdout is not None
            proc.stdin.write("\n\n")
            proc.stdin.flush()
            proc.stdin.close()

            bar: Optional[tqdm] = None
            try:
                for raw_line in proc.stdout:
                    line = raw_line.rstrip()
                    captured.append(line)
                    match = EXPORT_FRAME_REGEX.search(line)
                    if match is None:
                        continue
                    current = int(match.group(1)) + 1  # frames done (1-indexed)
                    total = int(match.group(2))
                    if bar is None:
                        bar = tqdm(
                            total=total,
                            desc=f"Export {svo_path.name}",
                            unit="frame",
                            leave=False,
                            dynamic_ncols=True,
                        )
                    if total != bar.total:
                        bar.total = total
                        bar.refresh()
                    delta = current - bar.n
                    if delta > 0:
                        bar.update(delta)
            finally:
                if bar is not None:
                    bar.close()
            returncode = proc.wait()
        except BaseException:
            proc.kill()
            proc.wait()
            raise

        if returncode != 0:
            for line in captured:
                sys.stderr.write(line + "\n")
            raise RuntimeError(
                f"ZED_SVO_Editor failed (exit {returncode}) for {svo_path}"
            )

        if not default_out.exists():
            for line in captured:
                sys.stderr.write(line + "\n")
            raise RuntimeError(
                f"ZED_SVO_Editor did not produce {default_out} for {svo_path}"
            )

        intermediate_path.parent.mkdir(parents=True, exist_ok=True)
        if intermediate_path.exists():
            intermediate_path.unlink()
        shutil.move(str(default_out), str(intermediate_path))
    finally:
        if stash_path is not None and stash_path.exists():
            # Restore the user's original sidecar file if we stashed it.
            if default_out.exists():
                # Should not happen (we just moved it), but be defensive.
                stash_path.unlink()
            else:
                stash_path.rename(default_out)
    return intermediate_path


def read_time_range(mcap_path: Path) -> Tuple[int, int]:
    """Return (start_ns, end_ns) for an MCAP. Falls back to scanning."""
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        summary = reader.get_summary()
        if summary is not None and summary.statistics is not None:
            stats = summary.statistics
            if stats.message_count > 0:
                return stats.message_start_time, stats.message_end_time

        start = end = None
        f.seek(0)
        reader = make_reader(f)
        for _schema, _channel, message in reader.iter_messages():
            t = message.log_time
            if start is None or t < start:
                start = t
            if end is None or t > end:
                end = t
    if start is None or end is None:
        raise RuntimeError(f"MCAP has no messages: {mcap_path}")
    return start, end


def _iter_with_source(
    mcap_path: Path,
    source_idx: int,
    *,
    start_time: Optional[int] = None,
    end_time: Optional[int] = None,
):
    """Yield (log_time, source_idx, schema, channel, message) for heapq.merge."""
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for schema, channel, message in reader.iter_messages(
            start_time=start_time,
            end_time=end_time,
            log_time_order=True,
        ):
            yield (message.log_time, source_idx, schema, channel, message)


def _make_ros1_compressed_image(
    svo_data: bytes, fallback_log_time_ns: int, frame_id: Optional[str]
) -> bytes:
    """Decode a side_by_side foxglove.CompressedImage JSON message, crop the
    left half, and serialize it as a ROS1 sensor_msgs/CompressedImage with
    a properly formed std_msgs/Header (so Foxglove can do TF-aware image
    annotations).

    The SVO message stores the JPEG bytes as a JSON list of byte integers
    and `timestamp` as a bare nanoseconds integer.
    """
    obj = json.loads(svo_data)

    raw_data = obj["data"]
    if isinstance(raw_data, list):
        img_bytes = bytes(raw_data)
    elif isinstance(raw_data, str):
        img_bytes = base64.b64decode(raw_data)
    else:
        raise RuntimeError(f"unexpected data field type: {type(raw_data).__name__}")
    img = cv2.imdecode(np.frombuffer(img_bytes, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError("failed to decode side_by_side image")
    left = img[:, : img.shape[1] // 2]
    fmt = (obj.get("format") or "jpeg").lower()
    ext = ".png" if fmt == "png" else ".jpg"
    ok, encoded = cv2.imencode(ext, left)
    if not ok:
        raise RuntimeError(f"failed to re-encode left image (format={fmt})")
    encoded_bytes = encoded.tobytes()

    ts_field = obj.get("timestamp")
    if isinstance(ts_field, int):
        ts_ns = ts_field
    elif isinstance(ts_field, dict):
        ts_ns = int(ts_field.get("sec", 0)) * 1_000_000_000 + int(
            ts_field.get("nsec", 0)
        )
    else:
        ts_ns = fallback_log_time_ns
    sec = ts_ns // 1_000_000_000
    nsec = ts_ns - sec * 1_000_000_000

    fid = (frame_id if frame_id is not None else obj.get("frame_id") or "").encode(
        "utf-8"
    )
    fmt_bytes = fmt.encode("utf-8")

    parts = [
        struct.pack("<I", 0),  # header.seq
        struct.pack("<II", sec, nsec),  # header.stamp
        struct.pack("<I", len(fid)) + fid,  # header.frame_id
        struct.pack("<I", len(fmt_bytes)) + fmt_bytes,  # format
        struct.pack("<I", len(encoded_bytes)) + encoded_bytes,  # data
    ]
    return b"".join(parts)


def read_first_header_frame_id(mcap_path: Path, topic: str) -> Optional[str]:
    """Return the std_msgs/Header.frame_id of the first message on `topic`.

    Assumes the message starts with a ROS1-serialized std_msgs/Header
    (uint32 seq + time stamp + string frame_id). All ROS1 messages whose
    schemas start with `Header header` have this layout.
    """
    HEADER_FIXED_BYTES = 12  # 4 (seq) + 4 (sec) + 4 (nsec)
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for _schema, _channel, message in reader.iter_messages(topics=[topic]):
            data = message.data
            if len(data) < HEADER_FIXED_BYTES + 4:
                return None
            (length,) = struct.unpack_from("<I", data, HEADER_FIXED_BYTES)
            end = HEADER_FIXED_BYTES + 4 + length
            if length == 0 or end > len(data):
                return None
            return data[HEADER_FIXED_BYTES + 4 : end].decode("utf-8", errors="replace")
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


def merge_mcaps(
    original_path: Path,
    svo_mcap_path: Path,
    output_path: Path,
    time_range_ns: Tuple[int, int],
    profile: str,
) -> None:
    """Stream-merge `original` (sliced) and `svo_mcap` into `output_path`."""
    start_ns, end_ns = time_range_ns
    output_path.parent.mkdir(parents=True, exist_ok=True)

    camera_frame_id = read_first_header_frame_id(original_path, CAMERA_INFO_TOPIC)
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

    retimer = HeaderStampRetimer(read_header_stamp_samples(original_path))
    if retimer:
        avg_offset_ns, max_abs_offset_ns = retimer.offset_stats_ns
        logger.info(
            "Retiming original MCAP via %d %s samples (avg log_time-stamp offset %.2f ms, max abs %.2f ms)",
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

    # Total is an upper bound: the original count is pre-time-slice, so the
    # bar may finish slightly before 100% on recordings that extend past the
    # SVO's time range. tqdm handles that gracefully.
    total_messages = _summary_message_count(original_path) + _summary_message_count(
        svo_mcap_path
    )

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

        def get_writer_channel_id(source_idx, schema, channel):
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
        # before the merge. We over-fetch by max_abs_offset so we don't drop
        # messages whose retimed time lands inside the window.
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

        original_iter = iter(original_buffer)
        svo_iter = _iter_with_source(svo_mcap_path, 1)

        with tqdm(
            total=total_messages if total_messages > 0 else None,
            desc=f"Merge {output_path.name}",
            unit="msg",
            unit_scale=True,
            leave=False,
            dynamic_ncols=True,
        ) as merge_bar:
            for log_time, source_idx, schema, channel, message in heapq.merge(
                original_iter, svo_iter, key=lambda x: (x[0], x[1])
            ):
                if source_idx == 1 and channel.topic == SVO_SIDE_BY_SIDE_TOPIC:
                    data = _make_ros1_compressed_image(
                        message.data, log_time, camera_frame_id
                    )
                    writer.add_message(
                        channel_id=left_image_channel_id,
                        log_time=log_time,
                        data=data,
                        publish_time=log_time,
                        sequence=message.sequence,
                    )
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
    keep_intermediate: bool,
    dry_run: bool,
    editor_bin: str,
) -> int:
    """Process one input MCAP. Returns the number of combined MCAPs written."""
    if not input_mcap.exists():
        logger.error("Input MCAP not found: %s", input_mcap)
        return 0

    referenced = extract_svo_paths(input_mcap)
    if not referenced:
        logger.warning("No SVO paths found in /rosout of %s", input_mcap)
        return 0

    out_dir = output_dir if output_dir is not None else input_mcap.parent
    profile = get_input_profile(input_mcap)
    written = 0

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
            logger.warning(
                "Referenced SVO %s not found on disk (searched as-is and in %s): %s",
                [str(d) for d in search_dirs],
                svo_ref,
            )
            continue

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

        intermediate_path = out_dir / f"{input_mcap.stem}__{svo_path.stem}.svo.mcap"
        try:
            convert_svo_to_mcap(svo_path, intermediate_path, editor_bin)
            try:
                start_ns, end_ns = read_time_range(intermediate_path)
            except RuntimeError as exc:
                logger.error("Skipping %s: %s", svo_path, exc)
                continue
            logger.info(
                "SVO time range: [%d, %d] ns (%.3f s span)",
                start_ns,
                end_ns,
                (end_ns - start_ns) / 1e9,
            )

            merge_mcaps(
                input_mcap,
                intermediate_path,
                combined_path,
                (start_ns, end_ns),
                profile,
            )
            logger.info("Wrote %s", combined_path)
            written += 1
        finally:
            if not keep_intermediate and intermediate_path.exists():
                intermediate_path.unlink()

    return written


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
            "Additional directory to search for SVOs by basename if the path "
            "logged in /rosout is missing. Repeatable. Defaults: data/svo, "
            "data/temp_svo (under the project root)."
        ),
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite existing combined MCAPs.",
    )
    parser.add_argument(
        "--keep-intermediate",
        action="store_true",
        help="Keep the per-SVO intermediate MCAP from ZED_SVO_Editor.",
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

    editor_bin = shutil.which("ZED_SVO_Editor")
    if editor_bin is None and not args.dry_run:
        logger.error(
            "ZED_SVO_Editor not found on PATH. Install the ZED SDK or pass --dry-run."
        )
        return 2

    search_dirs = (
        list(args.svo_search_dir) if args.svo_search_dir else list(DEFAULT_SEARCH_DIRS)
    )

    total_written = 0
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
        total_written += process_mcap(
            mcap_path.resolve() if mcap_path.exists() else mcap_path,
            args.output_dir.resolve() if args.output_dir else None,
            search_dirs,
            overwrite=args.overwrite,
            keep_intermediate=args.keep_intermediate,
            dry_run=args.dry_run,
            editor_bin=editor_bin or "",
        )

    logger.info("Done. Wrote %d combined MCAP(s).", total_written)
    return 0


if __name__ == "__main__":
    sys.exit(main())
