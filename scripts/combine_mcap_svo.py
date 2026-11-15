#!/usr/bin/env python3
"""Combine an MCAP recording with the SVO2 files it references."""

from __future__ import annotations

import argparse
import bisect
import heapq
import json
import logging
import re
import shutil
import sys
import tempfile
from contextlib import ExitStack, closing, contextmanager, suppress
from pathlib import Path
from typing import Any, Iterator, List, Optional, Tuple

from mcap.exceptions import McapError
from mcap.reader import make_reader
from mcap.records import Attachment, Channel, Header, Message, Metadata, Schema
from mcap.stream_reader import StreamReader
from mcap.writer import Writer
from tqdm import tqdm

from auto_battlebot.recording import mcap_io, mcap_write, svo2

logger = logging.getLogger("combine_mcap_svo")

TARGET_LEFT_IMAGE_TOPIC = "/camera/image"
SVO_FRAME_TOPIC = "/camera/svo_frame"
FRAME_ID_SOURCE_TOPIC = "/camera/camera_info"
FRAME_META_TOPIC = "/camera/frame_meta"
LOG_TOPIC = "/log"

SVO_FRAME_JSON_SCHEMA = (
    "auto_battlebot.SvoFrame",
    '{"type":"object","title":"auto_battlebot.SvoFrame","properties":{"svo_file":{"type":"string"},'
    '"svo_frame_index":{"type":"integer"},"svo_stamp_ns":{"type":"integer"}}}',
)

RECOVERED_SUBDIR = "recovered"
RECOVERY_LIBRARY = "combine_mcap_svo recover"

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_SEARCH_DIRS = [
    PROJECT_ROOT / "data" / "svo",
    PROJECT_ROOT / "data" / "temp_svo",
]

# spdlog::info("Resolved SVO path: {}", ...) and "SVO recording started: {}".
SVO_PATH_REGEX = re.compile(r"(?:Resolved SVO path|SVO recording started):\s+(\S+\.svo2)")


def _frame_meta_svo_path(data: bytes) -> Optional[Path]:
    """Return the svo_path from a /camera/frame_meta payload.

    Read straight off the JSON rather than through ``mcap_io.decode_frame_meta``: once video moved
    into the MCAP the field stopped being part of ``FrameMeta``, and only the recordings this
    script combines still carry it.
    """
    try:
        raw = str(mcap_io.decode_json(data).get("svo_path", ""))
    except (json.JSONDecodeError, UnicodeDecodeError, AttributeError, KeyError, ValueError):
        return None
    if not raw:
        return None
    return Path(raw)


def extract_svo_paths(mcap_path: Path) -> List[Path]:
    """Return the SVO paths this recording used, in first-seen order.

    Both /log and /camera/frame_meta are read, because neither is complete
    on its own. /log logs the SVO the recorder opened but not the ones it
    rotates to mid-run: auto_battlebot_..._2026-08-28_20-57-33 logs only
    2026-08-28T20-57-35.svo2 and never mentions the 2026-08-28T21-03-09.svo2
    that holds all but the first 123 of its frames. /camera/frame_meta names
    the file per frame, so it catches the rotations, but only recordings from
    2026-08 on publish it.
    """
    seen: "dict[Path, None]" = {}
    for topic, _log_time, data in mcap_io.iter_messages(mcap_path, [LOG_TOPIC, FRAME_META_TOPIC]):
        if topic == FRAME_META_TOPIC:
            path = _frame_meta_svo_path(data)
            if path is not None:
                seen.setdefault(path, None)
            continue
        for match in SVO_PATH_REGEX.finditer(mcap_io.decode_log(data).message):
            seen.setdefault(Path(match.group(1)), None)
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


def _make_left_image(jpeg: bytes, stamp_ns: int, frame_id: Optional[str]) -> Any:
    """Wrap a JPEG as foxglove.CompressedImage. `stamp_ns` is the SVO frame's own timestamp."""
    return mcap_write.compressed_image(stamp_ns, frame_id or "", jpeg)


def _make_svo_frame_payload(svo_name: str, frame_index: int, stamp_ns: int) -> str:
    """One /camera/svo_frame message naming the source SVO file and frame."""
    return json.dumps(
        {"svo_file": svo_name, "svo_frame_index": frame_index, "svo_stamp_ns": stamp_ns},
        separators=(",", ":"),
    )


def read_first_header_frame_id(mcap_path: Path, topic: str) -> Optional[str]:
    """Return the frame_id of the first calibration message on `topic`."""
    for _topic, _log_time, data in mcap_io.iter_messages(mcap_path, [topic]):
        frame_id = mcap_io.decode_camera_info(data).frame_id
        return frame_id or None
    return None


def read_header_stamp_samples(
    mcap_path: Path, topic: str = FRAME_ID_SOURCE_TOPIC
) -> List[Tuple[int, int]]:
    """Return sorted (log_time_ns, header_stamp_ns) pairs for `topic`."""
    samples: List[Tuple[int, int]] = []
    for _topic, log_time, data in mcap_io.iter_messages(mcap_path, [topic]):
        samples.append((log_time, mcap_io.decode_camera_info(data).stamp_ns))
    samples.sort()
    return samples


class HeaderStampRetimer:
    """Linear-interpolate a log_time to header.stamp mapping from samples."""

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
            FRAME_ID_SOURCE_TOPIC,
        )
    else:
        logger.warning(
            "Could not find frame_id on %s in %s; leaving SVO frame_id intact",
            FRAME_ID_SOURCE_TOPIC,
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
            FRAME_ID_SOURCE_TOPIC,
            avg_offset_ns / 1e6,
            max_abs_offset_ns / 1e6,
        )
    else:
        logger.warning(
            "No %s samples in %s; original log_times will not be retimed",
            FRAME_ID_SOURCE_TOPIC,
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
    ffmpeg_bin: str,
) -> None:
    """Stream-merge `original` (sliced) and the SVO's frames into `output_path`."""
    start_ns, end_ns = time_range_ns
    output_path.parent.mkdir(parents=True, exist_ok=True)

    camera_frame_id = read_first_header_frame_id(original_path, FRAME_ID_SOURCE_TOPIC)
    _log_frame_id_resolution(camera_frame_id, original_path)

    retimer = HeaderStampRetimer(read_header_stamp_samples(original_path))
    _log_retimer_stats(retimer, original_path)

    total_messages = _summary_message_count(original_path) + frame_count

    metadata = {}
    with open(original_path, "rb") as f:
        for record in make_reader(f).iter_metadata():
            metadata[record.name] = dict(record.metadata)

    with mcap_write.McapWriter(
        output_path, allow_overwrite=True, compression="zstd", metadata=metadata
    ) as writer:
        svo_name = svo_path.stem
        frame_index = 0

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
                    writer.log(
                        TARGET_LEFT_IMAGE_TOPIC,
                        _make_left_image(jpeg, log_time, camera_frame_id),
                        log_time,
                    )
                    writer.log_json(
                        SVO_FRAME_TOPIC,
                        _make_svo_frame_payload(svo_name, frame_index, log_time),
                        log_time,
                        schema=SVO_FRAME_JSON_SCHEMA,
                    )
                    frame_index += 1
                    merge_bar.update(1)
                    continue

                writer.log_raw(
                    channel.topic,
                    channel.message_encoding,
                    schema.name if schema is not None else None,
                    schema.encoding if schema is not None else None,
                    schema.data if schema is not None else None,
                    message.data,
                    log_time,
                )
                merge_bar.update(1)

            if next(jpegs, None) is not None:
                raise RuntimeError(
                    f"ffmpeg produced more frames than the {frame_count} in {svo_path}"
                )

        writer.write_metadata(
            "svo_export",
            {"svo_file": svo_name, "frame_count": str(frame_index)},
        )
        logger.info("Wrote %d SVO frames to %s", frame_index, SVO_FRAME_TOPIC)


def get_input_profile(mcap_path: Path) -> str:
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        return reader.get_header().profile or ""


def _describe_damage(mcap_path: Path) -> Optional[str]:
    """Return a description of the first fault in `mcap_path`, or None if it reads clean."""
    if mcap_path.stat().st_size == 0:
        return "the file is empty"
    try:
        with open(mcap_path, "rb") as f:
            if make_reader(f).get_summary() is None:
                return "no summary section, so the file was never finalized"
        with open(mcap_path, "rb") as f:
            for _record in StreamReader(f).records:
                pass
    except Exception as exc:
        # Any parse failure means the file is damaged. Truncation surfaces as an
        # McapError, but a corrupt chunk can also come out of the decompressor.
        return f"{type(exc).__name__}: {exc}".rstrip(": ")
    return None


def _copy_record(
    writer: Writer,
    record: Any,
    schema_ids: "dict[int, int]",
    channel_ids: "dict[int, int]",
) -> int:
    """Write one parsed record through to writer; returns 1 for a message."""
    if isinstance(record, Schema):
        schema_ids[record.id] = writer.register_schema(
            name=record.name, encoding=record.encoding, data=record.data
        )
    elif isinstance(record, Channel):
        channel_ids[record.id] = writer.register_channel(
            topic=record.topic,
            message_encoding=record.message_encoding,
            schema_id=schema_ids.get(record.schema_id, 0),
            metadata=record.metadata,
        )
    elif isinstance(record, Message):
        channel_id = channel_ids.get(record.channel_id)
        if channel_id is None:
            return 0
        writer.add_message(
            channel_id=channel_id,
            log_time=record.log_time,
            data=record.data,
            publish_time=record.publish_time,
            sequence=record.sequence,
        )
        return 1
    elif isinstance(record, Metadata):
        writer.add_metadata(record.name, record.metadata)
    elif isinstance(record, Attachment):
        writer.add_attachment(
            create_time=record.create_time,
            log_time=record.log_time,
            name=record.name,
            media_type=record.media_type,
            data=record.data,
        )
    return 0


def recover_mcap(source: Path, dest: Path) -> int:
    """Copy every record that still parses from source into a fresh MCAP."""
    dest.parent.mkdir(parents=True, exist_ok=True)
    schema_ids: "dict[int, int]" = {}
    channel_ids: "dict[int, int]" = {}
    messages = 0
    started = False
    with open(source, "rb") as in_f, open(dest, "wb") as out_f:
        writer = Writer(out_f)
        try:
            for record in StreamReader(in_f).records:
                if isinstance(record, Header):
                    writer.start(profile=record.profile, library=RECOVERY_LIBRARY)
                    started = True
                    continue
                if not started:
                    writer.start(library=RECOVERY_LIBRARY)
                    started = True
                messages += _copy_record(writer, record, schema_ids, channel_ids)
        except Exception as exc:
            logger.debug("Stopped reading %s after %d message(s): %s", source, messages, exc)
        if not started:
            writer.start(library=RECOVERY_LIBRARY)
        writer.finish()
    return messages


def _recover_into(source: Path, dest_dir: Path) -> Optional[Path]:
    """Recover `source` into `dest_dir` under the same name, or None if nothing reads."""
    dest = dest_dir / source.name
    messages = recover_mcap(source, dest)
    if messages == 0:
        dest.unlink(missing_ok=True)
        with suppress(OSError):
            dest_dir.rmdir()
        return None
    logger.info("Recovered %d message(s) from %s into %s", messages, source.name, dest)
    return dest


@contextmanager
def _recovered_copy(source: Path, out_dir: Path, *, keep: bool) -> Iterator[Optional[Path]]:
    """Yield a repaired copy of source, or None when nothing reads."""
    with ExitStack() as stack:
        if keep:
            dest_dir = out_dir / RECOVERED_SUBDIR
        else:
            dest_dir = Path(
                stack.enter_context(tempfile.TemporaryDirectory(prefix="combine_mcap_svo_"))
            )
        yield _recover_into(source, dest_dir)


def _readable_svo(svo_path: Path, out_dir: Path) -> Optional[Path]:
    """Return an SVO that reads cleanly, recovering a damaged one first."""
    damage = _describe_damage(svo_path)
    if damage is None:
        return svo_path
    logger.warning("SVO %s is damaged (%s). Trying to recover the readable part.", svo_path, damage)
    return _recover_into(svo_path, out_dir / RECOVERED_SUBDIR)


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
    """Process one input MCAP, repairing it first if it does not read cleanly.

    Returns (number of combined MCAPs written, number of referenced SVOs unusable).
    """
    if not input_mcap.exists():
        logger.error("Input MCAP not found: %s", input_mcap)
        return 0, 0

    out_dir = output_dir if output_dir is not None else input_mcap.parent
    damage = _describe_damage(input_mcap)
    if damage is None:
        return _combine_svos(
            input_mcap,
            out_dir,
            search_dirs,
            overwrite=overwrite,
            dry_run=dry_run,
            ffmpeg_bin=ffmpeg_bin,
            ignore_missing_svo=ignore_missing_svo,
        )

    logger.warning("%s is damaged (%s). Trying to recover the readable part.", input_mcap, damage)
    with _recovered_copy(input_mcap, out_dir, keep=not dry_run) as recovered:
        if recovered is None:
            logger.error("Nothing readable in %s; skipping it.", input_mcap)
            return 0, 0
        return _combine_svos(
            recovered,
            out_dir,
            search_dirs,
            overwrite=overwrite,
            dry_run=dry_run,
            ffmpeg_bin=ffmpeg_bin,
            ignore_missing_svo=ignore_missing_svo,
        )


def _combine_svos(
    input_mcap: Path,
    out_dir: Path,
    search_dirs: List[Path],
    *,
    overwrite: bool,
    dry_run: bool,
    ffmpeg_bin: str,
    ignore_missing_svo: bool,
) -> Tuple[int, int]:
    """Write one combined MCAP per SVO `input_mcap` references.

    `input_mcap` has to read cleanly by this point; process_mcap repairs it first
    if it does not.
    """
    referenced = extract_svo_paths(input_mcap)
    if not referenced:
        logger.warning("No SVO paths found in %s", input_mcap)
        return 0, 0

    profile = get_input_profile(input_mcap)
    if profile == "ros1":
        logger.error(
            "%s is a recording from before the Foxglove migration; rewrite it with "
            "scripts/convert_ros1_mcap.py first",
            input_mcap,
        )
        return 0, 0
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

        svo_source = _readable_svo(svo_path, out_dir)
        if svo_source is None:
            logger.error("Nothing readable in SVO %s; skipping it.", svo_path)
            missing += 1
            continue

        try:
            svo_topic, frame_count = svo2.find_side_by_side_topic(svo_source)
            start_ns, end_ns = read_svo_time_range(svo_source, svo_topic)
        except (McapError, RuntimeError, ValueError, OSError) as exc:
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
            svo_source,
            svo_topic,
            frame_count,
            combined_path,
            (start_ns, end_ns),
            ffmpeg_bin,
        )
        logger.info("Wrote %s", combined_path)
        written += 1

    return written, missing


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Combine an MCAP recording with the SVO2 files it references (in /log or "
            "/camera/frame_meta). Produces one combined MCAP per SVO with the cropped "
            "left-eye image per frame and the original messages inside the SVO's time range. "
            "Needs ffmpeg on PATH and the Python mcap package."
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
            "path logged in /log is missing. Repeatable. Defaults: data/svo, "
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
