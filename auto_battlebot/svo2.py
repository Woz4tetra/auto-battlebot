"""Read ZED ``.svo2`` recordings without the ZED SDK.

A ``.svo2`` file is an MCAP container written by ``libmcap stereolabs``:

  * ``<camera>/side_by_side`` (``zed_sdk_encoded``): one message per frame, an
    8-byte length prefix followed by an Annex-B H.264 access unit holding the
    ``[left | right]`` stereo pair. The message ``log_time`` is the camera
    frame timestamp in epoch nanoseconds.
  * ``<camera>/sensors`` and ``<camera>/integrated_sensors`` (JSON): IMU and
    friends, carried as an opaque base64 blob that only the SDK decodes.
  * ``svo_header`` / ``svo_footer`` (JSON): calibration, and an index listing
    every message timestamp per topic. Both carry a ``log_time`` unrelated to
    the recording's own timeline, so never take the time range from the MCAP
    summary statistics.

Depth is not available through this path: the SDK computes it on the device
and it is not stored in the recording. Use ``training/svo_export.py`` (pyzed)
if you need the depth channel.
"""

from __future__ import annotations

import json
import subprocess
import threading
from collections import deque
from pathlib import Path
from typing import IO, Generator, Iterator

from mcap.reader import make_reader

from auto_battlebot.mcap_io import iter_messages

SIDE_BY_SIDE_SUFFIX = "/side_by_side"
FOOTER_TOPIC = "svo_footer"

# Each side_by_side payload is two 4-byte little-endian sizes followed by the
# Annex-B stream, which always opens on a 4-byte start code.
_PAYLOAD_HEADER_BYTES = 8
_ANNEX_B_START_CODE = b"\x00\x00\x00\x01"

_JPEG_SOI = b"\xff\xd8\xff"
_READ_CHUNK = 1 << 20

_DEFAULT_FPS = 30.0
_FPS_SAMPLE_FRAMES = 240
_STDERR_TAIL_LINES = 20


def find_side_by_side_topic(path: Path | str) -> tuple[str, int]:
    """Return the ``*/side_by_side`` topic name and its message count."""
    with open(path, "rb") as file:
        summary = make_reader(file).get_summary()
        if summary is None:
            raise ValueError(f"{path} has no MCAP summary; not a valid .svo2 file")
        counts = summary.statistics.channel_message_counts if summary.statistics else {}
        for channel in summary.channels.values():
            if channel.topic.endswith(SIDE_BY_SIDE_SUFFIX):
                return channel.topic, counts.get(channel.id, 0)
    raise ValueError(f"No '*{SIDE_BY_SIDE_SUFFIX}' topic found in {path}")


def sample_fps(path: Path | str, topic: str) -> float:
    """Estimate frame rate from the median inter-frame timestamp gap."""
    stamps: list[int] = []
    for _topic, log_time_ns, _data in iter_messages(path, [topic]):
        stamps.append(log_time_ns)
        if len(stamps) >= _FPS_SAMPLE_FRAMES:
            break
    if len(stamps) < 2:
        return _DEFAULT_FPS
    diffs = sorted(stamps[i + 1] - stamps[i] for i in range(len(stamps) - 1))
    median_ns = diffs[len(diffs) // 2]
    if median_ns <= 0:
        return _DEFAULT_FPS
    return 1e9 / median_ns


def read_frame_stamps(path: Path | str, topic: str) -> list[int]:
    """Return every frame timestamp on `topic`, in file order.

    Reads ``svo_footer``'s per-topic index when it is present, which costs one
    message instead of a pass over the recording, and falls back to scanning
    the topic when the footer is missing (a truncated or interrupted file).
    """
    stamps = _read_footer_stamps(path, topic)
    if stamps:
        return stamps
    return [log_time_ns for _topic, log_time_ns, _data in iter_messages(path, [topic])]


def _read_footer_stamps(path: Path | str, topic: str) -> list[int]:
    """Return `topic`'s timestamp list from svo_footer, or [] if unavailable."""
    with open(path, "rb") as file:
        for _schema, _channel, message in make_reader(file).iter_messages(topics=[FOOTER_TOPIC]):
            try:
                footer = json.loads(message.data)
            except json.JSONDecodeError:
                return []
            entry = footer.get(topic)
            if isinstance(entry, list):
                return [int(stamp) for stamp in entry]
    return []


def iter_access_units(path: Path | str, topic: str) -> Iterator[bytes]:
    """Yield the Annex-B H.264 access unit of every message on `topic`."""
    for _topic, log_time_ns, data in iter_messages(path, [topic]):
        if data[_PAYLOAD_HEADER_BYTES : _PAYLOAD_HEADER_BYTES + 4] == _ANNEX_B_START_CODE:
            yield data[_PAYLOAD_HEADER_BYTES:]
            continue
        start = data.find(_ANNEX_B_START_CODE)
        if start < 0:
            raise RuntimeError(f"{path}: message at {log_time_ns} has no H.264 start code")
        yield data[start:]


def iter_left_jpegs(
    path: Path | str,
    topic: str,
    fps: float,
    *,
    quality: int = 2,
    ffmpeg_bin: str = "ffmpeg",
) -> Generator[bytes, None, None]:
    """Yield one JPEG of the left eye per `topic` message, in file order.

    ffmpeg decodes the concatenated access units and re-encodes the cropped
    left half to MJPEG. Output frames come out one-for-one with the messages
    fed in, so the Nth JPEG is SVO frame N.

    ``-r`` is required: a raw elementary stream carries no timing, and without
    an input rate the mjpeg encoder rejects the second frame with "Invalid pts
    (0) <= last (0)" and the export stops after a handful of frames.

    ZED records intra-refresh H.264 with no IDR frames, so the first frames of
    a recording decode partially until the refresh completes. They are emitted
    anyway to keep frame indices aligned with the SVO.

    Close the iterator (``contextlib.closing``) if you stop early, so ffmpeg
    is torn down instead of blocking on a full pipe.
    """
    command = [
        ffmpeg_bin,
        "-hide_banner",
        "-loglevel",
        "error",
        "-err_detect",
        "ignore_err",
        "-f",
        "h264",
        "-r",
        f"{fps:.6f}",
        "-i",
        "pipe:0",
        "-vf",
        "crop=iw/2:ih:0:0",
        "-f",
        "image2pipe",
        "-vcodec",
        "mjpeg",
        "-q:v",
        str(quality),
        # `-vsync passthrough` rather than `-fps_mode`: the distro ffmpeg on 22.04 is 4.4,
        # which predates `-fps_mode`, and newer builds still accept the old spelling.
        "-vsync",
        "passthrough",
        "pipe:1",
    ]
    proc = subprocess.Popen(
        command,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    assert proc.stdin is not None and proc.stdout is not None and proc.stderr is not None

    errors: deque[str] = deque(maxlen=_STDERR_TAIL_LINES)
    feeder = threading.Thread(
        target=_feed_access_units, args=(proc.stdin, path, topic), daemon=True
    )
    drainer = threading.Thread(target=_drain_stderr, args=(proc.stderr, errors), daemon=True)
    feeder.start()
    drainer.start()

    try:
        yield from _iter_jpegs(proc.stdout)
        feeder.join()
        drainer.join()
        returncode = proc.wait()
        if returncode != 0:
            tail = "\n".join(errors)
            raise RuntimeError(f"ffmpeg failed (exit {returncode}) for {path}\n{tail}")
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()


def _feed_access_units(stdin: IO[bytes], path: Path | str, topic: str) -> None:
    """Write every access unit to ffmpeg, tolerating an abandoned reader."""
    try:
        for access_unit in iter_access_units(path, topic):
            stdin.write(access_unit)
        stdin.close()
    except (BrokenPipeError, ValueError):
        # The consumer stopped early and ffmpeg was killed; nothing to do.
        pass


def _drain_stderr(stderr: IO[bytes], errors: deque[str]) -> None:
    """Keep the last few ffmpeg error lines so a failure can be reported."""
    for raw_line in stderr:
        line = raw_line.decode("utf-8", errors="replace").rstrip()
        if line:
            errors.append(line)


def _iter_jpegs(stream: IO[bytes]) -> Iterator[bytes]:
    """Split a concatenated MJPEG byte stream into individual JPEGs.

    Images are separated on the SOI marker. 0xFF bytes inside entropy-coded
    data are byte-stuffed as FF00, so FFD8FF only ever appears at the start of
    an image.
    """
    buffer = bytearray()
    # Where the next SOI search starts. Everything before it has been checked,
    # apart from a short overlap so a marker split across two reads is found.
    scan = 0
    header_checked = False
    while True:
        chunk = stream.read(_READ_CHUNK)
        if not chunk:
            break
        buffer += chunk
        if not header_checked and len(buffer) >= len(_JPEG_SOI):
            if not buffer.startswith(_JPEG_SOI):
                raise RuntimeError("ffmpeg output does not start with a JPEG marker")
            header_checked = True
        while True:
            index = buffer.find(_JPEG_SOI, scan)
            if index < 0:
                scan = max(0, len(buffer) - len(_JPEG_SOI) + 1)
                break
            if index > 0:
                yield bytes(buffer[:index])
                del buffer[:index]
            scan = len(_JPEG_SOI)
    if buffer:
        yield bytes(buffer)
