"""Wrap a plain video file into a recording shaped like the RGB camera's.

The camera records H.264 on ``/camera/video`` inside the same MCAP as the pipeline output
(``docs/foxglove_recording_format.md``). Fixed-camera footage from elsewhere, such as NHRL's
``Cage-N-Overhead-High`` feed, is the same shape of data from a different sensor, so re-wrapping it
gives ``VideoPlaybackCamera`` something to replay before the camera hardware exists.

What comes out is a recording, not a match: it carries the three camera channels and nothing else,
because everything else in a real file is pipeline *output* and replay exists to regenerate it.

The video is re-encoded rather than remuxed. A general MP4 uses B-frames and whatever GOP its
encoder chose; the camera writes ``max_b_frames = 0`` and an IDR every 30 frames, and both
``start_frame`` seeking and reading the nth message as the nth captured frame depend on that.

Two CLIs sit on this: ``scripts/video_to_mcap.py`` converts videos you already have, and
``scripts/nhrl_to_recordings.py`` runs the whole NHRL flow from fight to replayable recording set.
"""

from __future__ import annotations

import json
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterator

import foxglove.messages as fg

from auto_battlebot.recording.mcap_write import (
    FRAME_META_JSON_SCHEMA,
    FRAME_META_SCHEMA_NAME,
    McapWriter,
)

_NS = 1_000_000_000
CAMERA_FRAME_ID = "camera"
VIDEO_TOPIC = "/camera/video"
FRAME_META_TOPIC = "/camera/frame_meta"
CAMERA_INFO_TOPIC = "/camera/camera_info"
# What the camera itself writes. Half a second of seek granularity at 60 fps.
KEYFRAME_INTERVAL = 30
DEFAULT_CRF = 20
DEFAULT_PRESET = "veryfast"


@dataclass
class VideoInfo:
    width: int
    height: int
    fps: float
    frames: int


@dataclass
class ConversionResult:
    source: Path
    output: Path
    frames: int
    width: int
    height: int
    fps: float
    bytes_written: int


def probe(video: Path) -> VideoInfo:
    """Geometry and frame count from ffprobe."""
    result = subprocess.run(
        [
            "ffprobe",
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-show_entries",
            "stream=width,height,r_frame_rate,nb_frames,duration",
            "-of",
            "json",
            str(video),
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    stream = json.loads(result.stdout)["streams"][0]
    numerator, _, denominator = str(stream["r_frame_rate"]).partition("/")
    fps = float(numerator) / float(denominator or 1)
    frames = int(stream.get("nb_frames") or 0)
    if frames == 0:
        frames = int(round(float(stream.get("duration", 0.0)) * fps))
    return VideoInfo(int(stream["width"]), int(stream["height"]), fps, frames)


def iter_access_units(
    video: Path, fps: float, crf: int = DEFAULT_CRF, preset: str = DEFAULT_PRESET
) -> Iterator[bytes]:
    """Annex-B access units, one per frame, re-encoded to the camera's constraints.

    ffmpeg writes a raw H.264 elementary stream to stdout and libavcodec's own parser splits it
    back into one packet per coded picture. Hand-rolling that split from start codes gets the
    SPS-before-IDR case wrong in ways that only show up as a frame count that is quietly short.
    """
    import av

    command = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-i",
        str(video),
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        preset,
        "-crf",
        str(crf),
        # The camera's constraints. No B-frames, because they reorder PTS and everything
        # downstream reads the nth message as the nth captured frame. Fixed GOP with scene-cut
        # detection off, so the IDR interval is the seek granularity it claims to be.
        "-bf",
        "0",
        "-g",
        str(KEYFRAME_INTERVAL),
        "-keyint_min",
        str(KEYFRAME_INTERVAL),
        "-sc_threshold",
        "0",
        "-pix_fmt",
        "yuv420p",
        "-x264-params",
        "repeat-headers=1",
        "-f",
        "h264",
        "-r",
        f"{fps:.6f}",
        "pipe:1",
    ]
    parser = av.CodecContext.create("h264", "r")
    with subprocess.Popen(command, stdout=subprocess.PIPE, bufsize=1 << 22) as process:
        assert process.stdout is not None
        while True:
            block = process.stdout.read(1 << 20)
            if not block:
                break
            for packet in parser.parse(block):
                yield bytes(packet)
        for packet in parser.parse(None):
            yield bytes(packet)
    if process.returncode != 0:
        raise RuntimeError(f"ffmpeg failed with exit code {process.returncode} for {video}")


def camera_calibration(width: int, height: int, focal_px: float, stamp_ns: int) -> Any:
    """A pinhole model with no distortion, which is all we can honestly claim for footage whose
    lens we never measured."""
    k = [focal_px, 0.0, width / 2.0, 0.0, focal_px, height / 2.0, 0.0, 0.0, 1.0]
    return fg.CameraCalibration(
        timestamp=fg.Timestamp(sec=stamp_ns // _NS, nsec=stamp_ns % _NS),
        frame_id=CAMERA_FRAME_ID,
        width=width,
        height=height,
        distortion_model="plumb_bob",
        D=[0.0] * 5,
        K=k,
        R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        P=[k[0], k[1], k[2], 0.0, k[3], k[4], k[5], 0.0, k[6], k[7], k[8], 0.0],
    )


def convert_video(
    video: Path,
    output: Path,
    *,
    calibration_id: str,
    focal_px: float,
    start_stamp_ns: int,
    active_profile: str = "_orin_rgb",
    crf: int = DEFAULT_CRF,
    preset: str = DEFAULT_PRESET,
    progress: bool = True,
) -> ConversionResult:
    """Re-wrap one video as a recording. Returns what was written."""
    info = probe(video)
    if progress:
        print(
            f"{video.name}: {info.width}x{info.height} at {info.fps:.3f} fps, ~{info.frames} frames"
        )

    output.parent.mkdir(parents=True, exist_ok=True)
    period_ns = int(round(_NS / info.fps))
    written = 0
    with McapWriter(
        output,
        active_profile=active_profile,
        allow_overwrite=True,
        metadata={"auto_battlebot": {"calibration_id": calibration_id}},
    ) as writer:
        for index, access_unit in enumerate(iter_access_units(video, info.fps, crf, preset)):
            stamp_ns = start_stamp_ns + index * period_ns
            if index == 0:
                writer.log(
                    CAMERA_INFO_TOPIC,
                    camera_calibration(info.width, info.height, focal_px, stamp_ns),
                    stamp_ns,
                )
            writer.log(
                VIDEO_TOPIC,
                fg.CompressedVideo(
                    timestamp=fg.Timestamp(sec=stamp_ns // _NS, nsec=stamp_ns % _NS),
                    frame_id=CAMERA_FRAME_ID,
                    data=access_unit,
                    format="h264",
                ),
                stamp_ns,
            )
            writer.log_json(
                FRAME_META_TOPIC,
                {"image_stamp_ns": str(stamp_ns), "video_frame_index": index},
                stamp_ns,
                schema=(FRAME_META_SCHEMA_NAME, FRAME_META_JSON_SCHEMA),
            )
            written += 1
            if progress and written % 200 == 0:
                sys.stdout.write(f"\r  {written} frames")
                sys.stdout.flush()

    size = output.stat().st_size
    if progress:
        sys.stdout.write("\r")
        print(f"  wrote {written} frames to {output.name} ({size / (1024 * 1024):.0f} MB)")
    return ConversionResult(
        source=video,
        output=output,
        frames=written,
        width=info.width,
        height=info.height,
        fps=info.fps,
        bytes_written=size,
    )


def next_start_stamp_ns(previous: int, frames: int, fps: float) -> int:
    """Where the next recording's stamps begin, so two clips never overlap in time.

    Ten seconds of slack between them, which is enough that a tool joining on timestamps cannot
    silently pick up the neighbouring recording's frames.
    """
    return previous + int((frames * (_NS / fps)) + 10 * _NS)
