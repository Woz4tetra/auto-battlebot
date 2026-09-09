"""Extract the camera video from a pipeline recording, as MP4 or as still frames.

The video-MCAP counterpart to ``svo_rgb_extract.py``. A recording made with the RGB camera carries
one Annex-B H.264 access unit per frame on ``/camera/video``, in the same file as the pipeline
output, so there is no second file to find and no join to do. Frames come out pre-rectification,
which is what makes a revised lens calibration applicable to footage already shot.

The MP4 path pipes the raw access units straight into ffmpeg without decoding, which is fast and
lossless. The frames path decodes and writes PNGs, for labelling.

Usage:
    venv/bin/python training/video_mcap_extract.py recording.mcap
    venv/bin/python training/video_mcap_extract.py recording.mcap --frames out_dir/
    venv/bin/python training/video_mcap_extract.py recording.mcap --frames out_dir/ --stride 10
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

import cv2

from auto_battlebot.recording.mcap_io import (
    CAMERA_VIDEO_TOPIC,
    decode_compressed_video_bytes,
    iter_messages,
    iter_video_frames,
    recording_topics,
)


def _require_video(recording: Path) -> None:
    topics = recording_topics(recording)
    if CAMERA_VIDEO_TOPIC not in topics:
        raise SystemExit(
            f"{recording} has no {CAMERA_VIDEO_TOPIC}. Recordings made with the ZED carry their "
            f"frames in a separate .svo2 file; use training/svo_rgb_extract.py for those."
        )


def estimate_fps(recording: Path) -> float:
    """Frame rate from the log times of the first and last video message."""
    first = None
    last = None
    count = 0
    for _topic, log_time, _payload in iter_messages(recording, [CAMERA_VIDEO_TOPIC]):
        if first is None:
            first = log_time
        last = log_time
        count += 1
    if first is None or last is None or count < 2 or last == first:
        return 60.0
    return (count - 1) / ((last - first) / 1e9)


def to_mp4(recording: Path, output: Path, crf: int, preset: str) -> None:
    _require_video(recording)
    fps = estimate_fps(recording)
    command = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-fflags",
        "+genpts",
        "-f",
        "h264",
        "-r",
        f"{fps:.6f}",
        "-i",
        "pipe:0",
        "-c:v",
        "libx264",
        "-crf",
        str(crf),
        "-preset",
        preset,
        "-pix_fmt",
        "yuv420p",
        str(output),
    ]
    print(f"Converting {recording} -> {output} ({fps:.2f} fps)")
    proc = subprocess.Popen(command, stdin=subprocess.PIPE)
    assert proc.stdin is not None
    written = 0
    try:
        for _topic, _log_time, payload in iter_messages(recording, [CAMERA_VIDEO_TOPIC]):
            _stamp_ns, _frame_id, _fmt, access_unit = decode_compressed_video_bytes(payload)
            proc.stdin.write(access_unit)
            written += 1
            if written % 50 == 0:
                sys.stdout.write(f"\r  {written} frames")
                sys.stdout.flush()
        proc.stdin.close()
    except BrokenPipeError:
        proc.stdin.close()
    return_code = proc.wait()
    sys.stdout.write("\n")
    if return_code != 0:
        raise SystemExit(f"ffmpeg failed with exit code {return_code} for {recording}")
    print(f"  wrote {written} frames to {output}")


def to_frames(recording: Path, out_dir: Path, stride: int) -> None:
    _require_video(recording)
    out_dir.mkdir(parents=True, exist_ok=True)
    written = 0
    for index, (stamp_ns, image) in enumerate(iter_video_frames(recording)):
        if index % stride != 0:
            continue
        # Index first so the filenames sort in capture order; the stamp is there so a frame can be
        # tied back to the pipeline messages around it.
        cv2.imwrite(str(out_dir / f"{index:06d}_{stamp_ns}.png"), image)
        written += 1
    print(f"wrote {written} frames to {out_dir}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path, help="pipeline .mcap carrying /camera/video")
    parser.add_argument("--output", type=Path, help="MP4 path (default: alongside the recording)")
    parser.add_argument("--frames", type=Path, help="write PNG frames to this directory instead")
    parser.add_argument("--stride", type=int, default=1, help="keep one frame in N (--frames only)")
    parser.add_argument("--crf", type=int, default=18)
    parser.add_argument("--preset", default="medium")
    args = parser.parse_args()

    if args.frames is not None:
        to_frames(args.recording, args.frames, max(1, args.stride))
        return
    output = args.output or args.recording.with_suffix(".mp4")
    to_mp4(args.recording, output, args.crf, args.preset)


if __name__ == "__main__":
    main()
