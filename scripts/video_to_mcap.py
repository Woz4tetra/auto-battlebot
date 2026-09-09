"""Convert video files into recordings the RGB camera could have written.

The primitive: one or more videos in, one MCAP each out. For the whole NHRL flow, from fight to a
named recording set with a playback config, use ``scripts/nhrl_to_recordings.py`` instead.

The conversion itself lives in ``auto_battlebot.recording.video_import``; this is a CLI over it.

Usage:
    venv/bin/python scripts/video_to_mcap.py clip.mp4 --output-dir data/saved_recordings/scratch
    venv/bin/python scripts/video_to_mcap.py dir/*.mp4 --output-dir out/ --calibration-id my_cam
"""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

from auto_battlebot.recording.video_import import (
    DEFAULT_CRF,
    DEFAULT_PRESET,
    convert_video,
    next_start_stamp_ns,
)

_NS = 1_000_000_000


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("videos", type=Path, nargs="+")
    parser.add_argument("--output-dir", type=Path, default=Path("data/saved_recordings"))
    parser.add_argument(
        "--calibration-id",
        default="brettzone_cage_high",
        help="written into the file metadata; playback resolves config/cameras/<id>.toml",
    )
    parser.add_argument(
        "--focal-px",
        type=float,
        default=1331.0,
        help="pinhole focal length for /camera/camera_info, in pixels",
    )
    parser.add_argument(
        "--start",
        default="2026-01-01T00:00:00",
        help="ISO time the first recording's stamps begin at; later ones follow it",
    )
    parser.add_argument("--crf", type=int, default=DEFAULT_CRF)
    parser.add_argument("--preset", default=DEFAULT_PRESET)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    # Stamps are synthetic but plausible: one recording per clip, laid out back to back so two
    # clips never share a timestamp.
    stamp_ns = int(datetime.fromisoformat(args.start).timestamp()) * _NS
    for video in args.videos:
        result = convert_video(
            video,
            args.output_dir / f"{video.stem}.mcap",
            calibration_id=args.calibration_id,
            focal_px=args.focal_px,
            start_stamp_ns=stamp_ns,
            crf=args.crf,
            preset=args.preset,
        )
        stamp_ns = next_start_stamp_ns(stamp_ns, result.frames, result.fps)


if __name__ == "__main__":
    main()
