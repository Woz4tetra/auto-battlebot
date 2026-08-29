"""Extract left/right RGB video from a ZED ``.svo2`` recording without the ZED SDK.

A ``.svo2`` file is an MCAP container holding H.264-encoded side-by-side stereo
frames on the ``<camera>/side_by_side`` topic. Each message is one access unit:
an 8-byte length prefix followed by an Annex-B H.264 stream. This script reads
those frames with the ``mcap`` library, crops the requested eye, and pipes the
raw H.264 straight into ffmpeg to write an MP4. No ``pyzed`` / CUDA required.

Depth is NOT available through this path. The ZED SDK computes depth from the
stereo pair on the device and it is not stored in the recording; use the SDK
(``svo_export.py``) if you need the depth channel.

Usage:
    python training/svo_rgb_extract.py recording.svo2                 # left eye
    python training/svo_rgb_extract.py recording.svo2 --channel 1     # right eye
    python training/svo_rgb_extract.py a.svo2 b.svo2 --output out.mp4
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from auto_battlebot.svo2 import find_side_by_side_topic, iter_access_units, sample_fps

_EYES = {0: "left", 1: "right"}


def convert_svo_file(svo_file: Path, channel: int, output: Path, crf: int, preset: str) -> None:
    """Decode one eye of a ``.svo2`` recording into an MP4 via ffmpeg."""
    topic, total = find_side_by_side_topic(svo_file)
    fps = sample_fps(svo_file, topic)

    # crop the requested half of the side-by-side frame using intrinsic width.
    crop = "crop=iw/2:ih:0:0" if channel == 0 else "crop=iw/2:ih:iw/2:0"
    # ZED uses intra-refresh (GDR) H.264 with periodic SPS/PPS and no IDR frames,
    # so ffmpeg logs benign "pps_id out of range" / "Overread VUI" warnings on a
    # few parameter sets. -err_detect ignore_err conceals them and keeps decoding;
    # -r sets the timing the raw elementary stream lacks; -vsync cfr keeps every
    # input frame at a constant rate.
    command = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-fflags",
        "+genpts",
        "-err_detect",
        "ignore_err",
        "-f",
        "h264",
        "-r",
        f"{fps:.6f}",
        "-i",
        "pipe:0",
        "-vf",
        crop,
        "-c:v",
        "libx264",
        "-crf",
        str(crf),
        "-preset",
        preset,
        "-pix_fmt",
        "yuv420p",
        "-vsync",
        "cfr",
        str(output),
    ]

    print(f"Converting {svo_file} [{_EYES[channel]}] -> {output} ({fps:.2f} fps)")
    proc = subprocess.Popen(command, stdin=subprocess.PIPE)
    assert proc.stdin is not None
    written = 0
    try:
        for access_unit in iter_access_units(svo_file, topic):
            proc.stdin.write(access_unit)
            written += 1
            if total and written % 50 == 0:
                percent = 100.0 * written / total
                sys.stdout.write(f"\r  {written}/{total} frames ({percent:.0f}%)")
                sys.stdout.flush()
        proc.stdin.close()
    except BrokenPipeError:
        proc.stdin.close()
    return_code = proc.wait()
    sys.stdout.write("\n")
    if return_code != 0:
        raise RuntimeError(f"ffmpeg failed with exit code {return_code} for {svo_file}")
    print(f"  wrote {written} frames to {output}")


def default_output(svo_file: Path, channel: int) -> Path:
    """Derive ``<stem>_<eye>.mp4`` next to the input recording."""
    return svo_file.with_name(f"{svo_file.stem}_{_EYES[channel]}.mp4")


def main(opt: argparse.Namespace) -> None:
    if opt.channel not in _EYES:
        raise SystemExit(
            f"--channel must be 0 (left) or 1 (right); depth is not available without "
            f"the ZED SDK (got {opt.channel})"
        )
    svo_files = [Path(p) for p in opt.svo_files]
    if opt.output and len(svo_files) > 1:
        raise SystemExit("--output cannot be used with multiple input files")

    for svo_file in svo_files:
        if not svo_file.exists():
            raise SystemExit(f"Input file does not exist: {svo_file}")
        output = Path(opt.output) if opt.output else default_output(svo_file, opt.channel)
        convert_svo_file(svo_file, opt.channel, output, opt.crf, opt.preset)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("svo_files", type=str, nargs="+", help="Path to .svo2 file(s)")
    parser.add_argument(
        "--channel",
        type=int,
        default=0,
        help="Eye to export (0: left, 1: right). Depth is not supported.",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="",
        help=(
            "Output MP4 path (single input only). If omitted, writes"
            " <stem>_<eye>.mp4 next to each input file."
        ),
    )
    parser.add_argument("--crf", type=int, default=23, help="x264 quality (lower is better).")
    parser.add_argument("--preset", type=str, default="medium", help="x264 encode preset.")
    main(parser.parse_args())
