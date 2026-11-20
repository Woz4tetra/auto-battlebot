"""Extract left/right RGB video from a ZED ``.svo2`` recording without the ZED SDK."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from auto_battlebot.recording.svo2 import find_side_by_side_topic, iter_access_units, sample_fps

_EYES = {0: "left", 1: "right"}


def ffmpeg_command(fps: float, channel: int, output: Path, crf: int, preset: str) -> list[str]:
    """ffmpeg argv that decodes the raw side-by-side H.264 pipe and keeps one eye."""
    crop = "crop=iw/2:ih:0:0" if channel == 0 else "crop=iw/2:ih:iw/2:0"
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
    return command


def convert_svo_file(svo_file: Path, channel: int, output: Path, crf: int, preset: str) -> None:
    """Decode one eye of a ``.svo2`` recording into an MP4 via ffmpeg."""
    topic, total = find_side_by_side_topic(svo_file)
    fps = sample_fps(svo_file, topic)

    command = ffmpeg_command(fps, channel, output, crf, preset)

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
