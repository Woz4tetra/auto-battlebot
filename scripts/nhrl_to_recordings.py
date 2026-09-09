"""Turn NHRL fixed-cage fight video into a replayable recording set.

One command for the whole flow: fetch the fights (optional), re-wrap each clip as a recording the
RGB camera could have written, land them in ``data/saved_recordings/<set>/``, and write the
playback config that points at them.

NHRL's ``Cage-N-Overhead-High`` is the one angle that does not move, which makes it the closest
thing to the RGB camera that exists before the camera does: one fixed 1080p view, no depth, no
visual odometry. Every other recording in the repo is a moving ZED.

    # fights you have already downloaded
    venv/bin/python scripts/nhrl_to_recordings.py --from-dir data/downloads/mrsbuff_may26

    # fetch and convert in one pass
    venv/bin/python scripts/nhrl_to_recordings.py --bot "MRS BUFF" --since 2026-06-01 --limit 6

    # name the set explicitly rather than taking it from the source directory
    venv/bin/python scripts/nhrl_to_recordings.py --from-dir dir/ --name BrettZone_2026-06-14

Re-running skips clips already converted, so adding fights to a set costs only the new ones.

**Two things this footage is not.** The intrinsics in ``config/cameras/brettzone_cage_high.toml``
are backed out of NHRL's stated field of view rather than measured, and the homography field fit
inherits that error linearly, so ranges off these recordings are indicative rather than trusted.
And the detectors were trained on ZED frames at a different mount, so detections here report
transfer rather than accuracy.
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

from auto_battlebot.recording.mcap_io import message_counts
from auto_battlebot.recording.video_import import (
    DEFAULT_CRF,
    DEFAULT_PRESET,
    VIDEO_TOPIC,
    ConversionResult,
    convert_video,
    next_start_stamp_ns,
    probe,
)

_NS = 1_000_000_000
DOWNLOADER = Path("playground/bgsub_cage/download_cage_video.py")
DEFAULT_CALIBRATION_ID = "brettzone_cage_high"
# NHRL shoots this angle with an iPhone Pro at 1x, about 71.6 degrees horizontal, which at 1920 px
# wide backs out to 960 / tan(35.8 deg). An estimate, not a calibration.
DEFAULT_FOCAL_PX = 1331.0
SAVED_RECORDINGS = Path("data/saved_recordings")
DOWNLOADS = Path("data/downloads")
PLAYBACK_CONFIG_DIR = Path("config/playback")


MONTHS = {
    "jan": 1,
    "feb": 2,
    "mar": 3,
    "apr": 4,
    "may": 5,
    "jun": 6,
    "jul": 7,
    "aug": 8,
    "sep": 9,
    "oct": 10,
    "nov": 11,
    "dec": 12,
}


def infer_set_name(source_dir: Path) -> str:
    """A set name from the clips themselves, so a fetched batch names itself.

    BrettZone exports are ``BZ-<tournament>-<p1>-<p2>-<game>-Cage-<n>-Overhead-High.mp4`` and the
    tournament id carries the event month: ``nhrl_may26`` is NHRL May 2026, not the 26th. Sibling
    sets in data/saved_recordings are named ``<Venue>_<date>``, so this produces
    ``BrettZone_2026-05`` rather than reproducing NHRL's own shorthand.
    """
    for video in sorted(source_dir.glob("*.mp4")):
        match = re.match(r"^BZ-nhrl_([a-z]{3})(\d{2})_", video.name)
        if match and match.group(1) in MONTHS:
            return f"BrettZone_20{match.group(2)}-{MONTHS[match.group(1)]:02d}"
    return f"BrettZone_{source_dir.name}"


def download(args: argparse.Namespace, into: Path) -> None:
    """Run the existing downloader as its own CLI rather than re-implementing it."""
    if not DOWNLOADER.exists():
        raise SystemExit(f"{DOWNLOADER} not found; download the clips yourself and use --from-dir")
    command = [sys.executable, str(DOWNLOADER), str(into)]
    if args.bot:
        command += ["--bot", args.bot]
    if args.tournament:
        command += ["--tournament", args.tournament]
    if args.since:
        command += ["--since", args.since]
    if args.limit:
        command += ["--limit", str(args.limit), "--per-tournament", str(args.limit)]
    print(f"$ {' '.join(command)}")
    result = subprocess.run(command)
    if result.returncode != 0:
        raise SystemExit(f"download_cage_video.py failed with exit code {result.returncode}")


def source_videos(source_dir: Path) -> list[Path]:
    """Top-level clips only.

    ``annotated/`` holds detection-overlay renders from the background-subtraction work. Those are
    derived pictures with boxes burned into them, not camera frames, and converting one would put a
    previous model's opinion into a recording the pipeline is supposed to form its own.
    """
    return sorted(source_dir.glob("*.mp4"))


def write_manifest(
    output_dir: Path, source_dir: Path, results: list[ConversionResult], args: argparse.Namespace
) -> None:
    """Provenance next to the recordings.

    ``data/`` is gitignored, so this file is the only place that says where these came from and
    what was assumed about the camera. Without it a recording set is anonymous in six months.
    """
    total = sum(result.bytes_written for result in results)
    lines = [
        f"# {output_dir.name}",
        "",
        "NHRL fixed-cage fight video re-wrapped as RGB-camera recordings by",
        f"`scripts/nhrl_to_recordings.py` on {datetime.now(timezone.utc):%Y-%m-%d}.",
        "",
        f"- Source clips: `{source_dir}`",
        f"- Camera calibration: `config/cameras/{args.calibration_id}.toml`"
        f" (f = {args.focal_px:.0f} px, estimated from NHRL's stated field of view, not measured)",
        f"- Encode: libx264 crf {args.crf} preset {args.preset}, no B-frames, IDR every 30 frames",
        f"- Playback: `config/playback/{args.calibration_id}.toml`",
        "",
        "Each file carries `/camera/video`, `/camera/frame_meta` and `/camera/camera_info` and",
        "nothing else. Everything else in a real recording is pipeline output, which replay exists",
        "to regenerate.",
        "",
        "| recording | frames | fps | MB |",
        "| --- | --- | --- | --- |",
    ]
    for result in sorted(results, key=lambda item: item.output.name):
        lines.append(
            f"| {result.output.name} | {result.frames} | {result.fps:.2f} |"
            f" {result.bytes_written / (1024 * 1024):.0f} |"
        )
    lines += ["", f"{len(results)} recordings, {total / 1e9:.2f} GB."]
    (output_dir / "MANIFEST.md").write_text("\n".join(lines) + "\n")


def write_playback_config(
    output_dir: Path, results: list[ConversionResult], args: argparse.Namespace
) -> Path:
    """A playback config with every recording listed, the first live and the rest commented out.

    Same shape as `config/playback/_playback.toml`'s SVO list, so switching clips is uncommenting a
    line rather than pasting a path.
    """
    path = PLAYBACK_CONFIG_DIR / f"{args.calibration_id}.toml"
    names = sorted(result.output for result in results)
    entries = []
    for index, output in enumerate(names):
        prefix = "" if index == 0 else "# "
        entries.append(f'{prefix}video_file_path = "{output.as_posix()}"')
    body = f"""# Replay of NHRL's fixed `Cage-N-Overhead-High` footage, re-wrapped into the
# recording layout by scripts/nhrl_to_recordings.py. The nearest thing to the RGB camera that
# exists before the camera does: one fixed 1080p view, no depth, no visual odometry.
#
# Two things this footage cannot give you. The intrinsics in
# config/cameras/{args.calibration_id}.toml are backed out of NHRL's stated field of view rather
# than measured, and the homography path inherits that error linearly, so treat ranges as
# indicative. And the detectors were trained on ZED frames at a different mount, so detections here
# report transfer rather than accuracy.
#
# Regenerated by scripts/nhrl_to_recordings.py; edits to the clip list will not survive a re-run.
extends = "playback/_video_playback"

[rgbd_camera]
{chr(10).join(entries)}
start_frame = 0

[robot_mask_model]
# Without this every detection is classified OTHER and dropped before it becomes a keypoint,
# however confident the model was. debug_visualization draws the raw detections, so the symptom is
# a annotated frame full of boxes and an empty filter.
their_robot_labels = ["OPPONENT"]

[robot_filter.label_mapping]
# Sub-tables merge by key, so OPPONENT and HOUSE_BOT from _common.toml survive this.
"{args.our_robot}" = ["OUR_ROBOT_1"]

[field_filter]
# The outline fit rather than the board: there is no fiducial board in broadcast footage.
type = "HomographyFieldFilter"
field_size_x = 2.35
field_size_y = 2.35
"""
    path.write_text(body)
    return path


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    source = parser.add_argument_group("where the clips come from")
    source.add_argument("--from-dir", type=Path, help="directory of already-downloaded .mp4 clips")
    source.add_argument("--bot", help='fetch every fight for this bot, e.g. "MRS BUFF"')
    source.add_argument("--tournament", help="restrict a --bot fetch to one tournament id")
    source.add_argument("--since", help="fetch fights from tournaments after this date")
    source.add_argument("--limit", type=int, help="cap how many fights a fetch downloads")

    parser.add_argument("--name", help="recording set name (default: inferred from the clips)")
    parser.add_argument("--calibration-id", default=DEFAULT_CALIBRATION_ID)
    parser.add_argument(
        "--our-robot",
        default="MRS_BUFF_MK3",
        help="keypoint label of our robot in these fights, mapped to OUR_ROBOT_1",
    )
    parser.add_argument("--focal-px", type=float, default=DEFAULT_FOCAL_PX)
    parser.add_argument("--crf", type=int, default=DEFAULT_CRF)
    parser.add_argument("--preset", default=DEFAULT_PRESET)
    parser.add_argument(
        "--force", action="store_true", help="re-convert clips that already have a recording"
    )
    parser.add_argument("--no-config", action="store_true", help="skip writing the playback config")
    args = parser.parse_args()

    if shutil.which("ffmpeg") is None or shutil.which("ffprobe") is None:
        print("ffmpeg and ffprobe must be on PATH")
        return 1
    if not args.from_dir and not (args.bot or args.since):
        parser.error("give --from-dir, or --bot/--since to fetch")

    source_dir = args.from_dir
    if source_dir is None:
        source_dir = DOWNLOADS / (args.name or f"nhrl_{datetime.now():%Y%m%d}")
        download(args, source_dir)
    if not source_dir.is_dir():
        print(f"{source_dir} is not a directory")
        return 1

    videos = source_videos(source_dir)
    if not videos:
        print(f"No .mp4 clips in {source_dir}")
        return 1

    set_name = args.name or infer_set_name(source_dir)
    output_dir = SAVED_RECORDINGS / set_name
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"{len(videos)} clip(s) from {source_dir} -> {output_dir}")

    # Stamps are synthetic but plausible, laid out back to back from the set's own epoch so two
    # recordings never share a timestamp.
    stamp_ns = int(datetime(2026, 1, 1).timestamp()) * _NS
    results: list[ConversionResult] = []
    converted = 0
    for video in videos:
        output = output_dir / f"{video.stem}.mcap"
        if output.exists() and not args.force:
            # Counted from the existing recording's summary rather than assumed, so the manifest
            # says the same thing whether a clip was converted now or last week.
            existing = message_counts(output).get(VIDEO_TOPIC, 0)
            print(f"{output.name}: already converted ({existing} frames), skipping")
            results.append(
                ConversionResult(
                    source=video,
                    output=output,
                    frames=existing,
                    width=0,
                    height=0,
                    fps=probe(video).fps,
                    bytes_written=output.stat().st_size,
                )
            )
            continue
        result = convert_video(
            video,
            output,
            calibration_id=args.calibration_id,
            focal_px=args.focal_px,
            start_stamp_ns=stamp_ns,
            crf=args.crf,
            preset=args.preset,
        )
        stamp_ns = next_start_stamp_ns(stamp_ns, result.frames, result.fps)
        results.append(result)
        converted += 1

    write_manifest(output_dir, source_dir, results, args)
    print(f"\n{converted} converted, {len(results) - converted} already present")
    print(f"Manifest: {output_dir / 'MANIFEST.md'}")
    if not args.no_config:
        config = write_playback_config(output_dir, results, args)
        print(f"Playback: ./scripts/build_and_run.sh -c {config}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
