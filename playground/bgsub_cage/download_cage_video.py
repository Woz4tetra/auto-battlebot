"""Download NHRL fixed overhead cage video from BrettZone as 1080p clips.

Every recording in this repo comes from a moving ZED, so background subtraction
cannot be evaluated on it. This pulls the one NHRL camera that never moves:
``Cage-N-Overhead-High``, a near top-down view of the cage floor.

The source object on Linode is 3840x2160 at 59.94 fps and about 1.4 GB per fight.
It serves byte ranges, so ffmpeg seeks straight to the fight window and scales to
1080p in the same pass. The 4K never lands on disk.

Output names match the BrettZone export pattern that
``training/deeplab/field_labels.py`` already parses::

    BZ-<tournamentID>-<p1clean>-<p2clean>-<gameID>-Cage-<n>-Overhead-High.mp4

Usage:
    python download_cage_video.py OUTPUT_DIR --limit 20 --seed 0 --since 2026-04-01
    python download_cage_video.py OUTPUT_DIR --dry-run
"""

from __future__ import annotations

import argparse
import json
import random
import re
import shutil
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import requests

API_BASE = "https://brettzone.nhrl.io/brettZone/api.php"
REQUEST_TIMEOUT = 30
# The docs ask for no more than one request every 5 s against live endpoints. The
# archive endpoints used here carry no such limit, but a small gap keeps us polite.
REQUEST_GAP_S = 0.25

OVERHEAD_RE = re.compile(r"^Cage-(\d+)-Overhead-High$")
NULL_DATE_PREFIX = "0000"
DATE_FIELDS = ("endTime", "startTime", "scheduledStartTime", "createTime")

# Pre-roll before the fight starts and post-roll after it ends, in seconds. The
# pre-roll catches the countdown with both robots parked, which is the cleanest
# background frame a clip contains.
PRE_ROLL_S = 3.0
POST_ROLL_S = 5.0

MANIFEST_JSON = "manifest.json"
MANIFEST_MD = "MANIFEST.md"


@dataclass
class Clip:
    """One downloaded fight, as recorded in the manifest."""

    tournament_id: str
    tournament_name: str
    game_id: str
    player1: str
    player2: str
    weight_class: int
    camera: str
    cage: str
    match_length_s: int
    fight_start_offset_s: float
    clip_start_s: float
    clip_duration_s: float
    source_url: str
    filename: str
    bytes_written: int = 0


def api_get(path: str, api_key: str | None) -> dict[str, Any]:
    headers = {"Accept": "application/json"}
    if api_key:
        headers["X-API-Key"] = api_key
    response = requests.get(f"{API_BASE}{path}", headers=headers, timeout=REQUEST_TIMEOUT)
    response.raise_for_status()
    time.sleep(REQUEST_GAP_S)
    return response.json()


def tournament_date(tournament: dict[str, Any]) -> str:
    """Best available date string, since most fields are '0000-00-00 00:00:00'."""
    for field in DATE_FIELDS:
        value = tournament.get(field) or ""
        if value and not value.startswith(NULL_DATE_PREFIX):
            return value
    return ""


def select_tournaments(api_key: str | None, since: str) -> list[dict[str, Any]]:
    payload = api_get("/tournaments", api_key)
    chosen = [
        tournament
        for tournament in payload["tournaments"]
        if tournament_date(tournament) >= since
        and tournament.get("privacy") == "public"
        and not tournament.get("isTest")
    ]
    chosen.sort(key=tournament_date)
    return chosen


def select_fights(
    tournaments: list[dict[str, Any]],
    api_key: str | None,
    min_length_s: int,
    min_cams: int,
) -> list[dict[str, Any]]:
    """Every fight long enough and filmed by enough cameras to carry the overhead feed."""
    fights: list[dict[str, Any]] = []
    for tournament in tournaments:
        tournament_id = tournament["tournamentID"]
        try:
            payload = api_get(f"/tournaments/{tournament_id}/fights", api_key)
        except requests.HTTPError as error:
            print(f"  {tournament_id}: fight list failed ({error}), skipping")
            continue
        kept = [
            fight
            for fight in payload.get("fights", [])
            if (fight.get("matchLength") or 0) >= min_length_s
            and (fight.get("cams") or 0) >= min_cams
            and fight.get("id")
        ]
        for fight in kept:
            fight["_tournamentName"] = tournament["tournamentName"]
        print(f"  {tournament_id}: {len(kept)} eligible fights")
        fights.extend(kept)
    return fights


def bot_fights(
    bot: str,
    api_key: str | None,
    tournament_prefix: str | None,
    min_length_s: int,
) -> list[dict[str, Any]]:
    """Every public fight for one bot, newest first, optionally one event.

    The bot endpoint keys the game as `gameID` where the tournament endpoint uses `id`;
    `build_clip` accepts either. `cams` is not filtered here: a bot's own fights are the
    whole point of the run, so a thin camera list is reported rather than dropped.
    """
    payload = api_get(f"/bots/{bot}/fights", api_key)
    fights = payload.get("fights", []) if isinstance(payload, dict) else payload
    kept = []
    for fight in fights:
        tournament_id = fight.get("tournamentID") or ""
        if tournament_prefix and not tournament_id.startswith(tournament_prefix):
            continue
        if (fight.get("matchLength") or 0) < min_length_s:
            continue
        fight["_tournamentName"] = fight.get("tournamentName", "")
        kept.append(fight)
    return kept


def sample_fights(
    fights: list[dict[str, Any]], limit: int, per_tournament: int, seed: int
) -> list[dict[str, Any]]:
    """Seeded draw, capped per tournament so one event cannot dominate the set."""
    rng = random.Random(seed)
    shuffled = fights[:]
    rng.shuffle(shuffled)
    picked: list[dict[str, Any]] = []
    counts: dict[str, int] = {}
    for fight in shuffled:
        tournament_id = fight["tournamentID"]
        if counts.get(tournament_id, 0) >= per_tournament:
            continue
        picked.append(fight)
        counts[tournament_id] = counts.get(tournament_id, 0) + 1
        if len(picked) == limit:
            break
    return picked


def find_overhead(recordings: list[dict[str, Any]]) -> dict[str, Any] | None:
    for recording in recordings:
        if OVERHEAD_RE.match(recording.get("camera") or "") and recording.get("s3http") == "200":
            return recording
    return None


def clean_name(value: str) -> str:
    """BrettZone's own slug rule: lowercase, drop everything non-alphanumeric."""
    return re.sub(r"[^a-z0-9]", "", (value or "").lower()) or "unknown"


def pick_encoder() -> list[str]:
    """h264_nvenc when the box has it, libx264 otherwise."""
    probe = subprocess.run(
        ["ffmpeg", "-hide_banner", "-encoders"], capture_output=True, text=True, check=False
    )
    if "h264_nvenc" in probe.stdout:
        return ["-c:v", "h264_nvenc", "-preset", "p5", "-cq", "21"]
    return ["-c:v", "libx264", "-crf", "20", "-preset", "medium"]


def download_clip(clip: Clip, destination: Path, encoder: list[str]) -> bool:
    """Range-seek the 4K source, scale to 1080p, write one clip. True on success."""
    command = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-nostdin",
        "-ss",
        f"{clip.clip_start_s:.3f}",
        "-i",
        clip.source_url,
        "-t",
        f"{clip.clip_duration_s:.3f}",
        "-vf",
        "scale=1920:1080:flags=lanczos",
        *encoder,
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        "-an",
        "-y",
        str(destination),
    ]
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    if result.returncode != 0 or not destination.exists():
        print(f"    ffmpeg failed: {result.stderr.strip()[:300]}")
        destination.unlink(missing_ok=True)
        return False
    clip.bytes_written = destination.stat().st_size
    return True


def build_clip(fight: dict[str, Any], api_key: str | None) -> Clip | None:
    """Resolve one fight to an overhead recording and a fight-window clip spec."""
    tournament_id = fight["tournamentID"]
    game_id = fight.get("id") or fight["gameID"]
    try:
        detail = api_get(f"/tournaments/{tournament_id}/fights/{game_id}", api_key)["fight"]
    except (requests.HTTPError, KeyError) as error:
        print(f"  {tournament_id}/{game_id}: detail failed ({error})")
        return None

    recording = find_overhead(detail.get("recordings", []))
    if recording is None:
        print(f"  {tournament_id}/{game_id}: no reachable Cage-N-Overhead-High")
        return None

    offset = float(detail.get("fightStartOffset") or 0.0)
    match_length = int(fight.get("matchLength") or 0)
    start = max(0.0, offset - PRE_ROLL_S)
    duration = match_length + POST_ROLL_S + (offset - start)

    camera = recording["camera"]
    player1 = clean_name(fight.get("player1clean") or fight.get("player1"))
    player2 = clean_name(fight.get("player2clean") or fight.get("player2"))
    filename = f"BZ-{tournament_id}-{player1}-{player2}-{game_id}-{camera}.mp4"

    return Clip(
        tournament_id=tournament_id,
        tournament_name=fight.get("_tournamentName", ""),
        game_id=game_id,
        player1=fight.get("player1") or "",
        player2=fight.get("player2") or "",
        weight_class=int(fight.get("weightClass") or 0),
        camera=camera,
        cage=recording.get("cage") or "",
        match_length_s=match_length,
        fight_start_offset_s=offset,
        clip_start_s=start,
        clip_duration_s=duration,
        source_url=recording["s3path"],
        filename=filename,
    )


def corpus_tournaments(tarball: Path) -> list[str] | None:
    """Tournament IDs baked into the detector's training corpus, for the overlap note.

    The 2class models were trained on nhrl_robots_bbox_2class, whose train split holds
    most of the labelled cage-high scenes. Listing the tarball is the only way to learn
    which tournaments those came from without extracting 12 GB.

    Returns None when the tarball is missing, an empty list when it holds no
    BrettZone-style names. Those are different answers and the manifest says which.
    """
    if not tarball.exists():
        return None
    listing = subprocess.run(
        # `tar tf` without an explicit -z: GNU tar sniffs the compression, and the
        # shipped nhrl_robots_bbox_2class.tar.gz is in fact an uncompressed tar.
        f"tar tf {tarball} | grep -oE 'tournamentID-[a-z0-9_]+' | sort -u",
        shell=True,
        capture_output=True,
        text=True,
        check=False,
    )
    return [line.removeprefix("tournamentID-") for line in listing.stdout.split() if line]


def write_manifest(output_dir: Path, clips: list[Clip], args: argparse.Namespace) -> None:
    (output_dir / MANIFEST_JSON).write_text(
        json.dumps([asdict(clip) for clip in clips], indent=2) + "\n"
    )

    overlap_note = "Not checked (`--skip-corpus-check`)."
    if not args.skip_corpus_check:
        corpus = corpus_tournaments(Path(args.corpus_tarball))
        if corpus is None:
            overlap_note = f"Tarball not found at `{args.corpus_tarball}`, overlap unknown."
        elif not corpus:
            overlap_note = (
                f"`{args.corpus_tarball}` carries no `tournamentID-` filenames, so the "
                "overlap could not be read off the corpus."
            )
        else:
            downloaded = {clip.tournament_id for clip in clips}
            shared = sorted(downloaded & set(corpus))
            overlap_note = f"{len(corpus)} tournaments appear in the training corpus. " + (
                f"**{len(shared)} of them are in this download: {', '.join(shared)}.**"
                if shared
                else "None of them are in this download."
            )

    total_bytes = sum(clip.bytes_written for clip in clips)
    lines = [
        "# BrettZone fixed overhead cage clips",
        "",
        f"{len(clips)} fights from NHRL's `Cage-N-Overhead-High` camera, the one NHRL angle that",
        "does not move. Downloaded to give background subtraction a stationary-camera test set,",
        "which no recording already in this repo provides.",
        "",
        "## How these were made",
        "",
        "```bash",
        f"venv/bin/python playground/bgsub_cage/download_cage_video.py {output_dir} \\",
        f"    --limit {args.limit} --seed {args.seed} --since {args.since} \\",
        f"    --per-tournament {args.per_tournament} --min-length {args.min_length}",
        "```",
        "",
        "Source objects are 3840x2160 at 59.94 fps on",
        "`nhrl-matches.us-east-1.linodeobjects.com`, about 1.4 GB per fight. ffmpeg range-seeks to",
        "the fight window and scales to 1920x1080 in one pass, so the 4K is never stored. The",
        f"window runs from {PRE_ROLL_S:.0f} s before `fightStartOffset` to "
        f"{POST_ROLL_S:.0f} s after the",
        "match ends. The pre-roll is the countdown, with both robots parked in their",
        "squares: the cleanest background frames the clip contains.",
        "",
        "## Detector overlap",
        "",
        overlap_note,
        "",
        "The `yolo26*_nhrl_robots_bbox_2class` models were trained on cage-high scenes from this",
        "same archive. Any tournament listed above as shared means detections on that clip report",
        "training fit rather than detection.",
        "",
        "## Clips",
        "",
        f"Total {total_bytes / 1e9:.2f} GB.",
        "",
        "| # | tournament | fight | robots | class | camera | fight s | clip s | MB |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for index, clip in enumerate(clips, start=1):
        lines.append(
            f"| {index} | {clip.tournament_id} | {clip.game_id} | "
            f"{clip.player1} vs {clip.player2} | {clip.weight_class} lb | {clip.camera} | "
            f"{clip.match_length_s} | {clip.clip_duration_s:.0f} | "
            f"{clip.bytes_written / 1e6:.0f} |"
        )
    lines.append("")
    (output_dir / MANIFEST_MD).write_text("\n".join(lines))


def fetch_one(clip: Clip, output_dir: Path, encoder: list[str], index: int, dry_run: bool) -> bool:
    """Report, then download unless the clip already exists or this is a dry run."""
    destination = output_dir / clip.filename
    if dry_run:
        print(
            f"[{index:2d}] {clip.filename}\n"
            f"      {clip.clip_duration_s:.0f} s from {clip.clip_start_s:.1f} s\n"
            f"      {clip.source_url}"
        )
        return True
    if destination.exists() and destination.stat().st_size > 0:
        clip.bytes_written = destination.stat().st_size
        print(f"[{index:2d}] {clip.filename} (exists, {clip.bytes_written / 1e6:.0f} MB)")
        return True
    print(f"[{index:2d}] {clip.filename} ({clip.clip_duration_s:.0f} s)...")
    if not download_clip(clip, destination, encoder):
        return False
    print(f"      {clip.bytes_written / 1e6:.0f} MB")
    return True


def gather_clips(
    candidates: list[dict[str, Any]],
    output_dir: Path,
    encoder: list[str],
    args: argparse.Namespace,
) -> list[Clip]:
    """Walk the oversampled candidates until `--limit` clips are in hand."""
    clips: list[Clip] = []
    counts: dict[str, int] = {}
    for fight in candidates:
        if len(clips) == args.limit:
            break
        tournament_id = fight["tournamentID"]
        if counts.get(tournament_id, 0) >= args.per_tournament:
            continue
        clip = build_clip(fight, args.brettzone_api_key)
        if clip is None or not fetch_one(clip, output_dir, encoder, len(clips) + 1, args.dry_run):
            continue
        clips.append(clip)
        counts[tournament_id] = counts.get(tournament_id, 0) + 1
    return clips


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("output_dir", type=Path, help="Directory to write clips into")
    parser.add_argument("--limit", type=int, default=20, help="Number of clips (default 20)")
    parser.add_argument("--seed", type=int, default=0, help="Sampling seed (default 0)")
    parser.add_argument(
        "--since",
        default="2026-04-01",
        help="Only tournaments on or after this date. Recent events are least likely to be in "
        "the detector's training corpus (default 2026-04-01)",
    )
    parser.add_argument(
        "--per-tournament", type=int, default=3, help="Max clips per tournament (default 3)"
    )
    parser.add_argument(
        "--min-length", type=int, default=45, help="Skip fights shorter than this (default 45 s)"
    )
    parser.add_argument(
        "--min-cams", type=int, default=10, help="Skip fights with fewer cameras (default 10)"
    )
    parser.add_argument(
        "--bot",
        default=None,
        help="BrettZone cleanName (e.g. mrsbuff). Takes every fight this bot appears in "
        "instead of sampling tournaments, so --limit/--seed/--per-tournament are ignored",
    )
    parser.add_argument(
        "--tournament",
        default=None,
        help="Restrict to tournament IDs starting with this (e.g. nhrl_may26)",
    )
    parser.add_argument("--brettzone-api-key", default=None, help="Optional X-API-Key header")
    parser.add_argument(
        "--corpus-tarball",
        default="training/data/nhrl_robots_bbox_2class.tar.gz",
        help="Training corpus tarball, listed once for the overlap note",
    )
    parser.add_argument(
        "--skip-corpus-check", action="store_true", help="Skip the slow tarball listing"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the chosen fights and exit without downloading",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if shutil.which("ffmpeg") is None:
        print("ffmpeg not found on PATH")
        return 1

    if args.bot:
        print(
            f"Listing fights for {args.bot}" + (f" in {args.tournament}" if args.tournament else "")
        )
        candidates = bot_fights(args.bot, args.brettzone_api_key, args.tournament, args.min_length)
        print(f"{len(candidates)} fights")
        if not candidates:
            print("No fights matched. Check the cleanName with /search?q=<name>.")
            return 1
        # Every one of them is wanted, so the caps that shape a random sample do not apply.
        args.limit = len(candidates)
        args.per_tournament = len(candidates)
    else:
        print(f"Listing tournaments since {args.since}...")
        tournaments = select_tournaments(args.brettzone_api_key, args.since)
        print(f"{len(tournaments)} public tournaments")
        if not tournaments:
            print("Nothing to do. Try an earlier --since.")
            return 1

        fights = select_fights(tournaments, args.brettzone_api_key, args.min_length, args.min_cams)
        print(f"{len(fights)} eligible fights across {len(tournaments)} tournaments")

        # Oversample so fights whose overhead recording is missing can be replaced without
        # a second pass over the API.
        candidates = sample_fights(fights, args.limit * 3, args.per_tournament * 3, args.seed)

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    encoder = pick_encoder()
    print(f"Encoder: {' '.join(encoder)}")

    clips = gather_clips(candidates, output_dir, encoder, args)

    if args.dry_run:
        print(f"\nDry run: {len(clips)} fights selected, nothing downloaded.")
        return 0

    if len(clips) < args.limit:
        print(f"\nOnly {len(clips)} of {args.limit} clips downloaded.")

    print("\nWriting manifest...")
    write_manifest(output_dir, clips, args)
    total = sum(clip.bytes_written for clip in clips)
    print(f"{len(clips)} clips, {total / 1e9:.2f} GB in {output_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
