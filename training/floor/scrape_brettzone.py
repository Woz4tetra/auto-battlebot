#!/usr/bin/env python3
"""Scrape BrettZone fights and download random fight videos."""

from __future__ import annotations

import argparse
import json
import random
import re
from dataclasses import asdict
from enum import Enum
from pathlib import Path
from urllib.parse import parse_qs, urljoin, urlparse

import requests

from common import (
    LOGGER,
    append_jsonl,
    choose_deterministic,
    configure_logging,
    json_load,
    json_dump,
    load_pipeline_config,
    mkdir,
    now_ts,
    sha256_file,
)

try:
    from bs4 import BeautifulSoup
except ModuleNotFoundError:  # pragma: no cover
    BeautifulSoup = None  # type: ignore


FIGHT_RE = re.compile(
    r"(fightReviewSync\.php\?gameID=[^&]+&tournamentID=[^\"'<> ]+)", re.IGNORECASE
)


class FightStatus(str, Enum):
    DONE = "done"
    DUPLICATE_HASH = "duplicate_hash"
    ALREADY_SEEN = "already_seen"
    DOWNLOAD_NONE = "download_none"
    EXTRACT_FAILED = "extract_failed"
    NO_ENTRIES = "no_entries"
    DOWNLOAD_FAILED = "download_failed"


SKIP_STATUSES: set[FightStatus] = {
    FightStatus.DONE,
    FightStatus.DUPLICATE_HASH,
    FightStatus.ALREADY_SEEN,
    FightStatus.DOWNLOAD_NONE,
}


def parse_fight_status(value: str | None) -> FightStatus | None:
    if value is None:
        return None
    try:
        return FightStatus(value)
    except ValueError:
        return None


def fetch_html(url: str, timeout: int, retries: int = 1) -> str:
    last_exc: Exception | None = None
    for attempt in range(1, retries + 1):
        try:
            response = requests.get(url, timeout=timeout)
            response.raise_for_status()
            return response.text
        except requests.RequestException as exc:
            last_exc = exc
            LOGGER.warning("Request attempt %d/%d failed for %s: %s", attempt, retries, url, exc)
    if last_exc is None:
        raise RuntimeError(f"Failed to fetch {url}")
    raise last_exc


def discover_fight_links(base_url: str, html: str) -> list[str]:
    links: set[str] = set()
    if BeautifulSoup is not None:
        soup = BeautifulSoup(html, "lxml")
        for anchor in soup.select("a[href]"):
            href = anchor["href"].strip()
            if "fightReviewSync.php?gameID=" in href:
                links.add(urljoin(base_url, href))

    for match in FIGHT_RE.finditer(html):
        links.add(urljoin(base_url, match.group(1)))

    return sorted(links)


def parse_fight_identifiers(url: str) -> dict[str, str]:
    query = parse_qs(urlparse(url).query)
    return {
        "game_id": query.get("gameID", ["unknown"])[0],
        "tournament_id": query.get("tournamentID", ["unknown"])[0],
    }


def _extract_recordings_from_html(html: str) -> list[dict]:
    """Extract recording objects from window.MATCH_DATA in fight pages."""
    match = re.search(r"recordings\s*:\s*(\[.*?\])\s*,\s*gameID\s*:", html, re.S)
    if not match:
        return []
    try:
        recordings = json.loads(match.group(1))
    except json.JSONDecodeError:
        return []
    return [r for r in recordings if isinstance(r, dict)]


def _recording_to_entry(recording: dict, game_id: str, tournament_id: str) -> dict | None:
    # Prefer manageable resolutions first, then native source.
    url = recording.get("proxy720") or recording.get("proxy360") or recording.get("s3path")
    if not isinstance(url, str) or not url:
        return None
    camera = recording.get("camera") or "unknown_camera"
    category = recording.get("category") or "unknown_category"
    title = f"{tournament_id}_{game_id}_{camera}"
    return {
        "url": url,
        "title": title,
        "camera": camera,
        "category": category,
    }


def _extract_mp4_urls(html: str) -> list[str]:
    urls = re.findall(r"https:\\/\\/[^\"'\\s>]+\\.mp4", html)
    normalized = [u.replace("\\/", "/") for u in urls]
    return sorted(set(normalized))


def list_downloadables(
    fight_url: str,
    timeout: int,
    retries: int,
    allow_all_camera_angles: bool,
) -> list[dict]:
    html = fetch_html(fight_url, timeout=timeout, retries=retries)
    ids = parse_fight_identifiers(fight_url)
    recordings = _extract_recordings_from_html(html)
    entries: list[dict] = []
    for rec in recordings:
        entry = _recording_to_entry(
            recording=rec,
            game_id=ids["game_id"],
            tournament_id=ids["tournament_id"],
        )
        if entry is None:
            continue
        if not allow_all_camera_angles:
            category = str(entry.get("category", "")).lower()
            camera = str(entry.get("camera", "")).lower()
            if category not in {"overhead", "program"} and "overhead" not in camera:
                continue
        entries.append(entry)

    if not entries:
        # Older/variant fight pages may not expose MATCH_DATA recordings.
        # Fallback to direct MP4 URL extraction.
        fallback_urls = _extract_mp4_urls(html)
        for idx, media_url in enumerate(fallback_urls):
            entries.append(
                {
                    "url": media_url,
                    "title": f"{ids['tournament_id']}_{ids['game_id']}_fallback_{idx:02d}",
                    "camera": "fallback",
                    "category": "fallback",
                }
            )
    return entries


def download_entry(entry: dict, out_dir: Path, timeout: int, retries: int) -> Path | None:
    url = entry["url"]
    title = entry.get("title") or "fight"
    safe_title = re.sub(r"[^A-Za-z0-9._-]+", "_", title).strip("_")
    output_path = out_dir / f"{safe_title}.mp4"
    temp_path = out_dir / f"{safe_title}.part"

    last_exc: Exception | None = None
    for attempt in range(1, retries + 1):
        try:
            with requests.get(url, timeout=timeout, stream=True) as response:
                response.raise_for_status()
                with temp_path.open("wb") as f:
                    for chunk in response.iter_content(chunk_size=1024 * 1024):
                        if chunk:
                            f.write(chunk)
            temp_path.replace(output_path)
            return output_path
        except requests.RequestException as exc:
            last_exc = exc
            LOGGER.warning(
                "Download attempt %d/%d failed for %s: %s",
                attempt,
                retries,
                url,
                exc,
            )
    if temp_path.exists():
        temp_path.unlink(missing_ok=True)
    if last_exc is None:
        return None
    raise last_exc


def main() -> None:
    parser = argparse.ArgumentParser(description="Scrape and download BrettZone fight videos")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).parent / "config" / "pipeline.toml",
    )
    parser.add_argument("--limit", type=int, default=None, help="Override max fights")
    parser.add_argument("--seed", type=int, default=None, help="Override random seed")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    configure_logging(args.verbose)
    cfg = load_pipeline_config(args.config)
    scrape_cfg = cfg.scrape
    max_fights = args.limit if args.limit is not None else scrape_cfg.max_fights
    seed = args.seed if args.seed is not None else scrape_cfg.random_seed

    mkdir(cfg.paths.raw_videos)
    mkdir(cfg.paths.metadata_dir)
    state_file = cfg.paths.metadata_dir / "scrape_state.json"
    index_file = cfg.paths.metadata_dir / "videos_index.jsonl"
    prior_state = json_load(state_file, default={"downloaded_sha256": {}, "fight_urls": []})
    downloaded_sha = prior_state.get("downloaded_sha256", {})
    downloaded_media_urls = set(prior_state.get("downloaded_media_urls", []))
    processed_fights: dict[str, str] = dict(prior_state.get("processed_fights", {}))

    LOGGER.info("Fetching BrettZone pages for fight discovery")
    pages = [
        scrape_cfg.index_url,
        scrape_cfg.robot_fights_url,
    ]
    all_fights: set[str] = set()
    for page in pages:
        try:
            html = fetch_html(page, scrape_cfg.timeout_seconds, retries=scrape_cfg.retries)
            all_fights.update(discover_fight_links(scrape_cfg.base_url, html))
        except requests.RequestException as exc:
            LOGGER.warning("Failed to fetch %s: %s", page, exc)

    if not all_fights:
        raise RuntimeError("No fight links discovered from BrettZone pages")

    fights = sorted(all_fights)
    selected_fights = choose_deterministic(fights, max_fights, seed)
    LOGGER.info("Discovered %d fights, selected %d", len(fights), len(selected_fights))

    downloaded_rows: list[dict] = []
    rng = random.Random(seed)
    for idx, fight_url in enumerate(selected_fights, start=1):
        fight_ids = parse_fight_identifiers(fight_url)
        LOGGER.info("[%d/%d] Processing %s", idx, len(selected_fights), fight_url)

        prior_status = parse_fight_status(processed_fights.get(fight_url))
        if prior_status in SKIP_STATUSES:
            LOGGER.info(
                "Fight already processed in prior run (%s), skipping: %s",
                prior_status.value if prior_status else "unknown",
                fight_url,
            )
            continue

        try:
            entries = list_downloadables(
                fight_url=fight_url,
                timeout=scrape_cfg.timeout_seconds,
                retries=scrape_cfg.retries,
                allow_all_camera_angles=scrape_cfg.allow_all_camera_angles,
            )
        except Exception as exc:  # noqa: BLE001
            LOGGER.warning("Failed to extract media for %s: %s", fight_url, exc)
            processed_fights[fight_url] = FightStatus.EXTRACT_FAILED.value
            continue

        if not entries:
            LOGGER.warning("No downloadable entries for %s", fight_url)
            processed_fights[fight_url] = FightStatus.NO_ENTRIES.value
            continue

        unseen_entries = [e for e in entries if e.get("url") not in downloaded_media_urls]
        if not unseen_entries:
            LOGGER.info("All media URLs already seen for %s, skipping download", fight_url)
            processed_fights[fight_url] = FightStatus.ALREADY_SEEN.value
            continue

        entry = rng.choice(unseen_entries)
        try:
            video_path = download_entry(
                entry,
                cfg.paths.raw_videos,
                timeout=scrape_cfg.timeout_seconds,
                retries=scrape_cfg.retries,
            )
        except Exception as exc:  # noqa: BLE001
            LOGGER.warning("Download failed for %s: %s", fight_url, exc)
            processed_fights[fight_url] = FightStatus.DOWNLOAD_FAILED.value
            continue
        if video_path is None:
            processed_fights[fight_url] = FightStatus.DOWNLOAD_NONE.value
            continue

        digest = sha256_file(video_path)
        media_url = str(entry.get("url", ""))
        if digest in downloaded_sha:
            LOGGER.info("Duplicate content hash, removing %s", video_path.name)
            video_path.unlink(missing_ok=True)
            if media_url:
                downloaded_media_urls.add(media_url)
            processed_fights[fight_url] = FightStatus.DUPLICATE_HASH.value
            continue

        downloaded_sha[digest] = str(video_path)
        if media_url:
            downloaded_media_urls.add(media_url)
        processed_fights[fight_url] = FightStatus.DONE.value
        row = {
            "timestamp": now_ts(),
            "fight_url": fight_url,
            "download_path": str(video_path),
            "sha256": digest,
            "title": entry.get("title"),
            "duration": entry.get("duration"),
            "camera": entry.get("camera"),
            "category": entry.get("category"),
            "media_url": entry.get("url"),
            **fight_ids,
        }
        downloaded_rows.append(row)
        append_jsonl(index_file, [row])

    next_state = {
        "config": asdict(scrape_cfg),
        "fight_urls": selected_fights,
        "downloaded_sha256": downloaded_sha,
        "downloaded_media_urls": sorted(downloaded_media_urls),
        "processed_fights": processed_fights,
        "downloaded_count": len(downloaded_rows),
        "updated_at": now_ts(),
    }
    json_dump(state_file, next_state)
    LOGGER.info("Done. Newly downloaded videos: %d", len(downloaded_rows))


if __name__ == "__main__":
    main()
