#!/usr/bin/env python3
"""Scrape BrettZone fights and download random fight videos."""

from __future__ import annotations

import argparse
import random
import re
from dataclasses import asdict
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


def fetch_html(url: str, timeout: int) -> str:
    response = requests.get(url, timeout=timeout)
    response.raise_for_status()
    return response.text


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


def list_downloadables(fight_url: str) -> list[dict]:
    try:
        from yt_dlp import YoutubeDL
    except ModuleNotFoundError as exc:  # pragma: no cover
        raise RuntimeError(
            "yt-dlp is required for scraping/downloading BrettZone videos. "
            "Install with: pip install -e '.[floor]'"
        ) from exc
    ydl_opts = {
        "quiet": True,
        "no_warnings": True,
        "skip_download": True,
        "extract_flat": False,
    }
    with YoutubeDL(ydl_opts) as ydl:
        info = ydl.extract_info(fight_url, download=False)
    if info is None:
        return []

    entries = info.get("entries")
    if isinstance(entries, list):
        return [e for e in entries if isinstance(e, dict)]
    return [info]


def download_entry(entry: dict, out_dir: Path) -> Path | None:
    try:
        from yt_dlp import YoutubeDL
    except ModuleNotFoundError as exc:  # pragma: no cover
        raise RuntimeError(
            "yt-dlp is required for scraping/downloading BrettZone videos. "
            "Install with: pip install -e '.[floor]'"
        ) from exc
    title = entry.get("title") or entry.get("id") or "fight"
    safe_title = re.sub(r"[^A-Za-z0-9._-]+", "_", title).strip("_")
    output_tpl = str(out_dir / f"{safe_title}.%(ext)s")
    ydl_opts = {
        "quiet": True,
        "no_warnings": True,
        "format": "bv*[height<=720]+ba/b[height<=720]/best",
        "merge_output_format": "mp4",
        "outtmpl": output_tpl,
    }
    with YoutubeDL(ydl_opts) as ydl:
        ydl.download([entry["webpage_url"] if "webpage_url" in entry else entry["url"]])

    matches = sorted(out_dir.glob(f"{safe_title}.*"))
    if not matches:
        return None
    return matches[-1]


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

    LOGGER.info("Fetching BrettZone pages for fight discovery")
    pages = [
        scrape_cfg.index_url,
        scrape_cfg.robot_fights_url,
    ]
    all_fights: set[str] = set()
    for page in pages:
        try:
            html = fetch_html(page, scrape_cfg.timeout_seconds)
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
        try:
            entries = list_downloadables(fight_url)
        except Exception as exc:  # noqa: BLE001
            LOGGER.warning("Failed to extract media for %s: %s", fight_url, exc)
            continue

        if not entries:
            LOGGER.warning("No downloadable entries for %s", fight_url)
            continue

        entry = rng.choice(entries)
        try:
            video_path = download_entry(entry, cfg.paths.raw_videos)
        except Exception as exc:  # noqa: BLE001
            LOGGER.warning("Download failed for %s: %s", fight_url, exc)
            continue
        if video_path is None:
            continue

        digest = sha256_file(video_path)
        if digest in downloaded_sha:
            LOGGER.info("Duplicate content hash, removing %s", video_path.name)
            video_path.unlink(missing_ok=True)
            continue

        downloaded_sha[digest] = str(video_path)
        row = {
            "timestamp": now_ts(),
            "fight_url": fight_url,
            "download_path": str(video_path),
            "sha256": digest,
            "title": entry.get("title"),
            "duration": entry.get("duration"),
            "camera": entry.get("format_note") or entry.get("channel"),
            **fight_ids,
        }
        downloaded_rows.append(row)
        append_jsonl(index_file, [row])

    next_state = {
        "config": asdict(scrape_cfg),
        "fight_urls": selected_fights,
        "downloaded_sha256": downloaded_sha,
        "downloaded_count": len(downloaded_rows),
        "updated_at": now_ts(),
    }
    json_dump(state_file, next_state)
    LOGGER.info("Done. Newly downloaded videos: %d", len(downloaded_rows))


if __name__ == "__main__":
    main()
