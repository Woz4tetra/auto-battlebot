"""Fetch the NHRL bot roster from BrettZone and cache thumbnails for meshing.

Step 1 of the NHRL robot distractor pipeline.  Pages the BrettZone bots API,
caches the roster locally, ranks bots by activity, downloads their thumbnails,
and records which bots lack a usable thumbnail in the shared state file.  No
Meshy credits are spent here.

Usage:
    python download_nhrl_bots.py ../data/distractor_models/robots --limit 300
    python download_nhrl_bots.py ../data/distractor_models/robots --min-fights 5 --refresh-bot-list
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Any

import nhrl_common as nc
import requests

_CACHE_TTL_SECONDS = 7 * 24 * 3600
_PAGE_LIMIT = 200
_REQUEST_TIMEOUT = 30


def _api_headers(api_key: str | None) -> dict[str, str]:
    headers = {"Accept": "application/json"}
    if api_key:
        headers["X-API-Key"] = api_key
    return headers


def fetch_all_bots(api_key: str | None) -> list[dict[str, Any]]:
    """Page through the BrettZone bots endpoint and return every bot."""
    bots: list[dict[str, Any]] = []
    page = 1
    while True:
        url = f"{nc.BRETTZONE_API_BASE}/bots?page={page}&limit={_PAGE_LIMIT}"
        resp = requests.get(url, headers=_api_headers(api_key), timeout=_REQUEST_TIMEOUT)
        resp.raise_for_status()
        payload = resp.json()
        page_bots = payload.get("bots", [])
        bots.extend(page_bots)
        pagination = payload.get("pagination", {})
        total_pages = int(pagination.get("pages", page))
        print(f"  Fetched page {page}/{total_pages} ({len(page_bots)} bots)")
        if page >= total_pages or not page_bots:
            break
        page += 1
        # Fair-use: stay well under the 5s real-time poll ceiling for archive data.
        time.sleep(0.3)
    return bots


def load_or_fetch_bots(
    cache_path: Path, api_key: str | None, refresh: bool
) -> list[dict[str, Any]]:
    """Return the bot roster from cache when fresh, otherwise fetch and cache it."""
    if not refresh and cache_path.exists():
        age = time.time() - cache_path.stat().st_mtime
        cached = nc.load_json(cache_path)
        if cached is not None and age < _CACHE_TTL_SECONDS:
            bots = cached.get("bots", [])
            print(f"Using cached bot roster: {len(bots)} bots ({age / 3600:.1f}h old)")
            return list(bots)

    print("Fetching bot roster from BrettZone...")
    bots = fetch_all_bots(api_key)
    nc.atomic_write_json(cache_path, {"fetched_at": time.time(), "bots": bots})
    print(f"Cached {len(bots)} bots to {cache_path}")
    return bots


def download_thumbnail(clean_name: str, dest: Path, api_key: str | None) -> bool:
    """Download one bot thumbnail. Returns True on success, False on 404/error."""
    if dest.exists() and dest.stat().st_size > 0:
        return True
    url = nc.bot_thumbnail_url(clean_name)
    try:
        resp = requests.get(url, headers=_api_headers(api_key), timeout=_REQUEST_TIMEOUT)
    except requests.RequestException as e:
        print(f"  Thumbnail request failed for {clean_name}: {e}")
        return False
    if resp.status_code != 200 or not resp.content:
        return False
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_bytes(resp.content)
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dir", type=Path, help="Robot distractor directory")
    parser.add_argument("--limit", type=int, default=300, help="Target robot count (default: 300)")
    parser.add_argument(
        "--min-fights",
        type=int,
        default=1,
        help="Drop bots with fewer than this many fights (default: 1, excludes placeholders)",
    )
    parser.add_argument(
        "--oversample",
        type=float,
        default=0.2,
        help="Fractional overselect to absorb thumbnail 404s (default: 0.2)",
    )
    parser.add_argument(
        "--refresh-bot-list",
        action="store_true",
        help="Force a fresh roster fetch even if the cache is recent",
    )
    parser.add_argument(
        "--brettzone-api-key",
        default=None,
        help="Optional X-API-Key value for BrettZone usage attribution",
    )
    args = parser.parse_args()

    output_dir = nc.resolve_cli_path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    thumb_dir = output_dir / "thumbnails"
    cache_path = output_dir / nc.BOT_CACHE_FILENAME
    state_path = output_dir / nc.STATE_FILENAME

    bots = load_or_fetch_bots(cache_path, args.brettzone_api_key, args.refresh_bot_list)
    selected = nc.select_bots(bots, args.limit, args.min_fights, args.oversample)
    print(
        f"Selected {len(selected)} bots (target {args.limit}, "
        f"+{args.oversample:.0%} oversample) from {len(bots)} total"
    )

    state = nc.load_state(state_path)
    ok = 0
    missing = 0
    rejected = 0
    print(f"\n{'bot':<32}{'weight':<12}{'fights':>7}  thumbnail")
    print("-" * 72)
    for bot in selected:
        clean_name = str(bot["cleanName"])
        name = str(bot.get("name", clean_name))
        wc = nc.weight_class_labels(bot.get("weightClasses", []))
        fights = int(bot.get("totalFights", 0))

        entry = state.get(clean_name, {})
        if nc.is_rejected(entry):
            # Human-rejected thumbnail: never re-download, keep the flag sticky.
            rejected += 1
            state[clean_name] = entry
            print(f"{name[:31]:<32}{'/'.join(wc)[:11]:<12}{fights:>7}  rejected")
            continue

        dest = thumb_dir / f"{clean_name}.png"
        got = download_thumbnail(clean_name, dest, args.brettzone_api_key)

        entry.update(
            {
                "name": name,
                "clean_name": clean_name,
                "weight_classes": wc,
                "total_fights": fights,
                "thumbnail_url": nc.bot_thumbnail_url(clean_name),
                "thumbnail_file": dest.name if got else None,
            }
        )
        if got:
            ok += 1
            entry.setdefault("status", "selected")
        else:
            missing += 1
            entry["status"] = nc.STATUS_SKIPPED_NO_THUMBNAIL
        state[clean_name] = entry

        print(f"{name[:31]:<32}{'/'.join(wc)[:11]:<12}{fights:>7}  {'ok' if got else 'MISSING'}")

    nc.save_state(state_path, state)
    rejected_note = f", {rejected} rejected (skipped)" if rejected else ""
    print(
        f"\nThumbnails: {ok} ok, {missing} missing{rejected_note}. "
        f"Usable bots: {ok} (target {args.limit}).\n"
        f"State written to {state_path}\n"
        f"Next: ../../venv/bin/python review_nhrl_thumbnails.py {args.output_dir}"
    )


if __name__ == "__main__":
    main()
