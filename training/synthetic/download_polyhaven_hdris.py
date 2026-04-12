"""Download a random set of HDRIs from Poly Haven.

Usage:
    python download_polyhaven_hdris.py ../data/hdris --count 20
    python download_polyhaven_hdris.py ../data/hdris --count 30 --resolution 4k --format exr
"""

from __future__ import annotations

import argparse
import json
import random
import shutil
import urllib.request
from pathlib import Path
from urllib.parse import urlsplit

POLYHAVEN_ASSETS_API = "https://api.polyhaven.com/assets?t=hdris"
POLYHAVEN_FILES_API_TEMPLATE = "https://api.polyhaven.com/files/{asset_id}"

DEFAULT_INDOOR_KEYWORDS = [
    "indoor",
    "interior",
    "studio",
    "warehouse",
    "hall",
    "room",
    "garage",
    "arena",
]


def fetch_json(url: str) -> dict:
    """Fetch JSON from a URL."""
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    with urllib.request.urlopen(req) as resp:
        return json.load(resp)


def existing_asset_ids(output_dir: Path) -> set[str]:
    """Infer existing Poly Haven asset ids from local .hdr/.exr files."""
    ids: set[str] = set()
    for ext in ("*.hdr", "*.exr"):
        for file_path in output_dir.glob(ext):
            stem = file_path.stem
            # Typical filenames are like "studio_small_08_2k.hdr".
            # Strip a trailing resolution token when present.
            for suffix in ("_1k", "_2k", "_4k", "_8k", "_16k"):
                if stem.endswith(suffix):
                    stem = stem.removesuffix(suffix)
                    break
            ids.add(stem)
    return ids


def choose_assets(
    assets: dict,
    count: int,
    rng: random.Random,
    keywords: list[str],
    prefer_indoor: bool,
) -> list[str]:
    """Pick random asset ids, optionally preferring indoor-like environments."""
    asset_ids = list(assets.keys())
    if count >= len(asset_ids):
        rng.shuffle(asset_ids)
        return asset_ids

    if not prefer_indoor:
        rng.shuffle(asset_ids)
        return asset_ids[:count]

    keywords_lower = [k.lower() for k in keywords if k.strip()]
    preferred: list[str] = []
    fallback: list[str] = []

    for asset_id in asset_ids:
        meta = assets.get(asset_id, {})
        categories = [str(c).lower() for c in meta.get("categories", [])]
        name = str(meta.get("name", "")).lower()
        searchable = " ".join([asset_id.lower(), name] + categories)
        if any(keyword in searchable for keyword in keywords_lower):
            preferred.append(asset_id)
        else:
            fallback.append(asset_id)

    rng.shuffle(preferred)
    rng.shuffle(fallback)
    selected = preferred[:count]
    if len(selected) < count:
        selected.extend(fallback[: count - len(selected)])
    return selected


def get_download_url(files_metadata: dict, resolution: str, preferred_format: str) -> tuple[str, str]:
    """Resolve a downloadable HDRI URL and extension from Poly Haven file metadata."""
    hdri_section = files_metadata.get("hdri", {})
    resolution_data = hdri_section.get(resolution)
    if not isinstance(resolution_data, dict):
        available = ", ".join(sorted(hdri_section.keys())) or "none"
        raise ValueError(
            f"Requested resolution '{resolution}' unavailable (available: {available})"
        )

    format_order = [preferred_format, "hdr", "exr"]
    seen: set[str] = set()
    for file_format in format_order:
        if file_format in seen:
            continue
        seen.add(file_format)
        fmt_data = resolution_data.get(file_format)
        if isinstance(fmt_data, dict):
            url = fmt_data.get("url")
            if isinstance(url, str) and url:
                return url, file_format

    available_formats = ", ".join(sorted(resolution_data.keys())) or "none"
    raise ValueError(
        f"No '{preferred_format}', 'hdr', or 'exr' URL found (available: {available_formats})"
    )


def download_file(url: str, output_path: Path) -> None:
    """Download a file to disk."""
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    with urllib.request.urlopen(req) as resp, open(output_path, "wb") as out:
        shutil.copyfileobj(resp, out)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "output_dir",
        type=Path,
        help="Directory to store HDRIs (e.g. ../data/hdris)",
    )
    parser.add_argument(
        "--count",
        type=int,
        required=True,
        help="Number of HDRIs to download",
    )
    parser.add_argument(
        "--resolution",
        default="2k",
        choices=["1k", "2k", "4k", "8k", "16k"],
        help="Preferred Poly Haven resolution (default: 2k)",
    )
    parser.add_argument(
        "--format",
        default="hdr",
        choices=["hdr", "exr"],
        help="Preferred file format; falls back to other if unavailable (default: hdr)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed for deterministic selection (default: 42)",
    )
    parser.add_argument(
        "--keywords",
        nargs="+",
        default=DEFAULT_INDOOR_KEYWORDS,
        help=f"Indoor-preference keywords (default: {DEFAULT_INDOOR_KEYWORDS})",
    )
    parser.add_argument(
        "--no-indoor-preference",
        action="store_true",
        help="Disable indoor/arena keyword preference and sample fully at random",
    )
    args = parser.parse_args()

    if args.count <= 0:
        parser.error("--count must be > 0")

    output_dir: Path = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    print("Fetching Poly Haven HDRI index...")
    assets = fetch_json(POLYHAVEN_ASSETS_API)
    if not assets:
        raise RuntimeError("No HDRIs returned by Poly Haven API")

    existing_ids = existing_asset_ids(output_dir)
    available_assets = {
        asset_id: meta for asset_id, meta in assets.items() if asset_id not in existing_ids
    }

    if not available_assets:
        print(f"No new HDRIs to download. Existing library already has {len(existing_ids)} assets.")
        return

    requested = min(args.count, len(available_assets))
    rng = random.Random(args.seed)
    selected_ids = choose_assets(
        available_assets,
        requested,
        rng,
        keywords=args.keywords,
        prefer_indoor=not args.no_indoor_preference,
    )

    print(
        f"Downloading {len(selected_ids)} HDRIs "
        f"(requested={args.count}, available_new={len(available_assets)})..."
    )

    successes = 0
    for index, asset_id in enumerate(selected_ids, start=1):
        print(f"[{index}/{len(selected_ids)}] {asset_id}")
        try:
            files_url = POLYHAVEN_FILES_API_TEMPLATE.format(asset_id=asset_id)
            files_metadata = fetch_json(files_url)
            download_url, chosen_format = get_download_url(
                files_metadata,
                resolution=args.resolution,
                preferred_format=args.format,
            )

            filename = Path(urlsplit(download_url).path).name or f"{asset_id}_{args.resolution}.{chosen_format}"
            output_path = output_dir / filename

            if output_path.exists():
                print(f"  Skipping existing file: {output_path.name}")
                successes += 1
                continue

            print(f"  Downloading {filename} ...")
            download_file(download_url, output_path)
            successes += 1
        except Exception as exc:  # noqa: BLE001 - CLI script should continue on per-asset errors
            print(f"  ERROR: {exc}")

    print(f"\nDone. Downloaded {successes}/{len(selected_ids)} HDRIs to {output_dir}")


if __name__ == "__main__":
    main()
