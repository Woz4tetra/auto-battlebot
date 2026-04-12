"""Download PBR textures from ambientCG for use with BlenderProc.

Downloads zip files, extracts them into the cc_textures folder structure
that BlenderProc's load_ccmaterials() expects, and removes files that
BlenderProc doesn't use.

Usage:
    python download_ambientcg.py data/cc_textures Plastic007 Metal012 Rubber001
    python download_ambientcg.py data/cc_textures "https://ambientcg.com/get?file=Plastic007_2K-JPG.zip"
    python download_ambientcg.py data/cc_textures --from-config config.toml
    python download_ambientcg.py data/cc_textures --list-random 20
"""

import argparse
import json
import random
import shutil
import tempfile
import urllib.request
import zipfile
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import tomllib

AMBIENTCG_URL_TEMPLATE = "https://ambientcg.com/get?file={asset}_2K-JPG.zip"
AMBIENTCG_ASSETS_API = "https://ambientcg.com/api/v3/assets"

# BlenderProc only uses these texture map suffixes (from CCMaterialLoader source).
KEEP_SUFFIXES = {
    "_Color",
    "_AmbientOcclusion",
    "_Metalness",
    "_Roughness",
    "_NormalGL",
    "_Normal",
    "_Displacement",
}


def parse_asset_from_url(url: str) -> str:
    """Extract the asset name (e.g. 'Plastic007') from an ambientCG download URL."""
    parsed = urlparse(url)
    qs = parse_qs(parsed.query)
    filename = qs.get("file", [""])[0]
    if not filename:
        raise ValueError(f"Could not parse asset name from URL: {url}")
    # Plastic007_2K-JPG.zip → Plastic007
    return filename.split("_")[0]


def download_url(url: str) -> str:
    """Extract the asset base name (e.g. 'Plastic007_2K-JPG') from the URL."""
    parsed = urlparse(url)
    qs = parse_qs(parsed.query)
    filename = qs.get("file", [""])[0]
    return filename.removesuffix(".zip") if filename else ""


def resolve_input(value: str) -> tuple[str, str]:
    """Resolve a CLI argument into (download_url, asset_name).

    Accepts either a full ambientCG URL or a bare asset name like 'Plastic007'.
    """
    if value.startswith("http://") or value.startswith("https://"):
        return value, parse_asset_from_url(value)
    asset = value
    url = AMBIENTCG_URL_TEMPLATE.format(asset=asset)
    return url, asset


def is_kept_texture(filename: str) -> bool:
    """Check if a file is one of the texture maps BlenderProc uses."""
    stem = Path(filename).stem
    return any(stem.endswith(suffix) for suffix in KEEP_SUFFIXES)


def download_and_extract(url: str, asset_name: str, output_dir: Path) -> Path:
    """Download a zip from ambientCG, extract useful textures, clean up the rest."""
    asset_dir = output_dir / asset_name
    asset_dir.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory() as tmp:
        zip_path = Path(tmp) / "download.zip"

        print(f"  Downloading {url} ...")
        req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
        with urllib.request.urlopen(req) as resp, open(zip_path, "wb") as out:
            shutil.copyfileobj(resp, out)

        print("  Extracting ...")
        with zipfile.ZipFile(zip_path) as zf:
            zf.extractall(asset_dir)

    # Remove files BlenderProc doesn't need
    removed = 0
    kept = 0
    for f in sorted(asset_dir.iterdir()):
        if f.is_dir():
            shutil.rmtree(f)
            removed += 1
            continue
        if is_kept_texture(f.name):
            kept += 1
        else:
            f.unlink()
            removed += 1

    print(f"  Kept {kept} textures, removed {removed} unnecessary files")
    return asset_dir


def assets_from_config(config_path: Path) -> list[str]:
    """Read cc_texture asset names from config.toml.

    Collects from both [materials.*].cc_texture entries (robot PBR textures)
    and [environment].ground_textures (ground plane randomization).
    """
    with open(config_path, "rb") as f:
        config = tomllib.load(f)

    assets: list[str] = []

    for mat_cfg in config.get("materials", {}).values():
        name = mat_cfg.get("cc_texture")
        if name:
            assets.append(name)

    assets.extend(config.get("environment", {}).get("ground_textures", []))

    return assets


def fetch_asset_page(limit: int, offset: int) -> dict:
    """Fetch one page of material assets from ambientCG API v3."""
    url = f"{AMBIENTCG_ASSETS_API}?type=material&sort=alphabet&limit={limit}&offset={offset}"
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    with urllib.request.urlopen(req) as resp:
        return json.load(resp)


def list_random_assets(count: int, seed: int) -> list[str]:
    """List random material asset IDs available on ambientCG."""
    page_size = 500
    first_page = fetch_asset_page(limit=page_size, offset=0)
    total = int(first_page.get("totalResults", 0))
    assets = first_page.get("assets", [])
    ids = [a.get("id") for a in assets if a.get("id")]

    # Fetch remaining pages (if any).
    for offset in range(page_size, total, page_size):
        page = fetch_asset_page(limit=page_size, offset=offset)
        ids.extend(a.get("id") for a in page.get("assets", []) if a.get("id"))

    # De-duplicate while preserving order.
    unique_ids = list(dict.fromkeys(ids))
    if not unique_ids:
        return []

    rng = random.Random(seed)
    if count >= len(unique_ids):
        rng.shuffle(unique_ids)
        return unique_ids
    return rng.sample(unique_ids, count)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "output_dir",
        type=Path,
        help="Directory to store textures (e.g. data/cc_textures)",
    )
    parser.add_argument(
        "assets",
        nargs="*",
        help="Asset names (e.g. Plastic007) or full ambientCG download URLs",
    )
    parser.add_argument(
        "--from-config",
        type=Path,
        default=None,
        help="Read cc_texture names from a config.toml file",
    )
    parser.add_argument(
        "--resolution",
        default="2K-JPG",
        help="ambientCG resolution/format variant (default: 2K-JPG)",
    )
    parser.add_argument(
        "--list-random",
        type=int,
        default=0,
        help="List N random texture asset IDs from ambientCG and exit",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed used by --list-random (default: 42)",
    )
    args = parser.parse_args()

    if args.list_random < 0:
        parser.error("--list-random must be >= 0")

    # List mode: print a random set of assets without downloading.
    if args.list_random > 0:
        print(f"Fetching ambientCG materials and sampling {args.list_random} at random...")
        try:
            sampled = list_random_assets(args.list_random, seed=args.seed)
        except Exception as e:
            parser.error(f"Failed to query ambientCG API: {e}")

        if not sampled:
            print("No texture assets were returned by ambientCG API.")
            return

        print("\nRandom texture asset IDs:")
        for asset in sampled:
            print(f"  {asset}")

        joined_assets = " ".join(sampled)
        print("\nDownload these with:")
        print(
            f"python download_ambientcg.py {args.output_dir} {joined_assets} "
            f"--resolution {args.resolution}"
        )
        return

    inputs: list[str] = list(args.assets)
    if args.from_config:
        inputs.extend(assets_from_config(args.from_config))

    if not inputs:
        parser.error("Provide asset names/URLs as arguments or use --from-config")

    # Override URL template if non-default resolution
    global AMBIENTCG_URL_TEMPLATE
    AMBIENTCG_URL_TEMPLATE = (
        f"https://ambientcg.com/get?file={{asset}}_{args.resolution}.zip"
    )

    output_dir: Path = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    for value in inputs:
        url, asset_name = resolve_input(value)
        existing = output_dir / asset_name
        if existing.exists() and any(existing.iterdir()):
            print(f"Skipping {asset_name} (already exists at {existing})")
            continue
        print(f"Downloading {asset_name} ...")
        try:
            download_and_extract(url, asset_name, output_dir)
        except Exception as e:
            print(f"  ERROR: {e}")
            continue

    print(f"\nDone. Textures saved to {output_dir}")


if __name__ == "__main__":
    main()
