#!/usr/bin/env python3
"""
Sync model files from a public Google Drive folder using gdown.

This script downloads all files from the configured public Drive folder into
the local models directory, but never overwrites existing local files.

Usage:
    python scripts/sync_models.py                # Download missing models
    python scripts/sync_models.py --dry-run      # Preview without copying files
    python scripts/sync_models.py -o ./custom    # Download to custom directory
    python scripts/sync_models.py --url URL      # Override Drive folder URL
"""

import platform
import argparse
import gdown
import sys
from pathlib import Path

DEFAULT_DRIVE_FOLDER_URL = (
    "https://drive.google.com/drive/u/0/folders/1rkVKwUzK0L4K7sQYzrZ-hZ0uGaTnZWpo"
)


def get_project_root() -> Path:
    """Get the project root directory."""
    return Path(__file__).parent.parent.resolve()


def get_models_dir() -> Path:
    """Get the default models directory path."""
    return get_project_root() / "data" / "models"


def format_size(size_bytes: int) -> str:
    """Format file size in human-readable format."""
    size = float(size_bytes)
    for unit in ["B", "KB", "MB", "GB"]:
        if size < 1024:
            return f"{size:.1f} {unit}"
        size /= 1024
    return f"{size:.1f} TB"


def list_remote_files(url: str, output_dir: Path) -> list:
    """List files in the Drive folder without downloading them."""
    try:
        return gdown.download_folder(
            url=url,
            output=str(output_dir),
            quiet=False,
            skip_download=True,
        )
    except Exception as error:
        print(f"Error listing folder with gdown: {error}")
        sys.exit(1)


def sync_models_cmd(output_dir: Path, url: str, dry_run: bool = False) -> None:
    """Sync missing model files from Drive into the destination directory."""
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Source: {url}")
    print(f"Destination: {output_dir}")
    if dry_run:
        print("Mode: dry run")
    print()

    remote_files = list_remote_files(url, output_dir)

    if not remote_files:
        print("No files found in the Drive folder.")
        print_platform_tag_matches(get_models_dir())
        return

    downloaded = 0
    skipped_existing = 0

    for remote_file in remote_files:
        rel_path = Path(remote_file.path)
        dest_path = output_dir / rel_path

        if dest_path.exists():
            print(f"[SKIP] {rel_path} - local file already exists")
            skipped_existing += 1
            continue

        if dry_run:
            print(f"[WOULD DOWNLOAD] {rel_path}")
            continue

        dest_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            gdown.download(
                id=remote_file.id,
                output=str(dest_path),
                quiet=False,
                resume=True,
            )
            size_str = format_size(dest_path.stat().st_size)
            print(f"[DOWNLOADED] {rel_path} ({size_str})")
            downloaded += 1
        except Exception as error:
            print(f"[ERROR] {rel_path} - {error}")

    print()
    if dry_run:
        pending = len(remote_files) - skipped_existing
        print(
            f"Dry run complete: {pending} file(s) would be downloaded, "
            f"{skipped_existing} skipped (already exists)."
        )
    elif downloaded == 0:
        print("No new files downloaded; all listed files already exist locally.")
    else:
        print(
            f"Sync complete: {downloaded} downloaded, "
            f"{skipped_existing} skipped (already exists)."
        )
    print_platform_tag_matches(get_models_dir())


def _get_compute_capability() -> tuple[int, int]:
    """Query GPU compute capability of device 0."""
    try:
        import torch
    except ImportError:
        return (0, 0)
    if torch.cuda.is_available():
        return torch.cuda.get_device_capability(0)
    raise RuntimeError("CUDA not available — cannot determine GPU compute capability")


def get_platform_compute_tag() -> str:
    """Return the platform + compute capability tag used in engine filenames."""
    arch = platform.machine()
    major, minor = _get_compute_capability()
    return f"{arch}_sm{major}{minor}"


def print_platform_tag_matches(models_dir: Path) -> None:
    """Print current tag and model files in models_dir that match it."""
    print()
    try:
        tag = get_platform_compute_tag()
    except Exception as error:
        print(f"Platform / compute tag unavailable: {error}")
        return

    print(f"Platform/compute tag: {tag}")
    matches = sorted(
        file_path
        for file_path in models_dir.rglob("*")
        if file_path.is_file() and f"_{tag}" in file_path.name
    )
    if not matches:
        print(f"No models in {models_dir} match tag '{tag}'.")
        return

    print(f"Models in {models_dir} matching tag '{tag}':")
    for file_path in matches:
        print(f"  - {file_path.relative_to(models_dir)}")


def engine_path_with_platform_tag(path: Path) -> Path:
    """Append platform + GPU compute capability tag so incompatible engines are distinct.

    Produces filenames like ``model_x86_64_sm89.engine`` — the ``sm`` tag
    prevents silently loading an engine built for a different GPU architecture.
    """
    tag = get_platform_compute_tag()
    suffix = path.suffix if path.suffix else ".engine"
    return path.parent / f"{path.stem}_{tag}{suffix}"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Sync model files from a public Google Drive folder using gdown",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default=None,
        help="Output directory for models (default: data/models in project root)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show what would be copied without writing files",
    )
    parser.add_argument(
        "--url",
        type=str,
        default=DEFAULT_DRIVE_FOLDER_URL,
        help="Public Google Drive folder URL",
    )
    args = parser.parse_args()

    output_dir = Path(args.output).resolve() if args.output else get_models_dir()
    sync_models_cmd(output_dir=output_dir, url=args.url, dry_run=args.dry_run)


if __name__ == "__main__":
    main()
