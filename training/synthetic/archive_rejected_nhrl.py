"""Move rejected NHRL robot models out of the distractor pool into an archive.

Step 3.6 of the NHRL robot distractor pipeline, run on the HOST after
``review_nhrl_keypoints.py`` marks bad meshes with ``rejected: true`` in their
sidecar.  ``render_scenes.py`` discovers distractors by globbing model files in
the source directory, so the only reliable way to keep a rejected robot out of
the render pool is to move its files elsewhere.  This tool does exactly that:
for every sidecar flagged ``rejected``, it relocates the model file, the sidecar
JSON, and the top-down preview PNG into an archive directory, preserving the
``topdown/`` layout so the move is reversible.

Run in a root project venv (only needs the stdlib + ``nhrl_common``):

    venv/bin/python training/synthetic/archive_rejected_nhrl.py \
        training/data/distractor_models/robots

By default the archive lands in ``<models_dir>/../rejected_robots``.  Pass
``--dry-run`` first to see exactly what would move without touching anything.
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import nhrl_common as nc

# Model container extensions the render pool globs (mirrors render_scenes.py).
_MODEL_EXTENSIONS = (".glb", ".gltf", ".obj", ".ply")


def _model_file_for(sidecar: Path) -> Path | None:
    """Sibling model file for a ``<stem>.json`` sidecar, or None if missing."""
    for ext in _MODEL_EXTENSIONS:
        candidate = sidecar.with_suffix(ext)
        if candidate.exists():
            return candidate
    return None


def _topdown_image_for(sidecar: Path, data: dict) -> Path | None:
    """Top-down PNG for a sidecar, from its ``topdown.image`` path if present."""
    td = data.get("topdown") or {}
    rel = td.get("image")
    if rel:
        img = sidecar.parent / str(rel)
        if img.exists():
            return img
    # Fall back to the conventional layout even if the sidecar lacks the field.
    fallback = sidecar.parent / "topdown" / f"{sidecar.stem}.png"
    return fallback if fallback.exists() else None


def _move(src: Path, models_dir: Path, archive_dir: Path, dry_run: bool) -> None:
    """Move *src* into *archive_dir*, preserving its path relative to models_dir."""
    dest = archive_dir / src.relative_to(models_dir)
    print(f"    {src.relative_to(models_dir)} -> {dest}")
    if dry_run:
        return
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(str(src), str(dest))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("models_dir", type=Path, help="Directory of robot .glb models")
    parser.add_argument(
        "--archive-dir",
        type=Path,
        default=None,
        help="Where to move rejected files (default: <models_dir>/../rejected_robots)",
    )
    parser.add_argument("--models", default="*.glb", help="Glob for models (default: *.glb)")
    parser.add_argument(
        "--dry-run", action="store_true", help="Print what would move without moving anything"
    )
    args = parser.parse_args()

    models_dir = nc.resolve_cli_path(args.models_dir)
    if not models_dir.is_dir():
        raise SystemExit(f"Not a directory: {models_dir}")

    archive_dir = (
        nc.resolve_cli_path(args.archive_dir)
        if args.archive_dir is not None
        else models_dir.parent / "rejected_robots"
    )
    if archive_dir == models_dir:
        raise SystemExit("Archive directory must differ from the models directory")

    moved = 0
    skipped_missing = 0
    for glb_path in sorted(models_dir.glob(args.models)):
        sidecar = nc.sidecar_path_for(glb_path)
        data = nc.load_json(sidecar)
        if not data or not data.get("rejected"):
            continue

        model_file = _model_file_for(sidecar)
        if model_file is None:
            print(f"  {glb_path.name}: rejected but no model file found, skipping")
            skipped_missing += 1
            continue

        print(f"  Archiving {data.get('name') or glb_path.stem}:")
        _move(model_file, models_dir, archive_dir, args.dry_run)
        _move(sidecar, models_dir, archive_dir, args.dry_run)
        topdown = _topdown_image_for(sidecar, data)
        if topdown is not None:
            _move(topdown, models_dir, archive_dir, args.dry_run)
        moved += 1

    verb = "Would archive" if args.dry_run else "Archived"
    print(f"\n{verb} {moved} rejected model(s) to {archive_dir}.")
    if skipped_missing:
        print(f"{skipped_missing} rejected sidecar(s) had no model file and were left in place.")


if __name__ == "__main__":
    main()
