"""Register a manually-downloaded NHRL robot thumbnail into the generation state.

Use this when you drop a thumbnail PNG into ``<output_dir>/thumbnails/`` yourself
(bypassing ``download_nhrl_bots.py``, e.g. for a specific opponent) and want a proper
state entry so ``generate_nhrl_meshes.py`` can pick it up, without hand-editing
``.nhrl_generation_state.json``.

The clean_name is derived from the display name the same way BrettZone does it:
lowercase, then strip every non-alphanumeric character. ``Iron Warrior`` -> ``ironwarrior``.
The thumbnail is expected at ``<output_dir>/thumbnails/<clean_name>.png`` unless
``--thumbnail`` overrides it.

Usage:
  venv/bin/python training/synthetic/register_nhrl_thumbnail.py \
      training/data/distractor_models/robots "Iron Warrior"

  # several at once
  venv/bin/python training/synthetic/register_nhrl_thumbnail.py \
      training/data/distractor_models/robots "Sphinx" "Wreck Creation" "Iron Warrior"

  # override the derived clean_name / thumbnail file (single robot only)
  venv/bin/python training/synthetic/register_nhrl_thumbnail.py \
      training/data/distractor_models/robots "M.C.B." --clean-name mcb --thumbnail mcb.png

Then generate just those with:
  venv/bin/python training/synthetic/generate_nhrl_meshes.py \
      training/data/distractor_models/robots --only ironwarrior sphinx wreckcreation --limit 3
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path
from typing import Any

import nhrl_common as nc

_CLEAN_RE = re.compile(r"[^a-z0-9]+")


def to_clean_name(name: str) -> str:
    """Derive a BrettZone-style clean_name: lowercase, strip non-alphanumerics."""
    return _CLEAN_RE.sub("", name.lower())


def build_arg_parser() -> argparse.ArgumentParser:
    """Construct the command line argument parser."""
    parser = argparse.ArgumentParser(
        description="Register manually-downloaded NHRL thumbnails into the generation state"
    )
    parser.add_argument("output_dir", type=Path, help="Robot distractor directory")
    parser.add_argument("names", nargs="+", help="Robot display name(s), e.g. 'Iron Warrior'")
    parser.add_argument(
        "--clean-name",
        default=None,
        help="Override the derived clean_name (only valid with a single name)",
    )
    parser.add_argument(
        "--thumbnail",
        default=None,
        help="Override the thumbnail filename (only valid with a single name)",
    )
    parser.add_argument("--weight-class", default="3lb", help="Weight-class label (default: 3lb)")
    parser.add_argument(
        "--total-fights", type=int, default=0, help="Total fights (metadata only, default: 0)"
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Re-register even if the entry is already done (resets it to 'selected')",
    )
    return parser


def main() -> None:
    """Register one or more thumbnails into the generation state."""
    args = build_arg_parser().parse_args()
    if (args.clean_name or args.thumbnail) and len(args.names) != 1:
        raise SystemExit("--clean-name/--thumbnail may only be used with a single name")

    models_dir = nc.resolve_cli_path(args.output_dir)
    thumb_dir = models_dir / "thumbnails"
    state_path = models_dir / nc.STATE_FILENAME
    state = nc.load_state(state_path)

    registered: list[str] = []
    for name in args.names:
        clean = args.clean_name or to_clean_name(name)
        if not clean:
            print(f"SKIP {name!r}: produces an empty clean_name")
            continue
        thumb_file = args.thumbnail or f"{clean}.png"
        thumb_path = thumb_dir / thumb_file
        if not thumb_path.exists():
            print(f"SKIP {name!r}: thumbnail not found at {thumb_path} (drop the PNG there first)")
            continue

        entry: dict[str, Any] = dict(state.get(clean) or {})
        if entry.get("status") == nc.STATUS_DONE and not args.force:
            print(f"SKIP {clean}: already done (use --force to re-register)")
            continue

        existing_fights = entry.get("total_fights")
        entry.update(
            {
                "name": name,
                "clean_name": clean,
                "weight_classes": entry.get("weight_classes") or [args.weight_class],
                "total_fights": existing_fights
                if existing_fights is not None
                else args.total_fights,
                "thumbnail_url": nc.bot_thumbnail_url(clean),
                "thumbnail_file": thumb_file,
                "status": "selected",
                "thumbnail_reviewed": True,
                "rejected": False,
            }
        )
        state[clean] = entry
        registered.append(clean)
        print(f"registered {clean}: {name!r} -> thumbnails/{thumb_file} (status selected)")

    nc.save_state(state_path, state)
    if registered:
        joined = " ".join(registered)
        print(f"\n{len(registered)} registered. Next, generate just these:")
        print(
            f"  venv/bin/python training/synthetic/generate_nhrl_meshes.py {args.output_dir} "
            f"--only {joined} --limit {len(registered)}"
        )
    else:
        print("\nNothing registered.")


if __name__ == "__main__":
    main()
