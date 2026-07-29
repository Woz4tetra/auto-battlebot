#!/usr/bin/env python3
"""Remap each per-scene YOLO export to a shared vocabulary, keeping the per-export layout.

`remap_labels_by_name.py` does the same name-based remap but flattens every scene into one
merged dataset. That is right when the next step is training and wrong when the next step is
review: the merge prefixes filenames and leaves the hand-made `validation_state.json` behind,
so the frame verdicts in `validate_yolo_dataset.py` no longer line up.

This tool writes a mirror of the source tree instead. One output directory per export, same
image and label filenames, `validation_state.json` carried across, and a `data.yml` holding the
new vocabulary. Source exports are never touched, so the per-robot class names survive.

Config format is `remap_labels_by_name.py`'s: `target_names`, `default`, and a `[map]` of source
class name -> target name (or `__drop__`). Names with no rule fall through to `default` and are
all printed, so a new non-robot class appearing upstream is visible rather than silently folded
in.

Usage:
    python training/yolo/remap_exports_by_name.py \
        --src ~/Desktop/new_data \
        --config training/yolo/remap_config_2class.toml \
        --out ~/Desktop/new_data_2class
"""

from __future__ import annotations

import argparse
import shutil
from collections import Counter
from pathlib import Path

import yaml
from remap_labels_by_name import build_index_map, convert_rows, find_image, load_config, scene_dirs


def convert_scene(
    scene: Path,
    out_scene: Path,
    index_map: dict[int, int],
    targets: list[str],
    counts: Counter,
    dry_run: bool,
) -> tuple[int, int]:
    """Rewrite one export into `out_scene`. Returns (frames written, labels with no image)."""
    labels_dir, images_dir = scene / "labels", scene / "images"
    if not labels_dir.is_dir():
        return 0, 0

    if not dry_run:
        (out_scene / "images").mkdir(parents=True, exist_ok=True)
        (out_scene / "labels").mkdir(parents=True, exist_ok=True)

    frames = missing = 0
    for label in sorted(labels_dir.glob("*.txt")):
        image = find_image(images_dir, label.stem)
        if image is None:
            missing += 1
            continue
        rows = convert_rows(label.read_text(), index_map, to_bbox=False)
        for row in rows:
            counts[targets[int(row.split()[0])]] += 1
        frames += 1
        if dry_run:
            continue
        (out_scene / "labels" / label.name).write_text("\n".join(rows) + ("\n" if rows else ""))
        shutil.copy2(image, out_scene / "images" / image.name)

    state = scene / "validation_state.json"
    if state.is_file() and not dry_run:
        shutil.copy2(state, out_scene / state.name)
    return frames, missing


def write_data_yml(path: Path, targets: list[str]) -> None:
    """Write the export's data.yml.

    The split keys are inherited from what SegmentFlow emits and are decorative here -- an export
    is a flat images/ + labels/ pair with no train/ or val/ under it. Downstream splitting tools
    write their own data.yml.
    """
    data = {
        "train": "train/images",
        "val": "val/images",
        "nc": len(targets),
        "names": targets,
    }
    path.write_text(yaml.safe_dump(data, sort_keys=False, default_flow_style=False))


def build_arg_parser() -> argparse.ArgumentParser:
    """Construct the command line argument parser."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("--src", type=Path, required=True, help="Root of per-scene exports")
    parser.add_argument("--config", type=Path, required=True, help="TOML name-mapping config")
    parser.add_argument("--out", type=Path, required=True, help="Output root (mirrors --src)")
    parser.add_argument("--dry-run", action="store_true", help="Report only, write nothing")
    return parser


def main() -> None:
    """Remap every export under --src into a mirrored tree under --out."""
    args = build_arg_parser().parse_args()
    targets, mapping, default = load_config(args.config)
    scenes = scene_dirs(args.src)
    if not scenes:
        raise SystemExit(f"no per-scene exports under {args.src}")
    print(f"{len(scenes)} exports -> {targets} (default: {default!r})")

    counts: Counter = Counter()
    defaulted: set[str] = set()
    frames = missing = 0

    for scene in scenes:
        scene_names = list(yaml.safe_load((scene / "data.yml").read_text())["names"])
        index_map, hit_default = build_index_map(scene_names, mapping, default, targets)
        defaulted |= hit_default
        out_scene = args.out / scene.name
        kept, skipped = convert_scene(scene, out_scene, index_map, targets, counts, args.dry_run)
        if not args.dry_run:
            write_data_yml(out_scene / "data.yml", targets)
        frames += kept
        missing += skipped

    print(
        f"\nframes: {frames}" + (f"  ({missing} labels with no image, skipped)" if missing else "")
    )
    print("annotations per target class:")
    for i, name in enumerate(targets):
        print(f"  {i} {name:12s} {counts[name]:7d}")
    print(f"  total        {sum(counts.values()):7d}")

    if defaulted:
        print(
            f"\n{len(defaulted)} source name(s) had no explicit rule and used default "
            f"{default!r} -- confirm none of these is a non-robot class:"
        )
        for name in sorted(defaulted):
            print(f"    {name}")

    if args.dry_run:
        print("\ndry run: nothing written")
        return
    print(f"\nwrote {args.out} (images copied, no links)")


if __name__ == "__main__":
    main()
