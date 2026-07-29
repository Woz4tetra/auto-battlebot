#!/usr/bin/env python3
"""Assemble a segmask tree into one train/val field-segmentation dataset.

`merge_segmask_datasets.py` + `split_segmask_dataset.py` would also produce a merged dataset, but
`split_segmask_dataset.py` shuffles at frame level. Every corpus here is video-derived at a fixed
stride, so a frame-level shuffle puts near-identical neighbours on both sides of the split and the
val score stops measuring generalisation. This tool splits at the level of the source video
instead, and never splits one video across train and val.

It also refuses to reshuffle a corpus that is already split. A source subtree holding its own
`train/` and `val/` is treated as settled: its frames keep the side they are already on. That
keeps a previously published split, and any anchor scores computed against it, still comparable
after new material is folded in. Only the unsplit material gets an assignment, sized to match the
held corpus's val fraction.

Masks are assumed to already share one label space; the tool checks this rather than remapping,
since a remap needs per-source class names that flat segmask exports do not carry.

Usage:
    python training/deeplab/build_field_dataset.py \
        --src ~/Desktop/deeplab_field_2026-07-29 \
        --out ~/Desktop/deeplab_field_merged_2026-07-29
"""

from __future__ import annotations

import argparse
import json
import random
import re
import shutil
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import yaml
from PIL import Image

IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png")
MASK_SUFFIX = "_mask.png"
SPLITS = ("train", "val")
# SegmentFlow names an export after the video it was cut from, and a YouTube id is 11 characters.
# Slices of one video differ only after that prefix, so the prefix is the video identity.
YOUTUBE_ID_LEN = 11


def frames_in(root: Path) -> List[Path]:
    """Every annotatable frame under `root`, excluding the `_mask.png` files themselves."""
    return sorted(
        p
        for p in root.rglob("*")
        if p.suffix.lower() in IMAGE_SUFFIXES and not p.stem.endswith("_mask") and p.is_file()
    )


def mask_for(frame: Path) -> Path:
    """The mask that annotates `frame`."""
    return frame.with_name(f"{frame.stem}{MASK_SUFFIX}")


def held_corpora(src: Path) -> List[Path]:
    """Source subtrees that are already split, and whose assignment must be preserved."""
    return sorted(
        d
        for d in src.iterdir()
        if d.is_dir() and all((d / split).is_dir() for split in SPLITS) and not d.is_symlink()
    )


def video_key(rel: Path) -> str:
    """Identify the source video a frame came from.

    Two naming schemes are in play. Clips cut locally are `<video>__<offset>_segmask`, so the
    offset is dropped. SegmentFlow exports lead with the video id, so the id is taken directly.
    Anything else falls back to its top-level directory, which at worst splits one video into
    several groups -- conservative, since it can only ever over-separate, never leak.
    """
    parts = rel.parts
    top = parts[0]
    if len(parts) > 1 and re.search(r"__\d+_segmask$", parts[1]):
        return f"{top}/{re.sub(r'__[0-9]+_segmask$', '', parts[1])}"
    if len(top) > YOUTUBE_ID_LEN and re.fullmatch(r"[A-Za-z0-9_-]+", top[:YOUTUBE_ID_LEN]):
        return top[:YOUTUBE_ID_LEN]
    return top


def scene_stem(rel: Path) -> str:
    """A collision-proof output name for a frame at `rel` within its source root.

    Sub-exports reuse frame numbers heavily -- one corpus here carries 1,407 frames over 90
    distinct basenames -- so the directory path has to survive into the filename.
    """
    slug = re.sub(r"[^A-Za-z0-9._-]+", "_", "__".join(rel.parts[:-1]))
    return f"{slug}__{rel.stem}" if slug else rel.stem


def mask_labels(mask: Path) -> set:
    """The label ids present in one mask."""
    array = np.array(Image.open(mask))
    if array.ndim == 3:
        array = array[..., 0]
    return set(np.unique(array).tolist())


def check_label_space(frames: List[Path], expected: int, sample: int = 40) -> set:
    """Sample masks and return every label id seen, to catch a source in a different id space."""
    seen: set = set()
    step = max(1, len(frames) // sample)
    for frame in frames[::step][:sample]:
        seen |= mask_labels(mask_for(frame))
    unexpected = {label for label in seen if label >= expected}
    if unexpected:
        raise SystemExit(
            f"mask label(s) {sorted(unexpected)} exceed the {expected} classes in data.yaml -- "
            "sources are not in one label space, remap before merging"
        )
    return seen


def assign_videos(
    groups: Dict[str, List[Tuple[Path, Path]]], val_frac: float, seed: int
) -> Dict[str, str]:
    """Assign whole videos to train or val, filling val up to `val_frac` by frame count.

    Videos are taken in a seeded random order and only while they still fit under the target, so
    val never overshoots. Overshooting is the worse error in both directions: it starves training
    of material, and one oversized video would dominate the val score with a single venue.
    """
    total = sum(len(v) for v in groups.values())
    target = total * val_frac
    order = sorted(groups)
    random.Random(seed).shuffle(order)

    assignment: Dict[str, str] = {}
    in_val = 0
    for key in order:
        n = len(groups[key])
        if in_val + n <= target:
            assignment[key] = "val"
            in_val += n
        else:
            assignment[key] = "train"
    return assignment


def materialize(src: Path, dst: Path) -> None:
    """Copy `src` to `dst`, preserving mtime."""
    shutil.copy2(src, dst)


def place(pairs: List[Tuple[str, Path]], out_split: Path, dry_run: bool) -> int:
    """Write one split's frames and masks. `pairs` is (output stem, source frame)."""
    if not dry_run:
        out_split.mkdir(parents=True, exist_ok=True)
    for stem, frame in pairs:
        if dry_run:
            continue
        materialize(frame, out_split / f"{stem}{frame.suffix}")
        materialize(mask_for(frame), out_split / f"{stem}{MASK_SUFFIX}")
    return len(pairs)


def build_arg_parser() -> argparse.ArgumentParser:
    """Construct the command line argument parser."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("--src", type=Path, required=True, help="Root holding the source corpora")
    parser.add_argument("--out", type=Path, required=True, help="Output dataset root")
    parser.add_argument("--seed", type=int, default=0, help="Seed for the video-level assignment")
    parser.add_argument(
        "--val-frac",
        type=float,
        default=None,
        help="Val fraction for the new material (default: match the held corpus)",
    )
    parser.add_argument("--dry-run", action="store_true", help="Report only, write nothing")
    return parser


def collect(
    src: Path,
) -> Tuple[Dict[str, List[Tuple[str, Path]]], Dict[str, List[Path]], List[Path]]:
    """Split the tree into held (already assigned) frames, unsplit frames by video, and skips."""
    held: Dict[str, List[Tuple[str, Path]]] = {split: [] for split in SPLITS}
    unsplit: Dict[str, List[Path]] = defaultdict(list)
    no_mask: List[Path] = []

    corpora = held_corpora(src)
    for corpus in corpora:
        for split in SPLITS:
            for frame in frames_in(corpus / split):
                if mask_for(frame).is_file():
                    held[split].append((frame.stem, frame))
                else:
                    no_mask.append(frame)

    skip = set(corpora) | {src / "validation_backup"}
    for entry in sorted(src.iterdir()):
        if not entry.is_dir() or entry in skip or entry.is_symlink():
            continue
        for frame in frames_in(entry):
            if mask_for(frame).is_file():
                unsplit[video_key(frame.relative_to(src))].append(frame)
            else:
                no_mask.append(frame)
    return held, dict(unsplit), no_mask


def main() -> None:
    """Assemble the source tree into one scene-disjoint train/val dataset."""
    args = build_arg_parser().parse_args()
    src, out = args.src, args.out

    classes = yaml.safe_load((src / "data.yaml").read_text())
    names = list(classes["names"])

    held, unsplit, no_mask = collect(src)
    corpora = held_corpora(src)
    print(f"held corpora ({len(corpora)}): " + ", ".join(c.name for c in corpora))
    for split in SPLITS:
        print(f"  {split:5s} {len(held[split]):6d} frames (assignment preserved)")
    total_new = sum(len(v) for v in unsplit.values())
    print(f"unsplit material: {total_new} frames in {len(unsplit)} source videos")
    if no_mask:
        print(f"  {len(no_mask)} frame(s) with no mask, skipped")

    held_total = len(held["train"]) + len(held["val"])
    val_frac = args.val_frac
    if val_frac is None:
        val_frac = len(held["val"]) / held_total if held_total else 0.2
        print(f"  val fraction taken from the held corpus: {val_frac:.3f}")

    sample = check_label_space(
        [f for pairs in held.values() for _, f in pairs] + [f for v in unsplit.values() for f in v],
        len(names),
    )
    print(f"label space: {names} -- ids seen in sampled masks: {sorted(sample)}")

    assignment = assign_videos(unsplit, val_frac, args.seed)
    out_pairs: Dict[str, List[Tuple[str, Path]]] = {split: list(held[split]) for split in SPLITS}
    for key, frames in unsplit.items():
        split = assignment[key]
        out_pairs[split].extend((scene_stem(f.relative_to(src)), f) for f in frames)

    print("\nvideo assignment:")
    for key in sorted(assignment, key=lambda k: (-len(unsplit[k]), k)):
        print(f"  {assignment[key]:5s} {len(unsplit[key]):5d}  {key}")

    stems = Counter(stem for pairs in out_pairs.values() for stem, _ in pairs)
    clashes = [stem for stem, n in stems.items() if n > 1]
    if clashes:
        raise SystemExit(f"{len(clashes)} output name collision(s), e.g. {clashes[:3]}")

    print("\noutput:")
    for split in SPLITS:
        new = len(out_pairs[split]) - len(held[split])
        print(
            f"  {split:5s} {len(out_pairs[split]):6d} frames  (held {len(held[split])} + new {new})"
        )
    n_val = len(out_pairs["val"])
    print(f"  val fraction {n_val / (n_val + len(out_pairs['train'])):.3f}")

    if args.dry_run:
        print("\ndry run: nothing written")
        return

    out.mkdir(parents=True, exist_ok=True)
    for split in SPLITS:
        place(out_pairs[split], out / split, args.dry_run)

    (out / "data.yaml").write_text(yaml.safe_dump(classes, sort_keys=False))
    (out / "splits.json").write_text(
        json.dumps(
            {
                "source_root": str(src),
                "seed": args.seed,
                "val_frac": val_frac,
                "held_corpora": [c.name for c in corpora],
                "held_counts": {s: len(held[s]) for s in SPLITS},
                "video_assignment": assignment,
                "video_counts": {k: len(v) for k, v in unsplit.items()},
                "counts": {s: len(out_pairs[s]) for s in SPLITS},
                "materialize": "copy",
            },
            indent=2,
            sort_keys=True,
        )
    )
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
