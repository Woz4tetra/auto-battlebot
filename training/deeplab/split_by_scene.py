#!/usr/bin/env python3
"""Split a segmask dataset into scene-disjoint train/val groups.

The deeplab counterpart to `training/yolo/split_by_scene.py`, and the replacement for
`split_segmask_dataset.py` on any corpus built from video. `split_segmask_dataset.py` shuffles at
frame level; every frame here is one sample of a recording taken at a fixed stride, so its
neighbours are near-identical. A frame-level shuffle therefore puts a frame's own neighbours on
the other side of the split, val turns into a memorisation check, and the score reads high for a
model that generalises to nothing. This tool only ever moves whole scenes.

A *scene* is one source recording. Scenes are read from a `field_index.json` when the corpus
carries one, and otherwise inferred from the frame stem; a stem matching no known scheme is an
error rather than a guess, since an unrecognised scene is exactly the one that would leak.

The assignment is stratified on *field* -- which cage the footage was shot in -- alongside frame
count. Field is the distribution shift that matters for field segmentation: a val set missing a
cage says nothing about that cage, and a val set that is mostly one cage grades a single arena.

Frames and their `_mask.png` are hardlinked, so a split costs no meaningful disk.

Usage:
    python training/deeplab/split_by_scene.py \
        --src ~/Desktop/deeplab_field_merged_2026-07-29 \
        --out ~/Desktop/deeplab_field_split_2026-07-29 \
        --val-frac 0.2
"""

from __future__ import annotations

import argparse
import json
import os
import re
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import yaml

IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png")
MASK_SUFFIX = "_mask.png"
SPLITS = ("train", "val")

# Frame stems in the field corpora, most specific first.
SCENE_PATTERNS = (
    # Locally cut clips, flattened by build_field_dataset.py:
    #   segmask_floor__<video>__<offset>_segmask__segmask__frame_000160
    re.compile(r"^segmask_floor__(?P<scene>.+?)__\d+_segmask__.*frame_\d+$"),
    # SegmentFlow exports, which lead with the source video id.
    re.compile(r"^(?P<scene>[A-Za-z0-9_-]{11})[^/]*?__frame_\d+$"),
    # Frames extracted from a recording by make_seg_dataset.py.
    re.compile(r"^(?P<scene>.*)_yolo_seg__frame_\d+$"),
    # Roboflow exports: <scene><sep><index>_jpg.rf.<hash>
    re.compile(r"^(?P<scene>.*)[-_]\d{6}_jpg\.rf\.[A-Za-z0-9]+$"),
)

CAGE_RE = re.compile(r"[Cc]age[-_ ]?(\d+)")


@dataclass
class Scene:
    """One source recording: its frames and the field they were shot in."""

    key: str
    frames: List[Path] = field(default_factory=list)
    field_name: str = "unknown"

    @property
    def n_frames(self) -> int:
        """Number of frames in this scene."""
        return len(self.frames)


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


def load_field_index(src: Path) -> Dict[str, dict]:
    """Merge every `field_index.json` under `src`, keyed by frame stem.

    These are written by `build_field_manifest.py` and carry the hand-checked scene and field for
    the frames they cover. Where one exists it outranks anything inferable from a filename.
    """
    index: Dict[str, dict] = {}
    for path in sorted(src.rglob("field_index.json")):
        try:
            entries = json.loads(path.read_text())
        except (OSError, ValueError):
            print(f"  skipped unreadable {path}")
            continue
        for stem, entry in entries.items():
            if isinstance(entry, dict):
                index[Path(stem).stem] = entry
    return index


def scene_key(stem: str) -> Optional[str]:
    """Recording key for a frame stem, or None if it matches no known scheme."""
    for pattern in SCENE_PATTERNS:
        match = pattern.match(stem)
        if match:
            return match.group("scene")
    return None


def field_of(key: str) -> str:
    """Infer the field (cage) a scene was shot in, or 'unknown'."""
    cage = CAGE_RE.search(key)
    return f"cage{cage.group(1)}" if cage else "unknown"


def collect_scenes(src: Path) -> Dict[str, Scene]:
    """Group every frame under `src` into its scene."""
    index = load_field_index(src)
    scenes: Dict[str, Scene] = {}
    unmatched: List[str] = []
    no_mask: List[str] = []

    for frame in frames_in(src):
        if not mask_for(frame).is_file():
            no_mask.append(frame.name)
            continue
        entry = index.get(frame.stem, {})
        key = entry.get("scene") or scene_key(frame.stem)
        if key is None:
            unmatched.append(frame.stem)
            continue
        scene = scenes.setdefault(
            key, Scene(key=key, field_name=entry.get("field") or field_of(key))
        )
        scene.frames.append(frame)

    if no_mask:
        raise SystemExit(f"{len(no_mask)} frame(s) have no {MASK_SUFFIX}, e.g. {no_mask[:3]}")
    if unmatched:
        raise SystemExit(
            f"{len(unmatched)} frame(s) match no known scene naming scheme, e.g. "
            f"{unmatched[:3]}. A frame with no scene cannot be split without leaking -- "
            "add its pattern to SCENE_PATTERNS."
        )
    return scenes


def metrics_of(scene: Scene, fields: List[str]) -> List[float]:
    """Balance vector for a scene: frame count, then its frames under each field."""
    return [float(scene.n_frames)] + [
        float(scene.n_frames if scene.field_name == f else 0) for f in fields
    ]


def assign_stratified(
    scenes: List[Scene], shares: Dict[str, float], fields: List[str]
) -> Dict[str, List[Scene]]:
    """Greedily assign whole scenes to groups to match each group's target share.

    Scenes are placed scarcest-first (largest share of any single metric), so footage from a
    thinly covered field picks its side before the bulk frames crowd the targets. Each scene then
    goes wherever it minimises the summed squared deviation from every group's target, across
    frame count and every field at once.
    """
    totals = [sum(m) for m in zip(*(metrics_of(s, fields) for s in scenes))]
    totals = [t if t > 0 else 1.0 for t in totals]

    def scarcity(scene: Scene) -> float:
        return float(max(v / t for v, t in zip(metrics_of(scene, fields), totals)))

    order = sorted(scenes, key=lambda s: (-scarcity(s), -s.n_frames, s.key))
    groups: Dict[str, List[Scene]] = {g: [] for g in shares}
    current = {g: [0.0] * len(totals) for g in shares}

    for scene in order:
        values = metrics_of(scene, fields)
        best, best_cost = None, None
        for candidate in shares:
            cost = 0.0
            for group in shares:
                extra = values if group == candidate else [0.0] * len(values)
                for i, total in enumerate(totals):
                    achieved = (current[group][i] + extra[i]) / total
                    cost += (achieved - shares[group]) ** 2
            if best_cost is None or cost < best_cost:
                best, best_cost = candidate, cost
        assert best is not None
        groups[best].append(scene)
        for i, value in enumerate(values):
            current[best][i] += value

    return groups


def link_group(scenes: List[Scene], out_dir: Path) -> int:
    """Hardlink each scene's frame and mask into `out_dir`. Returns frame count."""
    out_dir.mkdir(parents=True, exist_ok=True)
    linked = 0
    for scene in scenes:
        for frame in scene.frames:
            for src, dst in (
                (frame, out_dir / frame.name),
                (mask_for(frame), out_dir / mask_for(frame).name),
            ):
                if dst.exists():
                    dst.unlink()
                try:
                    os.link(src, dst)
                except OSError:
                    import shutil

                    shutil.copy2(src, dst)
            linked += 1
    return linked


def report(groups: Dict[str, List[Scene]], fields: List[str]) -> dict:
    """Print per-group composition and return it as a manifest-ready dict."""
    total = sum(s.n_frames for g in groups.values() for s in g)
    header = f"{'group':6s} {'scenes':>7s} {'frames':>8s} {'share':>7s}  " + "  ".join(
        f"{f[:10]:>10s}" for f in fields
    )
    print(header)
    print("-" * len(header))
    manifest: Dict[str, object] = {}
    for group, scenes in groups.items():
        frames = sum(s.n_frames for s in scenes)
        per_field: Counter = Counter()
        for scene in scenes:
            per_field[scene.field_name] += scene.n_frames
        print(
            f"{group:6s} {len(scenes):7d} {frames:8d} {frames / total if total else 0:6.1%}  "
            + "  ".join(f"{per_field[f]:10d}" for f in fields)
        )
        manifest[group] = {
            "n_scenes": len(scenes),
            "frames": frames,
            "fields": {f: per_field[f] for f in fields if per_field[f]},
            "scenes": sorted(s.key for s in scenes),
        }

    print("\nval share per field (a field absent from val is a field val cannot grade):")
    for f in fields:
        totals = {g: sum(s.n_frames for s in sc if s.field_name == f) for g, sc in groups.items()}
        overall = sum(totals.values())
        if not overall:
            continue
        print(f"  {f:12s} " + "  ".join(f"{g}={n / overall:.1%}" for g, n in totals.items()))
    return manifest


def build_arg_parser() -> argparse.ArgumentParser:
    """Construct the command line argument parser."""
    parser = argparse.ArgumentParser(description="Scene-disjoint split of a segmask dataset")
    parser.add_argument("--src", type=Path, required=True, help="Source segmask dataset")
    parser.add_argument("--out", type=Path, required=True, help="Output root for train/ and val/")
    parser.add_argument("--val-frac", type=float, default=0.2, help="Val share of the frames")
    parser.add_argument("--dry-run", action="store_true", help="Report the split, link nothing")
    return parser


def main() -> None:
    """Split a segmask dataset into scene-disjoint train and val groups."""
    args = build_arg_parser().parse_args()
    classes_path = args.src / "data.yaml"
    if not classes_path.is_file():
        raise SystemExit(f"no data.yaml under {args.src}")
    classes = yaml.safe_load(classes_path.read_text())

    scenes = collect_scenes(args.src)
    total = sum(s.n_frames for s in scenes.values())
    fields = sorted({s.field_name for s in scenes.values()})
    print(f"{len(scenes)} scenes, {total} frames, {len(fields)} field(s): {fields}")

    groups = assign_stratified(
        sorted(scenes.values(), key=lambda s: s.key),
        {"train": 1.0 - args.val_frac, "val": args.val_frac},
        fields,
    )
    groups = {split: groups[split] for split in SPLITS}

    keys = [{s.key for s in groups[split]} for split in SPLITS]
    if keys[0] & keys[1]:
        raise SystemExit(f"scene overlap between train and val: {sorted(keys[0] & keys[1])[:3]}")

    print()
    manifest = report(groups, fields)

    if args.dry_run:
        print("\ndry run: nothing linked")
        return

    args.out.mkdir(parents=True, exist_ok=True)
    print()
    for split in SPLITS:
        linked = link_group(groups[split], args.out / split)
        print(f"linked {linked:6d} frames -> {args.out / split}")

    (args.out / "data.yaml").write_text(yaml.safe_dump(classes, sort_keys=False))
    (args.out / "split_manifest.json").write_text(
        json.dumps(
            {
                "source": str(args.src.resolve()),
                "val_frac": args.val_frac,
                "stratified_on": ["frames"] + fields,
                "groups": manifest,
            },
            indent=2,
        )
    )
    print(f"\nwrote {args.out}/data.yaml and split_manifest.json")


if __name__ == "__main__":
    main()
