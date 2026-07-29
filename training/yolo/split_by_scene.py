"""Split a YOLO dataset into scene-disjoint groups.

A *scene* is one source recording. Frames within a scene are consecutive video frames
of the same fight and are near-duplicates of each other, so any frame-level split leaks:
the val half is then neighbouring frames of the train half, and val stops measuring
generalization. This tool only ever moves whole scenes.

Produces four groups::

    old        what a "deployed" model trained on
    new        footage that model has never seen
    hold_old   scene-disjoint probe of the old distribution (retention)
    hold_new   scene-disjoint probe of the new distribution (acquisition)

Two ways to divide old from new:

``--mode temporal`` (default)
    Split on recording date at ``--cutoff``. This matches the operational question --
    new footage arrives from a *later* event, so it genuinely differs in cage, lighting
    and robot roster. Class balance across the halves is then whatever the calendar
    gives; it is reported, not corrected.

``--mode stratified``
    Balance the halves on frame count and per-class box counts, ignoring date. Cleaner
    numbers, but old and new end up the same distribution -- so there is nothing unique
    for a new-data-only fine-tune to forget, and the retention probe measures little.

The holdout carve inside each half is always stratified: a holdout is meant to be a
faithful sample of its own half, not another distribution shift.

Images and their stem-matched labels are hardlinked (no copy); ``.npy`` / ``labels.cache``
files are ultralytics artifacts and are skipped -- training regenerates them.

Usage:
  python training/yolo/split_by_scene.py \
      --src training/data/nhrl_robots_bbox_real \
      --out training/data/scenesplit_2026-07-25 \
      --mode temporal --cutoff 2025-11 --holdout-frac 0.2 \
      --stratify-class robot,house_bot
"""

from __future__ import annotations

import argparse
import json
import os
import re
from collections import Counter
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

import yaml

IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp")
SPLITS = ("train", "val", "test")
GROUPS = ("old", "new", "hold_old", "hold_new")

# Three filename schemes live in the bbox corpora. NHRL fight frames carry the
# recording stem before the frame index; frames cut from event-recap video carry an
# `_export` stem instead; merged our-robot keypoint frames carry a roboflow hash suffix.
SCENE_PATTERNS = (
    re.compile(r"^(?P<scene>our_robot_kp__.+?)-\d{6}_jpg\.rf\.[A-Za-z0-9]+$"),
    re.compile(r"^(?P<scene>.*)_yolo_seg__frame_\d+$"),
    re.compile(r"^(?P<scene>.*_export)__frame_\d+$"),
)

DATE_RE = re.compile(r"(20\d{2})-(\d{2})-(\d{2})")
TOURNAMENT_RE = re.compile(r"nhrl_([a-z]+)(\d{2})_")
# Some early recordings are named by unix epoch seconds (e.g. "1729981569.1952372").
EPOCH_RE = re.compile(r"(?<!\d)(1[5-9]\d{8})(?!\d)")
MONTHS = {
    "jan": 1, "feb": 2, "mar": 3, "apr": 4, "may": 5, "june": 6, "jun": 6,
    "july": 7, "jul": 7, "aug": 8, "sep": 9, "sept": 9, "oct": 10, "nov": 11, "dec": 12,
}  # fmt: skip


@dataclass
class Scene:
    """One source recording: its frames, class histogram, and inferred date."""

    key: str
    frames: list[tuple[Path, Path]] = field(default_factory=list)  # (image, label)
    classes: Counter = field(default_factory=Counter)
    period: str | None = None  # "YYYY-MM", or None when undated

    @property
    def n_frames(self) -> int:
        return len(self.frames)


def scene_key(stem: str) -> str | None:
    """Recording key for a frame stem, or None if it matches no known scheme."""
    for pattern in SCENE_PATTERNS:
        match = pattern.match(stem)
        if match:
            return match.group("scene")
    return None


def scene_period(key: str) -> str | None:
    """Infer ``YYYY-MM`` from an explicit date, else from an NHRL tournament tag."""
    date = DATE_RE.search(key)
    if date:
        return f"{date.group(1)}-{date.group(2)}"
    tournament = TOURNAMENT_RE.search(key)
    if tournament and tournament.group(1) in MONTHS:
        return f"20{tournament.group(2)}-{MONTHS[tournament.group(1)]:02d}"
    epoch = EPOCH_RE.search(key)
    if epoch:
        stamp = datetime.fromtimestamp(int(epoch.group(1)), tz=timezone.utc)
        return f"{stamp.year}-{stamp.month:02d}"
    return None


def find_image(labels_dir: Path, images_dir: Path, label_name: str) -> Path | None:
    """Locate the image paired with a label file (stems may contain dots)."""
    stem = label_name[: -len(".txt")]
    for ext in IMAGE_EXTENSIONS:
        candidate = images_dir / (stem + ext)
        if candidate.is_file():
            return candidate
    return None


def scan_split(
    src: Path, split: str, scenes: dict[str, Scene], unmatched: list[str], orphans: list[str]
) -> None:
    """Fold one split's frames into ``scenes``, recording anything that does not fit."""
    labels_dir, images_dir = src / split / "labels", src / split / "images"
    if not labels_dir.is_dir():
        return
    for entry in sorted(os.scandir(labels_dir), key=lambda e: e.name):
        if not entry.name.endswith(".txt"):
            continue
        key = scene_key(entry.name[: -len(".txt")])
        if key is None:
            unmatched.append(entry.name)
            continue
        image = find_image(labels_dir, images_dir, entry.name)
        if image is None:
            orphans.append(entry.name)
            continue

        scene = scenes.setdefault(key, Scene(key=key, period=scene_period(key)))
        scene.frames.append((image, Path(entry.path)))
        for line in Path(entry.path).read_text().splitlines():
            if line.strip():
                scene.classes[int(line.split()[0])] += 1


def collect_scenes(src: Path) -> dict[str, Scene]:
    """Group every frame in the source dataset under its scene key."""
    scenes: dict[str, Scene] = {}
    unmatched: list[str] = []
    orphans: list[str] = []
    for split in SPLITS:
        scan_split(src, split, scenes, unmatched, orphans)

    if unmatched:
        raise SystemExit(
            f"{len(unmatched)} frame(s) match no known scene naming scheme, e.g. "
            f"{unmatched[:3]}. A frame with no scene cannot be split without leaking -- "
            "add its pattern to SCENE_PATTERNS."
        )
    if orphans:
        raise SystemExit(f"{len(orphans)} label file(s) have no paired image, e.g. {orphans[:3]}")
    return scenes


def metrics_of(scene: Scene, stratify: list[int]) -> list[float]:
    """Balance vector for a scene: frame count followed by each stratified class count."""
    return [float(scene.n_frames)] + [float(scene.classes[c]) for c in stratify]


def assign_stratified(
    scenes: list[Scene], shares: dict[str, float], stratify: list[int]
) -> dict[str, list[Scene]]:
    """Greedily assign whole scenes to groups to match each group's target share.

    Scenes are placed scarcest-first (largest share of any single metric), so the rare
    classes get to pick their group before the bulk frames crowd the targets. Each scene
    goes wherever it minimizes the summed squared deviation from every group's target --
    across frame count and every stratified class at once.
    """
    totals = [sum(m) for m in zip(*(metrics_of(s, stratify) for s in scenes), strict=True)]
    totals = [t if t > 0 else 1.0 for t in totals]

    def scarcity(scene: Scene) -> float:
        return float(max(v / t for v, t in zip(metrics_of(scene, stratify), totals, strict=True)))

    order = sorted(scenes, key=lambda s: (-scarcity(s), -s.n_frames, s.key))
    groups: dict[str, list[Scene]] = {g: [] for g in shares}
    current = {g: [0.0] * len(totals) for g in shares}

    for scene in order:
        values = metrics_of(scene, stratify)
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


def split_temporal(
    scenes: list[Scene], cutoff: str, undated: str
) -> tuple[list[Scene], list[Scene]]:
    """Partition scenes into (old, new) by recording period against ``cutoff``."""
    old: list[Scene] = []
    new: list[Scene] = []
    for scene in scenes:
        if scene.period is None:
            if undated == "drop":
                continue
            (old if undated == "old" else new).append(scene)
        elif scene.period < cutoff:
            old.append(scene)
        else:
            new.append(scene)
    return old, new


def link_group(scenes: list[Scene], out_dir: Path) -> int:
    """Hardlink each scene's image+label pairs into ``out_dir``. Returns frame count."""
    (out_dir / "images").mkdir(parents=True, exist_ok=True)
    (out_dir / "labels").mkdir(parents=True, exist_ok=True)
    linked = 0
    for scene in scenes:
        for image, label in scene.frames:
            for src, dst in (
                (image, out_dir / "images" / image.name),
                (label, out_dir / "labels" / label.name),
            ):
                if dst.exists():
                    dst.unlink()
                os.link(src, dst)
            linked += 1
    return linked


def write_group_yaml(out: Path, name: str, names: list[str], colors: list[str] | None) -> None:
    """Write a data.yml that trains on one group and validates on the two holdouts."""
    data: dict[str, object] = {
        "path": str(out.resolve()),
        "train": [f"{g}/images" for g in name.split("+")],
        "val": ["hold_old/images", "hold_new/images"],
        "nc": len(names),
        "names": names,
    }
    if colors:
        data["colors"] = colors
    (out / f"{name}.yml").write_text(yaml.safe_dump(data, sort_keys=False))


def report(groups: dict[str, list[Scene]], names: list[str], stratify: list[int]) -> dict:
    """Print per-group composition and return it as a manifest-ready dict."""
    total_frames = sum(s.n_frames for g in groups.values() for s in g)
    total_classes: Counter = Counter()
    for scenes in groups.values():
        for scene in scenes:
            total_classes.update(scene.classes)

    manifest: dict[str, object] = {}
    header = f"{'group':10s} {'scenes':>7s} {'frames':>8s} {'share':>7s}  " + "  ".join(
        f"{names[c][:12]:>12s}" for c in range(len(names))
    )
    print(header)
    print("-" * len(header))
    for group, scenes in groups.items():
        frames = sum(s.n_frames for s in scenes)
        classes: Counter = Counter()
        for scene in scenes:
            classes.update(scene.classes)
        share = frames / total_frames if total_frames else 0.0
        print(
            f"{group:10s} {len(scenes):7d} {frames:8d} {share:6.1%}  "
            + "  ".join(f"{classes[c]:12d}" for c in range(len(names)))
        )
        manifest[group] = {
            "scenes": sorted(s.key for s in scenes),
            "n_scenes": len(scenes),
            "frames": frames,
            "classes": {names[c]: classes[c] for c in range(len(names))},
            "periods": sorted({s.period or "UNKNOWN" for s in scenes}),
        }

    print("\nclass share per group (a class concentrated in one group cannot be stratified away):")
    for c in range(len(names)):
        if not total_classes[c]:
            continue
        shares = []
        for group, scenes in groups.items():
            got = sum(s.classes[c] for s in scenes)
            shares.append(f"{group}={got / total_classes[c]:.1%}")
        flag = "  <- stratified" if c in stratify else ""
        print(f"  {names[c]:16s} " + "  ".join(f"{s:>16s}" for s in shares) + flag)
    return manifest


def build_arg_parser() -> argparse.ArgumentParser:
    """Construct the command line argument parser."""
    parser = argparse.ArgumentParser(description="Scene-disjoint split of a YOLO dataset")
    parser.add_argument(
        "--src", type=Path, required=True, help="Source dataset (data.yml + splits)"
    )
    parser.add_argument("--out", type=Path, required=True, help="Output root for the groups")
    parser.add_argument("--mode", choices=("temporal", "stratified"), default="temporal")
    parser.add_argument("--cutoff", default="2025-11", help="temporal mode: YYYY-MM starting NEW")
    parser.add_argument(
        "--undated",
        choices=("old", "new", "drop"),
        default="old",
        help="temporal mode: where scenes with no inferable date go",
    )
    parser.add_argument(
        "--holdout-frac", type=float, default=0.2, help="Holdout share of each half"
    )
    parser.add_argument(
        "--stratify-class",
        default="robot,house_bot",
        help="Comma-separated class names to balance alongside frame count",
    )
    parser.add_argument("--dry-run", action="store_true", help="Report the split, link nothing")
    return parser


def build_groups(
    ordered: list[Scene], args: argparse.Namespace, stratify: list[int]
) -> dict[str, list[Scene]]:
    """Divide scenes into the four groups and assert the result is scene-disjoint."""
    if args.mode == "temporal":
        old_side, new_side = split_temporal(ordered, args.cutoff, args.undated)
        print(
            f"temporal cutoff {args.cutoff}: "
            f"old {len(old_side)} scenes / {sum(s.n_frames for s in old_side)} frames, "
            f"new {len(new_side)} scenes / {sum(s.n_frames for s in new_side)} frames"
        )
    else:
        halves = assign_stratified(ordered, {"old": 0.5, "new": 0.5}, stratify)
        old_side, new_side = halves["old"], halves["new"]

    groups: dict[str, list[Scene]] = {}
    for side_name, side in (("old", old_side), ("new", new_side)):
        groups.update(
            assign_stratified(
                side,
                {side_name: 1.0 - args.holdout_frac, f"hold_{side_name}": args.holdout_frac},
                stratify,
            )
        )

    keys = [{s.key for s in groups[g]} for g in GROUPS]
    for i, a in enumerate(keys):
        for b in keys[i + 1 :]:
            if a & b:
                raise SystemExit(f"scene overlap between groups: {sorted(a & b)[:3]}")
    return {g: groups[g] for g in GROUPS}


def main() -> None:
    """Split a dataset into scene-disjoint old/new/holdout groups."""
    args = build_arg_parser().parse_args()
    src_yaml = args.src / "data.yml"
    if not src_yaml.is_file():
        raise SystemExit(f"no data.yml under {args.src}")
    meta = yaml.safe_load(src_yaml.read_text())
    names: list[str] = list(meta["names"])

    wanted = [n.strip() for n in args.stratify_class.split(",") if n.strip()]
    missing = [n for n in wanted if n not in names]
    if missing:
        raise SystemExit(f"--stratify-class names not in dataset: {missing}")
    stratify = [names.index(n) for n in wanted]

    scenes = collect_scenes(args.src)
    total_frames = sum(s.n_frames for s in scenes.values())
    undated = [s.key for s in scenes.values() if s.period is None]
    print(f"{len(scenes)} scenes, {total_frames} frames, stratifying on {wanted}")
    if undated:
        print(f"{len(undated)} scene(s) with no inferable date -> '{args.undated}': {undated}")

    groups = build_groups(sorted(scenes.values(), key=lambda s: s.key), args, stratify)
    print()
    manifest = report(groups, names, stratify)

    if args.dry_run:
        print("\ndry run: nothing linked")
        return

    args.out.mkdir(parents=True, exist_ok=True)
    print()
    for group, scenes_ in groups.items():
        linked = link_group(scenes_, args.out / group)
        print(f"linked {linked:6d} frames -> {args.out / group}")

    colors = meta.get("colors")
    for arm in ("old", "new", "old+new"):
        write_group_yaml(args.out, arm, names, colors)
    (args.out / "split_manifest.json").write_text(
        json.dumps(
            {
                "source": str(args.src.resolve()),
                "mode": args.mode,
                "cutoff": args.cutoff if args.mode == "temporal" else None,
                "holdout_frac": args.holdout_frac,
                "stratify_class": wanted,
                "groups": manifest,
            },
            indent=2,
        )
    )
    print(f"\nwrote {args.out}/split_manifest.json and old.yml / new.yml / old+new.yml")


if __name__ == "__main__":
    main()
