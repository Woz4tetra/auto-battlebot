#!/usr/bin/env python3
"""Build the clean DeepLab field corpus: deduplicated, scene-disjoint, field-stratified.

Reads the manifest from `build_field_manifest.py` and materializes a corpus whose
train/eval line no scene crosses, so val IoU measures generalization rather than
partial memorization. Also writes the Phase-B leave-one-field-out arms.

Where one image was annotated twice, the **newer** mask wins: the `floor_dataset__` twin
is a re-label, and carrying the older annotation would train and score against a
superseded one. Masks are renamed after their image, since the surviving mask may come
from the twin.

`--materialize copy` (the default) writes real files so the corpus stands alone and the
tree it was built from can be archived. Arms stay plain image lists either way -- 18 arms
of physical copies would be ~350 GB against 282 GB free.

Layout written under --out:

    train/, val/        image + mask pairs (semantic_train.py's default layout)
    eval -> val         the name score_masks.py is pointed at
    arms/full.txt       every train image; the Phase-A arm
    arms/lofo_<probe>_n<N>.txt
                        probe field held out entirely (n0), then added back at
                        N frames drawn scene-spread. Rungs are nested, asserted here.
    splits.json         which scenes went where, and why
    field_index.json    image name -> field / scene / split, read by semantic_train.py
                        for macro-average model selection and by score_masks.py

Single-scene fields cannot be held out scene-disjointly. They stay wholly in train and
are reported as having no eval representation rather than silently breaking the rule.

Usage:
    python3 training/deeplab/make_field_splits.py \
        --manifest training/data/deeplab_field_2026-07-28/manifest.json \
        --out training/data/deeplab_field_2026-07-28 \
        --val-frac 0.15 --drop-augment-copies --drop-duplicates --seed 4176
"""

from __future__ import annotations

import argparse
import json
import os
import random
import shutil
from collections import defaultdict
from itertools import combinations
from pathlib import Path

# Phase B probes. Cage-6 is unusable: removing a third of the corpus would make the base
# arm differ in size as well as field coverage.
PROBE_FIELDS = ("cage2", "cage5", "zed2023")
LADDER = (0, 50, 150, 400, 1000, "all")

# Exhaustive subset search stays cheap below this; the largest field has 15 scenes.
MAX_SCENES_FOR_EXHAUSTIVE = 18


def load_frames(manifest_path: Path, drop_augment: bool, drop_duplicates: bool) -> list[dict]:
    manifest = json.loads(manifest_path.read_text())
    if drop_duplicates and "duplicate_verification" not in manifest:
        raise SystemExit(
            "Manifest has no duplicate verification. Run build_field_manifest.py "
            "--verify-duplicates before dropping duplicates at scale."
        )
    frames = []
    for record in manifest["frames"]:
        if drop_augment and record["augment_index"] is not None:
            continue
        if drop_duplicates and record["duplicate_of"] is not None:
            continue
        frames.append(record)
    if not frames:
        raise SystemExit("No frames left after filtering")
    return frames


def choose_eval_scenes(
    scene_sizes: dict[str, int], val_frac: float, rng: random.Random
) -> list[str]:
    """Pick the scene subset whose frame share lands closest to val_frac.

    At least one scene must stay on each side, so a field with a single scene gets an
    empty selection and is handled by the caller.
    """
    scenes = sorted(scene_sizes)
    if len(scenes) < 2:
        return []
    total = sum(scene_sizes.values())
    target = val_frac * total

    if len(scenes) <= MAX_SCENES_FOR_EXHAUSTIVE:
        candidates = [
            subset for size in range(1, len(scenes)) for subset in combinations(scenes, size)
        ]
    else:
        # Greedy over a seeded shuffle: add scenes while that moves the total toward
        # the target. Only reachable for a field with more scenes than we have today.
        order = scenes[:]
        rng.shuffle(order)
        picked: list[str] = []
        running = 0
        for scene in order[:-1]:
            if abs(running + scene_sizes[scene] - target) < abs(running - target):
                picked.append(scene)
                running += scene_sizes[scene]
        return picked or [order[0]]

    def score(subset: tuple[str, ...]) -> tuple[float, int]:
        held = sum(scene_sizes[s] for s in subset)
        # Fewer scenes breaks ties: a holdout concentrated in one recording is a
        # cleaner statement about that recording than one smeared across many.
        return abs(held - target), len(subset)

    best = min(candidates, key=score)
    return list(best)


def scene_spread_order(frames_by_scene: dict[str, list[str]], rng: random.Random) -> list[str]:
    """One global ordering of a field's frames, round-robin across its scenes.

    Prefixes of this list are the ladder rungs, which is what makes them nested: a drop
    between rungs cannot be blamed on which frames happened to be drawn. Round-robin
    spreads a small budget across recordings rather than spending it all on one, which
    is the choice `data_scaling` found mattered more than raw frame count.
    """
    pools = {}
    for scene, names in sorted(frames_by_scene.items()):
        shuffled = sorted(names)
        rng.shuffle(shuffled)
        pools[scene] = shuffled

    order: list[str] = []
    scenes = sorted(pools)
    index = 0
    while any(pools[s] for s in scenes):
        scene = scenes[index % len(scenes)]
        if pools[scene]:
            order.append(pools[scene].pop())
        index += 1
    return order


def mask_for(record: dict) -> str:
    """The mask to use for a frame: the newer annotation when the image was labelled twice.

    `newer_mask` is set by build_field_manifest.py --verify-duplicates. Without it the
    record's own mask is the only one there is.
    """
    return record.get("newer_mask", record["mask"])


def build_tree(records: list[dict], source_root: Path, target_dir: Path, materialize: str) -> None:
    """Write one split's images and masks. Rebuilt from scratch every run.

    The mask is always named after its image (`<stem>_mask.png`), which matters because
    the surviving mask may come from the frame's duplicate twin and so carry a different
    name in the source tree.

    `copy` makes the corpus standalone -- it no longer refers to the tree it was built
    from, so that tree can be archived. `symlink` costs nothing but keeps the dependency.
    """
    replace_path(target_dir)
    target_dir.mkdir(parents=True)
    for record in records:
        image_name = Path(record["path"]).name
        pairs = (
            (record["path"], image_name),
            (mask_for(record), f"{Path(image_name).stem}_mask.png"),
        )
        for relative, name in pairs:
            source = (source_root / relative).resolve()
            target = target_dir / name
            if materialize == "copy":
                shutil.copy2(source, target)
            else:
                target.symlink_to(os.path.relpath(source, target_dir))


def write_list(path: Path, records: list[dict]) -> None:
    """Image list, one path per line, relative to the corpus root (`train/<name>.jpg`)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = sorted(f"{r['_split_dir']}/{Path(r['path']).name}" for r in records)
    path.write_text("\n".join(lines) + "\n")


def replace_path(path: Path) -> None:
    """Clear a symlink or directory so it can be rebuilt from scratch."""
    if path.is_symlink():
        path.unlink()
    elif path.exists():
        shutil.rmtree(path)


def assign_splits(
    frames: list[dict], val_frac: float, rng: random.Random
) -> tuple[list[dict], list[dict], dict[str, dict], list[str]]:
    """Send whole scenes to train or eval, targeting val_frac within each field."""
    by_field: dict[str, dict[str, list[dict]]] = defaultdict(lambda: defaultdict(list))
    for record in frames:
        by_field[record["field"]][record["scene"]].append(record)

    train: list[dict] = []
    evaluation: list[dict] = []
    report: dict[str, dict] = {}
    no_eval_fields: list[str] = []

    for field in sorted(by_field, key=lambda f: -sum(len(v) for v in by_field[f].values())):
        scenes = by_field[field]
        sizes = {scene: len(records) for scene, records in scenes.items()}
        eval_scenes = choose_eval_scenes(sizes, val_frac, rng)
        if not eval_scenes:
            no_eval_fields.append(field)
        for scene, records in scenes.items():
            target = evaluation if scene in eval_scenes else train
            target.extend(records)
        held = sum(sizes[s] for s in eval_scenes)
        total = sum(sizes.values())
        report[field] = {
            "frames": total,
            "scenes": len(sizes),
            "eval_scenes": sorted(eval_scenes),
            "eval_frames": held,
            "train_frames": total - held,
            "eval_share": held / total if total else 0.0,
        }
    return train, evaluation, report, no_eval_fields


def print_split_report(
    report: dict[str, dict],
    train_count: int,
    eval_count: int,
    total: int,
    no_eval_fields: list[str],
) -> None:
    print(f"\n{'field':<16}{'frames':>8}{'train':>8}{'eval':>8}{'eval%':>8}  eval scenes")
    for field, entry in report.items():
        print(
            f"{field:<16}{entry['frames']:>8,}{entry['train_frames']:>8,}"
            f"{entry['eval_frames']:>8,}{entry['eval_share'] * 100:>7.1f}%  "
            f"{len(entry['eval_scenes'])}/{entry['scenes']}"
        )
    print(f"\ntotal: train {train_count:,}  eval {eval_count:,} ({eval_count / total * 100:.1f}%)")
    if no_eval_fields:
        print(
            "\nNo eval representation (single scene, cannot be held out scene-disjointly): "
            f"{', '.join(no_eval_fields)}"
        )
        print("  These stay wholly in train and are excluded from macro-average and worst-field.")


def build_probe_arms(
    probe: str,
    train: list[dict],
    evaluation: list[dict],
    arms_dir: Path,
    rng: random.Random,
) -> dict | None:
    """Write one probe's leave-one-field-out ladder. Returns its report, or None."""
    probe_train = [r for r in train if r["field"] == probe]
    if not probe_train:
        print(f"\nSkipping probe {probe!r}: no train frames")
        return None
    base = [r for r in train if r["field"] != probe]

    frames_by_scene: dict[str, list[str]] = defaultdict(list)
    by_name = {Path(r["path"]).name: r for r in probe_train}
    for record in probe_train:
        frames_by_scene[record["scene"]].append(Path(record["path"]).name)
    order = scene_spread_order(frames_by_scene, rng)

    previous: set[str] = set()
    rungs: dict[str, int] = {}
    for rung in LADDER:
        count = len(order) if rung == "all" else min(int(rung), len(order))
        picked = order[:count]
        assert previous <= set(picked), f"{probe} rung {rung} is not nested in its predecessor"
        previous = set(picked)
        arm = base + [by_name[name] for name in picked]
        write_list(arms_dir / f"lofo_{probe}_n{rung}.txt", arm)
        rungs[str(rung)] = len(arm)

    assert rungs["all"] == len(train), (
        f"{probe} n=all ({rungs['all']:,}) should equal the full train arm ({len(train):,})"
    )
    report = {
        "probe_train_frames": len(probe_train),
        "probe_train_scenes": len(frames_by_scene),
        "probe_eval_frames": sum(1 for r in evaluation if r["field"] == probe),
        "base_frames": len(base),
        "rungs": rungs,
    }
    print(
        f"\nprobe {probe}: {len(probe_train):,} train frames over "
        f"{len(frames_by_scene)} scenes, {report['probe_eval_frames']:,} eval frames"
    )
    print(f"  arms: {rungs}")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True, help="clean corpus root to write")
    parser.add_argument("--val-frac", type=float, default=0.15, help="target eval share per field")
    parser.add_argument("--drop-augment-copies", action="store_true")
    parser.add_argument("--drop-duplicates", action="store_true")
    parser.add_argument("--seed", type=int, default=4176)
    parser.add_argument(
        "--materialize",
        choices=("copy", "symlink"),
        default="copy",
        help="copy (default) makes the corpus standalone so the source tree can be "
        "archived; symlink costs no disk but keeps the dependency",
    )
    parser.add_argument(
        "--probes",
        default=",".join(PROBE_FIELDS),
        help="fields to build leave-one-field-out arms for",
    )
    args = parser.parse_args()

    manifest = json.loads(args.manifest.read_text())
    source_root = Path(manifest["root"])
    frames = load_frames(args.manifest, args.drop_augment_copies, args.drop_duplicates)
    print(f"{len(frames):,} frames after filtering (from {len(manifest['frames']):,} files)")

    rng = random.Random(args.seed)

    train, evaluation, split_report, no_eval_fields = assign_splits(frames, args.val_frac, rng)
    print_split_report(split_report, len(train), len(evaluation), len(frames), no_eval_fields)

    for record in train:
        record["_split_dir"] = "train"
    for record in evaluation:
        record["_split_dir"] = "val"

    args.out.mkdir(parents=True, exist_ok=True)
    relabelled = sum(1 for r in train + evaluation if "newer_mask" in r)
    print(f"\nWriting train/ and val/ ({args.materialize}) ...")
    if relabelled:
        print(f"  {relabelled:,} frames take the newer of their two annotations")
    build_tree(train, source_root, args.out / "train", args.materialize)
    build_tree(evaluation, source_root, args.out / "val", args.materialize)

    eval_alias = args.out / "eval"
    replace_path(eval_alias)
    eval_alias.symlink_to("val")

    arms_dir = args.out / "arms"
    arms_dir.mkdir(exist_ok=True)
    write_list(arms_dir / "full.txt", train)

    arm_report: dict[str, dict] = {}
    for probe in (p for p in args.probes.split(",") if p):
        report = build_probe_arms(probe, train, evaluation, arms_dir, rng)
        if report is not None:
            arm_report[probe] = report

    field_index = {
        Path(r["path"]).name: {
            "field": r["field"],
            "scene": r["scene"],
            "split": r["_split_dir"],
        }
        for r in train + evaluation
    }
    (args.out / "field_index.json").write_text(json.dumps(field_index, indent=1))

    (args.out / "splits.json").write_text(
        json.dumps(
            {
                "manifest": str(args.manifest),
                "source_root": str(source_root),
                "seed": args.seed,
                "val_frac": args.val_frac,
                "dropped_augment_copies": args.drop_augment_copies,
                "dropped_duplicates": args.drop_duplicates,
                "materialize": args.materialize,
                "frames_using_newer_relabel": relabelled,
                "train_frames": len(train),
                "eval_frames": len(evaluation),
                "fields_without_eval": no_eval_fields,
                "by_field": split_report,
                "arms": arm_report,
            },
            indent=1,
        )
    )
    print(f"\nWrote {args.out}/{{train,val,eval,arms,splits.json,field_index.json}}")


if __name__ == "__main__":
    main()
