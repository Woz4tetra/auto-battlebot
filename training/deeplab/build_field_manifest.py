#!/usr/bin/env python3
"""Inventory the floor-mask corpus: field type, scene, and provenance per frame.

The corpus is a flat pile of `<name>.jpg` + `<name>_mask.png` pairs under train/ val/
test/, and the filename is the only provenance. This script reads that filename for
every frame and writes a manifest recording:

    field / scene   the arena and the recording (see field_labels.py)
    source          the labelled frame identity; offline `_augment-N` copies share it
    duplicate_of    the unprefixed twin of a `floor_dataset__`-prefixed file

Nothing is deleted or moved -- the only write is the manifest. `make_field_splits.py`
consumes it to build the clean, scene-disjoint corpus.

Duplicate pairing is by name. `--verify-duplicates` re-reads the manifest and hashes
both sides of every pair, so the claim is checked at full scale before
`make_field_splits.py` acts on it; a pair whose bytes differ is un-paired and flagged
rather than dropped.

Usage:
    python3 training/deeplab/build_field_manifest.py training/data/floor_mask_dataset \
        --out training/data/deeplab_field_2026-07-28/manifest.json

    python3 training/deeplab/build_field_manifest.py training/data/floor_mask_dataset \
        --manifest training/data/deeplab_field_2026-07-28/manifest.json \
        --verify-duplicates --hash md5
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from collections import Counter, defaultdict
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
import numpy as np
from field_labels import parse_name
from tqdm import tqdm

MANIFEST_VERSION = 1
SPLIT_DIRS = ("train", "val", "test")
HASH_CHUNK_BYTES = 1 << 20


def scan_split(root: Path, split: str) -> list[dict]:
    """Every image in one split dir, labelled. Images without a mask are skipped."""
    split_dir = root / split
    if not split_dir.is_dir():
        return []
    records = []
    missing_masks = []
    for image_path in sorted(split_dir.glob("*.jpg")):
        mask_path = image_path.with_name(f"{image_path.stem}_mask.png")
        if not mask_path.exists():
            missing_masks.append(image_path.name)
            continue
        label = parse_name(image_path.stem)
        records.append(
            {
                "path": f"{split}/{image_path.name}",
                "mask": f"{split}/{mask_path.name}",
                "split": split,
                "canonical": label.canonical,
                "source": label.source,
                "scene": label.scene,
                "field": label.field,
                "augment_index": label.augment_index,
                "prefixed": label.is_prefixed,
                "bytes": image_path.stat().st_size,
                "duplicate_of": None,
            }
        )
    if missing_masks:
        print(
            f"  {split}: skipped {len(missing_masks)} image(s) with no mask "
            f"(first: {missing_masks[0]})",
            file=sys.stderr,
        )
    return records


def pair_duplicates(records: list[dict]) -> int:
    """Point each `floor_dataset__`-prefixed record at its unprefixed twin.

    Pairing is by canonical name, across splits -- putting the same pixels in train
    under one name and val under another is exactly the leak this has to surface.
    Returns the number of pairs made.
    """
    by_canonical: dict[str, list[dict]] = defaultdict(list)
    for record in records:
        by_canonical[record["canonical"]].append(record)

    paired = 0
    for group in by_canonical.values():
        originals = [r for r in group if not r["prefixed"]]
        prefixed = [r for r in group if r["prefixed"]]
        if not originals or not prefixed:
            continue
        # More than one original would make "the twin" ambiguous; take the first in
        # sorted order so the manifest is reproducible, and let the summary report it.
        original = min(originals, key=lambda r: r["path"])
        for record in prefixed:
            record["duplicate_of"] = original["path"]
            paired += 1
    return paired


def summarize(records: list[dict]) -> dict:
    """Counts the plan's tables are checked against: per-field frames/scenes, defects."""
    unique = [r for r in records if r["augment_index"] is None and r["duplicate_of"] is None]

    by_field: dict[str, dict] = {}
    for record in unique:
        entry = by_field.setdefault(record["field"], {"frames": 0, "scenes": set()})
        entry["frames"] += 1
        entry["scenes"].add(record["scene"])

    # The split was drawn per-file, so both kinds of copy leak across it. Counted
    # separately because they are separate defects: the augment figure stands even if
    # the duplicate claim were to fall over.
    augment_splits: dict[str, set[str]] = defaultdict(set)
    all_splits: dict[str, set[str]] = defaultdict(set)
    for record in records:
        all_splits[record["source"]].add(record["split"])
        if not record["prefixed"]:
            augment_splits[record["source"]].add(record["split"])
    leaked_augment = sum(1 for splits in augment_splits.values() if len(splits) > 1)
    leaked_any = sum(1 for splits in all_splits.values() if len(splits) > 1)

    return {
        "files": len(records),
        "augment_copies": sum(1 for r in records if r["augment_index"] is not None),
        "duplicates": sum(1 for r in records if r["duplicate_of"] is not None),
        "unique_frames": len(unique),
        "scenes": len({r["scene"] for r in unique}),
        "source_frames_leaked_by_augment": leaked_augment,
        "source_frames_leaked_any_copy": leaked_any,
        "by_split": {split: sum(1 for r in records if r["split"] == split) for split in SPLIT_DIRS},
        "by_field": {
            field: {"frames": entry["frames"], "scenes": len(entry["scenes"])}
            for field, entry in sorted(by_field.items(), key=lambda kv: -kv[1]["frames"])
        },
    }


def print_summary(summary: dict) -> None:
    print(f"\nFiles on disk:            {summary['files']:>7,}")
    print(f"  offline augment copies: {summary['augment_copies']:>7,}")
    print(f"  prefixed duplicates:    {summary['duplicates']:>7,}")
    print(f"  unique labelled frames: {summary['unique_frames']:>7,}")
    print(f"  scenes:                 {summary['scenes']:>7,}")
    print(f"  split as shipped:       {summary['by_split']}")
    print("  source frames straddling the train/val line:")
    print(f"    via augment siblings:  {summary['source_frames_leaked_by_augment']:>7,}")
    print(f"    via any copy:          {summary['source_frames_leaked_any_copy']:>7,}")
    total = summary["unique_frames"] or 1
    print(f"\n{'field':<16}{'frames':>9}{'scenes':>9}{'share':>9}")
    for field, entry in summary["by_field"].items():
        share = 100.0 * entry["frames"] / total
        print(f"{field:<16}{entry['frames']:>9,}{entry['scenes']:>9,}{share:>8.1f}%")


def file_digest(path: Path, algorithm: str) -> str:
    digest = hashlib.new(algorithm)
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(HASH_CHUNK_BYTES), b""):
            digest.update(chunk)
    return digest.hexdigest()


def mask_agreement(root: Path, left: str, right: str) -> float:
    """Foreground IoU between two labelings of one image. 1.0 when they agree exactly."""
    a = cv2.imread(str(root / left), cv2.IMREAD_UNCHANGED)
    b = cv2.imread(str(root / right), cv2.IMREAD_UNCHANGED)
    if a is None or b is None:
        return float("nan")
    a = a[:, :, 0] if a.ndim == 3 else a
    b = b[:, :, 0] if b.ndim == 3 else b
    if a.shape != b.shape:
        return 0.0
    fg_a, fg_b = a > 0, b > 0
    union = int((fg_a | fg_b).sum())
    return 1.0 if union == 0 else float((fg_a & fg_b).sum()) / union


def verify_duplicates(manifest: dict, root: Path, algorithm: str, workers: int) -> dict:
    """Hash both sides of every duplicate pair and record what actually matches.

    A pair is un-paired only when the *images* differ, because the leak this exists to
    remove is the same pixels appearing on both sides of the split -- a pair that shares
    an image but disagrees on the mask is still one frame, still leaks, and is kept
    paired with `mask_differs` set so the disagreement stays auditable. Their mask IoU is
    measured too: two labelings of one image bound the label noise in this corpus, which
    is what boundary F1 has to be read against.

    Mutates the manifest's records in place and returns the verification report.
    """
    records = manifest["frames"]
    by_path = {r["path"]: r for r in records}
    pairs = [r for r in records if r["duplicate_of"] is not None]
    if not pairs:
        return {"pairs": 0, "identical": 0, "differing": [], "algorithm": algorithm}

    # One hash per distinct file: an original with several prefixed copies is read once.
    wanted: set[str] = set()
    for record in pairs:
        original = by_path[record["duplicate_of"]]
        wanted.update((record["path"], record["mask"], original["path"], original["mask"]))

    ordered = sorted(wanted)
    with ThreadPoolExecutor(max_workers=workers) as pool:
        digests = dict(
            zip(
                ordered,
                tqdm(
                    pool.map(lambda rel: file_digest(root / rel, algorithm), ordered),
                    total=len(ordered),
                    desc=f"{algorithm} {len(pairs):,} pairs",
                    dynamic_ncols=True,
                ),
            )
        )

    identical = 0
    mask_only: list[dict] = []
    image_differs: list[dict] = []
    for record in pairs:
        original = by_path[record["duplicate_of"]]
        image_same = digests[record["path"]] == digests[original["path"]]
        mask_same = digests[record["mask"]] == digests[original["mask"]]
        if image_same and mask_same:
            identical += 1
            continue
        entry = {"duplicate": record["path"], "original": original["path"]}
        if image_same:
            record["mask_differs"] = True
            # Same photo, two annotations. The newer one is the re-label, and it is what
            # the clean corpus has to carry -- keeping the older mask would silently train
            # and score against a superseded annotation. mtime is the only ordering the
            # corpus records, so it is what decides.
            newer = max(
                (record["mask"], original["mask"]),
                key=lambda rel: (root / rel).stat().st_mtime,
            )
            original["newer_mask"] = newer
            entry["newer_mask"] = newer
            entry["newer_side"] = "duplicate" if newer == record["mask"] else "original"
            mask_only.append(entry)
        else:
            # Different picture: not a duplicate at all. Keep it and let it into the corpus.
            record["duplicate_of"] = None
            record["image_differs"] = True
            image_differs.append(entry)

    agreement: list[float] = []
    if mask_only:
        with ThreadPoolExecutor(max_workers=workers) as pool:
            agreement = list(
                tqdm(
                    pool.map(
                        lambda e: mask_agreement(
                            root, by_path[e["duplicate"]]["mask"], by_path[e["original"]]["mask"]
                        ),
                        mask_only,
                    ),
                    total=len(mask_only),
                    desc="mask IoU on relabelled pairs",
                    dynamic_ncols=True,
                )
            )
        for entry, iou in zip(mask_only, agreement):
            entry["mask_iou"] = iou

    report = {
        "pairs": len(pairs),
        "identical": identical,
        "same_image_different_mask": len(mask_only),
        "different_image": len(image_differs),
        "algorithm": algorithm,
        "relabel_pairs": mask_only,
        "image_differs": image_differs,
    }
    if agreement:
        values = np.asarray(agreement, dtype=np.float64)
        report["relabel_mask_iou"] = {
            "min": float(values.min()),
            "p05": float(np.percentile(values, 5)),
            "median": float(np.median(values)),
            "mean": float(values.mean()),
            "max": float(values.max()),
        }
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("dataset", type=Path, help="floor-mask corpus root (train/ val/ test/)")
    parser.add_argument("--out", type=Path, help="manifest to write (default: --manifest path)")
    parser.add_argument("--manifest", type=Path, help="existing manifest to re-read and update")
    parser.add_argument(
        "--verify-duplicates",
        action="store_true",
        help="hash both sides of every duplicate pair and un-pair any that differ",
    )
    parser.add_argument("--hash", default="md5", help="hash algorithm for --verify-duplicates")
    parser.add_argument("--workers", type=int, default=16, help="hashing threads")
    args = parser.parse_args()

    root = args.dataset
    if not root.is_dir():
        raise SystemExit(f"Dataset root not found: {root}")

    out_path = args.out or args.manifest
    if out_path is None:
        raise SystemExit("Pass --out to build a manifest, or --manifest to update one")

    if args.manifest and args.manifest.exists() and not args.out:
        manifest = json.loads(args.manifest.read_text())
        print(f"Loaded {len(manifest['frames']):,} records from {args.manifest}")
    else:
        print(f"Scanning {root}")
        records: list[dict] = []
        for split in SPLIT_DIRS:
            split_records = scan_split(root, split)
            print(f"  {split}: {len(split_records):,} images")
            records.extend(split_records)
        if not records:
            raise SystemExit(f"No image/mask pairs found under {root}")
        paired = pair_duplicates(records)
        print(f"  paired {paired:,} prefixed duplicates with their originals")
        manifest = {
            "version": MANIFEST_VERSION,
            "root": str(root),
            "frames": records,
        }

    if args.verify_duplicates:
        manifest["duplicate_verification"] = verify_duplicates(
            manifest, root, args.hash, args.workers
        )
        report = manifest["duplicate_verification"]
        print(f"\nVerified {report['pairs']:,} pairs ({report['algorithm']}):")
        print(f"  byte-identical:              {report['identical']:>7,}")
        relabelled = report["same_image_different_mask"]
        print(f"  same image, mask relabelled: {relabelled:>7,}  (kept paired)")
        print(f"  different image:             {report['different_image']:>7,}  (un-paired)")
        sides = Counter(e["newer_side"] for e in report["relabel_pairs"])
        if sides:
            print(f"  newer mask lives on the: {dict(sides)}  (that one is kept)")
        stats = report.get("relabel_mask_iou")
        if stats:
            print(
                "  mask IoU across the two labelings of one image: "
                f"min {stats['min']:.3f}  p05 {stats['p05']:.3f}  median {stats['median']:.3f}"
            )
            print("  ^ an empirical label-noise floor; boundary F1 has to be read against it.")
        for entry in report["image_differs"][:10]:
            print(f"  different image: {entry['duplicate']} vs {entry['original']}")

    manifest["summary"] = summarize(manifest["frames"])
    print_summary(manifest["summary"])

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(manifest, indent=1))
    print(f"\nWrote {out_path}")


if __name__ == "__main__":
    main()
