#!/usr/bin/env python3
"""Drop, prune, rename, and renumber classes in a YOLO dataset.

Unlike remap_labels.py (which applies an explicit source->target ID map and cannot delete
annotations), this removes whole classes: it deletes their annotation lines, renumbers the
surviving classes to a contiguous 0..N-1 range, applies renames, and rewrites data.yml. Use
it to strip a dominant non-target class (e.g. an arena "Floor" mask) or to prune classes
that ended up with zero instances after a merge.

Works for detection and segmentation label files (class id followed by box or polygon
coords); the geometry columns are preserved verbatim. Keypoint kpt_shape/flip_idx in
data.yml are carried through.

A frame whose only annotations were dropped keeps its image with an empty label file, i.e.
a background negative. Pass --drop-empty-images to remove those image/label pairs instead.

Layouts supported: split dirs (train/ val/ test/, each with images/ + labels/) or a single
flat images/ + labels/ pair at the dataset root.

Usage:
    # in place: drop the Floor class and every zero-instance class, renumber survivors
    python edit_yolo_classes.py training/data/nhrl_robots_indiv --drop Floor --drop-unused

    # write to a new dir, also renaming a class
    python edit_yolo_classes.py <dataset> --drop Floor --drop-unused \
        --rename 'Púca=Puca' --output <dataset>_clean
"""

from __future__ import annotations

import argparse
import collections
import colorsys
import shutil
from pathlib import Path

import yaml

IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp")
SPLIT_NAMES = ("train", "val", "test")


def load_names(meta: dict) -> list[str]:
    """Extract class names as a list, supporting the list and dict data.yml forms."""
    raw = meta.get("names", [])
    if isinstance(raw, list):
        return [str(v) for v in raw]
    if isinstance(raw, dict):
        parsed = {int(k): str(v) for k, v in raw.items()}
        return [parsed.get(i, f"class_{i}") for i in range(max(parsed) + 1)] if parsed else []
    return []


def find_data_yaml(root: Path) -> Path:
    for name in ("data.yml", "data.yaml"):
        if (root / name).exists():
            return root / name
    raise SystemExit(f"No data.yml/data.yaml in {root}")


def find_split_dirs(root: Path) -> list[tuple[Path, Path]]:
    """Return (images_dir, labels_dir) pairs for each split, or the flat root layout."""
    pairs = []
    for split in SPLIT_NAMES:
        images, labels = root / split / "images", root / split / "labels"
        if labels.is_dir():
            pairs.append((images, labels))
    if pairs:
        return pairs
    if (root / "labels").is_dir():
        return [(root / "images", root / "labels")]
    raise SystemExit(f"No train/val/test or images/labels layout under {root}")


def count_instances(split_dirs: list[tuple[Path, Path]]) -> collections.Counter[int]:
    counts: collections.Counter[int] = collections.Counter()
    for _images, labels in split_dirs:
        for label_path in labels.glob("*.txt"):
            for line in label_path.read_text().splitlines():
                if line.strip():
                    counts[int(line.split()[0])] += 1
    return counts


def resolve_drop_ids(
    names: list[str],
    drop_names: list[str],
    drop_unused: bool,
    counts: collections.Counter[int],
) -> set[int]:
    """Class ids to remove: the named classes plus, optionally, zero-instance classes."""
    name_to_id = {name: i for i, name in enumerate(names)}
    drop_ids: set[int] = set()
    for name in drop_names:
        if name not in name_to_id:
            raise SystemExit(f"--drop {name!r}: not a class in this dataset")
        drop_ids.add(name_to_id[name])
    if drop_unused:
        drop_ids |= {i for i in range(len(names)) if counts[i] == 0}
    return drop_ids


def build_new_schema(
    meta: dict, names: list[str], drop_ids: set[int], renames: dict[str, str]
) -> tuple[list[str], dict[int, int], dict]:
    """Return (new_names, old_id->new_id map, new data.yml dict)."""
    kept_ids = [i for i in range(len(names)) if i not in drop_ids]
    old_to_new = {old: new for new, old in enumerate(kept_ids)}

    new_names = [names[i] for i in kept_ids]
    for old_name, new_name in renames.items():
        if old_name not in new_names:
            raise SystemExit(f"--rename {old_name!r}: not a surviving class")
        new_names[new_names.index(old_name)] = new_name
    if len(set(new_names)) != len(new_names):
        raise SystemExit("rename produced a duplicate class name")

    src_colors = meta.get("colors")
    if isinstance(src_colors, list) and len(src_colors) == len(names):
        colors = [src_colors[i] for i in kept_ids]
    else:
        colors = generate_colors(len(new_names))

    new_meta: dict = {}
    if "path" in meta:
        new_meta["path"] = meta["path"]
    for split in SPLIT_NAMES:
        if split in meta:
            new_meta[split] = meta[split]
    new_meta["nc"] = len(new_names)
    new_meta["names"] = new_names
    new_meta["colors"] = colors
    for key in ("kpt_shape", "flip_idx"):
        if key in meta:
            new_meta[key] = meta[key]
    return new_names, old_to_new, new_meta


def generate_colors(n: int) -> list[str]:
    """Evenly spaced hues so the in-repo label editors get distinct per-class colors."""
    colors = []
    for i in range(n):
        r, g, b = colorsys.hsv_to_rgb((i * 0.61803) % 1.0, 0.85, 0.95)
        colors.append(f"#{int(r * 255):02x}{int(g * 255):02x}{int(b * 255):02x}")
    return colors


def remap_label_text(text: str, drop_ids: set[int], old_to_new: dict[int, int]) -> str:
    """Drop lines for removed classes and renumber the rest. Empty result -> background."""
    out_lines = []
    for line in text.splitlines():
        if not line.strip():
            continue
        parts = line.split()
        cid = int(parts[0])
        if cid in drop_ids:
            continue
        parts[0] = str(old_to_new[cid])
        out_lines.append(" ".join(parts))
    return "\n".join(out_lines) + ("\n" if out_lines else "")


def find_image_for_label(images_dir: Path, stem: str) -> Path | None:
    for ext in IMAGE_EXTENSIONS:
        for candidate in (images_dir / f"{stem}{ext}", images_dir / f"{stem}{ext.upper()}"):
            if candidate.exists():
                return candidate
    return None


def process_split(
    images_dir: Path,
    labels_dir: Path,
    out_images: Path,
    out_labels: Path,
    drop_ids: set[int],
    old_to_new: dict[int, int],
    in_place: bool,
    drop_empty_images: bool,
) -> tuple[int, int, int]:
    """Rewrite one split. Returns (kept_images, emptied, removed_images)."""
    if not in_place:
        out_images.mkdir(parents=True, exist_ok=True)
        out_labels.mkdir(parents=True, exist_ok=True)
    kept = emptied = removed = 0
    for label_path in sorted(labels_dir.glob("*.txt")):
        new_text = remap_label_text(label_path.read_text(), drop_ids, old_to_new)
        is_empty = not new_text.strip()
        image_path = find_image_for_label(images_dir, label_path.stem)
        if is_empty and drop_empty_images:
            removed += 1
            if in_place:
                label_path.unlink()
                if image_path is not None:
                    image_path.unlink()
            continue
        if is_empty:
            emptied += 1
        kept += 1
        if in_place:
            label_path.write_text(new_text)
        else:
            (out_labels / label_path.name).write_text(new_text)
            if image_path is not None:
                shutil.copy2(image_path, out_images / image_path.name)
    return kept, emptied, removed


def parse_renames(pairs: list[str]) -> dict[str, str]:
    renames: dict[str, str] = {}
    for pair in pairs:
        if "=" not in pair:
            raise SystemExit(f"--rename {pair!r}: expected 'OLD=NEW'")
        old, new = pair.split("=", 1)
        renames[old] = new
    return renames


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("dataset", type=Path, help="dataset root (has data.yml + splits)")
    parser.add_argument(
        "--drop", action="append", default=[], metavar="NAME", help="class name to remove"
    )
    parser.add_argument(
        "--drop-unused", action="store_true", help="also remove classes with zero instances"
    )
    parser.add_argument(
        "--rename", action="append", default=[], metavar="OLD=NEW", help="rename a class"
    )
    parser.add_argument(
        "--drop-empty-images",
        action="store_true",
        help="remove images whose annotations were all dropped (default: keep as background)",
    )
    parser.add_argument(
        "--output", type=Path, help="write to a new dir (copies images); default: edit in place"
    )
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    root: Path = args.dataset
    in_place = args.output is None
    out_root: Path = root if in_place else args.output

    meta = yaml.safe_load(find_data_yaml(root).read_text())
    names = load_names(meta)
    if not names:
        raise SystemExit(f"No class names in {root}")

    split_dirs = find_split_dirs(root)
    counts = count_instances(split_dirs)
    drop_ids = resolve_drop_ids(names, args.drop, args.drop_unused, counts)
    if not drop_ids and not args.rename:
        raise SystemExit("nothing to do: pass --drop, --drop-unused, and/or --rename")
    renames = parse_renames(args.rename)

    new_names, old_to_new, new_meta = build_new_schema(meta, names, drop_ids, renames)
    if not in_place and "path" in new_meta:
        new_meta["path"] = str(out_root.resolve())

    dropped = sorted(names[i] for i in drop_ids)
    print(f"dropping {len(drop_ids)} classes: {dropped}")
    print(f"kept classes: {len(new_names)}")

    total_kept = total_emptied = total_removed = 0
    for images_dir, labels_dir in split_dirs:
        rel = labels_dir.parent.relative_to(root)
        out_images = out_root / rel / "images"
        out_labels = out_root / rel / "labels"
        kept, emptied, removed = process_split(
            images_dir,
            labels_dir,
            out_images,
            out_labels,
            drop_ids,
            old_to_new,
            in_place,
            args.drop_empty_images,
        )
        total_kept += kept
        total_emptied += emptied
        total_removed += removed
        note = f"{emptied} now background" if not args.drop_empty_images else f"{removed} removed"
        print(f"  {rel}: {kept} images kept, {note}")

    (out_root / find_data_yaml(root).name).write_text(
        yaml.safe_dump(new_meta, sort_keys=False, allow_unicode=True)
    )

    # Sanity: every surviving label id is in range.
    max_id = -1
    for _images, labels in find_split_dirs(out_root):
        for label_path in labels.glob("*.txt"):
            for line in label_path.read_text().splitlines():
                if line.strip():
                    max_id = max(max_id, int(line.split()[0]))
    if max_id >= len(new_names):
        raise SystemExit(f"label id {max_id} out of range for {len(new_names)} classes")

    print(f"images kept: {total_kept}, background: {total_emptied}, removed: {total_removed}")
    print(f"max label id: {max_id} (classes: {len(new_names)})")
    print(f"wrote {out_root / find_data_yaml(root).name}")


if __name__ == "__main__":
    main()
