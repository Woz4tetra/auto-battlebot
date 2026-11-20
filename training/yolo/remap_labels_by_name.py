"""Merge per-scene YOLO datasets into one, remapping classes **by name**."""

from __future__ import annotations

import argparse
import json
import shutil
from collections import Counter, defaultdict
from pathlib import Path

import yaml
from seg_to_bbox import top_contours_bbox

from auto_battlebot.compat import tomllib

IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp")
DROP = "__drop__"


def load_config(path: Path) -> tuple[list[str], dict[str, str], str]:
    """Return (target names, name->target map, default target)."""
    cfg = tomllib.loads(path.read_text())
    names: list[str] = list(cfg["target_names"])
    mapping: dict[str, str] = dict(cfg.get("map", {}))
    default: str = cfg["default"]
    for target in set(mapping.values()) | {default}:
        if target != DROP and target not in names:
            raise SystemExit(f"config maps to unknown target {target!r}; target_names={names}")
    return names, mapping, default


def scene_dirs(src: Path) -> list[Path]:
    """Per-scene dataset directories (each with its own data.yml + images/ + labels/)."""
    return sorted(d for d in src.iterdir() if d.is_dir() and (d / "data.yml").is_file())


def passing_frames(scene: Path) -> set[str] | None:
    """Image stems marked `pass` in the scene's validation_state.json, or None if untracked."""
    state = scene / "validation_state.json"
    if not state.is_file():
        return None
    try:
        verdicts = json.loads(state.read_text())
    except (OSError, ValueError):
        return None
    return {Path(k).stem for k, v in verdicts.items() if v == "pass"}


def find_image(images_dir: Path, stem: str) -> Path | None:
    """Locate the image paired with a label stem."""
    for ext in IMAGE_EXTENSIONS:
        candidate = images_dir / (stem + ext)
        if candidate.is_file():
            return candidate
    return None


def convert_rows(
    text: str, index_map: dict[int, int], to_bbox: bool, min_side: float = 0.0
) -> list[str]:
    """Remap and optionally box-ify one label file's rows. Dropped classes vanish."""
    out: list[str] = []
    by_class: dict[int, list[list[float]]] = defaultdict(list)
    for raw in text.splitlines():
        parts = raw.split()
        if not parts:
            continue
        try:
            src_cls = int(float(parts[0]))
            coords = [float(v) for v in parts[1:]]
        except ValueError:
            continue
        dst = index_map.get(src_cls)
        if dst is None:
            continue
        if not to_bbox or len(coords) == 4:
            out.append(" ".join([str(dst), *(f"{c:.6f}" for c in coords)]))
            continue
        by_class[src_cls].append(coords)

    for src_cls in sorted(by_class):
        box = top_contours_bbox(by_class[src_cls])
        if box is None:
            continue
        if min_side and min(box[2], box[3]) < min_side:
            continue
        out.append(f"{index_map[src_cls]} " + " ".join(f"{v:.6f}" for v in box))
    return out


def build_index_map(
    scene_names: list[str], mapping: dict[str, str], default: str, targets: list[str]
) -> tuple[dict[int, int], set[str]]:
    """Per-scene source-index -> target-index map. Also returns names that hit the default."""
    index_map: dict[int, int] = {}
    defaulted: set[str] = set()
    for idx, name in enumerate(scene_names):
        target = mapping.get(name)
        if target is None:
            target = default
            defaulted.add(name)
        if target == DROP:
            continue
        index_map[idx] = targets.index(target)
    return index_map, defaulted


def convert_scene(
    scene: Path,
    index_map: dict[int, int],
    targets: list[str],
    counts: Counter,
    args: argparse.Namespace,
    out: tuple[Path, Path],
) -> tuple[int, int, int]:
    """Convert one scene. Returns (frames kept, images missing, frames rejected by review)."""
    out_images, out_labels = out
    labels_dir, images_dir = scene / "labels", scene / "images"
    if not labels_dir.is_dir():
        return 0, 0, 0

    keep = passing_frames(scene) if args.require_pass else None

    frames = missing = rejected = 0
    for label in sorted(labels_dir.glob("*.txt")):
        if keep is not None and label.stem not in keep:
            rejected += 1
            continue
        image = find_image(images_dir, label.stem)
        if image is None:
            missing += 1
            continue
        rows = convert_rows(label.read_text(), index_map, args.to_bbox, args.min_box_side)
        if args.drop_empty and not rows:
            rejected += 1
            continue
        frames += 1
        for row in rows:
            counts[targets[int(row.split()[0])]] += 1
        if args.dry_run:
            continue
        prefix = f"{scene.name}__"
        (out_labels / f"{prefix}{label.stem}.txt").write_text(
            "\n".join(rows) + ("\n" if rows else "")
        )
        shutil.copy2(image, out_images / f"{prefix}{image.name}")
    return frames, missing, rejected


def build_arg_parser() -> argparse.ArgumentParser:
    """Construct the command line argument parser."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("--src", type=Path, required=True, help="Root of per-scene datasets")
    parser.add_argument("--config", type=Path, required=True, help="TOML name-mapping config")
    parser.add_argument("--out", type=Path, required=True, help="Output dataset dir")
    parser.add_argument(
        "--to-bbox", action="store_true", help="Convert polygons to axis-aligned boxes in-pass"
    )
    parser.add_argument(
        "--require-pass",
        action="store_true",
        help="Keep only frames marked `pass` in each scene's validation_state.json. The archived "
        "scenes carry a human review verdict; 1,680 frames (4.7%%) are marked fail.",
    )
    parser.add_argument(
        "--min-box-side",
        type=float,
        default=0.0,
        help="Drop boxes whose smaller side is below this fraction of the image (e.g. 0.0125 = "
        "8px at 640). Suppresses slivers from shattered segmentation masks.",
    )
    parser.add_argument(
        "--drop-empty",
        action="store_true",
        help="Drop frames left with no annotations (valid background negatives, but "
        "unreviewable by eye)",
    )
    parser.add_argument("--dry-run", action="store_true", help="Report only, write nothing")
    return parser


def main() -> None:
    """Merge per-scene datasets into one, remapping classes by name."""
    args = build_arg_parser().parse_args()
    targets, mapping, default = load_config(args.config)
    scenes = scene_dirs(args.src)
    if not scenes:
        raise SystemExit(f"no per-scene datasets under {args.src}")
    print(f"{len(scenes)} scenes -> {targets} (default: {default!r})")

    counts: Counter = Counter()
    defaulted: set[str] = set()
    frames = missing = rejected = 0

    out_images = args.out / "train" / "images"
    out_labels = args.out / "train" / "labels"
    if not args.dry_run:
        out_images.mkdir(parents=True, exist_ok=True)
        out_labels.mkdir(parents=True, exist_ok=True)

    for scene in scenes:
        scene_names = list(yaml.safe_load((scene / "data.yml").read_text())["names"])
        index_map, hit_default = build_index_map(scene_names, mapping, default, targets)
        defaulted |= hit_default
        kept, skipped, refused = convert_scene(
            scene, index_map, targets, counts, args, (out_images, out_labels)
        )
        frames += kept
        missing += skipped
        rejected += refused

    print(
        f"\nframes: {frames}"
        + (f"  ({missing} labels with no image, skipped)" if missing else "")
        + (f"  ({rejected} rejected by validation_state.json)" if rejected else "")
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

    data = {
        "path": str(args.out.resolve()),
        "train": "train/images",
        "nc": len(targets),
        "names": targets,
    }
    (args.out / "data.yml").write_text(yaml.safe_dump(data, sort_keys=False))
    print(f"\nwrote {args.out} (images copied, no links)")


if __name__ == "__main__":
    main()
