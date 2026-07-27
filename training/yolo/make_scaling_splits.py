"""Build the data-scaling arms: one shared val set, plus nested scene- and frame-sampled subsets.

Answers "how much labelled data do I need, and does it matter whether it comes from more scenes or
more frames" -- annotation cost is per frame, acquisition cost is per fight.

Three properties this enforces, each of which a naive split gets wrong:

* **The val set is scene-disjoint and shared.** Consecutive frames of a recording are
  near-duplicates, so a frame-level val split measures memorisation. One val set is carved once and
  used by every arm, so the val curves are directly comparable.
* **Scene- and frame-sampled arms are matched on frame count.** Scene sizes here run 9 to 1,363
  frames, so "50 % of scenes" could be anywhere from 7.9 k to 23.5 k frames. Matching frame count
  and
  varying only how many scenes it is drawn from is what isolates the value of scene diversity.
* **Arms are nested.** ``scene50 ⊂ scene75 ⊂ base100`` and likewise for the random arms, built as
  prefixes of one fixed shuffle. Each step only *adds* data, so a drop in accuracy cannot be blamed
  on which scenes happened to be drawn.

Output is **image-list .txt files**, not copied or linked frames. Ultralytics accepts a ``.txt`` of
image paths for ``train``/``val``, so the arms cost kilobytes, share one ``cache="disk"`` ``.npy``
set
next to the original images, and cannot drift out of sync with the source dataset.

Usage:
  python training/yolo/make_scaling_splits.py \
      --src training/data/nhrl_robots_bbox_2class \
      --out training/data/datascale_2026-07-27
"""

from __future__ import annotations

import argparse
import json
import random
import re
from collections import Counter, defaultdict
from pathlib import Path

import yaml

SCENE_RE = re.compile(r"^(?P<scene>.*)_yolo_seg__frame_\d+$")
FRACTIONS = (0.75, 0.50)


def scene_of(stem: str) -> str:
    """Recording key for a frame stem."""
    match = SCENE_RE.match(stem)
    return match.group("scene") if match else stem


def index_dataset(src: Path) -> tuple[dict[str, list[Path]], dict[str, Counter]]:
    """Group image paths by scene, and count classes per scene."""
    images = defaultdict(list)
    classes: dict[str, Counter] = defaultdict(Counter)
    for image in sorted((src / "train" / "images").glob("*.jpg")):
        label = src / "train" / "labels" / f"{image.stem}.txt"
        if not label.is_file():
            continue
        scene = scene_of(image.stem)
        images[scene].append(image)
        for line in label.read_text().splitlines():
            if line.strip():
                classes[scene][int(line.split()[0])] += 1
    return dict(images), dict(classes)


def choose_val_scenes(
    images: dict[str, list[Path]],
    classes: dict[str, Counter],
    frac: float,
    count: int,
    seed: int,
    trials: int = 4000,
) -> list[str]:
    """Pick ``count`` scenes whose frame share is near ``frac`` and whose house_bot share matches.

    Searched rather than taken in order because scene sizes are so uneven that any fixed rule lands
    far off the target; a random search over whole-scene combinations gets within a fraction of a
    percent on both axes.
    """
    names = sorted(images)
    total_frames = sum(len(v) for v in images.values())
    total_hb = sum(classes[s][1] for s in names) or 1
    rng = random.Random(seed)

    best: tuple[float, list[str]] | None = None
    for _ in range(trials):
        pick = rng.sample(names, count)
        frame_share = sum(len(images[s]) for s in pick) / total_frames
        hb_share = sum(classes[s][1] for s in pick) / total_hb
        score = abs(frame_share - frac) / frac + abs(hb_share - frac) / frac
        if best is None or score < best[0]:
            best = (score, sorted(pick))
    assert best is not None
    return best[1]


def nested_scene_arms(
    pool_scenes: list[str], images: dict[str, list[Path]], seed: int
) -> dict[str, list[Path]]:
    """Nested scene-sampled arms: whole scenes added until each frame target is met."""
    order = list(pool_scenes)
    random.Random(seed).shuffle(order)
    total = sum(len(images[s]) for s in pool_scenes)

    arms: dict[str, list[Path]] = {}
    for frac in FRACTIONS:
        target = int(total * frac)
        taken: list[Path] = []
        for scene in order:
            if len(taken) >= target:
                break
            taken.extend(images[scene])
        arms[f"scene{int(frac * 100)}"] = taken
    return arms


def nested_random_arms(pool_frames: list[Path], seed: int) -> dict[str, list[Path]]:
    """Nested frame-sampled arms: prefixes of one shuffle, so each is a subset of the next."""
    shuffled = list(pool_frames)
    random.Random(seed).shuffle(shuffled)
    return {f"rand{int(frac * 100)}": shuffled[: int(len(shuffled) * frac)] for frac in FRACTIONS}


def write_arm(out: Path, name: str, frames: list[Path], names: list[str], val_list: Path) -> None:
    """Write an arm's image list and a data.yml pointing train at it and val at the shared set."""
    listing = out / f"{name}.txt"
    listing.write_text("\n".join(str(p.resolve()) for p in sorted(frames)) + "\n")
    (out / f"{name}.yml").write_text(
        yaml.safe_dump(
            {
                "path": str(out.resolve()),
                "train": listing.name,
                "val": val_list.name,
                "nc": len(names),
                "names": names,
            },
            sort_keys=False,
        )
    )


def summarize(name: str, frames: list[Path], classes: dict[str, Counter]) -> dict:
    """Per-arm stats for the manifest and the console table."""
    scenes = sorted({scene_of(p.stem) for p in frames})
    # class counts are per-scene, so only exact for whole-scene arms; report frames either way
    return {"frames": len(frames), "scenes": len(scenes), "scene_list": scenes}


def build_arg_parser() -> argparse.ArgumentParser:
    """Construct the command line argument parser."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("--src", type=Path, required=True, help="Validated 2-class dataset root")
    parser.add_argument("--out", type=Path, required=True, help="Output dir for lists and yamls")
    parser.add_argument("--val-frac", type=float, default=0.15, help="Val share of frames")
    parser.add_argument("--val-scenes", type=int, default=9, help="Number of whole scenes for val")
    parser.add_argument("--seed", type=int, default=0, help="Seed for val search and arm shuffles")
    return parser


def main() -> None:
    """Build the shared val set and the nested scaling arms."""
    args = build_arg_parser().parse_args()
    meta = yaml.safe_load((args.src / "data.yml").read_text())
    names: list[str] = list(meta["names"])

    images, classes = index_dataset(args.src)
    total = sum(len(v) for v in images.values())
    print(f"{len(images)} scenes, {total} frames, classes {names}")

    val_scenes = choose_val_scenes(images, classes, args.val_frac, args.val_scenes, args.seed)
    pool_scenes = [s for s in sorted(images) if s not in val_scenes]
    val_frames = [p for s in val_scenes for p in images[s]]
    pool_frames = [p for s in pool_scenes for p in images[s]]

    args.out.mkdir(parents=True, exist_ok=True)
    val_list = args.out / "val.txt"
    val_list.write_text("\n".join(str(p.resolve()) for p in sorted(val_frames)) + "\n")

    arms: dict[str, list[Path]] = {"base100": pool_frames}
    arms.update(nested_scene_arms(pool_scenes, images, args.seed))
    arms.update(nested_random_arms(pool_frames, args.seed))

    manifest = {
        "source": str(args.src.resolve()),
        "seed": args.seed,
        "val": {
            "frames": len(val_frames),
            "scenes": len(val_scenes),
            "scene_list": val_scenes,
        },
        "arms": {},
    }

    print(
        f"\nval: {len(val_scenes)} scenes, {len(val_frames)} frames "
        f"({len(val_frames) / total * 100:.1f} % of corpus) -- shared by every arm"
    )
    print(f"pool: {len(pool_scenes)} scenes, {len(pool_frames)} frames\n")
    print(f"{'arm':10s} {'frames':>7s} {'scenes':>7s}  {'% of pool':>9s}")
    print("-" * 40)
    for name in ("base100", "scene75", "scene50", "rand75", "rand50"):
        frames = arms[name]
        write_arm(args.out, name, frames, names, val_list)
        stats = summarize(name, frames, classes)
        manifest["arms"][name] = stats
        print(
            f"{name:10s} {stats['frames']:7d} {stats['scenes']:7d}  "
            f"{len(frames) / len(pool_frames) * 100:8.1f} %"
        )

    # nesting is a correctness property, not an intention -- assert it
    for smaller, larger in (
        ("scene50", "scene75"),
        ("scene75", "base100"),
        ("rand50", "rand75"),
        ("rand75", "base100"),
    ):
        if not set(arms[smaller]) <= set(arms[larger]):
            raise SystemExit(f"nesting violated: {smaller} is not a subset of {larger}")
    if set(val_frames) & set(pool_frames):
        raise SystemExit("val overlaps the training pool")
    print("\nnesting verified; val is disjoint from every arm")

    (args.out / "manifest.json").write_text(json.dumps(manifest, indent=2))
    print(f"wrote {args.out}/{{val.txt, <arm>.txt, <arm>.yml, manifest.json}}")


if __name__ == "__main__":
    main()
