"""Build the domain-mix arms: randomized and cage-domain synthetic at set counts, as image lists.

Step 4 of docs/experiments/perception_performance/synthetic_domain_mix_plan_2026-09-12.md. Every
arm is the corpus's 452 real frames, a prefix of the randomized pool, and a prefix of the domain
render, written as an image-list ``.txt`` plus a ``.yml`` that pairs it with the corpus's own val
split. Like ``make_scaling_splits.py`` the arms cost kilobytes and cannot drift from the source
datasets, but that script splits one dataset by scene; this one draws from several.

Two properties, both asserted before anything is written:

* **Arms nest.** Each source is drawn in one fixed order, so the 10k domain arm is a prefix of the
  20k one and a drop in accuracy cannot be blamed on which frames were picked.
* **The domain order is balanced at every prefix.** Each (venue, view) group is shuffled on its own,
  the views are interleaved within a venue and the venues interleaved after that, so any prefix is
  split evenly between the venues to within a frame and a third per view to within a frame per
  venue, which is how the render lands. Filters (one venue, one view, damage-free frames only)
  re-derive the order over what they keep.

Usage:
  venv/bin/python training/yolo/make_domain_mix_arms.py \\
      --corpus training/data/all_robot_keypoints \\
      --domain training/data/synth_cage_nhrl_2026-09-13 training/data/synth_cage_massd_2026-09-13 \\
      --out training/data/domain_mix_arms_2026-09-13
"""

from __future__ import annotations

import argparse
import json
import random
from collections import Counter
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, replace
from pathlib import Path

import yaml

NAMES = ["mr_stabs_mk2", "mrs_buff_mk3", "nhrl_robot", "house_bot"]
RANDOMIZED_PREFIX = "synthetic__"
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png"}
NHRL = "nhrl_cage"
MASSD = "massd_arena"


@dataclass(frozen=True)
class DomainFrame:
    """One rendered frame and the manifest fields the arms filter on."""

    image: Path
    venue: str
    view: str
    clean: bool


@dataclass(frozen=True)
class Arm:
    """One arm: how many randomized frames (None for all) and domain frames, and its filters."""

    name: str
    randomized: int | None
    domain: int
    venues: tuple[str, ...] = ()
    views: tuple[str, ...] = ()
    clean_only: bool = False


# The step 4 grid. `nodamage` is "best mix" in the plan, which is not known until the Q1 to Q3 arms
# are scored, so both candidates are built and the one matching the better mix gets trained.
ARMS = (
    Arm("base", None, 0),
    Arm("d2500", None, 2500),
    Arm("d5000", None, 5000),
    Arm("d10000", None, 10000),
    Arm("d20000", None, 20000),
    Arm("d40000", None, 40000),
    Arm("swap_half", 10000, 10000),
    Arm("swap_all", 0, 20000),
    Arm("nhrl_only", 0, 20000, venues=(NHRL,)),
    Arm("massd_only", 0, 20000, venues=(MASSD,)),
    Arm("nodamage_d20000", None, 20000, clean_only=True),
    Arm("nodamage_swap_all", 0, 20000, clean_only=True),
    Arm("view_pinhole", 0, 13333, views=("pinhole",)),
    Arm("view_rectified", 0, 13333, views=("rectified",)),
    Arm("view_distorted", 0, 13333, views=("distorted",)),
    Arm("view_mixed", 0, 13333),
)
# (smaller, larger) pairs whose training lists must nest.
NESTED = (
    ("base", "d2500"),
    ("d2500", "d5000"),
    ("d5000", "d10000"),
    ("d10000", "d20000"),
    ("d20000", "d40000"),
    ("view_mixed", "swap_all"),
    ("swap_all", "d20000"),
)


def selected_arms(only: Sequence[str] | None, scale: float) -> list[Arm]:
    """The grid, or the named subset of it, with every set count multiplied by *scale*.

    Raises:
        SystemExit: When *only* names an arm the grid does not have.
    """
    names = {arm.name for arm in ARMS}
    unknown = sorted(set(only or ()) - names)
    if unknown:
        raise SystemExit(f"unknown arms {unknown}; the grid has {sorted(names)}")
    return [
        replace(
            arm,
            randomized=None if arm.randomized is None else round(arm.randomized * scale),
            domain=round(arm.domain * scale),
        )
        for arm in ARMS
        if not only or arm.name in only
    ]


def load_domain(dataset: Path) -> list[DomainFrame]:
    """Every frame in a merged render, from its manifest.

    Raises:
        SystemExit: When a row has no view; the view arms cannot be built from such a render.
    """
    frames = []
    for line in (dataset / "manifest.jsonl").read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        if "view" not in row:
            raise SystemExit(f"{dataset}: manifest row for {row['image']} has no view")
        clean = all(float(i.get("damage", 0.0)) == 0.0 for i in row.get("instances", []))
        image = (dataset / "images" / row["image"]).resolve()
        frames.append(DomainFrame(image, str(row["venue"]), str(row["view"]), clean))
    return frames


def interleave(groups: Sequence[Sequence[Path]]) -> list[Path]:
    """Round-robin over *groups* until every one is exhausted."""
    out: list[Path] = []
    for index in range(max((len(group) for group in groups), default=0)):
        out.extend(group[index] for group in groups if index < len(group))
    return out


def domain_order(
    frames: Iterable[DomainFrame],
    seed: int,
    venues: Sequence[str] = (),
    views: Sequence[str] = (),
    clean_only: bool = False,
) -> list[Path]:
    """The fixed draw order over the frames the filters keep, balanced at every prefix."""
    kept = [
        frame
        for frame in frames
        if (not venues or frame.venue in venues)
        and (not views or frame.view in views)
        and (frame.clean or not clean_only)
    ]
    per_venue = []
    for venue in sorted({frame.venue for frame in kept}):
        per_view = []
        for view in sorted({frame.view for frame in kept if frame.venue == venue}):
            group = sorted(f.image for f in kept if f.venue == venue and f.view == view)
            random.Random(f"{seed}:{venue}:{view}").shuffle(group)
            per_view.append(group)
        per_venue.append(interleave(per_view))
    return interleave(per_venue)


def split_corpus(corpus: Path, seed: int) -> tuple[list[Path], list[Path], list[Path]]:
    """(real train frames, randomized train frames in draw order, val frames) of the corpus."""
    train = sorted(
        path.resolve()
        for path in (corpus / "train" / "images").iterdir()
        if path.suffix.lower() in IMAGE_SUFFIXES
    )
    real = [path for path in train if not path.name.startswith(RANDOMIZED_PREFIX)]
    randomized = [path for path in train if path.name.startswith(RANDOMIZED_PREFIX)]
    random.Random(f"{seed}:randomized").shuffle(randomized)
    val = sorted(
        path.resolve()
        for path in (corpus / "val" / "images").iterdir()
        if path.suffix.lower() in IMAGE_SUFFIXES
    )
    return real, randomized, val


def build_arm(
    arm: Arm,
    real: list[Path],
    randomized: list[Path],
    domain: list[DomainFrame],
    seed: int,
) -> tuple[list[Path], dict]:
    """One arm's training frames and its row for manifest.json.

    Raises:
        SystemExit: When a source holds fewer frames than the arm asks for.
    """
    count = len(randomized) if arm.randomized is None else arm.randomized
    if count > len(randomized):
        raise SystemExit(f"{arm.name}: wants {count} randomized frames, pool has {len(randomized)}")
    order = domain_order(domain, seed, arm.venues, arm.views, arm.clean_only)
    if arm.domain > len(order):
        raise SystemExit(f"{arm.name}: wants {arm.domain} domain frames, filter keeps {len(order)}")
    drawn = set(order[: arm.domain])
    picked = [frame for frame in domain if frame.image in drawn]
    frames = real + randomized[:count] + [frame.image for frame in picked]
    row = {
        "frames": len(frames),
        "real": len(real),
        "randomized": count,
        "domain": arm.domain,
        "domain_by_venue": dict(sorted(Counter(frame.venue for frame in picked).items())),
        "domain_by_view": dict(sorted(Counter(frame.view for frame in picked).items())),
        "domain_clean": sum(1 for frame in picked if frame.clean),
        "filters": {
            "venues": list(arm.venues),
            "views": list(arm.views),
            "clean_only": arm.clean_only,
        },
    }
    return frames, row


def write_arm(out: Path, name: str, frames: list[Path], val_list: Path) -> None:
    """The arm's image list, and a data.yml pointing train at it and val at the shared list."""
    listing = out / f"{name}.txt"
    listing.write_text("".join(f"{path}\n" for path in sorted(frames)), encoding="utf-8")
    meta = {
        "path": str(out.resolve()),
        "train": listing.name,
        "val": val_list.name,
        "nc": len(NAMES),
        "names": NAMES,
        "kpt_shape": [2, 3],
        "flip_idx": [0, 1],
    }
    (out / f"{name}.yml").write_text(yaml.safe_dump(meta, sort_keys=False), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("--corpus", type=Path, required=True, help="all_robot_keypoints root")
    parser.add_argument(
        "--domain",
        type=Path,
        nargs="*",
        default=[],
        help="Merged renders; none builds only arms with no domain frames, such as base",
    )
    parser.add_argument("--out", type=Path, required=True, help="Directory for lists and yamls")
    parser.add_argument("--seed", type=int, default=0, help="Seed for every draw order")
    parser.add_argument("--only", nargs="+", default=None, help="Build just these arms")
    parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="Multiply every set frame count, to check the builder against a small smoke render",
    )
    args = parser.parse_args()

    real, randomized, val = split_corpus(args.corpus, args.seed)
    domain = [frame for dataset in args.domain for frame in load_domain(dataset)]
    print(f"corpus: {len(real)} real, {len(randomized)} randomized, {len(val)} val")
    print(f"domain: {len(domain)} frames, {dict(Counter((f.venue, f.view) for f in domain))}")

    lists: dict[str, set[Path]] = {}
    rows: dict[str, dict] = {}
    built: list[tuple[str, list[Path]]] = []
    for arm in selected_arms(args.only, args.scale):
        frames, row = build_arm(arm, real, randomized, domain, args.seed)
        if len(set(frames)) != len(frames):
            raise SystemExit(f"{arm.name}: a frame appears twice")
        lists[arm.name] = set(frames)
        rows[arm.name] = row
        built.append((arm.name, frames))
    for smaller, larger in NESTED:
        if smaller in lists and larger in lists and not lists[smaller] <= lists[larger]:
            raise SystemExit(f"nesting violated: {smaller} is not a subset of {larger}")
    if {"swap_half", "d10000"} <= lists.keys() and not lists["swap_half"] <= lists["d10000"]:
        raise SystemExit("nesting violated: swap_half is not a subset of d10000")
    if set(val) & set().union(*lists.values()):
        raise SystemExit("val overlaps a training arm")

    args.out.mkdir(parents=True, exist_ok=True)
    val_list = args.out / "val.txt"
    val_list.write_text("".join(f"{path}\n" for path in val), encoding="utf-8")
    print(f"\n{'arm':20s} {'frames':>7s} {'rand':>6s} {'domain':>7s}  venues / views / clean")
    for name, frames in built:
        write_arm(args.out, name, frames, val_list)
        row = rows[name]
        print(
            f"{name:20s} {row['frames']:7d} {row['randomized']:6d} {row['domain']:7d}  "
            f"{row['domain_by_venue']} {row['domain_by_view']} clean {row['domain_clean']}"
        )
    manifest = {
        "corpus": str(args.corpus.resolve()),
        "domain": [str(path.resolve()) for path in args.domain],
        "seed": args.seed,
        "val_frames": len(val),
        "arms": rows,
    }
    (args.out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"\nnesting verified; wrote {args.out}/{{val.txt, <arm>.txt, <arm>.yml, manifest.json}}")


if __name__ == "__main__":
    main()
