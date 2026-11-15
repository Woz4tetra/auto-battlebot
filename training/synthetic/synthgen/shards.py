"""How a sharded render splits its frames over GPUs and views, and how the runs merge back."""

from __future__ import annotations

import json
import os
from collections import Counter
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path

from synthgen.constants import VIEWS

IMAGE_SUFFIX = ".jpg"
LABEL_SUFFIX = ".txt"
MANIFEST_NAME = "manifest.jsonl"
DATA_YML_NAME = "data.yml"
_RUN_SPECIFIC_YML_KEY = "path:"
VIEW_FOR_ROWS_BEFORE_VIEW_FIELD = "unrecorded"


@dataclass(frozen=True)
class ShardRun:
    """One ``render_scenes.py`` invocation: its shard, view, frame range and seed."""

    shard: int
    view: str
    start_index: int
    num_images: int
    seed: int

    @property
    def name(self) -> str:
        """The run's directory name inside the parts directory."""
        return f"shard{self.shard}_{self.view}"

    @property
    def end_index(self) -> int:
        """One past the last frame index this run writes."""
        return self.start_index + self.num_images


@dataclass(frozen=True)
class MergeSummary:
    """What a merge wrote."""

    images: int
    per_view: dict[str, int] = field(default_factory=dict)
    per_venue: dict[str, int] = field(default_factory=dict)


def split_evenly(total: int, parts: int, offset: int = 0) -> list[int]:
    """total over parts as evenly as integers allow, remainder from offset on."""
    if parts <= 0:
        raise ValueError(f"parts must be positive, got {parts}")
    if total < 0:
        raise ValueError(f"total must be non-negative, got {total}")
    base, extra = divmod(total, parts)
    counts = [base] * parts
    for k in range(extra):
        counts[(offset + k) % parts] += 1
    return counts


def plan_shard_runs(
    total: int, shards: int, views: Sequence[str], seed_base: int
) -> list[ShardRun]:
    """Every run of a sharded render, in (shard, view) order."""
    views = tuple(views)
    unknown = [view for view in views if view not in VIEWS]
    if unknown:
        raise ValueError(f"unknown views {unknown}; valid are {list(VIEWS)}")
    if not views or len(set(views)) != len(views):
        raise ValueError(f"views must be distinct and non-empty, got {list(views)}")
    per_view = split_evenly(total, len(views))
    counts = [split_evenly(count, shards, offset=v) for v, count in enumerate(per_view)]
    runs: list[ShardRun] = []
    start = 0
    for shard in range(shards):
        for v, view in enumerate(views):
            count = counts[v][shard]
            runs.append(ShardRun(shard, view, start, count, seed_base + shard * len(views) + v))
            start += count
    return runs


def existing_frames(image_dir: Path, run: ShardRun) -> list[int]:
    """Frame indices inside *run*'s range already written to *image_dir*, sorted."""
    if not image_dir.is_dir():
        return []
    return sorted(
        int(path.stem)
        for path in image_dir.glob(f"*{IMAGE_SUFFIX}")
        if path.stem.isdigit() and run.start_index <= int(path.stem) < run.end_index
    )


def resume_point(frames: Sequence[int], run: ShardRun) -> tuple[int, int]:
    """``(start_index, num_images)`` still to render: one past the highest frame on disk."""
    start = max(frames) + 1 if frames else run.start_index
    return start, run.end_index - start


def merge_parts(parts: Sequence[Path], out: Path) -> MergeSummary:
    """Hardlink every part's frames into one flat *out* and concatenate their manifests."""
    out_images, out_labels = out / "images", out / "labels"
    for existing in (out_images, out_labels, out / MANIFEST_NAME):
        if existing.exists():
            raise ValueError(f"{existing} already exists; merge into a fresh directory")

    seen: dict[str, Path] = {}
    contents = [_collect_part(part, out_images, out_labels, seen) for part in parts]
    first_body = contents[0].data_yml_body if contents else None
    for part, content in zip(parts, contents):
        if content.data_yml_body != first_body:
            raise ValueError(f"{part}: data.yml differs from the first part's beyond its path")

    out_images.mkdir(parents=True)
    out_labels.mkdir(parents=True)
    lines: list[str] = []
    per_view: Counter[str] = Counter()
    per_venue: Counter[str] = Counter()
    for content in contents:
        for source, target in content.links:
            os.link(source, target)
        for line, row in content.rows:
            lines.append(line)
            per_view[str(row.get("view", VIEW_FOR_ROWS_BEFORE_VIEW_FIELD))] += 1
            per_venue[str(row.get("venue"))] += 1
    (out / MANIFEST_NAME).write_text("".join(f"{line}\n" for line in lines), encoding="utf-8")
    if first_body is not None:
        yml = [f"{_RUN_SPECIFIC_YML_KEY} {out.resolve()}", *first_body]
        (out / DATA_YML_NAME).write_text("\n".join(yml) + "\n", encoding="utf-8")
    return MergeSummary(len(seen), dict(sorted(per_view.items())), dict(sorted(per_venue.items())))


@dataclass(frozen=True)
class _PartContents:
    """One checked part: the links it contributes, its manifest rows, and its data.yml body."""

    links: list[tuple[Path, Path]]
    rows: list[tuple[str, dict]]
    data_yml_body: list[str]


def _collect_part(
    part: Path, out_images: Path, out_labels: Path, seen: dict[str, Path]
) -> _PartContents:
    """Check one part and list what it adds to the merge."""
    images = sorted(
        path for path in (part / "images").glob(f"*{IMAGE_SUFFIX}") if path.stem.isdigit()
    )
    rows = _read_manifest(part / MANIFEST_NAME)
    _check_manifest_rows(part, {image.name for image in images}, rows)
    links: list[tuple[Path, Path]] = []
    for image in images:
        label = part / "labels" / f"{image.stem}{LABEL_SUFFIX}"
        if not label.is_file():
            raise ValueError(f"{part}: {image.name} has no label")
        if image.name in seen:
            raise ValueError(f"{image.name} is in both {seen[image.name]} and {part}")
        seen[image.name] = part
        links.append((image, out_images / image.name))
        links.append((label, out_labels / label.name))
    return _PartContents(links, rows, _data_yml_body(part / DATA_YML_NAME))


def _check_manifest_rows(part: Path, image_names: set[str], rows: list[tuple[str, dict]]) -> None:
    """Every image has exactly one manifest row, and every row has an image."""
    by_image = Counter(row["image"] for _, row in rows)
    if set(by_image) == image_names and all(count == 1 for count in by_image.values()):
        return
    missing = sorted(image_names - set(by_image))[:5]
    orphan = sorted(set(by_image) - image_names)[:5]
    repeated = sorted(name for name, count in by_image.items() if count > 1)[:5]
    raise ValueError(
        f"{part}: manifest does not match images (no row for {missing}, rows without an image"
        f" {orphan}, repeated rows {repeated})"
    )


def _read_manifest(path: Path) -> list[tuple[str, dict]]:
    """Each non-empty manifest line with its parsed row."""
    if not path.is_file():
        return []
    lines = [line for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    return [(line, json.loads(line)) for line in lines]


def _data_yml_body(path: Path) -> list[str]:
    """A data.yml's lines without its path line."""
    if not path.is_file():
        raise ValueError(f"{path} is missing")
    lines = path.read_text(encoding="utf-8").splitlines()
    return [line for line in lines if not line.startswith(_RUN_SPECIFIC_YML_KEY)]
