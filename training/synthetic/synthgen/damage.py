"""Battle damage as a per-instance random variable: what to remove, decided without Blender."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import NamedTuple

import numpy as np

MECHANISM_NONE = "none"
MECHANISM_PARTS = "parts"
MECHANISM_CHUNK = "chunk"
MECHANISM_NAMED = "named"

CUTTER_CUBE = "cube"
CUTTER_ICOSPHERE = "icosphere"
CUTTER_SHAPES = (CUTTER_CUBE, CUTTER_ICOSPHERE)

CUTTER_SEED_RETRIES = 12
SURFACE_SAMPLE_LIMIT = 64


class NamedPartRemoval(NamedTuple):
    part_name: str
    piece_indices: tuple[int, ...]


class Aabb(NamedTuple):
    x_min: float
    y_min: float
    z_min: float
    x_max: float
    y_max: float
    z_max: float


@dataclass(frozen=True)
class InstanceDamage:
    """What one robot instance ended up with, recorded per frame in ``manifest.jsonl``."""

    class_name: str
    damage: float
    mechanism: str
    parts: tuple[NamedPartRemoval, ...] = ()

    @property
    def is_damaged(self) -> bool:
        """Whether this instance carries any damage at all."""
        return self.mechanism != MECHANISM_NONE and self.damage > 0.0

    def as_row(self) -> dict[str, object]:
        """The manifest form: compact, and stable across mechanisms."""
        row: dict[str, object] = {
            "class": self.class_name,
            "damage": round(float(self.damage), 4),
            "mechanism": self.mechanism,
        }
        if self.parts:
            row["parts"] = {name: list(indices) for name, indices in self.parts}
        return row


@dataclass
class DamageBudget:
    """Tracks the damaged/clean scene split so a short run still lands on the ratio."""

    scenes: int = 0
    damaged: int = 0

    def take(self, scene_probability: float) -> bool:
        """Whether the next scene is a damaged one, and record the decision."""
        wanted = scene_probability * (self.scenes + 1)
        decision = self.damaged + 0.5 < wanted
        self.scenes += 1
        self.damaged += int(decision)
        return decision


@dataclass(frozen=True)
class PartDraw:
    """Which mesh parts to hide, and what fraction of the robot's parts that is."""

    part_indices: tuple[int, ...]
    damage: float


@dataclass(frozen=True)
class ChunkDraw:
    """A cutter to subtract: shape, where it sits in the parent frame, and how big."""

    shape: str
    centre: tuple[float, float, float]
    radius_m: float
    volume_fraction: float


@dataclass(frozen=True)
class RemovablePart:
    """One named assembly a robot can lose, as the draw sees it."""

    name: str
    split_piece_count: int
    subset: bool = False
    includes: tuple[str, ...] = ()
    count_weights: tuple[float, ...] = ()
    selectable: bool = True


@dataclass(frozen=True)
class NamedPartDraw:
    """Which pieces of which named parts to hide, and what fraction of all pieces that is."""

    removed: tuple[NamedPartRemoval, ...]
    damage: float


@dataclass(frozen=True)
class FacePiece:
    """Where one piece of a named part is, for assigning mesh faces to it at load."""

    boxes: tuple[Aabb, ...]
    objects: tuple[str, ...] = ()
    exclude_objects: tuple[str, ...] = ()


def model_box_to_blender_local(box: Aabb) -> Aabb:
    """A model-frame (glTF, Y-up) box in Blender local axes, the conversion keypoints get."""
    x0, y0, z0, x1, y1, z1 = box
    return Aabb(x0, -z1, y0, x1, -z0, y1)


def base_object_name(name: str) -> str:
    """*name* without Blender's ``.001`` duplicate suffix."""
    head, dot, tail = name.rpartition(".")
    return head if dot and tail.isdigit() else name


def object_name_matches(name: str, pattern: str) -> bool:
    """Whether Blender object *name* is *pattern*, or *pattern* plus a ``_N`` index."""
    base = base_object_name(name)
    if base == pattern:
        return True
    head, underscore, tail = base.rpartition("_")
    return bool(underscore) and head == pattern and tail.isdigit()


def assign_faces_to_pieces(
    centres: np.ndarray, object_name: str, pieces: list[FacePiece]
) -> np.ndarray:
    """Per face, the index of the first piece whose boxes hold its centre, or -1."""
    pts = np.asarray(centres, dtype=np.float64).reshape(-1, 3)
    assign = np.full(len(pts), -1, dtype=np.int64)
    for index, piece in enumerate(pieces):
        if piece.objects and not any(object_name_matches(object_name, p) for p in piece.objects):
            continue
        if any(object_name_matches(object_name, p) for p in piece.exclude_objects):
            continue
        inside = np.zeros(len(pts), dtype=bool)
        for x0, y0, z0, x1, y1, z1 in piece.boxes:
            inside |= np.all((pts >= (x0, y0, z0)) & (pts <= (x1, y1, z1)), axis=1)
        assign[inside & (assign == -1)] = index
    return assign


def piece_count(part: RemovablePart) -> int:
    """How many of a subset part's pieces one draw removes, from ``count_weights``."""
    counts = list(range(1, part.split_piece_count + 1))
    if not part.count_weights:
        return random.choice(counts)
    weights = [
        part.count_weights[i] if i < len(part.count_weights) else 0.0 for i in range(len(counts))
    ]
    if sum(weights) <= 0.0:
        return random.choice(counts)
    return random.choices(counts, weights=weights, k=1)[0]


def draw_named_part_damage(
    parts: list[RemovablePart],
    allowed: tuple[str, ...],
    count_range: tuple[int, int],
) -> NamedPartDraw | None:
    """Pick 1..n of the allowed named parts to remove, or None when none is allowed."""
    by_name = {part.name: part for part in parts if part.split_piece_count > 0}
    candidates = [
        name for name in by_name if by_name[name].selectable and (not allowed or name in allowed)
    ]
    if not candidates:
        return None
    high = max(1, min(count_range[1], len(candidates)))
    low = max(1, min(count_range[0], high))
    removed: dict[str, set[int]] = {}
    for name in random.sample(candidates, random.randint(low, high)):
        part = by_name[name]
        if part.subset:
            chosen = random.sample(range(part.split_piece_count), piece_count(part))
            removed.setdefault(name, set()).update(chosen)
        else:
            removed.setdefault(name, set()).update(range(part.split_piece_count))
        for included in part.includes:
            if included in by_name:
                removed.setdefault(included, set()).update(
                    range(by_name[included].split_piece_count)
                )
    total = sum(part.split_piece_count for part in by_name.values())
    return NamedPartDraw(
        removed=tuple(
            NamedPartRemoval(name, tuple(sorted(indices))) for name, indices in removed.items()
        ),
        damage=sum(len(indices) for indices in removed.values()) / total,
    )


def bbox_corner_volume(corners: np.ndarray) -> float:
    """Axis-aligned volume spanned by *corners*, an ``(n, 3)`` array of points."""
    pts = np.asarray(corners, dtype=np.float64).reshape(-1, 3)
    if pts.size == 0:
        return 0.0
    span = pts.max(axis=0) - pts.min(axis=0)
    return float(np.prod(np.maximum(span, 0.0)))


def bbox_contains(corners: np.ndarray, point: np.ndarray, margin_m: float) -> bool:
    """Whether *point* falls inside the axis-aligned box of *corners*, grown by *margin_m*."""
    pts = np.asarray(corners, dtype=np.float64).reshape(-1, 3)
    if pts.size == 0:
        return False
    low = pts.min(axis=0) - margin_m
    high = pts.max(axis=0) + margin_m
    p = np.asarray(point, dtype=np.float64).reshape(3)
    return bool(np.all(p >= low) and np.all(p <= high))


def protected_part_indices(
    part_corners: list[np.ndarray],
    part_names: list[str],
    keypoints: list[np.ndarray],
    name_patterns: tuple[str, ...],
    max_volume_fraction: float,
    keypoint_clearance_m: float,
) -> set[int]:
    """Parts that must survive: keypoint anchors, oversized parts, and named patterns."""
    total = sum(bbox_corner_volume(c) for c in part_corners)
    protected: set[int] = set()
    for index, corners in enumerate(part_corners):
        name = part_names[index].lower() if index < len(part_names) else ""
        if any(pattern and pattern in name for pattern in name_patterns):
            protected.add(index)
            continue
        if total > 0.0 and bbox_corner_volume(corners) / total > max_volume_fraction:
            protected.add(index)
            continue
        if any(bbox_contains(corners, kp, keypoint_clearance_m) for kp in keypoints):
            protected.add(index)
    return protected


def draw_part_damage(
    part_corners: list[np.ndarray],
    part_names: list[str],
    keypoints: list[np.ndarray],
    severity_range: tuple[float, float],
    name_patterns: tuple[str, ...],
    max_volume_fraction: float,
    keypoint_clearance_m: float,
) -> PartDraw | None:
    """Draw a subset of removable parts, or None when nothing may be removed."""
    if len(part_corners) < 2:
        return None
    protected = protected_part_indices(
        part_corners,
        part_names,
        keypoints,
        name_patterns,
        max_volume_fraction,
        keypoint_clearance_m,
    )
    removable = [i for i in range(len(part_corners)) if i not in protected]
    if not removable:
        return None
    severity = random.uniform(*severity_range)
    count = max(1, min(len(removable), int(round(severity * len(removable)))))
    chosen = tuple(sorted(random.sample(removable, count)))
    return PartDraw(part_indices=chosen, damage=count / len(part_corners))


def cutter_radius_m(bbox_volume_m3: float, volume_fraction: float, shape: str) -> float:
    """Half-extent (cube) or radius (icosphere) that removes *volume_fraction* of a box."""
    target = max(bbox_volume_m3, 0.0) * max(volume_fraction, 0.0)
    if shape == CUTTER_CUBE:
        return float((target / 8.0) ** (1.0 / 3.0))
    if shape == CUTTER_ICOSPHERE:
        return float((3.0 * target / (4.0 * math.pi)) ** (1.0 / 3.0))
    raise ValueError(f"cutter shape is {shape!r}; use one of {list(CUTTER_SHAPES)}")


def draw_chunk_damage(
    surface_points: list[np.ndarray],
    bbox_corners: np.ndarray,
    keypoints: list[np.ndarray],
    fraction_range: tuple[float, float],
    shapes: tuple[str, ...],
    keypoint_clearance_m: float,
) -> ChunkDraw | None:
    """Seed a cutter on the surface, clear of every keypoint, or None if no seed is clear."""
    if not surface_points:
        return None
    volume = bbox_corner_volume(bbox_corners)
    if volume <= 0.0:
        return None
    for _ in range(CUTTER_SEED_RETRIES):
        shape = random.choice(list(shapes))
        fraction = random.uniform(*fraction_range)
        radius = cutter_radius_m(volume, fraction, shape)
        centre = surface_points[random.randrange(len(surface_points))]
        reach = radius * math.sqrt(3.0) + keypoint_clearance_m
        if any(float(np.linalg.norm(np.asarray(kp) - centre)) <= reach for kp in keypoints):
            continue
        return ChunkDraw(
            shape=shape,
            centre=(float(centre[0]), float(centre[1]), float(centre[2])),
            radius_m=radius,
            volume_fraction=fraction,
        )
    return None


def sample_surface_points(
    points: list[np.ndarray], limit: int = SURFACE_SAMPLE_LIMIT
) -> list[np.ndarray]:
    """At most *limit* of *points*, drawn without replacement, as cutter seed candidates."""
    if len(points) <= limit:
        return list(points)
    return [points[i] for i in random.sample(range(len(points)), limit)]
