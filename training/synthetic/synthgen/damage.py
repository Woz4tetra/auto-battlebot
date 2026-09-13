"""Battle damage as a per-instance random variable: what to remove, decided without Blender.

A real robot two minutes into a match is missing parts. Every synthetic robot so far is
factory fresh, so the detector never sees a chewed one until the match does. This module
draws the damage; ``synthgen.damage_scene`` applies it to the Blender scene and undoes it.

Three mechanisms, chosen by what the robot's config and mesh structure allow:

* **Named part removal**, for a robot whose ``[[robots.damage_parts]]`` names its real
  assemblies (wheels, weapon disk, top plate). A CAD export grouped by material colour has
  no part structure to hide, so ``synthgen.robots.split_damage_parts`` cuts the model into
  those named pieces once at load, and a draw removes whole assemblies. The random-fraction
  mechanism below hid a few bolts at a time, which never looked like a robot that lost a
  fight. Named removal skips the keypoint protection: the batch asked for those parts gone.

* **Part removal**, for the CAD robots. ``import_gltf_as_robot`` returns the GLB's mesh
  objects, so those robots arrive already split into parts and a subset can simply go.
* **Chunk removal**, for the Meshy opponents. Those GLBs are usually one fused textured
  mesh, where part removal would either do nothing or delete the whole robot, so a cutter
  primitive seeded on the surface takes a bite out instead.

Three things are protected, because damage must not turn a label into a lie:

* a part whose bounding box contains a keypoint, since ``[robots.keypoints]`` are
  model-frame offsets and deleting what one sits on leaves it floating in mid-air;
* a part big enough that removing it removes most of the robot;
* anything matching a configured name pattern, for cases the geometry misses.

``damage`` in the manifest means the fraction of the robot's mesh parts removed for the
parts mechanism, and the fraction of bounding volume the cutter was sized to take for the
chunk mechanism. They are not the same scale and are not comparable across mechanisms;
``mechanism`` is recorded beside the number so an arm can separate them.

Draws use the module-level ``random`` like the rest of the pipeline, so ``--seed`` covers
damage too.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

import numpy as np

MECHANISM_NONE = "none"
MECHANISM_PARTS = "parts"
MECHANISM_CHUNK = "chunk"
MECHANISM_NAMED = "named"

CUTTER_CUBE = "cube"
CUTTER_ICOSPHERE = "icosphere"
CUTTER_SHAPES = (CUTTER_CUBE, CUTTER_ICOSPHERE)

# Tries at seeding a cutter away from every keypoint before giving up on this instance.
CHUNK_PLACEMENT_RETRIES = 12
# Surface vertices sampled as candidate cutter seeds. A fused Meshy mesh can carry 100k
# vertices and the draw only needs a handful of candidates.
SURFACE_SAMPLE_LIMIT = 64


@dataclass(frozen=True)
class InstanceDamage:
    """What one robot instance ended up with, recorded per frame in ``manifest.jsonl``."""

    class_name: str
    damage: float
    mechanism: str
    # Named removal only: (part name, removed piece indices), in draw order.
    parts: tuple[tuple[str, tuple[int, ...]], ...] = ()

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
    """Tracks the damaged/clean scene split so a short run still lands on the ratio.

    Damage is drawn per scene, so a run is a handful of draws, not thousands: a 100-frame
    probe at ten frames per scene is ten of them. A coin flip at 0.5 put one scene in ten
    on the damaged side on 2026-09-12, which is inside its binomial spread and useless as
    a clean pool. This tracks the split the way ``choose_cage`` tracks the scene mix: each
    scene goes to whichever side is behind its share, so the ratio holds at any length.
    """

    scenes: int = 0
    damaged: int = 0

    def take(self, scene_probability: float) -> bool:
        """Whether the next scene is a damaged one, and record the decision."""
        wanted = scene_probability * (self.scenes + 1)
        # The half-step is the usual rounding tracker: it keeps the running count on the
        # nearer side of the target share instead of always damaging the first scene.
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
    """One named assembly a robot can lose, as the draw sees it.

    ``pieces`` counts the separate pieces the load-time split produced for it: one for an
    assembly that goes whole, one per box for a ``subset`` part such as the four wheels,
    where a draw takes 1..pieces of them.
    """

    name: str
    pieces: int
    subset: bool = False
    # Parts that go with this one, e.g. the weapon disk with the weapon module it hangs on.
    includes: tuple[str, ...] = ()
    # Subset parts: relative odds of losing 1, 2, ... pieces. Empty means every count is
    # equally likely; a wheel usually comes off alone, so the wheels weight one heavily.
    count_weights: tuple[float, ...] = ()
    # False for a part that only goes with another (a decal with its plate): never drawn alone.
    selectable: bool = True


@dataclass(frozen=True)
class NamedPartDraw:
    """Which pieces of which named parts to hide, and what fraction of all pieces that is."""

    removed: tuple[tuple[str, tuple[int, ...]], ...]
    damage: float


@dataclass(frozen=True)
class FacePiece:
    """Where one piece of a named part is, for assigning mesh faces to it at load.

    Boxes are ``(x_min, y_min, z_min, x_max, y_max, z_max)`` in the robot's parent frame.
    ``objects`` and ``exclude_objects`` filter on the Blender object name, matched by
    ``object_name_matches``; for a GLB grouped by colour that name is the material colour.
    """

    boxes: tuple[tuple[float, float, float, float, float, float], ...]
    objects: tuple[str, ...] = ()
    exclude_objects: tuple[str, ...] = ()


def model_box_to_blender_local(
    box: tuple[float, float, float, float, float, float],
) -> tuple[float, float, float, float, float, float]:
    """A model-frame (glTF, Y-up) box in Blender local axes, the conversion keypoints get.

    ``(x, y, z)`` maps to ``(x, -z, y)``, so the z bounds swap and negate into y.
    """
    x0, y0, z0, x1, y1, z1 = box
    return (x0, -z1, y0, x1, -z0, y1)


def base_object_name(name: str) -> str:
    """*name* without Blender's ``.001`` duplicate suffix."""
    head, dot, tail = name.rpartition(".")
    return head if dot and tail.isdigit() else name


def object_name_matches(name: str, pattern: str) -> bool:
    """Whether Blender object *name* is *pattern*, or *pattern* plus a ``_N`` index.

    A Blender glTF export splits one colour into ``mat_128_128_128``, ``mat_128_128_128_1``,
    and so on, and a second import appends ``.001``; all of them are the pattern's colour.
    """
    base = base_object_name(name)
    if base == pattern:
        return True
    head, underscore, tail = base.rpartition("_")
    return bool(underscore) and head == pattern and tail.isdigit()


def assign_faces_to_pieces(
    centres: np.ndarray, object_name: str, pieces: list[FacePiece]
) -> np.ndarray:
    """Per face, the index of the first piece whose boxes hold its centre, or -1.

    First match wins, so config order settles faces two parts' boxes both hold: the weapon
    disk is listed before the weapon module that surrounds it.

    Args:
        centres: ``(n, 3)`` face centres in the parent frame.
        object_name: The Blender object the faces belong to.
        pieces: Every piece of every named part, in config order.
    """
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
    """How many of a subset part's pieces one draw removes, from ``count_weights``.

    Weights past the part's piece count are ignored and missing ones count as zero, so a
    wheels part that lost a box to an empty split still draws from the counts it can reach.
    """
    counts = list(range(1, part.pieces + 1))
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
    """Pick 1..n of the allowed named parts to remove, or None when none is allowed.

    Args:
        parts: The robot's named parts, in config order. Parts with no pieces are skipped.
        allowed: The batch's ``[damage].removable_parts``; empty allows every part.
        count_range: How many distinct parts one draw removes, clamped to what is allowed.
    """
    by_name = {part.name: part for part in parts if part.pieces > 0}
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
            chosen = random.sample(range(part.pieces), piece_count(part))
            removed.setdefault(name, set()).update(chosen)
        else:
            removed.setdefault(name, set()).update(range(part.pieces))
        for included in part.includes:
            if included in by_name:
                removed.setdefault(included, set()).update(range(by_name[included].pieces))
    total = sum(part.pieces for part in by_name.values())
    return NamedPartDraw(
        removed=tuple((name, tuple(sorted(indices))) for name, indices in removed.items()),
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
    """Parts that must survive: keypoint anchors, oversized parts, and named patterns.

    Args:
        part_corners: Per part, an ``(n, 3)`` array of points in the robot's parent frame.
        part_names: Per part, its Blender object name.
        keypoints: Keypoint positions in the same parent frame.
        name_patterns: Lowercase substrings; a part whose name contains one is protected.
        max_volume_fraction: Parts holding more than this share of the summed part
            bounding volumes are protected, so one draw cannot take most of the robot.
            Summed rather than the robot's own box, so the test needs nothing but the
            parts themselves; overlapping boxes make it a little conservative, which is
            the safe direction for a protection rule.
        keypoint_clearance_m: Bounding boxes are grown by this before the keypoint test.

    Returns:
        Indices into *part_corners* that may not be removed.
    """
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
    """Half-extent (cube) or radius (icosphere) that removes *volume_fraction* of a box.

    The cutter is seeded on the surface rather than buried in the middle, so in practice
    roughly half of it overlaps the mesh; the fraction is what the cutter was sized for,
    not a measured loss.

    Raises:
        ValueError: When *shape* is not a known cutter shape.
    """
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
    """Seed a cutter on the surface, clear of every keypoint, or None if no seed is clear.

    A cutter that swallows a keypoint would leave the annotation pointing at a hole, so
    candidate seeds within the cutter's own reach of a keypoint are rejected and redrawn.
    """
    if not surface_points:
        return None
    volume = bbox_corner_volume(bbox_corners)
    if volume <= 0.0:
        return None
    for _ in range(CHUNK_PLACEMENT_RETRIES):
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
