"""Battle damage as a per-instance random variable: what to remove, decided without Blender.

A real robot two minutes into a match is missing parts. Every synthetic robot so far is
factory fresh, so the detector never sees a chewed one until the match does. This module
draws the damage; ``synthgen.damage_scene`` applies it to the Blender scene and undoes it.

Two mechanisms, chosen by mesh structure:

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

    @property
    def is_damaged(self) -> bool:
        """Whether this instance carries any damage at all."""
        return self.mechanism != MECHANISM_NONE and self.damage > 0.0

    def as_row(self) -> dict[str, object]:
        """The manifest form: compact, and stable across mechanisms."""
        return {
            "class": self.class_name,
            "damage": round(float(self.damage), 4),
            "mechanism": self.mechanism,
        }


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
