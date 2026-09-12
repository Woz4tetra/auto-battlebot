"""Apply one scene's battle damage to the Blender scene, and undo it (requires Blender).

The decisions live in the pure ``synthgen.damage``; this module is the hands.

Nothing here edits mesh data, because it must not. ``load_robots`` and the distractor pool
load every model once and reuse it for every later scene, so a deleted part would stay
deleted for the rest of the run and a boolean applied to mesh data would compound. Instead:

* part removal hides the chosen parts from the render, which gives the same silhouette to
  the colour pass and the segmentation pass as deleting them would;
* chunk removal borrows a cutter from a pool built once at startup and attaches a boolean
  difference modifier to the target mesh.

Both are recorded in a ``DamageSession`` and reverted after the scene's frames are written,
which is also why the cutter pool is pre-built: objects created mid-run would miss the
segmentation pass arming that ``_enable_segmentation`` does over the meshes that exist at
call time. Pool cutters are created before it, never render, and carry the background
category id, so they are invisible to both passes.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

import bpy
import mathutils
import numpy as np

from synthgen.configuration import DamageConfig
from synthgen.constants import BACKGROUND_CATEGORY_ID, DISTRACTOR_OFFSCREEN_LOCATION
from synthgen.damage import (
    CUTTER_CUBE,
    CUTTER_ICOSPHERE,
    MECHANISM_CHUNK,
    MECHANISM_NONE,
    MECHANISM_PARTS,
    ChunkDraw,
    DamageBudget,
    InstanceDamage,
    PartDraw,
    draw_chunk_damage,
    draw_part_damage,
    sample_surface_points,
)
from synthgen.distractors import DistractorInstance
from synthgen.logsetup import get_logger
from synthgen.robots import RobotInstance

logger = get_logger(__name__)

CUTTER_NAME_PREFIX = "damage_cutter"
# Unit primitives, scaled per use: a cube of half-extent 1 and a sphere of radius 1.
_CUTTER_UNIT_SIZE = 2.0


@dataclass(frozen=True)
class DamageTarget:
    """One thing damage can happen to: its parent, its meshes, and its keypoints.

    Robots and robot-like distractors differ in type but not in what damage needs from
    them, so both arrive here. Keypoints are in the parent frame, as modelled.
    """

    parent: bpy.types.Object
    meshes: list[bpy.types.Object]
    keypoints: tuple[np.ndarray | None, ...]


@dataclass
class DamageSession:
    """Every change one scene's damage made, and how to put it back."""

    hidden: list[tuple[bpy.types.Object, bool, bool]] = field(default_factory=list)
    modifiers: list[tuple[bpy.types.Object, str]] = field(default_factory=list)
    cutters: list[bpy.types.Object] = field(default_factory=list)

    def revert(self) -> None:
        """Unhide every hidden part, drop every boolean, and park every cutter."""
        for obj, hide_render, hide_viewport in self.hidden:
            obj.hide_render = hide_render
            obj.hide_viewport = hide_viewport
        self.hidden.clear()
        for obj, modifier_name in self.modifiers:
            modifier = obj.modifiers.get(modifier_name)
            if modifier is not None:
                obj.modifiers.remove(modifier)
        self.modifiers.clear()
        for cutter in self.cutters:
            cutter.location = mathutils.Vector(DISTRACTOR_OFFSCREEN_LOCATION)
        self.cutters.clear()


class CutterPool:
    """Boolean cutter objects, built once and handed out per scene."""

    def __init__(self, cutters: list[bpy.types.Object]) -> None:
        self._cutters = cutters
        self._next = 0

    def __len__(self) -> int:
        return len(self._cutters)

    def reset(self) -> None:
        """Start handing out from the top again, for a new scene."""
        self._next = 0

    def acquire(self, shape: str) -> bpy.types.Object | None:
        """The next free cutter of *shape*, or None when the pool is exhausted.

        Cutters alternate shapes, so this scans forward for the first match and swaps it
        to the front of the free range rather than keeping a queue per shape.
        """
        for index in range(self._next, len(self._cutters)):
            cutter = self._cutters[index]
            if cutter.get("cutter_shape") == shape:
                self._cutters[self._next], self._cutters[index] = (
                    self._cutters[index],
                    self._cutters[self._next],
                )
                self._next += 1
                return self._cutters[self._next - 1]
        return None


def build_cutter_pool(size: int) -> CutterPool:
    """Create *size* parked cutters, half cubes and half icospheres.

    Call this before ``_enable_segmentation`` so the cutters are covered by the same arming
    as everything else, and before any render so they never appear in one.
    """
    cutters: list[bpy.types.Object] = []
    for index in range(max(size, 0)):
        shape = CUTTER_CUBE if index % 2 == 0 else CUTTER_ICOSPHERE
        if shape == CUTTER_CUBE:
            bpy.ops.mesh.primitive_cube_add(size=_CUTTER_UNIT_SIZE)
        else:
            bpy.ops.mesh.primitive_ico_sphere_add(radius=1.0, subdivisions=2)
        cutter = bpy.context.active_object
        cutter.name = f"{CUTTER_NAME_PREFIX}_{index:03d}"
        cutter["cutter_shape"] = shape
        cutter["category_id"] = BACKGROUND_CATEGORY_ID
        cutter.hide_render = True
        cutter.location = mathutils.Vector(DISTRACTOR_OFFSCREEN_LOCATION)
        cutters.append(cutter)
    logger.info("Built %d boolean cutters for battle damage", len(cutters))
    return CutterPool(cutters)


# The two mechanisms work in different frames, on purpose.
#
# Chunk removal works in WORLD space, because the cutter is an unparented world object: a
# distractor's parent carries a per-scene scale (place_scene_distractors resizes each Meshy
# model to the 3 lb class), so a radius measured in the parent frame and applied in the
# world comes out wrong by that scale factor.
#
# Part removal works in the robot's PARENT frame, because that is the frame the parts and
# the keypoints were modelled in. Bounding boxes there are axis-aligned to the robot, so
# which parts are protected does not depend on which way the robot happens to be facing;
# measured over three poses on 2026-09-12, the same robot's protected set held at 9 and 36
# parts in its own frame while world-space boxes wobbled between 6 and 71 as it rotated.
# The one length that does not survive the frame change is keypoint_clearance_m, which the
# caller divides by the parent's scale to keep it a clearance in metres.


def _world_corners(obj: bpy.types.Object) -> np.ndarray:
    """*obj*'s bounding box corners in world space."""
    corners = [list(obj.matrix_world @ mathutils.Vector(c)) for c in obj.bound_box]
    return np.array(corners, dtype=np.float64)


def _parent_corners(parent: bpy.types.Object, obj: bpy.types.Object) -> np.ndarray:
    """*obj*'s bounding box corners in *parent*'s frame."""
    to_parent = parent.matrix_world.inverted() @ obj.matrix_world
    corners = [list(to_parent @ mathutils.Vector(c)) for c in obj.bound_box]
    return np.array(corners, dtype=np.float64)


def _parent_scale(parent: bpy.types.Object) -> float:
    """*parent*'s uniform scale factor, for converting a metre length into its frame."""
    scale = parent.matrix_world.to_scale()
    mean = (abs(scale[0]) + abs(scale[1]) + abs(scale[2])) / 3.0
    return mean if mean > 1e-9 else 1.0


def _world_vertices(obj: bpy.types.Object) -> list[np.ndarray]:
    """A sample of *obj*'s vertices in world space, as cutter seed candidates."""
    mesh = getattr(obj, "data", None)
    if mesh is None or not getattr(mesh, "vertices", None):
        return []
    indices = sample_surface_points(list(range(len(mesh.vertices))))
    matrix = obj.matrix_world
    return [np.array(list(matrix @ mesh.vertices[int(i)].co), dtype=np.float64) for i in indices]


def _parent_keypoints(*points: np.ndarray | None) -> list[np.ndarray]:
    """The keypoints that exist. They are modelled in the parent frame already."""
    return [np.asarray(p, dtype=np.float64).reshape(3) for p in points if p is not None]


def _world_keypoints(parent: bpy.types.Object, *points: np.ndarray | None) -> list[np.ndarray]:
    """The keypoints that exist, lifted from the parent frame into world space."""
    world = []
    for point in points:
        if point is None:
            continue
        local = mathutils.Vector(np.asarray(point, dtype=np.float64).reshape(3).tolist())
        world.append(np.array(list(parent.matrix_world @ local), dtype=np.float64))
    return world


def _apply_part_draw(
    session: DamageSession, meshes: list[bpy.types.Object], draw: PartDraw
) -> None:
    """Hide the drawn parts, remembering what their visibility was."""
    for index in draw.part_indices:
        obj = meshes[index]
        session.hidden.append((obj, obj.hide_render, obj.hide_viewport))
        obj.hide_render = True
        obj.hide_viewport = True


def _apply_chunk_draw(
    session: DamageSession,
    pool: CutterPool,
    target: bpy.types.Object,
    draw: ChunkDraw,
) -> bool:
    """Place a pool cutter and attach the boolean. False when the pool is exhausted."""
    cutter = pool.acquire(draw.shape)
    if cutter is None:
        return False
    # The draw is already in world space, and the cutter is an unparented world object, so
    # its location and scale are the drawn numbers with no frame conversion.
    cutter.location = mathutils.Vector(draw.centre)
    cutter.scale = (draw.radius_m, draw.radius_m, draw.radius_m)
    # Random orientation so a cube cutter leaves a different facet shape each time.
    cutter.rotation_euler = tuple(random.uniform(0.0, 2.0 * math.pi) for _ in range(3))
    name = f"damage_{cutter.name}"
    modifier = target.modifiers.new(name=name, type="BOOLEAN")
    modifier.operation = "DIFFERENCE"
    # EXACT copes with the non-manifold Meshy meshes; FAST silently returns the input on them.
    modifier.solver = "EXACT"
    modifier.object = cutter
    session.modifiers.append((target, name))
    session.cutters.append(cutter)
    return True


def _damage_instance(
    cfg: DamageConfig,
    session: DamageSession,
    pool: CutterPool,
    class_name: str,
    instance: DamageTarget,
) -> InstanceDamage:
    """Draw and apply damage for one instance. Undamaged is a normal outcome."""
    undamaged = InstanceDamage(class_name=class_name, damage=0.0, mechanism=MECHANISM_NONE)
    meshes = instance.meshes
    if not meshes:
        return undamaged

    if len(meshes) > 1:
        part_draw = draw_part_damage(
            [_parent_corners(instance.parent, obj) for obj in meshes],
            [obj.name for obj in meshes],
            _parent_keypoints(*instance.keypoints),
            cfg.part_severity,
            cfg.protected_name_patterns,
            cfg.max_part_volume_fraction,
            cfg.keypoint_clearance_m / _parent_scale(instance.parent),
        )
        if part_draw is not None:
            _apply_part_draw(session, meshes, part_draw)
            return InstanceDamage(
                class_name=class_name, damage=part_draw.damage, mechanism=MECHANISM_PARTS
            )

    # One fused mesh, or a part robot where everything is protected: take a bite instead.
    target = max(meshes, key=lambda o: len(getattr(o.data, "vertices", ()) or ()))
    chunk_draw = draw_chunk_damage(
        _world_vertices(target),
        _world_corners(target),
        _world_keypoints(instance.parent, *instance.keypoints),
        cfg.chunk_volume_fraction,
        cfg.cutter_shapes,
        cfg.keypoint_clearance_m,
    )
    if chunk_draw is None:
        return undamaged
    if not _apply_chunk_draw(session, pool, target, chunk_draw):
        logger.debug("cutter pool exhausted; %s left undamaged this scene", class_name)
        return undamaged
    return InstanceDamage(
        class_name=class_name, damage=chunk_draw.volume_fraction, mechanism=MECHANISM_CHUNK
    )


def apply_scene_damage(
    cfg: DamageConfig,
    pool: CutterPool,
    budget: DamageBudget,
    robots: list[RobotInstance],
    distractors: list[DistractorInstance],
    distractor_class_name: str,
) -> tuple[DamageSession, list[InstanceDamage]]:
    """Damage a scene's robots and robot-like distractors; returns the session to revert.

    Args:
        cfg: The parsed ``[damage]`` block.
        pool: Pre-built cutters, reset here for this scene.
        budget: The run's damaged/clean scene tracker, advanced by one scene here.
        robots: The scene's own robots.
        distractors: The scene's active distractors; only robot-like ones are damaged,
            since a cut-up traffic cone teaches the detector nothing about a battlebot.
        distractor_class_name: The generic class the robot-like distractors annotate as.

    Returns:
        ``(session, per-instance damage)``, one entry per instance considered, damaged or
        not, in robots-then-distractors order. Every entry goes in the manifest, so an arm
        can filter on frames whose instances are all undamaged.
    """
    session = DamageSession()
    recorded: list[InstanceDamage] = []
    if not cfg.enabled:
        return session, recorded

    # Whole scenes are clean or not, which is what keeps the clean pool the size the arms
    # need. Rolling only per instance would not: a scene carries several robot-like
    # instances, so a frame with every one of them clean measured 15 percent of frames at
    # probability 0.35, not the half a damage-off arm has to draw from. The split is
    # tracked rather than flipped, because a run is only a handful of scenes.
    scene_damaged = budget.take(cfg.scene_probability)
    pool.reset()
    targets: list[tuple[str, DamageTarget]] = [
        (
            robot.name,
            DamageTarget(
                parent=robot.parent,
                meshes=[m.blender_obj for m in robot.meshes],
                keypoints=(robot.kp_front, robot.kp_back),
            ),
        )
        for robot in robots
    ]
    targets.extend(
        (
            distractor_class_name,
            DamageTarget(
                parent=distractor.parent,
                meshes=[m.blender_obj for m in distractor.meshes],
                keypoints=(distractor.kp_front, distractor.kp_back),
            ),
        )
        for distractor in distractors
        if distractor.is_robot_like
    )

    for class_name, instance in targets:
        if not scene_damaged or random.random() >= cfg.probability:
            recorded.append(
                InstanceDamage(class_name=class_name, damage=0.0, mechanism=MECHANISM_NONE)
            )
            continue
        recorded.append(_damage_instance(cfg, session, pool, class_name, instance))
    return session, recorded
