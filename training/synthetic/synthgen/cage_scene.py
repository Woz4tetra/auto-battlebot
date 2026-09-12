"""The NHRL cage as one half of the scene mix (requires Blender).

`synthgen.cage` builds the cage that `render_cage_view.py` grades against NHRL footage, from
the fitted pose of their fixed `Cage-2-Overhead-High` phone. This module wraps the same build
in what the generic pipeline needs to render cage scenes and HDRI-arena scenes in one run:

- every cage object in one Blender collection, so the whole cage hides and shows at once;
- a snapshot of the render settings the cage look depends on (Standard view transform with the
  solved exposure gain, glass bounces, its own sample count, the rectified camera intrinsics),
  restored exactly when a generic scene renders;
- camera poses sampled from `synthgen.cage_mount` instead of the fitted event poses, so the
  mount varies the way ours would.

The cage geometry, materials, lights and exposure are the spec's, unchanged: the numbers in
`cage/cage2_overhead_high.toml` were fitted against footage and the match is only as good as
they are.
"""

from __future__ import annotations

import math
import random
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import blenderproc as bproc
import bpy
import numpy as np

from auto_battlebot.perception.camera_calibration import load_camera_calibration, rectify_maps
from synthgen.asset_index import PathResolveFn
from synthgen.cage import add_lights, apply_exposure, build_cage, house_bot_keypoints, set_world
from synthgen.cage_mount import CageMount, mount_cam2world, sample_cage_mount
from synthgen.cage_spec import CageSceneSpec, all_tubes, load_cage_spec
from synthgen.camera import target_in_frame
from synthgen.configuration import CageConfig, EnvironmentConfig, OutputConfig
from synthgen.constants import (
    BACKGROUND_CATEGORY_ID,
    HOUSE_BOT_CATEGORY_ID,
    SEG_FLOOR_CLASS_ID,
    SEG_OBJECT_CLASS_ID,
)
from synthgen.logsetup import get_logger

logger = get_logger(__name__)

CAGE_COLLECTION = "cage"
# Mounts resampled before a scene settles for one that frames the robots anyway.
MOUNT_FRAMING_RETRIES = 40
# Lights the cage owns but that live outside its mesh collection as Blender lights.
CAGE_LIGHT_PREFIXES = ("wash_", "house_bot_fill")


@dataclass(frozen=True)
class HouseBot:
    """The placed house bot and the keypoints the annotation projects each frame."""

    parent: bpy.types.Object
    kp_front: np.ndarray
    kp_back: np.ndarray

    @property
    def world_mat(self) -> np.ndarray:
        """Its 4x4 world matrix, which the keypoints are relative to."""
        return np.array(self.parent.matrix_world)


@dataclass(frozen=True)
class RenderState:
    """Everything the cage branch changes, as it was before the first cage scene."""

    view_transform: str
    look: str
    exposure: float
    gamma: float
    display_device: str
    samples: int
    glossy_bounces: int
    transmission_bounces: int
    transparent_max_bounces: int
    world_color: tuple[float, float, float]
    world_strength: float
    lens_unit: str
    lens: float
    angle: float
    shift_x: float
    shift_y: float
    clip_start: float
    clip_end: float
    resolution_x: int
    resolution_y: int
    pixel_aspect_x: float
    pixel_aspect_y: float


def snapshot_render_state() -> RenderState:
    """Capture the current render, colour-management and camera-intrinsics settings."""
    scene = bpy.context.scene
    cam = scene.camera.data
    color, strength = _world_background()
    return RenderState(
        view_transform=scene.view_settings.view_transform,
        look=scene.view_settings.look,
        exposure=float(scene.view_settings.exposure),
        gamma=float(scene.view_settings.gamma),
        display_device=scene.display_settings.display_device,
        samples=int(scene.cycles.samples),
        glossy_bounces=int(scene.cycles.glossy_bounces),
        transmission_bounces=int(scene.cycles.transmission_bounces),
        transparent_max_bounces=int(scene.cycles.transparent_max_bounces),
        world_color=color,
        world_strength=strength,
        lens_unit=cam.lens_unit,
        lens=float(cam.lens),
        angle=float(cam.angle),
        shift_x=float(cam.shift_x),
        shift_y=float(cam.shift_y),
        clip_start=float(cam.clip_start),
        clip_end=float(cam.clip_end),
        resolution_x=int(scene.render.resolution_x),
        resolution_y=int(scene.render.resolution_y),
        pixel_aspect_x=float(scene.render.pixel_aspect_x),
        pixel_aspect_y=float(scene.render.pixel_aspect_y),
    )


def restore_render_state(state: RenderState) -> None:
    """Put back what `snapshot_render_state` captured."""
    scene = bpy.context.scene
    cam = scene.camera.data
    scene.view_settings.view_transform = state.view_transform
    scene.view_settings.look = state.look
    scene.view_settings.exposure = state.exposure
    scene.view_settings.gamma = state.gamma
    scene.display_settings.display_device = state.display_device
    bproc.renderer.set_max_amount_of_samples(state.samples)
    bproc.renderer.set_light_bounces(
        glossy_bounces=state.glossy_bounces,
        transmission_bounces=state.transmission_bounces,
        transparent_max_bounces=state.transparent_max_bounces,
    )
    bproc.renderer.set_world_background(list(state.world_color), state.world_strength)
    # lens_unit first: it decides whether `lens` or `angle` is the live value.
    cam.lens_unit = state.lens_unit
    cam.lens = state.lens
    cam.angle = state.angle
    cam.shift_x = state.shift_x
    cam.shift_y = state.shift_y
    cam.clip_start = state.clip_start
    cam.clip_end = state.clip_end
    scene.render.resolution_x = state.resolution_x
    scene.render.resolution_y = state.resolution_y
    scene.render.pixel_aspect_x = state.pixel_aspect_x
    scene.render.pixel_aspect_y = state.pixel_aspect_y


def _world_background() -> tuple[tuple[float, float, float], float]:
    """The world's flat background colour and strength, or BlenderProc's default."""
    world = bpy.context.scene.world
    tree = None if world is None else world.node_tree
    node = None if tree is None else tree.nodes.get("Background")
    if node is None:
        return (0.05, 0.05, 0.05), 1.0
    color = tuple(float(v) for v in node.inputs["Color"].default_value[:3])
    return color, float(node.inputs["Strength"].default_value)  # type: ignore[return-value]


class CageStage:
    """The built cage, plus the show/hide and camera work one scene needs."""

    def __init__(
        self,
        cage_cfg: CageConfig,
        spec: CageSceneSpec,
        k_rect: np.ndarray,
        width: int,
        height: int,
        collection: bpy.types.Collection,
        tubes: list[Any],
        house_bot: HouseBot | None,
        generic_state: RenderState,
    ) -> None:
        self._cfg = cage_cfg
        self._spec = spec
        self._k_rect = k_rect
        self._width = width
        self._height = height
        self._collection = collection
        self._tubes = tubes
        self.house_bot = house_bot
        self._generic_state = generic_state
        self._tube_strengths = [_tube_strength(tube) for tube in tubes]
        self._lights = [
            (obj, float(obj.data.energy))
            for obj in bpy.data.objects
            if obj.type == "LIGHT" and obj.name.startswith(CAGE_LIGHT_PREFIXES)
        ]

    @property
    def arena_radius(self) -> float:
        """Half the mat, less the margin kept clear of its edge."""
        return self._spec.mat.size / 2 - self._cfg.mat_margin_m

    @property
    def wall_half(self) -> float:
        """Half the cage interior span: where the walls, and so the mounts, sit."""
        return self._spec.cage.interior / 2

    def activate(self, point_lights: Sequence[bproc.types.Light] = ()) -> None:
        """Show the cage, take over the render settings, and silence the roaming lights."""
        self._collection.hide_render = False
        self._collection.hide_viewport = False
        for obj, energy in self._lights:
            obj.data.energy = energy
        for light in point_lights:
            light.set_energy(0)
        apply_exposure(self._spec, self._spec.exposure.gain)
        set_world(self._spec, tuple(float(v) for v in self._spec.exposure.color_gain))
        bproc.renderer.set_light_bounces(
            transmission_bounces=self._spec.render.transmission_bounces,
            glossy_bounces=self._spec.render.glossy_bounces,
            transparent_max_bounces=self._spec.render.transmission_bounces,
        )
        if self._cfg.render_samples is not None:
            bproc.renderer.set_max_amount_of_samples(self._cfg.render_samples)
        bproc.camera.set_intrinsics_from_K_matrix(self._k_rect, self._width, self._height)

    def deactivate(self) -> None:
        """Hide the cage and restore the settings a generic scene expects."""
        self._collection.hide_render = True
        self._collection.hide_viewport = True
        for obj, _ in self._lights:
            obj.data.energy = 0.0
        restore_render_state(self._generic_state)

    def jitter_lights(self) -> None:
        """Re-roll the LED tube strengths so the set is not one lighting state."""
        fraction = self._cfg.tube_jitter
        if fraction <= 0:
            return
        for tube, strength in zip(self._tubes, self._tube_strengths):
            _set_tube_strength(tube, strength * random.uniform(1 - fraction, 1 + fraction))

    def sample_camera_poses(
        self, count: int, look_at: list[float]
    ) -> tuple[list[np.ndarray], list[CageMount], int]:
        """Sample *count* mount poses that frame *look_at*.

        Returns:
            ``(cam2world poses, mounts, fallback_count)`` where fallback_count counts poses
            that ran out of retries and kept the last sample regardless of framing.
        """
        poses: list[np.ndarray] = []
        mounts: list[CageMount] = []
        fallbacks = 0
        for _ in range(count):
            mount, cam2world, framed = self._draw_framed_mount(look_at)
            if not framed:
                fallbacks += 1
            poses.append(cam2world)
            mounts.append(mount)
        return poses, mounts, fallbacks

    def _draw_framed_mount(self, look_at: list[float]) -> tuple[CageMount, np.ndarray, bool]:
        """Draw mounts until one frames *look_at*; the last draw is kept if none does."""
        mount = sample_cage_mount(self._cfg.mount)
        cam2world = mount_cam2world(mount, self.wall_half)
        for _ in range(MOUNT_FRAMING_RETRIES):
            if target_in_frame(cam2world, look_at):
                return mount, cam2world, True
            mount = sample_cage_mount(self._cfg.mount)
            cam2world = mount_cam2world(mount, self.wall_half)
        return mount, cam2world, False


def build_cage_stage(
    cage_cfg: CageConfig,
    output_cfg: OutputConfig,
    env_cfg: EnvironmentConfig,
    resolve: PathResolveFn,
    project_root: Path,
    is_segmentation_mode: bool,
) -> CageStage:
    """Build the cage into the current scene and return it hidden, ready to activate.

    Call this after every other mesh exists but before BlenderProc's segmentation output is
    armed, for the same reason the ground plane is built there: pass indices are assigned at
    that call.
    """
    spec = load_cage_spec(resolve(cage_cfg.spec))
    calibration = load_camera_calibration(resolve(cage_cfg.camera_calibration))
    width, height = output_cfg.image_width, output_cfg.image_height
    _, _, k_rect = rectify_maps(calibration, (width, height))
    logger.info(
        "cage camera %s rectified to %dx%d: fx %.1f, %.1f deg horizontal",
        calibration.calibration_id,
        width,
        height,
        k_rect[0, 0],
        math.degrees(2 * math.atan(width / 2 / k_rect[0, 0])),
    )

    generic_state = snapshot_render_state()
    # In keypoint mode the mat is background and the house bot is its own detector class; in
    # segmentation mode the mat is the floor and the house bot is one more object on it.
    mat_category_id = SEG_FLOOR_CLASS_ID if is_segmentation_mode else BACKGROUND_CATEGORY_ID
    house_bot_category_id = SEG_OBJECT_CLASS_ID if is_segmentation_mode else HOUSE_BOT_CATEGORY_ID
    cc_dir = None if env_cfg.cc_textures_dir is None else resolve(env_cfg.cc_textures_dir)

    before = set(bpy.data.objects)
    build_cage(
        spec,
        project_root,
        cc_dir,
        mat_category_id=mat_category_id,
        house_bot_category_id=house_bot_category_id,
    )
    lights = add_lights(spec, tuple(float(v) for v in spec.exposure.color_gain))
    created = [obj for obj in bpy.data.objects if obj not in before]
    collection = _isolate(created, CAGE_COLLECTION)

    tubes = [light for light in lights if not hasattr(light, "set_color")]
    house_bot = _house_bot(is_segmentation_mode)
    logger.info(
        "cage stage: %d objects, %d LED tubes, mat %.2f m, arena radius %.2f m",
        len(created),
        len(all_tubes(spec)),
        spec.mat.size,
        spec.mat.size / 2 - cage_cfg.mat_margin_m,
    )
    stage = CageStage(
        cage_cfg, spec, k_rect, width, height, collection, tubes, house_bot, generic_state
    )
    stage.deactivate()
    return stage


def _house_bot(is_segmentation_mode: bool) -> HouseBot | None:
    """The house bot record, or None when it is disabled, absent, or not a detector class."""
    parent = bpy.data.objects.get("house_bot")
    if parent is None or is_segmentation_mode:
        return None
    keypoints = house_bot_keypoints(parent)
    if keypoints is None:
        logger.warning("house bot has no geometry to place keypoints on; it will go unlabelled")
        return None
    front, back = keypoints
    logger.info("house bot keypoints (parent local): front %s, back %s", front, back)
    return HouseBot(parent, front, back)


def _isolate(objects: list[bpy.types.Object], name: str) -> bpy.types.Collection:
    """Move *objects* into their own collection, so one flag hides all of them."""
    collection = bpy.data.collections.get(name) or bpy.data.collections.new(name)
    if collection.name not in {child.name for child in bpy.context.scene.collection.children}:
        bpy.context.scene.collection.children.link(collection)
    for obj in objects:
        for parent in list(obj.users_collection):
            parent.objects.unlink(obj)
        collection.objects.link(obj)
    return collection


def _tube_strength(tube: Any) -> float:
    node = tube.get_materials()[0].get_the_one_node_with_type("BsdfPrincipled")
    return float(node.inputs["Emission Strength"].default_value)


def _set_tube_strength(tube: Any, strength: float) -> None:
    node = tube.get_materials()[0].get_the_one_node_with_type("BsdfPrincipled")
    node.inputs["Emission Strength"].default_value = strength
