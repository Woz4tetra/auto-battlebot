"""Preview and full-quality rendering for `pose_camera_server.py` (requires Blender).

Owns three things the HTTP half must never touch: the render settings for each mode, the camera
pose, and the three views a frame can be shown in.

**Views.** `pinhole` renders at the rectified matrix, which is what the batch pipeline writes
today. `distorted` renders what the sensor actually sees, through BlenderProc's own
`set_lens_distortion`. `rectified` puts that distorted frame back through the same
`initUndistortRectifyMap` the C++ `Rectifier` builds on the robot, black border and all. Flying in
`pinhole` and flipping to `rectified` is the comparison that decides whether the batch render
should carry distortion.

**Mode switching writes a complete set of properties, never deltas.** `restore_render_state` from
`cage_scene` puts back the drift-prone half (color management, bounces, K, resolution); this
module writes the rest on top (engine, samples, denoising, persistent data, image format,
resolution percentage). Preview and full mode each go through the same transition, so neither can
inherit a leftover from the other.

**The camera is posed by assigning `matrix_world`, never with `add_camera_pose`.** Keyframes would
accumulate on every WASD frame and a later render would walk the whole animation range. Nothing
here inserts a keyframe, so `animation_data` stays `None` for the life of the process, which
`_assert_no_keyframes` checks on every frame: if it ever appears, direct pose assignment starts
being silently overwritten and WASD mysteriously stops responding.
"""

from __future__ import annotations

import contextlib
import os
import sys
import time
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import blenderproc as bproc
import bpy
import cv2
import mathutils
import numpy as np

from auto_battlebot.perception.camera_calibration import CameraCalibration, rectify_maps
from synthgen.cage import panels_by_wall
from synthgen.cage_scene import RenderState, restore_render_state, snapshot_render_state
from synthgen.cage_spec import CageSceneSpec, panels_outside_camera
from synthgen.logsetup import get_logger
from synthgen.preview_server import ALPHAS, mat_sample_points

logger = get_logger(__name__)

# tmpfs, so the render never touches a disk. One fixed name: a varying one fills /dev/shm.
PREVIEW_PATH = Path("/dev/shm/ab_pose_preview.jpg")  # noqa: S108 - tmpfs is the point
PREVIEW_SAMPLES = 8
PREVIEW_QUALITY = 90


@dataclass(frozen=True)
class ViewSetup:
    """Everything one view needs, cached so a key press does not redo the expensive parts.

    `set_lens_distortion` solves the inverse distortion per output pixel and enlarges the render,
    so both its maps and the enlarged intrinsics it chose are kept here. Switching back to a view
    re-applies those intrinsics rather than solving again.
    """

    name: str
    size: tuple[int, int]
    intrinsics: np.ndarray
    intrinsics_size: tuple[int, int]
    resolution_percentage: int
    distortion_map: tuple[np.ndarray, np.ndarray] | None = None
    rectify_map: tuple[np.ndarray, np.ndarray] | None = None


class OneWayGlass:
    """Hide the pane the camera stands outside of, the way the batch render does.

    BlenderProc's segmentation stops at the polycarbonate, so a pose shooting through a pane from
    outside the cage loses every label behind it. `render_poses` solves that by hiding those panes,
    and the preview has to do the same or a mount flown outside the wall plane looks nothing like
    what it will render as.

    Unlike `cage.set_one_way_glass`, this writes `hide_render` directly instead of keyframing it:
    the batch path animates visibility because one pass covers several poses, where the preview
    only ever shows one. The hidden set is only rewritten when it changes, because toggling
    visibility re-syncs geometry and throws away Cycles' persistent-data cache.
    """

    def __init__(self, spec: CageSceneSpec, objects: dict[str, bproc.types.MeshObject]) -> None:
        self._spec = spec
        self._panels = panels_by_wall(objects)
        self._hidden: frozenset[str] | None = None
        for obj in self._panels.values():
            obj.blender_obj.animation_data_clear()

    @property
    def hidden(self) -> tuple[str, ...]:
        return tuple(sorted(self._hidden or ()))

    def update(self, camera_position: np.ndarray) -> None:
        hidden = frozenset(
            panels_outside_camera(
                self._spec, (float(camera_position[0]), float(camera_position[1]))
            )
        )
        if hidden == self._hidden:
            return
        for wall, obj in self._panels.items():
            obj.blender_obj.hide_render = wall in hidden
        self._hidden = hidden
        logger.debug("one-way glass hides %s", sorted(hidden) or "nothing")


class PreviewRenderer:
    """Renders the built cage from an arbitrary camera pose, fast enough to fly with."""

    def __init__(
        self,
        calibration: CameraCalibration,
        spec: CageSceneSpec,
        render_size: tuple[int, int],
        preview_percentage: int,
    ) -> None:
        self._calibration = calibration
        self._spec = spec
        self._render_size = render_size
        self._preview_percentage = preview_percentage
        self._preview_size = (
            max(1, round(render_size[0] * preview_percentage / 100)),
            max(1, round(render_size[1] * preview_percentage / 100)),
        )
        # Both rectification alphas up front: the page toggles between them live, and each one is
        # a different rectified matrix, so the pinhole view and the overlay both have to follow.
        self._k_rect = {alpha: rectify_maps(calibration, render_size, alpha)[2] for alpha in ALPHAS}
        self._cage_state: RenderState | None = None
        self._setups: dict[tuple[str, bool, float], ViewSetup] = {}
        self._applied: tuple[str, bool, float] | None = None

    # -- properties ------------------------------------------------------------------------

    def k_rect(self, alpha: float = 1.0) -> np.ndarray:
        """The rectified matrix for *alpha*, at the full render size."""
        return self._k_rect[alpha]

    @property
    def preview_size(self) -> tuple[int, int]:
        return self._preview_size

    @property
    def render_size(self) -> tuple[int, int]:
        return self._render_size

    # -- lifecycle -------------------------------------------------------------------------

    def capture_scene_state(self) -> None:
        """Remember the cage's own render settings. Call once, after the cage is active.

        The intrinsics go in first, so the snapshot `restore_render_state` puts back on every mode
        transition is this camera at full resolution rather than BlenderProc's 512x512 default.
        """
        bproc.camera.set_intrinsics_from_K_matrix(
            self._k_rect[1.0], self._render_size[0], self._render_size[1]
        )
        bpy.context.scene.render.resolution_percentage = 100
        self._cage_state = snapshot_render_state()

    def warm_up(self) -> float:
        """Render a throwaway frame so the OptiX kernel compile lands before the port opens.

        The first Cycles frame costs about 1.3 s on an RTX 4080 Laptop; every later one is about
        50 ms. Paying it here keeps it off the user's first keypress.
        """
        started = time.monotonic()
        self._apply("pinhole", preview=True, alpha=1.0)
        scene = bpy.context.scene
        was = scene.render.resolution_percentage
        scene.render.resolution_percentage = 2
        self._render_to(PREVIEW_PATH)
        scene.render.resolution_percentage = was
        self._applied = None  # force a full property write on the first real frame
        return time.monotonic() - started

    # -- rendering -------------------------------------------------------------------------

    def render_preview(
        self, cam2world: np.ndarray, view: str, alpha: float = 1.0
    ) -> tuple[bytes, float, tuple[int, int]]:
        """(JPEG bytes, milliseconds, size) for one preview frame."""
        started = time.monotonic()
        setup = self._apply(view, preview=True, alpha=alpha)
        self.set_pose(cam2world)
        self._render_to(PREVIEW_PATH)
        jpeg = self._finish(setup, PREVIEW_PATH, quality=PREVIEW_QUALITY)
        return jpeg, (time.monotonic() - started) * 1000.0, setup.size

    def render_full(
        self, cam2world: np.ndarray, view: str, alpha: float, out_path: Path
    ) -> tuple[Path, float]:
        """Render at full resolution and the spec's own sample count, straight to a PNG."""
        started = time.monotonic()
        setup = self._apply(view, preview=False, alpha=alpha)
        self.set_pose(cam2world)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = out_path.with_suffix(".render.png")
        self._render_to(temporary)
        image = cv2.imread(str(temporary), cv2.IMREAD_COLOR)
        temporary.unlink(missing_ok=True)
        if image is None:
            raise RuntimeError(f"Blender wrote no image to {temporary}")
        cv2.imwrite(str(out_path), self._warp(setup, image))
        return out_path, (time.monotonic() - started) * 1000.0

    def set_pose(self, cam2world: np.ndarray) -> None:
        """Pose the camera by assigning `matrix_world`. No keyframes, ever."""
        camera = bpy.context.scene.camera
        _assert_no_keyframes(camera)
        camera.matrix_world = mathutils.Matrix(np.asarray(cam2world, dtype=np.float64).tolist())
        bpy.context.view_layer.update()

    # -- mode and view switching -----------------------------------------------------------

    def _apply(self, view: str, preview: bool, alpha: float) -> ViewSetup:
        # The distorted view is the raw sensor frame, which no rectification alpha touches, so it
        # shares one cache entry instead of paying the lens-distortion solve again per alpha.
        key = (view, preview, 1.0 if view == "distorted" else alpha)
        if self._applied == key:
            return self._setups[key]
        if self._cage_state is None:
            raise RuntimeError("capture_scene_state() must run before any render")

        # Put back the complete drift-prone set first, so nothing survives from the other mode.
        restore_render_state(self._cage_state)
        scene = bpy.context.scene
        self._write_mode_settings(scene, preview)
        setup = self._setups.get(key) or self._build_setup(view, preview, key[2])
        # Intrinsics are written on every transition, never only on the first: restore_render_state
        # just put the base camera back, and a cached setup would otherwise render at that base.
        bproc.camera.set_intrinsics_from_K_matrix(setup.intrinsics, *setup.intrinsics_size)
        scene.render.resolution_percentage = setup.resolution_percentage
        self._setups[key] = setup
        self._applied = key
        return setup

    def _write_mode_settings(self, scene: bpy.types.Scene, preview: bool) -> None:
        """Every mode-dependent property, written whole. Never a delta off the other mode."""
        scene.render.engine = "CYCLES"
        cycles = scene.cycles
        denoiser = self._spec.render.denoiser
        if preview:
            cycles.samples = PREVIEW_SAMPLES
            cycles.use_adaptive_sampling = False  # overhead with no payoff at 8 spp
            # Bounces stay at the spec's own values, not a cheaper preview set. The mat is lit
            # through the polycarbonate panels, so truncating transmission bounces makes the
            # preview systematically darker than the render it is supposed to be previewing.
            # Samples are the lever here; the denoiser is what buys back the noise.
            cycles.max_bounces = 12
            cycles.transmission_bounces = self._spec.render.transmission_bounces
            cycles.transparent_max_bounces = self._spec.render.transmission_bounces
            cycles.glossy_bounces = self._spec.render.glossy_bounces
            # The scene is static and only the camera moves, so the cache is pure win here. The
            # batch pipeline turns this off for the opposite reason: it mutates geometry per frame.
            cycles.use_persistent_data = True
            scene.render.image_settings.file_format = "JPEG"
            scene.render.image_settings.quality = PREVIEW_QUALITY
            # Denoising is not optional at 8 spp; it is what makes the frame legible. Go through
            # BlenderProc so the view-layer flag OPTIX also needs gets set.
            denoiser = denoiser if denoiser.lower() != "none" else "OPTIX"
        else:
            cycles.samples = self._spec.render.samples
            cycles.use_adaptive_sampling = True
            cycles.max_bounces = 12
            cycles.transmission_bounces = self._spec.render.transmission_bounces
            cycles.transparent_max_bounces = self._spec.render.transmission_bounces
            cycles.glossy_bounces = self._spec.render.glossy_bounces
            cycles.use_persistent_data = False  # a stale cache must never reach a saved artifact
            scene.render.image_settings.file_format = "PNG"
        bproc.renderer.set_denoiser(denoiser if denoiser.lower() != "none" else None)
        # The Intel denoiser runs as a compositor node, so compositing has to stay on for it.
        scene.render.use_compositing = denoiser.upper() == "INTEL"
        scene.render.image_settings.color_mode = "RGB"
        scene.render.film_transparent = False
        scene.render.use_sequencer = False

    def _build_setup(self, view: str, preview: bool, alpha: float) -> ViewSetup:
        size = self._preview_size if preview else self._render_size
        if view == "pinhole":
            # Scale with resolution_percentage, never by rescaling K: it is applied after
            # projection, so the preview frames identically to the full render by construction.
            return ViewSetup(
                name=view,
                size=size,
                intrinsics=self._k_rect[alpha],
                intrinsics_size=self._render_size,
                resolution_percentage=self._preview_percentage if preview else 100,
            )

        # Distorted views build the mapping at their own size, because set_lens_distortion
        # enlarges the render and its mapping indexes that enlarged frame in pixels.
        calibration = self._calibration.scaled(*size)
        bproc.camera.set_intrinsics_from_K_matrix(calibration.K, size[0], size[1])
        bpy.context.scene.render.resolution_percentage = 100
        k1, k2, p1, p2, k3 = calibration.distortion
        mapping = bproc.camera.set_lens_distortion(k1, k2, k3, p1, p2)
        enlarged = bproc.camera.get_intrinsics_as_K_matrix()
        enlarged_size = (
            int(bpy.context.scene.render.resolution_x),
            int(bpy.context.scene.render.resolution_y),
        )
        # BlenderProc applies this with scipy's map_coordinates, which costs ~100 ms at preview
        # size. The same mapping as an OpenCV remap is sub-millisecond and bilinear either way.
        map_y = np.ascontiguousarray(mapping[0].reshape(size[1], size[0]), dtype=np.float32)
        map_x = np.ascontiguousarray(mapping[1].reshape(size[1], size[0]), dtype=np.float32)
        logger.info(
            "%s view at %dx%d renders an enlarged %dx%d frame",
            view,
            size[0],
            size[1],
            *enlarged_size,
        )
        rectify = None
        if view == "rectified":
            map_rx, map_ry, _ = rectify_maps(self._calibration, size, alpha)
            rectify = (map_rx, map_ry)
        return ViewSetup(
            name=view,
            size=size,
            intrinsics=np.asarray(enlarged),
            intrinsics_size=enlarged_size,
            resolution_percentage=100,
            distortion_map=(map_x, map_y),
            rectify_map=rectify,
        )

    # -- pixels ----------------------------------------------------------------------------

    def _render_to(self, path: Path) -> None:
        scene = bpy.context.scene
        scene.render.filepath = str(path)
        with _suppress_stdout():  # Blender prints a Fra:/Mem: line per render call
            bpy.ops.render.render(write_still=True)

    def _finish(self, setup: ViewSetup, path: Path, quality: int) -> bytes:
        if setup.distortion_map is None and setup.rectify_map is None:
            return path.read_bytes()  # Blender's own JPEG, already color managed
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"Blender wrote no image to {path}")
        ok, encoded = cv2.imencode(
            ".jpg", self._warp(setup, image), [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        )
        if not ok:
            raise RuntimeError("failed to encode the preview frame as JPEG")
        return bytes(encoded)

    def _warp(self, setup: ViewSetup, image: np.ndarray) -> np.ndarray:
        if setup.distortion_map is not None:
            map_x, map_y = setup.distortion_map
            image = cv2.remap(
                image, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE
            )
        if setup.rectify_map is not None:
            map_x, map_y = setup.rectify_map
            # Constant black, not edge replication: the border is the point of this view.
            image = cv2.remap(
                image,
                map_x,
                map_y,
                cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=(0, 0, 0),
            )
        return image


class RobotField:
    """One imported robot, standing at every mat sample point via linked duplicates.

    The robot is 511 mesh parts. Importing it nine times builds 4599 objects with their own mesh
    data, which is enough to take the process down; `obj.copy()` shares the mesh datablock, so
    eight clones cost eight transforms and nothing else. Materials come along for free because
    they hang off the shared mesh.

    All nine share a heading, so the projected widths in the readout compare like for like. They
    are placed once, at construction: the sample points are fixed in the world, and moving
    geometry per frame would throw away Cycles' persistent-data cache.
    """

    def __init__(self, robot: Any, mat_size_m: float, ground_roll_deg: float) -> None:
        import math

        from synthgen.robots import compute_ground_z

        rotation = (math.radians(-90.0), math.radians(ground_roll_deg), 0.0)
        ground_z = compute_ground_z(robot.meshes, robot.parent, rotation)
        originals = [mesh.blender_obj for mesh in robot.meshes]
        points = list(mat_sample_points(mat_size_m).values())

        self._meshes: list[bpy.types.Object] = list(originals)
        self._parents: list[bpy.types.Object] = [robot.parent]
        for index in range(1, len(points)):
            parent = robot.parent.copy()
            bpy.context.scene.collection.objects.link(parent)
            self._parents.append(parent)
            for original in originals:
                clone = original.copy()  # shares original.data, so no mesh is duplicated
                bpy.context.scene.collection.objects.link(clone)
                clone.parent = parent
                clone.matrix_parent_inverse = original.matrix_parent_inverse.copy()
                self._meshes.append(clone)

        for parent, point in zip(self._parents, points):
            parent.location = mathutils.Vector((point[0], point[1], ground_z))
            parent.rotation_euler = mathutils.Euler(rotation)
        bpy.context.view_layer.update()
        logger.info(
            "robot field: %d stands built from %d shared mesh parts",
            len(self._parents),
            len(originals),
        )

    def set_visible(self, visible: bool) -> None:
        for obj in self._meshes:
            obj.hide_render = not visible
            obj.hide_viewport = not visible
        bpy.context.view_layer.update()


def _assert_no_keyframes(camera: bpy.types.Object) -> None:
    """Nothing in this tool inserts a keyframe, so any that appear would silently break posing."""
    if camera.animation_data is not None or camera.data.animation_data is not None:
        raise RuntimeError(
            "the camera picked up animation data; direct matrix_world assignment is being "
            "overwritten by the animation system"
        )


def clear_camera_keyframes(camera: bpy.types.Object) -> None:
    """Drop anything BlenderProc setup left on the camera, before the first pose is assigned."""
    camera.animation_data_clear()
    camera.data.animation_data_clear()


@contextlib.contextmanager
def _suppress_stdout() -> Iterator[None]:
    """Silence Blender's per-render chatter. Safe: `logsetup` writes to stderr."""
    sys.stdout.flush()
    saved = os.dup(1)
    devnull = os.open(os.devnull, os.O_WRONLY)
    try:
        os.dup2(devnull, 1)
        yield
    finally:
        os.dup2(saved, 1)
        os.close(devnull)
        os.close(saved)
