"""Run orchestration: scene loop, rendering, frame processing, and summary.

Order-of-operations invariants (do not reorder casually):

1. ``bproc.init()`` -> resolution -> samples -> ``enable_depth_output`` before
   anything renders.
2. ``enable_segmentation_output`` only after all mesh objects exist, because it
   assigns pass indices at call time; it is re-armed after every distractor
   pool refresh for the same reason.
3. Per scene: reset keyframes -> ground scale -> environment -> select/pose
   robots -> pool refresh check -> place distractors -> lights -> material
   jitter -> cameras -> render.
4. The clean (distractor-free) second render runs after the main render, only
   when obstructions are not ignored and distractors are active.
5. ``frame_set(local_idx)`` before per-frame keypoint projection: Blender's
   ``world_to_camera_view`` reads the camera pose animated at that frame.
"""

import argparse
import gc
import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import blenderproc as bproc
import bpy
import cv2
import numpy as np

from synthgen.annotations import (
    YoloAnnotation,
    YoloSegAnnotation,
    build_names_list,
    segmentation_annotations_from_segmap,
    write_data_yml,
    write_label_index,
    write_yolo_labels,
    write_yolo_seg_labels,
)
from synthgen.asset_index import (
    DistractorClassIdAssigner,
    OutputLayout,
    load_distractor_vram_audit,
    resolve_output_layout,
    resolve_start_index,
    setup_segmentation_labels,
)
from synthgen.camera import setup_scene_cameras
from synthgen.configuration import ConfigError, RenderConfig, load_render_config
from synthgen.constants import (
    BACKGROUND_CATEGORY_ID,
    MAX_CONSECUTIVE_FAILED_SCENES,
    MAX_SCENE_ATTEMPTS_FACTOR,
    NHRL_ROBOT_CLASS_NAME,
    PROGRESS_LOG_SCENE_INTERVAL,
    SEG_FLOOR_CLASS_ID,
)
from synthgen.distractors import (
    DistractorInstance,
    DistractorPoolManager,
    hide_distractor,
    place_scene_distractors,
)
from synthgen.environment import (
    create_ground_plane,
    create_lights,
    load_environment_assets,
    log_category_distribution,
    randomize_environment,
    randomize_lights,
)
from synthgen.gating import decide_keypoint_frame, decide_seg_frame
from synthgen.imaging import apply_object_motion_blur
from synthgen.keypoints import (
    build_distractor_keypoint_annotations,
    build_robot_keypoint_annotations,
)
from synthgen.logsetup import fmt_ctx, get_logger
from synthgen.materials import jitter_materials, load_cc_materials
from synthgen.reporting import DropReason, FrameVerdict, RunAnomaly, RunStats
from synthgen.robots import (
    RobotInstance,
    hide_all_robots,
    load_robots,
    pose_scene_robots,
    select_and_show_robots,
)

logger = get_logger(__name__)


@dataclass(frozen=True)
class AnnotationScheme:
    """Everything class-id related, fixed for the whole run."""

    mode: str
    is_segmentation: bool
    seg_robot_class_ids: dict[int, int]
    seg_label_names: dict[int, str]
    nhrl_class_id: int | None
    assigner: DistractorClassIdAssigner


@dataclass(frozen=True)
class RunBudget:
    """How many images to produce and how many scene attempts are allowed."""

    start_index: int
    num_images: int
    images_per_scene: int
    max_scenes: int

    @property
    def target_index(self) -> int:
        """First global index past the end of the run."""
        return self.start_index + self.num_images

    def remaining(self, global_idx: int) -> int:
        """Images still to produce at *global_idx*."""
        return self.target_index - global_idx


@dataclass
class SceneAssets:
    """Long-lived Blender objects reused across scenes."""

    robots: list[RobotInstance]
    ground: bproc.types.MeshObject
    lights: list[bproc.types.Light]
    hdri_paths: list[Any]
    cc_textures: list[bproc.types.Material]


@dataclass
class SceneState:
    """Per-scene arrangement produced by the randomization steps."""

    scene_idx: int
    arena_radius: float
    scene_robots: list[RobotInstance]
    robot_positions: list[list[float]]
    active_distractors: list[DistractorInstance]
    cam_count: int


@dataclass
class RenderResult:
    """Outcome of one scene attempt."""

    frames_written: int
    render_failed: bool = False


def build_annotation_scheme(cfg: RenderConfig) -> AnnotationScheme:
    """Derive the class-id scheme from the config.

    Raises:
        ConfigError: In keypoint mode when a robot entry lacks ``class_id``.
    """
    is_segmentation = cfg.output.is_segmentation_mode
    seg_robot_class_ids, seg_label_names, next_seg_class_id = setup_segmentation_labels(
        cfg.robots, is_segmentation
    )
    assigner = DistractorClassIdAssigner(is_segmentation, next_seg_class_id)

    # CAD (NHRL robot) distractors get keypoint annotations under a single
    # generic class, but only in keypoint mode and only when a CAD distractor
    # source is configured.
    nhrl_class_id: int | None = None
    if not is_segmentation:
        class_ids = []
        for i, rc in enumerate(cfg.robots):
            if rc.class_id is None:
                raise ConfigError(
                    f"robots[{i}] ({rc.name}): 'class_id' is required in keypoint mode"
                )
            class_ids.append(rc.class_id)
        if cfg.distractors.has_cad_source():
            nhrl_class_id = max(class_ids, default=-1) + 1

    return AnnotationScheme(
        mode=cfg.output.annotation_mode,
        is_segmentation=is_segmentation,
        seg_robot_class_ids=seg_robot_class_ids,
        seg_label_names=seg_label_names,
        nhrl_class_id=nhrl_class_id,
        assigner=assigner,
    )


def _enable_segmentation() -> None:
    """(Re-)arm BlenderProc's segmentation output for all current meshes."""
    bproc.renderer.enable_segmentation_output(
        map_by=["category_id", "robot_instance_id"],
        default_values={"category_id": BACKGROUND_CATEGORY_ID, "robot_instance_id": 0},
    )


def _write_keypoint_data_yml(
    cfg: RenderConfig, scheme: AnnotationScheme, layout: OutputLayout
) -> None:
    """Write data.yml for keypoint mode using class ids from the robot configs."""
    keypoint_label_names: dict[int, str] = {}
    for rcfg in cfg.robots:
        if rcfg.class_id is not None:
            keypoint_label_names.setdefault(rcfg.class_id, rcfg.name)
    if scheme.nhrl_class_id is not None:
        keypoint_label_names[scheme.nhrl_class_id] = NHRL_ROBOT_CLASS_NAME
    write_data_yml(
        layout.data_yml_path,
        layout.dataset_root,
        build_names_list(keypoint_label_names),
        scheme.mode,
    )


def _write_seg_metadata(scheme: AnnotationScheme, layout: OutputLayout) -> None:
    """Write the segmentation label index and data.yml."""
    write_label_index(layout.label_dir / "label_index.txt", scheme.seg_label_names)
    write_data_yml(
        layout.data_yml_path,
        layout.dataset_root,
        build_names_list(scheme.seg_label_names),
        scheme.mode,
    )


def _refresh_pool(
    pool_mgr: DistractorPoolManager, scheme: AnnotationScheme, layout: OutputLayout
) -> None:
    """Refresh the distractor pool and rewrite the seg metadata it invalidates."""
    pool_mgr.refresh()
    if scheme.is_segmentation:
        _write_seg_metadata(scheme, layout)


def _save_debug_frame(data: dict, layout: OutputLayout) -> None:
    """Save the first rendered frame of the first scene as a debug image."""
    logger.info("Render data keys: %s", list(data.keys()))
    debug_img = data["colors"][0]
    debug_path = str(layout.image_dir / "_debug_frame0.jpg")
    cv2.imwrite(debug_path, cv2.cvtColor(debug_img, cv2.COLOR_RGB2BGR))
    logger.info("Saved debug image: %s", debug_path)


def _render_clean_inst_seg_maps(
    inst_seg_maps: Any, active_distractors: list[DistractorInstance]
) -> Any:
    """Render an occlusion-free instance segmentation pass with distractors hidden."""
    if inst_seg_maps is None:
        return None
    if not active_distractors:
        return inst_seg_maps

    saved_distractor_world = [d.parent.matrix_world.copy() for d in active_distractors]
    for distractor in active_distractors:
        hide_distractor(distractor)
    bpy.context.view_layer.update()

    clean_data = bproc.renderer.render()
    clean_inst_seg_maps = clean_data.get("robot_instance_id_segmaps")

    for distractor, mat in zip(active_distractors, saved_distractor_world):
        distractor.parent.matrix_world = mat
    bpy.context.view_layer.update()
    return clean_inst_seg_maps


def _compute_blur_category_ids(
    scene_robots: list[RobotInstance], active_distractors: list[DistractorInstance]
) -> tuple[set[int], set[int]]:
    """Collect blur-eligible category ids, split into (robot, distractor)."""
    robot_ids: set[int] = set()
    for robot in scene_robots:
        if robot.meshes:
            robot_ids.add(int(robot.meshes[0].blender_obj.get("category_id", -1)))
    distractor_ids: set[int] = set()
    for distractor in active_distractors:
        if distractor.meshes:
            distractor_ids.add(int(distractor.meshes[0].blender_obj.get("category_id", -1)))
    robot_ids.discard(-1)
    distractor_ids.discard(-1)
    # A category shared with a robot follows the robot blur probability.
    distractor_ids -= robot_ids
    return robot_ids, distractor_ids


def _apply_frame_motion_blur(
    color_img: np.ndarray,
    cat_seg: np.ndarray,
    blur_category_ids: tuple[set[int], set[int]],
    cfg: RenderConfig,
) -> np.ndarray:
    """Apply per-category motion blur to a rendered frame and return the result.

    ``blur_category_ids`` is ``(robot_ids, distractor_ids)``; robots use the
    ``[randomization]`` motion-blur probability and distractors use the
    ``[distractors]`` one (falling back to the robot probability).
    """
    robot_ids, distractor_ids = blur_category_ids
    robot_prob = cfg.randomization.motion_blur_probability
    dist_prob = (
        cfg.distractors.motion_blur_probability
        if cfg.distractors.motion_blur_probability is not None
        else robot_prob
    )
    blur_lo, blur_hi = cfg.randomization.motion_blur_strength_range
    for ids, prob in ((robot_ids, robot_prob), (distractor_ids, dist_prob)):
        for blur_cid in sorted(ids):
            if random.random() < prob:
                color_img = apply_object_motion_blur(
                    color_img,
                    cat_seg,
                    blur_cid,
                    random.randint(blur_lo, blur_hi),
                    random.uniform(0, 2 * math.pi),
                )
    return color_img


def _extract_frame_annotations(
    cfg: RenderConfig,
    scheme: AnnotationScheme,
    scene: SceneState,
    local_idx: int,
    cat_seg: np.ndarray,
    inst_seg: np.ndarray | None,
    clean_inst_seg: np.ndarray | None,
    depth_map: np.ndarray,
    stats: RunStats,
) -> tuple[list[YoloAnnotation], list[YoloSegAnnotation], FrameVerdict]:
    """Extract annotations for one frame and decide whether to keep it."""
    img_w, img_h = cfg.output.image_width, cfg.output.image_height
    keypoint_annotations: list[YoloAnnotation] = []
    seg_annotations: list[YoloSegAnnotation] = []

    if scheme.is_segmentation:
        visible_category_ids = {
            int(v) for v in np.unique(cat_seg.squeeze()).tolist() if int(v) > BACKGROUND_CATEGORY_ID
        }
        annotated_category_ids: set[int] = set()
        # Skip the (comparatively expensive) contour extraction for floor-only
        # frames; decide_seg_frame reports them as SEG_FLOOR_ONLY regardless.
        if visible_category_ids != {SEG_FLOOR_CLASS_ID}:
            seg_annotations = segmentation_annotations_from_segmap(
                cat_seg, img_w, img_h, min_bbox_dim=cfg.output.segmentation_min_bbox_dim
            )
            annotated_category_ids = {class_id for class_id, _ in seg_annotations}
        decision = decide_seg_frame(
            visible_category_ids,
            annotated_category_ids,
            set(scheme.seg_robot_class_ids.values()),
        )
        verdict = FrameVerdict(
            scene_idx=scene.scene_idx,
            frame_in_scene=local_idx,
            written=decision.keep,
            drop_reason=decision.drop_reason,
            detail=decision.detail,
        )
        return keypoint_annotations, seg_annotations, verdict

    keypoint_annotations, gate_verdicts = build_robot_keypoint_annotations(
        scene.scene_robots,
        cat_seg,
        inst_seg,
        clean_inst_seg,
        depth_map,
        img_w,
        img_h,
        cfg.output.min_robot_visibility,
        cfg.output.ignore_obstructions,
    )
    if scheme.nhrl_class_id is not None:
        keypoint_annotations.extend(
            build_distractor_keypoint_annotations(
                scene.active_distractors,
                inst_seg,
                depth_map,
                img_w,
                img_h,
                scheme.nhrl_class_id,
                ignore_occlusion=cfg.output.ignore_obstructions,
                stats=stats,
            )
        )
    decision = decide_keypoint_frame(gate_verdicts, len(keypoint_annotations))
    robot_skips = {
        v.stats.instance_id: v.skip_reason for v in gate_verdicts if v.skip_reason is not None
    }
    verdict = FrameVerdict(
        scene_idx=scene.scene_idx,
        frame_in_scene=local_idx,
        written=decision.keep,
        drop_reason=decision.drop_reason,
        robot_skips=robot_skips,
        detail=decision.detail,
    )
    return keypoint_annotations, seg_annotations, verdict


def _process_scene_frames(
    cfg: RenderConfig,
    scheme: AnnotationScheme,
    layout: OutputLayout,
    scene: SceneState,
    data: dict,
    clean_inst_seg_maps: Any,
    stats: RunStats,
    start_global_idx: int,
) -> int:
    """Extract annotations, apply motion blur, and write images+labels.

    Returns:
        The number of frames written.
    """
    colors = data["colors"]
    cat_seg_maps = data.get("category_id_segmaps", data.get("segmap"))
    if cat_seg_maps is None:
        raise RuntimeError("segmentation maps missing; render_scene must check before calling")
    inst_seg_maps = data.get("robot_instance_id_segmaps")
    depth_maps = data["depth"]
    blur_category_ids = _compute_blur_category_ids(scene.scene_robots, scene.active_distractors)

    global_idx = start_global_idx
    for local_idx in range(scene.cam_count):
        bpy.context.scene.frame_set(local_idx)

        color_img = colors[local_idx]
        cat_seg = cat_seg_maps[local_idx]
        # An absent instance pass may surface as None or an empty list.
        inst_seg = (
            inst_seg_maps[local_idx]
            if inst_seg_maps is not None and len(inst_seg_maps) > 0
            else None
        )
        clean_inst_seg = clean_inst_seg_maps[local_idx] if clean_inst_seg_maps is not None else None
        depth_map = depth_maps[local_idx]

        if scene.scene_idx == 0 and local_idx == 0:
            logger.info(
                "Cat segmap shape=%s, unique=%s", cat_seg.shape, np.unique(cat_seg).tolist()
            )
            if inst_seg is not None:
                logger.info("Inst segmap unique=%s", np.unique(inst_seg).tolist())

        keypoint_annotations, seg_annotations, verdict = _extract_frame_annotations(
            cfg, scheme, scene, local_idx, cat_seg, inst_seg, clean_inst_seg, depth_map, stats
        )

        for instance_id, reason in verdict.robot_skips.items():
            logger.debug(
                "%s: robot %d skipped (%s)",
                fmt_ctx(scene.scene_idx, local_idx),
                instance_id,
                reason.name,
            )

        if not verdict.written:
            reason_name = verdict.drop_reason.name if verdict.drop_reason else "unknown"
            logger.info(
                "%s: DROPPED %s — %s",
                fmt_ctx(scene.scene_idx, local_idx),
                reason_name,
                verdict.detail or "no detail",
            )
            stats.record(verdict)
            continue

        color_img = _apply_frame_motion_blur(color_img, cat_seg, blur_category_ids, cfg)

        frame_name = f"{global_idx:06d}"
        if scheme.is_segmentation:
            write_yolo_seg_labels(layout.label_dir / f"{frame_name}.txt", seg_annotations)
        else:
            write_yolo_labels(layout.label_dir / f"{frame_name}.txt", keypoint_annotations)
        cv2.imwrite(
            str(layout.image_dir / f"{frame_name}.jpg"),
            cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR),
        )
        verdict.global_idx = global_idx
        stats.record(verdict)
        global_idx += 1
    return global_idx - start_global_idx


def render_scene(
    cfg: RenderConfig,
    assets: SceneAssets,
    scheme: AnnotationScheme,
    layout: OutputLayout,
    pool_mgr: DistractorPoolManager,
    budget: RunBudget,
    stats: RunStats,
    scene_idx: int,
    global_idx: int,
) -> RenderResult:
    """Arrange, render, and write one scene."""
    bproc.utility.reset_keyframes()

    # -- Per-scene randomized dimensions --
    ground_size = random.uniform(*cfg.scene.ground_size_range)
    arena_radius = random.uniform(*cfg.scene.arena_radius_range)
    assets.ground.blender_obj.scale = (ground_size, ground_size, 1)
    bpy.context.view_layer.update()

    randomize_environment(assets.ground, assets.hdri_paths, assets.cc_textures, cfg.scene)

    scene_robots = select_and_show_robots(assets.robots, cfg.scene.max_robots_per_scene)
    robot_positions = pose_scene_robots(scene_robots, cfg.randomization, arena_radius)
    bpy.context.view_layer.update()

    if pool_mgr.refresh_due():
        _refresh_pool(pool_mgr, scheme, layout)

    active_distractors = place_scene_distractors(
        pool_mgr.pool, cfg.distractors, cfg.randomization, arena_radius
    )

    randomize_lights(assets.lights, cfg.randomization)

    for robot in scene_robots:
        jitter_materials(
            robot.meshes, cfg.randomization.roughness_jitter, cfg.randomization.hue_jitter_degrees
        )

    # -- Camera poses (look at centroid of all placed robots) --
    cam_poses, cam_count, fallback_count = setup_scene_cameras(
        scene_robots,
        cfg.camera,
        cfg.output.images_per_scene,
        budget.remaining(global_idx),
        active_distractors,
        robot_positions,
        cfg.output.ignore_obstructions,
    )
    for _ in range(fallback_count):
        stats.record_anomaly(RunAnomaly.CAMERA_TARGET_FALLBACK)
    for pose in cam_poses:
        bproc.camera.add_camera_pose(pose)

    data = bproc.renderer.render()

    if scene_idx == 0:
        _save_debug_frame(data, layout)

    # Check for segmentation maps BEFORE the expensive clean second render.
    cat_seg_maps = data.get("category_id_segmaps", data.get("segmap"))
    if cat_seg_maps is None:
        logger.error("%s: no segmentation maps in render output", fmt_ctx(scene_idx))
        for local_idx in range(cam_count):
            stats.record(
                FrameVerdict(
                    scene_idx=scene_idx,
                    frame_in_scene=local_idx,
                    written=False,
                    drop_reason=DropReason.SCENE_NO_SEGMAPS,
                )
            )
        return RenderResult(frames_written=0, render_failed=True)

    # True occlusion metric: visible robot pixels (with distractors) divided
    # by unobstructed robot pixels from a second pass with distractors hidden.
    # Skipped entirely when obstructions are ignored (the gate is bypassed).
    inst_seg_maps = data.get("robot_instance_id_segmaps")
    clean_inst_seg_maps = (
        None
        if cfg.output.ignore_obstructions
        else _render_clean_inst_seg_maps(inst_seg_maps, active_distractors)
    )

    scene = SceneState(
        scene_idx=scene_idx,
        arena_radius=arena_radius,
        scene_robots=scene_robots,
        robot_positions=robot_positions,
        active_distractors=active_distractors,
        cam_count=cam_count,
    )
    written = _process_scene_frames(
        cfg, scheme, layout, scene, data, clean_inst_seg_maps, stats, global_idx
    )

    completed = scene_idx + 1
    if completed % PROGRESS_LOG_SCENE_INTERVAL == 0:
        names = [r.name for r in scene_robots]
        logger.info(
            "Scene %d (%s) — %d/%d images generated",
            completed,
            names,
            global_idx + written - budget.start_index,
            budget.num_images,
        )
    return RenderResult(frames_written=written)


def _periodic_memory_cleanup(completed_scenes: int, memory_cleanup_interval: int) -> None:
    """Free transient render buffers/images at the configured scene interval."""
    if not (memory_cleanup_interval > 0 and completed_scenes % memory_cleanup_interval == 0):
        return
    gc.collect()
    # Free transient render buffers/images without touching materials.
    # Avoid orphans_purge here: it can delete cached cc_textures that are
    # intentionally kept for future scenes but temporarily have 0 users.
    for img in list(bpy.data.images):
        if img.name.startswith(("Render Result", "Viewer Node")):
            try:
                img.buffers_free()
            except RuntimeError as e:
                logger.debug("buffers_free failed for %s: %s", img.name, e)
            if img.users == 0:
                bpy.data.images.remove(img)


def run(args: argparse.Namespace) -> None:
    """Execute a full rendering run from parsed CLI arguments."""
    cfg = load_render_config(Path(args.config))
    scheme = build_annotation_scheme(cfg)

    num_images = args.num_images if args.num_images is not None else cfg.output.num_images
    layout = resolve_output_layout(cfg.output, cfg.resolver.resolve)
    start_index = resolve_start_index(args.start_index, layout.image_dir)
    budget = RunBudget(
        start_index=start_index,
        num_images=num_images,
        images_per_scene=cfg.output.images_per_scene,
        max_scenes=math.ceil(num_images / cfg.output.images_per_scene) * MAX_SCENE_ATTEMPTS_FACTOR,
    )

    # ------- Initialize BlenderProc -------

    bproc.init()
    bproc.camera.set_resolution(cfg.output.image_width, cfg.output.image_height)
    bproc.renderer.set_max_amount_of_samples(args.render_samples)
    bproc.renderer.enable_depth_output(activate_antialiasing=False)
    if hasattr(bpy.context.scene, "cycles"):
        # Prevent Cycles from keeping render data across frames/scenes.
        bpy.context.scene.cycles.use_persistent_data = False

    # ------- Load target robots -------

    cc_materials = load_cc_materials(
        cfg.materials, cfg.environment.cc_textures_dir, cfg.resolver.resolve
    )
    robots = load_robots(
        cfg.robots,
        cfg.materials,
        cc_materials,
        scheme.is_segmentation,
        scheme.seg_robot_class_ids,
        cfg.resolver.resolve,
    )
    logger.info("%d robot model(s) loaded: %s", len(robots), [r.name for r in robots])
    hide_all_robots(robots)

    if not scheme.is_segmentation:
        _write_keypoint_data_yml(cfg, scheme, layout)

    # ------- Load distractors and environment assets -------

    vram_estimates = load_distractor_vram_audit(
        cfg.distractors.vram_audit_csv, cfg.resolver.resolve
    )
    hdri_paths, cc_textures = load_environment_assets(cfg.environment, cfg.resolver.resolve)
    ground = create_ground_plane(scheme.is_segmentation)

    # Enable segmentation AFTER all mesh objects are in the scene, because
    # enable_segmentation_output assigns pass_index to every mesh at call time.
    _enable_segmentation()

    lights = create_lights(cfg.randomization.light_count_range[1])
    log_category_distribution(scheme.is_segmentation, scheme.seg_label_names)

    assets = SceneAssets(
        robots=robots, ground=ground, lights=lights, hdri_paths=hdri_paths, cc_textures=cc_textures
    )
    pool_mgr = DistractorPoolManager(
        cfg.distractors,
        scheme.assigner,
        cfg.resolver.resolve,
        load_keypoints=scheme.nhrl_class_id is not None,
        enable_segmentation=_enable_segmentation,
        vram_estimates=vram_estimates,
    )
    _refresh_pool(pool_mgr, scheme, layout)

    # ------- Render loop -------

    stats = RunStats()
    global_idx = start_index
    scene_idx = 0
    consecutive_failures = 0
    logger.info("Rendering %d images...", num_images)

    try:
        while global_idx < budget.target_index and scene_idx < budget.max_scenes:
            result = render_scene(
                cfg, assets, scheme, layout, pool_mgr, budget, stats, scene_idx, global_idx
            )
            global_idx += result.frames_written
            pool_mgr.note_images_written(result.frames_written)
            scene_idx += 1
            if result.render_failed:
                consecutive_failures += 1
                stats.record_anomaly(RunAnomaly.SCENE_RENDER_FAILED)
                if consecutive_failures >= MAX_CONSECUTIVE_FAILED_SCENES:
                    raise RuntimeError(
                        f"{consecutive_failures} consecutive scenes produced no segmentation "
                        "maps; aborting. Check the BlenderProc segmentation setup."
                    )
            else:
                consecutive_failures = 0
            _periodic_memory_cleanup(scene_idx, cfg.output.memory_cleanup_interval)
    finally:
        written = global_idx - start_index
        if written < num_images:
            stats.record_anomaly(RunAnomaly.RUN_SHORTFALL)
            logger.warning(
                "Run ended %d images short: %d/%d written after %d scene attempts "
                "(max_scenes=%d). See drop reasons below.",
                num_images - written,
                written,
                num_images,
                scene_idx,
                budget.max_scenes,
            )
        for line in stats.summary_lines(num_images, written, scene_idx):
            logger.info("%s", line)

    logger.info("Done. Generated %d images in %s", written, layout.image_dir)
