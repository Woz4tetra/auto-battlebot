#!/usr/bin/env python3
import blenderproc as bproc  # noqa: F401  # isort: skip  (blenderproc must be imported first)

# BlenderProc insists its import is the first statement, so the module docstring follows it.
DESCRIPTION = """
Render labelled robot samples inside the NHRL cage from the fixed cage-high camera.

The cage, lights, camera intrinsics and fitted poses are the ones render_cage_view.py grades
against footage; the robots, Meshy opponents, gating and YOLO pose labels are the same
synthgen pieces the generic pipeline uses, so the output is a drop-in dataset:

    <out>/images/000000.jpg ...   <out>/labels/000000.txt ...   <out>/data.yml   <out>/sheet.png

Per image one target robot (MRS BUFF MK3 by default) plus 1 to 2 Meshy opponents are placed
on the mat, the camera takes one of the fitted event poses, and the LED tube strength is
jittered so the set is not a single lighting state.

    training/synthetic/docker/run_synthetic.sh --gpu auto-battlebot-synthetic blenderproc run \\
        training/synthetic/render_cage_samples.py -- \\
        --spec training/synthetic/cage/cage2_overhead_high.toml \\
        --config training/synthetic/config.toml \\
        --poses /workspace/runs/cage_scene/poses \\
        --camera-rect /workspace/runs/cage_scene/camera_rect.json \\
        --out /workspace/runs/cage_scene/samples --num-images 100 --samples 128 --seed 0
"""

import argparse
import json
import random
import sys
from collections.abc import Callable
from dataclasses import replace
from pathlib import Path
from typing import Any

import bpy
import cv2
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
sys.path.insert(0, str(SCRIPT_DIR))
sys.path.insert(0, str(REPO_ROOT))

from synthgen import logsetup  # noqa: E402
from synthgen.asset_index import load_distractor_vram_audit, resolve_output_layout  # noqa: E402
from synthgen.cage import (  # noqa: E402
    add_lights,
    build_cage,
    house_bot_keypoints,
    set_camera,
    set_color_management,
    set_led_emission,
    set_world,
    solve_exposure,
)
from synthgen.cage_scene import HouseBot  # noqa: E402
from synthgen.cage_spec import (  # noqa: E402
    all_tubes,
    load_cage_spec,
    pit_rects,
    spec_to_dict,
)
from synthgen.configuration import load_render_config  # noqa: E402
from synthgen.constants import BACKGROUND_CATEGORY_ID  # noqa: E402
from synthgen.distractors import DistractorPoolManager, place_scene_distractors  # noqa: E402
from synthgen.materials import load_cc_materials  # noqa: E402
from synthgen.pipeline import (  # noqa: E402
    SceneState,
    _enable_segmentation,
    _process_scene_frames,
    _refresh_pool,
    _render_clean_inst_seg_maps,
    _write_keypoint_data_yml,
    build_annotation_scheme,
)
from synthgen.reporting import RunAnomaly, RunStats  # noqa: E402
from synthgen.robots import (  # noqa: E402
    hide_all_robots,
    load_robots,
    pose_scene_robots,
    select_and_show_robots,
)

from auto_battlebot.perception.cage_calibration import (  # noqa: E402
    blender_cam2world,
    load_cage_calibration,
)

logger = logsetup.get_logger("render_cage_samples")

DEFAULT_CC_TEXTURES = REPO_ROOT / "training/data/cc_textures"
MAX_SCENE_ATTEMPTS_FACTOR = 3
# Redraws allowed before a scene is accepted with something standing over a pit.
PLACEMENT_ATTEMPTS = 20


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=DESCRIPTION, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--spec", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=SCRIPT_DIR / "config.toml")
    parser.add_argument(
        "--poses", type=Path, required=True, help="directory of CageCalibration TOMLs"
    )
    parser.add_argument("--pose-glob", default="cage2_*.toml")
    parser.add_argument("--camera-rect", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--num-images", type=int, default=100)
    parser.add_argument("--samples", type=int, default=128)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    parser.add_argument(
        "--robots", nargs="+", default=["MRS_BUFF_MK3"], help="[[robots]] names to keep"
    )
    parser.add_argument("--opponents", type=int, nargs=2, default=[1, 2], metavar=("MIN", "MAX"))
    parser.add_argument("--air-probability", type=float, default=0.1)
    parser.add_argument(
        "--tube-jitter", type=float, default=0.25, help="+-fraction on tube strength"
    )
    parser.add_argument(
        "--mat-margin", type=float, default=0.2, help="metres kept clear of the mat edge"
    )
    parser.add_argument(
        "--pit-margin",
        type=float,
        default=0.12,
        help="metres of clearance kept around a pit opening when placing robots",
    )
    parser.add_argument("--cc-textures", type=Path, default=DEFAULT_CC_TEXTURES)
    parser.add_argument(
        "--auto-exposure",
        type=Path,
        default=None,
        metavar="TARGETS_DIR",
        help="solve exposure against the target frames before rendering, as render_cage_view does",
    )
    parser.add_argument("-v", "--verbose", action="store_true")
    return parser.parse_args()


def load_k(path: Path) -> tuple[np.ndarray, int, int]:
    data = json.loads(path.read_text())
    k = np.array([[data["fx"], 0.0, data["cx"]], [0.0, data["fy"], data["cy"]], [0.0, 0.0, 1.0]])
    return k, int(data["width"]), int(data["height"])


def jitter_tubes(tubes: list[Any], base: list[float], fraction: float) -> None:
    for tube, strength in zip(tubes, base):
        node = tube.get_materials()[0].get_the_one_node_with_type("BsdfPrincipled")
        node.inputs["Emission Strength"].default_value = strength * random.uniform(
            1 - fraction, 1 + fraction
        )


def contact_sheet(image_dir: Path, out: Path, columns: int = 5, rows: int = 4) -> None:
    images = sorted(image_dir.glob("*.jpg"))[: columns * rows]
    if not images:
        return
    thumbs = []
    for path in images:
        img = cv2.imread(str(path))
        if img is not None:
            thumbs.append(cv2.resize(img, (384, 216), interpolation=cv2.INTER_AREA))
    if not thumbs:
        return
    while len(thumbs) < columns * rows:
        thumbs.append(np.zeros_like(thumbs[0]))
    grid = np.vstack([np.hstack(thumbs[r * columns : (r + 1) * columns]) for r in range(rows)])
    cv2.imwrite(str(out), grid)


def _house_bot_record() -> HouseBot | None:
    """The placed house bot, so it is labelled here the way the generic pipeline labels it."""
    parent = bpy.data.objects.get("house_bot")
    keypoints = None if parent is None else house_bot_keypoints(parent)
    if keypoints is None:
        logger.warning("no house bot in the scene; it will go unlabelled")
        return None
    return HouseBot(parent, *keypoints)


def _keepouts(spec: Any, margin: float) -> list[tuple[float, float, float, float]]:
    """Pit openings grown by `margin`: nothing may be placed standing over a hole."""
    return [
        (x0 - margin, x1 + margin, y0 - margin, y1 + margin) for x0, x1, y0, y1 in pit_rects(spec)
    ]


def _over_a_pit(parents: list[Any], keepouts: list[tuple[float, float, float, float]]) -> bool:
    return any(
        x0 < float(parent.location.x) < x1 and y0 < float(parent.location.y) < y1
        for parent in parents
        for x0, x1, y0, y1 in keepouts
    )


def _place_clear_of_pits(
    place: Callable[[], Any],
    parents_of: Callable[[Any], list[Any]],
    keepouts: list[tuple[float, float, float, float]],
) -> Any:
    """Redraw a placement until nothing stands over a hole, then take what it gave."""
    placed = place()
    for _ in range(PLACEMENT_ATTEMPTS - 1):
        if not _over_a_pit(parents_of(placed), keepouts):
            break
        placed = place()
    return placed


def build_config(args: argparse.Namespace, width: int, height: int) -> Any:
    """The generic render config, narrowed to this camera, output dir and robot set."""
    cfg = load_render_config(args.config)
    robots_cfg = tuple(r for r in cfg.robots if r.name in args.robots)
    if not robots_cfg:
        raise SystemExit(
            f"none of {args.robots} in {args.config}; have {[r.name for r in cfg.robots]}"
        )
    return replace(
        cfg,
        robots=robots_cfg,
        output=replace(
            cfg.output,
            image_dir=args.out / "images",
            label_dir=args.out / "labels",
            num_images=args.num_images,
            image_width=width,
            image_height=height,
            images_per_scene=1,
        ),
        distractors=replace(
            cfg.distractors, min_per_scene=args.opponents[0], max_per_scene=args.opponents[1]
        ),
        randomization=replace(cfg.randomization, air_probability=args.air_probability),
    )


def configure_renderer(spec: Any, samples: int) -> None:
    bproc.renderer.set_output_format(file_format="PNG", view_transform="Standard")
    bproc.renderer.set_max_amount_of_samples(samples)
    bproc.renderer.set_denoiser(
        spec.render.denoiser if spec.render.denoiser.lower() != "none" else None
    )
    bproc.renderer.set_light_bounces(
        transmission_bounces=spec.render.transmission_bounces,
        glossy_bounces=spec.render.glossy_bounces,
        transparent_max_bounces=spec.render.transmission_bounces,
    )
    bproc.renderer.enable_depth_output(activate_antialiasing=False)
    if hasattr(bpy.context.scene, "cycles"):
        bpy.context.scene.cycles.use_persistent_data = False
    set_color_management(spec.exposure.gain)


def main() -> None:
    args = parse_args()
    logsetup.configure(1 if args.verbose else 0)
    random.seed(args.seed)
    np.random.seed(args.seed)

    spec = load_cage_spec(args.spec, args.set)
    k_rect, width, height = load_k(args.camera_rect)
    cfg = build_config(args, width, height)
    scheme = build_annotation_scheme(cfg)
    layout = resolve_output_layout(cfg.output, cfg.resolver.resolve)
    pose_paths = sorted(args.poses.glob(args.pose_glob))
    if not pose_paths:
        raise SystemExit(f"no poses matching {args.pose_glob} under {args.poses}")
    cam2worlds = [
        blender_cam2world(load_cage_calibration(p).tf_camera_from_fieldcenter) for p in pose_paths
    ]

    bproc.init()
    configure_renderer(spec, args.samples)

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

    cc_dir = args.cc_textures if args.cc_textures.exists() else None
    color_gain = tuple(float(v) for v in spec.exposure.color_gain)
    build_cage(spec, REPO_ROOT, cc_dir, mat_category_id=BACKGROUND_CATEGORY_ID)
    set_led_emission(spec, spec.exposure.gain)
    lights = add_lights(spec, color_gain)  # type: ignore[arg-type]
    set_world(spec, color_gain)  # type: ignore[arg-type]
    tubes = [light for light in lights if not hasattr(light, "set_color")]
    house_bot = _house_bot_record()
    _enable_segmentation()
    exposure_note: dict[str, Any] = {}
    if args.auto_exposure is not None:
        exposure_note = solve_exposure(
            spec,
            pose_paths,
            lights,
            color_gain,
            k_rect,
            width,
            height,
            args.samples,
            args.auto_exposure,
        )
    base_strengths = [
        float(
            t.get_materials()[0]
            .get_the_one_node_with_type("BsdfPrincipled")
            .inputs["Emission Strength"]
            .default_value
        )
        for t in tubes
    ]

    vram_estimates = load_distractor_vram_audit(
        cfg.distractors.vram_audit_csv, cfg.resolver.resolve
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

    arena_radius = spec.mat.size / 2 - args.mat_margin
    keepouts = _keepouts(spec, args.pit_margin)
    stats = RunStats()
    global_idx = 0
    scene_idx = 0
    max_scenes = args.num_images * MAX_SCENE_ATTEMPTS_FACTOR
    try:
        while global_idx < args.num_images and scene_idx < max_scenes:
            bproc.utility.reset_keyframes()
            scene_robots = select_and_show_robots(robots, len(robots))
            positions = _place_clear_of_pits(
                lambda: pose_scene_robots(scene_robots, cfg.randomization, arena_radius),
                lambda _: [robot.parent for robot in scene_robots],
                keepouts,
            )
            bpy.context.view_layer.update()
            if pool_mgr.refresh_due():
                _refresh_pool(pool_mgr, scheme, layout)
            active = _place_clear_of_pits(
                lambda: place_scene_distractors(
                    pool_mgr.pool, cfg.distractors, cfg.randomization, arena_radius
                ),
                lambda placed: [distractor.parent for distractor in placed],
                keepouts,
            )
            jitter_tubes(tubes, base_strengths, args.tube_jitter)
            set_camera(k_rect, width, height, random.choice(cam2worlds), frame=0)

            data = bproc.renderer.render()
            if not data.get("category_id_segmaps"):
                stats.record_anomaly(RunAnomaly.SCENE_RENDER_FAILED)
                scene_idx += 1
                continue
            inst_seg_maps = data.get("robot_instance_id_segmaps")
            clean = (
                None
                if cfg.output.ignore_obstructions
                else _render_clean_inst_seg_maps(inst_seg_maps, active)
            )
            scene = SceneState(
                scene_idx,
                arena_radius,
                scene_robots,
                positions,
                active,
                cam_count=1,
                house_bot=house_bot,
            )
            written = _process_scene_frames(
                cfg, scheme, layout, scene, data, clean, stats, global_idx
            )
            global_idx += written
            pool_mgr.note_images_written(written)
            scene_idx += 1
            if scene_idx % 10 == 0:
                logger.info("%d/%d images after %d scenes", global_idx, args.num_images, scene_idx)
    finally:
        for line in stats.summary_lines(args.num_images, global_idx, scene_idx):
            logger.info("%s", line)

    contact_sheet(layout.image_dir, args.out / "sheet.png")
    (args.out / "render_meta.json").write_text(
        json.dumps(
            {
                "spec_file": str(args.spec),
                "overrides": args.set,
                "spec": spec_to_dict(spec),
                "auto_exposure": exposure_note,
                "tubes": len(all_tubes(spec)),
                "config": str(args.config),
                "robots": [r.name for r in robots],
                "opponents_per_image": args.opponents,
                "poses": [str(p) for p in pose_paths],
                "K": k_rect.tolist(),
                "width": width,
                "height": height,
                "samples": args.samples,
                "seed": args.seed,
                "images_written": global_idx,
                "scenes_attempted": scene_idx,
            },
            indent=2,
        )
        + "\n"
    )
    logger.info("Done. %d images in %s", global_idx, layout.image_dir)


if __name__ == "__main__":
    main()
