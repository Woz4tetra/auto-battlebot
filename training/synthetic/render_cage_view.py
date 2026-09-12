#!/usr/bin/env python3
import blenderproc as bproc  # noqa: F401  # isort: skip  (blenderproc must be imported first)

# BlenderProc insists its import is the first statement, so the module docstring follows it.
DESCRIPTION = """
Render the empty NHRL cage from fitted cage-high camera poses, for grading against footage.

Runs under `blenderproc run` inside the auto-battlebot-synthetic image:

    training/synthetic/docker/run_synthetic.sh --gpu auto-battlebot-synthetic blenderproc run \\
        training/synthetic/render_cage_view.py -- \\
        --spec training/synthetic/cage/cage2_overhead_high.toml \\
        --pose /workspace/runs/cage_scene/poses \\
        --camera-rect /workspace/runs/cage_scene/camera_rect.json \\
        --out /workspace/runs/cage_scene/renders/r001 --samples 64 \\
        --save-blend /workspace/runs/cage_scene/cage2_overhead_high.blend

One PNG per pose file, named after the pose stem, plus `<stem>_matmask.png` (the mat's
segmentation) and `render_meta.json` (the spec after `--set` overrides, K, git sha).
`--auto-exposure TARGETS` solves the exposure gain and colour gain from the mat-region means of
the matching target frames before the final render.
"""


import argparse
import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Any

import bpy
import cv2
import mathutils
import numpy as np

# Blender's embedded Python ignores PYTHONPATH, so put the script's own directory and the repo
# root on sys.path here: synthgen for the cage, auto_battlebot.perception for the frame chain.
SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
sys.path.insert(0, str(SCRIPT_DIR))
sys.path.insert(0, str(REPO_ROOT))

from synthgen import logsetup  # noqa: E402
from synthgen.cage import (  # noqa: E402
    MAT_CATEGORY_ID,
    add_lights,
    build_cage,
    render_poses,
    set_color_management,
    set_led_emission,
    set_world,
    solve_exposure,
)
from synthgen.cage_spec import CageSceneSpec, load_cage_spec, spec_to_dict  # noqa: E402
from synthgen.configuration import load_render_config  # noqa: E402
from synthgen.constants import ROBOT_CATEGORY_ID  # noqa: E402
from synthgen.materials import load_cc_materials  # noqa: E402
from synthgen.robots import compute_ground_z, hide_all_robots, load_robots, show_robot  # noqa: E402

logger = logsetup.get_logger("render_cage_view")

DEFAULT_CC_TEXTURES = REPO_ROOT / "training/data/cc_textures"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=DESCRIPTION, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--spec", type=Path, required=True)
    parser.add_argument(
        "--pose", type=Path, required=True, help="one CageCalibration TOML or a directory of them"
    )
    parser.add_argument(
        "--pose-glob", default="*.toml", help="which TOMLs in a --pose directory to render"
    )
    parser.add_argument(
        "--camera-rect", type=Path, required=True, help="camera_rect.json from extract_targets.py"
    )
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=None, help="override [render].samples")
    parser.add_argument(
        "--set", action="append", default=[], metavar="KEY=VALUE", help="spec override"
    )
    parser.add_argument("--save-blend", type=Path, default=None)
    parser.add_argument("--auto-exposure", type=Path, default=None, metavar="TARGETS_DIR")
    parser.add_argument("--cc-textures", type=Path, default=DEFAULT_CC_TEXTURES)
    parser.add_argument(
        "--robot-frames",
        type=Path,
        default=None,
        help="runs/cage_scene/robot_frames: render MRS BUFF at each frame's recovered pose",
    )
    parser.add_argument(
        "--config", type=Path, default=SCRIPT_DIR / "config.toml", help="[[robots]] source"
    )
    parser.add_argument("-v", "--verbose", action="store_true")
    return parser.parse_args()


def load_k(path: Path) -> tuple[np.ndarray, int, int]:
    data = json.loads(path.read_text())
    k = np.array([[data["fx"], 0.0, data["cx"]], [0.0, data["fy"], data["cy"]], [0.0, 0.0, 1.0]])
    return k, int(data["width"]), int(data["height"])


def pose_files(path: Path, pattern: str) -> list[Path]:
    if path.is_dir():
        return sorted(path.glob(pattern))
    return [path]


def git_sha() -> str:
    try:
        return subprocess.run(
            ["git", "-C", str(REPO_ROOT), "rev-parse", "--short", "HEAD"],
            capture_output=True,
            text=True,
            check=False,
        ).stdout.strip()
    except OSError:
        return ""


def configure_renderer(spec: CageSceneSpec, samples: int) -> None:
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
    set_color_management(spec.exposure.gain)


def load_robot_frames(root: Path) -> list[dict[str, Any]]:
    frames = []
    for pose_json in sorted(root.glob("*/pose.json")):
        frame = json.loads(pose_json.read_text())
        frame["name"] = pose_json.parent.name
        frames.append(frame)
    if not frames:
        raise SystemExit(
            f"no robot frames under {root}; run playground/cage_scene/pick_robot_frames.py"
        )
    return frames


def place_robot(robot: Any, x_m: float, y_m: float, yaw_deg: float) -> None:
    """Grounded, upright, front along the heading; mirrors pose_single_robot's upright branch."""
    rotation = (
        math.radians(-90.0),
        math.radians(robot.config.ground_roll_upright),
        math.radians(yaw_deg),
    )
    ground_z = compute_ground_z(robot.meshes, robot.parent, rotation)
    robot.parent.location = mathutils.Vector((x_m, y_m, ground_z))
    robot.parent.rotation_euler = mathutils.Euler(rotation)
    bpy.context.view_layer.update()


def render_robot_frames(
    args: argparse.Namespace,
    spec: CageSceneSpec,
    k_rect: np.ndarray,
    width: int,
    height: int,
    cage_objects: dict[str, Any],
) -> list[Path]:
    """One render per real frame, MRS BUFF at the recovered pose, from that clip's camera."""
    frames = load_robot_frames(args.robot_frames)
    cfg = load_render_config(args.config)
    robot_name = {"mrs_buff_mk3": "MRS_BUFF_MK3", "mr_stabs_mk2": "MR_STABS_MK2"}[
        frames[0]["robot"]
    ]
    cfg_robots = tuple(r for r in cfg.robots if r.name == robot_name)
    cc_materials = load_cc_materials(
        cfg.materials, cfg.environment.cc_textures_dir, cfg.resolver.resolve
    )
    robots = load_robots(cfg_robots, cfg.materials, cc_materials, False, {}, cfg.resolver.resolve)
    hide_all_robots(robots)
    robot = robots[0]
    show_robot(robot)
    bproc.renderer.enable_segmentation_output(
        map_by=["category_id"], default_values={"category_id": 0}
    )
    written = []
    for frame in frames:
        place_robot(robot, frame["x_m"], frame["y_m"], frame["yaw_deg"])
        pose_path = Path(frame["pose_toml"])
        if not pose_path.is_absolute():
            pose_path = REPO_ROOT / pose_path
        colors, masks = render_poses(
            [pose_path],
            k_rect,
            width,
            height,
            category_ids=(MAT_CATEGORY_ID, ROBOT_CATEGORY_ID),
            spec=spec,
            objects=cage_objects,
        )
        cv2.imwrite(str(args.out / f"{frame['name']}.png"), colors[0][:, :, ::-1])
        cv2.imwrite(str(args.out / f"{frame['name']}_matmask.png"), masks[0][0])
        cv2.imwrite(str(args.out / f"{frame['name']}_robotmask.png"), masks[0][1])
        written.append(args.out / f"{frame['name']}.png")
        logger.info("wrote %s", written[-1])
    return written


def main() -> None:
    args = parse_args()
    logsetup.configure(1 if args.verbose else 0)
    spec = load_cage_spec(args.spec, args.set)
    samples = args.samples or spec.render.samples
    k_rect, width, height = load_k(args.camera_rect)
    poses = pose_files(args.pose, args.pose_glob)
    if not poses:
        raise SystemExit(f"no pose TOMLs under {args.pose}")
    args.out.mkdir(parents=True, exist_ok=True)

    bproc.init()
    configure_renderer(spec, samples)
    color_gain = tuple(float(v) for v in spec.exposure.color_gain)
    cc_dir = args.cc_textures if args.cc_textures.exists() else None
    cage_objects = build_cage(spec, REPO_ROOT, cc_dir)
    set_led_emission(spec, spec.exposure.gain)
    lights = add_lights(spec, color_gain)  # type: ignore[arg-type]
    set_world(spec, color_gain)  # type: ignore[arg-type]
    bproc.renderer.enable_segmentation_output(
        map_by=["category_id"], default_values={"category_id": 0}
    )

    exposure_note: dict[str, Any] = {}
    if args.auto_exposure is not None:
        exposure_note = solve_exposure(
            spec, poses, lights, color_gain, k_rect, width, height, samples, args.auto_exposure
        )

    if args.robot_frames is not None:
        render_robot_frames(args, spec, k_rect, width, height, cage_objects)
    else:
        colors, masks = render_poses(poses, k_rect, width, height, spec=spec, objects=cage_objects)
        for pose_path, rgb, mask in zip(poses, colors, masks):
            cv2.imwrite(str(args.out / f"{pose_path.stem}.png"), rgb[:, :, ::-1])
            cv2.imwrite(str(args.out / f"{pose_path.stem}_matmask.png"), mask)
            logger.info("wrote %s", args.out / f"{pose_path.stem}.png")

    meta = {
        "spec_file": str(args.spec),
        "overrides": args.set,
        "spec": spec_to_dict(spec),
        "auto_exposure": exposure_note,
        "poses": [str(p) for p in poses],
        "camera_rect": str(args.camera_rect),
        "K": k_rect.tolist(),
        "width": width,
        "height": height,
        "samples": samples,
        "git_sha": git_sha(),
    }
    (args.out / "render_meta.json").write_text(json.dumps(meta, indent=2) + "\n")

    if args.save_blend is not None:
        args.save_blend.parent.mkdir(parents=True, exist_ok=True)
        bpy.ops.wm.save_as_mainfile(filepath=str(args.save_blend))
        logger.info("saved %s", args.save_blend)


if __name__ == "__main__":
    main()
