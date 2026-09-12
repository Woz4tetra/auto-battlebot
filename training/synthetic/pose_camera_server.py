#!/usr/bin/env python3
import blenderproc as bproc  # noqa: F401  # isort: skip  (blenderproc must be imported first)

# BlenderProc insists its import is the first statement, so the module docstring follows it.
DESCRIPTION = """Fly a camera around the built cage with WASD and a mouse, and save where it landed.

Runs under `blenderproc run` inside the auto-battlebot-synthetic image, and serves a page the host
browser opens. The browser is what makes real mouse-look possible: `requestPointerLock` gives
relative mouse deltas with no cursor to hit a window edge, which an OpenCV window cannot do.

    training/synthetic/docker/run_synthetic.sh --gpu --port 8770 \\
        auto-battlebot-synthetic blenderproc run pose_camera_server.py -- \\
        --spec cage/cage2_overhead_high.toml \\
        --camera-calibration ../../config/cameras/ecam25_h01r1_estimated.toml \\
        --out ../data/cage_pose

Then open http://127.0.0.1:8770 on the host.

The cage is built once and never rebuilt: `build_cage` and `add_lights` create objects on every
call, so a second call would double the LED tubes and leave the panels on a `panel.001` material.
Reloading means restarting the process.

Two threads, one rule: only the main thread touches `bpy`. HTTP handler threads touch only the
mailbox in `synthgen.preview_server`, and everything they answer themselves (the live `CageMount`
readout, range clamping, the snap, the projected overlay) is pure `synthgen.freefly` math.

Outputs land under `--out`:

  * `mount_ranges.toml` -- the `[cages.mount]` block covering every marked pose, to paste into
    `training/synthetic/config.toml`. This is what the batch render consumes.
  * `<name>.toml` -- one `CageCalibration` pose, for `render_cage_view.py --pose`.
  * `camera_rect.json` and `<name>_camera.toml` -- the rectified K and the calibration behind it,
    so the saved pose can be re-rendered exactly.
"""

import argparse
import json
import shutil
import sys
import time
from pathlib import Path
from typing import Any

import bpy
import numpy as np

# Blender's embedded Python ignores PYTHONPATH, so put the script's own directory and the repo
# root on sys.path here: synthgen for the cage, auto_battlebot.perception for the frame chain.
SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
sys.path.insert(0, str(SCRIPT_DIR))
sys.path.insert(0, str(REPO_ROOT))

from synthgen import logsetup  # noqa: E402
from synthgen.cage import (  # noqa: E402
    add_lights,
    apply_exposure,
    build_cage,
    set_color_management,
    set_led_emission,
    set_world,
)
from synthgen.cage_mount import CageMount, CageMountRanges, mount_cam2world  # noqa: E402
from synthgen.cage_spec import load_cage_spec  # noqa: E402
from synthgen.configuration import load_render_config  # noqa: E402
from synthgen.freefly import (  # noqa: E402
    FreeflyPose,
    format_mount_ranges_toml,
    freefly_cam2world,
    freefly_from_cam2world,
    mount_from_cam2world,
    ranges_covering,
)
from synthgen.materials import load_cc_materials  # noqa: E402
from synthgen.preview_render import (  # noqa: E402
    OneWayGlass,
    PreviewRenderer,
    RobotField,
    clear_camera_keyframes,
)
from synthgen.preview_server import (  # noqa: E402
    ALPHAS,
    Command,
    Mailbox,
    RenderRequest,
    SceneInfo,
    fov_degrees,
    serve,
)
from synthgen.robots import load_robots  # noqa: E402

from auto_battlebot.perception.cage_calibration import (  # noqa: E402
    OPENCV_TO_BLENDER_CAMERA,
    CageCalibration,
    camera_from_world,
    save_cage_calibration,
)
from auto_battlebot.perception.camera_calibration import (  # noqa: E402
    load_camera_calibration,
    rectify_maps,
)

logger = logsetup.get_logger("pose_camera_server")

DEFAULT_CC_TEXTURES = REPO_ROOT / "training/data/cc_textures"
PAGE_PATH = SCRIPT_DIR / "web/pose_camera.html"
# MRS BUFF MK3 is a 3 lb bot; its two keypoints sit 0.1528 m apart front to back, and the shell
# overhangs both. These are the box the pixel-width overlay projects, not a measured hull.
DEFAULT_ROBOT_SIZE = (0.21, 0.18, 0.09)  # length, width, height in meters
IDLE_SLEEP_S = 0.002


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=DESCRIPTION, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--spec", type=Path, required=True, help="cage scene spec TOML")
    parser.add_argument(
        "--camera-calibration",
        type=Path,
        default=REPO_ROOT / "config/cameras/ecam25_h01r1_estimated.toml",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=SCRIPT_DIR.parent / "data/cage_pose",
        help="where the pose, ranges and renders land, beside the other scene tooling output",
    )
    parser.add_argument("--port", type=int, default=8770)
    parser.add_argument(
        "--config", type=Path, default=SCRIPT_DIR / "config.toml", help="[[cages]] and [[robots]]"
    )
    parser.add_argument(
        "--cage-name",
        default="nhrl_cage",
        help="which [[cages]] entry supplies the mount ranges the readout is judged against",
    )
    parser.add_argument("--width", type=int, default=1280, help="full render width")
    parser.add_argument("--height", type=int, default=720, help="full render height")
    parser.add_argument(
        "--preview-percentage",
        type=int,
        default=50,
        help="preview size as a percentage of the full render; 25 if 50 will not hold 5 fps",
    )
    parser.add_argument(
        "--robot-size",
        type=float,
        nargs=3,
        default=DEFAULT_ROBOT_SIZE,
        metavar=("LENGTH", "WIDTH", "HEIGHT"),
        help="robot box the pixel-width overlay projects, in meters",
    )
    parser.add_argument("--cc-textures", type=Path, default=DEFAULT_CC_TEXTURES)
    parser.add_argument("-v", "--verbose", action="store_true")
    return parser.parse_args()


def resolve_under(root: Path, path: Path) -> Path:
    """Relative paths are read against *root*, which for every flag here is `training/synthetic`.

    That is also the container's working directory, so the paths on the command line in this
    module's docstring mean the same thing whether they are typed for the container or for a
    checkout on the host.
    """
    return path if path.is_absolute() else (root / path).resolve()


def find_mount_ranges(config_path: Path, cage_name: str) -> CageMountRanges:
    """The `[cages.mount]` ranges the live readout is judged against."""
    try:
        config = load_render_config(config_path)
    except Exception as error:  # noqa: BLE001 - a bad config must not stop the tool from flying
        logger.warning(
            "could not read %s (%s); falling back to default mount ranges", config_path, error
        )
        return CageMountRanges()
    for cage in config.cages:
        if cage.name == cage_name:
            return cage.mount
    logger.warning(
        "no [[cages]] entry named %r in %s; falling back to default mount ranges",
        cage_name,
        config_path,
    )
    return CageMountRanges()


def build_scene(args: argparse.Namespace) -> tuple[Any, Any]:
    """`bproc.init()` plus the whole cage, exactly once. Returns (spec, cage objects)."""
    spec = load_cage_spec(resolve_under(SCRIPT_DIR, args.spec))
    bproc.init()
    color_gain = tuple(float(v) for v in spec.exposure.color_gain)
    cc_dir = args.cc_textures if args.cc_textures.exists() else None
    objects = build_cage(spec, REPO_ROOT, cc_dir)
    set_led_emission(spec, spec.exposure.gain)
    add_lights(spec, color_gain)  # type: ignore[arg-type]
    set_world(spec, color_gain)  # type: ignore[arg-type]
    apply_exposure(spec, spec.exposure.gain)
    set_color_management(spec.exposure.gain)
    return spec, objects


def starting_pose(ranges: CageMountRanges, wall_half_m: float) -> FreeflyPose:
    """Start at the middle of the configured ranges, which is a mount worth looking at."""
    middle = CageMount(
        wall=ranges.walls[0],
        along_m=sum(ranges.along_m) / 2,
        height_m=sum(ranges.height_m) / 2,
        inset_m=sum(ranges.inset_m) / 2,
        tilt_deg=sum(ranges.tilt_deg) / 2,
        yaw_deg=sum(ranges.yaw_deg) / 2,
        roll_deg=sum(ranges.roll_deg) / 2,
    )
    return freefly_from_cam2world(mount_cam2world(middle, wall_half_m))


class RobotToggle:
    """MRS BUFF MK3 on the mat, loaded the first time it is asked for.

    One import, then linked duplicates at every mat sample point, so a single toggle shows how a
    robot reads at the center and at all eight compass directions at once. The import takes a few
    seconds, which is why it does not happen at startup: the tool is usable long before anyone
    wants to check a silhouette.
    """

    def __init__(self, config_path: Path, mat_size_m: float) -> None:
        self._config_path = config_path
        self._mat_size_m = mat_size_m
        self._field: RobotField | None = None
        self._loaded = False
        self.shown = False

    def set(self, shown: bool) -> None:
        if shown and not self._loaded:
            self._load()
        if self._field is None:
            return
        self._field.set_visible(shown)
        self.shown = shown

    def _load(self) -> None:
        self._loaded = True
        config = load_render_config(self._config_path)
        entries = tuple(r for r in config.robots if r.name == "MRS_BUFF_MK3")
        if not entries:
            logger.warning("no MRS_BUFF_MK3 in %s; robot toggle does nothing", self._config_path)
            return
        cc_materials = load_cc_materials(
            config.materials, config.environment.cc_textures_dir, config.resolver.resolve
        )
        robots = load_robots(
            entries, config.materials, cc_materials, False, {}, config.resolver.resolve
        )
        self._field = RobotField(robots[0], self._mat_size_m, entries[0].ground_roll_upright)
        self._field.set_visible(False)


def write_pose(out_dir: Path, name: str, pose: FreeflyPose, mat_size_m: float) -> Path:
    """One `CageCalibration` TOML, in the exact keys `render_cage_view.py --pose` reads."""
    world_from_camera = freefly_cam2world(pose) @ np.linalg.inv(OPENCV_TO_BLENDER_CAMERA)
    calibration = CageCalibration(
        calibration_id=name,
        field_size_x=mat_size_m,
        field_size_y=mat_size_m,
        tf_camera_from_fieldcenter=camera_from_world(world_from_camera),
    )
    path = out_dir / f"{name}.toml"
    save_cage_calibration(
        path,
        calibration,
        header=(
            "Flown with training/synthetic/pose_camera_server.py, not fitted to footage.\n"
            f"Free-flight pose: position ({pose.x_m:.4f}, {pose.y_m:.4f}, {pose.z_m:.4f}) m, "
            f"yaw {pose.yaw_deg:.2f} deg, pitch {pose.pitch_deg:.2f} deg, "
            f"roll {pose.roll_deg:.2f} deg."
        ),
    )
    return path


def write_camera_rect(
    out_dir: Path, k_rect: np.ndarray, size: tuple[int, int], alpha: float
) -> Path:
    """The `--camera-rect` JSON `render_cage_view.py` wants, so the saved pose re-renders."""
    path = out_dir / "camera_rect.json"
    path.write_text(
        json.dumps(
            {
                "alpha": alpha,
                "fx": float(k_rect[0, 0]),
                "fy": float(k_rect[1, 1]),
                "cx": float(k_rect[0, 2]),
                "cy": float(k_rect[1, 2]),
                "width": size[0],
                "height": size[1],
            },
            indent=2,
        )
        + "\n"
    )
    return path


def render_command(args: argparse.Namespace, out_dir: Path, pose_path: Path) -> str:
    """The exact command line that reproduces the full artifact set for a saved pose."""
    spec = resolve_under(SCRIPT_DIR, args.spec)
    # Paths are absolute container paths: run_synthetic.sh runs with the working directory set to
    # /workspace/training/synthetic, so a repo-relative script path would not resolve.
    return (
        "training/synthetic/docker/run_synthetic.sh --gpu auto-battlebot-synthetic blenderproc "
        f"run {_workspace(SCRIPT_DIR / 'render_cage_view.py')} -- "
        f"--spec {_workspace(spec)} "
        f"--pose {_workspace(pose_path)} "
        f"--camera-rect {_workspace(out_dir / 'camera_rect.json')} "
        f"--out {_workspace(out_dir / 'renders')} --samples 128"
    )


def _workspace(path: Path) -> str:
    """Repo paths as the container sees them, since that is where the command runs."""
    try:
        return f"/workspace/{path.resolve().relative_to(REPO_ROOT)}"
    except ValueError:
        return str(path)


def main() -> None:
    args = parse_args()
    logsetup.configure(1 if args.verbose else 0)
    out_dir = resolve_under(SCRIPT_DIR, args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    calibration_path = resolve_under(SCRIPT_DIR, args.camera_calibration)
    calibration = load_camera_calibration(calibration_path)
    render_size = (args.width, args.height)
    spec, objects = build_scene(args)
    ranges = find_mount_ranges(resolve_under(SCRIPT_DIR, args.config), args.cage_name)
    wall_half_m = spec.cage.interior / 2

    renderer = PreviewRenderer(calibration, spec, render_size, args.preview_percentage)
    clear_camera_keyframes(bpy.context.scene.camera)
    renderer.capture_scene_state()
    warm_s = renderer.warm_up()
    logger.info("warm-up frame took %.2f s; OptiX kernels are compiled", warm_s)

    scaled = calibration.scaled(*render_size)
    k_rect = {alpha: rectify_maps(calibration, render_size, alpha)[2] for alpha in ALPHAS}
    length, width, height = (float(v) for v in args.robot_size)
    scene_info = SceneInfo(
        spec_path=str(resolve_under(SCRIPT_DIR, args.spec)),
        calibration_id=calibration.calibration_id,
        calibration_path=str(calibration_path),
        out_dir=str(out_dir),
        wall_half_m=wall_half_m,
        mat_size_m=spec.mat.size,
        render_width=render_size[0],
        render_height=render_size[1],
        preview_width=renderer.preview_size[0],
        preview_height=renderer.preview_size[1],
        k_rect_full=k_rect[1.0].tolist(),
        k_rect_cropped=k_rect[0.0].tolist(),
        k_calibrated=scaled.K.tolist(),
        distortion=[float(v) for v in calibration.distortion],
        rectified_fov_full_deg=fov_degrees(k_rect[1.0], *render_size),
        rectified_fov_cropped_deg=fov_degrees(k_rect[0.0], *render_size),
        calibrated_fov_deg=fov_degrees(scaled.K, *render_size),
        robot_length_m=length,
        robot_width_m=width,
        robot_height_m=height,
    )
    logger.info(
        "rectified alpha 1.0: %.1f x %.1f deg (fx %.1f); alpha 0.0: %.1f x %.1f deg (fx %.1f); "
        "calibrated %.1f x %.1f deg (fx %.1f)",
        *scene_info.rectified_fov_full_deg,
        k_rect[1.0][0, 0],
        *scene_info.rectified_fov_cropped_deg,
        k_rect[0.0][0, 0],
        *scene_info.calibrated_fov_deg,
        scaled.fx,
    )

    mailbox = Mailbox()
    mailbox.submit_pose(RenderRequest(pose=starting_pose(ranges, wall_half_m)))
    robots = RobotToggle(resolve_under(SCRIPT_DIR, args.config), spec.mat.size)
    glass = OneWayGlass(spec, objects)
    serve(mailbox, scene_info, ranges, spec, out_dir, PAGE_PATH.read_bytes, args.port)
    mailbox.set_status("ready")
    logger.info("serving on port %d; open http://127.0.0.1:%d on the host", args.port, args.port)

    render_loop(mailbox, renderer, robots, glass, scene_info, ranges, args, out_dir)


def render_loop(
    mailbox: Mailbox,
    renderer: PreviewRenderer,
    robots: RobotToggle,
    glass: OneWayGlass,
    scene: SceneInfo,
    ranges: CageMountRanges,
    args: argparse.Namespace,
    out_dir: Path,
) -> None:
    """The main thread, forever: drain commands, render the newest pose, publish the frame."""
    current: RenderRequest | None = None
    while True:
        for command in mailbox.drain_commands():
            handle_command(command, mailbox, renderer, glass, current, scene, ranges, args, out_dir)
        request = mailbox.take_pose()
        if request is None:
            time.sleep(IDLE_SLEEP_S)
            continue
        current = request
        glass.update(freefly_cam2world(request.pose)[:3, 3])
        if request.show_robots != robots.shown:
            robots.set(request.show_robots)
        try:
            jpeg, elapsed_ms, size = renderer.render_preview(
                freefly_cam2world(request.pose), request.view, request.alpha
            )
        except Exception as error:  # noqa: BLE001 - one bad frame must not end the session
            logger.exception("preview render failed: %s", error)
            mailbox.set_status(f"render failed: {error}")
            time.sleep(0.25)
            continue
        mailbox.publish(jpeg, request.view, elapsed_ms, size)


def handle_command(
    command: Command,
    mailbox: Mailbox,
    renderer: PreviewRenderer,
    glass: OneWayGlass,
    current: RenderRequest | None,
    scene: SceneInfo,
    ranges: CageMountRanges,
    args: argparse.Namespace,
    out_dir: Path,
) -> None:
    if current is None:
        mailbox.set_status("nothing rendered yet; move the camera first")
        return
    cam2world = freefly_cam2world(current.pose)
    if command.kind == "mark":
        mount = mount_from_cam2world(cam2world, scene.wall_half_m, walls=ranges.walls)
        count = mailbox.add_mark(mount)
        mailbox.set_status(
            f"marked {count} pose(s): {mount.wall} wall, tilt {mount.tilt_deg:.1f} deg"
        )
    elif command.kind == "clear_marks":
        mailbox.clear_marks()
        mailbox.set_status("cleared every mark")
    elif command.kind == "render_full":
        mailbox.set_status("rendering at full quality...")
        glass.update(cam2world[:3, 3])
        name = str(command.payload.get("name", "full")) or "full"
        path, elapsed_ms = renderer.render_full(
            cam2world,
            current.view,
            current.alpha,
            out_dir / f"{name}_{current.view}_alpha{current.alpha:g}.png",
        )
        mailbox.set_status(f"wrote {path.name} in {elapsed_ms / 1000:.1f} s")
        logger.info("full render: %s (%.1f s)", path, elapsed_ms / 1000)
    elif command.kind == "save":
        mailbox.set_status(save_outputs(command, mailbox, renderer, current, scene, args, out_dir))
    else:
        logger.warning("ignoring unknown command %r", command.kind)


def save_outputs(
    command: Command,
    mailbox: Mailbox,
    renderer: PreviewRenderer,
    current: RenderRequest,
    scene: SceneInfo,
    args: argparse.Namespace,
    out_dir: Path,
) -> str:
    """Write the pose, the ranges over every mark, and everything needed to re-render them."""
    name = str(command.payload.get("name", "cage_pose")) or "cage_pose"
    pad_m = float(command.payload.get("pad_m", 0.02))
    pad_deg = float(command.payload.get("pad_deg", 1.0))

    pose_path = write_pose(out_dir, name, current.pose, scene.mat_size_m)
    # The rectified K written here is the alpha that was on screen, so a re-render reproduces the
    # framing that was actually being judged.
    write_camera_rect(out_dir, renderer.k_rect(current.alpha), renderer.render_size, current.alpha)
    shutil.copy(scene.calibration_path, out_dir / f"{name}_camera.toml")

    written = [pose_path.name]
    marks = mailbox.marks
    if marks:
        header = (
            f"Mount ranges covering {len(marks)} pose(s) flown in pose_camera_server.py, "
            f"padded by {pad_m} m and {pad_deg} deg.\n"
            "Paste under the matching [[cages]] entry in training/synthetic/config.toml."
        )
        ranges_path = out_dir / "mount_ranges.toml"
        ranges_path.write_text(
            format_mount_ranges_toml(ranges_covering(marks, pad_m, pad_deg), header)
        )
        written.append(ranges_path.name)

    command_line = render_command(args, out_dir, pose_path)
    (out_dir / f"{name}_render_command.txt").write_text(command_line + "\n")
    logger.info("saved %s; re-render it with:\n  %s", ", ".join(written), command_line)
    return f"saved {', '.join(written)}; re-render command in {name}_render_command.txt"


if __name__ == "__main__":
    main()
