import blenderproc as bproc  # noqa: F401  # isort: skip  # must be first import for blenderproc run

"""Render top-down robot sprites for the simulation viewer and commit the PNGs.

Usage (run from training/synthetic/, same as render_scenes.py):
    PYTHONPATH="$PWD" blenderproc run render_sprites.py -- config_meshy_grade.toml

The viewer only ever reads the PNGs and the manifest beside them, so Blender and BlenderProc stay
out of the runtime dependencies. This is run by hand when a model changes.

Everything that makes a robot look like itself already lives in the synthetic config: the OnShape
part-colour to PBR-material mapping, the sticker textures, the upright pitch, and the per-model
ground roll. Rendering sprites from a second, hand-maintained table produced untextured models
lit by three area lamps, and half of them upside down. This reads the same config the training
data does, so a material fix lands in both places at once.

Two conventions the runtime depends on, both recorded in sprites.json beside the PNGs:

  - pixels_per_meter, so blitting scales correctly at any window size;
  - the robot front points +X in the render, so runtime rotation is a plain rotation by yaw with
    no per-asset offset.

The front direction is not hand-measured. It comes from the ``[robots.keypoints]`` front/back
pair the keypoint detector is trained against, so the sprite and the detector agree on which end
is the front by construction.

The models, textures, and HDRIs all live under training/data, which is gitignored. That is fine:
the PNGs are the committed artifact. Re-rendering needs a machine with the training data.
"""

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path

import bpy
import mathutils
import numpy as np

# Blender's embedded Python ignores PYTHONPATH, so the env var the docker wrapper sets does not
# reach an import here. Add this script's own directory so a bare host can run it too.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from synthgen import logsetup
from synthgen.configuration import RenderConfig, RobotConfig, load_render_config
from synthgen.geometry import model_to_blender_local
from synthgen.materials import apply_pbr_materials, load_cc_materials
from synthgen.robots import import_gltf_as_robot

# BlenderProc re-executes this script from a temp directory, so capture the real one now.
LAUNCH_CWD = Path(os.environ.get("BLENDERPROC_CWD", os.getcwd()))
REPO_ROOT = Path(__file__).resolve().parents[2]

# Sky-only outdoor HDRI. Sun plus blue-sky fill lights a top-down view evenly; the three-area-lamp
# rig this replaced left the flat upper surfaces that dominate a top-down sprite reading as
# near-black.
HDRI = REPO_ROOT / "training/data/hdris/table_mountain_1_puresky_2k.hdr"


@dataclass(frozen=True)
class SpriteSpec:
    """One sprite the viewer asks for by name.

    Attributes:
        name: Manifest key and PNG stem, and what a sim config's ``sprite`` key names. Sprites
            are named after the robot, not its role: which one is ours is a per-run choice.
        robot: ``[[robots]]`` entry to render, by its config ``name``.
        model_path: Override the config's model, keeping its materials and orientation. Used for
            robots the training set does not need but the viewer draws.
        front_model: Front direction in the model's own frame, for an override model that has
            no keypoints of its own. Same frame the config's ``[robots.keypoints]`` use, so it goes
            through the same Y-up to Z-up conversion.
    """

    name: str
    robot: str
    model_path: Path | None = None
    front_model: tuple[float, float, float] | None = None


SPRITES = (
    SpriteSpec(name="mrs_buff_mk3", robot="MRS_BUFF_MK3"),
    SpriteSpec(name="mr_stabs_mk2", robot="MR_STABS_MK2"),
    # Mrs Buff MK2 is not in the training set, but it shares the MK3's OnShape palette, so the
    # MK3's colour mapping textures it correctly. Its CAD origin sits at the back rather than the
    # front, hence the explicit front direction.
    SpriteSpec(
        name="mrs_buff_mk2",
        robot="MRS_BUFF_MK3",
        model_path=REPO_ROOT / "simulation/assets/robots/mrs_buff_mk2.glb",
        front_model=(0.0, -1.0, 0.0),
    ),
    # The house bot GLB carries baked PBR textures already, so the colour mapping finds nothing to
    # replace and leaves them alone. Its painted face points +X.
    SpriteSpec(
        name="house_bot",
        robot="MRS_BUFF_MK3",
        model_path=REPO_ROOT / "simulation/assets/robots/house_bot.glb",
        front_model=(1.0, 0.0, 0.0),
    ),
)


def clear_scene() -> None:
    """Empty the scene between sprites so one model's meshes cannot leak into the next."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for block in (bpy.data.meshes, bpy.data.objects, bpy.data.cameras, bpy.data.lights):
        for item in list(block):
            if item.users == 0:
                block.remove(item)


def upright_yaw(front_local: np.ndarray, upright: mathutils.Matrix) -> float:
    """Yaw that turns *front_local* onto +X once the model is standing upright.

    The front/back keypoints are authored in the model's own frame. Standing the model up moves
    them, and the ground roll tilts them out of the horizontal plane, so take the heading of the
    projection rather than the raw vector.
    """
    world_front = upright @ mathutils.Vector(tuple(float(v) for v in front_local))
    if world_front.xy.length < 1e-9:
        raise ValueError("front direction is vertical once upright; cannot derive a yaw")
    return -math.atan2(world_front.y, world_front.x)


def upright_matrix(robot_config: RobotConfig) -> mathutils.Matrix:
    """Stand a CAD-exported robot on its wheels, the way the scene pipeline poses it.

    ``pose_single_robot`` uses a -90 degree pitch for upright and a per-model roll that takes out
    the tilt left by where the CAD origin sits. Reproducing both here is what keeps the sprite's
    footprint equal to the robot's real footprint on the floor.
    """
    pitch = mathutils.Matrix.Rotation(-math.pi / 2.0, 4, "X")
    roll = mathutils.Matrix.Rotation(math.radians(robot_config.ground_roll_upright), 4, "Y")
    return pitch @ roll


def world_bounds(meshes: list[bproc.types.MeshObject]) -> tuple[mathutils.Vector, mathutils.Vector]:
    lo = mathutils.Vector((math.inf,) * 3)
    hi = mathutils.Vector((-math.inf,) * 3)
    for mesh in meshes:
        obj = mesh.blender_obj
        for corner in obj.bound_box:
            point = obj.matrix_world @ mathutils.Vector(corner)
            lo = mathutils.Vector(min(a, b) for a, b in zip(lo, point))
            hi = mathutils.Vector(max(a, b) for a, b in zip(hi, point))
    if lo.x == math.inf:
        raise RuntimeError("no mesh geometry in import")
    return lo, hi


def add_sky_lighting(strength: float, rotation_deg: float) -> None:
    """Light the scene from an outdoor sky HDRI.

    The sun gives the silhouette an edge and the sky fills the flat top surfaces, which is most of
    what a top-down sprite shows. Rotation aims the sun, so it decides which side catches the
    highlight.
    """
    if not HDRI.exists():
        raise FileNotFoundError(f"HDRI not found: {HDRI}")

    world = bpy.data.worlds.new("sprite_world")
    world.use_nodes = True
    tree = world.node_tree
    background = tree.nodes["Background"]
    background.inputs["Strength"].default_value = strength

    environment = tree.nodes.new("ShaderNodeTexEnvironment")
    environment.image = bpy.data.images.load(str(HDRI))
    mapping = tree.nodes.new("ShaderNodeMapping")
    mapping.inputs["Rotation"].default_value[2] = math.radians(rotation_deg)
    coordinate = tree.nodes.new("ShaderNodeTexCoord")
    tree.links.new(coordinate.outputs["Generated"], mapping.inputs["Vector"])
    tree.links.new(mapping.outputs["Vector"], environment.inputs["Vector"])
    tree.links.new(environment.outputs["Color"], background.inputs["Color"])
    bpy.context.scene.world = world


def render_sprite(
    spec: SpriteSpec,
    robot_config: RobotConfig,
    render_config: RenderConfig,
    cc_materials: dict[str, bproc.types.Material],
    out_dir: Path,
    args: argparse.Namespace,
) -> dict[str, object]:
    """Import one model, texture it, shoot it from straight above, write the PNG."""
    clear_scene()

    model_path = spec.model_path if spec.model_path is not None else robot_config.model_path
    resolve = render_config.resolver.resolve
    resolved_model = resolve(model_path)
    meshes, parent, _bbox = import_gltf_as_robot(model_path, resolve, robot_config.scale)
    apply_pbr_materials(
        meshes, robot_config.color_mapping, render_config.materials, cc_materials, resolve
    )

    front_local = (
        model_to_blender_local(list(spec.front_model))
        if spec.front_model is not None
        else robot_config.keypoints.front - robot_config.keypoints.back
    )
    upright = upright_matrix(robot_config)
    yaw_rad = upright_yaw(front_local, upright)
    front_rounded = tuple(round(float(v), 4) for v in front_local)
    print(f"  front {front_rounded} -> yaw {math.degrees(yaw_rad):+.1f} deg")
    yaw = mathutils.Matrix.Rotation(yaw_rad, 4, "Z")
    parent.matrix_world = yaw @ upright @ parent.matrix_world
    # The GLTF hierarchy hangs every mesh off the parent empty. Moving the empty only reaches the
    # children once the dependency graph re-evaluates, and world_bounds reads matrix_world right
    # afterwards; without this the transform silently does nothing.
    bpy.context.view_layer.update()

    lo, hi = world_bounds(meshes)
    center = (lo + hi) / 2.0
    extent_x = hi.x - lo.x
    extent_y = hi.y - lo.y
    # A little air around the silhouette so rotation in the viewer never clips a corner.
    span = max(extent_x, extent_y) * 1.08

    camera_data = bpy.data.cameras.new("sprite_cam")
    camera_data.type = "ORTHO"
    camera_data.ortho_scale = span
    camera = bpy.data.objects.new("sprite_cam", camera_data)
    camera.location = (center.x, center.y, hi.z + 2.0)
    camera.rotation_euler = (0.0, 0.0, 0.0)  # straight down, +X right and +Y up in the image
    bpy.context.collection.objects.link(camera)
    bpy.context.scene.camera = camera

    add_sky_lighting(args.hdri_strength, args.hdri_rotation_deg)

    pixels = max(32, int(round(span * args.pixels_per_meter)))
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.cycles.samples = args.samples
    scene.render.film_transparent = True
    scene.render.resolution_x = pixels
    scene.render.resolution_y = pixels
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGBA"

    out_path = out_dir / f"{spec.name}.png"
    scene.render.filepath = str(out_path)
    bpy.ops.render.render(write_still=True)

    # The rendered image spans `span` metres, so this is the true scale of the file on disk, not
    # the requested one (which the pixel rounding perturbs).
    return {
        "file": out_path.name,
        "pixels_per_meter": round(pixels / span, 4),
        "footprint_m": [round(extent_x, 4), round(extent_y, 4)],
        "front_axis": "+X",
        # Repo-relative, so the manifest names the same file whatever directory it ran from.
        "source": str(resolved_model.relative_to(REPO_ROOT)),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path, help="Path to the synthetic render config")
    parser.add_argument("--out", type=Path, default=REPO_ROOT / "simulation/assets/sprites")
    parser.add_argument(
        "--pixels-per-meter",
        type=float,
        default=600.0,
        help="render scale; the manifest records what each file actually came out at",
    )
    parser.add_argument("--samples", type=int, default=96)
    parser.add_argument("--hdri-strength", type=float, default=10.0)
    parser.add_argument(
        "--hdri-rotation-deg",
        type=float,
        default=0.0,
        help="spin the sky about the vertical axis to aim the sun",
    )
    parser.add_argument("--only", default="", help="render just this sprite name")
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else sys.argv[1:]
    return parser.parse_args(argv)


def main() -> None:
    args = parse_args()
    logsetup.configure(verbosity=0)

    render_config = load_render_config(
        args.config, launch_cwd=LAUNCH_CWD, project_root=REPO_ROOT / "training"
    )
    robots = {robot.name: robot for robot in render_config.robots}
    cc_materials = load_cc_materials(
        render_config.materials,
        render_config.environment.cc_textures_dir,
        render_config.resolver.resolve,
    )

    out_dir = args.out if args.out.is_absolute() else LAUNCH_CWD / args.out
    out_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = out_dir / "sprites.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    manifest.setdefault("sprites", {})

    for spec in SPRITES:
        if args.only and spec.name != args.only:
            continue
        if spec.robot not in robots:
            raise KeyError(f"{spec.name}: no [[robots]] entry named {spec.robot!r} in the config")
        print(f"rendering {spec.name} from {spec.model_path or robots[spec.robot].model_path}")
        manifest["sprites"][spec.name] = render_sprite(
            spec, robots[spec.robot], render_config, cc_materials, out_dir, args
        )

    manifest["front_axis"] = "+X"
    manifest["note"] = (
        "Rendered by training/synthetic/render_sprites.py. pixels_per_meter and the +X front "
        "convention are what the viewer reads; do not hand-edit."
    )
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"wrote {manifest_path}")


if __name__ == "__main__":
    main()
