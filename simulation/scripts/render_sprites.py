"""Render top-down robot sprites offline and commit the PNGs.

Blender stays out of the runtime dependencies: this is run by hand when a model changes, and the
viewer only ever reads the PNGs and the manifest beside them.

    blender --background --python simulation/scripts/render_sprites.py -- \
        --out simulation/assets/sprites

Per asset: import the GLB, point an orthographic camera straight down, make the film transparent,
light it flat rather than dramatically so the sprite still reads at 100 px, render RGBA at a fixed
pixels-per-metre, write the PNG.

Two conventions the runtime depends on, both recorded in sprites.json beside the PNGs:

  - pixels_per_meter, so blitting scales correctly at any window size;
  - the robot front points +X in the render, so runtime rotation is a plain rotation by yaw with
    no per-asset offset.

Get either wrong and every sprite sits slightly off, or the robots drive sideways. The manifest
exists so those two numbers live in one place instead of scattered through the viewer.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

# Provided by Blender at run time, not by pyproject.toml: this script is never imported by
# anything in the venv.
import bpy
import mathutils

REPO_ROOT = Path(__file__).resolve().parents[2]

# One entry per sprite the viewer asks for by name.
#
# `up_axis` is the model's own up direction after import. These GLBs keep glTF's Y-up convention,
# so a camera pointed down -Z renders a side view of the robot -- which is exactly what the first
# pass produced. Set it per model rather than guessing from the bounding box, which cannot tell a
# short wide robot from a tall narrow one.
#
# `yaw_offset_deg` then rotates the model about the (corrected) up axis so its front ends up along
# +X. Measure it once per model and record it here rather than teaching the viewer a per-asset
# offset. The values below were read off the renders themselves: render with 0, look at which way
# the weapon points, set the offset. Re-check after any model is re-exported.
ASSETS = [
    {
        "name": "our_robot",
        "source": "simulation/assets/robots/mrs_buff_mk2.glb",
        "up_axis": "Y",
        # Drum forward: the model stands with the weapon along +Y, so it wants a quarter turn.
        "yaw_offset_deg": -90.0,
    },
    {
        "name": "mr_stabs_mk2",
        "source": "simulation/assets/robots/mr_stabs_mk2.glb",
        "up_axis": "Y",
        "yaw_offset_deg": 180.0,
    },
    {
        "name": "opponent",
        "source": "simulation/assets/robots/mr_stabs_mk2.glb",
        "up_axis": "Y",
        "yaw_offset_deg": 180.0,
    },
    {
        "name": "house_bot",
        "source": "simulation/assets/robots/house_bot.glb",
        "up_axis": "Y",
        "yaw_offset_deg": 0.0,
    },
]


def clear_scene() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for block in (bpy.data.meshes, bpy.data.materials, bpy.data.images):
        for item in list(block):
            if item.users == 0:
                block.remove(item)


def import_glb(path: Path) -> list[Any]:
    before = set(bpy.data.objects)
    bpy.ops.import_scene.gltf(filepath=str(path))
    return [obj for obj in bpy.data.objects if obj not in before]


def world_bounds(objects: list[Any]) -> tuple[Any, Any]:
    lo = mathutils.Vector((math.inf,) * 3)
    hi = mathutils.Vector((-math.inf,) * 3)
    for obj in objects:
        if obj.type != "MESH":
            continue
        for corner in obj.bound_box:
            point = obj.matrix_world @ mathutils.Vector(corner)
            lo = mathutils.Vector(min(a, b) for a, b in zip(lo, point))
            hi = mathutils.Vector(max(a, b) for a, b in zip(hi, point))
    if lo.x == math.inf:
        raise RuntimeError("no mesh geometry in import")
    return lo, hi


def add_flat_lighting() -> None:
    """Three soft area lights, no key/fill contrast. A dramatic key reads as a shadow blob at
    sprite size; flat lighting keeps the silhouette legible."""
    for index, (x, y) in enumerate(((2.0, 2.0), (-2.0, 2.0), (0.0, -2.5))):
        light_data = bpy.data.lights.new(name=f"key_{index}", type="AREA")
        light_data.energy = 400.0
        light_data.size = 3.0
        light = bpy.data.objects.new(name=f"key_{index}", object_data=light_data)
        light.location = (x, y, 2.5)
        light.rotation_euler = (0.0, 0.0, 0.0)
        bpy.context.collection.objects.link(light)

    world = bpy.data.worlds.new("sprite_world")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[1].default_value = 0.6
    bpy.context.scene.world = world


def render_asset(
    asset: dict[str, Any], out_dir: Path, pixels_per_meter: float, samples: int
) -> dict[str, Any]:
    clear_scene()
    source = REPO_ROOT / asset["source"]
    if not source.exists():
        raise FileNotFoundError(source)
    objects = import_glb(source)

    # Stand the model up, then turn its front onto +X, then measure the footprint of what will
    # actually be rendered.
    transform = mathutils.Matrix.Identity(4)
    if asset.get("up_axis", "Z").upper() == "Y":
        transform = mathutils.Matrix.Rotation(math.pi / 2.0, 4, "X") @ transform
    yaw = math.radians(asset["yaw_offset_deg"])
    if yaw:
        transform = mathutils.Matrix.Rotation(yaw, 4, "Z") @ transform
    if transform != mathutils.Matrix.Identity(4):
        for obj in objects:
            if obj.parent is None:
                obj.matrix_world = transform @ obj.matrix_world
        # Some of these GLBs hang every mesh off one root empty. Moving the empty only reaches the
        # children once the dependency graph re-evaluates, and world_bounds reads matrix_world
        # straight afterwards -- without this the transform silently does nothing.
        bpy.context.view_layer.update()

    lo, hi = world_bounds(objects)
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

    add_flat_lighting()

    pixels = max(32, int(round(span * pixels_per_meter)))
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.cycles.samples = samples
    scene.render.film_transparent = True
    scene.render.resolution_x = pixels
    scene.render.resolution_y = pixels
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGBA"

    out_path = out_dir / f"{asset['name']}.png"
    scene.render.filepath = str(out_path)
    bpy.ops.render.render(write_still=True)

    # The rendered image spans `span` metres, so this is the true scale of the file on disk,
    # not the requested one (which the pixel rounding perturbs).
    actual_ppm = pixels / span
    return {
        "file": out_path.name,
        "pixels_per_meter": round(actual_ppm, 4),
        "footprint_m": [round(extent_x, 4), round(extent_y, 4)],
        "front_axis": "+X",
        "source": asset["source"],
    }


def main() -> None:
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=REPO_ROOT / "simulation/assets/sprites")
    parser.add_argument(
        "--pixels-per-meter",
        type=float,
        default=600.0,
        help="render scale; the manifest records what each file actually came out at",
    )
    parser.add_argument("--samples", type=int, default=48)
    parser.add_argument("--only", default="", help="render just this asset name")
    args = parser.parse_args(argv)

    out_dir = args.out if args.out.is_absolute() else REPO_ROOT / args.out
    out_dir.mkdir(parents=True, exist_ok=True)

    manifest_path = out_dir / "sprites.json"
    manifest: dict[str, Any] = {"sprites": {}}
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        manifest.setdefault("sprites", {})

    for asset in ASSETS:
        if args.only and asset["name"] != args.only:
            continue
        print(f"rendering {asset['name']} from {asset['source']}")
        manifest["sprites"][asset["name"]] = render_asset(
            asset, out_dir, args.pixels_per_meter, args.samples
        )

    manifest["front_axis"] = "+X"
    manifest["note"] = (
        "Rendered by simulation/scripts/render_sprites.py. pixels_per_meter and the +X front "
        "convention are what the viewer reads; do not hand-edit."
    )
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"wrote {manifest_path}")


if __name__ == "__main__":
    main()
