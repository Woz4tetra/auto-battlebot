import blenderproc as bproc  # isort: skip  # must be first import for blenderproc run

import argparse
import math
import os
import sys
import tomllib
from pathlib import Path

import bpy
import cv2
import mathutils
import numpy as np

# BlenderProc changes CWD to a temp directory, so capture it now.
_LAUNCH_CWD = Path(os.environ.get("BLENDERPROC_CWD", os.getcwd()))


def resolve_path(path: Path) -> Path:
    """Resolve a potentially relative path against the original launch directory."""
    if path.is_absolute():
        return path
    resolved = (_LAUNCH_CWD / path).resolve()
    return resolved


def load_config(config_path: Path) -> dict:
    with open(resolve_path(config_path), "rb") as f:
        return tomllib.load(f)


def import_gltf(model_path: Path, scale: float = 1.0) -> list[bpy.types.Object]:
    """Import a GLTF/GLB file and return all imported mesh objects."""
    model_path = resolve_path(model_path)
    if not model_path.exists():
        raise FileNotFoundError(
            f"Model file not found: {model_path}\n  Launch CWD was: {_LAUNCH_CWD}"
        )
    bpy.ops.import_scene.gltf(filepath=str(model_path))
    meshes = [obj for obj in bpy.context.selected_objects if obj.type == "MESH"]
    if not meshes:
        raise RuntimeError(f"No mesh objects found in {model_path}")
    if scale != 1.0:
        for obj in meshes:
            obj.scale = (scale, scale, scale)
        bpy.context.view_layer.update()
    return meshes


def _color_from_node(node: bpy.types.ShaderNode) -> tuple[int, int, int] | None:
    """Read a constant RGB value from node types where default_value is reliable."""
    if node.type == "RGB":
        c = node.outputs[0].default_value
        return (int(c[0] * 255), int(c[1] * 255), int(c[2] * 255))
    return None


def _color_from_material_name(name: str) -> tuple[int, int, int] | None:
    """Parse RGB from material names like '0.501961_0.501961_0.501961_...'

    Common in CAD-exported GLTFs where names are auto-generated from RGBA.
    """
    parts = name.split("_")
    if len(parts) >= 3:
        try:
            r, g, b = float(parts[0]), float(parts[1]), float(parts[2])
            if all(0.0 <= v <= 1.0 for v in (r, g, b)):
                return (int(r * 255), int(g * 255), int(b * 255))
        except ValueError:
            pass
    return None


def get_material_base_color(mat: bpy.types.Material) -> tuple[int, int, int] | None:
    """Extract the base color RGB (0-255) from a Blender material.

    Tries, in order: the Principled BSDF default value (when no texture is
    linked), the connected source node's color output, and finally the
    material name (for CAD-exported GLTFs that encode color in the name).
    """
    if not mat.use_nodes:
        return _color_from_material_name(mat.name)
    bsdf = None
    for node in mat.node_tree.nodes:
        if node.type == "BSDF_PRINCIPLED":
            bsdf = node
            break
    if bsdf is None:
        return _color_from_material_name(mat.name)

    bc_input = bsdf.inputs["Base Color"]
    if not bc_input.links:
        c = bc_input.default_value
        return (int(c[0] * 255), int(c[1] * 255), int(c[2] * 255))

    source_color = _color_from_node(bc_input.links[0].from_node)
    if source_color is not None:
        return source_color

    return _color_from_material_name(mat.name)


def inspect_model(model_path: Path, scale: float = 1.0) -> None:
    """Print all material names and base colors found in the GLTF."""
    meshes = import_gltf(model_path, scale)
    print(f"\nFound {len(meshes)} mesh objects in {model_path.name}:\n")
    seen_materials: set[str] = set()
    for mesh_obj in meshes:
        for slot in mesh_obj.material_slots:
            mat = slot.material
            if mat is None or mat.name in seen_materials:
                continue
            seen_materials.add(mat.name)
            color = get_material_base_color(mat)
            color_str = f"[{color[0]}, {color[1]}, {color[2]}]" if color else "N/A"
            print(f"  Material: {mat.name:30s}  Color (RGB): {color_str}")
    print(f"\n{len(seen_materials)} unique materials found.")
    print("Use these colors to fill in [[robots.color_mapping]] entries in config.toml.")


def color_distance(c1: tuple[int, int, int], c2: list[int]) -> float:
    """Euclidean distance in RGB space."""
    return float(np.sqrt(sum((a - b) ** 2 for a, b in zip(c1, c2))))


def match_material_type(
    color: tuple[int, int, int],
    color_mapping: list[dict],
) -> str | None:
    """Find the best matching material type for a given RGB color."""
    best_match = None
    best_dist = float("inf")
    for entry in color_mapping:
        dist = color_distance(color, entry["color"])
        if dist < entry["tolerance"] and dist < best_dist:
            best_dist = dist
            best_match = entry["material"]
    return best_match


def apply_pbr_materials(
    meshes: list[bpy.types.Object],
    color_mapping: list[dict],
    materials_config: dict[str, dict],
    cc_textures_dir: str | None,
) -> None:
    """Replace GLTF placeholder materials with PBR textures based on color mapping."""
    cc_materials: dict[str, bproc.types.Material] = {}
    cc_path = resolve_path(Path(cc_textures_dir)) if cc_textures_dir else None
    if cc_path and cc_path.exists():
        needed = {
            cfg["cc_texture"]
            for cfg in materials_config.values()
            if "cc_texture" in cfg
        }
        if needed:
            loaded = bproc.loader.load_ccmaterials(
                str(cc_path), used_assets=list(needed)
            )
            cc_materials = {mat.get_name(): mat for mat in loaded}

    for mesh_obj in meshes:
        bproc_mesh = bproc.types.MeshObject(mesh_obj)
        for bproc_mat in bproc_mesh.get_materials():
            bpy_mat = bproc_mat.blender_obj
            color = get_material_base_color(bpy_mat)
            if color is None:
                continue
            mat_type = match_material_type(color, color_mapping)
            if mat_type is None:
                print(f"  Warning: No mapping for color {color} on {bpy_mat.name}")
                continue
            mat_cfg = materials_config.get(mat_type)
            if mat_cfg is None:
                print(f"  Warning: No material config for type '{mat_type}'")
                continue

            cc_name = mat_cfg.get("cc_texture")
            if cc_name and cc_name in cc_materials:
                bproc_mesh.replace_materials(cc_materials[cc_name])
                print(f"  {bpy_mat.name} -> CC texture '{cc_name}'")
            else:
                bproc_mat.set_principled_shader_value(
                    "Metallic", mat_cfg.get("metallic", 0.0)
                )
                bproc_mat.set_principled_shader_value(
                    "Roughness", mat_cfg.get("roughness", 0.5)
                )
                print(
                    f"  {bpy_mat.name} -> PBR values "
                    f"(metallic={mat_cfg.get('metallic')}, "
                    f"roughness={mat_cfg.get('roughness')})"
                )


def cleanup_meshes(meshes: list[bpy.types.Object]) -> None:
    """Remove imported mesh objects and their data from the Blender scene."""
    for obj in meshes:
        mesh_data = obj.data
        bpy.data.objects.remove(obj, do_unlink=True)
        if mesh_data and mesh_data.users == 0:
            bpy.data.meshes.remove(mesh_data)


def render_preview(
    meshes: list[bpy.types.Object],
    output_path: Path,
    img_w: int = 480,
    img_h: int = 360,
    render_samples: int = 64,
) -> None:
    """Render 6 views (front/back/left/right/top/bottom) as a 3x2 grid."""
    bpy.context.view_layer.update()
    all_pts = [
        obj.matrix_world @ mathutils.Vector(c)
        for obj in meshes
        for c in obj.bound_box
    ]
    xs = [p.x for p in all_pts]
    ys = [p.y for p in all_pts]
    zs = [p.z for p in all_pts]
    center = np.array([
        (min(xs) + max(xs)) / 2,
        (min(ys) + max(ys)) / 2,
        (min(zs) + max(zs)) / 2,
    ])
    extent = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs), 1e-6)
    radius = extent * 2.0

    # (azimuth_deg, elevation_deg, label) — 6 cardinal directions
    view_angles = [
        (0, 15, "Front"),
        (180, 15, "Back"),
        (90, 15, "Left"),
        (270, 15, "Right"),
        (0, 89, "Top"),
        (0, -89, "Bottom"),
    ]

    bproc.camera.set_resolution(img_w, img_h)
    bproc.renderer.set_max_amount_of_samples(render_samples)

    light = bproc.types.Light()
    light.set_type("POINT")
    light.set_energy(300)
    light.set_location([radius * 1.5, -radius * 1.5, radius * 2.0])

    fill = bproc.types.Light()
    fill.set_type("POINT")
    fill.set_energy(100)
    fill.set_location([-radius * 1.5, radius * 1.5, radius * 1.0])

    bproc.utility.reset_keyframes()
    for azimuth_deg, elevation_deg, _label in view_angles:
        az = math.radians(azimuth_deg)
        el = math.radians(elevation_deg)
        cam_pos = center + radius * np.array([
            math.cos(el) * math.cos(az),
            math.cos(el) * math.sin(az),
            math.sin(el),
        ])
        forward = center - cam_pos
        forward = forward / np.linalg.norm(forward)
        rotation = bproc.camera.rotation_from_forward_vec(forward)
        cam2world = bproc.math.build_transformation_mat(cam_pos, rotation)
        bproc.camera.add_camera_pose(cam2world)

    data = bproc.renderer.render()
    frames = []
    font = cv2.FONT_HERSHEY_SIMPLEX
    for img, (_az, _el, label) in zip(data["colors"], view_angles):
        bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.putText(bgr, label, (10, 30), font, 0.8, (255, 255, 255), 2,
                    cv2.LINE_AA)
        cv2.putText(bgr, label, (10, 30), font, 0.8, (0, 0, 0), 1,
                    cv2.LINE_AA)
        frames.append(bgr)

    cols = 3
    rows = 2
    grid_rows = []
    for r in range(rows):
        row_frames = frames[r * cols : r * cols + cols]
        grid_rows.append(np.concatenate(row_frames, axis=1))
    grid = np.concatenate(grid_rows, axis=0)

    out = resolve_path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), grid)
    print(f"\nPreview saved to {out}  ({img_w * cols}x{img_h * rows})")

    for lgt in [light, fill]:
        bpy.data.objects.remove(lgt.blender_obj, do_unlink=True)


_IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".tif", ".webp"}


def _slugify(name: str) -> str:
    """Turn a robot name into a filesystem-safe slug."""
    return name.lower().replace(" ", "_").replace("/", "_")


def _resolve_output_path(
    user_path: Path, robot_name: str, default_ext: str, multi: bool
) -> Path:
    """Turn a user-supplied path into a concrete file path.

    If *user_path* looks like a directory (no image extension or ends with /),
    the output file is placed inside it.  When *multi* is true, the robot name
    is always embedded in the filename.
    """
    is_dir = not user_path.suffix or user_path.suffix.lower() not in _IMAGE_EXTENSIONS
    slug = _slugify(robot_name)
    if is_dir:
        return user_path / f"{slug}{default_ext}"
    if multi:
        return user_path.parent / f"{user_path.stem}_{slug}{user_path.suffix}"
    return user_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path, help="Path to config.toml")
    parser.add_argument(
        "--inspect",
        action="store_true",
        help="Only list material colors, don't apply textures",
    )
    parser.add_argument(
        "--save-blend",
        type=Path,
        default=None,
        help="Save the textured model as a .blend file for inspection "
        "(with multiple robots, the robot name is appended)",
    )
    parser.add_argument(
        "--preview",
        type=Path,
        default=None,
        help="Render 3 off-axis views and save concatenated image "
        "(with multiple robots, the robot name is appended)",
    )
    parser.add_argument(
        "--robot",
        type=str,
        default=None,
        help="Process only the robot with this name (default: all)",
    )
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else sys.argv[1:]
    args = parser.parse_args(argv)

    config = load_config(args.config)
    robot_configs = config["robots"]
    cc_textures_dir = config.get("environment", {}).get("cc_textures_dir")

    if args.robot:
        robot_configs = [
            r for r in robot_configs
            if r.get("name", Path(r["model_path"]).stem) == args.robot
        ]
        if not robot_configs:
            available = [
                r.get("name", Path(r["model_path"]).stem) for r in config["robots"]
            ]
            parser.error(f"Robot '{args.robot}' not found. Available: {available}")

    multi = len(robot_configs) > 1

    bproc.init()

    for rcfg in robot_configs:
        rname = rcfg.get("name", Path(rcfg["model_path"]).stem)
        model_path = Path(rcfg["model_path"])
        scale = rcfg.get("scale", 1.0)

        print(f"\n{'='*60}")
        print(f"Robot: {rname}")
        print(f"{'='*60}")

        if args.inspect:
            inspect_model(model_path, scale)
            continue

        print(f"Loading robot model from {model_path}")
        meshes = import_gltf(model_path, scale)
        print(f"Loaded {len(meshes)} mesh parts")

        print("\nApplying PBR materials:")
        apply_pbr_materials(
            meshes,
            rcfg["color_mapping"],
            config["materials"],
            cc_textures_dir,
        )

        if args.preview:
            out = _resolve_output_path(args.preview, rname, ".jpg", multi)
            render_preview(meshes, out)
            bproc.utility.reset_keyframes()

        if args.save_blend:
            out = _resolve_output_path(args.save_blend, rname, ".blend", multi)
            bpy.ops.wm.save_as_mainfile(filepath=str(resolve_path(out)))
            print(f"\nSaved textured model to {out}")

        cleanup_meshes(meshes)

    if not args.inspect and not args.preview and not args.save_blend:
        print("\nDone. Use --save-blend or --preview to output results.")


if __name__ == "__main__":
    main()
