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

    source_node = bc_input.links[0].from_node
    source_color = _color_from_node(source_node)
    if source_color is not None:
        return source_color

    # Non-constant source (image texture, procedural, etc.) — the material has
    # intentional visual content that shouldn't be remapped.
    return None


def _xterm256_index_to_rgb(i: int) -> tuple[int, int, int]:
    """RGB for xterm 256-color index 0..255."""
    if i < 16:
        ansi = [
            (0, 0, 0),
            (205, 0, 0),
            (0, 205, 0),
            (205, 205, 0),
            (0, 0, 238),
            (205, 0, 205),
            (0, 205, 205),
            (229, 229, 229),
            (127, 127, 127),
            (255, 0, 0),
            (0, 255, 0),
            (255, 255, 0),
            (92, 92, 255),
            (255, 0, 255),
            (0, 255, 255),
            (255, 255, 255),
        ]
        return ansi[i]
    if i >= 232:
        g = 8 + (i - 232) * 10
        return (g, g, g)
    i -= 16
    r = i // 36
    i %= 36
    g = i // 6
    b = i % 6
    levels = [0, 95, 135, 175, 215, 255]
    return (levels[r], levels[g], levels[b])


def _nearest_xterm256(r: int, g: int, b: int) -> int:
    """Pick the closest xterm 256-color index to RGB (0-255 per channel)."""
    best_i = 0
    best_d = float("inf")
    for i in range(256):
        cr, cg, cb = _xterm256_index_to_rgb(i)
        d = (cr - r) ** 2 + (cg - g) ** 2 + (cb - b) ** 2
        if d < best_d:
            best_d = d
            best_i = i
    return best_i


def _ansi_fg_xterm256(idx: int) -> str:
    return f"\033[38;5;{idx}m"


_ANSI_RESET = "\033[0m"


def _terminal_color_enabled() -> bool:
    return sys.stdout.isatty() and not os.environ.get("NO_COLOR", "")


def _describe_base_color_source(mat: bpy.types.Material) -> str:
    """Return a short description of how the Base Color is wired."""
    if not mat.use_nodes:
        return "no nodes"
    bsdf = None
    for node in mat.node_tree.nodes:
        if node.type == "BSDF_PRINCIPLED":
            bsdf = node
            break
    if bsdf is None:
        return "no Principled BSDF"
    bc_input = bsdf.inputs["Base Color"]
    if not bc_input.links:
        return "direct value"
    from_node = bc_input.links[0].from_node
    return f"linked <- {from_node.type} ({from_node.name})"


def inspect_model(model_path: Path, scale: float = 1.0) -> None:
    """Print material names, base colors (with nearest xterm-256 highlight), and wiring."""
    meshes = import_gltf(model_path, scale)
    print(f"\nFound {len(meshes)} mesh objects in {model_path.name}:\n")
    seen_materials: set[str] = set()
    use_color = _terminal_color_enabled()
    for mesh_obj in meshes:
        for slot in mesh_obj.material_slots:
            mat = slot.material
            if mat is None or mat.name in seen_materials:
                continue
            seen_materials.add(mat.name)
            color = get_material_base_color(mat)
            source = _describe_base_color_source(mat)
            if color:
                r, g, b = color
                idx = _nearest_xterm256(r, g, b)
                color_str = f"[{r}, {g}, {b}]"
                if use_color:
                    swatch = f"{_ansi_fg_xterm256(idx)}██{_ANSI_RESET} {color_str}"
                else:
                    swatch = f"{color_str}"
            else:
                swatch = "N/A"
            print(f"  Material: {mat.name:30s}  Color: {swatch}  ({source})")
    print(f"\n{len(seen_materials)} unique materials found.")
    print(
        "Use these colors to fill in [[robots.color_mapping]] entries in config.toml."
    )


def color_distance(c1: tuple[int, int, int], c2: list[int]) -> float:
    """Euclidean distance in RGB space."""
    return float(np.sqrt(sum((a - b) ** 2 for a, b in zip(c1, c2))))


def match_material_type(
    color: tuple[int, int, int],
    color_mapping: list[dict],
) -> tuple[str | None, float, bool]:
    """Find the best material type for an RGB color.

    Matching strategy:
    1) Use explicit color_mapping entries within their configured tolerance.
    2) If no explicit entry matches, allow a nearest-color fallback when the
       closest mapped color is still reasonably close.

    Returns:
        (material_name, distance, used_fallback)
    """
    best_match = None
    best_dist = float("inf")
    nearest_match = None
    nearest_dist = float("inf")
    for entry in color_mapping:
        dist = color_distance(color, entry["color"])
        if dist < nearest_dist:
            nearest_dist = dist
            nearest_match = entry["material"]
        if dist <= entry["tolerance"] and dist < best_dist:
            best_dist = dist
            best_match = entry["material"]

    if best_match is not None:
        return best_match, best_dist, False

    # CAD exports sometimes introduce tiny color variants for what is
    # semantically the same material.  Keep this conservative to avoid
    # accidentally remapping unrelated colors.
    fallback_max_dist = 45.0
    if nearest_match is not None and nearest_dist <= fallback_max_dist:
        return nearest_match, nearest_dist, True

    return None, nearest_dist, False


_PBR_SUFFIXES = [
    "Color",
    "Roughness",
    "Metalness",
    "NormalGL",
    "Normal",
    "Displacement",
]


def _find_texture_file(texture_dir: Path, suffix: str) -> Path | None:
    """Find a texture file in *texture_dir* whose name contains *suffix*."""
    for ext in (".jpg", ".jpeg", ".png", ".exr", ".tiff"):
        for p in texture_dir.iterdir():
            if p.suffix.lower() == ext and suffix.lower() in p.stem.lower():
                return p
    return None


def _add_tex_node(
    tree: bpy.types.NodeTree,
    image_path: Path,
    non_color: bool = False,
) -> bpy.types.ShaderNodeTexImage:
    """Create an Image Texture node loaded from *image_path*."""
    img = bpy.data.images.load(str(image_path))
    tex = tree.nodes.new("ShaderNodeTexImage")
    tex.image = img
    if non_color:
        img.colorspace_settings.name = "Non-Color"
    return tex


def _apply_pbr_textures(bpy_mat: bpy.types.Material, texture_dir: Path) -> None:
    """Load PBR image textures from *texture_dir* and wire them into the material.

    Scans for files containing standard PBR suffixes (Color, Roughness,
    Metalness, NormalGL, Displacement) and connects them to the corresponding
    Principled BSDF inputs.  Uses the model's existing UV mapping.
    """
    if not bpy_mat.use_nodes:
        bpy_mat.use_nodes = True
    tree = bpy_mat.node_tree
    bsdf = next((n for n in tree.nodes if n.type == "BSDF_PRINCIPLED"), None)
    if bsdf is None:
        print(f"  Warning: No Principled BSDF in {bpy_mat.name}, skipping textures")
        return

    def _disconnect(input_name: str) -> None:
        if input_name in bsdf.inputs:
            for link in list(bsdf.inputs[input_name].links):
                tree.links.remove(link)

    applied_channels: list[str] = []

    color_file = _find_texture_file(texture_dir, "Color")
    if color_file:
        _disconnect("Base Color")
        tex = _add_tex_node(tree, color_file)
        tree.links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
        applied_channels.append("Color")

    rough_file = _find_texture_file(texture_dir, "Roughness")
    if rough_file:
        _disconnect("Roughness")
        tex = _add_tex_node(tree, rough_file, non_color=True)
        tree.links.new(tex.outputs["Color"], bsdf.inputs["Roughness"])
        applied_channels.append("Roughness")

    metal_file = _find_texture_file(texture_dir, "Metalness")
    if metal_file:
        _disconnect("Metallic")
        tex = _add_tex_node(tree, metal_file, non_color=True)
        tree.links.new(tex.outputs["Color"], bsdf.inputs["Metallic"])
        applied_channels.append("Metalness")

    normal_file = _find_texture_file(texture_dir, "NormalGL") or _find_texture_file(
        texture_dir, "Normal"
    )
    if normal_file:
        _disconnect("Normal")
        tex = _add_tex_node(tree, normal_file, non_color=True)
        normal_map = tree.nodes.new("ShaderNodeNormalMap")
        tree.links.new(tex.outputs["Color"], normal_map.inputs["Color"])
        tree.links.new(normal_map.outputs["Normal"], bsdf.inputs["Normal"])
        applied_channels.append("Normal")

    disp_file = _find_texture_file(texture_dir, "Displacement")
    if disp_file:
        tex = _add_tex_node(tree, disp_file, non_color=True)
        disp_node = tree.nodes.new("ShaderNodeDisplacement")
        tree.links.new(tex.outputs["Color"], disp_node.inputs["Height"])
        mat_output = next((n for n in tree.nodes if n.type == "OUTPUT_MATERIAL"), None)
        if mat_output and "Displacement" in mat_output.inputs:
            for link in list(mat_output.inputs["Displacement"].links):
                tree.links.remove(link)
            tree.links.new(
                disp_node.outputs["Displacement"], mat_output.inputs["Displacement"]
            )
        applied_channels.append("Displacement")

    if not applied_channels:
        print(
            f"  Warning: No matching texture files found in {texture_dir} for {bpy_mat.name}"
        )


def _load_cc_materials(
    materials_config: dict[str, dict],
    cc_textures_dir: str | None,
) -> dict[str, bproc.types.Material]:
    """Load and cache configured CC materials once per script run."""
    cc_materials: dict[str, bproc.types.Material] = {}
    cc_path = resolve_path(Path(cc_textures_dir)) if cc_textures_dir else None
    if not (cc_path and cc_path.exists()):
        return cc_materials

    needed = {
        cfg["cc_texture"] for cfg in materials_config.values() if "cc_texture" in cfg
    }
    if not needed:
        return cc_materials

    loaded = bproc.loader.load_ccmaterials(str(cc_path), used_assets=list(needed))
    cc_materials = {mat.get_name(): mat for mat in loaded}

    # BlenderProc may return only newly loaded assets; include already-loaded
    # materials from bpy as a fallback so multi-robot runs can reuse them.
    for name in needed:
        if name in cc_materials:
            continue
        existing = bpy.data.materials.get(name)
        if existing is not None:
            cc_materials[name] = bproc.types.Material(existing)
    return cc_materials


def _find_cc_material(
    name: str, cc_materials: dict[str, bproc.types.Material]
) -> bproc.types.Material | None:
    """Find a CC material by exact name, then by substring match."""
    if name in cc_materials:
        return cc_materials[name]
    lowered = name.lower()
    for key, mat in cc_materials.items():
        if lowered in key.lower():
            return mat
    return None


def _material_has_image_textures(mat: bpy.types.Material) -> bool:
    """True when a material node tree contains at least one image texture node."""
    if not mat.use_nodes:
        return False
    return any(
        node.type == "TEX_IMAGE" and getattr(node, "image", None) is not None
        for node in mat.node_tree.nodes
    )


def apply_pbr_materials(
    meshes: list[bpy.types.Object],
    color_mapping: list[dict],
    materials_config: dict[str, dict],
    cc_textures_dir: str | None,
    cc_materials_cache: dict[str, bproc.types.Material],
) -> None:
    """Replace GLTF placeholder materials with PBR textures based on color mapping."""
    # Keep prior behavior (load during apply) while preserving materials across
    # multiple robots in one run.
    loaded_this_pass = _load_cc_materials(materials_config, cc_textures_dir)
    if loaded_this_pass:
        cc_materials_cache.update(loaded_this_pass)

    for mesh_obj in meshes:
        bproc_mesh = bproc.types.MeshObject(mesh_obj)
        for slot_idx, bproc_mat in enumerate(bproc_mesh.get_materials()):
            bpy_mat = bproc_mat.blender_obj
            color = get_material_base_color(bpy_mat)
            if color is None:
                continue
            mat_type, match_dist, used_fallback = match_material_type(
                color, color_mapping
            )
            if mat_type is None:
                print(
                    f"  Warning: No mapping for color {color} on {bpy_mat.name} "
                    f"(nearest distance={match_dist:.2f})"
                )
                continue
            if used_fallback:
                print(
                    f"  Note: Fallback mapped color {color} on {bpy_mat.name} "
                    f"to '{mat_type}' (distance={match_dist:.2f}). "
                    "Consider adding an explicit [[robots.color_mapping]] entry."
                )
            mat_cfg = materials_config.get(mat_type)
            if mat_cfg is None:
                print(f"  Warning: No material config for type '{mat_type}'")
                continue

            tex_dir = mat_cfg.get("texture_dir")
            cc_name = mat_cfg.get("cc_texture")

            if tex_dir:
                td = resolve_path(Path(tex_dir))
                if td.is_dir():
                    _apply_pbr_textures(bpy_mat, td)
                    print(f"  {bpy_mat.name} -> PBR textures from '{td.name}/'")
                else:
                    print(f"  Warning: texture_dir not found: {td}")
            elif cc_name:
                cc_mat = _find_cc_material(cc_name, cc_materials_cache)
                if cc_mat is not None:
                    assigned_mat = cc_mat.blender_obj
                    if _material_has_image_textures(assigned_mat):
                        mesh_obj.data.materials[slot_idx] = assigned_mat
                        print(f"  {bpy_mat.name} -> CC texture '{cc_name}'")
                        continue

                    # Some loaded CC entries can still be effectively flat in
                    # specific Blender/asset combinations. Fall back to wiring
                    # maps directly from the texture directory on the current
                    # slot material.
                    cc_dir = (
                        resolve_path(Path(cc_textures_dir)) / cc_name
                        if cc_textures_dir
                        else None
                    )
                    if cc_dir and cc_dir.is_dir():
                        _apply_pbr_textures(bpy_mat, cc_dir)
                        print(
                            f"  {bpy_mat.name} -> CC fallback maps from '{cc_dir.name}/' "
                            "(assigned CC material had no image nodes)"
                        )
                    else:
                        print(
                            f"  Warning: CC material '{cc_name}' on {bpy_mat.name} "
                            "has no image texture nodes and no fallback directory was found."
                        )
                else:
                    print(
                        f"  Warning: cc_texture '{cc_name}' not loaded for "
                        f"{bpy_mat.name}; falling back to PBR scalar values."
                    )
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
        obj.matrix_world @ mathutils.Vector(c) for obj in meshes for c in obj.bound_box
    ]
    xs = [p.x for p in all_pts]
    ys = [p.y for p in all_pts]
    zs = [p.z for p in all_pts]
    center = np.array(
        [
            (min(xs) + max(xs)) / 2,
            (min(ys) + max(ys)) / 2,
            (min(zs) + max(zs)) / 2,
        ]
    )
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
        cam_pos = center + radius * np.array(
            [
                math.cos(el) * math.cos(az),
                math.cos(el) * math.sin(az),
                math.sin(el),
            ]
        )
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
        cv2.putText(bgr, label, (10, 30), font, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(bgr, label, (10, 30), font, 0.8, (0, 0, 0), 1, cv2.LINE_AA)
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


_KNOWN_OUTPUT_EXTENSIONS = {
    ".jpg",
    ".jpeg",
    ".png",
    ".bmp",
    ".tiff",
    ".tif",
    ".webp",
    ".blend",
    ".glb",
    ".gltf",
}


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
    is_dir = (
        not user_path.suffix or user_path.suffix.lower() not in _KNOWN_OUTPUT_EXTENSIONS
    )
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
        "--preview-output",
        type=Path,
        default=Path("../data/previews"),
        help="Output path for preview image(s) (default: previews/ directory); "
        "render 3 off-axis views and save concatenated image "
        "(with multiple robots, the robot name is appended)",
    )
    parser.add_argument(
        "--skip-preview",
        action="store_true",
        help="Skip rendering preview images",
    )
    parser.add_argument(
        "--robot",
        type=str,
        default=None,
        help="Process only the robot with this name (default: all)",
    )
    parser.add_argument(
        "--export-gltf",
        type=Path,
        default=None,
        help="Export the textured model as a .glb file "
        "(directory or file path; robot name appended when processing multiple)",
    )
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else sys.argv[1:]
    args = parser.parse_args(argv)

    config = load_config(args.config)
    robot_configs = config["robots"]
    cc_textures_dir = config.get("environment", {}).get("cc_textures_dir")

    if args.robot:
        robot_configs = [
            r
            for r in robot_configs
            if r.get("name", Path(r["model_path"]).stem) == args.robot
        ]
        if not robot_configs:
            available = [
                r.get("name", Path(r["model_path"]).stem) for r in config["robots"]
            ]
            parser.error(f"Robot '{args.robot}' not found. Available: {available}")

    multi = len(robot_configs) > 1

    bproc.init()
    cc_materials_cache: dict[str, bproc.types.Material] = {}

    for rcfg in robot_configs:
        rname = rcfg.get("name", Path(rcfg["model_path"]).stem)
        model_path = Path(rcfg["model_path"])
        scale = rcfg.get("scale", 1.0)

        print(f"\n{'=' * 60}")
        print(f"Robot: {rname}")
        print(f"{'=' * 60}")

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
            cc_materials_cache,
        )

        if not args.skip_preview:
            out = _resolve_output_path(args.preview_output, rname, ".jpg", multi)
            render_preview(meshes, out)
            bproc.utility.reset_keyframes()
            print(f"Saving preview to {out}")

        if args.save_blend:
            out = _resolve_output_path(args.save_blend, rname, ".blend", multi)
            bpy.ops.wm.save_as_mainfile(filepath=str(resolve_path(out)))
            print(f"\nSaved textured model to {out}")

        if args.export_gltf:
            out = _resolve_output_path(args.export_gltf, rname, ".glb", multi)
            out = resolve_path(out)
            out.parent.mkdir(parents=True, exist_ok=True)
            # Select only this robot's meshes for export
            bpy.ops.object.select_all(action="DESELECT")
            for obj in meshes:
                obj.select_set(True)
            bpy.ops.export_scene.gltf(
                filepath=str(out),
                use_selection=True,
                export_format="GLB",
                export_image_format="AUTO",
                export_materials="EXPORT",
                export_texcoords=True,
                export_normals=True,
            )
            print(f"\nExported textured GLTF to {out}")

        cleanup_meshes(meshes)

    if (
        not args.inspect
        and args.skip_preview
        and not args.save_blend
        and not args.export_gltf
    ):
        print(
            "\nDone. Use --preview-output, --save-blend, or --export-gltf to output results."
        )


if __name__ == "__main__":
    main()
