import blenderproc as bproc  # isort: skip  # must be first import for blenderproc run

import argparse
import csv
import gc
import math
import os
import random
import sys
import tomllib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import bpy
import cv2
import mathutils
import numpy as np
from bpy_extras.object_utils import world_to_camera_view

_LAUNCH_CWD = Path(os.environ.get("BLENDERPROC_CWD", os.getcwd()))
_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_CONFIG_BASE_DIR: Path | None = None

ANNOTATION_MODE_KEYPOINTS_BBOX = "keypoints_bbox"
ANNOTATION_MODE_SEGMENTATION_BBOX = "segmentation_bbox"

BACKGROUND_CATEGORY_ID = 0
ROBOT_CATEGORY_ID = 1
DISTRACTOR_CATEGORY_ID = 2

SEG_FLOOR_CLASS_ID = 1
SEG_OBJECT_CLASS_ID = 2

MODEL_EXTENSIONS = {".glb", ".gltf", ".obj", ".ply"}


@dataclass
class RobotInstance:
    """All state for one loaded robot model."""

    name: str
    instance_id: int
    meshes: list[bproc.types.MeshObject]
    parent: bpy.types.Object
    bbox: list[mathutils.Vector]
    size: float
    kp_front: np.ndarray
    kp_back: np.ndarray
    class_id: int
    weight: float = 1.0
    config: dict = field(default_factory=dict)


def hide_robot(robot: RobotInstance) -> None:
    """Hide a robot from rendering and viewport."""
    for mesh in robot.meshes:
        mesh.blender_obj.hide_render = True
        mesh.blender_obj.hide_viewport = True


def show_robot(robot: RobotInstance) -> None:
    """Make a robot visible for rendering and viewport."""
    for mesh in robot.meshes:
        mesh.blender_obj.hide_render = False
        mesh.blender_obj.hide_viewport = False


def choose_scene_robots(
    robots: list[RobotInstance], max_count: int
) -> list[RobotInstance]:
    """Weighted sampling without replacement to pick 1..max_count robots."""
    max_count = min(max_count, len(robots))
    num = random.randint(1, max_count)
    if num >= len(robots):
        return list(robots)
    pool = list(robots)
    weights = [r.weight for r in pool]
    selected: list[RobotInstance] = []
    for _ in range(num):
        chosen = random.choices(pool, weights=weights, k=1)[0]
        idx = pool.index(chosen)
        selected.append(chosen)
        pool.pop(idx)
        weights.pop(idx)
    return selected


def resolve_path(path: Path) -> Path:
    """Resolve relative paths using config dir, launch CWD, then project root."""
    if path.is_absolute():
        return path
    candidates: list[Path] = []
    if _CONFIG_BASE_DIR is not None:
        candidates.append((_CONFIG_BASE_DIR / path).resolve())
    candidates.append((_LAUNCH_CWD / path).resolve())
    candidates.append((_PROJECT_ROOT / path).resolve())

    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def load_config(config_path: Path) -> dict:
    resolved_config = resolve_path(config_path)
    global _CONFIG_BASE_DIR
    _CONFIG_BASE_DIR = resolved_config.parent
    with open(resolved_config, "rb") as f:
        return tomllib.load(f)


def model_to_blender_local(vec: list[float] | np.ndarray) -> np.ndarray:
    """Convert model-space keypoints (OnShape/GLTF) into Blender local axes.

    GLTF import applies axis conversion (Y-up -> Z-up), so keypoints authored in
    the model's native frame must be remapped before projection/visibility tests.
    """
    x, y, z = [float(v) for v in vec]
    return np.array([x, -z, y], dtype=float)


# ---------------------------------------------------------------------------
# GLTF import and material helpers
# ---------------------------------------------------------------------------


def _color_from_node(node: bpy.types.ShaderNode) -> tuple[int, int, int] | None:
    """Read a constant RGB value from node types where default_value is reliable."""
    if node.type == "RGB":
        c = node.outputs[0].default_value
        return (int(c[0] * 255), int(c[1] * 255), int(c[2] * 255))
    return None


def _color_from_material_name(name: str) -> tuple[int, int, int] | None:
    """Parse RGB from material names like '0.501961_0.501961_0.501961_...'"""
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


def color_distance(c1: tuple[int, int, int], c2: list[int]) -> float:
    return float(np.sqrt(sum((a - b) ** 2 for a, b in zip(c1, c2))))


def match_material_type(
    color: tuple[int, int, int], color_mapping: list[dict]
) -> tuple[str | None, float, bool]:
    """Return best material for a color.

    Strategy:
    1) Exact/near match within configured tolerance (inclusive).
    2) Conservative nearest-color fallback when nothing matches tolerance.

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

    # CAD exports can introduce close shade variants for the same part color.
    fallback_max_dist = 45.0
    if nearest_match is not None and nearest_dist <= fallback_max_dist:
        return nearest_match, nearest_dist, True
    return None, nearest_dist, False


def import_gltf_as_robot(
    model_path: Path, scale: float = 1.0, category_id: int = ROBOT_CATEGORY_ID
) -> tuple[list[bproc.types.MeshObject], bpy.types.Object, list[mathutils.Vector]]:
    """Import GLTF and parent all parts under an empty for group transforms.

    Returns (mesh_objects, parent_empty, bbox_corners) where bbox_corners are
    the 8 corners of the robot's axis-aligned bounding box in parent-rest-pose
    world space (used to compute ground clearance for arbitrary orientations).
    """
    model_path = resolve_path(model_path)
    if not model_path.exists():
        raise FileNotFoundError(
            f"Model file not found: {model_path}\n  Launch CWD was: {_LAUNCH_CWD}"
        )
    bpy.ops.import_scene.gltf(filepath=str(model_path))
    bpy_meshes = [o for o in bpy.context.selected_objects if o.type == "MESH"]
    if not bpy_meshes:
        raise RuntimeError(f"No meshes found in {model_path}")

    # Capture world transforms before re-parenting (the GLTF hierarchy uses
    # intermediate empties for positioning; re-parenting directly would lose them).
    bpy.context.view_layer.update()
    saved_world = {obj.name: obj.matrix_world.copy() for obj in bpy_meshes}

    parent = bpy.data.objects.new("robot_parent", None)
    bpy.context.scene.collection.objects.link(parent)
    if scale != 1.0:
        parent.scale = (scale, scale, scale)
    bpy.context.view_layer.update()

    for obj in bpy_meshes:
        mat = saved_world[obj.name]
        obj.parent = parent
        obj.matrix_parent_inverse = mathutils.Matrix.Identity(4)
        obj.matrix_world = mat

    bpy.context.view_layer.update()

    all_pts = [
        obj.matrix_world @ mathutils.Vector(c)
        for obj in bpy_meshes
        for c in obj.bound_box
    ]
    xs = [p.x for p in all_pts]
    ys = [p.y for p in all_pts]
    zs = [p.z for p in all_pts]
    bbox_corners = [
        mathutils.Vector((x, y, z))
        for x in [min(xs), max(xs)]
        for y in [min(ys), max(ys)]
        for z in [min(zs), max(zs)]
    ]
    print(
        f"  Robot AABB: x=[{min(xs):.4f},{max(xs):.4f}] "
        f"y=[{min(ys):.4f},{max(ys):.4f}] "
        f"z=[{min(zs):.4f},{max(zs):.4f}]"
    )

    bproc_meshes = []
    for obj in bpy_meshes:
        mesh = bproc.types.MeshObject(obj)
        mesh.set_cp("category_id", category_id)
        mesh.set_cp("is_distractor", 0)
        bproc_meshes.append(mesh)

    return bproc_meshes, parent, bbox_corners


def compute_ground_z(
    robot_meshes: list[bproc.types.MeshObject],
    robot_parent: bpy.types.Object,
    rotation_euler: tuple,
) -> float:
    """Place the robot at origin with the given rotation and return the z
    offset that puts its lowest point exactly on the ground plane."""
    robot_parent.location = mathutils.Vector((0, 0, 0))
    robot_parent.rotation_euler = mathutils.Euler(rotation_euler)
    bpy.context.view_layer.update()

    min_z = float("inf")
    for m in robot_meshes:
        obj = m.blender_obj
        for c in obj.bound_box:
            z = (obj.matrix_world @ mathutils.Vector(c)).z
            if z < min_z:
                min_z = z
    return -min_z


def tint_material_albedo(bpy_mat: bpy.types.Material, color_rgb: list[int]) -> None:
    """Tint a material's albedo by multiplying its Base Color texture with a color.

    If no texture is connected, sets the Base Color directly.
    """
    if not bpy_mat.use_nodes:
        return

    tree = bpy_mat.node_tree
    bsdf = next((n for n in tree.nodes if n.type == "BSDF_PRINCIPLED"), None)
    if bsdf is None:
        return

    bc_input = bsdf.inputs["Base Color"]
    tint = [c / 255.0 for c in color_rgb] + [1.0]

    if not bc_input.links:
        bc_input.default_value = tint
        print(f"    Tinted {bpy_mat.name}: set flat color {color_rgb}")
        return

    from_socket = bc_input.links[0].from_socket
    tree.links.remove(bc_input.links[0])

    mix = tree.nodes.new("ShaderNodeMix")
    mix.data_type = "RGBA"
    mix.blend_type = "MULTIPLY"
    mix.inputs[0].default_value = 1.0

    color_inputs = [s for s in mix.inputs if s.type == "RGBA"]
    color_outputs = [s for s in mix.outputs if s.type == "RGBA"]

    if len(color_inputs) >= 2 and len(color_outputs) >= 1:
        tree.links.new(from_socket, color_inputs[0])
        color_inputs[1].default_value = tint
        tree.links.new(color_outputs[0], bc_input)
        print(
            f"    Tinted {bpy_mat.name}: MULTIPLY with {color_rgb} "
            f"({len(color_inputs)} color inputs, {len(color_outputs)} color outputs)"
        )
    else:
        tree.links.new(from_socket, bc_input)
        print(
            f"    WARNING: Mix node has {len(color_inputs)} RGBA inputs, "
            f"{len(color_outputs)} RGBA outputs — tint skipped, reconnected original"
        )


def _find_cc_material(
    name: str, cc_materials: dict[str, bproc.types.Material]
) -> bproc.types.Material | None:
    """Find a CC material by exact name, then by substring match."""
    if name in cc_materials:
        return cc_materials[name]
    for key, mat in cc_materials.items():
        if name.lower() in key.lower():
            return mat
    return None


def _load_cc_materials(
    materials_config: dict[str, dict],
    cc_textures_dir: str | None,
) -> dict[str, bproc.types.Material]:
    """Load configured CC materials once and return lookup map."""
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

    # BlenderProc may return only newly loaded assets on subsequent calls.
    for name in needed:
        if name in cc_materials:
            continue
        existing = bpy.data.materials.get(name)
        if existing is not None:
            cc_materials[name] = bproc.types.Material(existing)
    return cc_materials


def _disconnect_base_color(bpy_mat: bpy.types.Material) -> None:
    """Remove any node links feeding into the Principled BSDF Base Color input."""
    if not bpy_mat.use_nodes:
        return
    for node in bpy_mat.node_tree.nodes:
        if node.type == "BSDF_PRINCIPLED":
            for link in list(node.inputs["Base Color"].links):
                bpy_mat.node_tree.links.remove(link)
            break


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

    color_file = _find_texture_file(texture_dir, "Color")
    if color_file:
        _disconnect("Base Color")
        tex = _add_tex_node(tree, color_file)
        tree.links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])

    rough_file = _find_texture_file(texture_dir, "Roughness")
    if rough_file:
        _disconnect("Roughness")
        tex = _add_tex_node(tree, rough_file, non_color=True)
        tree.links.new(tex.outputs["Color"], bsdf.inputs["Roughness"])

    metal_file = _find_texture_file(texture_dir, "Metalness")
    if metal_file:
        _disconnect("Metallic")
        tex = _add_tex_node(tree, metal_file, non_color=True)
        tree.links.new(tex.outputs["Color"], bsdf.inputs["Metallic"])

    normal_file = _find_texture_file(texture_dir, "NormalGL") or _find_texture_file(
        texture_dir, "Normal"
    )
    if normal_file:
        _disconnect("Normal")
        tex = _add_tex_node(tree, normal_file, non_color=True)
        normal_map = tree.nodes.new("ShaderNodeNormalMap")
        tree.links.new(tex.outputs["Color"], normal_map.inputs["Color"])
        tree.links.new(normal_map.outputs["Normal"], bsdf.inputs["Normal"])

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


def apply_pbr_materials(
    meshes: list[bproc.types.MeshObject],
    color_mapping: list[dict],
    materials_config: dict[str, dict],
    cc_materials: dict[str, bproc.types.Material],
) -> None:
    applied: dict[str, int] = {}
    for mesh in meshes:
        bpy_obj = mesh.blender_obj
        for slot_idx, bproc_mat in enumerate(mesh.get_materials()):
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
            mat_cfg = materials_config.get(mat_type, {})

            tex_dir = mat_cfg.get("texture_dir")
            cc_name = mat_cfg.get("cc_texture")
            base_color_rgb = mat_cfg.get("base_color")
            cc_mat = _find_cc_material(cc_name, cc_materials) if cc_name else None

            if tex_dir:
                td = resolve_path(Path(tex_dir))
                if td.is_dir():
                    _apply_pbr_textures(bpy_mat, td)
                else:
                    print(f"  Warning: texture_dir not found: {td}")
            elif cc_mat is not None:
                cc_bpy_mat = cc_mat.blender_obj
                if base_color_rgb:
                    cc_bpy_mat = cc_bpy_mat.copy()
                    tint_material_albedo(cc_bpy_mat, base_color_rgb)
                bpy_obj.data.materials[slot_idx] = cc_bpy_mat
            else:
                bproc_mat.set_principled_shader_value(
                    "Metallic", mat_cfg.get("metallic", 0.0)
                )
                bproc_mat.set_principled_shader_value(
                    "Roughness", mat_cfg.get("roughness", 0.5)
                )
                if base_color_rgb:
                    _disconnect_base_color(bpy_mat)
                    rgba = [c / 255.0 for c in base_color_rgb] + [1.0]
                    bproc_mat.set_principled_shader_value("Base Color", rgba)

            applied[mat_type] = applied.get(mat_type, 0) + 1

    print(f"  Material applications: {applied}")


# ---------------------------------------------------------------------------
# Distractor loading
# ---------------------------------------------------------------------------


def discover_model_files(
    sources: list[dict],
) -> list[tuple[list[Path], float, Path, str]]:
    """Find loadable 3D model files grouped by source directory.

    *sources* is a list of dicts with ``"path"`` and ``"weight"`` keys.
    Returns ``[(files, weight, source_dir, source_kind), ...]`` for each source that
    has files.
    """
    groups: list[tuple[list[Path], float, Path, str]] = []
    for src in sources:
        d = resolve_path(Path(src["path"]))
        weight = float(src.get("weight", 1.0))
        if not d.exists():
            continue
        source_kind = str(src.get("kind", "")).strip().lower()
        if not source_kind:
            source_kind = "objaverse" if "objaverse" in d.name.lower() else "cad"
        files: list[Path] = []
        for ext in MODEL_EXTENSIONS:
            files.extend(d.glob(f"*{ext}"))
        if files:
            groups.append((files, weight, d, source_kind))
    return groups


DistractorGroup = tuple[bpy.types.Object, list[bproc.types.MeshObject], float]
"""(parent_empty, meshes, native_max_dimension)"""


def _estimate_image_gpu_bytes(img: bpy.types.Image) -> int:
    """Approximate GPU memory footprint (bytes) for one Blender image."""
    w, h = int(img.size[0]), int(img.size[1])
    if w <= 0 or h <= 0:
        return 0

    channels = int(getattr(img, "channels", 4) or 4)
    channels = max(1, channels)
    bytes_per_channel = 4 if getattr(img, "is_float", False) else 1
    base_level = w * h * channels * bytes_per_channel
    # Include an approximate full mip chain cost.
    return int(base_level * (4.0 / 3.0))


def estimate_distractor_group_gpu_mb(group: DistractorGroup) -> float:
    """Estimate the distractor's VRAM footprint from mesh + texture data."""
    _parent, meshes, _ = group
    seen_mesh_data: set[int] = set()
    seen_images: set[int] = set()
    total_bytes = 0

    for mesh in meshes:
        obj = mesh.blender_obj
        mesh_data = obj.data
        if mesh_data is not None:
            mesh_ptr = mesh_data.as_pointer()
            if mesh_ptr not in seen_mesh_data:
                seen_mesh_data.add(mesh_ptr)
                mesh_data.calc_loop_triangles()
                vertex_count = len(mesh_data.vertices)
                tri_count = len(mesh_data.loop_triangles)

                # Approximate attribute payload per vertex.
                uv_bytes = 8 if mesh_data.uv_layers else 0
                color_bytes = (
                    4 if (mesh_data.color_attributes or mesh_data.vertex_colors) else 0
                )
                total_bytes += vertex_count * (12 + 12 + uv_bytes + color_bytes)
                # 3 uint32 indices per triangle.
                total_bytes += tri_count * 12

        for slot in obj.material_slots:
            mat = slot.material
            if not mat or not mat.use_nodes or not mat.node_tree:
                continue
            for node in mat.node_tree.nodes:
                if node.type != "TEX_IMAGE":
                    continue
                img = getattr(node, "image", None)
                if img is None:
                    continue
                img_ptr = img.as_pointer()
                if img_ptr in seen_images:
                    continue
                seen_images.add(img_ptr)
                total_bytes += _estimate_image_gpu_bytes(img)

    return total_bytes / (1024**2)


def load_distractor_vram_audit(csv_path: Path) -> dict[Path, float]:
    """Load precomputed distractor VRAM estimates keyed by absolute model path."""
    resolved = resolve_path(csv_path)
    if not resolved.exists():
        print(f"  VRAM audit CSV not found: {resolved}")
        return {}

    estimates: dict[Path, float] = {}
    try:
        with resolved.open(newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                file_str = row.get("file")
                mb_str = row.get("total_gpu_mb_est")
                if not file_str or not mb_str:
                    continue
                try:
                    key = resolve_path(Path(file_str))
                    estimates[key] = float(mb_str)
                except Exception:
                    continue
    except Exception as e:
        print(f"  Warning: failed to parse VRAM audit CSV {resolved}: {e}")
        return {}

    print(f"  Loaded VRAM audit entries: {len(estimates)} from {resolved}")
    return estimates


def _wire_vertex_colors(obj: bpy.types.Object) -> None:
    """Connect vertex color attributes to Principled BSDF Base Color.

    GLB models with vertex colors (e.g. from Poisson reconstruction) import
    with the color data in a mesh attribute but the shader's Base Color
    defaults to white.  This wires a Color Attribute node into the BSDF so
    the vertex colors actually render.
    """
    mesh_data = obj.data
    attr_name = None
    if mesh_data.color_attributes:
        attr_name = mesh_data.color_attributes[0].name
    elif mesh_data.vertex_colors:
        attr_name = mesh_data.vertex_colors[0].name
    if attr_name is None:
        return

    if not obj.material_slots:
        mat = bpy.data.materials.new(name=f"vcol_{obj.name}")
        mat.use_nodes = True
        obj.data.materials.append(mat)

    for slot in obj.material_slots:
        mat = slot.material
        if not mat:
            continue
        if not mat.use_nodes:
            mat.use_nodes = True
        tree = mat.node_tree
        bsdf = next((n for n in tree.nodes if n.type == "BSDF_PRINCIPLED"), None)
        if bsdf is None:
            continue
        for link in list(bsdf.inputs["Base Color"].links):
            tree.links.remove(link)
        vcol = tree.nodes.new("ShaderNodeVertexColor")
        vcol.layer_name = attr_name
        tree.links.new(vcol.outputs["Color"], bsdf.inputs["Base Color"])


def load_distractor(
    file_path: Path, category_id: int, source_kind: str
) -> DistractorGroup | None:
    """Load a single distractor model under a parent empty.

    Returns ``(parent, meshes, native_size)`` or *None* on failure.  The
    parent empty controls transform for the whole group so sub-parts keep
    their relative positions.
    """
    try:
        suffix = file_path.suffix.lower()
        if suffix in (".glb", ".gltf"):
            bpy.ops.import_scene.gltf(filepath=str(file_path))
        elif suffix == ".obj":
            bpy.ops.wm.obj_import(filepath=str(file_path))
        elif suffix == ".ply":
            bpy.ops.wm.ply_import(filepath=str(file_path))
        else:
            return None

        imported = [o for o in bpy.context.selected_objects if o.type == "MESH"]
        if not imported:
            return None

        for obj in imported:
            _wire_vertex_colors(obj)

        bpy.context.view_layer.update()
        saved_world = {obj.name: obj.matrix_world.copy() for obj in imported}

        parent = bpy.data.objects.new(f"dist_{file_path.stem}", None)
        parent["distractor_source_kind"] = source_kind
        bpy.context.scene.collection.objects.link(parent)
        bpy.context.view_layer.update()

        for obj in imported:
            mat = saved_world[obj.name]
            obj.parent = parent
            obj.matrix_parent_inverse = mathutils.Matrix.Identity(4)
            obj.matrix_world = mat

        bpy.context.view_layer.update()

        all_pts = [
            obj.matrix_world @ mathutils.Vector(c)
            for obj in imported
            for c in obj.bound_box
        ]
        xs = [p.x for p in all_pts]
        ys = [p.y for p in all_pts]
        zs = [p.z for p in all_pts]
        native_size = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs), 1e-6)

        meshes = []
        for obj in imported:
            mesh = bproc.types.MeshObject(obj)
            mesh.set_cp("category_id", category_id)
            mesh.set_cp("robot_instance_id", 0)
            mesh.set_cp("is_distractor", 1)
            meshes.append(mesh)

        return (parent, meshes, native_size)
    except Exception as e:
        print(f"  Warning: failed to load distractor {file_path.name}: {e}")
        return None


def load_distractor_pool(
    sources: list[dict],
    max_count: int,
    assign_class_id: Callable[[Path, str], int],
    vram_budget_mb: float | None = None,
    vram_estimates_mb: dict[Path, float] | None = None,
) -> list[DistractorGroup]:
    """Load a weighted-random pool of distractor models.

    *sources* is a list of ``{"path": ..., "weight": ...}`` dicts.
    Models are drawn from each source directory with probability
    proportional to its weight until *max_count* is reached.
    """
    groups = discover_model_files(sources)
    if not groups:
        print("  No distractor models found.")
        return []

    for file_list, _, _, _ in groups:
        random.shuffle(file_list)

    total_weight = sum(w for _, w, _, _ in groups)
    budgets = [max(1, round(max_count * w / total_weight)) for _, w, _, _ in groups]
    remaining = max_count - sum(budgets)
    if remaining > 0:
        budgets[0] += remaining

    pool: list[DistractorGroup] = []
    pool_vram_mb = 0.0
    for (file_list, weight, _source_dir, source_kind), budget in zip(groups, budgets):
        loaded = 0
        for f in file_list:
            if loaded >= budget:
                break
            f_resolved = resolve_path(f)
            est_mb_pre = (
                vram_estimates_mb.get(f_resolved)
                if vram_estimates_mb is not None
                else None
            )
            if (
                est_mb_pre is not None
                and vram_budget_mb is not None
                and vram_budget_mb > 0
                and (pool_vram_mb + est_mb_pre) > vram_budget_mb
            ):
                print(
                    f"  Skipped distractor (pre-check): {f.name} "
                    f"(est {est_mb_pre:.1f} MB) "
                    f"— pool budget {vram_budget_mb:.1f} MB would be exceeded "
                    f"({pool_vram_mb + est_mb_pre:.1f} MB)"
                )
                continue

            class_id = assign_class_id(f, source_kind)
            group = load_distractor(f, class_id, source_kind)
            if group:
                est_mb = est_mb_pre
                if est_mb is None:
                    est_mb = estimate_distractor_group_gpu_mb(group)
                    if vram_estimates_mb is not None:
                        vram_estimates_mb[f_resolved] = est_mb
                if (
                    vram_budget_mb is not None
                    and vram_budget_mb > 0
                    and (pool_vram_mb + est_mb) > vram_budget_mb
                ):
                    unload_distractor_pool([group])
                    print(
                        f"  Skipped distractor: {f.name} "
                        f"(est {est_mb:.1f} MB) "
                        f"— pool budget {vram_budget_mb:.1f} MB would be exceeded "
                        f"({pool_vram_mb + est_mb:.1f} MB)"
                    )
                    continue

                pool.append(group)
                pool_vram_mb += est_mb
                _parent, meshes, sz = group
                print(
                    f"  Loaded distractor: {f.name} "
                    f"({len(meshes)} meshes, native {sz:.3f}m, est {est_mb:.1f} MB)"
                )
                loaded += 1
        dir_name = Path(file_list[0]).parent.name if file_list else "?"
        print(f"    {dir_name}: {loaded}/{budget} slots (weight {weight:.1f})")

    random.shuffle(pool)
    if vram_budget_mb is not None and vram_budget_mb > 0:
        print(
            f"  Distractor pool VRAM estimate: {pool_vram_mb:.1f} MB "
            f"(budget {vram_budget_mb:.1f} MB)"
        )
    return pool


def unload_distractor_pool(pool: list[DistractorGroup]) -> None:
    """Remove all distractor Blender objects and their mesh data."""
    for parent, meshes, _ in pool:
        for m in meshes:
            obj = m.blender_obj
            mesh_data = obj.data
            bpy.data.objects.remove(obj, do_unlink=True)
            if mesh_data and mesh_data.users == 0:
                bpy.data.meshes.remove(mesh_data)
        bpy.data.objects.remove(parent, do_unlink=True)
    pool.clear()


def hide_distractor(group: DistractorGroup) -> None:
    """Move a distractor group far off-screen via its parent."""
    parent = group[0]
    parent.location = mathutils.Vector((1000, 1000, 1000))


def place_distractor(
    group: DistractorGroup,
    arena_radius: float,
    scale_range: list[float],
    robot_size: float,
    air_probability: float = 0.15,
    air_height_range: list[float] | None = None,
) -> None:
    """Place a distractor at a random position within the arena.

    ``scale_range`` is interpreted as multiples of the robot's largest
    dimension, so [0.5, 3.0] means the distractor will be between half
    and three times the robot's size regardless of its native dimensions.
    The parent empty is used for transform so sub-parts keep their
    relative arrangement.

    Rotation is fully randomised on all three axes.  With probability
    ``air_probability`` the object floats above the ground; otherwise it
    sits on the ground plane.
    """
    parent, meshes, native_size = group
    desired_size = robot_size * random.uniform(scale_range[0], scale_range[1])
    s = desired_size / native_size

    parent.scale = (s, s, s)
    rot = (
        random.uniform(0, 2 * math.pi),
        random.uniform(0, 2 * math.pi),
        random.uniform(0, 2 * math.pi),
    )
    parent.rotation_euler = mathutils.Euler(rot)
    parent.location = mathutils.Vector((0, 0, 0))
    bpy.context.view_layer.update()

    all_pts = []
    for m in meshes:
        obj = m.blender_obj
        all_pts.extend(obj.matrix_world @ mathutils.Vector(c) for c in obj.bound_box)
    ground_z = -min(p.z for p in all_pts)

    airborne = random.random() < air_probability
    if airborne:
        hr = air_height_range or [0.02, 0.15]
        z = ground_z + random.uniform(hr[0], hr[1])
    else:
        z = ground_z

    parent.location = mathutils.Vector(
        (
            random.uniform(-arena_radius, arena_radius),
            random.uniform(-arena_radius, arena_radius),
            z,
        )
    )


def _ray_hits_distractor(
    scene: bpy.types.Scene,
    depsgraph: bpy.types.Depsgraph,
    origin: mathutils.Vector,
    target: mathutils.Vector,
) -> bpy.types.Object | None:
    """Cast a ray from *origin* toward *target*.

    Returns the first distractor object hit between the two points, or
    *None* if the path is clear (or hits a non-distractor).
    """
    direction = target - origin
    dist = direction.length
    direction.normalize()
    hit, _loc, _norm, _idx, obj, _mat = scene.ray_cast(
        depsgraph, origin, direction, distance=dist * 0.99
    )
    if hit and obj.get("is_distractor", 0) == 1:
        return obj
    return None


def clear_blocking_distractors(
    cam_poses: list[np.ndarray],
    keypoints_world: list[mathutils.Vector],
    active_distractors: list[DistractorGroup],
) -> None:
    """Hide distractors that block the camera's view of every robot keypoint.

    For each camera pose, rays are cast toward each keypoint.  A distractor
    is only hidden when it blocks *all* keypoints from a given camera — if at
    least one keypoint is reachable the distractor is kept.  The process
    repeats until every camera can see at least one keypoint.
    """
    bpy.context.view_layer.update()
    scene = bpy.context.scene
    hidden: set[int] = set()

    for pose in cam_poses:
        cam_origin = mathutils.Vector(pose[:3, 3].tolist())

        for _attempt in range(5):
            depsgraph = bpy.context.evaluated_depsgraph_get()

            blockers: dict[int, set[int]] = {}
            any_kp_clear = False
            for kp in keypoints_world:
                obj = _ray_hits_distractor(scene, depsgraph, cam_origin, kp)
                if obj is None:
                    any_kp_clear = True
                    break
                for gi, group in enumerate(active_distractors):
                    if gi in hidden:
                        continue
                    if any(m.blender_obj == obj for m in group[1]):
                        blockers.setdefault(gi, set()).add(id(kp))
                        break

            if any_kp_clear:
                break

            worst_gi = max(blockers, key=lambda gi: len(blockers[gi]))
            hide_distractor(active_distractors[worst_gi])
            hidden.add(worst_gi)
            bpy.context.view_layer.update()


# ---------------------------------------------------------------------------
# Camera sampling
# ---------------------------------------------------------------------------


def sample_camera_pose(
    look_at: list[float],
    min_dist: float,
    max_dist: float,
    height_range: list[float],
    noise: float,
    robot_center: list[float] | None = None,
    max_retries: int = 100,
) -> np.ndarray:
    """Sample a camera pose on a shell looking at a target point.

    When *robot_center* is provided, the look-at noise is resampled (up to
    *max_retries* times) until the robot center projects inside the camera
    frame.
    """
    distance = random.uniform(min_dist, max_dist)
    azimuth = random.uniform(0, 2 * math.pi)
    height = random.uniform(height_range[0], height_range[1])

    cam_pos = np.array(
        [
            look_at[0] + distance * math.cos(azimuth),
            look_at[1] + distance * math.sin(azimuth),
            height,
        ]
    )

    cam2world = None
    for _ in range(max_retries):
        target = np.array(
            [
                look_at[0] + random.gauss(0, noise),
                look_at[1] + random.gauss(0, noise),
                look_at[2] + random.gauss(0, noise),
            ]
        )

        forward = target - cam_pos
        forward = forward / np.linalg.norm(forward)

        rotation = bproc.camera.rotation_from_forward_vec(forward)
        cam2world = bproc.math.build_transformation_mat(cam_pos, rotation)

        if robot_center is None:
            return cam2world

        camera = bpy.context.scene.camera
        camera.matrix_world = mathutils.Matrix(cam2world.tolist())
        bpy.context.view_layer.update()

        co_2d = world_to_camera_view(
            bpy.context.scene, camera, mathutils.Vector(robot_center)
        )
        if 0 <= co_2d.x <= 1 and 0 <= co_2d.y <= 1 and co_2d.z > 0:
            return cam2world

    return cam2world


# ---------------------------------------------------------------------------
# Annotation helpers
# ---------------------------------------------------------------------------


def project_keypoint_to_2d(
    kp_local: np.ndarray, robot_world_mat: np.ndarray
) -> tuple[float, float, float] | None:
    """Project a 3D model-space keypoint to normalized 2D image coordinates.

    Uses Blender's built-in projection which handles all camera conventions.
    Returns (x_norm, y_norm, depth) or None if behind camera.
    """
    kp_world = robot_world_mat @ np.append(kp_local, 1.0)
    point_3d = mathutils.Vector(kp_world[:3])

    scene = bpy.context.scene
    camera = scene.camera
    co_2d = world_to_camera_view(scene, camera, point_3d)

    if co_2d.z <= 0:
        return None

    x_norm = co_2d.x
    y_norm = 1.0 - co_2d.y  # Blender is bottom-left origin, YOLO is top-left
    return (x_norm, y_norm, co_2d.z)


def check_keypoint_visibility(
    x_norm: float,
    y_norm: float,
    kp_depth: float,
    depth_map: np.ndarray,
    img_w: int,
    img_h: int,
    tolerance: float = 0.05,
) -> int:
    """Determine keypoint visibility: 0=out-of-frame, 1=occluded, 2=visible."""
    if not (0 <= x_norm <= 1 and 0 <= y_norm <= 1):
        return 0

    px = min(int(x_norm * img_w), img_w - 1)
    py = min(int(y_norm * img_h), img_h - 1)
    rendered_depth = depth_map[py, px]

    if abs(rendered_depth - kp_depth) < tolerance:
        return 2
    return 1


def bbox_from_category_segmap(
    seg_map: np.ndarray, category_id: int, img_w: int, img_h: int
) -> tuple[float, float, float, float] | None:
    """Compute a YOLO-format bounding box from the segmentation map.

    Returns (cx, cy, w, h) normalized to [0, 1], or None if not visible.
    """
    mask = seg_map.squeeze() == category_id
    if not np.any(mask):
        return None

    ys, xs = np.where(mask)
    x_min, x_max = int(xs.min()), int(xs.max())
    y_min, y_max = int(ys.min()), int(ys.max())

    cx = (x_min + x_max) / 2.0 / img_w
    cy = (y_min + y_max) / 2.0 / img_h
    w = (x_max - x_min) / img_w
    h = (y_max - y_min) / img_h
    return (cx, cy, w, h)


YoloAnnotation = tuple[
    int,
    tuple[float, float, float, float],
    list[tuple[float, float, int]],
]
"""(class_id, (cx, cy, w, h), [(kp_x, kp_y, vis), ...])"""


def write_yolo_labels(
    filepath: Path,
    annotations: list[YoloAnnotation],
) -> None:
    """Write one YOLO keypoint annotation line per detected object."""
    with open(filepath, "w") as f:
        for class_id, bbox, keypoints in annotations:
            cx, cy, w, h = bbox
            parts = [f"{class_id} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}"]
            for kp_x, kp_y, vis in keypoints:
                if vis == 0:
                    parts.append("0.000000 0.000000 0")
                else:
                    parts.append(f"{kp_x:.6f} {kp_y:.6f} {vis}")
            f.write(" ".join(parts) + "\n")


YoloSegAnnotation = tuple[int, list[tuple[float, float]]]
"""(class_id, [(x1, y1), (x2, y2), ...])"""


def normalize_annotation_mode(mode: str) -> str:
    mode_norm = mode.strip().lower()
    valid_modes = {
        ANNOTATION_MODE_KEYPOINTS_BBOX,
        ANNOTATION_MODE_SEGMENTATION_BBOX,
    }
    if mode_norm not in valid_modes:
        raise ValueError(
            f"Unsupported output.annotation_mode='{mode}'. "
            f"Expected one of: {sorted(valid_modes)}"
        )
    return mode_norm


def segmentation_annotations_from_segmap(
    seg_map: np.ndarray,
    img_w: int,
    img_h: int,
    min_bbox_dim: int = 1,
) -> list[YoloSegAnnotation]:
    """Convert a category-id segmentation map into YOLO-seg annotations."""
    seg_2d = seg_map.squeeze()
    annotations: list[YoloSegAnnotation] = []
    for class_id in sorted(int(v) for v in np.unique(seg_2d).tolist()):
        if class_id <= BACKGROUND_CATEGORY_ID:
            continue

        mask = (seg_2d == class_id).astype(np.uint8)
        if not np.any(mask):
            continue

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            xs = contour[:, 0, 0]
            ys = contour[:, 0, 1]
            bbox_w = int(xs.max() - xs.min() + 1)
            bbox_h = int(ys.max() - ys.min() + 1)
            if bbox_w < min_bbox_dim or bbox_h < min_bbox_dim:
                continue

            if contour.shape[0] >= 3:
                polygon = [
                    (float(x) / img_w, float(y) / img_h) for x, y in contour[:, 0, :]
                ]
            else:
                # Preserve tiny visible objects (e.g. heavily occluded or edge-clipped)
                # by emitting a minimal rectangle polygon.
                x_min = int(xs.min())
                x_max = int(xs.max())
                y_min = int(ys.min())
                y_max = int(ys.max())
                polygon = [
                    (float(x_min) / img_w, float(y_min) / img_h),
                    (float(x_max) / img_w, float(y_min) / img_h),
                    (float(x_max) / img_w, float(y_max) / img_h),
                    (float(x_min) / img_w, float(y_max) / img_h),
                ]
            annotations.append((class_id, polygon))
    return annotations


def write_yolo_seg_labels(filepath: Path, annotations: list[YoloSegAnnotation]) -> None:
    """Write YOLO segmentation labels (class + polygon points)."""
    with open(filepath, "w") as f:
        for class_id, polygon in annotations:
            parts = [str(class_id)]
            for x, y in polygon:
                parts.append(f"{x:.6f}")
                parts.append(f"{y:.6f}")
            f.write(" ".join(parts) + "\n")


def write_label_index(filepath: Path, labels: dict[int, str]) -> None:
    """Write a human-readable class-id map for segmentation mode."""
    with open(filepath, "w") as f:
        for class_id, name in sorted(labels.items()):
            f.write(f"{class_id}:{name}\n")


def build_names_list(labels: dict[int, str]) -> list[str]:
    """Build a dense names list indexed by class id."""
    if not labels:
        return []
    max_id = max(int(class_id) for class_id in labels)
    names: list[str] = []
    for class_id in range(max_id + 1):
        names.append(labels.get(class_id, f"unknown_{class_id}"))
    return names


def write_data_yml(
    filepath: Path, dataset_root: Path, names: list[str], annotation_mode: str
) -> None:
    """Write YOLO dataset metadata as data.yml."""
    lines = [
        f"path: {dataset_root.resolve()}",
        "train: images",
        "val: images",
        f"nc: {len(names)}",
        f"names: {names}",
    ]
    if annotation_mode == ANNOTATION_MODE_KEYPOINTS_BBOX:
        lines.extend(
            [
                "kpt_shape: [2, 3]",
                "flip_idx: [0, 1]",
            ]
        )
    filepath.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _directional_blur_kernel(kernel_size: int, angle: float) -> np.ndarray:
    """Build a normalized 1-D directional blur kernel."""
    ks = max(3, kernel_size) | 1
    kernel = np.zeros((ks, ks), dtype=np.float32)
    cx = ks // 2
    dx, dy = math.cos(angle), math.sin(angle)
    for i in range(ks):
        t = i - cx
        x = int(round(cx + t * dx))
        y = int(round(cx + t * dy))
        if 0 <= x < ks and 0 <= y < ks:
            kernel[y, x] = 1.0
    kernel /= max(kernel.sum(), 1.0)
    return kernel


def apply_object_motion_blur(
    image: np.ndarray,
    seg_map: np.ndarray,
    category_id: int,
    kernel_size: int,
    angle: float,
) -> np.ndarray:
    """Apply directional motion blur to pixels of a single *category_id*.

    A linear blur kernel of *kernel_size* pixels at *angle* radians is
    applied to the full image.  The segmentation mask (dilated by the kernel
    radius) selects which output pixels come from the blurred vs. original
    image, so only that object's region shows the streak.
    """
    kernel = _directional_blur_kernel(kernel_size, angle)
    ks = kernel.shape[0]

    seg_2d = seg_map.squeeze()
    mask = (seg_2d == category_id).astype(np.uint8) * 255

    dilate_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ks, ks))
    mask = cv2.dilate(mask, dilate_k, iterations=1)
    mask_f = mask[:, :, np.newaxis].astype(np.float32) / 255.0

    blurred = cv2.filter2D(image, -1, kernel)
    return (blurred * mask_f + image * (1.0 - mask_f)).astype(np.uint8)


# ---------------------------------------------------------------------------
# Material randomization
# ---------------------------------------------------------------------------


def jitter_materials(
    meshes: list[bproc.types.MeshObject],
    roughness_jitter: float,
    hue_jitter_deg: float,
) -> None:
    """Apply small random perturbations to material properties."""
    for mesh in meshes:
        for mat in mesh.get_materials():
            try:
                roughness = mat.get_principled_shader_value("Roughness")
                if isinstance(roughness, (int, float)):
                    new_r = max(
                        0,
                        min(
                            1,
                            roughness
                            + random.uniform(-roughness_jitter, roughness_jitter),
                        ),
                    )
                    mat.set_principled_shader_value("Roughness", new_r)
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Main rendering loop
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path, help="Path to config.toml")
    parser.add_argument("--num-images", type=int, default=None)
    parser.add_argument("--render-samples", type=int, default=64)
    parser.add_argument(
        "--start-index",
        type=int,
        default=None,
        help="Starting frame index (for resuming). Defaults to auto-detecting the next index from existing output files.",
    )
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else sys.argv[1:]
    args = parser.parse_args(argv)

    config = load_config(args.config)
    annotation_mode = normalize_annotation_mode(
        config["output"].get("annotation_mode", ANNOTATION_MODE_KEYPOINTS_BBOX)
    )
    is_segmentation_mode = annotation_mode == ANNOTATION_MODE_SEGMENTATION_BBOX

    num_images = args.num_images or config["output"]["num_images"]
    img_w = config["output"].get("image_width", 1280)
    img_h = config["output"].get("image_height", 720)
    images_per_scene = config["output"].get("images_per_scene", 5)
    max_scenes = math.ceil(num_images / images_per_scene) * 3

    robot_configs = config["robots"]
    seg_robot_class_ids: dict[int, int] = {}
    seg_label_names: dict[int, str] = {}
    if is_segmentation_mode:
        seg_label_names[BACKGROUND_CATEGORY_ID] = "background"
        seg_label_names[SEG_FLOOR_CLASS_ID] = "floor"
        seg_label_names[SEG_OBJECT_CLASS_ID] = "object"
        next_seg_class_id = SEG_OBJECT_CLASS_ID + 1
        for ri, rcfg in enumerate(robot_configs, start=1):
            seg_robot_class_ids[ri] = next_seg_class_id
            name = str(rcfg.get("name", Path(rcfg["model_path"]).stem))
            seg_label_names[next_seg_class_id] = name
            next_seg_class_id += 1
    else:
        next_seg_class_id = SEG_OBJECT_CLASS_ID + 1

    seg_distractor_class_ids: dict[Path, int] = {}

    def assign_distractor_class_id(model_path: Path, source_kind: str) -> int:
        nonlocal next_seg_class_id
        if not is_segmentation_mode:
            return DISTRACTOR_CATEGORY_ID

        if source_kind == "objaverse":
            return SEG_OBJECT_CLASS_ID

        key = resolve_path(model_path)
        if key not in seg_distractor_class_ids:
            seg_distractor_class_ids[key] = next_seg_class_id
            seg_label_names[next_seg_class_id] = key.stem
            next_seg_class_id += 1
        return seg_distractor_class_ids[key]

    output_image_dir = resolve_path(Path(config["output"]["image_dir"]))
    output_label_dir = resolve_path(Path(config["output"]["label_dir"]))
    output_image_dir.mkdir(parents=True, exist_ok=True)
    output_label_dir.mkdir(parents=True, exist_ok=True)
    dataset_root = (
        output_image_dir.parent
        if output_image_dir.parent == output_label_dir.parent
        else output_label_dir.parent
    )
    data_yml_path = dataset_root / "data.yml"

    if args.start_index is None:
        existing = [
            int(p.stem) for p in output_image_dir.glob("*.jpg") if p.stem.isdigit()
        ]
        args.start_index = max(existing) + 1 if existing else 0
        if existing:
            print(
                f"Auto-resuming from index {args.start_index} ({len(existing)} existing images found)"
            )

    # ------- Initialize BlenderProc -------

    bproc.init()
    bproc.camera.set_resolution(img_w, img_h)
    bproc.renderer.set_max_amount_of_samples(args.render_samples)
    bproc.renderer.enable_depth_output(activate_antialiasing=False)
    if hasattr(bpy.context.scene, "cycles"):
        # Prevent Cycles from keeping render data across frames/scenes.
        bpy.context.scene.cycles.use_persistent_data = False

    # ------- Load target robots -------

    cc_textures_dir = config.get("environment", {}).get("cc_textures_dir")
    cc_materials = _load_cc_materials(config["materials"], cc_textures_dir)
    robots: list[RobotInstance] = []

    for ri, rcfg in enumerate(robot_configs, start=1):
        rname = rcfg.get("name", Path(rcfg["model_path"]).stem)
        model_path = Path(rcfg["model_path"])
        robot_scale = rcfg.get("scale", 1.0)
        robot_category_id = (
            seg_robot_class_ids[ri] if is_segmentation_mode else ROBOT_CATEGORY_ID
        )
        print(f"Loading robot model: {rname} ({model_path})")
        meshes, parent, bbox = import_gltf_as_robot(
            model_path, robot_scale, category_id=robot_category_id
        )
        print(f"  {len(meshes)} mesh parts loaded, instance_id={ri}")
        size = max(
            max(c.x for c in bbox) - min(c.x for c in bbox),
            max(c.y for c in bbox) - min(c.y for c in bbox),
            max(c.z for c in bbox) - min(c.z for c in bbox),
        )
        print(f"  Robot max dimension: {size:.4f} m")

        for mesh in meshes:
            mesh.set_cp("robot_instance_id", ri)

        print(f"  Applying PBR materials to {rname}...")
        apply_pbr_materials(
            meshes,
            rcfg["color_mapping"],
            config["materials"],
            cc_materials,
        )

        robot = RobotInstance(
            name=rname,
            instance_id=ri,
            meshes=meshes,
            parent=parent,
            bbox=bbox,
            size=size,
            kp_front=model_to_blender_local(rcfg["keypoints"]["front"]),
            kp_back=model_to_blender_local(rcfg["keypoints"]["back"]),
            class_id=(robot_category_id if is_segmentation_mode else rcfg["class_id"]),
            weight=rcfg.get("weight", 1.0),
            config=rcfg,
        )
        robots.append(robot)

    if not robots:
        raise RuntimeError("No [[robots]] entries found in config.")
    print(f"\n{len(robots)} robot model(s) loaded: {[r.name for r in robots]}")

    for robot in robots:
        hide_robot(robot)

    if not is_segmentation_mode:
        keypoint_label_names: dict[int, str] = {}
        for rcfg in robot_configs:
            class_id = int(rcfg["class_id"])
            keypoint_label_names.setdefault(
                class_id, str(rcfg.get("name", Path(rcfg["model_path"]).stem))
            )
        write_data_yml(
            data_yml_path,
            dataset_root,
            build_names_list(keypoint_label_names),
            annotation_mode,
        )

    # ------- Load distractors -------

    dist_cfg = config.get("distractors", {})
    dist_sources = dist_cfg.get("sources", [])
    dist_max_per_scene = dist_cfg.get("max_per_scene", 5)
    dist_min_per_scene = dist_cfg.get("min_per_scene", 0)
    dist_shuffle_interval = dist_cfg.get("shuffle_interval", 100)
    dist_vram_budget_mb = dist_cfg.get("vram_budget_mb")
    if dist_vram_budget_mb is not None:
        dist_vram_budget_mb = float(dist_vram_budget_mb)
    dist_vram_audit_csv = dist_cfg.get(
        "vram_audit_csv", "training/data/distractor_models/distractor_gpu_audit.csv"
    )
    dist_vram_estimates = load_distractor_vram_audit(Path(dist_vram_audit_csv))

    def refresh_distractor_pool(
        old_pool: list[DistractorGroup],
    ) -> list[DistractorGroup]:
        """Unload old distractors and load a fresh random pool."""
        if old_pool:
            unload_distractor_pool(old_pool)
        print("Loading distractor models...")
        pool = load_distractor_pool(
            dist_sources,
            dist_max_per_scene,
            assign_class_id=assign_distractor_class_id,
            vram_budget_mb=dist_vram_budget_mb,
            vram_estimates_mb=dist_vram_estimates,
        )
        print(f"  {len(pool)} distractors in pool")
        if dist_min_per_scene > len(pool):
            src_paths = [s["path"] for s in dist_sources]
            raise RuntimeError(
                f"distractors.min_per_scene is {dist_min_per_scene} but only "
                f"{len(pool)} distractor model(s) were found.\n"
                f"  Searched directories: {src_paths}\n"
                f"  Either add distractor models to those directories, or set "
                f"min_per_scene <= {len(pool)} in config.toml."
            )
        for group in pool:
            hide_distractor(group)
        if is_segmentation_mode:
            write_label_index(output_label_dir / "label_index.txt", seg_label_names)
            write_data_yml(
                data_yml_path,
                dataset_root,
                build_names_list(seg_label_names),
                annotation_mode,
            )
        return pool

    distractor_pool = refresh_distractor_pool([])
    images_since_shuffle = 0

    # ------- Load environment assets -------

    env_cfg = config.get("environment", {})
    hdri_dir = resolve_path(Path(env_cfg.get("hdri_dir", "data/hdris")))
    hdri_paths: list[Path] = []
    if hdri_dir.exists():
        hdri_paths = list(hdri_dir.glob("*.hdr")) + list(hdri_dir.glob("*.exr"))
    print(f"  {len(hdri_paths)} HDRIs available")

    cc_textures: list[bproc.types.Material] = []
    cc_dir = env_cfg.get("cc_textures_dir")
    if cc_dir and resolve_path(Path(cc_dir)).exists():
        cc_textures = bproc.loader.load_ccmaterials(str(resolve_path(Path(cc_dir))))
    print(f"  {len(cc_textures)} CC textures available for ground")

    # ------- Create ground plane -------

    scene_cfg = config.get("scene", {})
    ground_category_id = (
        SEG_FLOOR_CLASS_ID if is_segmentation_mode else BACKGROUND_CATEGORY_ID
    )
    ground = bproc.object.create_primitive("PLANE", scale=[1, 1, 1], location=[0, 0, 0])
    ground.set_cp("category_id", ground_category_id)
    ground.set_cp("robot_instance_id", 0)
    ground.set_cp("is_distractor", 0)

    # Enable segmentation AFTER all mesh objects are in the scene, because
    # enable_segmentation_output assigns pass_index to every mesh at call time.
    bproc.renderer.enable_segmentation_output(
        map_by=["category_id", "robot_instance_id"],
        default_values={"category_id": BACKGROUND_CATEGORY_ID, "robot_instance_id": 0},
    )

    # ------- Create reusable lights -------

    rand_cfg = config.get("randomization", {})
    max_lights = rand_cfg.get("light_count_range", [1, 3])[1]
    lights: list[bproc.types.Light] = []
    for _ in range(max_lights):
        light = bproc.types.Light()
        light.set_type("POINT")
        lights.append(light)

    # ------- Camera config -------

    cam_cfg = config.get("camera", {})
    ground_size_range = scene_cfg.get("ground_size_range", [2.0, 5.0])
    arena_radius_range = scene_cfg.get("arena_radius_range", [0.5, 1.5])
    max_robots_per_scene = scene_cfg.get("max_robots_per_scene", 1)
    memory_cleanup_interval = int(config["output"].get("memory_cleanup_interval", 25))
    seg_min_bbox_dim = int(config["output"].get("segmentation_min_bbox_dim", 1))

    # ------- Verify category_ids -------

    all_scene_meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
    id_counts: dict[int, int] = {}
    for obj in all_scene_meshes:
        cid = obj.get("category_id", -1)
        id_counts[cid] = id_counts.get(cid, 0) + 1
    print(f"\nCategory ID distribution across {len(all_scene_meshes)} mesh objects:")
    for cid, count in sorted(id_counts.items()):
        if is_segmentation_mode:
            default_label = "background" if cid == BACKGROUND_CATEGORY_ID else "unknown"
            label = seg_label_names.get(cid, default_label)
        else:
            label = {
                ROBOT_CATEGORY_ID: "robot",
                DISTRACTOR_CATEGORY_ID: "distractor",
                BACKGROUND_CATEGORY_ID: "background",
            }.get(cid, "UNSET" if cid == -1 else "unknown")
        print(f"  category_id={cid} ({label}): {count} objects")

    # ------- Render loop -------

    global_idx = args.start_index
    scene_idx = 0
    print(f"\nRendering {num_images} images...\n")

    while global_idx < args.start_index + num_images and scene_idx < max_scenes:
        bproc.utility.reset_keyframes()

        # -- Per-scene randomized dimensions --
        ground_size = random.uniform(*ground_size_range)
        arena_radius = random.uniform(*arena_radius_range)
        ground.blender_obj.scale = (ground_size, ground_size, 1)
        bpy.context.view_layer.update()

        # -- Environment randomization --
        if hdri_paths:
            bproc.world.set_world_background_hdr_img(str(random.choice(hdri_paths)))

        ground_prob = scene_cfg.get("ground_visibility", 0.8)
        show_ground = random.random() < ground_prob
        ground.blender_obj.hide_render = not show_ground
        ground.blender_obj.hide_viewport = not show_ground

        if show_ground and cc_textures:
            ground.replace_materials(random.choice(cc_textures))

        # -- Select robots for this scene --
        scene_robots = choose_scene_robots(robots, max_robots_per_scene)
        for r in robots:
            if r in scene_robots:
                show_robot(r)
            else:
                hide_robot(r)

        # -- Pose each robot independently --
        robot_positions: list[list[float]] = []
        for robot in scene_robots:
            rcfg = robot.config
            airborne = random.random() < rand_cfg.get("air_probability", 0.15)
            if airborne:
                robot_rot = (
                    random.uniform(0, 2 * math.pi),
                    random.uniform(0, 2 * math.pi),
                    random.uniform(0, 2 * math.pi),
                )
                ground_z = compute_ground_z(robot.meshes, robot.parent, robot_rot)
                air_cfg = rand_cfg.get("air_height_range", [0.02, 0.15])
                robot_z = ground_z + random.uniform(air_cfg[0], air_cfg[1])
            else:
                if random.random() < 0.5:
                    pitch_deg = -90.0
                    roll_deg = rcfg.get("ground_roll_upright", 0.0)
                else:
                    pitch_deg = 90.0
                    roll_deg = rcfg.get("ground_roll_inverted", 0.0)
                robot_rot = (
                    math.radians(pitch_deg),
                    math.radians(roll_deg),
                    random.uniform(0, 2 * math.pi),
                )
                robot_z = compute_ground_z(robot.meshes, robot.parent, robot_rot)
            robot_pos = [
                random.uniform(-arena_radius * 0.5, arena_radius * 0.5),
                random.uniform(-arena_radius * 0.5, arena_radius * 0.5),
                robot_z,
            ]
            robot.parent.location = mathutils.Vector(robot_pos)
            robot.parent.rotation_euler = mathutils.Euler(robot_rot)
            robot_positions.append(robot_pos)

        bpy.context.view_layer.update()

        # -- Distractor pool refresh --
        if dist_shuffle_interval and images_since_shuffle >= dist_shuffle_interval:
            distractor_pool = refresh_distractor_pool(distractor_pool)
            images_since_shuffle = 0

        # -- Distractor placement (scale relative to largest robot in scene) --
        max_robot_size = max(r.size for r in scene_robots)
        num_dist = random.randint(dist_min_per_scene, len(distractor_pool))
        active_distractors = (
            random.sample(distractor_pool, num_dist) if num_dist > 0 else []
        )
        for group in distractor_pool:
            hide_distractor(group)
        for group in active_distractors:
            place_distractor(
                group,
                arena_radius,
                dist_cfg.get("scale_range", [0.5, 3.0]),
                max_robot_size,
                air_probability=rand_cfg.get("air_probability", 0.15),
                air_height_range=rand_cfg.get("air_height_range", [0.02, 0.15]),
            )

        # -- Light randomization --
        num_active_lights = random.randint(*rand_cfg.get("light_count_range", [1, 3]))
        intensity_range = rand_cfg.get("light_intensity_range", [100, 500])
        for i, light in enumerate(lights):
            if i < num_active_lights:
                light.set_location(
                    [
                        random.uniform(-2, 2),
                        random.uniform(-2, 2),
                        random.uniform(1.5, 3.5),
                    ]
                )
                light.set_energy(random.uniform(*intensity_range))
            else:
                light.set_energy(0)

        # -- Material jitter on all scene robots --
        for robot in scene_robots:
            jitter_materials(
                robot.meshes,
                rand_cfg.get("roughness_jitter", 0.1),
                rand_cfg.get("hue_jitter_degrees", 5),
            )

        # -- Camera poses (look at centroid of all placed robots) --
        centroid = [
            sum(p[i] for p in robot_positions) / len(robot_positions) for i in range(3)
        ]
        cam_count = min(images_per_scene, args.start_index + num_images - global_idx)
        cam_poses = []
        for _ in range(cam_count):
            pose = sample_camera_pose(
                look_at=centroid,
                min_dist=cam_cfg.get("min_distance", 0.3),
                max_dist=cam_cfg.get("max_distance", 1.5),
                height_range=cam_cfg.get("height_range", [0.1, 0.8]),
                noise=cam_cfg.get("look_at_noise", 0.05),
                robot_center=centroid,
            )
            cam_poses.append(pose)

        # -- Pre-render visibility check (all robots' keypoints) --
        all_keypoints_world: list[mathutils.Vector] = []
        for robot in scene_robots:
            wmat = np.array(robot.parent.matrix_world)
            for kp in [robot.kp_front, robot.kp_back]:
                all_keypoints_world.append(
                    mathutils.Vector((wmat @ np.append(kp, 1.0))[:3])
                )
        clear_blocking_distractors(cam_poses, all_keypoints_world, active_distractors)

        for pose in cam_poses:
            bproc.camera.add_camera_pose(pose)

        # -- Render with distractors enabled --
        data = bproc.renderer.render()

        if scene_idx == 0:
            print(f"  Render data keys: {list(data.keys())}")
            debug_img = data["colors"][0]
            debug_path = str(output_image_dir / "_debug_frame0.jpg")
            cv2.imwrite(debug_path, cv2.cvtColor(debug_img, cv2.COLOR_RGB2BGR))
            print(f"  Saved debug image: {debug_path}")

        # -- Extract annotations and save --
        colors = data["colors"]
        cat_seg_maps = data.get("category_id_segmaps", data.get("segmap"))
        inst_seg_maps = data.get("robot_instance_id_segmaps")
        depth_maps = data["depth"]
        clean_inst_seg_maps = None

        # True occlusion metric: visible robot pixels (with distractors) divided
        # by unobstructed robot pixels from a second pass with distractors hidden.
        if inst_seg_maps is not None:
            if active_distractors:
                saved_distractor_world = [
                    group[0].matrix_world.copy() for group in active_distractors
                ]
                for group in active_distractors:
                    hide_distractor(group)
                bpy.context.view_layer.update()

                clean_data = bproc.renderer.render()
                clean_inst_seg_maps = clean_data.get("robot_instance_id_segmaps")

                for group, mat in zip(active_distractors, saved_distractor_world):
                    group[0].matrix_world = mat
                bpy.context.view_layer.update()
            else:
                clean_inst_seg_maps = inst_seg_maps

        if cat_seg_maps is None:
            if scene_idx == 0:
                print("  ERROR: No segmentation maps in render output!")
            continue

        min_vis = config["output"].get("min_robot_visibility", 0.10)
        blur_category_ids: set[int] = set()
        for robot in scene_robots:
            if robot.meshes:
                blur_category_ids.add(
                    int(robot.meshes[0].blender_obj.get("category_id", -1))
                )
        for group in active_distractors:
            meshes = group[1]
            if meshes:
                blur_category_ids.add(int(meshes[0].blender_obj.get("category_id", -1)))
        blur_category_ids.discard(-1)

        for local_idx in range(cam_count):
            bpy.context.scene.frame_set(local_idx)

            color_img = colors[local_idx]
            cat_seg = cat_seg_maps[local_idx]
            inst_seg = inst_seg_maps[local_idx] if inst_seg_maps else None
            clean_inst_seg = (
                clean_inst_seg_maps[local_idx]
                if clean_inst_seg_maps is not None
                else None
            )
            depth_map = depth_maps[local_idx]

            if scene_idx == 0 and local_idx == 0:
                print(
                    f"  Cat segmap shape={cat_seg.shape}, "
                    f"unique={np.unique(cat_seg).tolist()}"
                )
                if inst_seg is not None:
                    print(f"  Inst segmap unique={np.unique(inst_seg).tolist()}")

            keypoint_annotations: list[YoloAnnotation] = []
            seg_annotations: list[YoloSegAnnotation] = []
            if is_segmentation_mode:
                visible_category_ids = {
                    int(v)
                    for v in np.unique(cat_seg.squeeze()).tolist()
                    if int(v) > BACKGROUND_CATEGORY_ID
                }
                # Skip floor-only frames (no robots or distractor objects visible).
                if visible_category_ids == {SEG_FLOOR_CLASS_ID}:
                    continue
                seg_annotations = segmentation_annotations_from_segmap(
                    cat_seg,
                    img_w,
                    img_h,
                    min_bbox_dim=seg_min_bbox_dim,
                )
                if not seg_annotations:
                    continue
                annotated_category_ids = {class_id for class_id, _ in seg_annotations}
                # In segmentation mode, require every visible foreground class
                # (robots + floor + distractors/object) to be present in labels.
                required_visible_category_ids = set(visible_category_ids)

                missing_required_ids = (
                    required_visible_category_ids - annotated_category_ids
                )
                if missing_required_ids:
                    # Discard frames when expected visible classes (especially robots)
                    # are dropped from YOLO-seg labels by contour filtering.
                    continue
            else:
                rendered_robot_ids: set[int] = set()
                if inst_seg is not None:
                    visible_ids = {
                        int(v) for v in np.unique(inst_seg.squeeze()).tolist()
                    }
                    rendered_robot_ids = {
                        robot.instance_id
                        for robot in scene_robots
                        if robot.instance_id in visible_ids
                    }

                annotated_robot_ids: set[int] = set()
                for robot in scene_robots:
                    if inst_seg is not None:
                        seg_for_bbox = inst_seg
                        seg_id = robot.instance_id
                    else:
                        seg_for_bbox = cat_seg
                        seg_id = ROBOT_CATEGORY_ID

                    bbox = bbox_from_category_segmap(seg_for_bbox, seg_id, img_w, img_h)
                    if bbox is None:
                        continue

                    cx, cy, bw, bh = bbox
                    w_px = int(bw * img_w)
                    h_px = int(bh * img_h)
                    min_bbox_dim = 32
                    if w_px < min_bbox_dim or h_px < min_bbox_dim:
                        continue

                    if inst_seg is not None and clean_inst_seg is not None:
                        visible_px = int(np.sum(inst_seg.squeeze() == seg_id))
                        unobstructed_px = int(
                            np.sum(clean_inst_seg.squeeze() == seg_id)
                        )
                        if unobstructed_px <= 0:
                            continue
                        if (visible_px / unobstructed_px) < min_vis:
                            continue
                    else:
                        # Fallback for configurations where instance segmaps are absent.
                        robot_px = int(np.sum(seg_for_bbox.squeeze() == seg_id))
                        bbox_area_px = max(1, w_px * h_px)
                        if robot_px < bbox_area_px * min_vis:
                            continue

                    robot_world_mat = np.array(robot.parent.matrix_world)
                    keypoints_2d: list[tuple[float, float, int]] = []
                    for kp_local in [robot.kp_front, robot.kp_back]:
                        proj = project_keypoint_to_2d(kp_local, robot_world_mat)
                        if proj is None:
                            keypoints_2d.append((0.0, 0.0, 0))
                            continue
                        x_n, y_n, depth = proj
                        vis = check_keypoint_visibility(
                            x_n, y_n, depth, depth_map, img_w, img_h
                        )
                        keypoints_2d.append((x_n, y_n, vis))

                    keypoint_annotations.append((robot.class_id, bbox, keypoints_2d))
                    annotated_robot_ids.add(robot.instance_id)

                missing_robot_ids = rendered_robot_ids - annotated_robot_ids
                if missing_robot_ids:
                    # Discard this frame if any robot instance that appears in the
                    # rendered segmentation is missing from YOLO annotations.
                    continue

                if not keypoint_annotations:
                    continue

            # -- Motion blur (applied to all robot pixels jointly) --
            blur_prob = rand_cfg.get("motion_blur_probability", 0.0)
            blur_range = rand_cfg.get("motion_blur_strength_range", [5, 25])
            blur_lo, blur_hi = int(blur_range[0]), int(blur_range[1])
            for blur_cid in sorted(blur_category_ids):
                if random.random() < blur_prob:
                    color_img = apply_object_motion_blur(
                        color_img,
                        cat_seg,
                        blur_cid,
                        random.randint(blur_lo, blur_hi),
                        random.uniform(0, 2 * math.pi),
                    )

            frame_name = f"{global_idx:06d}"
            if is_segmentation_mode:
                write_yolo_seg_labels(
                    output_label_dir / f"{frame_name}.txt", seg_annotations
                )
            else:
                write_yolo_labels(
                    output_label_dir / f"{frame_name}.txt",
                    keypoint_annotations,
                )
            cv2.imwrite(
                str(output_image_dir / f"{frame_name}.jpg"),
                cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR),
            )
            global_idx += 1
            images_since_shuffle += 1

        scene_idx += 1
        if memory_cleanup_interval > 0 and scene_idx % memory_cleanup_interval == 0:
            gc.collect()
            # Free transient render buffers/images without touching materials.
            # Avoid orphans_purge here: it can delete cached cc_textures that are
            # intentionally kept for future scenes but temporarily have 0 users.
            for img in list(bpy.data.images):
                if img.name.startswith(("Render Result", "Viewer Node")):
                    try:
                        img.buffers_free()
                    except Exception:
                        pass
                    if img.users == 0:
                        bpy.data.images.remove(img)

        if scene_idx % 50 == 0:
            names = [r.name for r in scene_robots]
            print(
                f"  Scene {scene_idx} ({names}) — "
                f"{global_idx - args.start_index}/{num_images} images generated"
            )

    total = global_idx - args.start_index
    print(f"\nDone. Generated {total} images in {output_image_dir}")


if __name__ == "__main__":
    main()
