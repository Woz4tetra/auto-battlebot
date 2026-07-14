"""Material inspection, mapping, and PBR application (requires Blender).

Robot GLTF exports carry flat part colors; these are matched against the
config's color mapping (pure logic in ``synthgen.colorspec``) and replaced with
PBR materials: tiling CC textures, UV-mapped texture directories, or scalar
metallic/roughness values.
"""

import random
from pathlib import Path

import blenderproc as bproc
import bpy

from synthgen.asset_index import PathResolveFn
from synthgen.colorspec import ColorMappingEntry, Rgb, color_from_material_name
from synthgen.colorspec import match_material_type as _match_material_type
from synthgen.configuration import MaterialConfig
from synthgen.logsetup import get_logger

logger = get_logger(__name__)

_PBR_SUFFIXES = [
    "Color",
    "Roughness",
    "Metalness",
    "NormalGL",
    "Normal",
    "Displacement",
]


def _color_from_node(node: bpy.types.ShaderNode) -> Rgb | None:
    """Read a constant RGB value from node types where default_value is reliable."""
    if node.type == "RGB":
        c = node.outputs[0].default_value
        return (int(c[0] * 255), int(c[1] * 255), int(c[2] * 255))
    return None


def get_material_base_color(mat: bpy.types.Material) -> Rgb | None:
    """Extract the flat base color of a material, or None when it has real content."""
    if not mat.use_nodes:
        return color_from_material_name(mat.name)
    bsdf = None
    for node in mat.node_tree.nodes:
        if node.type == "BSDF_PRINCIPLED":
            bsdf = node
            break
    if bsdf is None:
        return color_from_material_name(mat.name)

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


def tint_material_albedo(bpy_mat: bpy.types.Material, color_rgb: "list[int] | Rgb") -> None:
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
        logger.debug("Tinted %s: set flat color %s", bpy_mat.name, list(color_rgb))
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
        logger.debug("Tinted %s: MULTIPLY with %s", bpy_mat.name, list(color_rgb))
    else:
        tree.links.new(from_socket, bc_input)
        logger.warning(
            "Mix node has %d RGBA inputs, %d RGBA outputs — tint skipped, reconnected original",
            len(color_inputs),
            len(color_outputs),
        )


def find_cc_material(
    name: str, cc_materials: dict[str, bproc.types.Material]
) -> bproc.types.Material | None:
    """Find a CC material by exact name, then by substring match."""
    if name in cc_materials:
        return cc_materials[name]
    for key, mat in cc_materials.items():
        if name.lower() in key.lower():
            return mat
    return None


def load_cc_materials(
    materials_config: dict[str, MaterialConfig],
    cc_textures_dir: "Path | None",
    resolve: PathResolveFn,
) -> dict[str, bproc.types.Material]:
    """Load configured CC materials once and return a lookup map."""
    cc_materials: dict[str, bproc.types.Material] = {}
    cc_path = resolve(cc_textures_dir) if cc_textures_dir is not None else None
    if not (cc_path and cc_path.exists()):
        return cc_materials

    needed = {cfg.cc_texture for cfg in materials_config.values() if cfg.cc_texture is not None}
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


def _find_texture_file(texture_dir: "Path", suffix: str) -> "Path | None":
    """Find a texture file in *texture_dir* whose name contains *suffix*."""
    for ext in (".jpg", ".jpeg", ".png", ".exr", ".tiff"):
        for p in texture_dir.iterdir():
            if p.suffix.lower() == ext and suffix.lower() in p.stem.lower():
                return p
    return None


def _add_tex_node(
    tree: bpy.types.NodeTree,
    image_path: "Path",
    non_color: bool = False,
) -> bpy.types.ShaderNodeTexImage:
    """Create an Image Texture node loaded from *image_path*."""
    img = bpy.data.images.load(str(image_path))
    tex = tree.nodes.new("ShaderNodeTexImage")
    tex.image = img
    if non_color:
        img.colorspace_settings.name = "Non-Color"
    return tex


def _disconnect_bsdf_input(tree: bpy.types.NodeTree, bsdf: bpy.types.Node, input_name: str) -> None:
    """Remove any existing links feeding *input_name* on the BSDF node."""
    if input_name in bsdf.inputs:
        for link in list(bsdf.inputs[input_name].links):
            tree.links.remove(link)


def _wire_color_channel(
    tree: bpy.types.NodeTree,
    bsdf: bpy.types.Node,
    texture_dir: "Path",
    suffix: str,
    input_name: str,
    non_color: bool = False,
) -> None:
    """Wire a single-output texture (*suffix*) into *input_name* if the file exists."""
    tex_file = _find_texture_file(texture_dir, suffix)
    if not tex_file:
        return
    _disconnect_bsdf_input(tree, bsdf, input_name)
    tex = _add_tex_node(tree, tex_file, non_color=non_color)
    tree.links.new(tex.outputs["Color"], bsdf.inputs[input_name])


def _wire_normal_channel(
    tree: bpy.types.NodeTree, bsdf: bpy.types.Node, texture_dir: "Path"
) -> None:
    """Wire a normal map through a Normal Map node into the BSDF if present."""
    normal_file = _find_texture_file(texture_dir, "NormalGL") or _find_texture_file(
        texture_dir, "Normal"
    )
    if not normal_file:
        return
    _disconnect_bsdf_input(tree, bsdf, "Normal")
    tex = _add_tex_node(tree, normal_file, non_color=True)
    normal_map = tree.nodes.new("ShaderNodeNormalMap")
    tree.links.new(tex.outputs["Color"], normal_map.inputs["Color"])
    tree.links.new(normal_map.outputs["Normal"], bsdf.inputs["Normal"])


def _wire_displacement_channel(tree: bpy.types.NodeTree, texture_dir: "Path") -> None:
    """Wire a displacement map into the material output if present."""
    disp_file = _find_texture_file(texture_dir, "Displacement")
    if not disp_file:
        return
    tex = _add_tex_node(tree, disp_file, non_color=True)
    disp_node = tree.nodes.new("ShaderNodeDisplacement")
    tree.links.new(tex.outputs["Color"], disp_node.inputs["Height"])
    mat_output = next((n for n in tree.nodes if n.type == "OUTPUT_MATERIAL"), None)
    if mat_output and "Displacement" in mat_output.inputs:
        for link in list(mat_output.inputs["Displacement"].links):
            tree.links.remove(link)
        tree.links.new(disp_node.outputs["Displacement"], mat_output.inputs["Displacement"])


def _apply_pbr_textures(bpy_mat: bpy.types.Material, texture_dir: "Path") -> None:
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
        logger.warning("No Principled BSDF in %s, skipping textures", bpy_mat.name)
        return

    _wire_color_channel(tree, bsdf, texture_dir, "Color", "Base Color")
    _wire_color_channel(tree, bsdf, texture_dir, "Roughness", "Roughness", non_color=True)
    _wire_color_channel(tree, bsdf, texture_dir, "Metalness", "Metallic", non_color=True)
    _wire_normal_channel(tree, bsdf, texture_dir)
    _wire_displacement_channel(tree, texture_dir)


def _apply_scalar_material(
    bproc_mat: bproc.types.Material,
    bpy_mat: bpy.types.Material,
    mat_cfg: MaterialConfig,
) -> None:
    """Apply scalar metallic/roughness and optional base color to a material."""
    bproc_mat.set_principled_shader_value("Metallic", mat_cfg.metallic)
    bproc_mat.set_principled_shader_value("Roughness", mat_cfg.roughness)
    if mat_cfg.base_color:
        _disconnect_base_color(bpy_mat)
        rgba = [c / 255.0 for c in mat_cfg.base_color] + [1.0]
        bproc_mat.set_principled_shader_value("Base Color", rgba)


def _apply_material_to_slot(
    bpy_obj: bpy.types.Object,
    bproc_mat: bproc.types.Material,
    slot_idx: int,
    color_mapping: tuple[ColorMappingEntry, ...],
    materials_config: dict[str, MaterialConfig],
    cc_materials: dict[str, bproc.types.Material],
    resolve: PathResolveFn,
) -> str | None:
    """Resolve and apply a material for one slot. Returns the matched type, or None."""
    bpy_mat = bproc_mat.blender_obj
    color = get_material_base_color(bpy_mat)
    if color is None:
        return None
    mat_type, match_dist, used_fallback = _match_material_type(color, color_mapping)
    if mat_type is None:
        logger.warning(
            "No mapping for color %s on %s (nearest distance=%.2f)",
            color,
            bpy_mat.name,
            match_dist,
        )
        return None
    if used_fallback:
        logger.info(
            "Fallback mapped color %s on %s to '%s' (distance=%.2f). "
            "Consider adding an explicit [[robots.color_mapping]] entry.",
            color,
            bpy_mat.name,
            mat_type,
            match_dist,
        )
    mat_cfg = materials_config.get(mat_type, MaterialConfig())

    cc_mat = find_cc_material(mat_cfg.cc_texture, cc_materials) if mat_cfg.cc_texture else None

    if mat_cfg.texture_dir is not None:
        td = resolve(mat_cfg.texture_dir)
        if td.is_dir():
            _apply_pbr_textures(bpy_mat, td)
        else:
            logger.warning("texture_dir not found: %s", td)
    elif cc_mat is not None:
        cc_bpy_mat = cc_mat.blender_obj
        if mat_cfg.base_color:
            cc_bpy_mat = cc_bpy_mat.copy()
            tint_material_albedo(cc_bpy_mat, mat_cfg.base_color)
        bpy_obj.data.materials[slot_idx] = cc_bpy_mat
    else:
        _apply_scalar_material(bproc_mat, bpy_mat, mat_cfg)

    return mat_type


def apply_pbr_materials(
    meshes: list[bproc.types.MeshObject],
    color_mapping: tuple[ColorMappingEntry, ...],
    materials_config: dict[str, MaterialConfig],
    cc_materials: dict[str, bproc.types.Material],
    resolve: PathResolveFn,
) -> None:
    """Apply the configured material for every slot of every mesh."""
    applied: dict[str, int] = {}
    for mesh in meshes:
        bpy_obj = mesh.blender_obj
        for slot_idx, bproc_mat in enumerate(mesh.get_materials()):
            mat_type = _apply_material_to_slot(
                bpy_obj, bproc_mat, slot_idx, color_mapping, materials_config, cc_materials, resolve
            )
            if mat_type is not None:
                applied[mat_type] = applied.get(mat_type, 0) + 1

    logger.info("Material applications: %s", applied)


def jitter_materials(
    meshes: list[bproc.types.MeshObject],
    roughness_jitter: float,
    hue_jitter_deg: float,
) -> None:
    """Apply small random perturbations to material properties.

    Args:
        meshes: Meshes whose materials get jittered.
        roughness_jitter: Max absolute roughness perturbation.
        hue_jitter_deg: Accepted for config parity; hue jitter is not
            currently implemented (matches the original behavior).
    """
    del hue_jitter_deg
    for mesh in meshes:
        for mat in mesh.get_materials():
            try:
                roughness = mat.get_principled_shader_value("Roughness")
                if isinstance(roughness, (int, float)):
                    new_r = max(
                        0.0,
                        min(
                            1.0,
                            roughness + random.uniform(-roughness_jitter, roughness_jitter),
                        ),
                    )
                    mat.set_principled_shader_value("Roughness", new_r)
            except (RuntimeError, KeyError, AttributeError) as e:
                logger.debug("Roughness jitter failed on %s: %s", mat.get_name(), e)
