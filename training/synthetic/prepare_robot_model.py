import blenderproc as bproc  # isort: skip  # must be first import for blenderproc run

import argparse
import os
import sys
import tomllib
from pathlib import Path

import bpy
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


def get_material_base_color(mat: bpy.types.Material) -> tuple[int, int, int] | None:
    """Extract the base color RGB (0-255) from a Blender material's Principled BSDF."""
    if not mat.use_nodes:
        return None
    for node in mat.node_tree.nodes:
        if node.type == "BSDF_PRINCIPLED":
            color = node.inputs["Base Color"].default_value
            return (
                int(color[0] * 255),
                int(color[1] * 255),
                int(color[2] * 255),
            )
    return None


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
    print("Use these colors to fill in [[robot.color_mapping]] entries in config.toml.")


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
        help="Save the textured model as a .blend file for inspection",
    )
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else sys.argv[1:]
    args = parser.parse_args(argv)

    config = load_config(args.config)
    model_path = Path(config["robot"]["model_path"])
    scale = config["robot"].get("scale", 1.0)

    bproc.init()

    if args.inspect:
        inspect_model(model_path, scale)
        return

    print(f"Loading robot model from {model_path}")
    meshes = import_gltf(model_path, scale)
    print(f"Loaded {len(meshes)} mesh parts")

    print("\nApplying PBR materials:")
    apply_pbr_materials(
        meshes,
        config["robot"]["color_mapping"],
        config["materials"],
        config.get("environment", {}).get("cc_textures_dir"),
    )

    if args.save_blend:
        bpy.ops.wm.save_as_mainfile(filepath=str(resolve_path(args.save_blend)))
        print(f"\nSaved textured model to {args.save_blend}")
    else:
        print("\nDone. Use --save-blend to save the result.")


if __name__ == "__main__":
    main()
