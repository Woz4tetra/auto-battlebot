#!/usr/bin/env python3
"""Simplify a GLTF robot model by replacing or removing internal parts.

Reads a GLTF/GLB file, applies per-part rules based on node names, and
exports a cleaned-up GLB suitable for fast loading in Genesis.

Rules:
  - REPLACE patterns: small fasteners replaced with bounding cylinders
  - DELETE patterns: hidden internals removed entirely
  - Everything else: kept as-is

Usage:
    python simplify_gltf.py INPUT OUTPUT [--max-faces N]

OUTPUT is the desired filename only: the file is always written next to INPUT
as a single GLB (embedded buffers), not a .gltf + sidecar .bin files.
"""

from __future__ import annotations

import argparse
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
import trimesh
import trimesh.visual.material

REPLACE_WITH_CYLINDER: list[str] = [
    "Torx",
    "Rounded Head Plastite",
    "Set Screw",
]

DELETE_PARTS: list[str] = [
    "24MM_BODY",
    "24MM_SCREW_4",
    "24MM_SCREW_5",
    "Battery",
    "Flycolor",
    "RadioMaster R84",
    "ESP32",
    "BNO055",
    "Crossfire",
    "BEC",
    "DYS D3530",
]

CYLINDER_COLOR = (40, 40, 40, 255)
DEFAULT_COLOR = (178, 178, 178, 255)


def _matches(name: str, patterns: list[str]) -> bool:
    return any(p in name for p in patterns)


def _get_color_key(mesh: trimesh.Trimesh) -> tuple[int, int, int, int]:
    """Extract a representative RGBA color from a mesh visual."""
    vis = mesh.visual
    if vis.kind == "texture" and hasattr(vis, "material"):
        color = getattr(vis.material, "baseColorFactor", None)
        if color is not None:
            vals = [float(c) for c in color[:4]]
            # Trimesh can expose baseColorFactor as either 0..1 or 0..255.
            if max(vals) <= 1.0:
                vals = [v * 255.0 for v in vals]
            return tuple(int(np.clip(round(v), 0, 255)) for v in vals)
    elif vis.kind == "vertex" and hasattr(vis, "vertex_colors"):
        vc = np.asarray(vis.vertex_colors)
        if vc.ndim == 2 and vc.shape[1] >= 3 and vc.shape[0] > 0:
            rgba = np.mean(vc[:, :4], axis=0)
            if rgba.shape[0] < 4:
                rgba = np.append(rgba, 255.0)
            return tuple(int(np.clip(round(v), 0, 255)) for v in rgba[:4])
    return DEFAULT_COLOR


def _make_cylinder_for(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """Create a dark cylinder that encompasses the mesh's bounding box."""
    obb = mesh.bounding_box_oriented
    extents = obb.extents
    sorted_axes = np.argsort(extents)
    height = extents[sorted_axes[-1]]
    radius = max(extents[sorted_axes[0]], extents[sorted_axes[1]]) / 2.0

    cyl = trimesh.creation.cylinder(
        radius=max(radius, 0.0002),
        height=max(height, 0.0002),
        sections=8,
    )
    cyl.apply_transform(obb.primitive.transform)
    return cyl


def _apply_pbr(mesh: trimesh.Trimesh, rgba: tuple[int, int, int, int]) -> None:
    """Assign a PBRMaterial with the given color to a mesh."""
    r, g, b, a = rgba
    mat = trimesh.visual.material.PBRMaterial(
        baseColorFactor=[r, g, b, a],
        metallicFactor=0.0,
        roughnessFactor=0.8,
    )
    mesh.visual = trimesh.visual.TextureVisuals(material=mat)


def _mesh_has_uv(mesh: trimesh.Trimesh) -> bool:
    """Whether a mesh visual contains UV coordinates."""
    uv = getattr(mesh.visual, "uv", None)
    return uv is not None and len(uv) > 0


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="Input GLTF/GLB")
    parser.add_argument(
        "output",
        type=Path,
        help="Output base name (saved under the same directory as INPUT; always GLB)",
    )
    parser.add_argument(
        "--max-faces",
        type=int,
        default=0,
        help="Decimate to this many faces (0 = no decimation)",
    )
    args = parser.parse_args()

    if not args.input.exists():
        print(f"Error: {args.input} not found", file=sys.stderr)
        sys.exit(1)

    print(f"Loading {args.input}...")
    scene = trimesh.load(str(args.input))
    if not isinstance(scene, trimesh.Scene):
        print("Error: file did not load as a Scene", file=sys.stderr)
        sys.exit(1)

    kept = 0
    replaced = 0
    deleted = 0

    # Collect meshes grouped by part name.
    part_submeshes: dict[
        str, list[tuple[trimesh.Trimesh, tuple[int, int, int, int]]]
    ] = defaultdict(list)

    for node_name in scene.graph.nodes_geometry:
        transform, geom_name = scene.graph[node_name]
        geom = scene.geometry[geom_name]
        if not isinstance(geom, trimesh.Trimesh):
            continue

        mesh_copy = geom.copy()
        mesh_copy.apply_transform(transform)
        color_key = _get_color_key(mesh_copy)

        parts = node_name.rsplit("_", 1)
        part_name = parts[0] if len(parts) == 2 and len(parts[1]) == 6 else node_name
        part_submeshes[part_name].append((mesh_copy, color_key))

    # Process parts: delete, replace, or keep.
    processed: list[tuple[str, trimesh.Trimesh, bool]] = []
    # (output_name, mesh, force_color_only_material)

    for part_name, entries in part_submeshes.items():
        if _matches(part_name, DELETE_PARTS):
            deleted += 1
            continue

        meshes = [m for m, _ in entries]
        merged = trimesh.util.concatenate(meshes) if len(meshes) > 1 else meshes[0]

        if _matches(part_name, REPLACE_WITH_CYLINDER):
            cyl = _make_cylinder_for(merged)
            processed.append((part_name, cyl, True))
            replaced += 1
        else:
            processed.append((part_name, merged, False))
            kept += 1

    print(
        f"Parts — kept: {kept}, replaced with cylinder: {replaced}, deleted: {deleted}"
    )

    total_faces = sum(mesh.faces.shape[0] for _, mesh, _ in processed)
    print(f"Output parts: {len(processed)}, total faces: {total_faces:,}")

    # Decimate only non-UV meshes so UV-mapped surfaces remain texture-compatible.
    if args.max_faces and total_faces > args.max_faces:
        protected_faces = sum(
            mesh.faces.shape[0]
            for _, mesh, force_color_only in processed
            if _mesh_has_uv(mesh) and not force_color_only
        )
        decimatable_faces = total_faces - protected_faces
        target_decimatable = max(args.max_faces - protected_faces, 0)
        ratio = (
            min(1.0, target_decimatable / decimatable_faces)
            if decimatable_faces > 0
            else 1.0
        )

        if protected_faces > 0:
            print(
                f"Skipping decimation for {protected_faces:,} UV-protected faces "
                f"(target {args.max_faces:,})."
            )

        decimated: list[tuple[str, trimesh.Trimesh, bool]] = []
        for name, mesh, force_color_only in processed:
            can_decimate = not (_mesh_has_uv(mesh) and not force_color_only)
            m = mesh
            if can_decimate:
                target = max(100, int(m.faces.shape[0] * ratio))
                if m.faces.shape[0] > target:
                    m = m.simplify_quadric_decimation(face_count=target)
            decimated.append((name, m, force_color_only))
        processed = decimated
        total_faces = sum(mesh.faces.shape[0] for _, mesh, _ in processed)
        print(f"After decimation: {total_faces:,} faces")

    output_scene = trimesh.Scene()
    name_counts: Counter[str] = Counter()
    for part_name, mesh, force_color_only in processed:
        mesh.update_faces(mesh.nondegenerate_faces())
        mesh.remove_unreferenced_vertices()
        mesh.fix_normals()

        if force_color_only:
            _apply_pbr(mesh, CYLINDER_COLOR)
            base_name = "mat_40_40_40"
        else:
            rgba = _get_color_key(mesh)
            # If no usable visual remained, restore a deterministic flat material.
            if mesh.visual is None:
                _apply_pbr(mesh, rgba)
            base_name = f"mat_{rgba[0]}_{rgba[1]}_{rgba[2]}"

        idx = name_counts[base_name]
        name_counts[base_name] += 1
        geom_name = base_name if idx == 0 else f"{base_name}_{idx}"
        output_scene.add_geometry(mesh, geom_name=geom_name)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    output_scene.export(str(args.output), file_type="glb")
    print(f"Saved to {args.output}")


if __name__ == "__main__":
    main()
