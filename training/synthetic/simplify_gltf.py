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
from collections import defaultdict
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
    """Extract the PBR baseColorFactor as an RGBA tuple, or a default grey."""
    vis = mesh.visual
    if vis.kind == "texture" and hasattr(vis, "material"):
        color = getattr(vis.material, "baseColorFactor", None)
        if color is not None:
            return tuple(int(c) for c in color)
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

    # Collect meshes grouped by part name, preserving material color
    part_submeshes: dict[
        str, list[tuple[trimesh.Trimesh, tuple[int, int, int, int]]]
    ] = defaultdict(list)

    for node_name in scene.graph.nodes_geometry:
        transform, geom_name = scene.graph[node_name]
        geom = scene.geometry[geom_name]
        if not isinstance(geom, trimesh.Trimesh):
            continue

        color_key = _get_color_key(geom)
        mesh_copy = geom.copy()
        mesh_copy.visual = trimesh.visual.ColorVisuals()
        mesh_copy.apply_transform(transform)

        parts = node_name.rsplit("_", 1)
        part_name = parts[0] if len(parts) == 2 and len(parts[1]) == 6 else node_name
        part_submeshes[part_name].append((mesh_copy, color_key))

    # Process parts: delete, replace, or keep — then bucket by color
    color_buckets: dict[tuple[int, int, int, int], list[trimesh.Trimesh]] = defaultdict(
        list
    )

    for part_name, entries in part_submeshes.items():
        if _matches(part_name, DELETE_PARTS):
            deleted += 1
            continue

        meshes = [m for m, _ in entries]
        merged = trimesh.util.concatenate(meshes) if len(meshes) > 1 else meshes[0]

        if _matches(part_name, REPLACE_WITH_CYLINDER):
            cyl = _make_cylinder_for(merged)
            color_buckets[CYLINDER_COLOR].append(cyl)
            replaced += 1
        else:
            dominant_color = max(
                set(c for _, c in entries),
                key=lambda c: sum(1 for _, cc in entries if cc == c),
            )
            color_buckets[dominant_color].append(merged)
            kept += 1

    print(
        f"Parts — kept: {kept}, replaced with cylinder: {replaced}, deleted: {deleted}"
    )

    # Merge each color bucket into one mesh and assign a PBR material
    output_scene = trimesh.Scene()
    total_faces = 0

    for color, bucket_meshes in color_buckets.items():
        merged = (
            trimesh.util.concatenate(bucket_meshes)
            if len(bucket_meshes) > 1
            else bucket_meshes[0]
        )

        if args.max_faces:
            # Proportional decimation per color group
            pass  # handled below on the combined result

        merged.update_faces(merged.nondegenerate_faces())
        merged.remove_unreferenced_vertices()
        merged.fix_normals()

        _apply_pbr(merged, color)
        r, g, b, a = color
        name = f"mat_{r}_{g}_{b}"
        output_scene.add_geometry(merged, geom_name=name)
        total_faces += merged.faces.shape[0]

    print(f"Color groups: {len(color_buckets)}, total faces: {total_faces:,}")

    if args.max_faces and total_faces > args.max_faces:
        # Re-do with proportional decimation per group
        ratio = args.max_faces / total_faces
        output_scene = trimesh.Scene()
        total_faces = 0
        for color, bucket_meshes in color_buckets.items():
            merged = (
                trimesh.util.concatenate(bucket_meshes)
                if len(bucket_meshes) > 1
                else bucket_meshes[0]
            )
            target = max(100, int(merged.faces.shape[0] * ratio))
            if merged.faces.shape[0] > target:
                merged = merged.simplify_quadric_decimation(face_count=target)
            merged.update_faces(merged.nondegenerate_faces())
            merged.remove_unreferenced_vertices()
            merged.fix_normals()
            _apply_pbr(merged, color)
            r, g, b, a = color
            output_scene.add_geometry(merged, geom_name=f"mat_{r}_{g}_{b}")
            total_faces += merged.faces.shape[0]
        print(f"After decimation: {total_faces:,} faces")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    output_scene.export(str(args.output), file_type="glb")
    print(f"Saved to {args.output}")


if __name__ == "__main__":
    main()
