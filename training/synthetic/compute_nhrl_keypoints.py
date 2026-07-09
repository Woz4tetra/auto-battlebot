import blenderproc as bproc  # isort: skip  # must be first import for blenderproc run

# Compute front/back keypoints and render a top-down preview for robot meshes.
#
# Step 3 of the NHRL robot distractor pipeline.  Runs under BlenderProc so it can
# render each robot with its real materials (the "sheen" a flat vertex-color bake
# loses).  For every .glb it:
#   - imports the mesh with bpy and reads world-space vertices,
#   - derives centered front/back keypoints from the footprint (nhrl_common),
#   - renders a full-material ORTHOGRAPHIC top-down PNG, and
#   - writes a <stem>.json sidecar with the keypoints plus a "topdown" block
#     (image path, model->pixel affine, footprint hull, y_ground, default axis)
#     so review_nhrl_keypoints.py can overlay a draggable centerline that lines
#     up exactly with the render, without loading the mesh itself.
#
# Usage (under BlenderProc, e.g. inside the synthetic Docker image):
#     blenderproc run compute_nhrl_keypoints.py -- ../data/distractor_models/robots
#     blenderproc run compute_nhrl_keypoints.py -- ../data/distractor_models/robots --overwrite

import argparse
import os
import sys
from datetime import datetime, timezone
from pathlib import Path

import bpy
import cv2
import mathutils
import numpy as np
from bpy_extras.object_utils import world_to_camera_view

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nhrl_common as nc  # noqa: E402  - must follow the sys.path insert above


def _clear_scene() -> None:
    for obj in list(bpy.context.scene.objects):
        if obj.type in ("MESH", "EMPTY"):
            bpy.data.objects.remove(obj, do_unlink=True)


def _import_glb(glb_path: Path) -> list[bpy.types.Object]:
    bpy.ops.import_scene.gltf(filepath=str(glb_path))
    meshes = [o for o in bpy.context.selected_objects if o.type == "MESH"]
    bpy.context.view_layer.update()
    return meshes


def _world_vertices(meshes: list[bpy.types.Object]) -> np.ndarray | None:
    """Concatenated world-space (Blender Z-up) vertices of the imported meshes."""
    chunks: list[np.ndarray] = []
    for obj in meshes:
        mesh = obj.data
        n = len(mesh.vertices)
        if n == 0:
            continue
        co = np.empty(n * 3, dtype=np.float64)
        mesh.vertices.foreach_get("co", co)
        local = np.column_stack([co.reshape(n, 3), np.ones(n)])
        world = local @ np.array(obj.matrix_world).T
        chunks.append(world[:, :3])
    if not chunks:
        return None
    return np.vstack(chunks)


def _world_to_model(world: np.ndarray) -> np.ndarray:
    """Blender world (Z-up) -> GLTF-native model frame (Y-up), matching trimesh.

    Blender's glTF importer rotates the model by +90 deg about X (Y-up -> Z-up),
    i.e. ``model_to_blender`` is ``[x, -z, y]``; this applies the exact inverse.
    """
    return np.column_stack([world[:, 0], world[:, 2], -world[:, 1]])


def _setup_scene(samples: int) -> None:
    bproc.renderer.set_max_amount_of_samples(samples)
    world = bpy.context.scene.world
    world.use_nodes = True
    bg = world.node_tree.nodes.get("Background")
    if bg is not None:
        bg.inputs[0].default_value = (0.16, 0.16, 0.16, 1.0)
        bg.inputs[1].default_value = 0.7  # ambient fill so dark bodies aren't black


def _add_lights(center: np.ndarray, radius: float) -> list:
    lights = []
    for offset, energy in (
        ((1.5, -1.5, 2.5), 600.0),
        ((-1.5, 1.5, 1.5), 250.0),
    ):
        light = bproc.types.Light()
        light.set_type("POINT")
        light.set_energy(energy)
        light.set_location(
            [
                float(center[0] + offset[0] * radius),
                float(center[1] + offset[1] * radius),
                float(center[2] + offset[2] * radius),
            ]
        )
        lights.append(light)
    return lights


def _render_topdown(
    world_verts: np.ndarray, out_png: Path, res: int, pad: float
) -> list[list[float]]:
    """Render an orthographic top-down PNG and return the model(x,z)->pixel affine."""
    lo = world_verts.min(axis=0)
    hi = world_verts.max(axis=0)
    center = 0.5 * (lo + hi)
    ext = hi - lo
    radius = float(max(ext)) * 0.5 + 1e-3

    cam_pos = np.array([center[0], center[1], center[2] + ext[2] * 0.5 + max(ext[0], ext[1]) + 1.0])
    rotation = bproc.camera.rotation_from_forward_vec(np.array([0.0, 0.0, -1.0]))
    cam2world = bproc.math.build_transformation_mat(cam_pos, rotation)
    bproc.camera.set_resolution(res, res)
    bproc.utility.reset_keyframes()
    bproc.camera.add_camera_pose(cam2world)

    cam_ob = bpy.context.scene.camera
    cam_ob.data.type = "ORTHO"
    cam_ob.data.ortho_scale = (1.0 + pad) * max(float(ext[0]), float(ext[1]), 1e-3)
    bpy.context.view_layer.update()

    lights = _add_lights(center, radius)
    data = bproc.renderer.render()
    for light in lights:
        bpy.data.objects.remove(light.blender_obj, do_unlink=True)

    bgr = cv2.cvtColor(data["colors"][0], cv2.COLOR_RGB2BGR)
    out_png.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_png), bgr)

    # Derive the affine [x, z, 1] -> [px, py]. Ortho top-down makes model (x,z)
    # -> pixel exactly affine and independent of height, so 3 points define it.
    def _project(mx: float, mz: float) -> np.ndarray:
        world = mathutils.Vector((mx, -mz, 0.0))  # model->blender world = [x,-z,y]
        co = world_to_camera_view(bpy.context.scene, cam_ob, world)
        return np.array([co.x * res, (1.0 - co.y) * res])

    p00, p10, p01 = _project(0.0, 0.0), _project(1.0, 0.0), _project(0.0, 1.0)
    col0, col1 = p10 - p00, p01 - p00
    return [
        [float(col0[0]), float(col1[0]), float(p00[0])],
        [float(col0[1]), float(col1[1]), float(p00[1])],
    ]


def _metadata_for(glb_path: Path, by_glb: dict[str, dict]) -> dict:
    entry = by_glb.get(glb_path.name)
    if entry is not None:
        return {
            "name": str(entry.get("name", glb_path.stem)),
            "clean_name": str(entry.get("clean_name", "")),
            "weight_classes": list(entry.get("weight_classes", [])),
            "total_fights": int(entry.get("total_fights", 0)),
            "thumbnail_url": str(entry.get("thumbnail_url", "")),
            "meshy_params": entry.get("meshy"),
            "source": "nhrl_meshy",
        }
    return {
        "name": glb_path.stem,
        "clean_name": "",
        "weight_classes": [],
        "total_fights": 0,
        "thumbnail_url": "",
        "meshy_params": None,
        "source": "manual_meshy_export",
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Compute NHRL keypoints + top-down previews")
    parser.add_argument("models_dir", type=Path, help="Directory of robot .glb models")
    parser.add_argument("--models", default="*.glb", help="Glob for models (default: *.glb)")
    parser.add_argument("--overwrite", action="store_true", help="Recompute existing sidecars")
    parser.add_argument("--limit", type=int, default=0, help="Max models to process (0 = all)")
    parser.add_argument("--resolution", type=int, default=640, help="Top-down render size (px)")
    parser.add_argument("--render-samples", type=int, default=64, help="Cycles samples")
    parser.add_argument("--pad", type=float, default=0.12, help="Framing padding fraction")
    parser.add_argument("--slice-fraction", type=float, default=0.05, help="Footprint slice frac")
    args = parser.parse_args()

    models_dir = nc.resolve_cli_path(args.models_dir)
    state = nc.load_state(models_dir / nc.STATE_FILENAME)
    by_glb = {
        str(e["glb_file"]): e for e in state.values() if isinstance(e, dict) and e.get("glb_file")
    }
    glb_files = sorted(models_dir.glob(args.models))
    if args.limit > 0:
        glb_files = glb_files[: args.limit]
    if not glb_files:
        raise SystemExit(f"No models matched {args.models} in {models_dir}")

    bproc.init()
    _setup_scene(args.render_samples)
    generated_at = datetime.now(timezone.utc).isoformat()
    written = degenerate = kept_reviewed = 0

    for glb_path in glb_files:
        sidecar = nc.sidecar_path_for(glb_path)
        existing = nc.load_json(sidecar) if sidecar.exists() else None
        if existing is not None and existing.get("reviewed"):
            kept_reviewed += 1  # human-confirmed; never recompute, even with --overwrite
            continue
        if existing is not None and not args.overwrite:
            continue

        meta = _metadata_for(glb_path, by_glb)
        _clear_scene()
        meshes = _import_glb(glb_path)
        world_verts = _world_vertices(meshes) if meshes else None
        keypoints = None
        topdown = None

        if world_verts is not None:
            model_verts = _world_to_model(world_verts)
            axes = nc.footprint_axes(model_verts, args.slice_fraction)
            if axes is not None:
                _c, u1, _u2, _slice, y_ground = axes
                xz = model_verts[:, [0, 2]]
                base = 0.5 * (xz.min(axis=0) + xz.max(axis=0))
                keypoints = nc.keypoints_along_axis(base, u1, model_verts, y_ground)
                affine = _render_topdown(
                    world_verts,
                    models_dir / "topdown" / f"{glb_path.stem}.png",
                    args.resolution,
                    args.pad,
                )
                hull = nc.convex_hull_2d(xz)
                topdown = {
                    "image": f"topdown/{glb_path.stem}.png",
                    "image_w": args.resolution,
                    "image_h": args.resolution,
                    "affine": affine,
                    "hull": [[round(float(x), 6), round(float(z), 6)] for x, z in hull],
                    "y_ground": round(float(y_ground), 6),
                    "default_axis": [round(float(u1[0]), 6), round(float(u1[1]), 6)],
                }

        sidecar_data = nc.build_sidecar(
            name=meta["name"],
            clean_name=meta["clean_name"],
            weight_classes=meta["weight_classes"],
            total_fights=meta["total_fights"],
            thumbnail_url=meta["thumbnail_url"],
            meshy_params=meta["meshy_params"],
            keypoints=keypoints,
            generated_at=generated_at,
            source=meta["source"],
            topdown=topdown,
        )
        nc.atomic_write_json(sidecar, sidecar_data)
        written += 1
        if keypoints is None:
            degenerate += 1
            print(f"  {glb_path.name}: DEGENERATE (keypoints: null)")
        else:
            print(f"  {glb_path.name}: rendered + keypoints")

    print(
        f"\nWrote {written} sidecars ({degenerate} degenerate, {kept_reviewed} reviewed kept) "
        f"to {models_dir}.\nNext (on host): python review_nhrl_keypoints.py {args.models_dir}"
    )


if __name__ == "__main__":
    main()
