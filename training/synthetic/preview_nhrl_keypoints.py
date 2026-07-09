import blenderproc as bproc  # isort: skip  # must be first import for blenderproc run

# Render 6-view keypoint previews for NHRL robot distractor models.
#
# Step 4 of the NHRL robot distractor pipeline.  For each model it imports the
# GLB, reads the <stem>.json sidecar, drops a red marker at the front keypoint
# and a blue marker at the back keypoint, and renders a 3x2 grid of cardinal
# views (mirroring prepare_robot_model.py previews) so the bottom-slice
# heuristic can be eyeballed.
#
# Usage:
#     blenderproc run preview_nhrl_keypoints.py -- ../data/distractor_models/robots
#     blenderproc run preview_nhrl_keypoints.py -- ../data/distractor_models/robots --limit 10

import argparse
import json
import math
from pathlib import Path

import bpy
import cv2
import mathutils
import numpy as np

_FRONT_COLOR = (0.9, 0.05, 0.05)  # red
_BACK_COLOR = (0.05, 0.15, 0.9)  # blue
_VIEW_ANGLES = [
    (0, 15, "Front"),
    (180, 15, "Back"),
    (90, 15, "Left"),
    (270, 15, "Right"),
    (0, 89, "Top"),
    (0, -89, "Bottom"),
]


def _model_to_blender_local(vec: list[float]) -> np.ndarray:
    """GLTF (Y-up) model coords -> Blender (Z-up) coords, matching the renderer."""
    x, y, z = (float(v) for v in vec)
    return np.array([x, -z, y], dtype=float)


def _clear_mesh_objects() -> None:
    for obj in list(bpy.context.scene.objects):
        if obj.type in ("MESH", "EMPTY"):
            bpy.data.objects.remove(obj, do_unlink=True)


def _import_model(glb_path: Path) -> list[bpy.types.Object]:
    bpy.ops.import_scene.gltf(filepath=str(glb_path))
    meshes = [o for o in bpy.context.selected_objects if o.type == "MESH"]
    bpy.context.view_layer.update()
    return meshes


def _add_marker(location: np.ndarray, color: tuple[float, float, float], radius: float) -> None:
    sphere = bproc.object.create_primitive("SPHERE", location=[float(v) for v in location])
    obj = sphere.blender_obj
    obj.scale = (radius, radius, radius)
    mat = bpy.data.materials.new(name="kp_marker")
    mat.use_nodes = True
    bsdf = next((n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED"), None)
    if bsdf is not None:
        bsdf.inputs["Base Color"].default_value = (*color, 1.0)
        if "Emission Color" in bsdf.inputs:
            bsdf.inputs["Emission Color"].default_value = (*color, 1.0)
            bsdf.inputs["Emission Strength"].default_value = 6.0
    obj.data.materials.append(mat)


def _read_keypoints(sidecar: Path) -> tuple[np.ndarray, np.ndarray] | None:
    if not sidecar.exists():
        return None
    with sidecar.open() as f:
        data = json.load(f)
    kp = data.get("keypoints")
    if not kp or kp.get("front") is None or kp.get("back") is None:
        return None
    return _model_to_blender_local(kp["front"]), _model_to_blender_local(kp["back"])


def render_grid(
    meshes: list[bpy.types.Object], output_path: Path, img_w: int = 480, img_h: int = 360
) -> None:
    """Render the 6 cardinal views into a labeled 3x2 grid image."""
    bpy.context.view_layer.update()
    all_pts = [obj.matrix_world @ mathutils.Vector(c) for obj in meshes for c in obj.bound_box]
    xs = [p.x for p in all_pts]
    ys = [p.y for p in all_pts]
    zs = [p.z for p in all_pts]
    center = np.array(
        [(min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, (min(zs) + max(zs)) / 2]
    )
    extent = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs), 1e-6)
    radius = extent * 2.0

    bproc.camera.set_resolution(img_w, img_h)
    bproc.renderer.set_max_amount_of_samples(48)

    light = bproc.types.Light()
    light.set_type("POINT")
    light.set_energy(300)
    light.set_location([radius * 1.5, -radius * 1.5, radius * 2.0])
    fill = bproc.types.Light()
    fill.set_type("POINT")
    fill.set_energy(100)
    fill.set_location([-radius * 1.5, radius * 1.5, radius])

    bproc.utility.reset_keyframes()
    for azimuth_deg, elevation_deg, _label in _VIEW_ANGLES:
        az = math.radians(azimuth_deg)
        el = math.radians(elevation_deg)
        cam_pos = center + radius * np.array(
            [math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)]
        )
        forward = center - cam_pos
        forward = forward / np.linalg.norm(forward)
        rotation = bproc.camera.rotation_from_forward_vec(forward)
        cam2world = bproc.math.build_transformation_mat(cam_pos, rotation)
        bproc.camera.add_camera_pose(cam2world)

    data = bproc.renderer.render()
    font = cv2.FONT_HERSHEY_SIMPLEX
    frames = []
    for img, (_az, _el, label) in zip(data["colors"], _VIEW_ANGLES):
        bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.putText(bgr, label, (10, 30), font, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(bgr, label, (10, 30), font, 0.8, (0, 0, 0), 1, cv2.LINE_AA)
        frames.append(bgr)

    grid_rows = [np.concatenate(frames[r * 3 : r * 3 + 3], axis=1) for r in range(2)]
    grid = np.concatenate(grid_rows, axis=0)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), grid)
    print(f"  Preview saved to {output_path}")

    for lgt in (light, fill):
        bpy.data.objects.remove(lgt.blender_obj, do_unlink=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("models_dir", type=Path, help="Directory of robot .glb models")
    parser.add_argument("--models", default="*.glb", help="Glob for models (default: *.glb)")
    parser.add_argument("--output", type=Path, default=Path("../data/previews/nhrl"))
    parser.add_argument("--limit", type=int, default=0, help="Max models to preview (0 = all)")
    args = parser.parse_args()

    models_dir = args.models_dir.resolve()
    glb_files = sorted(models_dir.glob(args.models))
    if args.limit > 0:
        glb_files = glb_files[: args.limit]
    if not glb_files:
        raise SystemExit(f"No models matched {args.models} in {models_dir}")

    bproc.init()
    output_dir = args.output if args.output.is_absolute() else (Path.cwd() / args.output)
    for glb_path in glb_files:
        print(f"Previewing {glb_path.name}")
        _clear_mesh_objects()
        meshes = _import_model(glb_path)
        if not meshes:
            print("  No meshes imported, skipping")
            continue

        extent = 1.0
        all_pts = [obj.matrix_world @ mathutils.Vector(c) for obj in meshes for c in obj.bound_box]
        if all_pts:
            xs = [p.x for p in all_pts]
            ys = [p.y for p in all_pts]
            zs = [p.z for p in all_pts]
            extent = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs), 1e-6)
        marker_radius = max(extent * 0.02, 1e-4)

        keypoints = _read_keypoints(glb_path.with_suffix(".json"))
        if keypoints is None:
            print("  No keypoints in sidecar (missing or degenerate)")
        else:
            front, back = keypoints
            _add_marker(front, _FRONT_COLOR, marker_radius)
            _add_marker(back, _BACK_COLOR, marker_radius)

        render_grid(meshes, output_dir / f"{glb_path.stem}.jpg")

    print(f"\nPreviews written to {output_dir} (red = front, blue = back)")


if __name__ == "__main__":
    main()
