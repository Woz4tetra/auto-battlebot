import blenderproc as bproc  # isort: skip  # must be first import for blenderproc run

import argparse
import math
import os
import random
import sys
import tomllib
from pathlib import Path

import bpy
import cv2
import mathutils
import numpy as np
from bpy_extras.object_utils import world_to_camera_view

_LAUNCH_CWD = Path(os.environ.get("BLENDERPROC_CWD", os.getcwd()))

ROBOT_CATEGORY_ID = 1
DISTRACTOR_CATEGORY_ID = 2
BACKGROUND_CATEGORY_ID = 0

MODEL_EXTENSIONS = {".glb", ".gltf", ".obj", ".ply"}


def resolve_path(path: Path) -> Path:
    """Resolve a potentially relative path against the original launch directory."""
    if path.is_absolute():
        return path
    return (_LAUNCH_CWD / path).resolve()


def load_config(config_path: Path) -> dict:
    with open(resolve_path(config_path), "rb") as f:
        return tomllib.load(f)


# ---------------------------------------------------------------------------
# GLTF import and material helpers
# ---------------------------------------------------------------------------


def get_material_base_color(mat: bpy.types.Material) -> tuple[int, int, int] | None:
    if not mat.use_nodes:
        return None
    for node in mat.node_tree.nodes:
        if node.type == "BSDF_PRINCIPLED":
            color = node.inputs["Base Color"].default_value
            return (int(color[0] * 255), int(color[1] * 255), int(color[2] * 255))
    return None


def color_distance(c1: tuple[int, int, int], c2: list[int]) -> float:
    return float(np.sqrt(sum((a - b) ** 2 for a, b in zip(c1, c2))))


def match_material_type(
    color: tuple[int, int, int], color_mapping: list[dict]
) -> str | None:
    best_match = None
    best_dist = float("inf")
    for entry in color_mapping:
        dist = color_distance(color, entry["color"])
        if dist < entry["tolerance"] and dist < best_dist:
            best_dist = dist
            best_match = entry["material"]
    return best_match


def import_gltf_as_robot(
    model_path: Path, scale: float = 1.0
) -> tuple[list[bproc.types.MeshObject], bpy.types.Object]:
    """Import GLTF and parent all parts under an empty for group transforms.

    Returns (mesh_objects, parent_empty).
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

    parent = bpy.data.objects.new("robot_parent", None)
    bpy.context.scene.collection.objects.link(parent)
    for obj in bpy_meshes:
        obj.parent = parent
    if scale != 1.0:
        parent.scale = (scale, scale, scale)

    bpy.context.view_layer.update()

    bproc_meshes = []
    for obj in bpy_meshes:
        mesh = bproc.types.MeshObject(obj)
        mesh.set_cp("category_id", ROBOT_CATEGORY_ID)
        bproc_meshes.append(mesh)

    return bproc_meshes, parent


def apply_pbr_materials(
    meshes: list[bproc.types.MeshObject],
    color_mapping: list[dict],
    materials_config: dict[str, dict],
    cc_textures_dir: str | None,
) -> None:
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

    for mesh in meshes:
        for bproc_mat in mesh.get_materials():
            bpy_mat = bproc_mat.blender_obj
            color = get_material_base_color(bpy_mat)
            if color is None:
                continue
            mat_type = match_material_type(color, color_mapping)
            if mat_type is None:
                continue
            mat_cfg = materials_config.get(mat_type, {})

            cc_name = mat_cfg.get("cc_texture")
            if cc_name and cc_name in cc_materials:
                mesh.replace_materials(cc_materials[cc_name])
            else:
                bproc_mat.set_principled_shader_value(
                    "Metallic", mat_cfg.get("metallic", 0.0)
                )
                bproc_mat.set_principled_shader_value(
                    "Roughness", mat_cfg.get("roughness", 0.5)
                )


# ---------------------------------------------------------------------------
# Distractor loading
# ---------------------------------------------------------------------------


def discover_model_files(model_dirs: list[str]) -> list[Path]:
    """Find all loadable 3D model files across the distractor directories."""
    files: list[Path] = []
    for dir_path in model_dirs:
        d = resolve_path(Path(dir_path))
        if not d.exists():
            continue
        for ext in MODEL_EXTENSIONS:
            files.extend(d.glob(f"*{ext}"))
    return files


def load_distractor(file_path: Path) -> list[bproc.types.MeshObject] | None:
    """Load a single distractor model. Returns meshes or None on failure."""
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
        meshes = []
        for obj in imported:
            mesh = bproc.types.MeshObject(obj)
            mesh.set_cp("category_id", DISTRACTOR_CATEGORY_ID)
            meshes.append(mesh)
        return meshes if meshes else None
    except Exception as e:
        print(f"  Warning: failed to load distractor {file_path.name}: {e}")
        return None


def load_distractor_pool(
    model_dirs: list[str], pool_size: int
) -> list[list[bproc.types.MeshObject]]:
    """Load a pool of distractor models, returning a list of mesh groups."""
    files = discover_model_files(model_dirs)
    if not files:
        print("  No distractor models found.")
        return []

    random.shuffle(files)
    pool: list[list[bproc.types.MeshObject]] = []
    for f in files:
        if len(pool) >= pool_size:
            break
        meshes = load_distractor(f)
        if meshes:
            pool.append(meshes)
            print(f"  Loaded distractor: {f.name} ({len(meshes)} meshes)")

    return pool


def hide_distractor(meshes: list[bproc.types.MeshObject]) -> None:
    """Move a distractor group far off-screen."""
    for m in meshes:
        m.set_location([1000, 1000, 1000])


def place_distractor(
    meshes: list[bproc.types.MeshObject],
    arena_radius: float,
    scale_range: list[float],
) -> None:
    """Place a distractor at a random position within the arena."""
    pos = [
        random.uniform(-arena_radius, arena_radius),
        random.uniform(-arena_radius, arena_radius),
        0,
    ]
    rot = [0, 0, random.uniform(0, 2 * math.pi)]
    s = random.uniform(scale_range[0], scale_range[1])
    for m in meshes:
        m.set_location(pos)
        m.set_rotation_euler(rot)
        m.set_scale([s, s, s])


# ---------------------------------------------------------------------------
# Camera sampling
# ---------------------------------------------------------------------------


def sample_camera_pose(
    look_at: list[float],
    min_dist: float,
    max_dist: float,
    height_range: list[float],
    noise: float,
) -> np.ndarray:
    """Sample a camera pose on a shell looking at a target point."""
    distance = random.uniform(min_dist, max_dist)
    azimuth = random.uniform(0, 2 * math.pi)
    height = random.uniform(height_range[0], height_range[1])

    cam_x = look_at[0] + distance * math.cos(azimuth)
    cam_y = look_at[1] + distance * math.sin(azimuth)
    cam_z = height

    target = [
        look_at[0] + random.gauss(0, noise),
        look_at[1] + random.gauss(0, noise),
        look_at[2] + random.gauss(0, noise),
    ]

    forward = np.array(target) - np.array([cam_x, cam_y, cam_z])
    forward = forward / np.linalg.norm(forward)

    rotation = bproc.camera.rotation_from_forward_vec(forward)
    cam2world = bproc.math.build_transformation_mat(
        np.array([cam_x, cam_y, cam_z]), rotation
    )
    return cam2world


# ---------------------------------------------------------------------------
# Annotation helpers
# ---------------------------------------------------------------------------


def project_keypoint_to_2d(
    kp_local: np.ndarray, robot_world_mat: np.ndarray, image_size: int
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
    image_size: int,
    tolerance: float = 0.05,
) -> int:
    """Determine keypoint visibility: 0=out-of-frame, 1=occluded, 2=visible."""
    if not (0 <= x_norm <= 1 and 0 <= y_norm <= 1):
        return 0

    px = min(int(x_norm * image_size), image_size - 1)
    py = min(int(y_norm * image_size), image_size - 1)
    rendered_depth = depth_map[py, px]

    if abs(rendered_depth - kp_depth) < tolerance:
        return 2
    return 1


def bbox_from_category_segmap(
    seg_map: np.ndarray, category_id: int, image_size: int
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

    cx = (x_min + x_max) / 2.0 / image_size
    cy = (y_min + y_max) / 2.0 / image_size
    w = (x_max - x_min) / image_size
    h = (y_max - y_min) / image_size
    return (cx, cy, w, h)


def write_yolo_label(
    filepath: Path,
    class_id: int,
    bbox: tuple[float, float, float, float],
    keypoints: list[tuple[float, float, int]],
) -> None:
    """Write a single YOLO keypoint annotation line."""
    cx, cy, w, h = bbox
    parts = [f"{class_id} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}"]
    for kp_x, kp_y, vis in keypoints:
        parts.append(f"{kp_x:.6f} {kp_y:.6f} {vis}")
    with open(filepath, "w") as f:
        f.write(" ".join(parts) + "\n")


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
        "--start-index", type=int, default=0, help="Starting frame index (for resuming)"
    )
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else sys.argv[1:]
    args = parser.parse_args(argv)

    config = load_config(args.config)
    num_images = args.num_images or config["output"]["num_images"]
    image_size = config["output"]["image_size"]
    images_per_scene = config["output"].get("images_per_scene", 5)
    num_scenes = math.ceil(num_images / images_per_scene)

    output_image_dir = resolve_path(Path(config["output"]["image_dir"]))
    output_label_dir = resolve_path(Path(config["output"]["label_dir"]))
    output_image_dir.mkdir(parents=True, exist_ok=True)
    output_label_dir.mkdir(parents=True, exist_ok=True)

    # ------- Initialize BlenderProc -------

    bproc.init()
    bproc.camera.set_resolution(image_size, image_size)
    bproc.renderer.set_max_amount_of_samples(args.render_samples)
    bproc.renderer.enable_depth_output(activate_antialiasing=False)

    # ------- Load target robot -------

    model_path = Path(config["robot"]["model_path"])
    robot_scale = config["robot"].get("scale", 1.0)
    print(f"Loading robot model: {model_path}")
    robot_meshes, robot_parent = import_gltf_as_robot(model_path, robot_scale)
    print(f"  {len(robot_meshes)} mesh parts loaded")

    print("Applying PBR materials to robot...")
    apply_pbr_materials(
        robot_meshes,
        config["robot"]["color_mapping"],
        config["materials"],
        config.get("environment", {}).get("cc_textures_dir"),
    )

    kp_front = np.array(config["robot"]["keypoints"]["front"])
    kp_back = np.array(config["robot"]["keypoints"]["back"])
    class_id = config["robot"]["class_id"]

    # ------- Load distractors -------

    dist_cfg = config.get("distractors", {})
    print("Loading distractor models...")
    distractor_pool = load_distractor_pool(
        dist_cfg.get("model_dirs", []),
        dist_cfg.get("pool_size", 30),
    )
    print(f"  {len(distractor_pool)} distractors in pool")

    for group in distractor_pool:
        hide_distractor(group)

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

    ground_size = config.get("scene", {}).get("ground_size", 3.0)
    ground = bproc.object.create_primitive(
        "PLANE", scale=[ground_size, ground_size, 1], location=[0, 0, 0]
    )
    ground.set_cp("category_id", BACKGROUND_CATEGORY_ID)

    # Enable segmentation AFTER all mesh objects are in the scene, because
    # enable_segmentation_output assigns pass_index to every mesh at call time.
    bproc.renderer.enable_segmentation_output(map_by=["category_id"])

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
    arena_radius = config.get("scene", {}).get("arena_radius", 1.0)

    # ------- Verify category_ids -------

    all_scene_meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
    id_counts: dict[int, int] = {}
    for obj in all_scene_meshes:
        cid = obj.get("category_id", -1)
        id_counts[cid] = id_counts.get(cid, 0) + 1
    print(f"\nCategory ID distribution across {len(all_scene_meshes)} mesh objects:")
    for cid, count in sorted(id_counts.items()):
        label = {
            ROBOT_CATEGORY_ID: "robot",
            DISTRACTOR_CATEGORY_ID: "distractor",
            BACKGROUND_CATEGORY_ID: "background",
        }.get(cid, "UNSET" if cid == -1 else "unknown")
        print(f"  category_id={cid} ({label}): {count} objects")

    # ------- Render loop -------

    global_idx = args.start_index
    print(f"\nRendering {num_images} images across {num_scenes} scenes...\n")

    for scene_idx in range(num_scenes):
        if global_idx >= args.start_index + num_images:
            break

        bproc.utility.reset_keyframes()

        # -- Environment randomization --
        if hdri_paths:
            bproc.world.set_world_background_hdr_img(str(random.choice(hdri_paths)))

        if cc_textures:
            ground.replace_materials(random.choice(cc_textures))

        # -- Robot pose --
        robot_pos = [
            random.uniform(-arena_radius * 0.5, arena_radius * 0.5),
            random.uniform(-arena_radius * 0.5, arena_radius * 0.5),
            0,
        ]
        robot_rot = [0, 0, random.uniform(0, 2 * math.pi)]
        robot_parent.location = mathutils.Vector(robot_pos)
        robot_parent.rotation_euler = mathutils.Euler(robot_rot)
        bpy.context.view_layer.update()

        # -- Distractor placement --
        num_dist = random.randint(
            dist_cfg.get("min_per_scene", 0),
            min(dist_cfg.get("max_per_scene", 5), len(distractor_pool)),
        )
        active_distractors = (
            random.sample(distractor_pool, num_dist) if num_dist > 0 else []
        )
        for group in distractor_pool:
            hide_distractor(group)
        for group in active_distractors:
            place_distractor(
                group,
                arena_radius,
                dist_cfg.get("scale_range", [0.5, 2.0]),
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

        # -- Material jitter on robot --
        jitter_materials(
            robot_meshes,
            rand_cfg.get("roughness_jitter", 0.1),
            rand_cfg.get("hue_jitter_degrees", 5),
        )

        # -- Camera poses --
        look_at = [robot_pos[0], robot_pos[1], robot_pos[2]]
        cam_count = min(images_per_scene, args.start_index + num_images - global_idx)
        for _ in range(cam_count):
            pose = sample_camera_pose(
                look_at=look_at,
                min_dist=cam_cfg.get("min_distance", 0.3),
                max_dist=cam_cfg.get("max_distance", 1.5),
                height_range=cam_cfg.get("height_range", [0.1, 0.8]),
                noise=cam_cfg.get("look_at_noise", 0.05),
            )
            bproc.camera.add_camera_pose(pose)

        # -- Render --
        data = bproc.renderer.render()

        if scene_idx == 0:
            print(f"  Render data keys: {list(data.keys())}")
            debug_img = data["colors"][0]
            debug_path = str(output_image_dir / "_debug_frame0.jpg")
            cv2.imwrite(debug_path, cv2.cvtColor(debug_img, cv2.COLOR_RGB2BGR))
            print(f"  Saved debug image: {debug_path}")

        # -- Extract annotations and save --
        robot_world_mat = np.array(robot_parent.matrix_world)
        colors = data["colors"]
        seg_maps = data.get("category_id_segmaps", data.get("segmap"))
        depth_maps = data["depth"]

        if seg_maps is None:
            if scene_idx == 0:
                print("  ERROR: No segmentation maps in render output!")
            continue

        for local_idx in range(cam_count):
            color_img = colors[local_idx]
            seg_map = seg_maps[local_idx]
            depth_map = depth_maps[local_idx]

            if scene_idx == 0 and local_idx == 0:
                print(f"  Segmap shape={seg_map.shape}, dtype={seg_map.dtype}, "
                      f"unique={np.unique(seg_map).tolist()}")

            bbox = bbox_from_category_segmap(seg_map, ROBOT_CATEGORY_ID, image_size)
            if bbox is None:
                continue

            keypoints_2d: list[tuple[float, float, int]] = []
            for kp_local in [kp_front, kp_back]:
                proj = project_keypoint_to_2d(kp_local, robot_world_mat, image_size)
                if proj is None:
                    keypoints_2d.append((0.0, 0.0, 0))
                    continue
                x_n, y_n, depth = proj
                vis = check_keypoint_visibility(x_n, y_n, depth, depth_map, image_size)
                keypoints_2d.append((x_n, y_n, vis))

            frame_name = f"{global_idx:06d}"
            write_yolo_label(
                output_label_dir / f"{frame_name}.txt",
                class_id,
                bbox,
                keypoints_2d,
            )
            cv2.imwrite(
                str(output_image_dir / f"{frame_name}.jpg"),
                cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR),
            )
            global_idx += 1

        if (scene_idx + 1) % 50 == 0 or scene_idx == num_scenes - 1:
            print(
                f"  Scene {scene_idx + 1}/{num_scenes} — "
                f"{global_idx - args.start_index} images generated"
            )

    total = global_idx - args.start_index
    print(f"\nDone. Generated {total} images in {output_image_dir}")


if __name__ == "__main__":
    main()
