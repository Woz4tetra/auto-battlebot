"""Target robot loading, selection, and posing (requires Blender)."""

import math
import random
from dataclasses import dataclass
from pathlib import Path

import blenderproc as bproc
import bpy
import mathutils
import numpy as np

from synthgen.asset_index import PathResolveFn
from synthgen.configuration import ConfigError, MaterialConfig, RandomizationConfig, RobotConfig
from synthgen.constants import ROBOT_CATEGORY_ID
from synthgen.logsetup import get_logger
from synthgen.materials import apply_pbr_materials

logger = get_logger(__name__)


@dataclass(eq=False)
class RobotInstance:
    """All state for one loaded robot model.

    ``eq=False`` keeps identity semantics: instances hold numpy arrays and live
    Blender handles, and membership tests (``robot in scene_robots``) must mean
    "the same robot object", not field equality.
    """

    name: str
    instance_id: int
    meshes: list[bproc.types.MeshObject]
    parent: bpy.types.Object
    bbox: list[mathutils.Vector]
    size: float
    kp_front: np.ndarray
    kp_back: np.ndarray
    class_id: int
    config: RobotConfig
    weight: float = 1.0


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


def hide_all_robots(robots: list[RobotInstance]) -> None:
    """Hide every robot from rendering."""
    for robot in robots:
        hide_robot(robot)


def choose_scene_robots(robots: list[RobotInstance], max_count: int) -> list[RobotInstance]:
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


def select_and_show_robots(
    robots: list[RobotInstance], max_robots_per_scene: int
) -> list[RobotInstance]:
    """Choose the robots for this scene and toggle their visibility accordingly."""
    scene_robots = choose_scene_robots(robots, max_robots_per_scene)
    for r in robots:
        if r in scene_robots:
            show_robot(r)
        else:
            hide_robot(r)
    return scene_robots


def import_gltf_as_robot(
    model_path: Path,
    resolve: PathResolveFn,
    scale: float = 1.0,
    category_id: int = ROBOT_CATEGORY_ID,
) -> tuple[list[bproc.types.MeshObject], bpy.types.Object, list[mathutils.Vector]]:
    """Import GLTF and parent all parts under an empty for group transforms.

    Returns (mesh_objects, parent_empty, bbox_corners) where bbox_corners are
    the 8 corners of the robot's axis-aligned bounding box in parent-rest-pose
    world space (used to compute ground clearance for arbitrary orientations).
    """
    model_path = resolve(model_path)
    if not model_path.exists():
        raise FileNotFoundError(f"Model file not found: {model_path}")
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

    all_pts = [obj.matrix_world @ mathutils.Vector(c) for obj in bpy_meshes for c in obj.bound_box]
    xs = [p.x for p in all_pts]
    ys = [p.y for p in all_pts]
    zs = [p.z for p in all_pts]
    bbox_corners = [
        mathutils.Vector((x, y, z))
        for x in [min(xs), max(xs)]
        for y in [min(ys), max(ys)]
        for z in [min(zs), max(zs)]
    ]
    logger.info(
        "Robot AABB: x=[%.4f,%.4f] y=[%.4f,%.4f] z=[%.4f,%.4f]",
        min(xs),
        max(xs),
        min(ys),
        max(ys),
        min(zs),
        max(zs),
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


def robot_max_dimension(bbox: list[mathutils.Vector]) -> float:
    """Largest axis-aligned extent of a bounding box."""
    return float(
        max(
            max(c.x for c in bbox) - min(c.x for c in bbox),
            max(c.y for c in bbox) - min(c.y for c in bbox),
            max(c.z for c in bbox) - min(c.z for c in bbox),
        )
    )


def load_robots(
    robot_configs: tuple[RobotConfig, ...],
    materials_config: dict[str, MaterialConfig],
    cc_materials: dict[str, bproc.types.Material],
    is_segmentation_mode: bool,
    seg_robot_class_ids: dict[int, int],
    resolve: PathResolveFn,
) -> list[RobotInstance]:
    """Import each robot model, apply PBR materials, and build RobotInstance records.

    Args:
        robot_configs: Parsed ``[[robots]]`` entries.
        materials_config: Parsed ``[materials.*]`` entries.
        cc_materials: Preloaded CC materials by name.
        is_segmentation_mode: Whether segmentation labels are being produced.
        seg_robot_class_ids: Robot instance id -> segmentation class id.
        resolve: Path resolver for model paths.

    Returns:
        One RobotInstance per config entry, instance ids starting at 1.

    Raises:
        ConfigError: In keypoint mode when a robot entry lacks ``class_id``.
    """
    robots: list[RobotInstance] = []
    for ri, rcfg in enumerate(robot_configs, start=1):
        robot_category_id = seg_robot_class_ids[ri] if is_segmentation_mode else ROBOT_CATEGORY_ID
        if is_segmentation_mode:
            yolo_class_id = robot_category_id
        else:
            if rcfg.class_id is None:
                raise ConfigError(
                    f"robots[{ri - 1}] ({rcfg.name}): 'class_id' is required in keypoint mode"
                )
            yolo_class_id = rcfg.class_id

        logger.info("Loading robot model: %s (%s)", rcfg.name, rcfg.model_path)
        meshes, parent, bbox = import_gltf_as_robot(
            rcfg.model_path, resolve, rcfg.scale, category_id=robot_category_id
        )
        logger.info("%d mesh parts loaded, instance_id=%d", len(meshes), ri)
        size = robot_max_dimension(bbox)
        logger.info("Robot max dimension: %.4f m", size)

        for mesh in meshes:
            mesh.set_cp("robot_instance_id", ri)

        logger.info("Applying PBR materials to %s...", rcfg.name)
        apply_pbr_materials(meshes, rcfg.color_mapping, materials_config, cc_materials, resolve)

        robots.append(
            RobotInstance(
                name=rcfg.name,
                instance_id=ri,
                meshes=meshes,
                parent=parent,
                bbox=bbox,
                size=size,
                kp_front=rcfg.keypoints.front,
                kp_back=rcfg.keypoints.back,
                class_id=yolo_class_id,
                config=rcfg,
                weight=rcfg.weight,
            )
        )
    return robots


def pose_single_robot(
    robot: RobotInstance, rand_cfg: RandomizationConfig, arena_radius: float
) -> list[float]:
    """Randomly pose one robot (airborne or grounded) and return its position."""
    rcfg = robot.config
    airborne = random.random() < rand_cfg.air_probability
    if airborne:
        robot_rot = (
            random.uniform(0, 2 * math.pi),
            random.uniform(0, 2 * math.pi),
            random.uniform(0, 2 * math.pi),
        )
        ground_z = compute_ground_z(robot.meshes, robot.parent, robot_rot)
        air_range = rand_cfg.air_height_range
        robot_z = ground_z + random.uniform(air_range[0], air_range[1])
    else:
        if random.random() < 0.5:
            pitch_deg = -90.0
            roll_deg = rcfg.ground_roll_upright
        else:
            pitch_deg = 90.0
            roll_deg = rcfg.ground_roll_inverted
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
    return robot_pos


def pose_scene_robots(
    scene_robots: list[RobotInstance], rand_cfg: RandomizationConfig, arena_radius: float
) -> list[list[float]]:
    """Pose each robot in the scene and return their positions."""
    return [pose_single_robot(robot, rand_cfg, arena_radius) for robot in scene_robots]
