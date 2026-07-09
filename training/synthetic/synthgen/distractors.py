"""Distractor model loading, placement, and pool management (requires Blender).

Distractors are decoy objects (Objaverse props and CAD NHRL robots) placed
around the target robots. The pool is loaded within an approximate VRAM budget
and periodically re-rolled; CAD distractors can carry keypoint sidecars that
turn into ``nhrl_robot`` annotations in keypoint mode.
"""

import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import blenderproc as bproc
import bpy
import mathutils
import numpy as np

from synthgen.asset_index import (
    DistractorClassIdAssigner,
    PathResolveFn,
    compute_source_budgets,
    discover_model_files,
    load_sidecar_keypoints,
)
from synthgen.configuration import DistractorsConfig, RandomizationConfig
from synthgen.constants import DISTRACTOR_OFFSCREEN_LOCATION
from synthgen.logsetup import get_logger

logger = get_logger(__name__)


@dataclass(eq=False)
class DistractorInstance:
    """One loaded distractor model grouped under a parent empty.

    ``eq=False``: identity semantics for membership tests on live Blender
    handles, same as RobotInstance.
    """

    parent: bpy.types.Object
    meshes: list[bproc.types.MeshObject]
    native_size: float
    source_kind: str
    instance_id: int = 0
    kp_front: np.ndarray | None = None
    kp_back: np.ndarray | None = None

    @property
    def is_robot_like(self) -> bool:
        """True for CAD robot distractors that pose like real robots."""
        return self.source_kind == "cad"

    @property
    def has_keypoints(self) -> bool:
        """True when the model carries sidecar keypoints for annotation."""
        return self.kp_front is not None and self.kp_back is not None


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
    file_path: Path,
    category_id: int,
    source_kind: str,
    instance_id: int = 0,
    keypoints: tuple[np.ndarray, np.ndarray] | None = None,
) -> DistractorInstance | None:
    """Load a single distractor model under a parent empty.

    Returns the loaded instance, or None on failure. The parent empty controls
    transform for the whole group so sub-parts keep their relative positions.
    When *keypoints* is given (CAD distractors in keypoint mode), the front/back
    positions and *instance_id* are kept for later annotation extraction.
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
        bpy.context.scene.collection.objects.link(parent)
        bpy.context.view_layer.update()

        for obj in imported:
            mat = saved_world[obj.name]
            obj.parent = parent
            obj.matrix_parent_inverse = mathutils.Matrix.Identity(4)
            obj.matrix_world = mat

        bpy.context.view_layer.update()

        all_pts = [
            obj.matrix_world @ mathutils.Vector(c) for obj in imported for c in obj.bound_box
        ]
        xs = [p.x for p in all_pts]
        ys = [p.y for p in all_pts]
        zs = [p.z for p in all_pts]
        native_size = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs), 1e-6)

        meshes = []
        for obj in imported:
            mesh = bproc.types.MeshObject(obj)
            mesh.set_cp("category_id", category_id)
            mesh.set_cp("robot_instance_id", instance_id)
            mesh.set_cp("is_distractor", 1)
            meshes.append(mesh)

        return DistractorInstance(
            parent=parent,
            meshes=meshes,
            native_size=native_size,
            source_kind=source_kind,
            instance_id=instance_id,
            kp_front=keypoints[0] if keypoints is not None else None,
            kp_back=keypoints[1] if keypoints is not None else None,
        )
    except Exception as e:
        # Model files come from the wild (Objaverse, mesh reconstruction) and
        # can fail import in arbitrary ways; a bad file must not kill the run.
        logger.warning("Failed to load distractor %s: %s", file_path.name, e)
        return None


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


def estimate_distractor_gpu_mb(distractor: DistractorInstance) -> float:
    """Estimate the distractor's VRAM footprint from mesh + texture data."""
    seen_mesh_data: set[int] = set()
    seen_images: set[int] = set()
    total_bytes = 0

    for mesh in distractor.meshes:
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
                color_bytes = 4 if (mesh_data.color_attributes or mesh_data.vertex_colors) else 0
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


def unload_distractor_pool(pool: list[DistractorInstance]) -> None:
    """Remove all distractor Blender objects and their mesh data."""
    for distractor in pool:
        for m in distractor.meshes:
            obj = m.blender_obj
            mesh_data = obj.data
            bpy.data.objects.remove(obj, do_unlink=True)
            if mesh_data and mesh_data.users == 0:
                bpy.data.meshes.remove(mesh_data)
        bpy.data.objects.remove(distractor.parent, do_unlink=True)
    pool.clear()


def hide_distractor(distractor: DistractorInstance) -> None:
    """Move a distractor far off-screen via its parent."""
    distractor.parent.location = mathutils.Vector(DISTRACTOR_OFFSCREEN_LOCATION)


def place_distractor(
    distractor: DistractorInstance,
    arena_radius: float,
    scale_range: tuple[float, float],
    base_size: float,
    air_probability: float,
    air_height_range: tuple[float, float],
) -> None:
    """Place a distractor at a random position within the arena.

    ``scale_range`` is interpreted as multiples of ``base_size`` (the fixed 3 lb
    weight-class reference dimension), so [0.5, 2.0] means the distractor will be
    between half and twice the 3 lb base size regardless of its native
    dimensions.  The parent empty is used for transform so sub-parts keep their
    relative arrangement.

    With probability ``air_probability`` the object floats above the ground
    with a fully random tumbling orientation.  Otherwise it sits on the
    ground: generic objects keep a fully random orientation, while robot-like
    (CAD robot) distractors sit flat with a random yaw and a 50% chance of
    being inverted, mirroring the target robots' ground poses.
    """
    parent = distractor.parent
    desired_size = base_size * random.uniform(scale_range[0], scale_range[1])
    s = desired_size / distractor.native_size
    parent.scale = (s, s, s)

    airborne = random.random() < air_probability
    if distractor.is_robot_like and not airborne:
        # Grounded robot pose: flat on the field, random yaw, 50% inverted.
        # GLB import is Z-up, so identity roll/pitch sits the model upright.
        roll = math.pi if random.random() < 0.5 else 0.0
        rot = (roll, 0.0, random.uniform(0, 2 * math.pi))
    else:
        rot = (
            random.uniform(0, 2 * math.pi),
            random.uniform(0, 2 * math.pi),
            random.uniform(0, 2 * math.pi),
        )
    parent.rotation_euler = mathutils.Euler(rot)
    parent.location = mathutils.Vector((0, 0, 0))
    bpy.context.view_layer.update()

    all_pts: list[mathutils.Vector] = []
    for m in distractor.meshes:
        obj = m.blender_obj
        all_pts.extend(obj.matrix_world @ mathutils.Vector(c) for c in obj.bound_box)
    ground_z = -min(p.z for p in all_pts)

    if airborne:
        z = ground_z + random.uniform(air_height_range[0], air_height_range[1])
    else:
        z = ground_z

    parent.location = mathutils.Vector(
        (
            random.uniform(-arena_radius, arena_radius),
            random.uniform(-arena_radius, arena_radius),
            z,
        )
    )


def place_scene_distractors(
    pool: list[DistractorInstance],
    dist_cfg: DistractorsConfig,
    rand_cfg: RandomizationConfig,
    arena_radius: float,
) -> list[DistractorInstance]:
    """Select and place a random subset of distractors for one scene."""
    num_dist = random.randint(dist_cfg.min_per_scene, len(pool))
    active = random.sample(pool, num_dist) if num_dist > 0 else []
    for distractor in pool:
        hide_distractor(distractor)
    default_air_prob = rand_cfg.air_probability
    default_air_range = rand_cfg.air_height_range
    # Distractor robots are all treated as the 3 lb weight class; scale off this
    # fixed base rather than whichever robot happens to be in the scene.
    for distractor in active:
        if distractor.is_robot_like:
            air_prob = (
                dist_cfg.robot_air_probability
                if dist_cfg.robot_air_probability is not None
                else default_air_prob
            )
            air_range = (
                dist_cfg.robot_air_height_range
                if dist_cfg.robot_air_height_range is not None
                else default_air_range
            )
        else:
            air_prob = default_air_prob
            air_range = default_air_range
        place_distractor(
            distractor,
            arena_radius,
            dist_cfg.scale_range,
            dist_cfg.base_dimension_m,
            air_probability=air_prob,
            air_height_range=air_range,
        )
    return active


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
    active_distractors: list[DistractorInstance],
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
                for gi, distractor in enumerate(active_distractors):
                    if gi in hidden:
                        continue
                    if any(m.blender_obj == obj for m in distractor.meshes):
                        blockers.setdefault(gi, set()).add(id(kp))
                        break

            if any_kp_clear:
                break

            worst_gi = max(blockers, key=lambda gi: len(blockers[gi]))
            hide_distractor(active_distractors[worst_gi])
            hidden.add(worst_gi)
            bpy.context.view_layer.update()


class DistractorPoolManager:
    """Owns the loaded distractor pool, its VRAM budget, and the shuffle cycle.

    ``refresh()`` re-arms BlenderProc's segmentation output because pass
    indices are assigned to meshes at call time: newly loaded distractor meshes
    would otherwise be missing from the segmentation maps.
    """

    def __init__(
        self,
        dist_cfg: DistractorsConfig,
        assigner: DistractorClassIdAssigner,
        resolve: PathResolveFn,
        load_keypoints: bool,
        enable_segmentation: Callable[[], None],
        vram_estimates: dict[Path, float],
    ) -> None:
        self._cfg = dist_cfg
        self._assigner = assigner
        self._resolve = resolve
        self._load_keypoints = load_keypoints
        self._enable_segmentation = enable_segmentation
        self._vram_estimates = vram_estimates
        self.pool: list[DistractorInstance] = []
        self.images_since_shuffle = 0

    def refresh_due(self) -> bool:
        """True when the shuffle interval has elapsed."""
        return (
            self._cfg.shuffle_interval > 0
            and self.images_since_shuffle >= self._cfg.shuffle_interval
        )

    def note_images_written(self, count: int) -> None:
        """Advance the shuffle counter by *count* written images."""
        self.images_since_shuffle += count

    def refresh(self) -> None:
        """Unload the old pool, load a fresh random one, and re-arm segmentation."""
        if self.pool:
            unload_distractor_pool(self.pool)
        logger.info("Loading distractor models...")
        self.pool = self._load_pool()
        logger.info("%d distractors in pool", len(self.pool))
        if self._cfg.min_per_scene > len(self.pool):
            src_paths = [str(s.path) for s in self._cfg.sources]
            raise RuntimeError(
                f"distractors.min_per_scene is {self._cfg.min_per_scene} but only "
                f"{len(self.pool)} distractor model(s) were found.\n"
                f"  Searched directories: {src_paths}\n"
                f"  Either add distractor models to those directories, or set "
                f"min_per_scene <= {len(self.pool)} in config.toml."
            )
        for distractor in self.pool:
            hide_distractor(distractor)
        # Segmentation pass indices are assigned at call time only. Refreshing
        # distractors creates new meshes, so we must re-run this assignment.
        self._enable_segmentation()
        self.images_since_shuffle = 0

    def _try_load_file(
        self, f: Path, source_kind: str, pool_vram_mb: float
    ) -> tuple[DistractorInstance, float] | None:
        """Load one distractor file within the VRAM budget."""
        budget = self._cfg.vram_budget_mb
        f_resolved = self._resolve(f)
        est_mb_pre = self._vram_estimates.get(f_resolved)
        if (
            est_mb_pre is not None
            and budget is not None
            and budget > 0
            and (pool_vram_mb + est_mb_pre) > budget
        ):
            logger.info(
                "Skipped distractor (pre-check): %s (est %.1f MB) — pool budget %.1f MB "
                "would be exceeded (%.1f MB)",
                f.name,
                est_mb_pre,
                budget,
                pool_vram_mb + est_mb_pre,
            )
            return None

        class_id = self._assigner(f, source_kind)
        keypoints = None
        instance_id = 0
        if self._load_keypoints and source_kind == "cad":
            keypoints = load_sidecar_keypoints(f)
            if keypoints is not None:
                instance_id = self._assigner.allocate_keypoint_instance_id()
        distractor = load_distractor(f, class_id, source_kind, instance_id, keypoints)
        if distractor is None:
            return None

        est_mb = est_mb_pre
        if est_mb is None:
            est_mb = estimate_distractor_gpu_mb(distractor)
            self._vram_estimates[f_resolved] = est_mb
        if budget is not None and budget > 0 and (pool_vram_mb + est_mb) > budget:
            unload_distractor_pool([distractor])
            logger.info(
                "Skipped distractor: %s (est %.1f MB) — pool budget %.1f MB "
                "would be exceeded (%.1f MB)",
                f.name,
                est_mb,
                budget,
                pool_vram_mb + est_mb,
            )
            return None

        return distractor, est_mb

    def _load_pool(self) -> list[DistractorInstance]:
        """Load a weighted-random pool of distractor models."""
        groups = discover_model_files(self._cfg.sources, self._resolve)
        if not groups:
            logger.info("No distractor models found.")
            return []

        for group in groups:
            random.shuffle(group.files)

        budgets = compute_source_budgets(groups, self._cfg.max_per_scene)

        pool: list[DistractorInstance] = []
        pool_vram_mb = 0.0
        for group, budget in zip(groups, budgets):
            loaded = 0
            for f in group.files:
                if loaded >= budget:
                    break
                result = self._try_load_file(f, group.kind, pool_vram_mb)
                if result is None:
                    continue
                distractor, est_mb = result
                pool.append(distractor)
                pool_vram_mb += est_mb
                logger.info(
                    "Loaded distractor: %s (%d meshes, native %.3fm, est %.1f MB)",
                    f.name,
                    len(distractor.meshes),
                    distractor.native_size,
                    est_mb,
                )
                loaded += 1
            logger.info(
                "%s: %d/%d slots (weight %.1f)",
                group.source_dir.name,
                loaded,
                budget,
                group.weight,
            )

        random.shuffle(pool)
        vram_budget = self._cfg.vram_budget_mb
        if vram_budget is not None and vram_budget > 0:
            logger.info(
                "Distractor pool VRAM estimate: %.1f MB (budget %.1f MB)",
                pool_vram_mb,
                vram_budget,
            )
        return pool
