"""Distractor model loading, placement, and pool management (requires Blender)."""

import blenderproc as bproc

import math
import random
from dataclasses import dataclass
from pathlib import Path

import bpy
import mathutils
import numpy as np

from synthgen.asset_index import (
    DistractorClassIdAssigner,
    PathResolveFn,
    VramMb,
    compute_source_budgets,
    discover_model_files,
    load_sidecar_keypoints,
)
from synthgen.configuration import DistractorsConfig, RandomizationConfig
from synthgen.constants import DISTRACTOR_OFFSCREEN_LOCATION
from synthgen.logsetup import get_logger
from synthgen.mesh_bytes import (
    COLOR_BYTES,
    UV_BYTES,
    estimate_geometry_bytes,
    estimate_mip_chain_bytes,
)
from synthgen.pass_index import assign_pass_index

logger = get_logger(__name__)


@dataclass(eq=False)
class DistractorInstance:
    """One loaded distractor model grouped under a parent empty."""

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
    """Connect vertex color attributes to Principled BSDF Base Color."""
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
    """Load a single distractor model under a parent empty, or None on failure."""
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
            assign_pass_index(mesh)
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
    return estimate_mip_chain_bytes(base_level)


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

                uv_bytes = UV_BYTES if mesh_data.uv_layers else 0
                has_color = mesh_data.color_attributes or mesh_data.vertex_colors
                color_bytes = COLOR_BYTES if has_color else 0
                total_bytes += estimate_geometry_bytes(
                    vertex_count, tri_count, uv_bytes=uv_bytes, color_bytes=color_bytes
                )

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
    """Take a distractor out of the render: hidden, and parked far off-screen via its parent."""
    for mesh in distractor.meshes:
        mesh.blender_obj.hide_render = True
        mesh.blender_obj.hide_viewport = True
    distractor.parent.location = mathutils.Vector(DISTRACTOR_OFFSCREEN_LOCATION)


def show_distractor(distractor: DistractorInstance) -> None:
    """Make a distractor renderable again; ``place_distractor`` positions it."""
    for mesh in distractor.meshes:
        mesh.blender_obj.hide_render = False
        mesh.blender_obj.hide_viewport = False


def place_distractor(
    distractor: DistractorInstance,
    arena_radius: float,
    scale_range: tuple[float, float],
    base_size: float,
    air_probability: float,
    air_height_range: tuple[float, float],
) -> None:
    """Place a distractor at a random position within the arena."""
    show_distractor(distractor)
    parent = distractor.parent
    desired_size = base_size * random.uniform(scale_range[0], scale_range[1])
    s = desired_size / distractor.native_size
    parent.scale = (s, s, s)

    airborne = random.random() < air_probability
    if distractor.is_robot_like and not airborne:
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
    """Cast a ray from origin toward target; return the first distractor hit or None."""
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
    """Hide distractors that block the camera's view of every robot keypoint."""
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
    """Owns the loaded distractor pool, its VRAM budget, and the shuffle cycle."""

    def __init__(
        self,
        dist_cfg: DistractorsConfig,
        assigner: DistractorClassIdAssigner,
        resolve: PathResolveFn,
        load_keypoints: bool,
        vram_estimates: dict[Path, VramMb],
    ) -> None:
        self._cfg = dist_cfg
        self._assigner = assigner
        self._resolve = resolve
        self._load_keypoints = load_keypoints
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
        """Unload the old pool and load a fresh random one."""
        if self.pool:
            unload_distractor_pool(self.pool)
        logger.info("Loading distractor models...")
        self.pool = self._load_pool()
        logger.info("%d distractors in pool", len(self.pool))
        if self._cfg.min_per_scene > len(self.pool):
            src_paths = [str(s.path) for s in self._cfg.sources]
            raise RuntimeError(
                f"distractors.min_per_scene is {self._cfg.min_per_scene} but only "
                f"{len(self.pool)} model(s) found in {src_paths}"
            )
        for distractor in self.pool:
            hide_distractor(distractor)
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
