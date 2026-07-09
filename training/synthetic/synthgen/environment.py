"""World, ground plane, and lighting setup/randomization (requires Blender)."""

import random
from pathlib import Path

import blenderproc as bproc
import bpy

from synthgen.asset_index import PathResolveFn
from synthgen.configuration import EnvironmentConfig, RandomizationConfig, SceneConfig
from synthgen.constants import (
    BACKGROUND_CATEGORY_ID,
    DISTRACTOR_CATEGORY_ID,
    ROBOT_CATEGORY_ID,
    SEG_FLOOR_CLASS_ID,
)
from synthgen.logsetup import get_logger

logger = get_logger(__name__)


def load_environment_assets(
    env_cfg: EnvironmentConfig, resolve: PathResolveFn
) -> tuple[list[Path], list[bproc.types.Material]]:
    """Load HDRIs and CC ground textures referenced by the environment config."""
    hdri_dir = resolve(env_cfg.hdri_dir)
    hdri_paths: list[Path] = []
    if hdri_dir.exists():
        hdri_paths = list(hdri_dir.glob("*.hdr")) + list(hdri_dir.glob("*.exr"))
    logger.info("%d HDRIs available", len(hdri_paths))

    cc_textures: list[bproc.types.Material] = []
    if env_cfg.cc_textures_dir is not None and resolve(env_cfg.cc_textures_dir).exists():
        cc_textures = bproc.loader.load_ccmaterials(str(resolve(env_cfg.cc_textures_dir)))
    logger.info("%d CC textures available for ground", len(cc_textures))
    return hdri_paths, cc_textures


def create_ground_plane(is_segmentation_mode: bool) -> bproc.types.MeshObject:
    """Create the ground plane primitive with the correct segmentation category id."""
    ground_category_id = SEG_FLOOR_CLASS_ID if is_segmentation_mode else BACKGROUND_CATEGORY_ID
    ground = bproc.object.create_primitive("PLANE", scale=[1, 1, 1], location=[0, 0, 0])
    ground.set_cp("category_id", ground_category_id)
    ground.set_cp("robot_instance_id", 0)
    ground.set_cp("is_distractor", 0)
    return ground


def create_lights(max_lights: int) -> list[bproc.types.Light]:
    """Create a pool of reusable point lights."""
    lights: list[bproc.types.Light] = []
    for _ in range(max_lights):
        light = bproc.types.Light()
        light.set_type("POINT")
        lights.append(light)
    return lights


def randomize_environment(
    ground: bproc.types.MeshObject,
    hdri_paths: list[Path],
    cc_textures: list[bproc.types.Material],
    scene_cfg: SceneConfig,
) -> None:
    """Randomize the world HDRI and ground plane visibility/material for one scene."""
    if hdri_paths:
        bproc.world.set_world_background_hdr_img(str(random.choice(hdri_paths)))

    show_ground = random.random() < scene_cfg.ground_visibility
    ground.blender_obj.hide_render = not show_ground
    ground.blender_obj.hide_viewport = not show_ground

    if show_ground and cc_textures:
        ground.replace_materials(random.choice(cc_textures))


def randomize_lights(lights: list[bproc.types.Light], rand_cfg: RandomizationConfig) -> None:
    """Randomize the position and energy of active lights for one scene."""
    num_active_lights = random.randint(*rand_cfg.light_count_range)
    intensity_range = rand_cfg.light_intensity_range
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


def log_category_distribution(is_segmentation_mode: bool, seg_label_names: dict[int, str]) -> None:
    """Log the category_id distribution across all mesh objects in the scene."""
    all_scene_meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
    id_counts: dict[int, int] = {}
    for obj in all_scene_meshes:
        cid = obj.get("category_id", -1)
        id_counts[cid] = id_counts.get(cid, 0) + 1
    logger.info("Category ID distribution across %d mesh objects:", len(all_scene_meshes))
    for cid, count in sorted(id_counts.items()):
        if is_segmentation_mode:
            default_label = "background" if cid == BACKGROUND_CATEGORY_ID else "unknown"
            label = seg_label_names.get(cid, default_label)
        else:
            label = {
                ROBOT_CATEGORY_ID: "robot",
                DISTRACTOR_CATEGORY_ID: "distractor",
                BACKGROUND_CATEGORY_ID: "background",
            }.get(cid, "UNSET" if cid == -1 else "unknown")
        logger.info("  category_id=%d (%s): %d objects", cid, label, count)
