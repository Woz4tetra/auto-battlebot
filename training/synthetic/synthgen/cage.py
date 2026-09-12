"""Build the NHRL cage in Blender from a `CageSceneSpec` (requires Blender).

Geometry comes from `synthgen.cage_spec`; this module only turns boxes and cylinders into
mesh objects, wires materials, places lights, and sets the camera from a fitted
`CageCalibration`. The pure geometry stays testable without a renderer.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import blenderproc as bproc
import bpy
import cv2
import mathutils
import numpy as np

from auto_battlebot.perception.cage_calibration import blender_cam2world, load_cage_calibration
from synthgen.cage_spec import (
    Box,
    CageSceneSpec,
    Cylinder,
    all_tubes,
    bolt_positions,
    frame_rails,
    house_bot_box,
    mat_boxes,
    mat_image_uv,
    panels,
    pit_boxes,
    posts,
    stage_riser_boxes,
    venue_floor,
)
from synthgen.constants import HOUSE_BOT_CATEGORY_ID
from synthgen.logsetup import get_logger

logger = get_logger(__name__)

# Segmentation id for the mat, away from the robot/distractor ids in synthgen.constants.
MAT_CATEGORY_ID = 7
# HSV saturation above which a house bot texel counts as a lit LED rather than grey steel.
LED_SATURATION_THRESHOLD = 0.35
LED_SATURATION_BOOST = 1.6
LED_STRENGTH_NODE = "led_strength"


def _srgb_to_linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def _linear_rgba(color: tuple[float, float, float]) -> list[float]:
    """Spec colours are sRGB-ish design values; Blender wants linear."""
    return [_srgb_to_linear(float(v)) for v in color] + [1.0]


def _principled(mat: bproc.types.Material) -> Any:
    return mat.get_the_one_node_with_type("BsdfPrincipled")


def make_flat_material(
    name: str, color: tuple[float, float, float], roughness: float, metallic: float = 0.0
) -> bproc.types.Material:
    mat = bproc.material.create(name)
    mat.set_principled_shader_value("Base Color", _linear_rgba(color))
    mat.set_principled_shader_value("Roughness", float(roughness))
    mat.set_principled_shader_value("Metallic", float(metallic))
    return mat


def make_glass_material(
    name: str, ior: float, roughness: float, tint: tuple[float, float, float]
) -> bproc.types.Material:
    mat = bproc.material.create(name)
    mat.set_principled_shader_value("Base Color", [float(v) for v in tint] + [1.0])
    mat.set_principled_shader_value("Roughness", float(roughness))
    mat.set_principled_shader_value("IOR", float(ior))
    mat.set_principled_shader_value("Transmission Weight", 1.0)
    return mat


def make_emission_material(
    name: str, color: tuple[float, float, float], strength: float
) -> bproc.types.Material:
    mat = bproc.material.create(name)
    bsdf = _principled(mat)
    bsdf.inputs["Base Color"].default_value = [0.0, 0.0, 0.0, 1.0]
    bsdf.inputs["Emission Color"].default_value = [float(v) for v in color] + [1.0]
    bsdf.inputs["Emission Strength"].default_value = float(strength)
    return mat


def make_mat_material(
    name: str, albedo_path: Path, roughness: float, specular: float, albedo_gain: float = 1.0
) -> bproc.types.Material:
    """Principled BSDF with the orthographic albedo as base colour."""
    if not albedo_path.exists():
        raise FileNotFoundError(
            f"mat albedo {albedo_path} is missing; build it with "
            "training/synthetic/build_cage_floor_texture.py (needs extract_targets.py and "
            "fit_cage_camera.py first)"
        )
    mat = bproc.material.create(name)
    bsdf = _principled(mat)
    image = bpy.data.images.load(str(albedo_path))
    image.colorspace_settings.name = "sRGB"
    tex = mat.new_node("ShaderNodeTexImage")
    tex.image = image
    tex.interpolation = "Cubic"
    if abs(albedo_gain - 1.0) > 1e-6:
        scale = mat.new_node("ShaderNodeVectorMath")
        scale.operation = "SCALE"
        scale.inputs["Scale"].default_value = float(albedo_gain)
        mat.link(tex.outputs["Color"], scale.inputs[0])
        mat.link(scale.outputs["Vector"], bsdf.inputs["Base Color"])
    else:
        mat.link(tex.outputs["Color"], bsdf.inputs["Base Color"])
    bsdf.inputs["Roughness"].default_value = float(roughness)
    bsdf.inputs["Specular IOR Level"].default_value = float(specular)
    return mat


def _set_mat_uvs(obj: bproc.types.MeshObject, box: Box, mat_size: float) -> None:
    """UVs so the albedo (hfield layout) lands on the mat in the W frame.

    The world position comes from the box, not from `matrix_world`: a primitive created on
    this tick still carries the identity matrix until the depsgraph catches up, and a stale
    matrix silently rescales the texture, or repeats it once the mat is cut around a pit.
    """
    mesh = obj.blender_obj.data
    if not mesh.uv_layers:
        mesh.uv_layers.new(name="UVMap")
    uv_layer = mesh.uv_layers.active.data
    for poly in mesh.polygons:
        for loop_index in poly.loop_indices:
            vertex = mesh.vertices[mesh.loops[loop_index].vertex_index]
            world_x = box.center[0] + float(vertex.co.x) * box.size[0] / 2.0
            world_y = box.center[1] + float(vertex.co.y) * box.size[1] / 2.0
            uv_layer[loop_index].uv = mat_image_uv(world_x, world_y, mat_size)


def add_box(
    box: Box, material: bproc.types.Material, category_id: int = 0
) -> bproc.types.MeshObject:
    obj = bproc.object.create_primitive("CUBE")
    obj.set_name(box.name)
    obj.set_location(list(box.center))
    obj.set_scale([s / 2.0 for s in box.size])
    obj.set_rotation_euler([0.0, 0.0, math.radians(box.yaw_deg)])
    obj.set_cp("category_id", category_id)
    obj.replace_materials(material)
    return obj


def add_cylinder(cyl: Cylinder, material: bproc.types.Material) -> bproc.types.MeshObject:
    obj = bproc.object.create_primitive("CYLINDER", vertices=24)
    obj.set_name(cyl.name)
    obj.set_location(list(cyl.center))
    obj.set_scale([cyl.radius, cyl.radius, cyl.length / 2.0])
    rotation = {"z": [0.0, 0.0, 0.0], "x": [0.0, math.pi / 2, 0.0], "y": [math.pi / 2, 0.0, 0.0]}[
        cyl.axis
    ]
    obj.set_rotation_euler(rotation)
    obj.set_cp("category_id", 0)
    obj.replace_materials(material)
    return obj


def load_cc_material(cc_textures_dir: Path | None, name: str) -> bproc.types.Material | None:
    if not name or name.lower() == "none":
        return None
    if cc_textures_dir is None or not (cc_textures_dir / name).exists():
        return None
    loaded = bproc.loader.load_ccmaterials(str(cc_textures_dir), used_assets=[name])
    for mat in loaded:
        if name.lower() in mat.get_name().lower():
            return mat
    existing = bpy.data.materials.get(name)
    return bproc.types.Material(existing) if existing is not None else None


def _tinted_cc_material(
    cc_textures_dir: Path | None,
    cc_texture: str,
    name: str,
    color: tuple[float, float, float],
    roughness: float,
    metallic: float = 0.6,
    tint_cc: bool = True,
) -> bproc.types.Material:
    """The named ambientCG set if it is on disk, else a flat colour standing in for it."""
    material = load_cc_material(cc_textures_dir, cc_texture)
    if material is None:
        return make_flat_material(name, color, roughness, metallic=metallic)
    if tint_cc:
        material.set_principled_shader_value("Base Color", _linear_rgba(color))
        material.set_principled_shader_value("Roughness", float(roughness))
    return material


def make_pit_materials(
    spec: CageSceneSpec, cc_textures_dir: Path | None
) -> dict[str, bproc.types.Material]:
    """One wall material and one floor material per pit, keyed by the box material name."""
    materials: dict[str, bproc.types.Material] = {}
    for i, pit in enumerate(spec.pits):
        wall = load_cc_material(cc_textures_dir, pit.wall_cc_texture)
        if wall is None:
            wall = make_flat_material(f"pit{i}_wall", pit.wall_color, pit.wall_roughness)
        else:
            wall.set_principled_shader_value("Base Color", _linear_rgba(pit.wall_color))
            wall.set_principled_shader_value("Roughness", float(pit.wall_roughness))
        materials[f"pit{i}_wall"] = wall
        materials[f"pit{i}_floor"] = make_flat_material(
            f"pit{i}_floor", pit.floor_color, pit.floor_roughness
        )
    return materials


def build_cage(
    spec: CageSceneSpec,
    repo_root: Path,
    cc_textures_dir: Path | None = None,
    mat_category_id: int = MAT_CATEGORY_ID,
    house_bot_category_id: int = HOUSE_BOT_CATEGORY_ID,
) -> dict[str, bproc.types.MeshObject]:
    """Every static object of the cage scene, keyed by name."""
    objects: dict[str, bproc.types.MeshObject] = {}

    mat_material = make_mat_material(
        "mat",
        repo_root / spec.mat.albedo,
        spec.mat.roughness,
        spec.mat.specular,
        spec.mat.albedo_gain,
    )
    frame_material = _tinted_cc_material(
        cc_textures_dir, spec.frame.cc_texture, "frame", spec.frame.color, spec.frame.roughness
    )
    floor_material = _tinted_cc_material(
        cc_textures_dir,
        spec.venue.floor_cc_texture,
        "venue_floor",
        spec.venue.floor_color,
        spec.venue.floor_roughness,
        metallic=0.0,
        tint_cc=False,
    )
    panel_material = make_glass_material(
        "panel", spec.panel.ior, spec.panel.roughness, spec.panel.tint
    )
    box_material = make_flat_material(
        "house_bot_box", spec.house_bot_box.color, spec.house_bot_box.roughness
    )
    backdrop_material = make_flat_material("backdrop", spec.backdrop.color, 0.9)

    for box in mat_boxes(spec):
        piece = add_box(box, mat_material, mat_category_id)
        _set_mat_uvs(piece, box, spec.mat.size)
        objects[box.name] = piece

    pit_materials = make_pit_materials(spec, cc_textures_dir)
    for box in pit_boxes(spec):
        objects[box.name] = add_box(box, pit_materials[box.material], mat_category_id)

    for box in frame_rails(spec) + posts(spec) + stage_riser_boxes(spec):
        objects[box.name] = add_box(box, frame_material)
    for bolt in bolt_positions(spec):
        objects[bolt.name] = add_cylinder(bolt, frame_material)
    for box in panels(spec):
        objects[box.name] = add_box(box, panel_material)
    objects["venue_floor"] = add_box(venue_floor(spec), floor_material)
    hb = house_bot_box(spec)
    if hb is not None:
        model = add_house_bot_model(spec, repo_root, house_bot_category_id)
        objects[hb.name] = (
            model if model is not None else add_box(hb, box_material, house_bot_category_id)
        )

    backdrop = bproc.object.create_primitive("CYLINDER", vertices=64)
    backdrop.set_name("backdrop")
    backdrop.set_location([0.0, 0.0, spec.backdrop.height / 2 - spec.venue.floor_drop])
    backdrop.set_scale([spec.backdrop.radius, spec.backdrop.radius, spec.backdrop.height / 2])
    backdrop.set_cp("category_id", 0)
    backdrop.replace_materials(backdrop_material)
    # Drop the cap faces so the tubes above still light the scene.
    mesh = backdrop.blender_obj.data
    cap_polys = [p.index for p in mesh.polygons if abs(p.normal.z) > 0.9]
    if cap_polys:
        import bmesh

        bm = bmesh.new()
        bm.from_mesh(mesh)
        bm.faces.ensure_lookup_table()
        bmesh.ops.delete(bm, geom=[bm.faces[i] for i in cap_polys], context="FACES")
        bm.to_mesh(mesh)
        bm.free()
    objects["backdrop"] = backdrop

    strip_material = make_emission_material(
        "backdrop_strip", spec.backdrop.strip_color, spec.backdrop.strip_strength
    )
    segments = 24
    radius = spec.backdrop.radius - 0.05
    for i in range(segments):
        angle = 2 * math.pi * i / segments
        length = 2 * math.pi * radius / segments
        strip = Box(
            f"strip_{i}",
            (radius * math.cos(angle), radius * math.sin(angle), spec.backdrop.strip_height),
            (0.02, length, spec.backdrop.strip_thickness),
            "strip",
            math.degrees(angle),
        )
        objects[strip.name] = add_box(strip, strip_material)

    logger.info("cage built: %d objects", len(objects))
    return objects


def _find_texture(texture_dir: Path, suffix: str) -> Path | None:
    for path in sorted(texture_dir.iterdir()):
        if suffix.lower() in path.name.lower() and path.suffix.lower() in (".jpg", ".png"):
            return path
    return None


def make_textured_material(
    name: str, texture_dir: Path, albedo_gain: float = 1.0, led_emission: float = 0.0
) -> bproc.types.Material:
    """Principled BSDF fed by an ambientCG-style set (Color, Roughness, Metalness, NormalGL).

    `albedo_gain` scales the colour map. `led_emission` makes the saturated texels (the LED
    eyes and mouth on the house bot) glow with their own colour; grey texels stay dark.
    """
    mat = bproc.material.create(name)
    bsdf = _principled(mat)
    for suffix, socket, non_color in (
        ("Color", "Base Color", False),
        ("Roughness", "Roughness", True),
        ("Metalness", "Metallic", True),
    ):
        path = _find_texture(texture_dir, suffix)
        if path is None:
            continue
        image = bpy.data.images.load(str(path))
        if non_color:
            image.colorspace_settings.name = "Non-Color"
        tex = mat.new_node("ShaderNodeTexImage")
        tex.image = image
        if socket == "Base Color":
            if abs(albedo_gain - 1.0) > 1e-6:
                scale = mat.new_node("ShaderNodeVectorMath")
                scale.operation = "SCALE"
                scale.inputs["Scale"].default_value = float(albedo_gain)
                mat.link(tex.outputs["Color"], scale.inputs[0])
                mat.link(scale.outputs["Vector"], bsdf.inputs[socket])
            else:
                mat.link(tex.outputs["Color"], bsdf.inputs[socket])
            if led_emission > 0:
                hsv = mat.new_node("ShaderNodeSeparateColor")
                hsv.mode = "HSV"
                mat.link(tex.outputs["Color"], hsv.inputs["Color"])
                # The body texel is a slightly bluish grey (saturation ~0.1), the LEDs are
                # 0.6 and up, so gate on saturation above a threshold before scaling.
                gate = mat.new_node("ShaderNodeMath")
                gate.operation = "GREATER_THAN"
                gate.inputs[1].default_value = LED_SATURATION_THRESHOLD
                mat.link(hsv.outputs["Green"], gate.inputs[0])  # saturation
                strength = mat.new_node("ShaderNodeMath")
                strength.name = LED_STRENGTH_NODE
                strength.operation = "MULTIPLY"
                strength.inputs[1].default_value = float(led_emission)
                mat.link(gate.outputs["Value"], strength.inputs[0])
                # Push the LED colour further from grey so it survives the exposure gain.
                saturate = mat.new_node("ShaderNodeHueSaturation")
                saturate.inputs["Saturation"].default_value = LED_SATURATION_BOOST
                mat.link(tex.outputs["Color"], saturate.inputs["Color"])
                mat.link(saturate.outputs["Color"], bsdf.inputs["Emission Color"])
                mat.link(strength.outputs["Value"], bsdf.inputs["Emission Strength"])
            continue
        mat.link(tex.outputs["Color"], bsdf.inputs[socket])
    normal = _find_texture(texture_dir, "NormalGL")
    if normal is not None:
        image = bpy.data.images.load(str(normal))
        image.colorspace_settings.name = "Non-Color"
        tex = mat.new_node("ShaderNodeTexImage")
        tex.image = image
        normal_map = mat.new_node("ShaderNodeNormalMap")
        mat.link(tex.outputs["Color"], normal_map.inputs["Color"])
        mat.link(normal_map.outputs["Normal"], bsdf.inputs["Normal"])
    return mat


def add_house_bot_model(
    spec: CageSceneSpec, repo_root: Path, category_id: int = HOUSE_BOT_CATEGORY_ID
) -> bproc.types.MeshObject | None:
    """The textured house bot GLB on the mat, footprint scaled to the spec size.

    Every mesh of the model carries *category_id* so the segmentation pass can box it as one
    object; the annotation itself is built in the pipeline.
    """
    hb = spec.house_bot_box
    model_path = repo_root / hb.model if hb.model else None
    texture_dir = repo_root / hb.texture_dir if hb.texture_dir else None
    if model_path is None or not model_path.exists():
        logger.warning("house bot model %s missing, using a flat box", model_path)
        return None
    loaded = [
        obj
        for obj in bproc.loader.load_obj(str(model_path))
        if isinstance(obj, bproc.types.MeshObject)
    ]
    if not loaded:
        return None
    if texture_dir is not None and texture_dir.exists():
        material = make_textured_material("house_bot", texture_dir, hb.albedo_gain, hb.led_emission)
        for obj in loaded:
            obj.replace_materials(material)
    parent = bpy.data.objects.new("house_bot", None)
    bpy.context.scene.collection.objects.link(parent)
    for obj in loaded:
        obj.blender_obj.parent = parent
        obj.set_cp("category_id", category_id)

    def world_corners() -> np.ndarray:
        bpy.context.view_layer.update()
        return np.array(
            [
                obj.blender_obj.matrix_world @ mathutils.Vector(c)
                for obj in loaded
                for c in obj.blender_obj.bound_box
            ]
        )

    # The GLB ships lying on its side: roll about its x axis (the LED-face normal) stands it
    # up, then the footprint is measured on the standing box. size[0] <= 0 keeps the model's
    # own dimensions.
    parent.rotation_euler = mathutils.Euler((math.radians(hb.roll_deg), 0.0, 0.0), "XYZ")
    extent = world_corners().max(axis=0) - world_corners().min(axis=0)
    scale = hb.size[0] / max(float(extent[0]), float(extent[1]), 1e-6) if hb.size[0] > 0 else 1.0
    parent.scale = (scale, scale, scale)
    parent.rotation_euler = mathutils.Euler(
        (math.radians(hb.roll_deg), 0.0, math.radians(hb.yaw_deg)), "XYZ"
    )
    corners = world_corners()
    centre_xy = (corners.max(axis=0)[:2] + corners.min(axis=0)[:2]) / 2
    parent.location = (
        hb.position[0] - centre_xy[0],
        hb.position[1] - centre_xy[1],
        -float(corners.min(axis=0)[2]),
    )
    bpy.context.view_layer.update()
    if hb.front_fill > 0:
        # The overhead tubes barely reach the box's vertical face; the real venue lights it
        # from the crowd side. A small area light on the camera side stands in for that.
        light = bproc.types.Light()
        light.set_type("SPOT")
        light.set_name("house_bot_fill")
        target = np.array([hb.position[0], hb.position[1], 0.12])
        source = target + np.array([0.0, -1.5, 0.6])
        light.set_location(source.tolist())
        light.set_energy(float(hb.front_fill))
        light.blender_obj.data.spot_size = math.radians(28.0)
        light.blender_obj.data.spot_blend = 0.4
        light.set_rotation_mat(bproc.camera.rotation_from_forward_vec(target - source))
    return loaded[0]


def house_bot_keypoints(parent: bpy.types.Object) -> tuple[np.ndarray, np.ndarray] | None:
    """Front/back keypoints of the placed house bot, in its parent's local frame.

    Same convention as every other robot in the set, the one
    `nhrl_common.keypoints_along_axis` applies to the CAD distractors: both points sit on the
    model's centerline at ground height, at the front and back extremes of its footprint. The
    axis is not fitted here, it is known: the house bot's LED face is its own +x, so +x is the
    front, the way `HouseBotBoxSpec.roll_deg` and `yaw_deg` already treat it.

    Returns:
        ``(front, back)`` in parent-local metres, or None when the model has no geometry.
    """
    meshes = [child for child in parent.children_recursive if child.type == "MESH"]
    points = np.array(
        [
            (child.matrix_world @ vertex.co).to_tuple()
            for child in meshes
            for vertex in child.data.vertices
        ]
    )
    if points.size == 0:
        return None

    basis = parent.matrix_world.to_3x3()
    axis = np.array((basis @ mathutils.Vector((1.0, 0.0, 0.0))).normalized().to_tuple())
    axis[2] = 0.0  # the model stands upright, so its front axis is horizontal
    axis = axis / max(float(np.linalg.norm(axis)), 1e-9)

    center = (points.max(axis=0) + points.min(axis=0)) / 2
    along = (points - center) @ axis
    ground = float(points[:, 2].min())
    world_from_parent = np.array(parent.matrix_world.inverted())

    def to_local(offset: float) -> np.ndarray:
        world = center + axis * offset
        world[2] = ground
        return np.asarray((world_from_parent @ np.append(world, 1.0))[:3])

    return to_local(float(along.max())), to_local(float(along.min()))


def add_lights(
    spec: CageSceneSpec, color_gain: tuple[float, float, float] = (1.0, 1.0, 1.0)
) -> list[Any]:
    """Emissive LED tubes plus coloured area wash lights."""
    lights: list[Any] = []
    for i, tube in enumerate(all_tubes(spec)):
        color = tuple(float(c) * g for c, g in zip(tube.color, color_gain))
        material = make_emission_material(f"tube_{i}", color, tube.strength)  # type: ignore[arg-type]
        cyl = Cylinder(f"tube_{i}", tube.position, tube.radius, tube.length, "tube", tube.axis)
        lights.append(add_cylinder(cyl, material))
    for i, wash in enumerate(spec.lights.wash):
        light = bproc.types.Light()
        light.set_type("AREA")
        light.set_name(f"wash_{i}")
        light.set_location(list(wash.position))
        light.set_energy(float(wash.strength))
        light.set_color([float(c) * g for c, g in zip(wash.color, color_gain)])
        light.blender_obj.data.size = float(wash.size)
        direction = np.array(wash.look_at) - np.array(wash.position)
        rotation = bproc.camera.rotation_from_forward_vec(direction)
        light.set_rotation_mat(rotation)
        lights.append(light)
    return lights


def set_world(
    spec: CageSceneSpec, color_gain: tuple[float, float, float] = (1.0, 1.0, 1.0)
) -> None:
    color = [float(c) * g for c, g in zip(spec.world.background_color, color_gain)]
    bproc.renderer.set_world_background(color, spec.world.ambient_strength)


def set_led_emission(spec: CageSceneSpec, exposure_gain: float) -> None:
    """Scale the house bot LED emission by the inverse exposure gain.

    `led_emission` is the LED brightness in output units: 1.0 puts the texel colour at full
    scale after the view exposure, so pink stays pink instead of clipping to white.
    """
    for material in bpy.data.materials:
        if not material.use_nodes:
            continue
        node = material.node_tree.nodes.get(LED_STRENGTH_NODE)
        if node is not None:
            node.inputs[1].default_value = float(spec.house_bot_box.led_emission) / max(
                exposure_gain, 1e-6
            )


def apply_exposure(spec: CageSceneSpec, exposure_gain: float) -> None:
    """View exposure plus everything that must scale against it (the LED emission)."""
    set_color_management(exposure_gain)
    set_led_emission(spec, exposure_gain)


def set_color_management(exposure_gain: float) -> None:
    """Plain sRGB so a render is comparable to camera footage; gain in stops."""
    scene = bpy.context.scene
    scene.view_settings.view_transform = "Standard"
    scene.view_settings.look = "None"
    scene.view_settings.exposure = math.log2(max(exposure_gain, 1e-6))
    scene.view_settings.gamma = 1.0
    scene.display_settings.display_device = "sRGB"
    assert scene.view_settings.view_transform == "Standard"


def set_camera(
    k_rect: np.ndarray, width: int, height: int, cam2world: np.ndarray, frame: int | None = None
) -> int:
    """Fixed-camera intrinsics and one pose; returns the frame index."""
    bproc.camera.set_resolution(width, height)
    bproc.camera.set_intrinsics_from_K_matrix(k_rect, width, height)
    return int(bproc.camera.add_camera_pose(cam2world, frame=frame))


# ------------------------------------------------------------------ rendering helpers


def render_poses(
    poses: list[Path],
    k_rect: np.ndarray,
    width: int,
    height: int,
    category_ids: tuple[int, ...] | None = None,
) -> tuple[list[np.ndarray], list[Any]]:
    """Render every pose in one BlenderProc pass.

    Returns (RGB frames, masks). With `category_ids` None each mask is the mat mask; otherwise
    each entry is a list of one mask per requested category id.
    """
    bproc.utility.reset_keyframes()
    for i, pose_path in enumerate(poses):
        calibration = load_cage_calibration(pose_path)
        set_camera(
            k_rect,
            width,
            height,
            blender_cam2world(calibration.tf_camera_from_fieldcenter),
            frame=i,
        )
    data = bproc.renderer.render()
    colors = [np.asarray(frame)[:, :, :3] for frame in data["colors"]]
    segmaps = [np.asarray(seg) for seg in data["category_id_segmaps"]]
    if category_ids is None:
        return colors, [(seg == MAT_CATEGORY_ID).astype(np.uint8) * 255 for seg in segmaps]
    return colors, [
        [(seg == cid).astype(np.uint8) * 255 for cid in category_ids] for seg in segmaps
    ]


def targets_for_pose(pose_path: Path, targets_dir: Path) -> list[Path]:
    """Target frames a pose should be graded against: the clip itself, or every clip of an event."""
    direct = targets_dir / pose_path.stem / "target.png"
    if direct.exists():
        return [direct]
    matches = []
    for meta_path in sorted(targets_dir.glob("*/meta.json")):
        meta = json.loads(meta_path.read_text())
        group = str(meta.get("event_group") or meta.get("event", ""))
        if pose_path.stem.endswith(group) and group:
            matches.append(meta_path.parent / "target.png")
    return matches


def srgb_to_linear(image: np.ndarray) -> np.ndarray:
    c = image.astype(np.float64) / 255.0
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def mat_linear_mean(image_rgb: np.ndarray, mask: np.ndarray) -> np.ndarray:
    linear = srgb_to_linear(image_rgb)
    return np.asarray(linear[mask > 0].reshape(-1, 3).mean(axis=0))


def solve_exposure(
    spec: CageSceneSpec,
    poses: list[Path],
    lights: list[Any],
    color_gain: tuple[float, ...],
    k_rect: np.ndarray,
    width: int,
    height: int,
    samples: int,
    targets_dir: Path,
) -> dict[str, Any]:
    """One quick pass, then scale exposure and light colour so the mat's mean matches footage."""
    bproc.renderer.set_max_amount_of_samples(min(32, samples))
    colors, masks = render_poses(poses, k_rect, width, height)
    ratios = []
    for pose_path, render_rgb, mask in zip(poses, colors, masks):
        for target_path in targets_for_pose(pose_path, targets_dir):
            target_bgr = cv2.imread(str(target_path))
            target_mask = cv2.imread(
                str(target_path.with_name("hull_mask.png")), cv2.IMREAD_GRAYSCALE
            )
            if target_bgr is None or target_mask is None:
                continue
            # A render without a mat category id (the sample renderer) falls back to the hull.
            common = ((mask > 0) & (target_mask > 0)) if (mask > 0).any() else (target_mask > 0)
            if common.sum() < 1000:
                continue
            target_mean = mat_linear_mean(target_bgr[:, :, ::-1], common)
            render_mean = np.maximum(mat_linear_mean(render_rgb, common), 1e-6)
            ratios.append(target_mean / render_mean)
    bproc.renderer.set_max_amount_of_samples(samples)
    if not ratios:
        logger.warning("auto exposure: no target/render mat overlap, leaving exposure alone")
        return {}
    gain = np.median(np.stack(ratios), axis=0)
    overall = float(gain[1])
    tint = tuple(float(v) for v in gain / overall)
    logger.info(
        "auto exposure: gain %.3f, colour gain %s from %d pairs", overall, tint, len(ratios)
    )
    apply_exposure(spec, spec.exposure.gain * overall)
    for light in lights:
        if hasattr(light, "set_color"):
            light.set_color([c * t for c, t in zip(light.get_color(), tint)])
        else:
            node = light.get_materials()[0].get_the_one_node_with_type("BsdfPrincipled")
            colour = node.inputs["Emission Color"].default_value
            node.inputs["Emission Color"].default_value = [
                colour[i] * tint[i] for i in range(3)
            ] + [1.0]
    set_world(spec, tuple(c * t for c, t in zip(color_gain, tint)))  # type: ignore[arg-type]
    return {"gain": overall, "color_gain": list(tint), "pairs": len(ratios)}
