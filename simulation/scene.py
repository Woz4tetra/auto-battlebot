"""MuJoCo scene builder: arena, robots, camera."""

from __future__ import annotations

import math
from pathlib import Path

import cv2
import mujoco
import numpy as np
import numpy.typing as npt
from scipy.spatial.transform import Rotation as R

from camera_utils import camera_view_matrix, fov_to_intrinsics, project_panorama
from config import SimConfig
from config.robot import RobotConfig
from sim_types import SceneHandles, WheelDriveInfo
from sim_types.scene_handles import MocapHandle, RobotHandle


# ---------------------------------------------------------------------------
# Pose helpers
# ---------------------------------------------------------------------------


def _yaw_quat(
    model_euler_deg: list[float], yaw_rad: float
) -> tuple[float, float, float, float]:
    """Compose URDF model correction with world-space yaw. Returns (w, x, y, z)."""
    base = R.from_euler("XYZ", model_euler_deg, degrees=True)
    yaw = R.from_euler("Z", yaw_rad)
    combined = yaw * base
    x, y, z, w = combined.as_quat()
    return (w, x, y, z)


def _resolve(base_dir: Path, p: str) -> Path:
    pp = Path(p)
    return pp if pp.is_absolute() else (base_dir / pp).resolve()


def _pad3(pos: list[float]) -> tuple[float, float, float]:
    if len(pos) == 2:
        return (pos[0], pos[1], 0.0)
    return (pos[0], pos[1], pos[2])


def _camera_xyaxes(pos: list[float], lookat: list[float]) -> str:
    """MJCF xyaxes string for a camera looking from pos toward lookat."""
    p_arr = np.array(pos, dtype=float)
    t_arr = np.array(lookat, dtype=float)
    view_dir = t_arr - p_arr
    view_dir /= np.linalg.norm(view_dir)
    cam_z = -view_dir  # camera Z points away from scene
    world_up = np.array([0.0, 0.0, 1.0])
    cam_x = np.cross(world_up, cam_z)
    if np.linalg.norm(cam_x) < 1e-6:
        cam_x = np.array([1.0, 0.0, 0.0])
    else:
        cam_x /= np.linalg.norm(cam_x)
    cam_y = np.cross(cam_z, cam_x)
    cam_y /= np.linalg.norm(cam_y)
    return (
        f"{cam_x[0]:.6f} {cam_x[1]:.6f} {cam_x[2]:.6f} "
        f"{cam_y[0]:.6f} {cam_y[1]:.6f} {cam_y[2]:.6f}"
    )


# ---------------------------------------------------------------------------
# Mesh pre-processing helpers
# ---------------------------------------------------------------------------


def _glb_color_rgba(geom_key: str) -> str:
    """Decode 'mat_R_G_B' geometry key to MJCF rgba string, or return gray."""
    parts = geom_key.split("_")
    if len(parts) == 4 and parts[0] == "mat":
        try:
            r, g, b = int(parts[1]), int(parts[2]), int(parts[3])
            return f"{r / 255:.3f} {g / 255:.3f} {b / 255:.3f} 1"
        except ValueError:
            pass
    return "0.6 0.6 0.6 1"


def _convert_glb(glb_path: Path) -> list[tuple[str, Path, str]]:
    """Convert a GLB to per-geometry OBJ files in ``_mujoco/`` beside the source.

    Returns list of (geom_key, obj_path, rgba_string).
    Cached; rebuilds only when source GLB is newer than the stamp file.
    """
    import trimesh as _trimesh

    cache_dir = glb_path.parent / "_mujoco"
    sentinel = cache_dir / f"{glb_path.stem}.stamp"

    needs_rebuild = (
        not sentinel.exists() or glb_path.stat().st_mtime > sentinel.stat().st_mtime
    )

    if needs_rebuild:
        cache_dir.mkdir(exist_ok=True)
        scene = _trimesh.load(str(glb_path), force="scene", process=False)
        geoms = scene.geometry if hasattr(scene, "geometry") else {}
        for key, geom in geoms.items():
            safe_key = key.replace(" ", "_")
            out_obj = cache_dir / f"{glb_path.stem}_{safe_key}.obj"
            geom.export(str(out_obj))
        sentinel.touch()

    result: list[tuple[str, Path, str]] = []
    for f in sorted((glb_path.parent / "_mujoco").glob(f"{glb_path.stem}_*.obj")):
        geom_key = f.stem[len(glb_path.stem) + 1 :]
        rgba = _glb_color_rgba(geom_key)
        result.append((geom_key, f, rgba))
    return result


def _split_obj_by_material(
    obj_path: Path,
) -> list[tuple[str, Path, Path | None]]:
    """Split a multi-material OBJ (trimesh interleaved format) into one OBJ
    per material in a ``_mujoco/`` sub-directory beside the source file.

    Returns a list of (material_name, split_obj_path, texture_png_path_or_None).
    Results are cached; rebuild is triggered if the source OBJ is newer.
    """
    import trimesh as _trimesh

    cache_dir = obj_path.parent / "_mujoco"
    sentinel = cache_dir / f"{obj_path.stem}.stamp"

    needs_rebuild = (
        not sentinel.exists() or obj_path.stat().st_mtime > sentinel.stat().st_mtime
    )

    if needs_rebuild:
        cache_dir.mkdir(exist_ok=True)
        scene = _trimesh.load(str(obj_path), force="scene", process=False)
        geoms = scene.geometry if hasattr(scene, "geometry") else {}
        for _geom_key, geom in geoms.items():
            # Use the material's own name as the file stem (not the OBJ object name)
            mat = getattr(geom, "visual", None)
            mat_name: str = (
                getattr(getattr(mat, "material", None), "name", None) or _geom_key
            )
            out_obj = cache_dir / f"{obj_path.stem}_{mat_name}.obj"
            geom.export(str(out_obj))
        sentinel.touch()

    result: list[tuple[str, Path, Path | None]] = []
    cache_dir.mkdir(exist_ok=True)
    for f in sorted(cache_dir.glob(f"{obj_path.stem}_*.obj")):
        mat_name = f.stem[len(obj_path.stem) + 1 :]
        # look for a same-named PNG texture in the original mesh directory
        tex: Path | None = None
        for ext in (".png", ".jpg", ".jpeg"):
            candidate = obj_path.parent / f"{mat_name}{ext}"
            if candidate.exists():
                tex = candidate
                break
        result.append((mat_name, f, tex))

    return result


# ---------------------------------------------------------------------------
# Per-robot MJCF generators
# Each returns (asset_elems: list[str], body_xml: str, actuator_xml: str)
# ---------------------------------------------------------------------------


def _mr_stabs_xml(
    prefix: str,
    pos: tuple[float, float, float],
    quat_wxyz: tuple[float, float, float, float],
    mesh_dir: Path,
    cfg: RobotConfig,
) -> tuple[list[str], str, str]:
    """MJCF for mr_stabs_mk2: 2-wheel differential drive, caster spheres."""
    px, py, pz = pos
    qw, qx, qy, qz = quat_wxyz

    chassis_path = mesh_dir / "chassis.obj"
    chassis_col_path = mesh_dir / "chassis_collision.obj"
    wheel_path = mesh_dir / "wheel.obj"
    col_name = f"{prefix}chassis_col"
    col_file = chassis_col_path if chassis_col_path.exists() else chassis_path

    assets: list[str] = [
        f'<mesh name="{col_name}" file="{col_file}"/>',
    ]

    # Split chassis.obj into per-material sub-meshes so MuJoCo loads all faces.
    # (trimesh exports interleaved vertex/face blocks; MuJoCo only reads the first.)
    chassis_vis_geoms: list[str] = []
    for mat_name, sub_obj, tex_png in _split_obj_by_material(chassis_path):
        safe = mat_name.replace(".", "_").replace(" ", "_")
        mesh_id = f"{prefix}chassis_{safe}"
        assets.append(f'<mesh name="{mesh_id}" file="{sub_obj}"/>')
        mat_attr = ""
        if tex_png is not None:
            tex_id = f"{mesh_id}_tex"
            mat_id = f"{mesh_id}_mat"
            assets.append(f'<texture name="{tex_id}" type="2d" file="{tex_png}"/>')
            assets.append(
                f'<material name="{mat_id}" texture="{tex_id}"'
                ' specular="0.1" shininess="0.1"/>'
            )
            mat_attr = f' material="{mat_id}"'
        chassis_vis_geoms.append(
            f'<geom name="{prefix}chassis_vis_{safe}" type="mesh"'
            f' mesh="{mesh_id}"{mat_attr} contype="0" conaffinity="0"/>'
        )

    # Wheel visual mesh
    wheel_vis_l = wheel_vis_r = ""
    if wheel_path.exists():
        wheel_parts = _split_obj_by_material(wheel_path)
        if wheel_parts:
            # wheels are single-material; just use the first (and only) part
            wmat_name, w_obj, w_tex = wheel_parts[0]
            wsafe = wmat_name.replace(".", "_").replace(" ", "_")
            wmesh_id = f"{prefix}wheel_{wsafe}"
            assets.append(f'<mesh name="{wmesh_id}" file="{w_obj}"/>')
            wmat_attr = ""
            if w_tex is not None:
                tex_id = f"{wmesh_id}_tex"
                mat_id = f"{wmesh_id}_mat"
                assets.append(f'<texture name="{tex_id}" type="2d" file="{w_tex}"/>')
                assets.append(
                    f'<material name="{mat_id}" texture="{tex_id}"'
                    ' specular="0.05" shininess="0.05"/>'
                )
                wmat_attr = f' material="{mat_id}"'
            wheel_vis_l = (
                f'<geom name="{prefix}lw_vis" type="mesh" mesh="{wmesh_id}"'
                f'{wmat_attr} contype="0" conaffinity="0"/>'
            )
            wheel_vis_r = (
                f'<geom name="{prefix}rw_vis" type="mesh" mesh="{wmesh_id}"'
                f'{wmat_attr} contype="0" conaffinity="0"/>'
            )

    arm = cfg.wheel_armature
    dmp = cfg.wheel_damping
    frl = cfg.wheel_frictionloss
    kv = cfg.wheel_kv
    flim = cfg.wheel_force_limit
    cf = cfg.wheel_contact_friction
    fr = f"{cf[0]} {cf[1]} {cf[2]}"

    chassis_vis_block = "\n      ".join(chassis_vis_geoms)

    body_xml = f"""\
    <body name="{prefix}base_link" pos="{px} {py} {pz}" quat="{qw} {qx} {qy} {qz}">
      <freejoint name="{prefix}root"/>
      <inertial mass="4.228e-1" pos="0 0 -4e-3"
                fullinertia="7.503e-4 5.076e-4 1.187e-3 5.579e-8 1.835e-6 6.406e-9"/>
      <geom name="{prefix}chassis_col" type="mesh" mesh="{col_name}"
            group="3" friction="0.3 0.3 0.001"/>
      {chassis_vis_block}
      <body name="{prefix}caster_front" pos="0.08 0 -0.020">
        <inertial mass="1e-3" pos="0 0 0" diaginertia="1e-8 1e-8 1e-8"/>
        <geom name="{prefix}caster_f" type="sphere" size="0.001"
              group="3" friction="0.01 0.01 0.001"/>
      </body>
      <body name="{prefix}caster_rear" pos="-0.08 0 -0.020">
        <inertial mass="1e-3" pos="0 0 0" diaginertia="1e-8 1e-8 1e-8"/>
        <geom name="{prefix}caster_r" type="sphere" size="0.001"
              group="3" friction="0.01 0.01 0.001"/>
      </body>
      <body name="{prefix}left_wheel" pos="0 0.064 0">
        <joint name="{prefix}left_wheel_joint" type="hinge" axis="0 1 0"
               armature="{arm}" damping="{dmp}" frictionloss="{frl}"/>
        <inertial mass="1.5e-2" pos="0 0 5.8e-6"
                  fullinertia="1.821e-6 3.325e-6 1.821e-6 3.700e-9 -9.361e-9 -3.702e-9"/>
        <geom name="{prefix}lw_col" type="cylinder" size="0.025 0.006" euler="90 0 0"
              group="3" friction="{fr}"/>
        {wheel_vis_l}
      </body>
      <body name="{prefix}right_wheel" pos="0 -0.064 0">
        <joint name="{prefix}right_wheel_joint" type="hinge" axis="0 1 0"
               armature="{arm}" damping="{dmp}" frictionloss="{frl}"/>
        <inertial mass="1.5e-2" pos="0 0 5.8e-6"
                  fullinertia="1.821e-6 3.325e-6 1.821e-6 3.700e-9 -9.361e-9 -3.702e-9"/>
        <geom name="{prefix}rw_col" type="cylinder" size="0.025 0.006" euler="90 0 0"
              group="3" friction="{fr}"/>
        {wheel_vis_r}
      </body>
    </body>"""

    actuator_xml = (
        f'<velocity name="{prefix}left_wheel_joint_vel"'
        f' joint="{prefix}left_wheel_joint"'
        f' kv="{kv}" forcelimited="true" forcerange="-{flim} {flim}"/>\n'
        f'    <velocity name="{prefix}right_wheel_joint_vel"'
        f' joint="{prefix}right_wheel_joint"'
        f' kv="{kv}" forcelimited="true" forcerange="-{flim} {flim}"/>'
    )

    return assets, body_xml, actuator_xml


def _house_bot_xml(
    prefix: str,
    pos: tuple[float, float, float],
    quat_wxyz: tuple[float, float, float, float],
    mesh_dir: Path,
    cfg: RobotConfig,
) -> tuple[list[str], str, str]:
    """MJCF for house_bot: 4-wheel differential drive (2 left, 2 right)."""
    px, py, pz = pos
    qw, qx, qy, qz = quat_wxyz

    chassis_path = mesh_dir / "chassis.obj"
    chassis_col_path = mesh_dir / "chassis_collision.obj"
    col_name = f"{prefix}chassis_col"
    col_file = chassis_col_path if chassis_col_path.exists() else chassis_path

    assets: list[str] = [
        f'<mesh name="{col_name}" file="{col_file}"/>',
    ]

    # Split chassis.obj into per-material sub-meshes
    chassis_vis_geoms: list[str] = []
    for mat_name, sub_obj, tex_png in _split_obj_by_material(chassis_path):
        safe = mat_name.replace(".", "_").replace(" ", "_")
        mesh_id = f"{prefix}chassis_{safe}"
        assets.append(f'<mesh name="{mesh_id}" file="{sub_obj}"/>')
        mat_attr = ""
        if tex_png is not None:
            tex_id = f"{mesh_id}_tex"
            mat_id = f"{mesh_id}_mat"
            assets.append(f'<texture name="{tex_id}" type="2d" file="{tex_png}"/>')
            assets.append(
                f'<material name="{mat_id}" texture="{tex_id}"'
                ' specular="0.1" shininess="0.1"/>'
            )
            mat_attr = f' material="{mat_id}"'
        chassis_vis_geoms.append(
            f'<geom name="{prefix}chassis_vis_{safe}" type="mesh"'
            f' mesh="{mesh_id}"{mat_attr} contype="0" conaffinity="0"/>'
        )

    arm = cfg.wheel_armature
    dmp = cfg.wheel_damping
    frl = cfg.wheel_frictionloss
    kv = cfg.wheel_kv
    flim = cfg.wheel_force_limit
    cf = cfg.wheel_contact_friction
    fr = f"{cf[0]} {cf[1]} {cf[2]}"

    wheel_joint_tmpl = (
        '<body name="{prefix}{wname}" pos="{wx} {wy} {wz}">\n'
        '        <joint name="{prefix}{jname}" type="hinge" axis="0 1 0"'
        ' armature="{arm}" damping="{dmp}" frictionloss="{frl}"/>\n'
        '        <inertial mass="5e-1" pos="0 0 0" diaginertia="2.17e-4 4.0e-4 2.17e-4"/>\n'
        '        <geom name="{prefix}{gname}" type="cylinder" size="0.04 0.0125"'
        ' euler="90 0 0" group="3" friction="{fr}"/>\n'
        "      </body>"
    )

    wheel_bodies = ""
    for wname, jname, wx, wy, wz in [
        ("front_left_wheel", "front_left_wheel_joint", 0.1, 0.1, -0.083),
        ("front_right_wheel", "front_right_wheel_joint", 0.1, -0.1, -0.083),
        ("rear_left_wheel", "rear_left_wheel_joint", -0.1, 0.1, -0.083),
        ("rear_right_wheel", "rear_right_wheel_joint", -0.1, -0.1, -0.083),
    ]:
        wheel_bodies += "\n      " + wheel_joint_tmpl.format(
            prefix=prefix,
            wname=wname,
            jname=jname,
            wx=wx,
            wy=wy,
            wz=wz,
            arm=arm,
            dmp=dmp,
            frl=frl,
            gname=f"{wname}_col",
            fr=fr,
        )

    chassis_vis_block = "\n      ".join(chassis_vis_geoms)

    body_xml = f"""\
    <body name="{prefix}base_link" pos="{px} {py} {pz}" quat="{qw} {qx} {qy} {qz}">
      <freejoint name="{prefix}root"/>
      <inertial mass="4.8e1" pos="0 0 0"
                fullinertia="6.49e-1 4.72e-1 6.94e-1 0 0 0"/>
      <geom name="{prefix}chassis_col" type="mesh" mesh="{col_name}"
            group="3" friction="0.3 0.3 0.001"/>
      {chassis_vis_block}
      {wheel_bodies}
    </body>"""

    actuator_lines = []
    for jname in [
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    ]:
        actuator_lines.append(
            f'<velocity name="{prefix}{jname}_vel"'
            f' joint="{prefix}{jname}"'
            f' kv="{kv}" forcelimited="true" forcerange="-{flim} {flim}"/>'
        )
    actuator_xml = "\n    ".join(actuator_lines)

    return assets, body_xml, actuator_xml


def _mocap_xml(
    prefix: str,
    pos: tuple[float, float, float],
    quat_wxyz: tuple[float, float, float, float],
    model_path: Path,
) -> tuple[list[str], str, str]:
    """MJCF for a kinematically-controlled body (no physics joints).

    GLB files are converted to per-geometry OBJs via trimesh at build time.
    OBJ/STL/MSH files are loaded directly.
    """
    px, py, pz = pos
    qw, qx, qy, qz = quat_wxyz

    assets: list[str] = []
    vis_geoms: list[str] = []

    ext = model_path.suffix.lower()
    if ext in {".obj", ".stl", ".msh"}:
        assets.append(f'<mesh name="{prefix}body" file="{model_path}"/>')
        vis_geoms.append(
            f'<geom name="{prefix}vis" type="mesh" mesh="{prefix}body"'
            ' contype="0" conaffinity="0"/>'
        )
    elif ext in {".glb", ".gltf"}:
        for geom_key, obj_path, rgba in _convert_glb(model_path):
            safe = geom_key.replace(".", "_")
            mesh_id = f"{prefix}{safe}"
            assets.append(f'<mesh name="{mesh_id}" file="{obj_path}"/>')
            vis_geoms.append(
                f'<geom name="{prefix}vis_{safe}" type="mesh" mesh="{mesh_id}"'
                f' rgba="{rgba}" contype="0" conaffinity="0"/>'
            )
    else:
        # Unknown format — fall back to a gray box the size of the model's bbox
        vis_geoms.append(
            f'<geom name="{prefix}vis" type="box" size="0.12 0.10 0.04"'
            ' rgba="0.6 0.6 0.6 1" contype="0" conaffinity="0"/>'
        )

    vis_block = "\n      ".join(vis_geoms)

    body_xml = f"""\
    <body name="{prefix}base_link" mocap="true"
          pos="{px} {py} {pz}" quat="{qw} {qx} {qy} {qz}">
      {vis_block}
      <geom name="{prefix}col" type="cylinder" size="0.08 0.03" pos="0 0 0"
            group="3" friction="0.3 0.3 0.001"/>
    </body>"""

    return assets, body_xml, ""


def _robot_xml(
    prefix: str,
    pos: tuple[float, float, float],
    quat_wxyz: tuple[float, float, float, float],
    model_path: Path,
    cfg: RobotConfig,
) -> tuple[list[str], str, str]:
    ext = model_path.suffix.lower()
    if ext == ".urdf":
        robot_dir = model_path.parent.name.lower()
        if "mr_stabs" in robot_dir:
            return _mr_stabs_xml(
                prefix, pos, quat_wxyz, model_path.parent / "meshes", cfg
            )
        elif "house_bot" in robot_dir:
            return _house_bot_xml(
                prefix, pos, quat_wxyz, model_path.parent / "meshes", cfg
            )
        else:
            raise ValueError(f"Unknown URDF robot type in directory: {robot_dir!r}")
    else:
        return _mocap_xml(prefix, pos, quat_wxyz, model_path)


# ---------------------------------------------------------------------------
# Arena XML assembly
# ---------------------------------------------------------------------------


def _build_arena_xml(cfg: SimConfig, config_dir: Path) -> str:
    all_assets: list[str] = []
    all_bodies: list[str] = []
    all_actuators: list[str] = []

    # Our robot — prefix r0_
    our_cfg = cfg.our_robot
    our_pos = _pad3(our_cfg.start_pos)
    our_quat = _yaw_quat(our_cfg.model_euler, math.radians(our_cfg.start_rotation))
    our_model = _resolve(config_dir, our_cfg.model_path)
    assets, body, acts = _robot_xml("r0_", our_pos, our_quat, our_model, our_cfg)
    all_assets.extend(assets)
    all_bodies.append(body)
    if acts:
        all_actuators.append(acts)

    # Opponents
    for idx, opp_cfg in enumerate(cfg.opponents):
        prefix = f"r{idx + 1}_"
        opp_pos = _pad3(opp_cfg.start_pos)
        opp_quat = _yaw_quat(opp_cfg.model_euler, math.radians(opp_cfg.start_rotation))
        opp_model = _resolve(config_dir, opp_cfg.model_path)
        assets, body, acts = _robot_xml(prefix, opp_pos, opp_quat, opp_model, opp_cfg)
        all_assets.extend(assets)
        all_bodies.append(body)
        if acts:
            all_actuators.append(acts)

    cam = cfg.camera
    xyaxes = _camera_xyaxes(cam.pos, cam.lookat)
    cam_pos_str = " ".join(str(v) for v in cam.pos)

    arena = cfg.arena
    ff = arena.floor_friction
    half_w = arena.width / 2.0
    half_h = arena.height / 2.0
    wall_t = 0.01  # wall half-thickness (1cm)
    wall_h = 0.15  # wall half-height

    asset_block = "\n    ".join(all_assets)
    body_block = "\n\n  ".join(all_bodies)
    actuator_block = "\n    ".join(all_actuators)

    return f"""\
<mujoco model="auto_battlebot_arena">
  <compiler angle="degree"/>
  <option gravity="0 0 -9.81">
    <flag frictionloss="enable"/>
  </option>
  <visual>
    <map znear="0.01" zfar="{cam.far}"/>
    <global offwidth="{cam.res_width}" offheight="{cam.res_height}"/>
    <headlight ambient="0.4 0.4 0.4" diffuse="0.8 0.8 0.7" specular="0.1 0.1 0.1"/>
  </visual>
  <asset>
    {asset_block}
  </asset>
  <worldbody>
    <light name="top" pos="0 0 3.5" diffuse="1 1 1" specular="0.1 0.1 0.1"
           castshadow="false"/>
    <geom name="floor" type="plane" size="0 0 0.01" friction="{ff} {ff} 0.001"
          rgba="0 0 0 0"/>
    <geom name="floor_vis" type="box" size="{half_w} {half_h} 0.001"
          pos="0 0 -0.001" rgba="0.35 0.30 0.25 1" contype="0" conaffinity="0"/>
    <geom name="wall_n" type="box"
          pos="0 {half_h + wall_t} {wall_h}" size="{half_w} {wall_t} {wall_h}"
          rgba="0.6 0.6 0.6 1"/>
    <geom name="wall_s" type="box"
          pos="0 {-(half_h + wall_t)} {wall_h}" size="{half_w} {wall_t} {wall_h}"
          rgba="0.6 0.6 0.6 1"/>
    <geom name="wall_e" type="box"
          pos="{half_w + wall_t} 0 {wall_h}" size="{wall_t} {half_h} {wall_h}"
          rgba="0.6 0.6 0.6 1"/>
    <geom name="wall_w" type="box"
          pos="{-(half_w + wall_t)} 0 {wall_h}" size="{wall_t} {half_h} {wall_h}"
          rgba="0.6 0.6 0.6 1"/>
    <camera name="overhead_cam" pos="{cam_pos_str}" xyaxes="{xyaxes}" fovy="{cam.fov}"/>
    {body_block}
  </worldbody>
  <actuator>
    {actuator_block}
  </actuator>
</mujoco>"""


# ---------------------------------------------------------------------------
# SceneHandles construction helpers
# ---------------------------------------------------------------------------


def _robot_handle(
    model: mujoco.MjModel,
    prefix: str,
    pos: tuple[float, float, float],
    quat_wxyz: tuple[float, float, float, float],
    cfg: RobotConfig,
) -> RobotHandle:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{prefix}base_link")
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{prefix}root")
    qposadr = int(model.jnt_qposadr[joint_id])
    qveladr = int(model.jnt_dofadr[joint_id])
    wheel_drive = WheelDriveInfo.from_model(model, cfg, prefix)
    return RobotHandle(
        body_id=body_id,
        freejoint_qposadr=qposadr,
        freejoint_qveladr=qveladr,
        start_pos=pos,
        start_quat_wxyz=quat_wxyz,
        wheel_drive=wheel_drive,
    )


def _mocap_handle(
    model: mujoco.MjModel,
    prefix: str,
    pos: tuple[float, float, float],
    quat_wxyz: tuple[float, float, float, float],
) -> MocapHandle:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{prefix}base_link")
    mocap_id = int(model.body_mocapid[body_id])
    return MocapHandle(
        mocap_id=mocap_id,
        body_id=body_id,
        start_pos=pos,
        start_quat_wxyz=quat_wxyz,
    )


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------


def build_scene(cfg: SimConfig, config_dir: Path) -> SceneHandles:
    arena_xml = _build_arena_xml(cfg, config_dir)

    model = mujoco.MjModel.from_xml_string(arena_xml)
    model.opt.timestep = cfg.server.physics_dt / cfg.server.substeps

    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, cfg.camera.res_height, cfg.camera.res_width)

    # Our robot handle
    our_cfg = cfg.our_robot
    our_pos = _pad3(our_cfg.start_pos)
    our_quat = _yaw_quat(our_cfg.model_euler, math.radians(our_cfg.start_rotation))
    our_robot = _robot_handle(model, "r0_", our_pos, our_quat, our_cfg)

    # Opponent handles
    opponents: list[RobotHandle | MocapHandle] = []
    for idx, opp_cfg in enumerate(cfg.opponents):
        prefix = f"r{idx + 1}_"
        opp_pos = _pad3(opp_cfg.start_pos)
        opp_quat = _yaw_quat(opp_cfg.model_euler, math.radians(opp_cfg.start_rotation))
        opp_model_path = _resolve(config_dir, opp_cfg.model_path)
        if opp_model_path.suffix.lower() == ".urdf":
            opponents.append(_robot_handle(model, prefix, opp_pos, opp_quat, opp_cfg))
        else:
            opponents.append(_mocap_handle(model, prefix, opp_pos, opp_quat))

    # Panorama background (pre-projected)
    panorama_bg: npt.NDArray[np.uint8] | None = None
    if cfg.arena.panorama is not None:
        pano_path = (
            Path(__file__).resolve().parent
            / "assets"
            / "panoramas"
            / cfg.arena.panorama
        )
        pano_img: npt.NDArray[np.uint8] | None = cv2.imread(str(pano_path))
        if pano_img is None:
            print(f"Warning: could not load panorama '{pano_path}', skipping")
        else:
            cam = cfg.camera
            panorama_bg = project_panorama(
                pano_img, cam.pos, cam.lookat, cam.fov, cam.res_width, cam.res_height
            )

    return SceneHandles(
        model=model,
        data=data,
        renderer=renderer,
        our_robot=our_robot,
        opponents=opponents,
        panorama_bg=panorama_bg,
    )
