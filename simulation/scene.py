"""Genesis scene construction: arena, robots, camera, lighting."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import cv2
import numpy as np
import numpy.typing as npt
from scipy.spatial.transform import Rotation as R

import genesis as gs

if TYPE_CHECKING:
    from genesis.engine.entities.rigid_entity import RigidEntity
    from genesis.vis.camera import Camera

from camera_utils import camera_view_matrix, fov_to_intrinsics, project_panorama
from config import SimConfig
from config.robot import RobotConfig
from sim_types import SceneHandles


def resolve_path(base_dir: Path, p: str) -> str:
    pp = Path(p)
    if pp.is_absolute():
        return str(pp)
    return str((base_dir / pp).resolve())


def pad_pos_3d(pos: list[float]) -> tuple[float, float, float]:
    if len(pos) == 2:
        return (pos[0], pos[1], 0.0)
    return (pos[0], pos[1], pos[2])


def robot_quat(model_euler_deg: list[float], yaw_rad: float) -> list[float]:
    """Compose a robot's model correction with a world-space yaw.

    Returns (w, x, y, z) for Genesis's set_quat().
    """
    base = R.from_euler("XYZ", model_euler_deg, degrees=True)
    yaw = R.from_euler("Z", yaw_rad)
    combined = yaw * base
    x, y, z, w = combined.as_quat()
    return [w, x, y, z]


def _add_robot(
    scene: gs.Scene, robot_cfg: RobotConfig, config_dir: Path
) -> RigidEntity:
    model_path = resolve_path(config_dir, robot_cfg.model_path)
    is_urdf = model_path.lower().endswith(".urdf")

    if is_urdf:
        morph = gs.morphs.URDF(
            file=model_path,
            pos=pad_pos_3d(robot_cfg.start_pos),
            euler=tuple(robot_cfg.model_euler),
            fixed=False,
            decimate_aggressiveness=0,
        )
    else:
        morph = gs.morphs.Mesh(
            file=model_path,
            scale=robot_cfg.scale,
            pos=pad_pos_3d(robot_cfg.start_pos),
            euler=tuple(robot_cfg.model_euler),
            fixed=False,
        )

    material = gs.materials.Rigid(
        friction=robot_cfg.wheel_friction if robot_cfg.has_wheels else None,
    )
    return scene.add_entity(morph, material, visualize_contact=True)


def build_scene(cfg: SimConfig, config_dir: Path) -> SceneHandles:
    """Initialise Genesis and build the scene from config."""
    arena = cfg.arena
    cam = cfg.camera

    from genesis.options.vis import AmbientLight, DirectionalLight, PointLight

    vis_lights: list = []
    for lc in cfg.lights:
        if lc.type == "directional":
            vis_lights.append(
                DirectionalLight(
                    dir=tuple(lc.dir),
                    color=tuple(lc.color),
                    intensity=lc.intensity,
                )
            )
        elif lc.type == "point":
            vis_lights.append(
                PointLight(
                    pos=tuple(lc.pos),
                    color=tuple(lc.color),
                    intensity=lc.intensity,
                )
            )
        elif lc.type == "ambient":
            vis_lights.append(
                AmbientLight(
                    color=tuple(lc.color),
                    intensity=lc.intensity,
                )
            )

    gs.init(backend=gs.gpu)
    scene = gs.Scene(
        show_viewer=cfg.server.show_viewer,
        sim_options=gs.options.SimOptions(
            dt=cfg.server.physics_dt,
            substeps=cfg.server.substeps,
        ),
        rigid_options=gs.options.RigidOptions(
            enable_self_collision=False,
        ),
        vis_options=gs.options.VisOptions(
            lights=vis_lights if vis_lights else None,
            contact_force_scale=0.02,
        ),
        viewer_options=gs.options.ViewerOptions(
            res=(1280, 960),
            camera_pos=tuple(cam.pos),
            camera_lookat=tuple(cam.lookat),
            camera_fov=cam.fov,
            max_FPS=60,
        ),
    )

    floor_material = gs.materials.Rigid(friction=arena.floor_friction)
    scene.add_entity(gs.morphs.Plane(visualization=False), floor_material)

    floor_mesh = resolve_path(config_dir, arena.floor_mesh)
    scene.add_entity(
        gs.morphs.Mesh(
            file=floor_mesh,
            pos=(0, 0, 0),
            scale=(arena.width, arena.height, 1.0),
            fixed=True,
        ),
        floor_material,
    )

    our_robot: RigidEntity = _add_robot(scene, cfg.our_robot, config_dir)
    opponents: list[RigidEntity] = [
        _add_robot(scene, opp_cfg, config_dir) for opp_cfg in cfg.opponents
    ]

    camera: Camera = scene.add_camera(
        res=(cam.res_width, cam.res_height),
        pos=tuple(cam.pos),
        lookat=tuple(cam.lookat),
        fov=cam.fov,
        far=cam.far,
        GUI=False,
    )

    scene.build(n_envs=1)
    scene.step()

    panorama_bg: npt.NDArray[np.uint8] | None = None
    if arena.panorama is not None:
        pano_path = str(
            Path(__file__).resolve().parent / "assets" / "panoramas" / arena.panorama
        )
        pano_img: npt.NDArray[np.uint8] | None = cv2.imread(pano_path)
        if pano_img is None:
            print(f"Warning: could not load panorama '{pano_path}', skipping")
        else:
            panorama_bg = project_panorama(
                pano_img, cam.pos, cam.lookat, cam.fov, cam.res_width, cam.res_height
            )

    return SceneHandles(
        scene=scene,
        camera=camera,
        our_robot=our_robot,
        opponents=opponents,
        panorama_bg=panorama_bg,
    )
