"""Genesis simulation server for auto-battlebot.

Loads a scene from a TOML config, runs physics + rendering in Genesis,
and communicates with the C++ pipeline over a TCP socket using a simple
binary protocol.

Usage:
    python sim_server.py sim_config.toml
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import time
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
import numpy.typing as npt
from scipy.spatial.transform import Rotation as R

import genesis as gs

if TYPE_CHECKING:
    from genesis.engine.entities.rigid_entity import RigidEntity
    from genesis.vis.camera import Camera

from behaviors import OpponentBehavior, make_opponent_behavior
from camera_utils import camera_view_matrix, fov_to_intrinsics, project_panorama
from config import SimConfig, load_sim_config
from config.robot import RobotConfig
from protocol import REQUEST_FMT, REQUEST_SIZE, RESPONSE_HEADER_FMT, recv_all, send_all


@dataclass
class SceneHandles:
    """All Genesis objects and derived config needed after scene construction."""

    scene: gs.Scene
    camera: Camera
    our_robot: RigidEntity
    opponents: list[RigidEntity]
    panorama_bg: npt.NDArray[np.uint8] | None


def resolve_path(base_dir: Path, p: str) -> str:
    pp = Path(p)
    if pp.is_absolute():
        return str(pp)
    return str((base_dir / pp).resolve())


def _pad_pos_3d(pos: list[float]) -> tuple[float, float, float]:
    if len(pos) == 2:
        return (pos[0], pos[1], 0.0)
    return (pos[0], pos[1], pos[2])


def _robot_quat(model_euler_deg: list[float], yaw_rad: float) -> list[float]:
    """Compose a robot's model correction with a world-space yaw into one quaternion.

    The model_euler fixes the mesh orientation (e.g. upright correction).
    The yaw is applied on top in world space (rotation around Z).
    Returns (w, x, y, z) for Genesis's set_quat().
    """
    base = R.from_euler("XYZ", model_euler_deg, degrees=True)
    yaw = R.from_euler("Z", yaw_rad)
    combined = yaw * base
    x, y, z, w = combined.as_quat()  # scipy returns (x, y, z, w)
    return [w, x, y, z]  # Genesis expects (w, x, y, z)


def _add_robot(
    scene: gs.Scene, robot_cfg: RobotConfig, config_dir: Path
) -> RigidEntity:
    model_path = resolve_path(config_dir, robot_cfg.model_path)
    return scene.add_entity(
        gs.morphs.Mesh(
            file=model_path,
            scale=robot_cfg.scale,
            pos=_pad_pos_3d(robot_cfg.start_pos),
            euler=tuple(robot_cfg.model_euler),
            fixed=False,
        ),
    )


def build_scene(cfg: SimConfig, config_dir: Path) -> SceneHandles:
    """Initialise Genesis and build the scene from config."""
    arena = cfg.arena
    cam = cfg.camera

    gs.init(backend=gs.gpu)
    scene = gs.Scene(
        show_viewer=cfg.server.show_viewer,
        sim_options=gs.options.SimOptions(dt=cfg.server.physics_dt),
        viewer_options=gs.options.ViewerOptions(
            res=(1280, 960),
            camera_pos=tuple(cam.pos),
            camera_lookat=tuple(cam.lookat),
            camera_fov=cam.fov,
            max_FPS=60,
        ),
    )

    scene.add_entity(gs.morphs.Plane(visualization=False))

    floor_mesh = str(Path(__file__).resolve().parent / "assets" / "cage" / "floor.obj")
    scene.add_entity(
        gs.morphs.Mesh(
            file=floor_mesh,
            pos=(0, 0, 0),
            scale=(arena.width, arena.height, 1.0),
            fixed=True,
        )
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
        import cv2

        pano_path = str(
            Path(__file__).resolve().parent / "assets" / "panoramas" / arena.panorama
        )
        pano_img: npt.NDArray[np.uint8] = cv2.imread(pano_path)
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


def serve(cfg: SimConfig, handles: SceneHandles) -> None:
    """Run the TCP accept-loop, stepping the sim for each client request."""
    srv_cfg = cfg.server
    cam = cfg.camera
    arena = cfg.arena
    our = cfg.our_robot

    scene: gs.Scene = handles.scene
    camera: Camera = handles.camera

    fx, fy, cx, cy = fov_to_intrinsics(cam.fov, cam.res_width, cam.res_height)
    tf_matrix: npt.NDArray[np.float64] = camera_view_matrix(cam.pos, cam.lookat)

    half_w, half_h = arena.width / 2, arena.height / 2
    opp_behaviors: list[OpponentBehavior] = [
        make_opponent_behavior(opp_cfg, half_w, half_h) for opp_cfg in cfg.opponents
    ]

    print(f"Genesis sim ready. Listening on {srv_cfg.host}:{srv_cfg.port}")

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((srv_cfg.host, srv_cfg.port))
    srv.listen(1)

    # Free-body DOF indices: [vx, vy, vz, wx, wy, wz]
    DOF_VX, DOF_VY = 0, 1
    DOF_WZ = 5

    while True:
        print("Waiting for C++ client...")
        conn, addr = srv.accept()
        print(f"Client connected from {addr}")

        handles.our_robot.set_pos(list(_pad_pos_3d(our.start_pos)))
        our_yaw: float = math.radians(our.start_rotation)
        handles.our_robot.set_quat(_robot_quat(our.model_euler, our_yaw))
        n_dofs: int = handles.our_robot.n_dofs
        handles.our_robot.set_dofs_velocity(np.zeros(n_dofs))

        for opp_cfg, opp_entity in zip(cfg.opponents, handles.opponents):
            opp_entity.set_pos(list(_pad_pos_3d(opp_cfg.start_pos)))
            opp_entity.set_quat(
                _robot_quat(opp_cfg.model_euler, math.radians(opp_cfg.start_rotation))
            )
            opp_entity.set_dofs_velocity(np.zeros(opp_entity.n_dofs))

        prev_time: float = time.monotonic()
        sim_debt: float = 0.0
        phys_dt: float = srv_cfg.physics_dt

        try:
            while True:
                data: bytes = recv_all(conn, REQUEST_SIZE)
                linear_x, linear_y, angular_z = struct.unpack(REQUEST_FMT, data)

                now: float = time.monotonic()
                wall_dt: float = min(now - prev_time, 0.1)
                prev_time = now

                our_yaw += angular_z * our.max_angular_speed * wall_dt
                speed: float = linear_x * our.max_linear_speed
                vel_x: float = speed * math.cos(our_yaw)
                vel_y: float = speed * math.sin(our_yaw)

                handles.our_robot.set_dofs_velocity(
                    np.array([vel_x, vel_y, angular_z * our.max_angular_speed]),
                    [DOF_VX, DOF_VY, DOF_WZ],
                )

                for behavior, opp_entity in zip(opp_behaviors, handles.opponents):
                    behavior.step(opp_entity, wall_dt)

                sim_debt += wall_dt
                while sim_debt >= phys_dt:
                    scene.step()
                    sim_debt -= phys_dt
                rgb_raw, depth_raw, _, _ = camera.render(depth=True)

                rgb: npt.NDArray[np.uint8] = np.ascontiguousarray(
                    np.squeeze(rgb_raw)[:, :, ::-1]  # RGB -> BGR for OpenCV
                )
                depth: npt.NDArray[np.float32] = np.ascontiguousarray(
                    np.squeeze(depth_raw).astype(np.float32)
                )
                bg_mask = ~np.isfinite(depth) | (depth >= cam.far - 0.1)
                depth[bg_mask] = np.nan

                if handles.panorama_bg is not None:
                    rgb[bg_mask] = handles.panorama_bg[bg_mask]

                h, w = rgb.shape[:2]

                tf_flat: list[float] = tf_matrix.flatten().tolist()
                header: bytes = struct.pack(
                    RESPONSE_HEADER_FMT, w, h, *tf_flat, fx, fy, cx, cy
                )
                send_all(conn, header)
                send_all(conn, rgb.tobytes())
                send_all(conn, depth.tobytes())

        except (ConnectionError, BrokenPipeError, OSError) as e:
            print(f"Client disconnected: {e}")
            conn.close()
            continue


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", help="Path to sim_config.toml")
    args: argparse.Namespace = parser.parse_args()

    config_dir: Path = Path(args.config).resolve().parent
    cfg: SimConfig = load_sim_config(args.config)

    handles: SceneHandles = build_scene(cfg, config_dir)
    serve(cfg, handles)


if __name__ == "__main__":
    main()
