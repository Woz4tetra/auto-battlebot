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
from typing import TYPE_CHECKING, Any

import numpy as np
import numpy.typing as npt

try:
    import tomllib
except ModuleNotFoundError:
    import tomli as tomllib  # type: ignore[no-redef]

import genesis as gs

if TYPE_CHECKING:
    from genesis.engine.entities.rigid_entity import RigidEntity
    from genesis.vis.camera import Camera

from behaviors import OpponentBehavior, make_opponent_behavior
from camera_utils import camera_view_matrix, fov_to_intrinsics, project_panorama
from protocol import REQUEST_FMT, REQUEST_SIZE, RESPONSE_HEADER_FMT, recv_all, send_all


@dataclass
class SceneHandles:
    """All Genesis objects and derived config needed after scene construction."""

    scene: gs.Scene
    camera: Camera
    our_robot: RigidEntity
    opponent: RigidEntity
    our_cfg: dict[str, Any]
    opp_cfg: dict[str, Any]
    arena_w: float
    arena_h: float
    cam_pos: list[float]
    cam_lookat: list[float]
    fov: float
    cam_far: float
    res_w: int
    res_h: int
    panorama_bg: npt.NDArray[np.uint8] | None


def load_config(path: str) -> dict[str, Any]:
    with open(path, "rb") as f:
        return tomllib.load(f)


def resolve_path(base_dir: Path, p: str) -> str:
    pp = Path(p)
    if pp.is_absolute():
        return str(pp)
    return str((base_dir / pp).resolve())


def build_scene(cfg: dict[str, Any], config_dir: Path) -> SceneHandles:
    """Initialise Genesis and build the scene from config."""
    arena_cfg: dict[str, Any] = cfg.get("arena", {})
    arena_w: float = arena_cfg.get("width", 2.4)
    arena_h: float = arena_cfg.get("height", 2.4)

    cam_cfg: dict[str, Any] = cfg.get("camera", {})
    res_w: int = cam_cfg.get("res_width", 1280)
    res_h: int = cam_cfg.get("res_height", 720)
    fov: float = cam_cfg.get("fov", 70.0)
    cam_pos: list[float] = cam_cfg.get("pos", [0.0, -1.5, 1.8])
    cam_lookat: list[float] = cam_cfg.get("lookat", [0.0, 0.0, 0.0])

    our_cfg: dict[str, Any] = cfg.get("our_robot", {})
    opp_cfg: dict[str, Any] = cfg.get("opponent", {})

    gs.init(backend=gs.gpu)
    scene = gs.Scene(
        show_viewer=True,
        viewer_options=gs.options.ViewerOptions(
            res=(1280, 960),
            camera_pos=tuple(cam_pos),
            camera_lookat=tuple(cam_lookat),
            camera_fov=fov,
            max_FPS=60,
        ),
    )

    scene.add_entity(gs.morphs.Plane(visualization=False))

    floor_mesh: str = str(
        (Path(__file__).resolve().parent / "assets" / "cage" / "floor.obj")
    )
    scene.add_entity(
        gs.morphs.Mesh(
            file=floor_mesh,
            pos=(0, 0, 0),
            scale=(arena_w, arena_h, 1.0),
            fixed=True,
        )
    )

    our_model: str = resolve_path(config_dir, our_cfg.get("model_path", "robot.gltf"))
    our_euler: list[float] = our_cfg.get("model_euler", [0.0, 0.0, 0.0])
    our_start: list[float] = our_cfg.get("start_pos", [-0.5, 0.0, 0.0])
    our_scale: float = our_cfg.get("scale", 1.0)
    our_robot: RigidEntity = scene.add_entity(
        gs.morphs.Mesh(
            file=our_model,
            scale=our_scale,
            pos=tuple(our_start) + (0.0,) if len(our_start) == 2 else tuple(our_start),
            euler=tuple(our_euler),
            fixed=False,
        )
    )

    opp_model: str = resolve_path(
        config_dir, opp_cfg.get("model_path", "opponent.gltf")
    )
    opp_euler: list[float] = opp_cfg.get("model_euler", [0.0, 0.0, 0.0])
    opp_start: list[float] = opp_cfg.get("start_pos", [0.5, 0.0, 0.0])
    opp_scale: float = opp_cfg.get("scale", 1.0)
    opponent: RigidEntity = scene.add_entity(
        gs.morphs.Mesh(
            file=opp_model,
            scale=opp_scale,
            pos=tuple(opp_start) + (0.0,) if len(opp_start) == 2 else tuple(opp_start),
            euler=tuple(opp_euler),
            fixed=False,
        )
    )

    cam_far: float = cam_cfg.get("far", 10.0)
    camera: Camera = scene.add_camera(
        res=(res_w, res_h),
        pos=tuple(cam_pos),
        lookat=tuple(cam_lookat),
        fov=fov,
        far=cam_far,
        GUI=False,
    )

    scene.build(n_envs=1)
    scene.step()

    panorama_bg: npt.NDArray[np.uint8] | None = None
    panorama_name: str | None = arena_cfg.get("panorama", None)
    if panorama_name is not None:
        import cv2

        pano_path: str = str(
            Path(__file__).resolve().parent / "assets" / "panoramas" / panorama_name
        )
        pano_img: npt.NDArray[np.uint8] = cv2.imread(pano_path)
        if pano_img is None:
            print(f"Warning: could not load panorama '{pano_path}', skipping")
        else:
            panorama_bg = project_panorama(
                pano_img, cam_pos, cam_lookat, fov, res_w, res_h
            )

    return SceneHandles(
        scene=scene,
        camera=camera,
        our_robot=our_robot,
        opponent=opponent,
        our_cfg=our_cfg,
        opp_cfg=opp_cfg,
        arena_w=arena_w,
        arena_h=arena_h,
        cam_pos=cam_pos,
        cam_lookat=cam_lookat,
        fov=fov,
        cam_far=cam_far,
        res_w=res_w,
        res_h=res_h,
        panorama_bg=panorama_bg,
    )


def serve(cfg: dict[str, Any], handles: SceneHandles) -> None:
    """Run the TCP accept-loop, stepping the sim for each client request."""
    server_cfg: dict[str, Any] = cfg.get("server", {})
    host: str = server_cfg.get("host", "127.0.0.1")
    port: int = server_cfg.get("port", 14882)

    scene: gs.Scene = handles.scene
    camera: Camera = handles.camera
    our_robot: RigidEntity = handles.our_robot
    opponent: RigidEntity = handles.opponent
    our_cfg: dict[str, Any] = handles.our_cfg
    opp_cfg: dict[str, Any] = handles.opp_cfg
    arena_w: float = handles.arena_w
    arena_h: float = handles.arena_h

    our_start: list[float] = our_cfg.get("start_pos", [-0.5, 0.0, 0.0])
    opp_start: list[float] = opp_cfg.get("start_pos", [0.5, 0.0, 0.0])
    our_max_linear: float = our_cfg.get("max_linear_speed", 2.0)
    our_max_angular: float = our_cfg.get("max_angular_speed", 6.0)

    fx, fy, cx, cy = fov_to_intrinsics(handles.fov, handles.res_w, handles.res_h)
    tf_matrix: npt.NDArray[np.float64] = camera_view_matrix(
        handles.cam_pos, handles.cam_lookat
    )

    opp_behavior: OpponentBehavior = make_opponent_behavior(
        opp_cfg, arena_w / 2, arena_h / 2
    )

    print(f"Genesis sim ready. Listening on {host}:{port}")

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)

    while True:
        print("Waiting for C++ client...")
        conn, addr = srv.accept()
        print(f"Client connected from {addr}")

        our_start_3: list[float] = list(our_start) + [0.0] * (3 - len(our_start))
        opp_start_3: list[float] = list(opp_start) + [0.0] * (3 - len(opp_start))
        our_robot.set_pos(our_start_3)
        opponent.set_pos(opp_start_3)
        our_yaw: float = 0.0

        prev_time: float = time.monotonic()

        try:
            while True:
                data: bytes = recv_all(conn, REQUEST_SIZE)
                linear_x, linear_y, angular_z = struct.unpack(REQUEST_FMT, data)

                now: float = time.monotonic()
                dt: float = min(now - prev_time, 0.1)
                prev_time = now

                our_yaw += angular_z * our_max_angular * dt
                vx: float = linear_x * our_max_linear
                dx: float = vx * math.cos(our_yaw) * dt
                dy: float = vx * math.sin(our_yaw) * dt
                pos: npt.NDArray[np.floating[Any]] = (
                    our_robot.get_pos().cpu().numpy().squeeze()
                )
                pos[0] += dx
                pos[1] += dy
                pos[0] = max(-arena_w / 2, min(arena_w / 2, pos[0]))
                pos[1] = max(-arena_h / 2, min(arena_h / 2, pos[1]))
                our_robot.set_pos(pos)

                opp_behavior.step(opponent, dt)

                scene.step()
                rgb_raw, depth_raw, _, _ = camera.render(depth=True)

                rgb: npt.NDArray[np.uint8] = np.ascontiguousarray(
                    np.squeeze(rgb_raw)[:, :, ::-1]  # RGB -> BGR for OpenCV
                )
                depth: npt.NDArray[np.float32] = np.ascontiguousarray(
                    np.squeeze(depth_raw).astype(np.float32)
                )
                bg_mask = ~np.isfinite(depth) | (depth >= handles.cam_far - 0.1)
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

    cfg: dict[str, Any] = load_config(args.config)
    config_dir: Path = Path(args.config).resolve().parent

    handles: SceneHandles = build_scene(cfg, config_dir)
    serve(cfg, handles)


if __name__ == "__main__":
    main()
