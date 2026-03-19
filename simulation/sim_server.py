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

import cv2
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
from protocol import (
    REQUEST_FMT,
    REQUEST_SIZE,
    RESPONSE_HEADER_FMT,
    configure_socket,
    recv_all,
    send_all,
)

DOF_VX: int = 0
DOF_VY: int = 1
DOF_WZ: int = 5


@dataclass
class SceneHandles:
    """All Genesis objects and derived config needed after scene construction."""

    scene: gs.Scene
    camera: Camera
    our_robot: RigidEntity
    opponents: list[RigidEntity]
    panorama_bg: npt.NDArray[np.uint8] | None


@dataclass
class FrameTimings:
    """Cumulative timings for profiling the serve loop."""

    recv: float = 0.0
    physics: float = 0.0
    render: float = 0.0
    process: float = 0.0
    send: float = 0.0
    count: int = 0

    def record(
        self,
        t_recv: float,
        t_physics: float,
        t_render: float,
        t_process: float,
        t_send: float,
    ) -> None:
        self.recv += t_recv
        self.physics += t_physics
        self.render += t_render
        self.process += t_process
        self.send += t_send
        self.count += 1

    def report(self, physics_steps: int, frame_total: float) -> str:
        n = self.count
        return (
            f"[frame {n}] avg ms/frame: "
            f"recv={1000 * self.recv / n:.1f}  "
            f"physics={1000 * self.physics / n:.1f} ({physics_steps} steps)  "
            f"render={1000 * self.render / n:.1f}  "
            f"process={1000 * self.process / n:.1f}  "
            f"send={1000 * self.send / n:.1f}  "
            f"total={1000 * frame_total:.1f}"
        )


# ---------------------------------------------------------------------------
# Scene construction
# ---------------------------------------------------------------------------


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

    from genesis.options.vis import DirectionalLight, PointLight, AmbientLight

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
        sim_options=gs.options.SimOptions(dt=cfg.server.physics_dt),
        vis_options=gs.options.VisOptions(
            lights=vis_lights if vis_lights else None,
        ),
        viewer_options=gs.options.ViewerOptions(
            res=(1280, 960),
            camera_pos=tuple(cam.pos),
            camera_lookat=tuple(cam.lookat),
            camera_fov=cam.fov,
            max_FPS=60,
        ),
    )

    scene.add_entity(gs.morphs.Plane(visualization=False))

    floor_mesh = resolve_path(config_dir, arena.floor_mesh)
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


# ---------------------------------------------------------------------------
# SimRunner
# ---------------------------------------------------------------------------


class SimRunner:
    """Drives the simulation loop: physics, rendering, and client I/O."""

    def __init__(self, cfg: SimConfig, handles: SceneHandles) -> None:
        self._cfg = cfg
        self._handles = handles

        cam = cfg.camera
        self._scene: gs.Scene = handles.scene
        self._camera: Camera = handles.camera

        self._our_cfg = cfg.our_robot
        self._phys_dt: float = cfg.server.physics_dt
        self._max_steps: int = cfg.server.max_physics_steps_per_frame

        fx, fy, cx, cy = fov_to_intrinsics(cam.fov, cam.res_width, cam.res_height)
        tf_matrix = camera_view_matrix(cam.pos, cam.lookat)

        self._header_bytes: bytes = struct.pack(
            RESPONSE_HEADER_FMT,
            cam.res_width,
            cam.res_height,
            *tf_matrix.flatten().tolist(),
            fx,
            fy,
            cx,
            cy,
        )

        self._rgb_buf: npt.NDArray[np.uint8] = np.empty(
            (cam.res_height, cam.res_width, 3),
            dtype=np.uint8,
        )
        self._depth_buf: npt.NDArray[np.float32] = np.empty(
            (cam.res_height, cam.res_width),
            dtype=np.float32,
        )
        self._bg_mask: npt.NDArray[np.bool_] = np.empty(
            (cam.res_height, cam.res_width),
            dtype=np.bool_,
        )
        self._far_thresh: float = cam.far - 0.1
        self._panorama_bg: npt.NDArray[np.uint8] | None = handles.panorama_bg

        half_w, half_h = cfg.arena.width / 2, cfg.arena.height / 2
        self._opp_behaviors: list[OpponentBehavior] = [
            make_opponent_behavior(opp_cfg, half_w, half_h) for opp_cfg in cfg.opponents
        ]

        self._our_yaw: float = 0.0
        self._sim_debt: float = 0.0
        self._prev_time: float = 0.0
        self._timings = FrameTimings()

    # -- lifecycle ----------------------------------------------------------

    def reset_robots(self) -> None:
        """Move all robots to their configured start positions."""
        our = self._our_cfg
        handles = self._handles

        handles.our_robot.set_pos(list(_pad_pos_3d(our.start_pos)))
        self._our_yaw = math.radians(our.start_rotation)
        handles.our_robot.set_quat(_robot_quat(our.model_euler, self._our_yaw))
        handles.our_robot.set_dofs_velocity(np.zeros(handles.our_robot.n_dofs))

        for opp_cfg, opp_entity in zip(self._cfg.opponents, handles.opponents):
            opp_entity.set_pos(list(_pad_pos_3d(opp_cfg.start_pos)))
            opp_entity.set_quat(
                _robot_quat(opp_cfg.model_euler, math.radians(opp_cfg.start_rotation))
            )
            opp_entity.set_dofs_velocity(np.zeros(opp_entity.n_dofs))

        self._sim_debt = 0.0
        self._prev_time = time.monotonic()
        self._timings = FrameTimings()

    # -- per-frame helpers --------------------------------------------------

    def _apply_command(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float,
        dt: float,
    ) -> None:
        our = self._our_cfg
        self._our_yaw += angular_z * our.max_angular_speed * dt
        speed = linear_x * our.max_linear_speed
        vel_x = speed * math.cos(self._our_yaw)
        vel_y = speed * math.sin(self._our_yaw)

        self._handles.our_robot.set_dofs_velocity(
            np.array([vel_x, vel_y, angular_z * our.max_angular_speed]),
            [DOF_VX, DOF_VY, DOF_WZ],
        )

    def _step_opponents(self, dt: float) -> None:
        for behavior, opp_entity in zip(
            self._opp_behaviors,
            self._handles.opponents,
        ):
            behavior.step(opp_entity, dt)

    def _step_physics(self, wall_dt: float) -> int:
        self._sim_debt += wall_dt
        steps = 0
        while self._sim_debt >= self._phys_dt and steps < self._max_steps:
            self._scene.step()
            self._sim_debt -= self._phys_dt
            steps += 1
        if self._sim_debt > self._phys_dt:
            self._sim_debt = self._phys_dt
        return steps

    def _render_and_process(self) -> None:
        rgb_raw, depth_raw, _, _ = self._camera.render(depth=True)

        cv2.cvtColor(np.squeeze(rgb_raw), cv2.COLOR_RGB2BGR, dst=self._rgb_buf)

        np.copyto(self._depth_buf, np.squeeze(depth_raw), casting="unsafe")
        np.greater_equal(self._depth_buf, self._far_thresh, out=self._bg_mask)
        np.logical_or(
            self._bg_mask,
            ~np.isfinite(self._depth_buf),
            out=self._bg_mask,
        )
        self._depth_buf[self._bg_mask] = np.nan

        if self._panorama_bg is not None:
            np.copyto(
                self._rgb_buf,
                self._panorama_bg,
                where=self._bg_mask[:, :, np.newaxis],
            )

    def _send_frame(self, conn: socket.socket) -> None:
        send_all(conn, self._header_bytes)
        send_all(conn, self._rgb_buf.data)
        send_all(conn, self._depth_buf.data)

    # -- main loop ----------------------------------------------------------

    def handle_client(self, conn: socket.socket) -> None:
        """Service one client connection until it disconnects."""
        self.reset_robots()

        try:
            while True:
                t0 = time.monotonic()
                data = recv_all(conn, REQUEST_SIZE)
                linear_x, linear_y, angular_z = struct.unpack(REQUEST_FMT, data)
                t1 = time.monotonic()

                wall_dt = min(t1 - self._prev_time, 0.1)
                self._prev_time = t1

                self._apply_command(linear_x, linear_y, angular_z, wall_dt)
                self._step_opponents(wall_dt)
                t2 = time.monotonic()

                steps = self._step_physics(wall_dt)
                t3 = time.monotonic()

                self._render_and_process()
                t4 = time.monotonic()

                self._send_frame(conn)
                t5 = time.monotonic()

                self._timings.record(
                    t_recv=t1 - t0,
                    t_physics=t3 - t2,
                    t_render=t4 - t3,
                    t_process=0.0,
                    t_send=t5 - t4,
                )

                if self._timings.count % 100 == 0:
                    print(self._timings.report(steps, t5 - t0))

        except (ConnectionError, BrokenPipeError, OSError) as e:
            print(f"Client disconnected: {e}")

    def serve_forever(self) -> None:
        """Accept clients in a loop, handling one at a time."""
        srv_cfg = self._cfg.server
        print(f"Genesis sim ready. Listening on {srv_cfg.host}:{srv_cfg.port}")

        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((srv_cfg.host, srv_cfg.port))
        srv.listen(1)

        while True:
            print("Waiting for C++ client...")
            conn, addr = srv.accept()
            configure_socket(conn)
            print(f"Client connected from {addr}")
            self.handle_client(conn)
            conn.close()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", help="Path to sim_config.toml")
    args: argparse.Namespace = parser.parse_args()

    config_dir: Path = Path(args.config).resolve().parent
    cfg: SimConfig = load_sim_config(args.config)

    handles: SceneHandles = build_scene(cfg, config_dir)
    runner = SimRunner(cfg, handles)
    runner.serve_forever()


if __name__ == "__main__":
    main()
