"""SimRunner: drives the simulation loop — physics, rendering, and client I/O."""

from __future__ import annotations

import math
import socket
import struct
import time

import cv2
import mujoco
import numpy as np
import numpy.typing as npt

from behaviors import OpponentBehavior, make_opponent_behavior
from camera_utils import camera_view_matrix, fov_to_intrinsics
from config import SimConfig
from protocol import (
    GT_COUNT_FMT,
    GT_POSE_FMT,
    REQUEST_FMT,
    REQUEST_SIZE,
    RESPONSE_HEADER_FMT,
    configure_socket,
    recv_all,
    send_all,
)
from scene import _pad3, _yaw_quat
from sim_types import FrameTimings, SceneHandles, WheelDriveInfo
from sim_types.scene_handles import MocapHandle, RobotHandle


class SimRunner:
    """Drives the simulation loop: physics, rendering, and client I/O."""

    def __init__(self, cfg: SimConfig, handles: SceneHandles) -> None:
        self._cfg = cfg
        self._handles = handles
        self._model: mujoco.MjModel = handles.model
        self._data: mujoco.MjData = handles.data
        self._renderer: mujoco.Renderer = handles.renderer

        cam = cfg.camera
        self._camera_name = "overhead_cam"
        self._phys_dt: float = cfg.server.physics_dt
        self._substeps: int = cfg.server.substeps
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
            (cam.res_height, cam.res_width, 3), dtype=np.uint8
        )
        self._depth_buf: npt.NDArray[np.float32] = np.empty(
            (cam.res_height, cam.res_width), dtype=np.float32
        )
        self._bg_mask: npt.NDArray[np.bool_] = np.empty(
            (cam.res_height, cam.res_width), dtype=np.bool_
        )
        self._far_thresh: float = cam.far - 0.1
        self._panorama_bg: npt.NDArray[np.uint8] | None = handles.panorama_bg

        half_w = cfg.arena.width / 2
        half_h = cfg.arena.height / 2
        self._opp_behaviors: list[OpponentBehavior] = [
            make_opponent_behavior(opp_cfg, half_w, half_h, handle)
            for opp_cfg, handle in zip(cfg.opponents, handles.opponents)
        ]

        self._sim_debt: float = 0.0
        self._prev_time: float = 0.0
        self._timings = FrameTimings()
        self._shutting_down: bool = False
        self._srv_socket: socket.socket | None = None

    # -- lifecycle ----------------------------------------------------------

    def reset_robots(self) -> None:
        """Reset all robots to their start positions."""
        mujoco.mj_resetData(self._model, self._data)

        # Settle physics before starting
        for _ in range(self._cfg.server.settle_steps):
            for _ in range(self._substeps):
                mujoco.mj_step(self._model, self._data)

        self._sim_debt = 0.0
        self._prev_time = time.monotonic()
        self._timings = FrameTimings()

    # -- per-frame helpers --------------------------------------------------

    def _apply_command(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        our = self._cfg.our_robot
        w = self._handles.our_robot.wheel_drive
        v_target = linear_x * our.max_linear_speed
        omega_target = angular_z * our.max_angular_speed
        v_left = (v_target - omega_target * w.half_track) / w.wheel_radius
        v_right = (v_target + omega_target * w.half_track) / w.wheel_radius
        for aid in w.left_act_ids:
            self._data.ctrl[aid] = v_left
        for aid in w.right_act_ids:
            self._data.ctrl[aid] = v_right

    def _step_opponents(self, dt: float) -> None:
        for behavior in self._opp_behaviors:
            behavior.step(self._data, dt)

    def _step_physics(self, wall_dt: float) -> int:
        self._sim_debt += wall_dt
        steps = 0
        while self._sim_debt >= self._phys_dt and steps < self._max_steps:
            for _ in range(self._substeps):
                mujoco.mj_step(self._model, self._data)
            self._sim_debt -= self._phys_dt
            steps += 1
        if self._sim_debt > self._phys_dt:
            self._sim_debt = self._phys_dt
        return steps

    def _render_and_process(self) -> None:
        renderer = self._renderer

        # RGB
        renderer.update_scene(self._data, camera=self._camera_name)
        rgb_raw = renderer.render()  # (H, W, 3) uint8 RGB
        cv2.cvtColor(rgb_raw, cv2.COLOR_RGB2BGR, dst=self._rgb_buf)

        # Depth
        renderer.enable_depth_rendering()
        renderer.update_scene(self._data, camera=self._camera_name)
        depth_raw = renderer.render()  # (H, W) float32, meters
        renderer.disable_depth_rendering()

        np.copyto(self._depth_buf, depth_raw, casting="unsafe")
        np.greater_equal(self._depth_buf, self._far_thresh, out=self._bg_mask)
        np.logical_or(self._bg_mask, ~np.isfinite(self._depth_buf), out=self._bg_mask)
        self._depth_buf[self._bg_mask] = np.nan

        if self._panorama_bg is not None:
            np.copyto(
                self._rgb_buf,
                self._panorama_bg,
                where=self._bg_mask[:, :, np.newaxis],
            )

    def _get_entity_yaw(self, body_id: int) -> float:
        """Extract yaw from MuJoCo's 3x3 rotation matrix stored row-major."""
        mat = self._data.xmat[body_id].reshape(3, 3)
        return math.atan2(float(mat[1, 0]), float(mat[0, 0]))

    def _build_ground_truth_bytes(self) -> bytes:
        handles = [self._handles.our_robot] + list(self._handles.opponents)
        buf = struct.pack(GT_COUNT_FMT, len(handles))
        for handle in handles:
            pos = self._data.xpos[handle.body_id]
            yaw = self._get_entity_yaw(handle.body_id)
            buf += struct.pack(GT_POSE_FMT, float(pos[0]), float(pos[1]), yaw)
        return buf

    def _send_frame(self, conn: socket.socket) -> None:
        gt_payload = self._build_ground_truth_bytes()
        send_all(conn, self._header_bytes)
        send_all(conn, self._rgb_buf.data)
        send_all(conn, self._depth_buf.data)
        send_all(conn, gt_payload)

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

                self._apply_command(linear_x, linear_y, angular_z)
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
                self._prev_time = t5

                if self._timings.count % 10 == 0:
                    print(self._timings.report(steps, t5 - t0))

        except (ConnectionError, BrokenPipeError, OSError) as e:
            print(f"Client disconnected: {e}")

    def shutdown(self) -> None:
        """Signal the server to stop. Safe to call from a signal handler."""
        self._shutting_down = True
        srv = self._srv_socket
        if srv is not None:
            try:
                srv.close()
            except OSError:
                pass

    def serve_forever(self) -> None:
        """Accept clients in a loop, handling one at a time."""
        srv_cfg = self._cfg.server
        print(f"MuJoCo sim ready. Listening on {srv_cfg.host}:{srv_cfg.port}")

        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((srv_cfg.host, srv_cfg.port))
        srv.listen(1)
        self._srv_socket: socket.socket | None = srv

        try:
            while not self._shutting_down:
                print("Waiting for C++ client...")
                try:
                    conn, addr = srv.accept()
                except OSError:
                    # Socket closed by shutdown() or a real error — either way, stop.
                    break
                configure_socket(conn)
                print(f"Client connected from {addr}")
                try:
                    self.handle_client(conn)
                finally:
                    conn.close()
        finally:
            self._srv_socket = None
            try:
                srv.close()
            except OSError:
                pass
            self._renderer.close()
            print("Simulation server stopped.")
