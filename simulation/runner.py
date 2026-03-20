"""SimRunner: drives the simulation loop — physics, rendering, and client I/O."""

from __future__ import annotations

import io
import math
import socket
import struct
import time
from typing import TYPE_CHECKING

import cv2
import numpy as np
import numpy.typing as npt

import genesis as gs

if TYPE_CHECKING:
    from genesis.vis.camera import Camera

from behaviors import OpponentBehavior, make_opponent_behavior
from camera_utils import camera_view_matrix, fov_to_intrinsics
from config import SimConfig
from protocol import (
    REQUEST_FMT,
    REQUEST_SIZE,
    RESPONSE_HEADER_FMT,
    configure_socket,
    recv_all,
    send_all,
)
from scene import pad_pos_3d, robot_quat
from sim_types import (
    DOF_VX,
    DOF_VY,
    DOF_WZ,
    FrameTimings,
    SceneHandles,
    WheelDriveInfo,
)


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

        # Resolve wheel DOF indices (requires scene to be built already)
        self._our_wheels: WheelDriveInfo | None = None
        if cfg.our_robot.has_wheels:
            self._our_wheels = WheelDriveInfo.from_entity(
                handles.our_robot, cfg.our_robot
            )

        self._opp_wheels: list[WheelDriveInfo | None] = []
        for opp_cfg, opp_entity in zip(cfg.opponents, handles.opponents):
            if opp_cfg.has_wheels:
                self._opp_wheels.append(WheelDriveInfo.from_entity(opp_entity, opp_cfg))
            else:
                self._opp_wheels.append(None)

        half_w, half_h = cfg.arena.width / 2, cfg.arena.height / 2
        self._opp_behaviors: list[OpponentBehavior] = [
            make_opponent_behavior(opp_cfg, half_w, half_h, winfo)
            for opp_cfg, winfo in zip(cfg.opponents, self._opp_wheels)
        ]

        self._our_yaw: float = 0.0
        self._sim_debt: float = 0.0
        self._prev_time: float = 0.0
        self._timings = FrameTimings()
        self._last_cmd_vel: list[float] = []
        self._diag_file: io.TextIOWrapper | None = None

    # -- lifecycle ----------------------------------------------------------

    def reset_robots(self) -> None:
        """Move all robots to their configured start positions."""
        our = self._our_cfg
        handles = self._handles

        handles.our_robot.set_pos(list(pad_pos_3d(our.start_pos)))
        self._our_yaw = math.radians(our.start_rotation)
        handles.our_robot.set_quat(robot_quat(our.model_euler, self._our_yaw))
        handles.our_robot.set_dofs_velocity(np.zeros(handles.our_robot.n_dofs))

        for opp_cfg, opp_entity in zip(self._cfg.opponents, handles.opponents):
            opp_entity.set_pos(list(pad_pos_3d(opp_cfg.start_pos)))
            opp_entity.set_quat(
                robot_quat(opp_cfg.model_euler, math.radians(opp_cfg.start_rotation))
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
        robot = self._handles.our_robot
        w = self._our_wheels

        if w is not None:
            v = linear_x * our.max_linear_speed
            omega = angular_z * our.max_angular_speed
            v_left = (v - omega * w.half_track) / w.wheel_radius
            v_right = (v + omega * w.half_track) / w.wheel_radius
            velocities = [v_left] * w.n_left + [v_right] * w.n_right
            robot.control_dofs_velocity(velocities, dofs_idx_local=w.all_dofs)
            self._last_cmd_vel = velocities
        else:
            self._our_yaw += angular_z * our.max_angular_speed * dt
            speed = linear_x * our.max_linear_speed
            vel_x = speed * math.cos(self._our_yaw)
            vel_y = speed * math.sin(self._our_yaw)
            robot.set_dofs_velocity(
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

    def _log_diagnostics(self) -> None:
        if self._diag_file is None:
            return
        try:
            robot = self._handles.our_robot
            w = self._our_wheels

            pos = robot.get_pos().cpu().numpy().squeeze()
            quat = robot.get_quat().cpu().numpy().squeeze()
            all_vel = robot.get_dofs_velocity().cpu().numpy().squeeze()

            lines = [f"[frame {self._timings.count} {time.time()}]"]
            lines.append(f"  pos={pos[0]:.3f},{pos[1]:.3f},{pos[2]:.4f}  quat={quat}")

            if w is not None:
                actual = [float(all_vel[i]) for i in w.all_dofs]
                cmd = self._last_cmd_vel
                lines.append(
                    f"  wheel cmd={[f'{v:.1f}' for v in cmd]}  "
                    f"actual={[f'{v:.1f}' for v in actual]}"
                )

                try:
                    force = robot.get_dofs_force().cpu().numpy().squeeze()
                    wheel_forces = [float(force[i]) for i in w.all_dofs]
                    lines.append(f"  wheel torque={[f'{v:.4f}' for v in wheel_forces]}")
                except Exception as e:
                    lines.append(f"  wheel torque=<error: {e}>")

            body_lin = np.linalg.norm(all_vel[:3])
            body_wz = float(all_vel[5]) if len(all_vel) > 5 else 0.0
            lines.append(
                f"  body speed={body_lin:.3f} m/s  yaw_rate={body_wz:.2f} rad/s"
            )
            n_dofs = robot.n_dofs
            lines.append(
                f"  n_dofs={n_dofs}  all_vel={np.array2string(all_vel, precision=2)}"
            )

            self._diag_file.write("\n".join(lines) + "\n")
            self._diag_file.flush()
        except Exception as e:
            self._diag_file.write(f"DIAG ERROR: {e}\n")
            self._diag_file.flush()

    def close_diagnostics(self) -> None:
        if self._diag_file is not None:
            self._diag_file.close()
            self._diag_file = None

    def _send_frame(self, conn: socket.socket) -> None:
        send_all(conn, self._header_bytes)
        send_all(conn, self._rgb_buf.data)
        send_all(conn, self._depth_buf.data)

    # -- main loop ----------------------------------------------------------

    def handle_client(self, conn: socket.socket) -> None:
        """Service one client connection until it disconnects."""
        self.reset_robots()

        for _ in range(self._cfg.server.settle_steps):
            self._scene.step()

        self._diag_file = open("sim_diagnostics.log", "w")
        self._diag_file.write("=== sim diagnostics started ===\n")
        self._diag_file.flush()
        print("Writing diagnostics to sim_diagnostics.log")

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

                if self._timings.count % 10 == 0:
                    print(self._timings.report(steps, t5 - t0))
                self._log_diagnostics()

        except (ConnectionError, BrokenPipeError, OSError) as e:
            print(f"Client disconnected: {e}")
        finally:
            self.close_diagnostics()

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
