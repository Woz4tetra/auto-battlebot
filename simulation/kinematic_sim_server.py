"""Fast headless kinematic simulation server for control tuning.

A lightweight alternative to the Genesis server (sim_server.py): same TCP protocol
(simulation/protocol.py), but 2D kinematic physics instead of rendering. Pair it with
config/headless_sim.toml on the C++ side (Noop perception + GroundTruthRobotFilter), so the real
PursuitNavigation runs unchanged and the resulting MCAP is scored by
playground/control_stage0/stage0_metrics.py exactly like a real fight.

The sim owns logical time: it sends its accumulated sim_time in each response, and the C++ side
adopts it (ManualClock), so the controller's dt is correct no matter how fast the loop free-runs.
Runs are deterministic (seeded) and can go far faster than real time.

Models the effects that actually drive overshoot (none of which a physics engine gives you):
  - first-order drivetrain lag / coast, capped at fitted max speeds
  - actuation latency (command ring buffer, in sim ticks)
  - perception latency, position noise, dropout, and the flat-plane projection bias
  - square arena with walls

Usage:
    source scripts/activate_python.sh
    python simulation/kinematic_sim_server.py simulation/kinematic_sim.toml
"""

from __future__ import annotations

import argparse
import csv
import math
import socket
import struct
from collections import deque
from pathlib import Path

import numpy as np
from camera_utils import camera_view_matrix, fov_to_intrinsics
from config.kinematic import (
    CameraConfig,
    KinematicSimConfig,
    OpponentConfig,
    PerceptionConfig,
    PlantConfig,
)
from config.loader import load_config
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

Pose = tuple[float, float, float]  # x, y, yaw in the field frame


# ---------------------------------------------------------------------------
# Plant and opponents
# ---------------------------------------------------------------------------


class Plant:
    """Unicycle with first-order velocity lag (coast) and wall clamping."""

    def __init__(self, cfg: PlantConfig, arena_w: float, arena_h: float) -> None:
        self._cfg = cfg
        self.x, self.y = cfg.start_pos
        self.yaw = math.radians(cfg.start_yaw_deg)
        self.v = 0.0
        self.w = 0.0
        self._half_x = arena_w / 2.0 - cfg.radius
        self._half_y = arena_h / 2.0 - cfg.radius

    @staticmethod
    def _apply_deadzone(cmd: float, dz: float) -> float:
        if dz <= 0.0 or abs(cmd) <= dz:
            return 0.0 if abs(cmd) <= dz else cmd
        return math.copysign((abs(cmd) - dz) / (1.0 - dz), cmd)

    def step(self, linear_cmd: float, angular_cmd: float, dt: float) -> None:
        cfg = self._cfg
        lc = self._apply_deadzone(float(np.clip(linear_cmd, -1.0, 1.0)), cfg.deadzone_linear)
        ac = self._apply_deadzone(float(np.clip(angular_cmd, -1.0, 1.0)), cfg.deadzone_angular)

        # Direction-dependent gain (forward/reverse asymmetry), with fall-backs to the
        # symmetric value.
        fwd = cfg.max_linear_speed_fwd or cfg.max_linear_speed
        rev = cfg.max_linear_speed_rev or fwd
        v_target = lc * (fwd if lc >= 0.0 else rev)
        # Steer-brake coupling: turning bleeds forward speed.
        v_target *= max(0.0, 1.0 - cfg.steer_brake_coeff * abs(ac))
        w_target = ac * cfg.max_angular_speed

        # Separate spin-up vs coast/brake time constants, again falling back to the single tau.
        tau_l_acc = cfg.tau_linear_accel or cfg.tau_linear
        tau_a_acc = cfg.tau_angular_accel or cfg.tau_angular
        tau_lin = tau_l_acc if abs(v_target) > abs(self.v) else (cfg.tau_linear_decel or tau_l_acc)
        tau_ang = tau_a_acc if abs(w_target) > abs(self.w) else (cfg.tau_angular_decel or tau_a_acc)
        a_lin = 1.0 - math.exp(-dt / max(tau_lin, 1e-4))
        a_ang = 1.0 - math.exp(-dt / max(tau_ang, 1e-4))
        self.v += a_lin * (v_target - self.v)
        self.w += a_ang * (w_target - self.w)

        next_yaw = self.yaw + self.w * dt
        self.yaw = math.atan2(math.sin(next_yaw), math.cos(next_yaw))
        nx = self.x + self.v * math.cos(self.yaw) * dt
        ny = self.y + self.v * math.sin(self.yaw) * dt

        hit = False
        if abs(nx) > self._half_x:
            nx = math.copysign(self._half_x, nx)
            hit = True
        if abs(ny) > self._half_y:
            ny = math.copysign(self._half_y, ny)
            hit = True
        self.x, self.y = nx, ny
        if hit:
            self.v = 0.0  # wall stops forward motion

    def pose(self) -> Pose:
        return self.x, self.y, self.yaw


class Opponent:
    def __init__(
        self, cfg: OpponentConfig, arena_w: float, arena_h: float, rng: np.random.Generator
    ) -> None:
        self._cfg = cfg
        self.x, self.y = cfg.start_pos
        self.yaw = math.radians(cfg.heading_deg)
        self._half_x = arena_w / 2.0 - 0.11
        self._half_y = arena_h / 2.0 - 0.11
        self._rng = rng
        self._angle = 0.0
        self._target = self._random_target()
        self._replay: list[Pose] = []
        self._replay_idx = 0
        if cfg.behavior == "replay" and cfg.replay_csv:
            self._replay = _load_replay_csv(Path(cfg.replay_csv))

    def _random_target(self) -> tuple[float, float]:
        return (
            float(self._rng.uniform(-self._half_x * 0.9, self._half_x * 0.9)),
            float(self._rng.uniform(-self._half_y * 0.9, self._half_y * 0.9)),
        )

    def _clamp(self) -> None:
        self.x = float(np.clip(self.x, -self._half_x, self._half_x))
        self.y = float(np.clip(self.y, -self._half_y, self._half_y))

    def step(self, dt: float) -> None:
        behavior = self._cfg.behavior
        if behavior == "static":
            return
        if behavior == "replay":
            if self._replay:
                idx = min(self._replay_idx, len(self._replay) - 1)
                self.x, self.y, self.yaw = self._replay[idx]
                self._replay_idx += 1
            return
        if behavior == "straight":
            self.x += self._cfg.speed * math.cos(self.yaw) * dt
            self.y += self._cfg.speed * math.sin(self.yaw) * dt
            if abs(self.x) >= self._half_x or abs(self.y) >= self._half_y:
                self.yaw = math.atan2(math.sin(self.yaw + math.pi), math.cos(self.yaw + math.pi))
            self._clamp()
            return
        if behavior == "circle":
            self._angle += (self._cfg.speed / max(self._cfg.radius, 0.01)) * dt
            self.x = self._cfg.radius * math.cos(self._angle)
            self.y = self._cfg.radius * math.sin(self._angle)
            self._clamp()
            return
        if behavior == "random_walk":
            dx, dy = self._target[0] - self.x, self._target[1] - self.y
            dist = math.hypot(dx, dy)
            if dist < 0.05:
                self._target = self._random_target()
                return
            self.x += self._cfg.speed * dx / dist * dt
            self.y += self._cfg.speed * dy / dist * dt
            self._clamp()
            return

    def pose(self) -> Pose:
        return self.x, self.y, self.yaw


def _load_replay_csv(path: Path) -> list[Pose]:
    """Load an opponent trajectory CSV with columns x, y, yaw (one row per tick)."""
    rows: list[Pose] = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append((float(row["x"]), float(row["y"]), float(row.get("yaw", 0.0))))
    return rows


# ---------------------------------------------------------------------------
# Perception emulation
# ---------------------------------------------------------------------------


class Perception:
    """Degrades true poses into observed ones: latency, noise, dropout, flat-plane bias."""

    def __init__(
        self,
        cfg: PerceptionConfig,
        cam: CameraConfig,
        dt: float,
        obs_latency_ms: float,
        rng: np.random.Generator,
    ) -> None:
        self._cfg = cfg
        self._cam_x, self._cam_y, self._cam_z = cam.pos
        self._rng = rng
        delay_ticks = max(0, round((obs_latency_ms / 1000.0) / dt))
        # Buffer of (our, [opp...]) true poses; observe the entry delay_ticks ago.
        self._buf: deque[tuple[Pose, list[Pose]]] = deque(maxlen=delay_ticks + 1)

    def _project_bias(self, x: float, y: float) -> tuple[float, float]:
        bias = self._cfg.projection_bias
        if not bias.enabled or self._cam_z <= 1e-6:
            return x, y
        dx, dy = x - self._cam_x, y - self._cam_y
        scale = bias.keypoint_height / self._cam_z
        return x + scale * dx, y + scale * dy

    def _noisy(self, pose: Pose) -> Pose:
        x, y, yaw = pose
        x, y = self._project_bias(x, y)
        x += self._rng.normal(0.0, self._cfg.pos_noise_std)
        y += self._rng.normal(0.0, self._cfg.pos_noise_std)
        yaw += self._rng.normal(0.0, self._cfg.yaw_noise_std)
        return (x, y, yaw)

    def observe(self, our: Pose, opps: list[Pose]) -> tuple[Pose, list[Pose]]:
        self._buf.append((our, opps))
        obs_our, obs_opps = self._buf[0]  # oldest within the delay window
        # Our robot: bias + noise but never dropped (the controller needs self-pose).
        out_our = self._noisy(obs_our)
        out_opps = [
            self._noisy(p) for p in obs_opps if self._rng.random() >= self._cfg.dropout_prob
        ]
        return out_our, out_opps


# ---------------------------------------------------------------------------
# Server
# ---------------------------------------------------------------------------


class KinematicServer:
    def __init__(self, cfg: KinematicSimConfig) -> None:
        self._cfg = cfg
        cam = cfg.camera
        fx, fy, cx, cy = fov_to_intrinsics(cam.fov, cam.res_width, cam.res_height)
        tf_matrix = camera_view_matrix(cam.pos, cam.lookat)
        # Constant header fields; sim_time is appended per frame in _send_frame.
        self._header_const: tuple[float, ...] = (
            cam.res_width,
            cam.res_height,
            *tf_matrix.flatten().tolist(),
            fx,
            fy,
            cx,
            cy,
        )
        self._rgb = np.zeros((cam.res_height, cam.res_width, 3), dtype=np.uint8)
        self._depth = np.zeros((cam.res_height, cam.res_width), dtype=np.float32)

    def _reset(self) -> None:
        cfg = self._cfg
        rng = np.random.default_rng(cfg.sim.seed)
        self._plant = Plant(cfg.our_robot, cfg.arena.width, cfg.arena.height)
        self._opponents = [
            Opponent(o, cfg.arena.width, cfg.arena.height, rng) for o in cfg.opponents
        ]
        self._perception = Perception(
            cfg.perception, cfg.camera, cfg.sim.dt, cfg.latency.observation_ms, rng
        )
        delay = max(0, round((cfg.latency.command_ms / 1000.0) / cfg.sim.dt))
        self._cmd_buf: deque[tuple[float, float]] = deque([(0.0, 0.0)] * delay, maxlen=delay + 1)
        self._tick = 0
        self._sim_time = 0.0

    def _send_frame(self, conn: socket.socket, our: Pose, opps: list[Pose]) -> None:
        header = struct.pack(RESPONSE_HEADER_FMT, *self._header_const, self._sim_time)
        gt = struct.pack(GT_COUNT_FMT, 1 + len(opps))
        gt += struct.pack(GT_POSE_FMT, *our)
        for pose in opps:
            gt += struct.pack(GT_POSE_FMT, *pose)
        send_all(conn, header)
        send_all(conn, self._rgb.data)
        send_all(conn, self._depth.data)
        send_all(conn, gt)

    def handle_client(self, conn: socket.socket) -> None:
        self._reset()
        cfg = self._cfg
        while self._tick < cfg.sim.max_ticks:
            data = recv_all(conn, REQUEST_SIZE)
            linear_x, _linear_y, angular_z = struct.unpack(REQUEST_FMT, data)

            # Actuation latency: apply the command issued command_latency ago.
            self._cmd_buf.append((linear_x, angular_z))
            applied = self._cmd_buf[0]
            self._plant.step(applied[0], applied[1], cfg.sim.dt)
            for opponent in self._opponents:
                opponent.step(cfg.sim.dt)

            obs_our, obs_opps = self._perception.observe(
                self._plant.pose(), [o.pose() for o in self._opponents]
            )
            self._send_frame(conn, obs_our, obs_opps)
            self._tick += 1
            self._sim_time += cfg.sim.dt
        print(f"Reached max_ticks={cfg.sim.max_ticks}; closing connection.")

    def serve_forever(self) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self._cfg.server.host, self._cfg.server.port))
        srv.listen(1)
        print(f"Kinematic sim ready on {self._cfg.server.host}:{self._cfg.server.port}")
        while True:
            print("Waiting for C++ client...")
            conn, addr = srv.accept()
            configure_socket(conn)
            print(f"Client connected from {addr}")
            try:
                self.handle_client(conn)
            except (ConnectionError, BrokenPipeError, OSError) as e:
                print(f"Client disconnected: {e}")
            finally:
                conn.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path, help="Path to kinematic sim TOML config")
    args = parser.parse_args()
    cfg = load_config(args.config, KinematicSimConfig)
    KinematicServer(cfg).serve_forever()


if __name__ == "__main__":
    main()
