"""Fast headless kinematic simulation server for control tuning.

A lightweight alternative to the Genesis server (sim_server.py): same TCP protocol
(simulation/protocol.py), but 2D kinematic physics instead of rendering. Pair it with
config/headless_sim.toml on the C++ side (Noop perception + GroundTruthRobotFilter), so the real
PursuitNavigation runs unchanged and the resulting MCAP is scored by
playground/control_stage0/stage0_metrics.py exactly like a real fight.

Lockstep: one C++ controller tick == one sim step, so runs are deterministic regardless of
wall-clock speed and latency is measured in sim-ticks.

Models the effects that actually drive overshoot (none of which a physics engine gives you):
  - first-order drivetrain lag / coast, capped at fitted max speeds
  - actuation latency (command ring buffer, in sim-ticks)
  - perception latency, position noise, dropout, and the flat-plane projection bias
  - square arena with walls

Usage:
    source scripts/activate_python.sh
    python simulation/kinematic_sim_server.py simulation/kinematic_sim.toml
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import tomllib
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from camera_utils import camera_view_matrix, fov_to_intrinsics
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


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------


@dataclass
class CameraCfg:
    res_width: int = 16
    res_height: int = 16
    fov: float = 70.0
    pos: tuple[float, float, float] = (0.0, -1.5, 1.0)
    lookat: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class PlantCfg:
    start_pos: tuple[float, float] = (-0.5, 0.0)
    start_yaw_deg: float = 0.0
    max_linear_speed: float = 2.0  # m/s at command magnitude 1.0
    max_angular_speed: float = 6.0  # rad/s at command magnitude 1.0
    tau_linear: float = 0.15  # s, first-order coast time constant
    tau_angular: float = 0.08  # s
    radius: float = 0.11  # m, half robot length for wall clamp


@dataclass
class OpponentCfg:
    behavior: str = "static"
    start_pos: tuple[float, float] = (0.5, 0.0)
    speed: float = 1.0
    heading_deg: float = 180.0
    radius: float = 0.5  # circle radius
    replay_csv: str = ""


@dataclass
class PerceptionCfg:
    pos_noise_std: float = 0.0
    yaw_noise_std: float = 0.0
    dropout_prob: float = 0.0
    bias_enabled: bool = False
    keypoint_height: float = 0.06


@dataclass
class SimCfg:
    host: str = "127.0.0.1"
    port: int = 14882
    dt: float = 1.0 / 30.0
    max_ticks: int = 5400  # ~3 min at 30 Hz; server closes the connection after this
    arena_w: float = 2.4
    arena_h: float = 2.4
    command_latency_ms: float = 0.0
    observation_latency_ms: float = 0.0
    seed: int = 0
    camera: CameraCfg = field(default_factory=CameraCfg)
    plant: PlantCfg = field(default_factory=PlantCfg)
    perception: PerceptionCfg = field(default_factory=PerceptionCfg)
    opponents: list[OpponentCfg] = field(default_factory=lambda: [OpponentCfg()])


def _tup(seq, n: int):
    vals = list(seq)
    return tuple(float(v) for v in vals[:n])


def load_config(path: Path) -> SimCfg:
    with open(path, "rb") as f:
        raw = tomllib.load(f)
    cfg = SimCfg()
    srv = raw.get("server", {})
    cfg.host = srv.get("host", cfg.host)
    cfg.port = int(srv.get("port", cfg.port))
    sim = raw.get("sim", {})
    cfg.dt = float(sim.get("dt", cfg.dt))
    cfg.max_ticks = int(sim.get("max_ticks", cfg.max_ticks))
    cfg.seed = int(sim.get("seed", cfg.seed))
    arena = raw.get("arena", {})
    cfg.arena_w = float(arena.get("width", cfg.arena_w))
    cfg.arena_h = float(arena.get("height", cfg.arena_h))
    lat = raw.get("latency", {})
    cfg.command_latency_ms = float(lat.get("command_ms", cfg.command_latency_ms))
    cfg.observation_latency_ms = float(
        lat.get("observation_ms", cfg.observation_latency_ms)
    )

    cam = raw.get("camera", {})
    cfg.camera = CameraCfg(
        res_width=int(cam.get("res_width", CameraCfg.res_width)),
        res_height=int(cam.get("res_height", CameraCfg.res_height)),
        fov=float(cam.get("fov", CameraCfg.fov)),
        pos=_tup(cam.get("pos", CameraCfg.pos), 3),
        lookat=_tup(cam.get("lookat", CameraCfg.lookat), 3),
    )

    p = raw.get("our_robot", {})
    cfg.plant = PlantCfg(
        start_pos=_tup(p.get("start_pos", PlantCfg.start_pos), 2),
        start_yaw_deg=float(p.get("start_yaw_deg", PlantCfg.start_yaw_deg)),
        max_linear_speed=float(p.get("max_linear_speed", PlantCfg.max_linear_speed)),
        max_angular_speed=float(p.get("max_angular_speed", PlantCfg.max_angular_speed)),
        tau_linear=float(p.get("tau_linear", PlantCfg.tau_linear)),
        tau_angular=float(p.get("tau_angular", PlantCfg.tau_angular)),
        radius=float(p.get("radius", PlantCfg.radius)),
    )

    perc = raw.get("perception", {})
    bias = perc.get("projection_bias", {})
    cfg.perception = PerceptionCfg(
        pos_noise_std=float(perc.get("pos_noise_std", 0.0)),
        yaw_noise_std=float(perc.get("yaw_noise_std", 0.0)),
        dropout_prob=float(perc.get("dropout_prob", 0.0)),
        bias_enabled=bool(bias.get("enabled", False)),
        keypoint_height=float(bias.get("keypoint_height", 0.06)),
    )

    opps = raw.get("opponent", [])
    if isinstance(opps, dict):
        opps = [opps]
    cfg.opponents = [
        OpponentCfg(
            behavior=str(o.get("behavior", "static")),
            start_pos=_tup(o.get("start_pos", OpponentCfg.start_pos), 2),
            speed=float(o.get("speed", 1.0)),
            heading_deg=float(o.get("heading_deg", 180.0)),
            radius=float(o.get("radius", 0.5)),
            replay_csv=str(o.get("replay_csv", "")),
        )
        for o in opps
    ] or [OpponentCfg()]
    return cfg


# ---------------------------------------------------------------------------
# Plant and opponents
# ---------------------------------------------------------------------------


class Plant:
    """Unicycle with first-order velocity lag (coast) and wall clamping."""

    def __init__(self, cfg: PlantCfg, arena_w: float, arena_h: float) -> None:
        self.cfg = cfg
        self.x, self.y = cfg.start_pos
        self.yaw = math.radians(cfg.start_yaw_deg)
        self.v = 0.0
        self.w = 0.0
        self.half_x = arena_w / 2.0 - cfg.radius
        self.half_y = arena_h / 2.0 - cfg.radius

    def step(self, linear_cmd: float, angular_cmd: float, dt: float) -> None:
        v_target = float(np.clip(linear_cmd, -1.0, 1.0)) * self.cfg.max_linear_speed
        w_target = float(np.clip(angular_cmd, -1.0, 1.0)) * self.cfg.max_angular_speed
        a_lin = 1.0 - math.exp(-dt / max(self.cfg.tau_linear, 1e-4))
        a_ang = 1.0 - math.exp(-dt / max(self.cfg.tau_angular, 1e-4))
        self.v += a_lin * (v_target - self.v)
        self.w += a_ang * (w_target - self.w)

        self.yaw = math.atan2(
            math.sin(self.yaw + self.w * dt), math.cos(self.yaw + self.w * dt)
        )
        nx = self.x + self.v * math.cos(self.yaw) * dt
        ny = self.y + self.v * math.sin(self.yaw) * dt

        hit = False
        if abs(nx) > self.half_x:
            nx = math.copysign(self.half_x, nx)
            hit = True
        if abs(ny) > self.half_y:
            ny = math.copysign(self.half_y, ny)
            hit = True
        self.x, self.y = nx, ny
        if hit:
            self.v = 0.0  # wall stops forward motion

    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, self.yaw


class Opponent:
    def __init__(
        self, cfg: OpponentCfg, arena_w: float, arena_h: float, rng: np.random.Generator
    ):
        self.cfg = cfg
        self.x, self.y = cfg.start_pos
        self.yaw = math.radians(cfg.heading_deg)
        self.half_x = arena_w / 2.0 - 0.11
        self.half_y = arena_h / 2.0 - 0.11
        self.rng = rng
        self._angle = 0.0
        self._target = self._random_target()
        self._replay: list[tuple[float, float, float]] = []
        self._replay_idx = 0
        if cfg.behavior == "replay" and cfg.replay_csv:
            self._replay = _load_replay_csv(Path(cfg.replay_csv))

    def _random_target(self) -> tuple[float, float]:
        return (
            float(self.rng.uniform(-self.half_x * 0.9, self.half_x * 0.9)),
            float(self.rng.uniform(-self.half_y * 0.9, self.half_y * 0.9)),
        )

    def _clamp(self) -> None:
        self.x = float(np.clip(self.x, -self.half_x, self.half_x))
        self.y = float(np.clip(self.y, -self.half_y, self.half_y))

    def step(self, dt: float) -> None:
        b = self.cfg.behavior
        if b == "static":
            return
        if b == "replay":
            if self._replay:
                self.x, self.y, self.yaw = self._replay[
                    min(self._replay_idx, len(self._replay) - 1)
                ]
                self._replay_idx += 1
            return
        if b == "straight":
            self.x += self.cfg.speed * math.cos(self.yaw) * dt
            self.y += self.cfg.speed * math.sin(self.yaw) * dt
            if abs(self.x) >= self.half_x or abs(self.y) >= self.half_y:
                self.yaw = math.atan2(
                    math.sin(self.yaw + math.pi), math.cos(self.yaw + math.pi)
                )
            self._clamp()
            return
        if b == "circle":
            self._angle += (self.cfg.speed / max(self.cfg.radius, 0.01)) * dt
            self.x = self.cfg.radius * math.cos(self._angle)
            self.y = self.cfg.radius * math.sin(self._angle)
            self._clamp()
            return
        if b == "random_walk":
            dx, dy = self._target[0] - self.x, self._target[1] - self.y
            dist = math.hypot(dx, dy)
            if dist < 0.05:
                self._target = self._random_target()
                return
            self.x += self.cfg.speed * dx / dist * dt
            self.y += self.cfg.speed * dy / dist * dt
            self._clamp()
            return

    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, self.yaw


def _load_replay_csv(path: Path) -> list[tuple[float, float, float]]:
    """Load an opponent trajectory CSV with columns x, y, yaw (one row per tick)."""
    import csv

    rows: list[tuple[float, float, float]] = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append((float(r["x"]), float(r["y"]), float(r.get("yaw", 0.0))))
    return rows


# ---------------------------------------------------------------------------
# Perception emulation
# ---------------------------------------------------------------------------


class Perception:
    """Degrades true poses into observed ones: latency, noise, dropout, flat-plane bias."""

    def __init__(
        self,
        cfg: PerceptionCfg,
        cam: CameraCfg,
        dt: float,
        obs_latency_ms: float,
        rng: np.random.Generator,
    ) -> None:
        self.cfg = cfg
        self.cam_x, self.cam_y, self.cam_z = cam.pos
        self.rng = rng
        delay_ticks = max(0, round((obs_latency_ms / 1000.0) / dt))
        # buffer of [(our, [opp...])] true poses; observe the entry delay_ticks ago
        self.buf: deque = deque(maxlen=delay_ticks + 1)

    def _project_bias(self, x: float, y: float) -> tuple[float, float]:
        if not self.cfg.bias_enabled or self.cam_z <= 1e-6:
            return x, y
        dx, dy = x - self.cam_x, y - self.cam_y
        scale = self.cfg.keypoint_height / self.cam_z
        return x + scale * dx, y + scale * dy

    def observe(self, our: tuple, opps: list[tuple]) -> tuple[tuple, list[tuple]]:
        self.buf.append((our, opps))
        obs_our, obs_opps = self.buf[0]  # oldest within the delay window

        def degrade(p: tuple) -> tuple | None:
            if self.rng.random() < self.cfg.dropout_prob:
                return None
            x, y, yaw = p
            x, y = self._project_bias(x, y)
            x += self.rng.normal(0.0, self.cfg.pos_noise_std)
            y += self.rng.normal(0.0, self.cfg.pos_noise_std)
            yaw += self.rng.normal(0.0, self.cfg.yaw_noise_std)
            return (x, y, yaw)

        # Our robot: bias + noise but never drop (controller needs self-pose).
        ox, oy, oyaw = obs_our
        ox, oy = self._project_bias(ox, oy)
        ox += self.rng.normal(0.0, self.cfg.pos_noise_std)
        oy += self.rng.normal(0.0, self.cfg.pos_noise_std)
        out_our = (ox, oy, oyaw + self.rng.normal(0.0, self.cfg.yaw_noise_std))
        out_opps = [d for d in (degrade(p) for p in obs_opps) if d is not None]
        return out_our, out_opps


# ---------------------------------------------------------------------------
# Server
# ---------------------------------------------------------------------------


class KinematicServer:
    def __init__(self, cfg: SimCfg) -> None:
        self.cfg = cfg
        cam = cfg.camera
        fx, fy, cx, cy = fov_to_intrinsics(cam.fov, cam.res_width, cam.res_height)
        tf_matrix = camera_view_matrix(cam.pos, cam.lookat)
        # Constant header fields; sim_time is appended per frame (the sim owns the only dt).
        self.header_const = [
            cam.res_width,
            cam.res_height,
            *tf_matrix.flatten().tolist(),
            fx,
            fy,
            cx,
            cy,
        ]
        self.rgb = np.zeros((cam.res_height, cam.res_width, 3), dtype=np.uint8)
        self.depth = np.zeros((cam.res_height, cam.res_width), dtype=np.float32)

    def _reset(self) -> None:
        cfg = self.cfg
        rng = np.random.default_rng(cfg.seed)
        self.plant = Plant(cfg.plant, cfg.arena_w, cfg.arena_h)
        self.opponents = [
            Opponent(o, cfg.arena_w, cfg.arena_h, rng) for o in cfg.opponents
        ]
        self.perception = Perception(
            cfg.perception, cfg.camera, cfg.dt, cfg.observation_latency_ms, rng
        )
        delay = max(0, round((cfg.command_latency_ms / 1000.0) / cfg.dt))
        self.cmd_buf: deque = deque([(0.0, 0.0)] * delay, maxlen=delay + 1)
        self.tick = 0

    def _send_frame(self, conn: socket.socket, our, opps, sim_time: float) -> None:
        gt = struct.pack(GT_COUNT_FMT, 1 + len(opps))
        gt += struct.pack(GT_POSE_FMT, *our)
        for p in opps:
            gt += struct.pack(GT_POSE_FMT, *p)
        header = struct.pack(RESPONSE_HEADER_FMT, *self.header_const, sim_time)
        send_all(conn, header)
        send_all(conn, self.rgb.data)
        send_all(conn, self.depth.data)
        send_all(conn, gt)

    def handle_client(self, conn: socket.socket) -> None:
        self._reset()
        cfg = self.cfg
        while self.tick < cfg.max_ticks:
            data = recv_all(conn, REQUEST_SIZE)
            linear_x, _linear_y, angular_z = struct.unpack(REQUEST_FMT, data)

            # Actuation latency: apply the command issued command_latency ago.
            self.cmd_buf.append((linear_x, angular_z))
            applied = self.cmd_buf[0]
            self.plant.step(applied[0], applied[1], cfg.dt)
            for opp in self.opponents:
                opp.step(cfg.dt)

            obs_our, obs_opps = self.perception.observe(
                self.plant.pose(), [o.pose() for o in self.opponents]
            )
            self._send_frame(conn, obs_our, obs_opps, self.tick * cfg.dt)
            self.tick += 1
        print(f"Reached max_ticks={cfg.max_ticks}; closing connection.")

    def serve_forever(self) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.cfg.host, self.cfg.port))
        srv.listen(1)
        print(f"Kinematic sim ready on {self.cfg.host}:{self.cfg.port}")
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
    cfg = load_config(args.config)
    KinematicServer(cfg).serve_forever()


if __name__ == "__main__":
    main()
