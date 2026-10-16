"""Fast headless kinematic simulation server for control tuning.

A lightweight alternative to the Genesis server (sim_server.py): same TCP protocol
(simulation/protocol.py), but 2D kinematic physics instead of rendering. Pair it with
config/headless_sim.toml on the C++ side (Noop perception + GroundTruthRobotFilter), so the real
PursuitNavigation runs unchanged and the resulting MCAP is scored by
playground/control_stage0/stage0_metrics.py exactly like a real fight.

The sim owns logical time: it sends its accumulated sim_time in each response, and the C++ side
adopts it (ManualClock), so the controller's dt is correct no matter how fast the loop free-runs.
Runs are deterministic (seeded) and can go far faster than real time.

Models the effects that actually drive overshoot:
  - our robot's drivetrain, from the plant [our_robot] plant selects (simulation/plants/): the
    first-order kinematic lag model, or the Mr Stabs Mk2 MuJoCo rigid body
  - actuation latency (command ring buffer, in sim ticks)
  - perception: the sensing model's latency, dropouts, noise and heading flips
    (auto_battlebot/perception/sensing_model.py), plus the flat-plane projection bias
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
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch
from camera_utils import (
    camera_view_matrix,
    fov_to_intrinsics,
    ground_plane_depth,
    ground_plane_homography,
)
from config.kinematic import (
    KinematicSimConfig,
    ObstacleConfig,
    OpponentConfig,
)
from config.loader import load_config
from plants import PlantInterface, Pose, make_plant
from protocol import (
    GT_COUNT_FMT,
    GT_POSE_FMT,
    MEAS_HEADER_FMT,
    MEAS_POSE_FMT,
    REQUEST_FMT,
    REQUEST_SIZE,
    RESPONSE_HEADER_FMT,
    configure_socket,
    recv_all,
    send_all,
)
from viewer import Viewer

from auto_battlebot.perception.sensing_model import (
    Context,
    MeasurementModel,
    SensingModelFile,
    TruePoses,
)
from hazards import load_hazards

# Footprint radius for an opponent with no hazard_radius: the half-diagonal of the 0.15 x 0.15 m
# box GroundTruthRobotFilter reports for every robot. Feeds the sensing model's robot-distance
# context only.
DEFAULT_OPPONENT_RADIUS_M = 0.106

# [sim] trace_csv columns. Poses are ground truth, not what perception reported; goal is the first
# opponent, which a go-to-point mission drives to. Pitch is 0 on the kinematic plant.
TRACE_HEADER = (
    "tick",
    "t",
    "cmd_lin",
    "cmd_ang",
    "x",
    "y",
    "yaw",
    "v",
    "w",
    "pitch",
    "goal_x",
    "goal_y",
)

# ---------------------------------------------------------------------------
# Opponents
# ---------------------------------------------------------------------------


class Opponent:
    def __init__(
        self,
        cfg: OpponentConfig,
        arena_w: float,
        arena_h: float,
        rng: np.random.Generator,
        obstacles: list[ObstacleConfig] | None = None,
    ) -> None:
        self._cfg = cfg
        self.x, self.y = cfg.start_pos
        self.yaw = math.radians(cfg.heading_deg)
        self._half_x = arena_w / 2.0 - 0.11
        self._half_y = arena_h / 2.0 - 0.11
        self._rng = rng
        # Opponents keep out of hazards too, or a run starts with one standing in the hole and
        # the safest-point solver spends the match routing around a target that cannot exist.
        self._keep_out = [(o.center[0], o.center[1], o.radius + 0.11) for o in (obstacles or [])]
        self.dragged = False
        self.hazard_radius = cfg.hazard_radius
        if self._in_hazard(self.x, self.y):
            self.x, self.y = self._push_out(self.x, self.y)
        self._angle = 0.0
        self._target = self._random_target()
        self._replay: list[Pose] = []
        self._replay_idx = 0
        if cfg.behavior == "replay" and cfg.replay_csv:
            self._replay = _load_replay_csv(Path(cfg.replay_csv))

    def _in_hazard(self, x: float, y: float) -> bool:
        return any(math.hypot(x - hx, y - hy) < hr for hx, hy, hr in self._keep_out)

    def _push_out(self, x: float, y: float) -> tuple[float, float]:
        """Nudge a position to the nearest hazard boundary. Used for spawns and for the
        drag handler, where a caller can put the opponent anywhere."""
        for hx, hy, hr in self._keep_out:
            dx, dy = x - hx, y - hy
            dist = math.hypot(dx, dy)
            if dist < hr:
                if dist < 1e-9:
                    dx, dy, dist = 1.0, 0.0, 1.0
                x, y = hx + dx / dist * hr, hy + dy / dist * hr
        return x, y

    def _random_target(self) -> tuple[float, float]:
        for _ in range(32):
            candidate = (
                float(self._rng.uniform(-self._half_x * 0.9, self._half_x * 0.9)),
                float(self._rng.uniform(-self._half_y * 0.9, self._half_y * 0.9)),
            )
            if not self._in_hazard(*candidate):
                return candidate
        # Hazards cover most of the reachable arena; fall back to the last draw pushed clear.
        return self._push_out(*candidate)

    def _clamp(self) -> None:
        self.x = float(np.clip(self.x, -self._half_x, self._half_x))
        self.y = float(np.clip(self.y, -self._half_y, self._half_y))
        self.x, self.y = self._push_out(self.x, self.y)

    def place(self, x: float, y: float) -> None:
        """Put the opponent somewhere directly (viewer drag). Re-seeds the parameterised
        behaviours so a release resumes from the drop point rather than snapping back."""
        self.x, self.y = x, y
        self._clamp()
        self._angle = math.atan2(self.y, self.x)
        self._target = self._random_target()

    def step(self, dt: float) -> None:
        if self.dragged:
            return
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


@dataclass
class Frame:
    """What perception delivered on one tick, one entry per ground-truth slot."""

    delivered: bool
    capture_time: float
    measured: list[bool]
    poses: list[Pose]


class Perception:
    """Degrades true poses into what perception delivers: the sensing model's measurement stream
    plus the flat-plane projection bias, or the true poses every tick when no model is set.

    The sim captures one perception frame per tick, so the model's frame rate is the tick rate.
    """

    def __init__(self, cfg: KinematicSimConfig) -> None:
        self._cfg = cfg.perception
        self._dt = cfg.sim.dt
        self._cam_x, self._cam_y, self._cam_z = cfg.camera.pos
        radius = [cfg.our_robot.radius] + [
            o.hazard_radius if o.hazard_radius > 0.0 else DEFAULT_OPPONENT_RADIUS_M
            for o in cfg.opponents
        ]
        self._radius = torch.tensor([radius], dtype=torch.float64)
        self._half = torch.tensor([cfg.arena.width / 2.0, cfg.arena.height / 2.0])
        self._previous: np.ndarray | None = None
        self._model: MeasurementModel | None = None
        if self._cfg.sensing_model:
            self._model = MeasurementModel(
                SensingModelFile.load(Path(self._cfg.sensing_model)),
                ["ours"] + ["opponent"] * len(cfg.opponents),
                worlds=1,
                seed=cfg.sim.seed,
                randomize=self._cfg.randomize,
                frame_rate_hz=1.0 / cfg.sim.dt,
            )
            self._model.reset(torch.ones(1, dtype=torch.bool), 0.0)

    def _project_bias(self, pose: Pose) -> Pose:
        x, y, yaw = pose
        bias = self._cfg.projection_bias
        if not bias.enabled or self._cam_z <= 1e-6:
            return pose
        dx, dy = x - self._cam_x, y - self._cam_y
        scale = bias.keypoint_height / self._cam_z
        return (x + scale * dx, y + scale * dy, yaw)

    def observe(self, now: float, poses: list[Pose]) -> Frame:
        if self._model is None:
            return Frame(True, now, [True] * len(poses), [self._project_bias(p) for p in poses])

        current = np.array(poses, dtype=np.float64)
        previous = current if self._previous is None else self._previous
        self._previous = current
        velocity = (current[:, :2] - previous[:, :2]) / self._dt
        yaw_rate = np.angle(np.exp(1j * (current[:, 2] - previous[:, 2]))) / self._dt
        truth = TruePoses(
            xy=torch.from_numpy(current[None, :, :2].copy()),
            yaw=torch.from_numpy(current[None, :, 2].copy()),
            velocity=torch.from_numpy(velocity[None]),
            yaw_rate=torch.from_numpy(yaw_rate[None]),
        )
        out = self._model.step(
            now, truth, Context.from_rect_arena(truth.xy, self._radius, self._half)
        )
        xy = out.xy[0].numpy()
        yaw = out.yaw[0].numpy()
        return Frame(
            delivered=bool(out.delivered[0]),
            capture_time=float(out.stamp[0]),
            measured=[bool(m) for m in out.measured[0]],
            poses=[
                self._project_bias((float(xy[r, 0]), float(xy[r, 1]), float(yaw[r])))
                for r in range(len(poses))
            ],
        )


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
        self._obstacles = load_hazards(cfg.obstacles_file) if cfg.obstacles_file else []
        if self._obstacles:
            summary = ", ".join(
                f"{o.kind}@({o.center[0]:.2f},{o.center[1]:.2f}) r={o.radius:.2f}"
                for o in self._obstacles
            )
            print(f"Obstacles from {cfg.obstacles_file}: {summary}")

        # One compositor, two consumers. The UI draws robot markers, the field border and hazard
        # rings by projecting field points through the intrinsics above, so the frame it draws on
        # has to be a plausible camera image or the whole overlay collapses into a few pixels. The
        # floor is a plane, so the camera view is an exact homography of this top-down composition
        # and needs no second renderer.
        self._world = Viewer(cfg, self._obstacles)
        self._viewer = self._world if cfg.viewer.enable else None
        self._homography = ground_plane_homography(
            tf_matrix, (fx, fy, cx, cy), self._world.metres_per_pixel(), cfg.viewer.window_px
        )
        # Constant: the camera does not move and the floor does not either.
        self._depth = ground_plane_depth(tf_matrix, (fx, fy, cx, cy), cam.res_width, cam.res_height)
        self._rgb = np.zeros((cam.res_height, cam.res_width, 3), dtype=np.uint8)

    def _reset(self) -> None:
        cfg = self._cfg
        rng = np.random.default_rng(cfg.sim.seed)
        self._plant = self._make_plant()
        self._opponents = [
            Opponent(o, cfg.arena.width, cfg.arena.height, rng, self._obstacles)
            for o in cfg.opponents
        ]
        self._perception = Perception(cfg)
        self._cmd_buf: deque[tuple[float, float]] = self._empty_command_buffer()
        self._tick = 0
        self._sim_time = 0.0

    def _make_plant(self) -> PlantInterface:
        cfg = self._cfg
        return make_plant(
            cfg.our_robot,
            cfg.arena.width,
            cfg.arena.height,
            self._obstacles,
            # Same filter and order as the set_moving_blocks call in handle_client.
            [o.hazard_radius for o in cfg.opponents if o.hazard_radius > 0.0],
        )

    def _render_camera(self) -> None:
        """Warp the top-down arena into the camera's view.

        Costs wall-clock time and nothing else: the sim owns logical time, so a slower render
        makes the sim slower in real time and changes nothing the controller sees.
        """
        cam = self._cfg.camera
        # Write into the existing buffer rather than rebinding it: _send_frame ships
        # `self._rgb.data` straight down the socket, so it has to stay the contiguous array the
        # C++ side reads as BGR.
        self._rgb[:] = cv2.warpPerspective(
            self._world.compose_world(self._plant, self._opponents),
            self._homography,
            (cam.res_width, cam.res_height),
            dst=self._rgb,
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )

    def _empty_command_buffer(self) -> deque[tuple[float, float]]:
        """Command pipeline pre-filled with neutral, sized to the configured actuation latency."""
        cfg = self._cfg
        delay = max(0, round((cfg.latency.command_ms / 1000.0) / cfg.sim.dt))
        return deque([(0.0, 0.0)] * delay, maxlen=delay + 1)

    def _respawn(self) -> None:
        """Put our robot back at its start pose, leaving everything else alone.

        Deliberately not `_reset`: that rewinds tick and sim_time, and the C++ side drives its
        ManualClock off sim_time. Time going backwards mid-connection desynchronises the filter in
        ways that read as control bugs. Opponents keep where they are, including where they were
        dragged to, because the point of continuing is to retry against the same situation.
        """
        self._plant = self._make_plant()
        self._cmd_buf = self._empty_command_buffer()

    def _finish_episode(self, outcome: str) -> bool:
        """Report an outcome. Returns True when the run should end."""
        self._report(outcome)
        if self._cfg.sim.stop_on_outcome:
            return True
        print(
            f"sim: {outcome} at t={self._sim_time:.2f} s; respawning our robot and continuing "
            f"([sim] stop_on_outcome = true to close the connection instead).",
            flush=True,
        )
        self._respawn()
        return False

    def _send_frame(self, conn: socket.socket, truth: list[Pose], frame: Frame) -> None:
        self._render_camera()
        header = struct.pack(RESPONSE_HEADER_FMT, *self._header_const, self._sim_time)
        gt = struct.pack(GT_COUNT_FMT, len(truth))
        for pose in truth:
            gt += struct.pack(GT_POSE_FMT, *pose)
        meas = struct.pack(MEAS_HEADER_FMT, float(frame.delivered), frame.capture_time)
        for measured, pose in zip(frame.measured, frame.poses):
            meas += struct.pack(MEAS_POSE_FMT, float(measured), *pose)
        send_all(conn, header)
        send_all(conn, self._rgb.data)
        send_all(conn, self._depth.data)
        send_all(conn, gt)
        send_all(conn, meas)

    def handle_client(self, conn: socket.socket) -> None:
        if not self._cfg.sim.trace_csv:
            self._run_client(conn, None)
            return
        with open(self._cfg.sim.trace_csv, "w", newline="") as handle:
            trace = csv.writer(handle)
            trace.writerow(TRACE_HEADER)
            self._run_client(conn, trace)

    def _run_client(self, conn: socket.socket, trace: Any | None) -> None:
        self._reset()
        cfg = self._cfg
        while cfg.sim.max_ticks == 0 or self._tick < cfg.sim.max_ticks:
            data = recv_all(conn, REQUEST_SIZE)
            linear_x, _linear_y, angular_z = struct.unpack(REQUEST_FMT, data)

            # Actuation latency: apply the command issued command_latency ago.
            self._cmd_buf.append((linear_x, angular_z))
            applied = self._cmd_buf[0]
            self._plant.set_moving_blocks(
                [(o.x, o.y, o.hazard_radius) for o in self._opponents if o.hazard_radius > 0.0]
            )
            self._plant.step(applied[0], applied[1], cfg.sim.dt)
            for opponent in self._opponents:
                opponent.step(cfg.sim.dt)
            if trace is not None:
                x, y, yaw = self._plant.pose()
                goal = self._opponents[0].pose() if self._opponents else (math.nan, math.nan, 0.0)
                trace.writerow(
                    [
                        self._tick,
                        f"{self._sim_time + cfg.sim.dt:.4f}",
                        f"{applied[0]:.5f}",
                        f"{applied[1]:.5f}",
                        f"{x:.5f}",
                        f"{y:.5f}",
                        f"{yaw:.5f}",
                        f"{self._plant.v:.5f}",
                        f"{self._plant.w:.5f}",
                        f"{getattr(self._plant, 'pitch', 0.0):.5f}",
                        f"{goal[0]:.5f}",
                        f"{goal[1]:.5f}",
                    ]
                )

            truth = [self._plant.pose()] + [o.pose() for o in self._opponents]
            self._send_frame(conn, truth, self._perception.observe(self._sim_time, truth))
            if self._viewer is not None:
                self._viewer.render(
                    self._plant, self._opponents, self._tick, self._sim_time, applied
                )
            self._tick += 1
            self._sim_time += cfg.sim.dt
            if self._plant.fell_in and self._finish_episode("FELL_IN"):
                return
        self._report("MAX_TICKS")
        print(
            f"sim: [sim] max_ticks = {cfg.sim.max_ticks} reached after "
            f"{self._sim_time:.1f} s of sim time. Closing the connection, which is what makes the "
            f"C++ side log 'failed to receive header' and exit. Set max_ticks = 0 to run without "
            f"a limit.",
            flush=True,
        )

    def _report(self, outcome: str) -> None:
        """One machine-readable line per episode. sim_sweep greps it for the hazard columns;
        a metric derived from the MCAP alone could not see a fall-in, because the run ends there."""
        clearance = self._plant.min_hazard_clearance
        clearance_str = "nan" if clearance == float("inf") else f"{clearance:.4f}"
        print(
            f"EPISODE outcome={outcome} tick={self._tick} sim_time={self._sim_time:.3f} "
            f"fell_in={int(self._plant.fell_in)} wall_hits={self._plant.wall_hits} "
            f"block_hits={self._plant.block_hits} min_clearance={clearance_str}",
            flush=True,
        )

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
