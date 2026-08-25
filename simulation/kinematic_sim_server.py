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

import cv2
import numpy as np
from camera_utils import (
    camera_view_matrix,
    fov_to_intrinsics,
    ground_plane_depth,
    ground_plane_homography,
)
from config.kinematic import (
    CameraConfig,
    KinematicSimConfig,
    ObstacleConfig,
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
from viewer import Viewer

from hazards import load_hazards

Pose = tuple[float, float, float]  # x, y, yaw in the field frame


# ---------------------------------------------------------------------------
# Plant and opponents
# ---------------------------------------------------------------------------


# Internal integration substep. Mirrors auto_battlebot/plant.py. At the calibrated 31.7 rad/s
# top yaw rate this is 0.06 rad of rotation per substep; a whole 33 ms tick is 1.05 rad, where
# straight-line integration of an arc is wrong by tens of degrees of heading.
SUBSTEP_S = 0.002
# Below this yaw rate the arc radius v/w blows up, so fall back to the straight-line form.
STRAIGHT_W = 1e-6


class Plant:
    """Unicycle with first-order velocity lag (coast) and wall clamping.

    Term for term the same drivetrain model as auto_battlebot/plant.py, which is what the
    velocity jig fits: per-sign deadzone and gain, steer-brake and angular-droop coupling on the
    steady-state target, asymmetric first-order lag, exact arc integration on 2 ms substeps. The
    transport delay lives outside this class, in the server's command ring buffer.
    """

    def __init__(
        self,
        cfg: PlantConfig,
        arena_w: float,
        arena_h: float,
        obstacles: list[ObstacleConfig] | None = None,
    ) -> None:
        self._cfg = cfg
        self.x, self.y = cfg.start_pos
        self.yaw = math.radians(cfg.start_yaw_deg)
        self.v = 0.0
        self.w = 0.0
        self._half_x = arena_w / 2.0 - cfg.radius
        self._half_y = arena_h / 2.0 - cfg.radius
        obstacles = obstacles or []
        # Blocks stop the chassis, so they are grown by the robot radius the same way the walls
        # are. A hole swallows the robot when its centre crosses the lip, so it is not grown.
        self._blocks = [
            (o.center[0], o.center[1], o.radius + cfg.radius)
            for o in obstacles
            if o.kind == "wall_block"
        ]
        self._holes = [(o.center[0], o.center[1], o.radius) for o in obstacles if o.kind == "hole"]
        self.fell_in = False
        self.wall_hits = 0
        self.block_hits = 0
        self.min_hazard_clearance = float("inf")
        # Refreshed each tick from the opponents that carry a hazard_radius. Same treatment as a
        # static block: the chassis is pushed out and the clearance is scored.
        self._moving_blocks: list[tuple[float, float, float]] = []

    def set_moving_blocks(self, blocks: list[tuple[float, float, float]]) -> None:
        self._moving_blocks = [(x, y, r + self._cfg.radius) for x, y, r in blocks]

    @staticmethod
    def _effective_command(cmd: float, dz_pos: float, dz_neg: float) -> float:
        """Deadzone removal, rescaled so full command still maps to full effect."""
        dz = min(max(dz_pos if cmd >= 0.0 else dz_neg, 0.0), 0.95)
        magnitude = max(abs(cmd) - dz, 0.0) / (1.0 - dz)
        return math.copysign(magnitude, cmd) if magnitude > 0.0 else 0.0

    def step(self, linear_cmd: float, angular_cmd: float, dt: float) -> None:
        cfg = self._cfg
        dz_lin_rev = (
            cfg.deadzone_linear if cfg.deadzone_linear_rev is None else cfg.deadzone_linear_rev
        )
        dz_ang_r = (
            cfg.deadzone_angular
            if cfg.deadzone_angular_right is None
            else cfg.deadzone_angular_right
        )
        lc = self._effective_command(
            float(np.clip(linear_cmd, -1.0, 1.0)), cfg.deadzone_linear, dz_lin_rev
        )
        ac = self._effective_command(
            float(np.clip(angular_cmd, -1.0, 1.0)), cfg.deadzone_angular, dz_ang_r
        )

        # Direction-dependent gain (forward/reverse asymmetry), with fall-backs to the
        # symmetric value.
        fwd = cfg.max_linear_speed_fwd or cfg.max_linear_speed
        rev = cfg.max_linear_speed_rev or fwd
        v_target = lc * (fwd if lc >= 0.0 else rev)
        # Coupling multiplies the steady-state target, not the achieved speed: turning costs a
        # fraction of the forward command's authority and driving costs a fraction of the turn's,
        # but neither brakes a robot that is already coasting.
        v_target *= max(0.0, 1.0 - cfg.steer_brake_coeff * abs(ac))
        w_target = ac * cfg.max_angular_speed
        w_target *= max(0.0, 1.0 - cfg.angular_droop_coeff * abs(lc))

        # Separate spin-up vs coast/brake time constants, again falling back to the single tau.
        tau_l_acc = cfg.tau_linear_accel or cfg.tau_linear
        tau_a_acc = cfg.tau_angular_accel or cfg.tau_angular
        tau_l_dec = cfg.tau_linear_decel or tau_l_acc
        tau_a_dec = cfg.tau_angular_decel or tau_a_acc

        substeps = max(1, round(dt / SUBSTEP_S))
        sub_dt = dt / substeps
        for _ in range(substeps):
            if self._substep(
                v_target, w_target, sub_dt, tau_l_acc, tau_l_dec, tau_a_acc, tau_a_dec
            ):
                break
            if self.fell_in:
                self.v = 0.0
                self.w = 0.0
                break

    def _resolve_obstacles(self, nx: float, ny: float) -> tuple[float, float, bool]:
        """Push the chassis out of any block, record the closest a hazard came, and latch a
        fall-in. Returns the corrected position and whether a block was hit."""
        blocked = False
        # A block is a wall with a curved face: push the chassis back out along the radius, which
        # is what the rectangle clamp does for the arena walls.
        for bx, by, br in self._blocks + self._moving_blocks:
            ddx, ddy = nx - bx, ny - by
            dist = math.hypot(ddx, ddy)
            if dist >= br:
                continue
            if dist < 1e-9:
                ddx, ddy, dist = 1.0, 0.0, 1.0
            nx, ny = bx + ddx / dist * br, by + ddy / dist * br
            blocked = True
            self.block_hits += 1

        for hx, hy, hr in self._holes:
            gap = math.hypot(nx - hx, ny - hy) - hr
            self.min_hazard_clearance = min(self.min_hazard_clearance, gap)
            if gap < 0.0:
                self.fell_in = True
        for bx, by, br in self._blocks + self._moving_blocks:
            gap = math.hypot(nx - bx, ny - by) - br
            self.min_hazard_clearance = min(self.min_hazard_clearance, gap)
        return nx, ny, blocked

    def _substep(
        self,
        v_target: float,
        w_target: float,
        dt: float,
        tau_l_acc: float,
        tau_l_dec: float,
        tau_a_acc: float,
        tau_a_dec: float,
    ) -> bool:
        """Advance one substep: pose along the arc, then the velocity update. Returns True on a
        wall hit, which ends the tick."""
        # Pose uses the velocity at the start of the substep, so the arc is exact for the v and w
        # actually held over it.
        next_yaw = self.yaw + self.w * dt
        if abs(self.w) > STRAIGHT_W:
            radius = self.v / self.w
            dx = radius * (math.sin(next_yaw) - math.sin(self.yaw))
            dy = -radius * (math.cos(next_yaw) - math.cos(self.yaw))
        else:
            dx = self.v * dt * math.cos(self.yaw)
            dy = self.v * dt * math.sin(self.yaw)

        # Accel or decel is decided per channel by whether the target is further from zero than
        # the current speed. Braking into a reversal uses the decel constant until the sign flips,
        # as the drivetrain does: friction first, then torque.
        tau_lin = tau_l_acc if abs(v_target) > abs(self.v) else tau_l_dec
        tau_ang = tau_a_acc if abs(w_target) > abs(self.w) else tau_a_dec
        a_lin = 1.0 - math.exp(-dt / max(tau_lin, 1e-4))
        a_ang = 1.0 - math.exp(-dt / max(tau_ang, 1e-4))

        nx, ny = self.x + dx, self.y + dy
        hit = False
        if abs(nx) > self._half_x:
            nx = math.copysign(self._half_x, nx)
            hit = True
        if abs(ny) > self._half_y:
            ny = math.copysign(self._half_y, ny)
            hit = True
        if hit:
            self.wall_hits += 1

        nx, ny, blocked = self._resolve_obstacles(nx, ny)
        hit = hit or blocked

        self.x, self.y = nx, ny
        self.yaw = math.atan2(math.sin(next_yaw), math.cos(next_yaw))
        self.v += a_lin * (v_target - self.v)
        self.w += a_ang * (w_target - self.w)
        if hit:
            self.v = 0.0  # wall stops forward motion
        return hit

    def pose(self) -> Pose:
        return self.x, self.y, self.yaw


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
        # Our robot keeps its slot in the ground-truth list whatever happens, because the C++ side
        # maps ground truth to frame ids by position. A dropped self-observation is signalled by
        # NaN in the slot rather than by removing it, which the filter reads as "no measurement
        # this frame" and answers with a held, stale pose.
        if self._cfg.our_dropout_prob > 0.0 and self._rng.random() < self._cfg.our_dropout_prob:
            out_our: Pose = (math.nan, math.nan, math.nan)
        else:
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
        self._plant = Plant(cfg.our_robot, cfg.arena.width, cfg.arena.height, self._obstacles)
        self._opponents = [
            Opponent(o, cfg.arena.width, cfg.arena.height, rng, self._obstacles)
            for o in cfg.opponents
        ]
        self._perception = Perception(
            cfg.perception, cfg.camera, cfg.sim.dt, cfg.latency.observation_ms, rng
        )
        self._cmd_buf: deque[tuple[float, float]] = self._empty_command_buffer()
        self._tick = 0
        self._sim_time = 0.0

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
        cfg = self._cfg
        self._plant = Plant(cfg.our_robot, cfg.arena.width, cfg.arena.height, self._obstacles)
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

    def _send_frame(self, conn: socket.socket, our: Pose, opps: list[Pose]) -> None:
        self._render_camera()
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

            obs_our, obs_opps = self._perception.observe(
                self._plant.pose(), [o.pose() for o in self._opponents]
            )
            self._send_frame(conn, obs_our, obs_opps)
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
