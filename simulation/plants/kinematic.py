"""2D kinematic drivetrain: first-order velocity lag, exact arc integration, clamped walls."""

from __future__ import annotations

import math

import numpy as np
from config.kinematic import ObstacleConfig, PlantConfig

from plants.base import HazardMonitor, Pose

# Internal integration substep. Mirrors auto_battlebot/control/plant.py. At the calibrated
# 31.7 rad/s top yaw rate this is 0.06 rad of rotation per substep; a whole 33 ms tick is
# 1.05 rad, where straight-line integration of an arc is wrong by tens of degrees of heading.
SUBSTEP_S = 0.002
# Below this yaw rate the arc radius v/w blows up, so fall back to the straight-line form.
STRAIGHT_W = 1e-6


class KinematicPlant:
    """Unicycle with first-order velocity lag (coast) and wall clamping.

    Term for term the same drivetrain model as auto_battlebot/control/plant.py, which is
    what the
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
        self._hazards = HazardMonitor(obstacles or [], cfg.radius)
        self.wall_hits = 0
        self.block_hits = 0

    @property
    def fell_in(self) -> bool:
        return self._hazards.fell_in

    @property
    def min_hazard_clearance(self) -> float:
        return self._hazards.min_clearance

    def set_moving_blocks(self, blocks: list[tuple[float, float, float]]) -> None:
        # Same treatment as a static block: the chassis is pushed out and the clearance is scored.
        self._hazards.set_moving_blocks(blocks)

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
        for bx, by, br in self._hazards.blocks():
            ddx, ddy = nx - bx, ny - by
            dist = math.hypot(ddx, ddy)
            if dist >= br:
                continue
            if dist < 1e-9:
                ddx, ddy, dist = 1.0, 0.0, 1.0
            nx, ny = bx + ddx / dist * br, by + ddy / dist * br
            blocked = True
            self.block_hits += 1
        self._hazards.record(nx, ny)
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
