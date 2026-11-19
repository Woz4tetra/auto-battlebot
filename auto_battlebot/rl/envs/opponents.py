"""Opponent behaviours for the RL environments (plan section 6), batched over worlds in torch."""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, NamedTuple

import torch

if TYPE_CHECKING:
    from auto_battlebot.rl.envs.replay_tracks import ReplayTracks

BEHAVIOURS = ("static", "straight", "circle", "random_walk", "pursuit", "corner", "replay")
STATIC, STRAIGHT, CIRCLE, RANDOM_WALK, PURSUIT, CORNER, REPLAY = range(len(BEHAVIOURS))
PURSUIT_HOLDOUT_SPEED = 2.3
PURSUIT_HOLDOUT_DELAY = 0.07


class PursuitParams(NamedTuple):
    top_speed_mps: float
    turn_rate_rad_s: float
    delay_s: float


PURSUIT_COL_TOP_SPEED, PURSUIT_COL_TURN_RATE, PURSUIT_COL_DELAY = range(3)
PURSUIT_EASY = PursuitParams(0.8, 4.0, 0.25)
PURSUIT_HARD = PursuitParams(2.5, 10.0, 0.05)
PURSUIT_GRIP_MPS2 = (8.0, 16.0)
PURSUIT_GRID_SPEEDS_MPS = (0.8, 1.2, 1.6, 2.0, 2.5)
PURSUIT_GRID_TURN_RATES_RAD_S = (4.0, 6.0, 8.0, 10.0)
PURSUIT_GRID_DELAYS_S = (0.05, 0.1, 0.15, 0.25)
PURSUIT_GRID = list(
    itertools.product(PURSUIT_GRID_SPEEDS_MPS, PURSUIT_GRID_TURN_RATES_RAD_S, PURSUIT_GRID_DELAYS_S)
)
HIST_X, HIST_Y, HIST_VX, HIST_VY, HIST_YAW = range(5)
HIST_XY = slice(HIST_X, HIST_Y + 1)
HIST_V = slice(HIST_VX, HIST_VY + 1)
DELAY_SLOTS = 16
MAX_CORNER = 3
OUR_HALF_DIAGONAL_M = 0.106
OUR_HALF_EXTENT_M = (0.0685, 0.08)
CONTACT_PRESS_M = 0.0


def box_support(
    unit_direction: torch.Tensor, yaw: torch.Tensor, half_length_width: torch.Tensor
) -> torch.Tensor:
    """Box extent [W] along unit_direction [W, 2]; half_length_width [W, 2], yaw [W]."""
    n = unit_direction
    c, s = torch.cos(yaw), torch.sin(yaw)
    along = (n[:, 0] * c + n[:, 1] * s).abs()
    across = (-n[:, 0] * s + n[:, 1] * c).abs()
    return along * half_length_width[:, 0] + across * half_length_width[:, 1]


def wrap(a: torch.Tensor) -> torch.Tensor:
    return torch.remainder(a + math.pi, 2.0 * math.pi) - math.pi


@dataclass
class OpponentParams:
    """Per-world episode draws, [W] each."""

    behaviour: torch.Tensor
    speed: torch.Tensor
    radius: torch.Tensor
    direction: torch.Tensor
    noise: torch.Tensor
    lead: torch.Tensor
    delay_steps: torch.Tensor
    turn_rate: torch.Tensor
    corner: torch.Tensor
    grip: torch.Tensor | None = None

    def __post_init__(self) -> None:
        for name in ("behaviour", "delay_steps", "corner"):
            if getattr(self, name).dtype != torch.long:
                raise TypeError(f"OpponentParams.{name} must be a long tensor")
        if not bool((self.direction.abs() == 1).all()):
            raise ValueError("OpponentParams.direction must be +1 or -1")
        if not bool(((self.corner >= 0) & (self.corner <= MAX_CORNER)).all()):
            raise ValueError(f"OpponentParams.corner must be in 0..{MAX_CORNER}")


class Opponents:
    def __init__(
        self,
        worlds: int,
        device: torch.device,
        dtype: torch.dtype,
        gen: torch.Generator,
        our_half: tuple[float, float] = OUR_HALF_EXTENT_M,
    ) -> None:
        """`our_half`: our footprint as half length and half width, centred on our pose."""
        self.w = worlds
        self.device = device
        self.dtype = dtype
        self.gen = gen
        z = torch.zeros(worlds, device=device, dtype=dtype)
        zl = torch.zeros(worlds, device=device, dtype=torch.long)
        self.p = OpponentParams(
            zl.clone(),
            z.clone(),
            z.clone(),
            z + 1,
            z.clone(),
            z.clone(),
            zl.clone(),
            z.clone(),
            zl.clone(),
            z.clone(),
        )
        self.walk_yaw_rate = z.clone()
        self.commanded_forward_mps = z.clone()
        self.history = torch.zeros(worlds, DELAY_SLOTS, HIST_YAW + 1, device=device, dtype=dtype)
        self.head = 0
        self.replay: ReplayTracks | None = None
        self.replay_index = zl.clone()
        self.replay_t = z.clone()
        self.pursuit_level = 1.0
        self.pursuit_hard: PursuitParams = PURSUIT_HARD
        self.pursuit_fixed: torch.Tensor | None = None
        self.our_half = torch.tensor(our_half, device=device, dtype=dtype)

    def _pursuit_draw(self, u: torch.Tensor, column: int) -> torch.Tensor:
        easy, hard = PURSUIT_EASY[column], self.pursuit_hard[column]
        if hard < easy:
            u = 1.0 - u
        return easy + self.pursuit_level * u * (hard - easy)

    def _rand(self, n: int) -> torch.Tensor:
        return torch.rand(n, generator=self.gen, device=self.device, dtype=self.dtype)

    def _uniform(self, lo: float, hi: float) -> torch.Tensor:
        return lo + (hi - lo) * self._rand(self.w)

    def reset(
        self,
        mask: torch.Tensor,
        allowed: tuple[int, ...],
        speed_scale: torch.Tensor,
        holdout: bool,
    ) -> None:
        n = self.w
        choice = torch.tensor(allowed, device=self.device)[
            torch.randint(len(allowed), (n,), generator=self.gen, device=self.device)
        ]
        p = self.p
        p.behaviour = torch.where(mask, choice, p.behaviour)
        speed = self._uniform(0.3, 1.5)
        speed = torch.where(choice == RANDOM_WALK, self._uniform(0.2, 1.2), speed)
        top = self._pursuit_draw(self._rand(n), PURSUIT_COL_TOP_SPEED)
        delay = self._pursuit_draw(self._rand(n), PURSUIT_COL_DELAY)
        if holdout:
            top = self._uniform(PURSUIT_HOLDOUT_SPEED, 2.5)
            delay = self._uniform(0.05, PURSUIT_HOLDOUT_DELAY)
        else:
            corner = (top > PURSUIT_HOLDOUT_SPEED) & (delay < PURSUIT_HOLDOUT_DELAY)
            delay = torch.where(corner, self._uniform(PURSUIT_HOLDOUT_DELAY, 0.25), delay)
        if self.pursuit_fixed is not None:
            top = self.pursuit_fixed[:, PURSUIT_COL_TOP_SPEED]
            delay = self.pursuit_fixed[:, PURSUIT_COL_DELAY]
        speed = torch.where(choice == PURSUIT, top, speed) * speed_scale
        p.speed = torch.where(mask, speed, p.speed)
        p.radius = torch.where(mask, self._uniform(0.2, 0.6), p.radius)
        p.direction = torch.where(
            mask, torch.where(self._rand(n) < 0.5, -1.0, 1.0).to(self.dtype), p.direction
        )
        p.noise = torch.where(mask, self._uniform(1.0, 4.0), p.noise)
        p.lead = torch.where(mask, self._uniform(0.0, 0.3), p.lead)
        p.delay_steps = torch.where(
            mask, torch.round(delay / 0.02).long().clamp(1, DELAY_SLOTS - 1), p.delay_steps
        )
        turn = self._pursuit_draw(self._rand(n), PURSUIT_COL_TURN_RATE)
        if self.pursuit_fixed is not None:
            turn = self.pursuit_fixed[:, PURSUIT_COL_TURN_RATE]
        p.turn_rate = torch.where(mask, turn, p.turn_rate)
        p.corner = torch.where(
            mask, torch.randint(4, (n,), generator=self.gen, device=self.device), p.corner
        )
        self.walk_yaw_rate = torch.where(
            mask, torch.zeros_like(self.walk_yaw_rate), self.walk_yaw_rate
        )
        if self.replay is not None:
            pick = torch.randint(self.replay.count, (n,), generator=self.gen, device=self.device)
            start = self._rand(n) * self.replay.start_span(pick)
            self.replay_index = torch.where(mask, pick, self.replay_index)
            self.replay_t = torch.where(mask, start, self.replay_t)
        lo, hi = PURSUIT_GRIP_MPS2
        grip = self._uniform(lo, hi)
        p.grip = torch.where(mask, grip, grip if p.grip is None else p.grip)
        self.commanded_forward_mps = torch.where(
            mask, torch.zeros_like(self.commanded_forward_mps), self.commanded_forward_mps
        )

    def record_us(
        self, our_xy: torch.Tensor, our_v: torch.Tensor, our_yaw: torch.Tensor | None = None
    ) -> None:
        self.head = (self.head + 1) % DELAY_SLOTS
        self.history[:, self.head, HIST_XY] = our_xy
        self.history[:, self.head, HIST_V] = our_v
        self.history[:, self.head, HIST_YAW] = 0.0 if our_yaw is None else our_yaw

    def command(
        self,
        xy: torch.Tensor,
        yaw: torch.Tensor,
        arena_half: torch.Tensor,
        footprint_half_diagonal: torch.Tensor,
        dt: float,
        opp_half_length_width: torch.Tensor | None = None,
        external_drive: tuple[torch.Tensor, torch.Tensor] | None = None,
    ) -> torch.Tensor:
        """World-frame (vx, vy, yaw rate) [W, 3] for every world's opponent this step."""
        p = self.p
        b = p.behaviour
        forward = torch.zeros_like(yaw)
        omega = torch.zeros_like(yaw)
        lim = arena_half - footprint_half_diagonal[:, None]

        heading = torch.stack([torch.cos(yaw), torch.sin(yaw)], 1)
        ahead = xy + heading * (p.speed[:, None] * dt * 5.0)
        out_x = ahead[:, 0].abs() > lim[:, 0]
        out_y = ahead[:, 1].abs() > lim[:, 1]
        reflected = torch.atan2(
            torch.where(out_y, -heading[:, 1], heading[:, 1]),
            torch.where(out_x, -heading[:, 0], heading[:, 0]),
        )
        straight = b == STRAIGHT
        omega = torch.where(straight & (out_x | out_y), wrap(reflected - yaw) / dt, omega)
        forward = torch.where(straight, p.speed, forward)

        circle = b == CIRCLE
        forward = torch.where(circle, p.speed, forward)
        omega = torch.where(circle, p.direction * p.speed / p.radius, omega)

        walk = b == RANDOM_WALK
        noise = torch.randn(self.w, generator=self.gen, device=self.device, dtype=self.dtype)
        self.walk_yaw_rate = torch.where(
            walk,
            0.9 * self.walk_yaw_rate + p.noise * math.sqrt(1 - 0.81) * noise,
            self.walk_yaw_rate,
        )
        centre_pull = wrap(torch.atan2(-xy[:, 1], -xy[:, 0]) - yaw)
        near_wall = (xy.abs() > lim - 0.2).any(1)
        omega = torch.where(
            walk, torch.where(near_wall, 3.0 * centre_pull, self.walk_yaw_rate), omega
        )
        forward = torch.where(walk, p.speed, forward)

        pursuit = b == PURSUIT
        slot = (self.head - p.delay_steps) % DELAY_SLOTS
        seen = self.history[torch.arange(self.w, device=self.device), slot]
        aim = seen[:, HIST_XY] + seen[:, HIST_V] * p.lead[:, None]
        err = wrap(torch.atan2(aim[:, 1] - xy[:, 1], aim[:, 0] - xy[:, 0]) - yaw)
        cap = p.turn_rate
        chase = p.speed * torch.cos(err).clamp(min=0.0)
        if p.grip is not None:
            step = p.grip * dt
            chase = self.commanded_forward_mps + (chase - self.commanded_forward_mps).clamp(
                min=-step, max=step
            )
            cap = torch.minimum(cap, p.grip / chase.abs().clamp(min=1e-3))
        omega = torch.where(pursuit, torch.maximum(torch.minimum(err / dt, cap), -cap), omega)
        forward = torch.where(pursuit, chase, forward)

        corner = b == CORNER
        sx = torch.where((p.corner % 2) == 0, 1.0, -1.0).to(self.dtype)
        sy = torch.where(p.corner < 2, 1.0, -1.0).to(self.dtype)
        spot = torch.stack([sx, sy], 1) * (lim - 0.05)
        to_spot = spot - xy
        dist = to_spot.norm(dim=1)
        away = torch.atan2(-to_spot[:, 1], -to_spot[:, 0])
        backing = dist > 0.08
        face_us = torch.atan2(
            self.history[:, self.head, HIST_Y] - xy[:, 1],
            self.history[:, self.head, HIST_X] - xy[:, 0],
        )
        target = torch.where(backing, away, face_us)
        cerr = wrap(target - yaw)
        omega = torch.where(corner, (cerr / dt).clamp(-3.0, 3.0), omega)
        forward = torch.where(corner & backing, -0.6 * torch.cos(cerr).clamp(min=0.0), forward)

        if self.replay is not None:
            rep = b == REPLAY
            self.replay_t = torch.where(rep, self.replay_t + dt, self.replay_t)
            target_xy, target_yaw = self.replay.sample(
                self.replay_index, self.replay_t, arena_half - footprint_half_diagonal[:, None]
            )
            vel = (target_xy - xy) / dt
            vel = vel.clamp(-3.0, 3.0)
            yaw_rate = (wrap(target_yaw - yaw) / dt).clamp(-15.0, 15.0)
            world = torch.stack([vel[:, 0], vel[:, 1], yaw_rate], 1)
        else:
            rep = torch.zeros_like(straight)
            world = torch.zeros(self.w, 3, device=self.device, dtype=self.dtype)

        self.commanded_forward_mps = forward
        v = torch.stack([forward * torch.cos(yaw), forward * torch.sin(yaw), omega], 1)
        v = torch.where(rep[:, None], world, v)
        if external_drive is not None:
            v = torch.where(external_drive[0][:, None], external_drive[1], v)
        stop_x = ((xy[:, 0] >= lim[:, 0]) & (v[:, 0] > 0)) | (
            (xy[:, 0] <= -lim[:, 0]) & (v[:, 0] < 0)
        )
        stop_y = ((xy[:, 1] >= lim[:, 1]) & (v[:, 1] > 0)) | (
            (xy[:, 1] <= -lim[:, 1]) & (v[:, 1] < 0)
        )
        v[:, 0] = torch.where(stop_x, 0.0, v[:, 0])
        v[:, 1] = torch.where(stop_y, 0.0, v[:, 1])
        if opp_half_length_width is None:
            opp_half_length_width = (footprint_half_diagonal / math.sqrt(2.0))[:, None].expand(
                -1, 2
            )
        ours = self.history[:, self.head, HIST_XY]
        our_yaw = self.history[:, self.head, HIST_YAW]
        away = ours - xy
        dist = away.norm(dim=1).clamp(min=1e-6)
        normal = away / dist[:, None]
        us_half = self.our_half.expand(self.w, 2)
        reach = box_support(normal, yaw, opp_half_length_width) + box_support(
            normal, our_yaw, us_half
        )
        allowed = ((dist - reach + CONTACT_PRESS_M) / dt).clamp(min=0.0)
        closing = (v[:, :2] * normal).sum(1)
        excess = (closing - allowed).clamp(min=0.0)
        v[:, :2] = v[:, :2] - excess[:, None] * normal
        return v
