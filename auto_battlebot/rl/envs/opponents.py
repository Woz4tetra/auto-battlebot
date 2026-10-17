"""Opponent behaviours for the RL environments (plan section 6), batched over worlds in torch.

Every behaviour drives a unicycle: a forward speed along the opponent's heading (negative backs
up) and a yaw rate, which the Warp environment hands to its `move_mocap` kernel each policy
step. Every behaviour has a heading, so the front arc is always exercised. Positions are kept
inside the arena by stopping the velocity component that points into a wall.

| Behaviour | Parameters drawn per episode |
| --- | --- |
| static | pose |
| straight | speed 0.3 to 1.5 m/s; reflects off walls |
| circle | speed 0.3 to 1.5 m/s, radius 0.2 to 0.6 m, direction |
| random_walk | speed 0.2 to 1.2 m/s, turn-rate noise 1 to 4 rad/s |
| pursuit | lead 0 to 0.3 s, delay 50 to 250 ms, top speed 0.8 to 2.5 m/s, turn 4 to 10 rad/s |
| corner | a corner; backs into it, then turns to face us |
| replay | a human opponent track (`replay_tracks.py`) |

The evaluation holds out the fastest, shortest-delay corner of `pursuit` (top speed above
2.3 m/s with a delay under 70 ms): training never draws it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import torch

if TYPE_CHECKING:
    from auto_battlebot.rl.envs.replay_tracks import ReplayTracks

BEHAVIOURS = ("static", "straight", "circle", "random_walk", "pursuit", "corner", "replay")
STATIC, STRAIGHT, CIRCLE, RANDOM_WALK, PURSUIT, CORNER, REPLAY = range(len(BEHAVIOURS))
PURSUIT_HOLDOUT_SPEED = 2.3
PURSUIT_HOLDOUT_DELAY = 0.07
DELAY_SLOTS = 16  # pursuit reaction-delay ring, in policy steps (16 x 20 ms > 250 ms)
OUR_HALF_DIAGONAL_M = 0.106  # Mr Stabs Mk2 footprint, 0.137 x 0.16 m
OUR_HALF_EXTENT_M = (0.0685, 0.08)  # the same footprint as half length and half width
# How far an opponent may press into our footprint along the line between centres. 0: it stops
# at contact. Against an idle robot, pursuit still registers 7 hits per episode at 0 (the boxes'
# corners meet before their centre-line extents do) and flips it 0.06 times per episode, against
# 0.09 at 5 mm; the corner overlap, not this depth, is what pries it.
CONTACT_PRESS_M = 0.0


def box_support(n: torch.Tensor, yaw: torch.Tensor, half: torch.Tensor) -> torch.Tensor:
    """Extent along unit direction `n` [W, 2] of a box with half sizes `half` [W, 2] (length
    along its heading, width across) turned by `yaw` [W]."""
    c, s = torch.cos(yaw), torch.sin(yaw)
    along = (n[:, 0] * c + n[:, 1] * s).abs()
    across = (-n[:, 0] * s + n[:, 1] * c).abs()
    return along * half[:, 0] + across * half[:, 1]


def wrap(a: torch.Tensor) -> torch.Tensor:
    return torch.remainder(a + math.pi, 2.0 * math.pi) - math.pi


@dataclass
class OpponentParams:
    """Per-world episode draws, [W] each."""

    behaviour: torch.Tensor  # long
    speed: torch.Tensor
    radius: torch.Tensor
    direction: torch.Tensor  # +1 / -1
    noise: torch.Tensor
    lead: torch.Tensor
    delay_steps: torch.Tensor  # long
    turn_rate: torch.Tensor
    corner: torch.Tensor  # long 0..3


class Opponents:
    def __init__(
        self, worlds: int, device: torch.device, dtype: torch.dtype, gen: torch.Generator
    ) -> None:
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
        )
        self.turn = z.clone()  # random-walk yaw rate state
        self.forward = z.clone()  # commanded forward speed this step
        self.history = torch.zeros(
            worlds, DELAY_SLOTS, 5, device=device, dtype=dtype
        )  # our x, y, vx, vy, yaw
        self.head = 0
        self.replay: ReplayTracks | None = None  # set when the environment loads replay tracks
        self.replay_index = zl.clone()
        self.replay_t = z.clone()

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
        """Draw behaviours and parameters for the `mask` worlds. `holdout` draws only the held-out
        pursuit corner (evaluation); otherwise pursuit never lands in it."""
        n = self.w
        choice = torch.tensor(allowed, device=self.device)[
            torch.randint(len(allowed), (n,), generator=self.gen, device=self.device)
        ]
        p = self.p
        p.behaviour = torch.where(mask, choice, p.behaviour)
        speed = self._uniform(0.3, 1.5)
        speed = torch.where(choice == RANDOM_WALK, self._uniform(0.2, 1.2), speed)
        top = self._uniform(0.8, 2.5)
        delay = self._uniform(0.05, 0.25)
        if holdout:
            top = self._uniform(PURSUIT_HOLDOUT_SPEED, 2.5)
            delay = self._uniform(0.05, PURSUIT_HOLDOUT_DELAY)
        else:
            corner = (top > PURSUIT_HOLDOUT_SPEED) & (delay < PURSUIT_HOLDOUT_DELAY)
            delay = torch.where(corner, self._uniform(PURSUIT_HOLDOUT_DELAY, 0.25), delay)
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
        p.turn_rate = torch.where(mask, self._uniform(4.0, 10.0), p.turn_rate)
        p.corner = torch.where(
            mask, torch.randint(4, (n,), generator=self.gen, device=self.device), p.corner
        )
        self.turn = torch.where(mask, torch.zeros_like(self.turn), self.turn)
        if self.replay is not None:
            pick = torch.randint(self.replay.count, (n,), generator=self.gen, device=self.device)
            start = self._rand(n) * self.replay.start_span(pick)
            self.replay_index = torch.where(mask, pick, self.replay_index)
            self.replay_t = torch.where(mask, start, self.replay_t)

    def record_us(
        self, our_xy: torch.Tensor, our_v: torch.Tensor, our_yaw: torch.Tensor | None = None
    ) -> None:
        self.head = (self.head + 1) % DELAY_SLOTS
        self.history[:, self.head, :2] = our_xy
        self.history[:, self.head, 2:4] = our_v
        self.history[:, self.head, 4] = 0.0 if our_yaw is None else our_yaw

    def command(
        self,
        xy: torch.Tensor,
        yaw: torch.Tensor,
        arena_half: torch.Tensor,
        footprint: torch.Tensor,
        dt: float,
        opp_half: torch.Tensor | None = None,
    ) -> torch.Tensor:
        """World-frame (vx, vy, yaw rate) for every world's opponent this step, [W, 3].

        `footprint` is the opponent's half-diagonal, which keeps it off the walls; `opp_half`
        [W, 2] is its half length and half width, which stops it at contact with us. Without it
        the opponent is taken as a square with that half-diagonal."""
        p = self.p
        b = p.behaviour
        forward = torch.zeros_like(yaw)
        omega = torch.zeros_like(yaw)
        lim = arena_half - footprint[:, None]

        # straight: reflect the heading when the next step would leave the arena.
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
        self.turn = torch.where(
            walk, 0.9 * self.turn + p.noise * math.sqrt(1 - 0.81) * noise, self.turn
        )
        centre_pull = wrap(torch.atan2(-xy[:, 1], -xy[:, 0]) - yaw)
        near_wall = (xy.abs() > lim - 0.2).any(1)
        omega = torch.where(walk, torch.where(near_wall, 3.0 * centre_pull, self.turn), omega)
        forward = torch.where(walk, p.speed, forward)

        pursuit = b == PURSUIT
        slot = (self.head - p.delay_steps) % DELAY_SLOTS
        seen = self.history[torch.arange(self.w, device=self.device), slot]
        aim = seen[:, :2] + seen[:, 2:4] * p.lead[:, None]
        err = wrap(torch.atan2(aim[:, 1] - xy[:, 1], aim[:, 0] - xy[:, 0]) - yaw)
        # Proportional: the rate that closes the error in one step, capped at turn_rate. The old
        # clamp(err / dt, -1, 1) * turn_rate saturated past 1 degree of error and turned
        # bang-bang, overshooting and reversing every 20 ms step.
        cap = p.turn_rate
        omega = torch.where(pursuit, torch.maximum(torch.minimum(err / dt, cap), -cap), omega)
        forward = torch.where(pursuit, p.speed * torch.cos(err).clamp(min=0.0), forward)

        corner = b == CORNER
        sx = torch.where((p.corner % 2) == 0, 1.0, -1.0).to(self.dtype)
        sy = torch.where(p.corner < 2, 1.0, -1.0).to(self.dtype)
        spot = torch.stack([sx, sy], 1) * (lim - 0.05)
        to_spot = spot - xy
        dist = to_spot.norm(dim=1)
        # Back in: the rear points at the corner, so the heading points away from it.
        away = torch.atan2(-to_spot[:, 1], -to_spot[:, 0])
        backing = dist > 0.08
        face_us = torch.atan2(
            self.history[:, self.head, 1] - xy[:, 1], self.history[:, self.head, 0] - xy[:, 0]
        )
        target = torch.where(backing, away, face_us)
        cerr = wrap(target - yaw)
        omega = torch.where(corner, (cerr / dt).clamp(-3.0, 3.0), omega)
        forward = torch.where(corner & backing, -0.6 * torch.cos(cerr).clamp(min=0.0), forward)

        if self.replay is not None:
            rep = b == REPLAY
            self.replay_t = torch.where(rep, self.replay_t + dt, self.replay_t)
            target_xy, target_yaw = self.replay.sample(
                self.replay_index, self.replay_t, arena_half - footprint[:, None]
            )
            vel = (target_xy - xy) / dt
            vel = vel.clamp(-3.0, 3.0)
            yaw_rate = (wrap(target_yaw - yaw) / dt).clamp(-15.0, 15.0)
            world = torch.stack([vel[:, 0], vel[:, 1], yaw_rate], 1)
        else:
            rep = torch.zeros_like(straight)
            world = torch.zeros(self.w, 3, device=self.device, dtype=self.dtype)

        self.forward = forward
        v = torch.stack([forward * torch.cos(yaw), forward * torch.sin(yaw), omega], 1)
        v = torch.where(rep[:, None], world, v)
        # Wall stop: no velocity component into a wall the opponent is touching.
        stop_x = ((xy[:, 0] >= lim[:, 0]) & (v[:, 0] > 0)) | (
            (xy[:, 0] <= -lim[:, 0]) & (v[:, 0] < 0)
        )
        stop_y = ((xy[:, 1] >= lim[:, 1]) & (v[:, 1] > 0)) | (
            (xy[:, 1] <= -lim[:, 1]) & (v[:, 1] < 0)
        )
        v[:, 0] = torch.where(stop_x, 0.0, v[:, 0])
        v[:, 1] = torch.where(stop_y, 0.0, v[:, 1])
        # No pushing through us. The opponent is a mocap body with infinite mass: driven into our
        # wedge it cannot ride up it the way a real robot would, so it pries our chassis over
        # instead (an idle robot flipped in 66 of 80 early-ending stage C episodes). Its closing
        # speed is capped so it arrives at contact (CONTACT_PRESS_M) and goes no further; it
        # still turns and slides along us. Contact is the two footprint boxes' extents along the
        # line between centres, so a pursuer reaches our nose and its arc touches. The earlier
        # two-circle rule stopped it 15 cm off a nose-on robot, out of contact for good.
        if opp_half is None:
            opp_half = (footprint / math.sqrt(2.0))[:, None].expand(-1, 2)
        ours = self.history[:, self.head, :2]
        our_yaw = self.history[:, self.head, 4]
        away = ours - xy
        dist = away.norm(dim=1).clamp(min=1e-6)
        normal = away / dist[:, None]
        us_half = torch.tensor(OUR_HALF_EXTENT_M, device=self.device, dtype=self.dtype).expand(
            self.w, 2
        )
        reach = box_support(normal, yaw, opp_half) + box_support(normal, our_yaw, us_half)
        allowed = ((dist - reach + CONTACT_PRESS_M) / dt).clamp(min=0.0)
        closing = (v[:, :2] * normal).sum(1)
        excess = (closing - allowed).clamp(min=0.0)
        v[:, :2] = v[:, :2] - excess[:, None] * normal
        return v
