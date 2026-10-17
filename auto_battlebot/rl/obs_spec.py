"""Observation spec v1 for `RlPolicyNavigation`: 138 floats, built twice and checked equal.

This module is the spec. `build_v1` implements it in torch for training; `ObservationBuilderV1`
(`src/rl_policy/observations/observation_v1.cpp`) implements it in C++ for the robot, and
`tests/rl_policy/test_observation_v1.cpp` checks the two against a golden fixture written by
`playground/rl/make_rl_golden_fixture.py`. Any change to the layout, a scale or a cap bumps
`OBS_SPEC_VERSION`, which breaks loading of every bundle trained on the old one (plan section 3).

Everything is in our body frame (x forward, y left) at our axle centre; the field frame's origin
is the field centre. Blocks, in order:

- our robot (6): forward speed /2.5, yaw rate /15, measurement age /0.5 (capped at 1), stale
  flag, mode one-hot ATTACK, RUN_AWAY
- history (32): last 8 policy steps, newest first: actions (linear, angular) x 8, then measured
  (forward speed /2.5, yaw rate /15) x 8
- target opponent (16): position x, y /2, distance /2, bearing sin, cos, heading relative to ours
  sin, cos, velocity x, y /2.5, yaw rate /15, position sigma /0.1, heading sigma /(pi/2),
  yaw-rate sigma /15 (each capped at 1, so unknown reads 1), footprint half-diagonal /0.2,
  measurement age /0.5 (capped at 1), stale flag
- range rays (48): 24 rays from heading 0 counter-clockwise at 15 degrees, two channels each
  (solid, hole), distance /1.5, 1.0 when nothing is within 1.5 m
- nearest hazards (36): 4 discs by edge distance, each: centre x, y /2, raw radius /0.3, kind
  one-hot (hole, wall block, tracked), velocity x, y /2.5, present flag

Solid rays see the field walls, wall blocks and tracked hazards (the house bot); hole rays see
holes. A ray that starts inside a disc reads 0. With no target opponent the opponent block is all
zeros except the three sigmas, the age and the stale flag, which read 1.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import torch

OBS_SPEC_VERSION = 1
HISTORY = 8
NUM_RAYS = 24
RAY_RANGE_M = 1.5
MAX_HAZARDS = 4
SPEED_SCALE = 2.5
YAW_RATE_SCALE = 15.0
AGE_SCALE_S = 0.5
POSITION_SCALE_M = 2.0
POSITION_SIGMA_SCALE_M = 0.1
HEADING_SIGMA_SCALE_RAD = math.pi / 2
FOOTPRINT_SCALE_M = 0.2
HAZARD_RADIUS_SCALE_M = 0.3

HOLE, WALL_BLOCK, TRACKED = 0, 1, 2
HAZARD_KINDS = ("hole", "wall_block", "tracked")


def _names() -> tuple[str, ...]:
    names = ["our_speed", "our_yaw_rate", "our_age", "our_stale", "mode_attack", "mode_run_away"]
    for k in range(HISTORY):
        names += [f"hist_action_linear_{k}", f"hist_action_angular_{k}"]
    for k in range(HISTORY):
        names += [f"hist_speed_{k}", f"hist_yaw_rate_{k}"]
    names += [
        "opp_x",
        "opp_y",
        "opp_distance",
        "opp_bearing_sin",
        "opp_bearing_cos",
        "opp_heading_sin",
        "opp_heading_cos",
        "opp_vx",
        "opp_vy",
        "opp_yaw_rate",
        "opp_position_sigma",
        "opp_heading_sigma",
        "opp_yaw_rate_sigma",
        "opp_half_diagonal",
        "opp_age",
        "opp_stale",
    ]
    for r in range(NUM_RAYS):
        names += [f"ray_solid_{r}", f"ray_hole_{r}"]
    for h in range(MAX_HAZARDS):
        names += [
            f"hazard_{h}_x",
            f"hazard_{h}_y",
            f"hazard_{h}_radius",
            f"hazard_{h}_hole",
            f"hazard_{h}_wall_block",
            f"hazard_{h}_tracked",
            f"hazard_{h}_vx",
            f"hazard_{h}_vy",
            f"hazard_{h}_present",
        ]
    return tuple(names)


OBS_NAMES = _names()
OBS_DIM = len(OBS_NAMES)
assert OBS_DIM == 138, OBS_DIM


@dataclass
class ObsInputs:
    """Everything the builder reads, batched over worlds [W, ...], field frame unless noted.

    Infinite sigmas mean unknown and are allowed. `hazard_*` hold any number of discs H; the
    builder picks the 4 nearest by edge distance. History is newest first.
    """

    our_xy: torch.Tensor  # [W, 2]
    our_yaw: torch.Tensor  # [W]
    our_speed: torch.Tensor  # [W] forward, m/s
    our_yaw_rate: torch.Tensor  # [W] rad/s
    our_age_s: torch.Tensor  # [W]
    our_stale: torch.Tensor  # [W] bool
    run_away: torch.Tensor  # [W] bool
    hist_action: torch.Tensor  # [W, 8, 2]
    hist_speed: torch.Tensor  # [W, 8] m/s
    hist_yaw_rate: torch.Tensor  # [W, 8] rad/s
    opp_present: torch.Tensor  # [W] bool
    opp_xy: torch.Tensor  # [W, 2]
    opp_yaw: torch.Tensor  # [W]
    opp_velocity: torch.Tensor  # [W, 2] field frame, m/s
    opp_yaw_rate: torch.Tensor  # [W]
    opp_position_sigma: torch.Tensor  # [W]
    opp_heading_sigma: torch.Tensor  # [W]
    opp_yaw_rate_sigma: torch.Tensor  # [W]
    opp_half_diagonal: torch.Tensor  # [W] m
    opp_age_s: torch.Tensor  # [W]
    opp_stale: torch.Tensor  # [W] bool
    field_half: torch.Tensor  # [W, 2] half size of the field rectangle
    hazard_xy: torch.Tensor  # [W, H, 2]
    hazard_radius: torch.Tensor  # [W, H] object_radius, never inflated
    hazard_kind: torch.Tensor  # [W, H] long: HOLE, WALL_BLOCK, TRACKED
    hazard_velocity: torch.Tensor  # [W, H, 2]
    hazard_present: torch.Tensor  # [W, H] bool


def wrap(a: torch.Tensor) -> torch.Tensor:
    return torch.remainder(a + math.pi, 2.0 * math.pi) - math.pi


def ray_angles(dtype: torch.dtype, device: torch.device) -> torch.Tensor:
    return torch.arange(NUM_RAYS, dtype=dtype, device=device) * (2.0 * math.pi / NUM_RAYS)


def ray_box(origin: torch.Tensor, direction: torch.Tensor, half: torch.Tensor) -> torch.Tensor:
    """Distance from `origin` [W, 2] along `direction` [W, R, 2] to the box `half` [W, 2] wall.
    0 for an origin on or outside the wall."""
    big = torch.full_like(direction[..., 0], math.inf)
    t = big
    for axis in range(2):
        d = direction[..., axis]
        p = origin[:, None, axis]
        h = half[:, None, axis]
        ta = torch.where(d.abs() > 1e-12, (torch.sign(d) * h - p) / d, big)
        t = torch.minimum(t, ta)
    return t.clamp(min=0.0)


def ray_discs(
    origin: torch.Tensor,
    direction: torch.Tensor,
    centre: torch.Tensor,
    radius: torch.Tensor,
    mask: torch.Tensor,
) -> torch.Tensor:
    """Nearest hit [W, R] of each ray on any masked disc ([W, H]); inf when none. A ray whose
    origin is inside a disc hits it at 0."""
    m = origin[:, None, :] - centre  # [W, H, 2]
    b = (m[:, None, :, :] * direction[:, :, None, :]).sum(-1)  # [W, R, H]
    c = (m * m).sum(-1) - radius * radius  # [W, H]
    c = c[:, None, :].expand_as(b)
    disc = b * b - c
    inside = c <= 0.0
    t = -b - torch.sqrt(disc.clamp(min=0.0))
    hit = (disc >= 0.0) & (t >= 0.0)
    dist = torch.where(
        inside, torch.zeros_like(t), torch.where(hit, t, torch.full_like(t, math.inf))
    )
    dist = torch.where(mask[:, None, :], dist, torch.full_like(dist, math.inf))
    if dist.shape[-1] == 0:
        return torch.full(dist.shape[:2], math.inf, dtype=dist.dtype, device=dist.device)
    return dist.min(dim=-1).values


def _cap(x: torch.Tensor) -> torch.Tensor:
    return torch.nan_to_num(x, nan=1.0, posinf=1.0).clamp(0.0, 1.0)


def build_v1(inp: ObsInputs) -> torch.Tensor:
    """The 138-float observation, [W, 138], in the input dtype."""
    dtype, device = inp.our_xy.dtype, inp.our_xy.device
    w = inp.our_xy.shape[0]
    c, s = torch.cos(inp.our_yaw), torch.sin(inp.our_yaw)

    def body(v: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """Field-frame vector(s) [W, ..., 2] into our body frame."""
        cc = c.reshape(w, *([1] * (v.dim() - 2)))
        ss = s.reshape(w, *([1] * (v.dim() - 2)))
        return cc * v[..., 0] + ss * v[..., 1], -ss * v[..., 0] + cc * v[..., 1]

    one = torch.ones(w, dtype=dtype, device=device)
    our = torch.stack(
        [
            inp.our_speed / SPEED_SCALE,
            inp.our_yaw_rate / YAW_RATE_SCALE,
            _cap(inp.our_age_s / AGE_SCALE_S),
            inp.our_stale.to(dtype),
            (~inp.run_away).to(dtype),
            inp.run_away.to(dtype),
        ],
        dim=1,
    )
    history = torch.cat(
        [
            inp.hist_action.reshape(w, 2 * HISTORY),
            torch.stack(
                [inp.hist_speed / SPEED_SCALE, inp.hist_yaw_rate / YAW_RATE_SCALE], dim=2
            ).reshape(w, 2 * HISTORY),
        ],
        dim=1,
    )

    rx, ry = body(inp.opp_xy - inp.our_xy)
    dist = torch.hypot(rx, ry)
    bearing = torch.atan2(ry, rx)
    heading = wrap(inp.opp_yaw - inp.our_yaw)
    vx, vy = body(inp.opp_velocity)
    known = inp.opp_present.to(dtype)
    opp = torch.stack(
        [
            known * rx / POSITION_SCALE_M,
            known * ry / POSITION_SCALE_M,
            known * dist / POSITION_SCALE_M,
            known * torch.sin(bearing),
            known * torch.cos(bearing),
            known * torch.sin(heading),
            known * torch.cos(heading),
            known * vx / SPEED_SCALE,
            known * vy / SPEED_SCALE,
            known * inp.opp_yaw_rate / YAW_RATE_SCALE,
            torch.where(
                inp.opp_present, _cap(inp.opp_position_sigma / POSITION_SIGMA_SCALE_M), one
            ),
            torch.where(
                inp.opp_present, _cap(inp.opp_heading_sigma / HEADING_SIGMA_SCALE_RAD), one
            ),
            torch.where(inp.opp_present, _cap(inp.opp_yaw_rate_sigma / YAW_RATE_SCALE), one),
            known * inp.opp_half_diagonal / FOOTPRINT_SCALE_M,
            torch.where(inp.opp_present, _cap(inp.opp_age_s / AGE_SCALE_S), one),
            torch.where(inp.opp_present, inp.opp_stale.to(dtype), one),
        ],
        dim=1,
    )

    angles = inp.our_yaw[:, None] + ray_angles(dtype, device)[None, :]
    direction = torch.stack([torch.cos(angles), torch.sin(angles)], dim=-1)  # [W, R, 2]
    walls = ray_box(inp.our_xy, direction, inp.field_half)
    present = inp.hazard_present
    solid_mask = present & (inp.hazard_kind != HOLE)
    hole_mask = present & (inp.hazard_kind == HOLE)
    solid = torch.minimum(
        walls, ray_discs(inp.our_xy, direction, inp.hazard_xy, inp.hazard_radius, solid_mask)
    )
    hole = ray_discs(inp.our_xy, direction, inp.hazard_xy, inp.hazard_radius, hole_mask)
    rays = torch.stack(
        [solid.clamp(max=RAY_RANGE_M) / RAY_RANGE_M, hole.clamp(max=RAY_RANGE_M) / RAY_RANGE_M],
        dim=2,
    ).reshape(w, 2 * NUM_RAYS)

    edge = (inp.hazard_xy - inp.our_xy[:, None, :]).norm(dim=-1) - inp.hazard_radius
    edge = torch.where(present, edge, torch.full_like(edge, math.inf))
    h = inp.hazard_xy.shape[1]
    slots = []
    if h > 0:
        # Stable sort, so ties keep input order in both implementations.
        order = torch.sort(edge, dim=1, stable=True).indices
    for k in range(MAX_HAZARDS):
        if k >= h:
            slots.append(torch.zeros(w, 9, dtype=dtype, device=device))
            continue
        idx = order[:, k]
        rows = torch.arange(w, device=device)
        ok = present[rows, idx]
        okf = ok.to(dtype)
        hx, hy = body(inp.hazard_xy[rows, idx] - inp.our_xy)
        hvx, hvy = body(inp.hazard_velocity[rows, idx])
        kind = inp.hazard_kind[rows, idx]
        slots.append(
            torch.stack(
                [
                    okf * hx / POSITION_SCALE_M,
                    okf * hy / POSITION_SCALE_M,
                    okf * inp.hazard_radius[rows, idx] / HAZARD_RADIUS_SCALE_M,
                    okf * (kind == HOLE).to(dtype),
                    okf * (kind == WALL_BLOCK).to(dtype),
                    okf * (kind == TRACKED).to(dtype),
                    okf * hvx / SPEED_SCALE,
                    okf * hvy / SPEED_SCALE,
                    okf,
                ],
                dim=1,
            )
        )
    hazards = torch.cat(slots, dim=1)
    out = torch.cat([our, history, opp, rays, hazards], dim=1)
    assert out.shape[1] == OBS_DIM
    return out
