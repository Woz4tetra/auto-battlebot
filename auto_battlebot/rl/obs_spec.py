"""Observation spec v1 for `RlPolicyNavigation`: 138 floats."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Annotated

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


PerWorld = Annotated[torch.Tensor, "[W]"]
PerWorldFlag = Annotated[torch.Tensor, "[W] bool"]
PerWorldVec2 = Annotated[torch.Tensor, "[W, 2]"]
PerWorldHistory = Annotated[torch.Tensor, "[W, 8]"]
PerWorldHistoryVec2 = Annotated[torch.Tensor, "[W, 8, 2]"]
PerHazard = Annotated[torch.Tensor, "[W, H]"]
PerHazardFlag = Annotated[torch.Tensor, "[W, H] bool"]
PerHazardKind = Annotated[torch.Tensor, "[W, H] long: HOLE, WALL_BLOCK, TRACKED"]
PerHazardVec2 = Annotated[torch.Tensor, "[W, H, 2]"]


@dataclass
class ObsInputs:
    """Everything the builder reads, batched over worlds, field frame unless noted."""

    our_xy: PerWorldVec2
    our_yaw: PerWorld
    our_speed: PerWorld
    our_yaw_rate: PerWorld
    our_age_s: PerWorld
    our_stale: PerWorldFlag
    run_away: PerWorldFlag
    hist_action: PerWorldHistoryVec2
    hist_speed: PerWorldHistory
    hist_yaw_rate: PerWorldHistory
    opp_present: PerWorldFlag
    opp_xy: PerWorldVec2
    opp_yaw: PerWorld
    opp_velocity: PerWorldVec2
    opp_yaw_rate: PerWorld
    opp_position_sigma: PerWorld
    opp_heading_sigma: PerWorld
    opp_yaw_rate_sigma: PerWorld
    opp_half_diagonal: PerWorld
    opp_age_s: PerWorld
    opp_stale: PerWorldFlag
    field_half: PerWorldVec2
    hazard_xy: PerHazardVec2
    hazard_radius: PerHazard
    hazard_kind: PerHazardKind
    hazard_velocity: PerHazardVec2
    hazard_present: PerHazardFlag


def wrap(a: torch.Tensor) -> torch.Tensor:
    return torch.remainder(a + math.pi, 2.0 * math.pi) - math.pi


def ray_angles(dtype: torch.dtype, device: torch.device) -> torch.Tensor:
    return torch.arange(NUM_RAYS, dtype=dtype, device=device) * (2.0 * math.pi / NUM_RAYS)


def ray_box(origin: torch.Tensor, direction: torch.Tensor, half: torch.Tensor) -> torch.Tensor:
    """Distance from `origin` along `direction` to the wall of the box `half`."""
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
    """Nearest hit of each ray on any masked disc."""
    m = origin[:, None, :] - centre
    b = (m[:, None, :, :] * direction[:, :, None, :]).sum(-1)
    c = (m * m).sum(-1) - radius * radius
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
    direction = torch.stack([torch.cos(angles), torch.sin(angles)], dim=-1)
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
