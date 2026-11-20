"""Battery clamp plate that holds the cells and carries the OpenUPS."""

from dataclasses import dataclass

import cage_layout as cl
import recipe
from build123d import Part

PROCESS = "fdm"
MATERIAL = "petg"


@dataclass
class Params:
    m3_clearance_dia: float = 3.4
    margin: float = 6.0
    corner: float = 5.0
    tube_d: float = cl.BOSS_D
    foot_chamfer: float = 0.5


def features(p: Params) -> list[dict]:
    """Plate on the OpenUPS hole pattern, then a standoff tube round each hole."""
    xs = [x for x, _ in cl.UPS_HOLES]
    ys = [y for _, y in cl.UPS_HOLES]
    w = max(xs) - min(xs) + 2 * p.margin
    h = max(ys) - min(ys) + 2 * p.margin
    xc, yc = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
    zt = cl.CLAMP_Z0 + cl.CLAMP_T
    holes = [{"circle": (x, y, p.m3_clearance_dia)} for x, y in cl.UPS_HOLES]
    ring_midpoints = [(x + (p.m3_clearance_dia + p.tube_d) / 4, y) for x, y in cl.UPS_HOLES]
    return [
        {"op": "sketch", "name": "Plate outline", "plane": "Top", "profiles": [
            {"rect": (xc, yc, w, h), "r": p.corner}, *holes]},
        {"op": "extrude", "name": "Plate", "sketch": "Plate outline", "regions": [(xc, yc)],
         "depth": cl.CLAMP_T, "start": cl.CLAMP_Z0},
        {"op": "sketch", "name": "Tubes", "plane": "Top", "profiles": [
            *[{"circle": (x, y, p.tube_d)} for x, y in cl.UPS_HOLES], *holes]},
        {"op": "extrude", "name": "Standoff tubes", "sketch": "Tubes", "regions": ring_midpoints,
         "depth": cl.UPS_TUBE_H, "start": zt, "mode": "add"},
    ]  # fmt: skip


def build(p: Params) -> Part:
    return recipe.build(features(p))


def sections(p: Params) -> list[str]:
    return [f"y={cl.UPS_HOLES[0][1]}"]


def expect(p: Params) -> dict:
    return {
        "bbox": (92.0 + 2 * p.margin, 47.0 + 2 * p.margin, cl.CLAMP_T + cl.UPS_TUBE_H),
        "holes": {p.m3_clearance_dia: 4},
    }
