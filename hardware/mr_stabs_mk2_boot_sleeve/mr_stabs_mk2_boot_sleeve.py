"""Insulating cap over the Mk2 H1 BOOT/RST header."""

import math
from dataclasses import dataclass
from pathlib import Path

import recipe
from build123d import Align, Box, Part, Pos, Rot, import_step

PROCESS = "fdm"
MATERIAL = "petg"

HEADER_STEP = (
    Path(__file__).resolve().parent.parent
    / "mr_stabs_mk2_pcb/pcba/lib/mk2.3dshapes/HDR-SMD_3P-P2.54-V-M_HX-PZ2.54-1X3P-TP-YQ.step"
)
C_MIN = (Align.CENTER, Align.CENTER, Align.MIN)
C_MAX = (Align.CENTER, Align.CENTER, Align.MAX)
MIN_Y = (Align.CENTER, Align.MIN, Align.CENTER)


@dataclass
class Params:
    body_x: float = 7.62
    body_y: float = 2.50
    body_top: float = 3.20
    pin_tip: float = 9.20
    pitch: float = 2.54
    pocket_clear: float = 0.10
    grip: float = 2.0
    pin_hole: float = 1.30
    tip_gap: float = 0.30
    roof: float = 1.2
    wall: float = 1.5
    corner_r: float = 1.0
    foot_chamfer: float = 0.4
    mouth_chamfer: float = 0.3
    at_x: float = -21.0
    at_y: float = -9.0
    board_t: float = 1.6
    guide_y0: float = -4.337
    guide_slope: float = 0.3478
    wall_x: float = -16.3
    lip_tip_y: float = -6.268
    lip_tip_z: float = -5.552
    relief_gap: float = 0.3


def z_bottom(p: Params) -> float:
    return -(p.pin_tip + p.tip_gap + p.roof)


def z_top(p: Params) -> float:
    return -(p.body_top - p.grip)


def _chassis_relief_ops(p: Params, ox: float, oy: float) -> list[dict]:
    y0, slope = guide_line(p, p.relief_gap)
    wall_x, tip_y, _ = to_part(p, p.wall_x, p.lip_tip_y, 0)
    sketch_z_lo, sketch_z_hi, sketch_far_y = z_bottom(p) - 1.0, 0.0, oy
    guide_crosses_tip_z = (tip_y - p.relief_gap - y0) / slope
    assert sketch_z_lo < guide_crosses_tip_z < sketch_z_hi
    beside_wall = [
        (y0 + slope * sketch_z_lo, sketch_z_lo),
        (sketch_far_y, sketch_z_lo),
        (sketch_far_y, sketch_z_hi),
        (y0 + slope * sketch_z_hi, sketch_z_hi),
    ]
    behind_tip = [
        (tip_y - p.relief_gap, sketch_z_lo),
        (sketch_far_y, sketch_z_lo),
        (sketch_far_y, sketch_z_hi),
        (y0 + slope * sketch_z_hi, sketch_z_hi),
        (tip_y - p.relief_gap, guide_crosses_tip_z),
    ]
    return [
        {
            "op": "sketch",
            "name": "wall relief",
            "plane": "Right",
            "profiles": [{"polygon": beside_wall}],
        },
        {
            "op": "extrude",
            "name": "wall relief",
            "sketch": "wall relief",
            "regions": [(sketch_far_y - 0.5, sketch_z_hi - 0.5)],
            "start": wall_x - p.relief_gap,
            "depth": ox,
            "mode": "remove",
        },
        {
            "op": "sketch",
            "name": "lip relief",
            "plane": "Right",
            "profiles": [{"polygon": behind_tip}],
        },
        {
            "op": "extrude",
            "name": "lip relief",
            "sketch": "lip relief",
            "regions": [(sketch_far_y - 0.5, sketch_z_lo + 0.5)],
            "start": -ox,
            "depth": 2 * ox,
            "mode": "remove",
        },
    ]


def features(p: Params) -> list[dict]:
    """The sleeve as 10 CAD features (recipe.py), replayed by build() and by the onshape skill."""
    px, py = p.body_x + p.pocket_clear, p.body_y + p.pocket_clear
    ox, oy = px + 2 * p.wall, py + 2 * p.wall
    pins = [(x, 0.0) for x in (-p.pitch, 0.0, p.pitch)]
    ring, pocket = (0.0, (oy + py) / 4), (p.pitch / 2, 0.0)

    return [
        {
            "op": "sketch",
            "name": "outline",
            "profiles": [
                {"rect": (0, 0, ox, oy), "r": p.corner_r},
                {"rect": (0, 0, px, py)},
                *[{"circle": (x, y, p.pin_hole)} for x, y in pins],
            ],
        },
        {
            "op": "extrude",
            "name": "body",
            "sketch": "outline",
            "regions": [ring, pocket, *pins],
            "start": z_bottom(p),
            "depth": z_top(p) - z_bottom(p),
        },
        {
            "op": "chamfer",
            "name": "foot",
            "edges": [(0, -oy / 2, z_bottom(p))],
            "size": p.foot_chamfer,
        },
        {
            "op": "extrude",
            "name": "pocket",
            "sketch": "outline",
            "regions": [pocket, *pins],
            "start": z_top(p),
            "depth": p.grip,
            "flip": True,
            "mode": "remove",
        },
        {
            "op": "chamfer",
            "name": "mouth",
            "size": p.mouth_chamfer,
            "tangent": False,
            "edges": [
                (0, py / 2, z_top(p)),
                (0, -py / 2, z_top(p)),
                (px / 2, 0, z_top(p)),
                (-px / 2, 0, z_top(p)),
            ],
        },
        {
            "op": "extrude",
            "name": "pin holes",
            "sketch": "outline",
            "regions": pins,
            "start": -p.body_top,
            "depth": p.pin_tip - p.body_top + p.tip_gap,
            "flip": True,
            "mode": "remove",
        },
        *_chassis_relief_ops(p, ox, oy),
    ]


def build(p: Params) -> Part:
    return recipe.build(features(p))


def to_part(p: Params, x: float, y: float, z: float) -> tuple[float, float, float]:
    """Board frame -> this part's frame."""
    return x - p.at_x, y - p.at_y, z + p.board_t


def guide_line(p: Params, offset: float) -> tuple[float, float]:
    """The guide face moved `offset` toward the sleeve, as y = y0 + slope * z (part frame)."""
    _, y0, _ = to_part(p, 0, p.guide_y0, 0)
    tilt = math.atan(p.guide_slope)
    return y0 - p.guide_slope * p.board_t - offset / math.cos(tilt), p.guide_slope


def guide_half(p: Params, offset: float) -> Part:
    """Everything behind the chassis guide face moved `offset` toward the sleeve (part frame)."""
    y0, _ = guide_line(p, offset)
    tilt = math.degrees(math.atan(p.guide_slope))
    return Pos(0, y0, 0) * Rot(-tilt, 0, 0) * Box(80, 80, 80, align=MIN_Y)


def chassis_proxy(p: Params) -> Part:
    """The guide wall and the lip above its tip, as solids in this part's frame."""
    wall_x, _, tip_z = to_part(p, p.wall_x, 0, p.lip_tip_z)
    wall = guide_half(p, 0) & Pos(wall_x, 0, -30) * Box(
        20, 40, 30, align=(Align.MIN, Align.CENTER, Align.MIN)
    )
    lip = guide_half(p, 0) & Pos(-20, 0, tip_z) * Box(
        20 + wall_x, 40, -tip_z, align=(Align.MIN, Align.CENTER, Align.MIN)
    )
    return wall + lip


def sections(p: Params) -> list[str]:
    return ["y=0", f"x={p.pitch}", "x=0", "x=4.9"]


def header_in_part() -> Part:
    """The header STEP as KiCad places it (model offset), flipped so its pins point -Z."""
    return Rot(180, 0, 0) * Pos(-2.54, -0.413, 0.7) * import_step(HEADER_STEP)


def mates(p: Params) -> dict:
    return {
        "header": header_in_part(),
        "board": Pos(0, 0, 0) * Box(20, 12, 1.6, align=C_MIN),
        "chassis": chassis_proxy(p),
    }


def expect(p: Params) -> dict:
    return {
        "bbox": (
            p.body_x + p.pocket_clear + 2 * p.wall,
            p.body_y + p.pocket_clear + 2 * p.wall,
            p.pin_tip + p.tip_gap + p.roof - p.body_top + p.grip,
        ),
        "holes": {p.pin_hole: 3},
        "hole_at": {p.pin_hole: [{"x": x, "y": 0} for x in (-p.pitch, 0.0, p.pitch)]},
        "clearance": {"board": 1.0, "chassis": p.relief_gap},
        "chamfers_min": 8,
        "min_wall": 1.2,
    }
