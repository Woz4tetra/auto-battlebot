"""ZED Box Mini to INNOREL CP10 cheese plate adapter."""

from dataclasses import dataclass

import recipe
from build123d import Align, Box, Cylinder, Part, Pos

PROCESS = "fdm"
MATERIAL = "pla"

INCH = 25.4
C_MIN = (Align.CENTER, Align.CENTER, Align.MIN)

PLATE_L, PLATE_W, PLATE_T = 200.0, 100.0, 10.0
ZED_BODY_L, ZED_BODY_W, ZED_BODY_H = 143.0, 86.0, 41.5
ZED_SHEET_T = 1.0
ZED_EAR_L, ZED_EAR_W = 60.0, 103.8
ZED_EAR_HOLE = 3.6
FBHCS_14_HEAD_D, FBHCS_14_HEAD_H = 0.560 * INCH, 0.132 * INCH
FBHCS_14_LEN = 0.5 * INCH
SHCS_M3_HEAD_D, SHCS_M3_HEAD_H, SHCS_M3_LEN = 5.5, 3.0, 6.0


@dataclass
class Params:
    length: float = 143.0
    width: float = 103.8
    thickness: float = 6.7
    corner_r: float = 5.0
    bed_chamfer: float = 0.5

    zed_hole_x: float = 24.0
    zed_hole_y: float = 47.3
    insert_hole: float = 4.2
    insert_len: float = 6.0
    insert_od: float = 5.0

    plate_hole_x: float = 49.5
    plate_hole_y: float = 30.85
    bolt_clear: float = 7.0
    cbore_d: float = 16.0
    cbore_depth: float = 4.0


def features(p: Params) -> list[dict]:
    """The adapter as 4 CAD features (recipe.py)."""
    holes = [(sx * p.zed_hole_x, sy * p.zed_hole_y) for sx in (-1, 1) for sy in (-1, 1)]
    bolts = [(sx * p.plate_hole_x, sy * p.plate_hole_y) for sx in (-1, 1) for sy in (-1, 1)]
    cbore_floor_mid_r = (p.bolt_clear + p.cbore_d) / 4
    return [
        {
            "op": "sketch",
            "name": "outline",
            "profiles": [{"rect": (0, 0, p.length, p.width), "r": p.corner_r}]
            + [{"circle": (x, y, p.insert_hole)} for x, y in holes]
            + [{"circle": (x, y, p.bolt_clear)} for x, y in bolts]
            + [{"circle": (x, y, p.cbore_d)} for x, y in bolts],
        },
        {"op": "extrude", "sketch": "outline", "regions": [(0, 0)], "depth": p.thickness},
        {
            "op": "extrude",
            "sketch": "outline",
            "regions": [(x + cbore_floor_mid_r, y) for x, y in bolts],
            "depth": p.thickness - p.cbore_depth,
            "mode": "add",
        },
        {"op": "chamfer", "edges": [(0, -p.width / 2, 0)], "size": p.bed_chamfer},
    ]


def build(p: Params) -> Part:
    return recipe.build(features(p))


def sections(p: Params) -> list[str]:
    """Through one insert column and one bolt column, and across both bolt rows."""
    return [f"x={p.zed_hole_x}", f"x={p.plate_hole_x}", f"y={p.plate_hole_y}", f"y={p.zed_hole_y}"]


def mates(p: Params) -> dict:
    top = p.thickness
    plate = Pos(0, 0, 0) * Box(
        PLATE_L, PLATE_W, PLATE_T, align=(Align.CENTER, Align.CENTER, Align.MAX)
    )

    zed = Pos(0, 0, top) * Box(ZED_EAR_L, ZED_EAR_W, ZED_SHEET_T, align=C_MIN)
    zed += Pos(0, 0, top) * Box(ZED_BODY_L, ZED_BODY_W, ZED_BODY_H, align=C_MIN)

    bolts = None
    m3 = None
    inserts = None
    for sx in (-1, 1):
        for sy in (-1, 1):
            zxy = Pos(sx * p.zed_hole_x, sy * p.zed_hole_y, 0)
            zed -= zxy * Pos(0, 0, top) * Cylinder(ZED_EAR_HOLE / 2, ZED_SHEET_T, align=C_MIN)
            s = (
                zxy
                * Pos(0, 0, top + ZED_SHEET_T)
                * (
                    Cylinder(SHCS_M3_HEAD_D / 2, SHCS_M3_HEAD_H, align=C_MIN)
                    + Cylinder(1.5, SHCS_M3_LEN, align=(Align.CENTER, Align.CENTER, Align.MAX))
                )
            )
            m3 = s if m3 is None else m3 + s
            ins = (
                zxy
                * Pos(0, 0, top)
                * Cylinder(
                    p.insert_od / 2,
                    p.insert_len,
                    align=(Align.CENTER, Align.CENTER, Align.MAX),
                )
            )
            inserts = ins if inserts is None else inserts + ins
            b = Pos(sx * p.plate_hole_x, sy * p.plate_hole_y, top - p.cbore_depth) * (
                Cylinder(FBHCS_14_HEAD_D / 2, FBHCS_14_HEAD_H, align=C_MIN)
                + Cylinder(
                    0.25 * INCH / 2, FBHCS_14_LEN, align=(Align.CENTER, Align.CENTER, Align.MAX)
                )
            )
            bolts = b if bolts is None else bolts + b
    return {
        "cheese_plate": plate,
        "zed_box_mini": zed,
        "m3_shcs": m3,
        "m3_insert": inserts,
        "fbhcs_1_4_20": bolts,
    }


def expect(p: Params) -> dict:
    return {
        "bbox": (143.0, 103.8, 6.7),
        "holes": {p.insert_hole: 4, p.bolt_clear: 4, p.cbore_d: 4},
        "hole_at": {
            4.2: [{"x": sx * 24.0, "y": sy * 47.3} for sx in (-1, 1) for sy in (-1, 1)],
            7.0: [{"x": sx * 49.5, "y": sy * 30.85} for sx in (-1, 1) for sy in (-1, 1)],
        },
        "allow_interference": ["m3_insert"],
        "flush": {"fbhcs_1_4_20": "+z"},
        "gap": {"fbhcs_1_4_20": 0.2},
    }


if __name__ == "__main__":
    print(build(Params()).bounding_box())
