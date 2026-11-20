"""Camera light shield hood on the +X wall."""

from dataclasses import dataclass

import cage_layout as cl
import recipe
from build123d import Part, Pos, Rot

PROCESS = "fdm"
MATERIAL = "petg"
PRINT_TO_ASSEMBLY = Pos(cl.X_OUT[1], 0, 0) * Rot(0, 90, 0)


@dataclass
class Params:
    t: float = cl.SHIELD_T
    boss_d_under_head_base: float = 22.0
    quarter_close_clearance_dia: float = 6.53
    m3_clearance_dia: float = 3.4
    flange_below_axis: float = 16.0
    rib_t: float = 2.4
    rib_tip_h: float = 3.0
    corner_r: float = 4.0


def _extent(p: Params) -> tuple[float, float, float, float, float, float]:
    x0 = cl.X_OUT[1]
    x1 = cl.HEAD_SEAT_X + cl.SHIELD_REACH
    y0, y1 = cl.SHIELD_Y
    zh0 = cl.SHIELD_HOOD_Z0
    return x0, x1, y0, y1, zh0, zh0 + p.t


def keep_out(p: Params | None = None) -> dict[str, tuple[tuple, tuple]]:
    """Boxes round the shield's pieces, for camera_range.py."""
    p = p or Params()
    x0, x1, y0, y1, zh0, zh1 = _extent(p)
    return {
        "shield flange": ((x0, y0, cl.CAM_AXIS_Z - p.flange_below_axis), (x0 + p.t, y1, zh1)),
        "shield hood": ((x0, y0, zh0), (x1, y1, zh1 + cl.SHIELD_RIB)),
        "shield boss": (
            (
                x0,
                cl.CAM_Y - p.boss_d_under_head_base / 2,
                cl.CAM_AXIS_Z - p.boss_d_under_head_base / 2,
            ),
            (
                cl.HEAD_SEAT_X,
                cl.CAM_Y + p.boss_d_under_head_base / 2,
                cl.CAM_AXIS_Z + p.boss_d_under_head_base / 2,
            ),
        ),
    }


def _print(xa: float, za: float) -> tuple[float, float]:
    """An assembly-frame (X, Z) point in the print frame's (X, Z): PRINT_TO_ASSEMBLY inverted."""
    return -za, xa - cl.X_OUT[1]


def _rib_profile(p: Params, xf: float, x1: float, zh1: float) -> list[tuple[float, float]]:
    """Rib outline, SHIELD_RIB tall at the flange and rib_tip_h at its end."""
    return [
        _print(xf, zh1),
        _print(x1 - p.corner_r, zh1),
        _print(x1 - p.corner_r, zh1 + p.rib_tip_h),
        _print(xf, zh1 + cl.SHIELD_RIB),
    ]


def features(p: Params) -> list[dict]:
    """Shield features in the print frame."""
    x0, x1, y0, y1, zh0, zh1 = _extent(p)
    t, yc = p.t, cl.CAM_Y
    xf = x0 + t
    fx0, _ = _print(0, zh1 + cl.SHIELD_RIB)
    fx1, _ = _print(0, cl.CAM_AXIS_Z - p.flange_below_axis)
    bx, _ = _print(0, cl.CAM_AXIS_Z)
    sx, _ = _print(0, cl.SHIELD_SCREW_Z)
    boss_top = cl.HEAD_SEAT_X - x0
    reach = x1 - x0
    hx0, hx1 = _print(0, zh1)[0], _print(0, zh0)[0]
    ring_midpoint = (bx + (p.quarter_close_clearance_dia + p.boss_d_under_head_base) / 4, yc)
    flange_pick_point = ((fx0 + fx1) / 2, y0 + 1.0)
    rib = _rib_profile(p, xf, x1, zh1)
    rib_pt = ((rib[0][0] + rib[3][0]) / 2 - 1.0, (rib[0][1] + rib[1][1]) / 2)
    rib_y0s = [y0, (y0 + y1 - p.rib_t) / 2, y1 - p.rib_t]
    return [
        {"op": "sketch", "name": "Flange", "plane": "Top", "profiles": [
            {"rect": ((fx0 + fx1) / 2, (y0 + y1) / 2, fx1 - fx0, y1 - y0)},
            {"circle": (bx, yc, p.boss_d_under_head_base)},
            {"circle": (bx, yc, p.quarter_close_clearance_dia)},
            {"circle": (sx, yc, p.m3_clearance_dia)},
        ]},
        {"op": "extrude", "name": "Flange", "sketch": "Flange",
         "regions": [flange_pick_point, ring_midpoint], "depth": t},
        {"op": "extrude", "name": "Spacer boss", "sketch": "Flange", "regions": [ring_midpoint],
         "depth": boss_top - t, "start": t, "mode": "add"},
        {"op": "sketch", "name": "Hood", "plane": "Right", "profiles": [
            {"rect": ((y0 + y1) / 2, reach / 2, y1 - y0, reach)}]},
        {"op": "extrude", "name": "Hood", "sketch": "Hood", "regions": [((y0 + y1) / 2, reach / 2)],
         "depth": t, "start": hx0, "mode": "add"},
        {"op": "fillet", "name": "Hood corners", "size": p.corner_r, "tangent": False,
         "edges": [((hx0 + hx1) / 2, y, reach) for y in (y0, y1)]},
        {"op": "sketch", "name": "Rib", "plane": "Front", "profiles": [{"polygon": rib}]},
        *[{"op": "extrude", "name": f"Rib {i + 1}", "sketch": "Rib", "regions": [rib_pt],
           "depth": p.rib_t, "start": -(y + p.rib_t), "mode": "add"}
           for i, y in enumerate(rib_y0s)],
    ]  # fmt: skip


def build(p: Params) -> Part:
    return recipe.build(features(p))


def sections(p: Params) -> list[str]:
    return [f"y={cl.CAM_Y}"]


def expect(p: Params) -> dict:
    y0, y1 = cl.SHIELD_Y
    return {
        "holes": {p.quarter_close_clearance_dia: 1, p.m3_clearance_dia: 1},
        "bbox": (None, y1 - y0, cl.HEAD_SEAT_X + cl.SHIELD_REACH - cl.X_OUT[1]),
    }
