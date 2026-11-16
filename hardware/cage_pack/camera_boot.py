from dataclasses import dataclass

import b123d_fix  # noqa: F401
import cage_layout as cl
import enclosure
from build123d import Align, Box, Cylinder, Part, Plane, Polygon, Pos, Rot, extrude

PROCESS = "fdm"
MATERIAL = "petg"
C_MIN = (Align.CENTER, Align.CENTER, Align.MIN)
C_MAX = (Align.CENTER, Align.CENTER, Align.MAX)


@dataclass
class Params:
    wall: float = cl.BOOT_WALL
    tab: float = 9.0
    tab_reach: float = 11.0
    quarter_insert_hole: float = 8.0
    quarter_insert_depth: float = 8.6
    insert_hole: float = 4.0
    insert_depth: float = 6.7
    floor_top_z: float = 70.0
    window_clear: float = 0.5


def _box(x0, x1, y0, y1, z0, z1) -> Part:
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, z0) * Box(x1 - x0, y1 - y0, z1 - z0, align=C_MIN)


def _gusset(x0: float, x1: float, y_root: float, sy: int, reach: float, depth: float) -> Part:
    z1 = cl.PANEL_Z0
    pts = [
        (y_root, z1),
        (y_root + sy * reach, z1),
        (y_root + sy * reach, z1 - depth),
        (y_root, z1 - depth - reach),
    ]
    g = extrude(Plane.YZ * Polygon(*pts, align=None), amount=x1 - x0)
    return Pos(x0 - g.bounding_box().min.X, 0, 0) * g


def build(p: Params) -> Part:
    w = p.wall
    ix0, ix1 = cl.BOOT_IN_X
    iy1 = cl.BOOT_IN_Y1
    fy1 = cl.CAM_FLOOR_Y
    fy0 = fy1 - cl.BOOT_FLOOR_T
    z0, zp = cl.BOOT_RIM_Z, cl.PANEL_Z0
    x0, x1 = ix0 - w, ix1 + w
    part = _box(x0, ix0, fy0, iy1 + w, z0, zp)
    part += _box(ix1, x1, fy0, iy1 + w, z0, zp)
    part += _box(x0, x1, iy1, iy1 + w, z0, zp)
    part += _box(x0, x1, fy0, fy1, z0, zp)
    wx0, wx1 = cl.WINDOW[0] + 4.0, cl.WINDOW[1] - 4.0
    part += _box(wx0, wx1, fy0, fy1, zp - 1.0, p.floor_top_z)
    part += _gusset(x0, x1, fy0, -1, p.tab_reach, p.tab)
    part += _gusset(x0, x1, iy1 + w, 1, p.tab_reach, p.tab)
    part -= (
        Pos(cl.CAM_XC, fy1, cl.CAM_HEAD_Z)
        * Rot(-90, 0, 0)
        * Cylinder(p.quarter_insert_hole / 2, p.quarter_insert_depth, align=C_MAX)
    )
    for x, y in enclosure.boot_holes():
        part -= Pos(x, y, zp) * Cylinder(p.insert_hole / 2, p.insert_depth, align=C_MAX)
    return part


def sections(p: Params) -> list[str]:
    return [f"x={cl.CAM_XC}", f"y={cl.CAM_FLOOR_Y - 4.5}"]


def expect(p: Params) -> dict:
    return {"holes": {p.quarter_insert_hole: 1, p.insert_hole: 4}}
