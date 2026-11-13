"""Camera boot: a black PETG hood between the tray panel and the glass around the camera.

The ZED X One S hangs in it on a SmallRig 2948 ball head. The boot keeps stray light off the
glass in front of the lens, carries the ball head on its floor, and stands 3 mm proud of
the lens, so a dropped pack lands on the boot's rim before the lens.

Spec
- Process: FDM PETG, black. Printed rim-down (the glass-side face on the bed), so the walls
  rise straight up and the two screw tabs are 45° gussets that print without support.
  `build()` returns it in the assembly frame, which is also the print frame.
- Frame: cage_layout.py.
- Interfaces:
  - Tray panel: the walls' top faces and the tabs sit on the panel's glass-side face
    (Z = CUP_H); 4 x M3 x 8 button heads from inside the tray into M3 heat-set inserts
    (Ø4.0 x 6.7) in the tabs at enclosure.boot_holes().
  - Ball head: its floor's top face is the ball head's seat. A 1/4-20 x 1/2" set screw stud
    goes 6.35 into a McMaster 93365A160 brass insert in the floor and 6.35 into the head's
    bottom thread. The floor passes up through the panel window to Z = 70, past the top of
    the head's Ø23 base.
  - Glass: the rim stops 3 mm short; 3 mm black EPDM foam tape on the rim seals it.
- Critical dimensions: inside 58 x 80 (Y, floor to top wall); lens 6 mm off the glass, so
  3 mm behind the rim; floor 10.6 thick, 2 mm under the insert.
"""

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
    tab: float = 9.0  # tab depth along Z at the panel; its 45° face runs down from there
    tab_reach: float = 11.0  # beyond the floor's underside / the top wall's outside
    quarter_insert_hole: float = 8.0  # 93365A160, 0.300" long
    quarter_insert_depth: float = 8.6
    insert_hole: float = 4.0  # M3 heat-set, fasteners.md
    insert_depth: float = 6.7
    floor_top_z: float = 70.0
    window_clear: float = 0.5


def _box(x0, x1, y0, y1, z0, z1) -> Part:
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, z0) * Box(x1 - x0, y1 - y0, z1 - z0, align=C_MIN)


def _gusset(x0: float, x1: float, y_root: float, sy: int, reach: float, depth: float) -> Part:
    """A screw tab: `depth` thick at its outer end, its underside rising back to the wall
    at 45° so it prints rim-down without support."""
    z1 = cl.PANEL_Z0
    pts = [
        (y_root, z1),
        (y_root + sy * reach, z1),
        (y_root + sy * reach, z1 - depth),
        (y_root, z1 - depth - reach),
    ]
    g = extrude(Plane.YZ * Polygon(*pts, align=None), amount=x1 - x0)
    # The extrude direction follows the polygon's winding, so place it by its bounding box.
    return Pos(x0 - g.bounding_box().min.X, 0, 0) * g


def build(p: Params) -> Part:
    w = p.wall
    ix0, ix1 = cl.BOOT_IN_X
    iy1 = cl.BOOT_IN_Y1
    fy1 = cl.CAM_FLOOR_Y
    fy0 = fy1 - cl.BOOT_FLOOR_T
    z0, zp = cl.BOOT_RIM_Z, cl.PANEL_Z0
    x0, x1 = ix0 - w, ix1 + w
    part = _box(x0, ix0, fy0, iy1 + w, z0, zp)  # -X wall
    part += _box(ix1, x1, fy0, iy1 + w, z0, zp)  # +X wall
    part += _box(x0, x1, iy1, iy1 + w, z0, zp)  # top wall
    part += _box(x0, x1, fy0, fy1, z0, zp)  # floor, below the panel
    # Inset past the window's 3 mm corner radii.
    wx0, wx1 = cl.WINDOW[0] + 4.0, cl.WINDOW[1] - 4.0
    part += _box(wx0, wx1, fy0, fy1, zp - 1.0, p.floor_top_z)  # floor, up through the window
    part += _gusset(x0, x1, fy0, -1, p.tab_reach, p.tab)
    part += _gusset(x0, x1, iy1 + w, 1, p.tab_reach, p.tab)
    # Ball head insert, axis +Y, in the floor's top face under the head's axis.
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
