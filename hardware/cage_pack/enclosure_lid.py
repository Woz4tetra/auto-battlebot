"""Cage pack enclosure lid, printed in black PETG."""

from dataclasses import dataclass
from typing import NamedTuple

import b123d_fix  # noqa: F401
import cage_layout as cl
import enclosure
import recipe
import regions
from build123d import Part, Pos, Rectangle, RectangleRounded, Rot

PROCESS = "fdm"
MATERIAL = "petg"
LID_TOP_Z = cl.LID_UNDERSIDE_Z + cl.LID_T
PRINT_TO_ASSEMBLY = Pos(0, 0, LID_TOP_Z) * Rot(180, 0, 0)


class Rim(NamedTuple):
    width: float
    height: float


@dataclass
class Params:
    m3_clearance_dia: float = 3.4
    vent: tuple = (3.0, 30.0)
    vent_pitch: float = 9.0
    notch_clear: float = 0.5
    elephant_foot_chamfer: float = 0.5
    skin_t: float = 1.2
    pocket_land_w: float = 3.0
    window_margin_per_side: float = 0.4
    window_r: float = 1.0
    window_chamfer_45: float = 0.8
    glass_slip_clear: float = 0.3
    rim: Rim = Rim(1.2, 1.0)
    boss_d: float = 8.0
    insert_hole: float = cl.INSERT_HOLE


def _zp(z: float) -> float:
    """Assembly Z to print Z: the outside face (LID_TOP_Z) is the bed, Z = 0."""
    return LID_TOP_Z - z


def _pocket_sketch(p: Params):
    """Lid pockets between lands."""
    m = p.pocket_land_w
    lip_inner_offset = cl.LID_LIP_WALL_GAP + cl.LID_LIP_T + 1.0
    xs = [cl.X_IN[0] + lip_inner_offset, -56.0, 4.5, 48.5, cl.X_IN[1] - lip_inner_offset]
    ys = [cl.Y_IN[0] + lip_inner_offset, -116.0, cl.ZED_YC, cl.Y_IN[1] - lip_inner_offset]
    o = cl.LID_BOSS_SIZE / 2
    boxes = [(x - o, x + o, y - o, y + o) for x, y in enclosure.lid_screws()]
    gx0, gx1 = cl.DISP_GLASS_X_FROM_TOUCH
    ey = max(abs(y) for _, y in cl.DISP_EARS) + p.boss_d / 2
    boxes.append(
        (
            cl.DISP_TOUCH_X + gx0 - 2.0,
            cl.DISP_TOUCH_X + gx1 + 2.0,
            cl.DISP_TOUCH_Y - ey,
            cl.DISP_TOUCH_Y + ey,
        )
    )
    return enclosure.pocket_sketch(xs, ys, m, (), boxes, ysign=-1.0)


def features(p: Params) -> list[dict]:
    """The lid as CAD features in the print frame."""
    t = cl.LID_T
    xc, yc = (cl.X_OUT[0] + cl.X_OUT[1]) / 2, -(cl.Y_OUT[0] + cl.Y_OUT[1]) / 2
    w, h = cl.X_OUT[1] - cl.X_OUT[0], cl.Y_OUT[1] - cl.Y_OUT[0]
    g, lt = cl.LID_LIP_WALL_GAP, cl.LID_LIP_T
    lx0, lx1 = cl.X_IN[0] + g, cl.X_IN[1] - g
    ly0, ly1 = cl.Y_IN[0] + g, cl.Y_IN[1] - g
    lr = max(cl.OUTER_CORNER_R - cl.WALL_T - g, 0.5)
    lxc, lyc = (lx0 + lx1) / 2, -(ly0 + ly1) / 2
    n = cl.LID_BOSS_SIZE + 2 * p.notch_clear
    screws = [(x, -y) for x, y in enclosure.lid_screws()]
    lip_face = Pos(lxc, lyc) * RectangleRounded(lx1 - lx0, ly1 - ly0, lr)
    lip_face -= Pos(lxc, lyc) * Rectangle(lx1 - lx0 - 2 * lt, ly1 - ly0 - 2 * lt)
    for x, y in screws:
        lip_face -= Pos(x, y) * Rectangle(n, n)
    pocket_profiles, pocket_pts = _pocket_sketch(p)
    x, y = cl.DISP_TOUCH_X, -cl.DISP_TOUCH_Y
    vx, vy = (v + 2 * p.window_margin_per_side for v in cl.DISP_VA)
    gx0, gx1 = cl.DISP_GLASS_X_FROM_TOUCH
    gy = cl.DISP_GLASS_SIZE_XY[1] / 2
    rw, rh = p.rim.width, p.rim.height
    c = p.glass_slip_clear
    gxc, gw = x + (gx0 + gx1) / 2, gx1 - gx0
    boss_h = cl.LID_UNDERSIDE_Z - cl.DISP_BACK_Z
    ears = [(x + dx, y - dy) for dx, dy in cl.DISP_EARS]
    vents = [(cl.ZED_XC - 45.0 + i * p.vent_pitch, -(cl.ZED_YC + s)) for i in range(11)
             for s in (-20.0, 20.0)]  # fmt: skip
    return [
        {"op": "sketch", "name": "Plate outline", "plane": "Top", "profiles": [
            {"rect": (xc, yc, w, h), "r": cl.OUTER_CORNER_R}]},
        {"op": "extrude", "name": "Plate", "sketch": "Plate outline", "regions": [(xc, yc)],
         "depth": t},
        {"op": "chamfer", "name": "Elephant's foot chamfer", "size": p.elephant_foot_chamfer,
         "edges": [(xc, yc - h / 2, 0.0)]},
        {"op": "sketch", "name": "Lip", "plane": "Top", "profiles": [
            {"rect": (lxc, lyc, lx1 - lx0, ly1 - ly0), "r": lr},
            {"rect": (lxc, lyc, lx1 - lx0 - 2 * lt, ly1 - ly0 - 2 * lt)},
            *[{"rect": (sx, sy, n, n)} for sx, sy in screws]]},
        {"op": "extrude", "name": "Lip", "sketch": "Lip", "regions": regions.points(lip_face),
         "depth": cl.LID_LIP_DEPTH, "start": t, "mode": "add"},
        {"op": "sketch", "name": "Pockets", "plane": "Top", "profiles": pocket_profiles},
        {"op": "extrude", "name": "Pockets", "sketch": "Pockets", "regions": pocket_pts,
         "depth": t - p.skin_t, "start": p.skin_t, "mode": "remove"},
        {"op": "sketch", "name": "Display window", "plane": "Top", "profiles": [
            {"rect": (x, y, vx, vy), "r": p.window_r}]},
        {"op": "extrude", "name": "Display window", "sketch": "Display window",
         "regions": [(x, y)], "depth": t, "mode": "remove"},
        {"op": "chamfer", "name": "Window chamfer", "size": p.window_chamfer_45,
         "edges": [(x, y - vy / 2, 0.0)]},
        {"op": "sketch", "name": "Glass rim", "plane": "Top", "profiles": [
            {"rect": (gxc, y, gw + 2 * (c + rw), 2 * (gy + c + rw)), "r": 2.0 + c + rw},
            {"rect": (gxc, y, gw + 2 * c, 2 * (gy + c)), "r": 2.0 + c}]},
        {"op": "extrude", "name": "Glass rim", "sketch": "Glass rim",
         "regions": [(gxc, y + gy + c + rw / 2)], "depth": rh, "start": t, "mode": "add"},
        {"op": "sketch", "name": "Display bosses", "plane": "Top", "profiles": [
            *[{"circle": (ex, ey, p.boss_d)} for ex, ey in ears],
            *[{"circle": (ex, ey, p.insert_hole)} for ex, ey in ears]]},
        {"op": "extrude", "name": "Display bosses", "sketch": "Display bosses",
         "regions": [(ex + (p.boss_d + p.insert_hole) / 4, ey) for ex, ey in ears] + ears,
         "depth": boss_h, "start": t, "mode": "add"},
        {"op": "extrude", "name": "Insert holes", "sketch": "Display bosses", "regions": ears,
         "depth": cl.INSERT_DEPTH, "start": t + boss_h - cl.INSERT_DEPTH, "mode": "remove"},
        {"op": "sketch", "name": "Screw holes and vents", "plane": "Top", "profiles": [
            *[{"circle": (sx, sy, p.m3_clearance_dia)} for sx, sy in screws],
            *[{"rect": (vx_, vy_, p.vent[0], p.vent[1]), "r": p.vent[0] / 2 - 0.01}
              for vx_, vy_ in vents]]},
        {"op": "extrude", "name": "Screw holes and vents", "sketch": "Screw holes and vents",
         "regions": screws + vents, "depth": t, "mode": "remove"},
    ]  # fmt: skip


def build(p: Params) -> Part:
    return recipe.build(features(p))


def sections(p: Params) -> list[str]:
    x, _ = enclosure.lid_screws()[0]
    ex = cl.DISP_TOUCH_X + cl.DISP_EARS[0][0]
    return [f"x={x}", f"y={-cl.ZED_YC}", f"x={ex}", f"y={-cl.DISP_TOUCH_Y}"]


def expect(p: Params) -> dict:
    return {
        "bbox": (
            cl.X_OUT[1] - cl.X_OUT[0],
            cl.Y_OUT[1] - cl.Y_OUT[0],
            cl.LID_T + cl.LID_UNDERSIDE_Z - cl.DISP_BACK_Z,
        ),
        "holes": {p.m3_clearance_dia: 6, p.insert_hole: 4},
        "mass_g": (None, 250),
    }
