"""Cage pack enclosure lid: a flat black PETG plate over the tray's open front.

Spec
- Process: FDM, black PETG, printed outside face down on a Bambu P1S, the lip rising from
  it. `build()` returns it in that print frame: outside face on Z = 0, turned over about X.
  `ASSEMBLY_AT` puts it back on the tray. PROCESS = "fdm".
- Frame: cage_layout.py. Underside on the tray walls' edges (Z = WALL_EDGE_Z); outline equal
  to the tray's, so it fits the same 256 x 256 plate.
- Interfaces:
  - 6 x Ø3.4 over the tray's lid bosses (enclosure.lid_screws()), M3 x 8 button heads into
    M3 x 6.4 heat-set inserts.
  - Lip: 1.6 thick, 5 deep, 0.3 inside the walls. It locates the lid and closes the light
    path at the joint; notched 0.5 clear round each boss.
- Vents: 3 x 30 slots over the ZED Box Mini's top face, which carries its fan intake.
- Touch display (cage_layout.DISP_*): the Waveshare 2.8" module's glass sits on the
  underside, landscape, in the lower right, clear of the ZED Box Mini and the OpenUPS's
  plugs. The lid is its bezel: a window 0.4 larger than the 58.2 x 43.8 touch area each
  side, R1 corners, a 0.8 chamfer on the outside edge for a fingertip. A 1.0 tall, 1.2
  wide rim 0.3 round the glass locates it. Four Ø8 bosses hang to the display's
  standoff plane with M3 x 6.4 inserts for display_bracket.py, which clamps the glass up
  against the lid. The plate round the window is left full thickness (no pockets).
- Weight: pocketed from the inside to a 1.2 mm skin between 3 mm lands, a land round the
  lip's inside and each screw boss; print_mass.py slices it at 86 g (143 g unpocketed).
  Printed face down, so the pockets face up and need no support.
- Critical dimensions: outline 250.8 x 213.8 with R4 corners; lip outside 245.4 x 208.4;
  display bosses on cage_layout.DISP_EARS, their end faces in the standoffs' plane.
- Model: features() (recipe.py) in the print frame, so the checked solid and the Onshape
  Part Studio are the same 17 features, in print order: plate, foot chamfer, lip, pockets,
  display window and its chamfer, glass rim, display bosses and insert holes, then the
  screw holes and vents.
"""

from dataclasses import dataclass

import b123d_fix  # noqa: F401
import cage_layout as cl
import enclosure
import recipe  # build123d-part/scripts, on the path when run through scripts/run
import regions
from build123d import Part, Pos, Rectangle, RectangleRounded, Rot

PROCESS = "fdm"
MATERIAL = "petg"
LID_TOP_Z = cl.WALL_EDGE_Z + cl.LID_T
ASSEMBLY_AT = Pos(0, 0, LID_TOP_Z) * Rot(180, 0, 0)  # print frame -> assembly frame


@dataclass
class Params:
    clear: float = 3.4  # M3 normal clearance, fasteners.md
    vent: tuple = (3.0, 30.0)
    vent_pitch: float = 9.0
    notch_clear: float = 0.5
    foot_chamfer: float = 0.5  # elephant's foot on the outside face, fdm.md
    skin: float = 1.2  # plate left over the pockets: 6 layers, all solid
    pocket_land: float = 3.0  # full-thickness plate kept along lines and round the screws
    window_margin: float = 0.4  # display window past the touch area, each side
    window_r: float = 1.0
    window_chamfer: float = 0.8  # outside edge; 45°, prints face down
    glass_clear: float = 0.3  # locating rim to the glass, fdm.md slip fit
    rim: tuple = (1.2, 1.0)  # display rim: width, height
    boss_d: float = 8.0  # fasteners.md: M3 insert needs >= 7.0
    insert_hole: float = cl.INSERT_HOLE  # 94180A333 in plastic


def _zp(z: float) -> float:
    """Assembly Z to print Z: the outside face (LID_TOP_Z) is the bed, Z = 0."""
    return LID_TOP_Z - z


def _pocket_sketch(p: Params):
    """Plate thinned to p.skin from the inside between 3 mm lands, on the tray panel's
    grid except x = 4.5, which falls between two vent slots where the spine's x = 0 would
    cut through one; a land runs round the inside of the lip, each screw boss and the
    display."""
    m = p.pocket_land
    e = cl.LID_GAP + cl.LID_LIP_T + 1.0  # lip's inside face, plus 1
    xs = [cl.X_IN[0] + e, -56.0, 4.5, 48.5, cl.X_IN[1] - e]
    ys = [cl.Y_IN[0] + e, -116.0, cl.ZED_YC, cl.Y_IN[1] - e]
    o = cl.LID_BOSS / 2
    boxes = [(x - o, x + o, y - o, y + o) for x, y in enclosure.lid_screws()]
    gx0, gx1 = cl.DISP_GLASS_X
    ey = max(abs(y) for _, y in cl.DISP_EARS) + p.boss_d / 2
    boxes.append((cl.DISP_X + gx0 - 2.0, cl.DISP_X + gx1 + 2.0, cl.DISP_Y - ey, cl.DISP_Y + ey))
    return enclosure.pocket_sketch(xs, ys, m, (), boxes, ysign=-1.0)


def features(p: Params) -> list[dict]:
    """The lid in its print frame (outside face on the bed, Y mirrored): plate, lip, pockets,
    display window, rim and bosses, screw holes and vents, all sketched on Top."""
    t = cl.LID_T
    xc, yc = (cl.X_OUT[0] + cl.X_OUT[1]) / 2, -(cl.Y_OUT[0] + cl.Y_OUT[1]) / 2
    w, h = cl.X_OUT[1] - cl.X_OUT[0], cl.Y_OUT[1] - cl.Y_OUT[0]
    # Lip: 1.6 wide, LID_GAP inside the walls, notched round the tray's lid bosses.
    g, lt = cl.LID_GAP, cl.LID_LIP_T
    lx0, lx1 = cl.X_IN[0] + g, cl.X_IN[1] - g
    ly0, ly1 = cl.Y_IN[0] + g, cl.Y_IN[1] - g
    lr = max(cl.CORNER_R - cl.WALL_T - g, 0.5)
    lxc, lyc = (lx0 + lx1) / 2, -(ly0 + ly1) / 2
    n = cl.LID_BOSS + 2 * p.notch_clear
    screws = [(x, -y) for x, y in enclosure.lid_screws()]
    lip_face = Pos(lxc, lyc) * RectangleRounded(lx1 - lx0, ly1 - ly0, lr)
    lip_face -= Pos(lxc, lyc) * Rectangle(lx1 - lx0 - 2 * lt, ly1 - ly0 - 2 * lt)
    for x, y in screws:
        lip_face -= Pos(x, y) * Rectangle(n, n)
    pocket_profiles, pocket_pts = _pocket_sketch(p)
    # Display, in print coordinates: window, glass rim, bracket bosses.
    x, y = cl.DISP_X, -cl.DISP_Y
    vx, vy = (v + 2 * p.window_margin for v in cl.DISP_VA)
    gx0, gx1 = cl.DISP_GLASS_X
    gy = cl.DISP_GLASS[1] / 2
    rw, rh = p.rim
    c = p.glass_clear
    gxc, gw = x + (gx0 + gx1) / 2, gx1 - gx0
    boss_h = cl.WALL_EDGE_Z - cl.DISP_BACK_Z
    ears = [(x + dx, y - dy) for dx, dy in cl.DISP_EARS]
    vents = [(cl.ZED_XC - 45.0 + i * p.vent_pitch, -(cl.ZED_YC + s)) for i in range(11)
             for s in (-20.0, 20.0)]  # fmt: skip
    return [
        {"op": "sketch", "name": "Plate outline", "plane": "Top", "profiles": [
            {"rect": (xc, yc, w, h), "r": cl.CORNER_R}]},
        {"op": "extrude", "name": "Plate", "sketch": "Plate outline", "regions": [(xc, yc)],
         "depth": t},
        {"op": "chamfer", "name": "Elephant's foot chamfer", "size": p.foot_chamfer,
         "edges": [(xc, yc - h / 2, 0.0)]},
        {"op": "sketch", "name": "Lip", "plane": "Top", "profiles": [
            {"rect": (lxc, lyc, lx1 - lx0, ly1 - ly0), "r": lr},
            {"rect": (lxc, lyc, lx1 - lx0 - 2 * lt, ly1 - ly0 - 2 * lt)},
            *[{"rect": (sx, sy, n, n)} for sx, sy in screws]]},
        {"op": "extrude", "name": "Lip", "sketch": "Lip", "regions": regions.points(lip_face),
         "depth": cl.LID_LIP, "start": t, "mode": "add"},
        {"op": "sketch", "name": "Pockets", "plane": "Top", "profiles": pocket_profiles},
        {"op": "extrude", "name": "Pockets", "sketch": "Pockets", "regions": pocket_pts,
         "depth": t - p.skin, "start": p.skin, "mode": "remove"},
        {"op": "sketch", "name": "Display window", "plane": "Top", "profiles": [
            {"rect": (x, y, vx, vy), "r": p.window_r}]},
        {"op": "extrude", "name": "Display window", "sketch": "Display window",
         "regions": [(x, y)], "depth": t, "mode": "remove"},
        {"op": "chamfer", "name": "Window chamfer", "size": p.window_chamfer,
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
            *[{"circle": (sx, sy, p.clear)} for sx, sy in screws],
            *[{"rect": (vx_, vy_, p.vent[0], p.vent[1]), "r": p.vent[0] / 2 - 0.01}
              for vx_, vy_ in vents]]},
        {"op": "extrude", "name": "Screw holes and vents", "sketch": "Screw holes and vents",
         "regions": screws + vents, "depth": t, "mode": "remove"},
    ]  # fmt: skip


def build(p: Params) -> Part:
    return recipe.build(features(p))


def sections(p: Params) -> list[str]:
    x, _ = enclosure.lid_screws()[0]
    ex = cl.DISP_X + cl.DISP_EARS[0][0]
    return [f"x={x}", f"y={-cl.ZED_YC}", f"x={ex}", f"y={-cl.DISP_Y}"]


def expect(p: Params) -> dict:
    return {
        "bbox": (
            cl.X_OUT[1] - cl.X_OUT[0],
            cl.Y_OUT[1] - cl.Y_OUT[0],
            cl.LID_T + cl.WALL_EDGE_Z - cl.DISP_BACK_Z,
        ),
        "holes": {p.clear: 6, p.insert_hole: 4},
        "mass_g": (None, 250),
    }
