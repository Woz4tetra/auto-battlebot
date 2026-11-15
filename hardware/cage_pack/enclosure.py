"""Cage pack enclosure tray: the one part that carries everything, printed in black PETG.

A 3 mm back panel with four 2.4 mm walls rising toward the viewer, printed on a Bambu P1S
with the panel's glass-side face on the bed. Two printed arms (cup_arm.py) screw to the
panel's underside and carry a suction cup each, beside the side walls; everything inside
screws into heat-set inserts in printed bosses. The lid (enclosure_lid.py) closes the front.

Spec
- Process: FDM, black PETG, 0.4 nozzle, 0.2 layers, printed panel-down (the build frame is
  the print frame). Outside 250.8 x 213.8 x 73.0 on the P1S's 256 x 256 plate: 2.6 mm each
  side, so no brim; PETG over ASA for that reason (less warp) and for its impact toughness.
  PROCESS = "fdm".
- Frame: cage_layout.py. Panel's glass-side face at Z = CUP_H (level with the cups' rail
  tops).
- Stiffness: a cross of 6 mm ribs on the panel, a spine along X = 0 between the two CP10
  holes and a cross rib along Y = -50 under the ZED Box Mini wall to wall, with a ring
  around each CP10 hole. The 70 mm walls close the box.
- Weight: the panel is pocketed from the inside down to a 1.2 mm floor (six solid layers),
  leaving 3 mm full-thickness lands along the ribs, at x = -56, 48.5 and y = -116, round
  every boss and CP10 ring, and round the panel's edge. print_mass.py slices it at 282 g
  (317 g unpocketed).
- Interfaces:
  - Cup arms: 4 x Ø4.0 x 7.4 insert holes up from the panel's glass side, 28 in from each
    side wall at y = CUP_Y +- 12, for M3 x 6.4 heat-set inserts pressed from below. Each
    sits under a Ø8 x 6 boss on the panel's inside; the +X pair is under the ZED Box
    Mini's edge, 2 mm below its underside. M3 x 12 socket heads come up through each arm's
    tongue.
  - Tripod (CP10): 2 x Ø10.3 (3/8" normal clearance) at (0, 0) and (0, -153.5), where the
    cups used to bolt on, each in a Ø27 x 1.4 washer spot face. A 3/8-16 x 1/2" button head
    over a 1" washer from inside reaches a CP10 cheese plate under the panel.
  - Bosses Ø8 with Ø4.0 x 7.4 holes for M3 x 6.4 heat-set inserts (94180A333): four 8 mm
    under the ZED Box Mini's ears, four 8 mm under the PCBA, four 25 mm under the battery
    clamp and OpenUPS.
  - Camera (+X wall, outside): a 1/4-20 x 1" socket head from inside through a 4 mm pad on
    the wall's inside face (Ø6.53 close clearance), the shield's flange and spacer boss,
    into the ball head's base thread. An M3 insert through the pad (from inside) takes the
    shield's anti-rotation screw. A Ø14 pointed-roof hole passes the GMSL cable's FAKRA plug.
  - Service wall (-X): HR10 Ø8.2 + key slot in a 1.8 mm spot face (catalog panel 0.7 to
    2.0). GX12 hole with a flat-topped teardrop roof, under its nut.
  - Bottom wall: Ø19.3 power button hole (CIT AH, 19.00 cut-out) with a teardrop roof that
    stays under the button's Ø21.8 bezel. Beside it at cage_layout.RJ45_AT, the ModDIY RJ45
    panel jack cut-out 21.8 x 21.6 with a 0.2 mm roof that stays under its 22 mm flange (a
    21.4 mm bridge), 2 x Ø3.4 on 27.5 mm.
  - Antenna wall (+X): 3 x Ø6.5 for RP-SMA bulkheads.
  - Lid: six 9 x 9 bosses hung from the walls' tops, each with an M3 insert hole along -Z,
    carried by 50° gussets so they print without support.
  - Tether: Ø6.5 through the top wall at the -X end, in a pad that makes the wall 6 mm thick
    there.
- Vents: 3 x 30 slots with pointed roofs in the top and bottom walls.
- Critical dimensions: inside 209 x 209 x 70; CP10 holes 153.5 apart on X = 0, the INNOREL
  CP10's 3/8-16 pair, so the tray bolts to a CP10 on a tripod; arm insert holes on
  cage_layout.arm_screws(); fits the P1S plate with BED_MARGIN each side.

Open questions
- The arm inserts are pressed from the glass side and pull out toward it under load; pull
  test a printed coupon. PETG creeps under the arm screws' heads: check them before each
  event.
- Tether pull-out of the 6 mm pad is about 3 kN in shear at 30 MPa: estimate only. Pull test
  a printed coupon before trusting it.
- Panel stiffness against the aluminium tray is lower; dynamics.py assumes a stiff tray.
- The 1.2 mm pocket floors carry the glass-side face; a hit through the glass at a pocket is
  unchecked. Raise `floor` to 1.6 for about +10 g if a test print feels soft.
"""

import math
from dataclasses import dataclass
from typing import NamedTuple

import b123d_fix  # noqa: F401
import cage_layout as cl
import hr10
import recipe  # build123d-part/scripts, on the path when run through scripts/run
import regions
from build123d import Part, Pos

PROCESS = "fdm"
MATERIAL = "petg"


BOTTOM_VENT_X_MAX = 8.0


class RibRing(NamedTuple):
    inside_dia: float
    wall: float


@dataclass
class Params:
    cp10_clearance_dia: float = 10.3
    m3_clearance_dia: float = 3.4
    gx12_m12_hole: float = 12.2
    switch_hole: float = cl.SWITCH_HOLE
    rpsma_hole: float = 6.5
    tether_hole: float = 6.5
    tether_pad_xtz: tuple = (20.0, 3.6, 24.0)
    rj45_cut_xz: tuple = (21.8, 21.6)
    rj45_roof_rise: float = 0.2
    rj45_screw_pitch_x: float = 27.5
    hr10_spot_dia: float = 14.0
    vent_wl: tuple = (3.0, 30.0)
    vent_pitch: float = 9.0
    quarter_close_clearance_dia: float = 6.53
    gmsl_hole_dia: float = 14.0
    ring: RibRing = RibRing(28.0, 2.4)
    gusset_slope: float = 50.0
    foot_chamfer: float = 0.5
    floor: float = 1.2
    pocket_land: float = 3.0


def ups_lower_holes() -> list[tuple[float, float]]:
    return list(cl.UPS_HOLES)


def pcba_holes() -> list[tuple[float, float]]:
    import pcba_cage

    return pcba_cage.holes()


def lid_bosses() -> list[tuple[str, float]]:
    """(wall, coordinate along it) for the lid's screw bosses."""
    return [
        ("top", -60.0),
        ("top", 70.0),
        ("bottom", -50.0),
        ("bottom", -30.0),  # the RJ45 jack takes x = 11 to 49
        ("-x", -176.0),  # clear of the OpenUPS
        ("+x", -100.0),
    ]


def lid_screws() -> list[tuple[float, float]]:
    """(x, y) of each lid screw: the boss's insert axis."""
    o = cl.LID_BOSS_SIZE / 2
    out = []
    for wall, a in lid_bosses():
        out.append(
            {
                "top": (a, cl.Y_IN[1] - o),
                "bottom": (a, cl.Y_IN[0] + o),
                "-x": (cl.X_IN[0] + o, a),
                "+x": (cl.X_IN[1] - o, a),
            }[wall]
        )
    return out


def _teardrop(cx: float, cy: float, d: float, flat: float = 0.5):
    """Circle with a flat 45 degree roof so it prints in a wall."""
    r = d / 2
    k = r / math.sqrt(2)
    top = r + flat
    roof_half_w = r * math.sqrt(2) - top
    roof = [
        (cx - k, cy + k),
        (cx + k, cy + k),
        (cx + roof_half_w, cy + top),
        (cx - roof_half_w, cy + top),
    ]
    pts = [(cx, cy), (cx, cy + (k + r) / 2), (cx, cy + (r + top) / 2)]
    return [{"circle": (cx, cy, d)}, {"polygon": roof}], pts


def _roofed_rect(cx: float, cy: float, w: float, h: float, rise: float) -> dict:
    """Rectangle w x h with a 45° roof `rise` tall, leaving a w - 2 rise bridge."""
    x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2
    return {"polygon": [(x0, y0), (x1, y0), (x1, y1), (x1 - rise, y1 + rise),
                        (x0 + rise, y1 + rise), (x0, y1)]}  # fmt: skip


def _slot(cx: float, cy: float, w: float, h: float) -> dict:
    """Vent slot along Z: flat bottom, pointed roof."""
    return {"polygon": [(cx - w / 2, cy - h / 2), (cx + w / 2, cy - h / 2),
                        (cx + w / 2, cy + h / 2 - w / 2), (cx, cy + h / 2),
                        (cx - w / 2, cy + h / 2 - w / 2)]}  # fmt: skip


def _span(a: float, b: float, pitch: float) -> list[float]:
    """Positions from a to at most b, `pitch` apart."""
    return [a + i * pitch for i in range(int((b - a) / pitch) + 1)]


def _through_x(x_mid: float, depth: float) -> dict:
    """Extrude range for a Right-plane cut centred on x_mid (normal +X)."""
    return {"start": x_mid - depth / 2, "depth": depth}


def _through_y(y_mid: float, depth: float) -> dict:
    """Extrude range for a Front-plane cut centred on y_mid (normal -Y, so start is -y)."""
    return {"start": -(y_mid + depth / 2), "depth": depth}


def _touches(cells, x: float, y: float, r: float) -> bool:
    """True if a circle is tangent to an edge of any cell."""
    for x0, x1, y0, y1 in cells:
        for (ax, ay), (bx, by) in (((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)),
                                   ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))):  # fmt: skip
            t = max(0.0, min(1.0, ((x - ax) * (bx - ax) + (y - ay) * (by - ay))
                             / ((bx - ax) ** 2 + (by - ay) ** 2)))  # fmt: skip
            if abs(math.hypot(x - ax - t * (bx - ax), y - ay - t * (by - ay)) - r) < 0.01:
                return True
    return False


def pocket_sketch(xs, ys, land, keep_circles=(), keep_boxes=(), ysign: float = 1.0):
    """Pockets as a recipe sketch: (profiles, region points). Profiles are the cells and the
    land round each keep-out that crosses one; the points pick every piece of the cells left
    outside the lands. `ysign` -1 mirrors Y, for a part built in a frame turned over about X
    (the lid's print frame).

    A land circle that only touches a cell's edge leaves a pocket whose boundary meets
    itself at a point, which Onshape's kernel will not extrude (the PCBA's (25.5, -107.5)
    boss land touched the y = -114.5 cell edge). Such a land grows by 0.1 so it crosses."""
    import regions
    from build123d import Circle, Rectangle

    h = land / 2
    cells = []
    for xa, xb in zip(xs[:-1], xs[1:], strict=True):
        for ya, yb in zip(ys[:-1], ys[1:], strict=True):
            x0, x1 = xa + (h if xa != xs[0] else 0), xb - (h if xb != xs[-1] else 0)
            y0, y1 = ya + (h if ya != ys[0] else 0), yb - (h if yb != ys[-1] else 0)
            cells.append((x0, x1, y0, y1))
    s = ysign

    def rect(x0, x1, y0, y1):
        return Pos((x0 + x1) / 2, s * (y0 + y1) / 2) * Rectangle(x1 - x0, y1 - y0)

    keeps = []
    for x, y, r in keep_circles:
        rk = r + land + (0.1 if _touches(cells, x, y, r + land) else 0.0)
        keeps.append((Pos(x, s * y) * Circle(rk), {"circle": (x, s * y, 2 * rk)}))
    keeps += [(rect(x0 - land, x1 + land, y0 - land, y1 + land),
               {"rect": ((x0 + x1) / 2, s * (y0 + y1) / 2, x1 - x0 + 2 * land, y1 - y0 + 2 * land)})
              for x0, x1, y0, y1 in keep_boxes]  # fmt: skip
    used, pts = set(), []
    for c in cells:
        face = rect(*c)
        for i, (k, _) in enumerate(keeps):
            common = face & k
            if common is not None and common.area > 1e-6:
                used.add(i)
                face -= k
        pts += regions.points(face)
    profiles = [{"rect": ((x0 + x1) / 2, s * (y0 + y1) / 2, x1 - x0, y1 - y0)}
                for x0, x1, y0, y1 in cells]  # fmt: skip
    profiles += [prof for i, (_, prof) in enumerate(keeps) if i in used]
    return profiles, pts


def _pocket_sketch(p: Params):
    """Panel pockets between lands."""
    m = p.pocket_land
    xs = [cl.X_IN[0] + m, -56.0, 0.0, 48.5, cl.X_IN[1] - m]
    ys = [cl.Y_IN[0] + m, -116.0, cl.ZED_YC, cl.Y_IN[1] - m]
    r_ring = p.ring.inside_dia / 2 + p.ring.wall
    holes = cl.ZED_EAR_HOLES + pcba_holes() + ups_lower_holes() + cl.arm_screws()
    circles = [(x, y, cl.BOSS_D / 2) for x, y in holes] + [(0.0, y, r_ring) for y in cl.CP10_Y]
    h = cl.RIB_T / 2
    tx, tn, _ = p.tether_pad_xtz
    _, d, _ = cl.CAM_PAD
    w = cl.CAM_PAD[0]
    spine_box = (-h, h, cl.Y_IN[0], cl.Y_IN[1])
    cross_rib_box = (cl.X_IN[0], cl.X_IN[1], cl.ZED_YC - h, cl.ZED_YC + h)
    tether_pad_box = (-100.0 - tx / 2, -100.0 + tx / 2, cl.Y_IN[1] - tn, cl.Y_IN[1])
    cam_pad_box = (cl.X_IN[1] - d, cl.X_IN[1], cl.CAM_Y - w / 2, cl.CAM_Y + w / 2)
    boxes = [spine_box, cross_rib_box, tether_pad_box, cam_pad_box]
    return pocket_sketch(xs, ys, m, circles, boxes)


def _ribs_sketch(p: Params):
    """Spine, cross rib and CP10 rings sketch."""
    from build123d import Circle, Rectangle

    r_out = p.ring.inside_dia / 2 + p.ring.wall
    yc_spine = (cl.Y_IN[0] + cl.Y_IN[1]) / 2
    spine = {"rect": (0.0, yc_spine, cl.RIB_T, cl.Y_IN[1] - cl.Y_IN[0])}
    xc_cross = (cl.X_IN[0] + cl.X_IN[1]) / 2
    cross = {"rect": (xc_cross, cl.ZED_YC, cl.X_IN[1] - cl.X_IN[0], cl.RIB_T)}
    rings = [{"circle": (0.0, y, 2 * r_out)} for y in cl.CP10_Y]
    insides = [{"circle": (0.0, y, p.ring.inside_dia)} for y in cl.CP10_Y]
    target = Pos(0, yc_spine) * Rectangle(cl.RIB_T, cl.Y_IN[1] - cl.Y_IN[0])
    target += Pos(xc_cross, cl.ZED_YC) * Rectangle(cl.X_IN[1] - cl.X_IN[0], cl.RIB_T)
    for y in cl.CP10_Y:
        target += Pos(0, y) * Circle(r_out)
    for y in cl.CP10_Y:
        target -= Pos(0, y) * Circle(p.ring.inside_dia / 2)
    profiles = [spine, cross, *rings, *insides]
    return profiles, regions.cell_points(profiles, target)


def _lid_boss_profile(wall: str, p: Params) -> tuple[list[tuple[float, float]], tuple]:
    """A lid boss's side profile in its sketch plane: (polygon, a point inside). A 9 x 9
    block under the wall's edge, its underside a 50° gusset back into the wall; it starts
    0.2 inside the wall so the block fuses. Top and bottom walls sketch on Right (Y, Z), the
    ±X walls on Front (X, Z)."""
    w, d = cl.LID_BOSS_SIZE, cl.LID_BOSS_FULL_DEPTH
    drop = w * math.tan(math.radians(p.gusset_slope))
    zt = cl.LID_UNDERSIDE_Z
    face, inward = {"top": (cl.Y_IN[1], -1), "bottom": (cl.Y_IN[0], 1),
                    "-x": (cl.X_IN[0], 1), "+x": (cl.X_IN[1], -1)}[wall]  # fmt: skip
    pts = [(-0.2, zt), (w, zt), (w, zt - d), (-0.2, zt - d - drop - 0.2)]
    poly = [(face + inward * n, z) for n, z in pts]
    return poly, (face + inward * w / 2, zt - d / 2)


def _shell_ops(p: Params) -> list[dict]:
    t = cl.WALL_T
    z0, z1, ze = cl.PANEL_Z0, cl.PANEL_Z1, cl.LID_UNDERSIDE_Z
    xo, yo = (cl.X_OUT[0] + cl.X_OUT[1]) / 2, (cl.Y_OUT[0] + cl.Y_OUT[1]) / 2
    xi, yi = (cl.X_IN[0] + cl.X_IN[1]) / 2, (cl.Y_IN[0] + cl.Y_IN[1]) / 2
    wo, ho = cl.X_OUT[1] - cl.X_OUT[0], cl.Y_OUT[1] - cl.Y_OUT[0]
    wi, hi = cl.X_IN[1] - cl.X_IN[0], cl.Y_IN[1] - cl.Y_IN[0]
    return [
        {
            "op": "sketch",
            "name": "Tray outline",
            "plane": "Top",
            "profiles": [
                {"rect": (xo, yo, wo, ho), "r": cl.OUTER_CORNER_R},
                {"rect": (xi, yi, wi, hi), "r": cl.OUTER_CORNER_R - t},
            ],
        },
        {
            "op": "extrude",
            "name": "Tray",
            "sketch": "Tray outline",
            "regions": [(xo, cl.Y_OUT[0] + t / 2), (xi, yi)],
            "depth": ze - z0,
            "start": z0,
        },
        {
            "op": "extrude",
            "name": "Inside",
            "sketch": "Tray outline",
            "regions": [(xi, yi)],
            "depth": ze - z1,
            "start": z1,
            "mode": "remove",
        },
        {
            "op": "chamfer",
            "name": "Elephant's foot chamfer",
            "size": p.foot_chamfer,
            "edges": [(xo, cl.Y_OUT[0], z0)],
        },
    ]


def _rib_ops(p: Params) -> list[dict]:
    z1 = cl.PANEL_Z1
    ops: list[dict] = []
    rib_profiles, rib_pts = _ribs_sketch(p)
    ops += [
        {"op": "sketch", "name": "Ribs", "plane": "Top", "profiles": rib_profiles},
        {
            "op": "extrude",
            "name": "Ribs",
            "sketch": "Ribs",
            "regions": rib_pts,
            "depth": cl.RIB_H,
            "start": z1,
            "mode": "add",
        },
    ]
    return ops


def _insert_boss_ops(p: Params) -> list[dict]:
    z1 = cl.PANEL_Z1
    ops: list[dict] = []
    short = cl.ZED_EAR_HOLES + pcba_holes()
    tall = ups_lower_holes()
    arms = cl.arm_screws()
    ring = (cl.BOSS_D + cl.INSERT_HOLE) / 4
    ops += [
        {
            "op": "sketch",
            "name": "Insert bosses",
            "plane": "Top",
            "profiles": [
                *[{"circle": (x, y, cl.BOSS_D)} for x, y in short + tall + arms],
                *[{"circle": (x, y, cl.INSERT_HOLE)} for x, y in short + tall + arms],
            ],
        },
        {
            "op": "extrude",
            "name": "ZED and PCBA bosses",
            "sketch": "Insert bosses",
            "regions": [(x + ring, y) for x, y in short] + short,
            "depth": cl.BOSS_H,
            "start": z1,
            "mode": "add",
        },
        {
            "op": "extrude",
            "name": "OpenUPS bosses",
            "sketch": "Insert bosses",
            "regions": [(x + ring, y) for x, y in tall] + tall,
            "depth": cl.UPS_BOSS_H,
            "start": z1,
            "mode": "add",
        },
        {
            "op": "extrude",
            "name": "Cup arm bosses",
            "sketch": "Insert bosses",
            "regions": [(x + ring, y) for x, y in arms] + arms,
            "depth": cl.ARM_BOSS_H,
            "start": z1,
            "mode": "add",
        },
    ]
    return ops


def _lid_boss_ops(p: Params) -> list[dict]:
    ops: list[dict] = []
    by_plane = {"Right": [], "Front": []}
    for wall, a in lid_bosses():
        poly, pt = _lid_boss_profile(wall, p)
        by_plane["Right" if wall in ("top", "bottom") else "Front"].append((wall, a, poly, pt))
    for plane, items in by_plane.items():
        name = f"Lid bosses, {'top and bottom' if plane == 'Right' else 'side'} walls"
        polys = {tuple(map(tuple, poly)): pt for _, _, poly, pt in items}
        ops.append(
            {
                "op": "sketch",
                "name": name,
                "plane": plane,
                "profiles": [{"polygon": list(poly)} for poly in polys],
            }
        )
        for k, (wall, a, _, pt) in enumerate(items):
            w = cl.LID_BOSS_SIZE
            rng = {"start": a - w / 2, "depth": w} if plane == "Right" else _through_y(a, w)
            ops.append(
                {
                    "op": "extrude",
                    "name": f"Lid boss {wall} {a:g}",
                    "sketch": name,
                    "regions": [pt],
                    "mode": "add",
                    **rng,
                }
            )
    return ops


def _pad_ops(p: Params) -> list[dict]:
    z1 = cl.PANEL_Z1
    ops: list[dict] = []
    tx, tn, tz = p.tether_pad_xtz
    cw, cd, ctop = cl.CAM_PAD
    ops += [
        {
            "op": "sketch",
            "name": "Pads",
            "plane": "Top",
            "profiles": [
                {"rect": (-100.0, cl.Y_IN[1] - tn / 2, tx, tn)},
                {"rect": (cl.X_IN[1] - cd / 2, cl.CAM_Y, cd, cw)},
            ],
        },
        {
            "op": "extrude",
            "name": "Tether pad",
            "sketch": "Pads",
            "regions": [(-100.0, cl.Y_IN[1] - tn / 2)],
            "depth": tz,
            "start": z1,
            "mode": "add",
        },
        {
            "op": "extrude",
            "name": "Camera pad",
            "sketch": "Pads",
            "regions": [(cl.X_IN[1] - cd / 2, cl.CAM_Y)],
            "depth": ctop - z1,
            "start": z1,
            "mode": "add",
        },
    ]
    return ops


def _pocket_ops(p: Params) -> list[dict]:
    z0, z1 = cl.PANEL_Z0, cl.PANEL_Z1
    ops: list[dict] = []
    pocket_profiles, pocket_pts = _pocket_sketch(p)
    ops += [
        {"op": "sketch", "name": "Panel pockets", "plane": "Top", "profiles": pocket_profiles},
        {
            "op": "extrude",
            "name": "Panel pockets",
            "sketch": "Panel pockets",
            "regions": pocket_pts,
            "depth": z1 - (z0 + p.floor),
            "start": z0 + p.floor,
            "mode": "remove",
        },
    ]
    return ops


def _panel_hole_ops(p: Params) -> list[dict]:
    z0, z1, ze = cl.PANEL_Z0, cl.PANEL_Z1, cl.LID_UNDERSIDE_Z
    short = cl.ZED_EAR_HOLES + pcba_holes()
    tall = ups_lower_holes()
    arms = cl.arm_screws()
    ops: list[dict] = []
    cp10 = [(0.0, y) for y in cl.CP10_Y]
    spot_ring = (p.cp10_clearance_dia + cl.CP10_SPOT[0]) / 4
    lids = lid_screws()
    ops += [
        {
            "op": "sketch",
            "name": "CP10 screws",
            "plane": "Top",
            "profiles": [
                *[{"circle": (x, y, p.cp10_clearance_dia)} for x, y in cp10],
                *[{"circle": (x, y, cl.CP10_SPOT[0])} for x, y in cp10],
            ],
        },
        {
            "op": "extrude",
            "name": "CP10 screw holes",
            "sketch": "CP10 screws",
            "regions": cp10,
            "depth": cl.PANEL_T,
            "start": z0,
            "mode": "remove",
        },
        {
            "op": "extrude",
            "name": "Washer spot faces",
            "sketch": "CP10 screws",
            "regions": [(x + spot_ring, y) for x, y in cp10],
            "depth": cl.CP10_SPOT[1],
            "start": z1 - cl.CP10_SPOT[1],
            "mode": "remove",
        },
        {
            "op": "extrude",
            "name": "ZED and PCBA insert holes",
            "sketch": "Insert bosses",
            "regions": short,
            "depth": cl.INSERT_DEPTH,
            "start": z1 + cl.BOSS_H - cl.INSERT_DEPTH,
            "mode": "remove",
        },
        {
            "op": "extrude",
            "name": "OpenUPS insert holes",
            "sketch": "Insert bosses",
            "regions": tall,
            "depth": cl.INSERT_DEPTH,
            "start": z1 + cl.UPS_BOSS_H - cl.INSERT_DEPTH,
            "mode": "remove",
        },
        {
            "op": "extrude",
            "name": "Cup arm insert holes",
            "sketch": "Insert bosses",
            "regions": arms,
            "depth": cl.INSERT_DEPTH,
            "start": z0,
            "mode": "remove",
        },
        {
            "op": "sketch",
            "name": "Lid inserts",
            "plane": "Top",
            "profiles": [{"circle": (x, y, cl.INSERT_HOLE)} for x, y in lids],
        },
        {
            "op": "extrude",
            "name": "Lid insert holes",
            "sketch": "Lid inserts",
            "regions": lids,
            "depth": cl.INSERT_DEPTH,
            "start": ze - cl.INSERT_DEPTH,
            "mode": "remove",
        },
    ]
    return ops


def _minus_x_wall_ops(p: Params) -> list[dict]:
    t = cl.WALL_T
    ops: list[dict] = []
    t3 = 3 * t
    mid_mx = cl.X_IN[0] - t / 2
    gy, gz = cl.SERVICE_PORTS_YZ["gx12"]
    hy, hz = cl.SERVICE_PORTS_YZ["hr10"]
    gx12, gx12_pts = _teardrop(gy, gz, p.gx12_m12_hole)
    key_h = hr10.KEY_R + 0.05
    hr10_hole = [{"circle": (hy, hz, hr10.PANEL_HOLE + 0.2)},
                 {"rect": (hy, hz - key_h / 2, hr10.KEY_W + 0.2, key_h)}]  # fmt: skip
    hr10_pts = [(hy, hz + 2.0), (hy, hz - 2.0), (hy, hz - (hr10.PANEL_HOLE + 0.2 + 2 * key_h) / 4)]
    ops += [
        {
            "op": "sketch",
            "name": "-X wall ports",
            "plane": "Right",
            "profiles": [*gx12, *hr10_hole],
        },
        {
            "op": "extrude",
            "name": "GX12 hole",
            "sketch": "-X wall ports",
            "regions": gx12_pts,
            "mode": "remove",
            **_through_x(mid_mx, t3),
        },
        {
            "op": "extrude",
            "name": "HR10 hole and key",
            "sketch": "-X wall ports",
            "regions": hr10_pts,
            "mode": "remove",
            "start": cl.X_OUT[0] - 0.5,
            "depth": t + 1.0,
        },
    ]
    spot = t - cl.HR10_PANEL_T
    hr10_spot, hr10_spot_pts = _teardrop(hy, hz, p.hr10_spot_dia)
    ops += [
        {"op": "sketch", "name": "HR10 spot face", "plane": "Right", "profiles": hr10_spot},
        {
            "op": "extrude",
            "name": "HR10 spot face",
            "sketch": "HR10 spot face",
            "regions": hr10_spot_pts,
            "mode": "remove",
            **_through_x(cl.X_IN[0], 2 * spot),
        },
    ]
    return ops


def _plus_x_wall_ops(p: Params) -> list[dict]:
    t = cl.WALL_T
    t3 = 3 * t
    mid_px = cl.X_IN[1] + t / 2
    ops: list[dict] = []
    gmsl, gmsl_pts = _teardrop(*cl.GMSL_HOLE, p.gmsl_hole_dia)
    antennas = list(cl.ANTENNAS_YZ.values())
    pad_holes = [(cl.CAM_Y, cl.CAM_AXIS_Z), (cl.CAM_Y, cl.SHIELD_SCREW_Z)]
    pad = t + cl.CAM_PAD[1]
    ops += [
        {
            "op": "sketch",
            "name": "+X wall ports",
            "plane": "Right",
            "profiles": [
                *[{"circle": (y, z, p.rpsma_hole)} for y, z in antennas],
                *gmsl,
                {"circle": (*pad_holes[0], p.quarter_close_clearance_dia)},
                {"circle": (*pad_holes[1], cl.INSERT_HOLE)},
            ],
        },
        {
            "op": "extrude",
            "name": "Antennas and GMSL",
            "sketch": "+X wall ports",
            "regions": antennas + gmsl_pts,
            "mode": "remove",
            **_through_x(mid_px, t3),
        },
        {
            "op": "extrude",
            "name": "Ball head screw and shield insert",
            "sketch": "+X wall ports",
            "regions": pad_holes,
            "mode": "remove",
            **_through_x(mid_px, 3 * pad),
        },
    ]
    return ops


def _bottom_and_top_wall_ops(p: Params) -> list[dict]:
    t = cl.WALL_T
    z1 = cl.PANEL_Z1
    t3 = 3 * t
    ops: list[dict] = []
    vz = z1 + cl.RIB_H + 4.0 + p.vent_wl[1] / 2
    rx, rz = cl.RJ45_AT
    screws = [(rx + s * p.rj45_screw_pitch_x / 2, rz) for s in (-1, 1)]
    bottom_vents = [(x, vz) for x in _span(-100.0, BOTTOM_VENT_X_MAX, p.vent_pitch)]
    top_vents = [(x, vz) for x in _span(-84.0, cl.X_IN[1] - 8.0, p.vent_pitch)]
    switch, switch_pts = _teardrop(cl.SWITCH_X, cl.SWITCH_Z, p.switch_hole)
    tether = (-100.0, z1 + 12.0)
    ops += [
        {
            "op": "sketch",
            "name": "Bottom wall ports",
            "plane": "Front",
            "profiles": [
                *switch,
                _roofed_rect(rx, rz, *p.rj45_cut_xz, p.rj45_roof_rise),
                *[{"circle": (x, z, p.m3_clearance_dia)} for x, z in screws],
                *[_slot(x, z, *p.vent_wl) for x, z in bottom_vents],
            ],
        },
        {
            "op": "extrude",
            "name": "Power button, RJ45 and bottom vents",
            "sketch": "Bottom wall ports",
            "regions": switch_pts + [(rx, rz), *screws] + bottom_vents,
            "mode": "remove",
            **_through_y(cl.Y_IN[0] - t / 2, t3),
        },
        {
            "op": "sketch",
            "name": "Top wall ports",
            "plane": "Front",
            "profiles": [
                *[_slot(x, z, *p.vent_wl) for x, z in top_vents],
                {"circle": (*tether, p.tether_hole)},
            ],
        },
        {
            "op": "extrude",
            "name": "Top vents",
            "sketch": "Top wall ports",
            "regions": top_vents,
            "mode": "remove",
            **_through_y(cl.Y_IN[1] + t / 2, t3),
        },
        {
            "op": "extrude",
            "name": "Tether hole",
            "sketch": "Top wall ports",
            "regions": [tether],
            "mode": "remove",
            **_through_y(cl.Y_IN[1] + t / 2, 4 * (t + 4)),
        },
    ]
    return ops


def features(p: Params) -> list[dict]:
    """The tray as CAD features in build order."""
    return [
        *_shell_ops(p),
        *_rib_ops(p),
        *_insert_boss_ops(p),
        *_lid_boss_ops(p),
        *_pad_ops(p),
        *_pocket_ops(p),
        *_panel_hole_ops(p),
        *_minus_x_wall_ops(p),
        *_plus_x_wall_ops(p),
        *_bottom_and_top_wall_ops(p),
    ]


def build(p: Params) -> Part:
    return recipe.build(features(p))


def sections(p: Params) -> list[str]:
    return ["x=0", f"y={cl.ZED_YC}", f"x={cl.SWITCH_X}", f"x={cl.RJ45_AT[0]}", f"y={cl.CAM_Y}",
            "x=-60",
            f"y={cl.arm_screws()[0][1]}"]  # fmt: skip


def expect(p: Params) -> dict:
    w = cl.X_OUT[1] - cl.X_OUT[0]
    hgt = cl.Y_OUT[1] - cl.Y_OUT[0]
    return {
        "bbox": (w, hgt, cl.DEPTH + cl.PANEL_T),
        "holes": {
            p.cp10_clearance_dia: 2,
            cl.INSERT_HOLE: len(
                cl.ZED_EAR_HOLES + pcba_holes() + ups_lower_holes() + cl.arm_screws() + lid_screws()
            )
            + 1,
            p.m3_clearance_dia: 2,
            p.quarter_close_clearance_dia: 1,
            p.rpsma_hole: 3 + 1,
            p.ring.inside_dia: 1,
            cl.CP10_SPOT[0]: 2,
        },
        "hole_at": {
            p.cp10_clearance_dia: [{"x": 0.0, "y": y} for y in cl.CP10_Y],
            cl.INSERT_HOLE: [{"x": x, "y": y} for x, y in cl.arm_screws()],
        },
        "mass_g": (None, 700),
        "min_wall": 1.2,
    }


if __name__ == "__main__":
    w = cl.X_OUT[1] - cl.X_OUT[0]
    h = cl.Y_OUT[1] - cl.Y_OUT[0]
    print(f"footprint {w:.1f} x {h:.1f} on a {cl.BED:.0f} plate: margins "
          f"{(cl.BED - w) / 2:.1f}, {(cl.BED - h) / 2:.1f}")  # fmt: skip
    for _, (y, z) in list(cl.SERVICE_PORTS_YZ.items()) + list(cl.ANTENNAS_YZ.items()):
        assert cl.PANEL_Z1 + cl.RIB_H + 8 <= z <= cl.PORT_Z_MAX - 8, z
    print("port heights ok")
