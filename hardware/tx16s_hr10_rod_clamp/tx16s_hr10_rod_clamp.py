"""TX16S MK2 HR10 rod clamp: holds an HR10A-7R-4S receptacle on the bare handle rod.

Variant of ../tx16s_hr10_handle_clamp for the handle with its plastic sheath removed. Two
printed halves close on the bare Ø5 rod around the handle's right corner, so they grip the
end of the back run and the top of the right arm at once. The two legs point in different
directions, so the clamp cannot turn about either one: it is located by the bend, and the
screws only have to hold it closed. A tower on the top half stands the receptacle with its
axis vertical, as on the other two mounts.

Spec
- Process: FDM PLA. Top half prints on its split face, bottom half on its underside; both
  are built in the rod plane, so neither needs flipping.
- Rod frame (u, v, n): origin on the arm axis at the corner, u along the run (pilot's
  left, radio -X), v down the arm toward the pilot, n normal to the rod plane (up, tilted
  20.6 deg toward the pilot). Radio frame as ../tx16s_hr10_mount/scan_patch.py.
- Rod: Ø5.00, cylinder fit to the bare arm in the scan (4087 vertices, 0.11 RMS, y 32..42).
  The fitted arm axis, run up to the back run, meets the run sheath's centre height within
  0.15 mm, so the rod bends in one plane (n = 0).
- Bore: Ø5.2 (FDM close fit, dfm/fdm.md: holes print 0.1-0.2 under) with the halves 0.8
  apart, so the screws close each half 0.1 onto the rod and still leave 0.6 of squeeze.
- Run position: unknown inside the old sheath. Centred it would sit at v +0.2; flush with
  the sheath's back face, as the arm sat flush with its outer face, at v -3.1. The run's
  groove is a slot from v -3.4 to +0.5, tight in n only.
- Bend radius: unknown, it was inside the sheath. The corner is a pocket clear of the rod
  for any centreline radius up to 10 mm; the grips start 10 mm from the corner on each leg.
  check.py passes mate.rod with the rod at both ends of the slot and bend radii 2.6 and 10
  (--params '{"rod_v": -3.4, "rod_bend_r": 2.6}' and so on).
- Fasteners: 3 x M3 x 8 SHCS along -n, counterbored in the top half, into M3 hex nuts in
  pockets on the bottom half's underside (fasteners.md). One beside the run, one beside the
  arm, one outside the corner, so each groove has a screw on either side of it.
- Receptacle: M8x0.5 nut in a 10.1 AF hex cavity under a 2.0 panel (catalog max), Ø8.3
  hole with a Ø10.6 x 0.8 key-tab counterbore, as the other mounts. The cavity opens to
  pilot's right (+X) as a slot the nut slides in through; the pigtail leaves by the same
  slot and drops down outside the arm.

Assembly
1. Slide the M8 nut into the tower's side slot, thread the receptacle down into it from
   the top until the flange seats, medium threadlocker on the thread.
2. Solder the pigtail and lead it out of the slot.
3. Drop the three M3 nuts into the bottom half's pockets, close both halves on the corner,
   screw up evenly until the halves stop rocking on the rod.

Open questions
- Run position and bend radius (above). If the halves bottom out on the rod, check the
  run sits inside the slot; if the clamp slides along the arm, shim the slot.
"""

from dataclasses import dataclass
from functools import lru_cache

import numpy as np
from build123d import (
    Align,
    Box,
    Cylinder,
    Edge,
    Location,
    Plane,
    Polyline,
    Pos,
    RegularPolygon,
    Rot,
    SlotCenterToCenter,
    Solid,
    Vector,
    Wire,
    extrude,
    make_face,
)

PROCESS = "fdm"
MATERIAL = "pla"

C_MIN = (Align.CENTER, Align.CENTER, Align.MIN)
C_MAX = (Align.CENTER, Align.CENTER, Align.MAX)

# Cylinder fit to the bare arm (see docstring): a point on its axis and its direction.
ARM_POINT = np.array([85.552, 37.0, -34.645])
ARM_DIR = np.array([0.01683, 1.0, 0.37518])
CORNER_Y = 57.7  # rod frame origin: the arm axis at the back run's sheath centre


@dataclass
class Params:
    half: str = "top"  # "top" or "bottom"
    rod_d: float = 5.0  # scan fit
    bore_d: float = 5.2  # rod + 0.2, FDM close fit
    gap: float = 0.8  # between halves on the rod
    t: float = 6.0  # each half's outer face from the rod plane: 3.4 over the bore
    # Run slot: rod centre anywhere from flush-back to centred (docstring)
    run_v_lo: float = -3.4
    run_v_hi: float = 0.5
    bend_max: float = 10.0  # corner pocket clears centreline radii up to this
    run_len: float = 27.0  # grip on the run ends at u 27 (radio x 59)
    arm_len: float = 26.0  # grip on the arm ends at v 26 (radio y 33, clear of the body)
    wall: float = 3.0  # outside the bore, in plane
    # Screws, rod frame (u, v)
    screws: tuple = ((17.0, 8.5), (8.5, 17.0), (-7.5, -10.5))
    clearance_d: float = 3.4  # M3 normal clearance, fasteners.md
    cbore_d: float = 6.5  # M3 SHCS counterbore, fasteners.md
    cbore_depth: float = 3.3
    nut_af: float = 5.7  # M3 nut 5.5 AF + 0.2, fasteners.md
    nut_depth: float = 3.4  # nut 2.4, sunk 1.0 so an M3 x 8 runs through it
    boss_r: float = 4.5  # outline around each screw: counterbore + 1.25
    # Receptacle tower, radio frame
    tower_u: float = 4.0  # over the corner, rod frame
    tower_v: float = 4.0
    tower_half_x: float = 7.5  # hex corner 5.83 + 1.67; the +X side is the open slot
    tower_half_y: float = 7.55  # hex flat 5.05 + 2.5
    floor_skin: float = 1.2  # cavity floor over the bore (dfm/fdm.md min wall)
    cavity_depth: float = 9.7  # panel underside to cavity floor: 7.7 behind panel + 2
    panel_t: float = 2.0  # catalog max
    hole_d: float = 8.3
    key_d: float = 10.6
    key_depth: float = 0.8
    m8_af: float = 10.1
    # Rod shown in the mates, rod frame (unknowns: any value inside the ranges above)
    rod_v: float = -1.45
    rod_bend_r: float = 5.0


# ----------------------------------------------------------------------------- frames


@lru_cache(maxsize=1)
def rod_axes() -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Origin, u, v, n of the rod frame in radio coordinates."""
    d = ARM_DIR / np.linalg.norm(ARM_DIR)
    origin = ARM_POINT + d * (CORNER_Y - ARM_POINT[1]) / d[1]
    v = -d
    u = np.array([-1.0, 0, 0]) - np.dot([-1.0, 0, 0], v) * v
    u /= np.linalg.norm(u)
    return origin, u, v, np.cross(u, v)


def rod_to_radio() -> Location:
    o, u, _, n = rod_axes()
    return Plane(origin=tuple(o), x_dir=tuple(u), z_dir=tuple(n)).location


def radio_to_rod() -> Location:
    return rod_to_radio().inverse()


def rod_to_part(p: Params, half: str | None = None) -> Pos:
    """Print frame: split face (top) or underside (bottom) on z = 0."""
    return Pos(0, 0, -p.gap / 2) if (half or p.half) == "top" else Pos(0, 0, p.t)


def radio_to_part(p: Params, half: str | None = None) -> Location:
    return rod_to_part(p, half) * radio_to_rod()


def tower_xy() -> tuple[float, float]:
    o, u, v, _ = rod_axes()
    p = Params()
    c = o + p.tower_u * u + p.tower_v * v
    return float(c[0]), float(c[1])


# ------------------------------------------------------------------------------- rod


def rod(p: Params) -> Solid:
    """Bare rod around the corner, rod frame: run at v = rod_v, bend, arm at u = 0.

    Straight legs are plain cylinders so the hole measurement reads them; only the bend
    is swept.
    """
    r, v0, rr = p.rod_bend_r, p.rod_v, p.rod_d / 2
    run = Pos(r, v0, 0) * Rot(0, 90, 0) * Cylinder(rr, p.run_len + 8 - r, align=C_MIN)
    arm = Pos(0, v0 + r, 0) * Rot(-90, 0, 0) * Cylinder(rr, p.arm_len + 8 - v0 - r, align=C_MIN)
    arc = Edge.make_three_point_arc(
        Vector(r, v0, 0),
        Vector(r - r * np.sqrt(0.5), v0 + r - r * np.sqrt(0.5), 0),
        Vector(0, v0 + r, 0),
    )
    profile = Wire.make_circle(rr, Plane(origin=(r, v0, 0), z_dir=(-1, 0, 0)))
    return run + Solid.sweep(profile, Wire([arc])) + arm


# ------------------------------------------------------------------------------ halves


def _cross2(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


def _outline_points(p: Params) -> np.ndarray:
    """Convex outline of a half in the rod plane: walls round the grooves, screw bosses."""
    rb = p.bore_d / 2
    v_lo, v_hi = p.run_v_lo - rb - p.wall, p.run_v_hi + rb + p.wall
    u_lo, u_hi = -rb - p.wall, rb + p.wall
    corners = [
        (u_lo, v_lo),
        (p.run_len, v_lo),
        (p.run_len, v_hi),
        (u_hi, p.arm_len),
        (u_lo, p.arm_len),
    ]
    a = np.linspace(0, 2 * np.pi, 48, endpoint=False)
    ring = np.stack([np.cos(a), np.sin(a)], axis=1)
    pts = [np.array(c) + 1.5 * ring for c in corners]
    pts += [np.array(s) + p.boss_r * ring for s in p.screws]
    pts = np.vstack(pts)
    # Monotone-chain convex hull
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]

    def half_hull(seq):
        out: list = []
        for q in seq:
            while len(out) >= 2 and _cross2(out[-1] - out[-2], q - out[-2]) <= 0:
                out.pop()
            out.append(q)
        return out

    lower, upper = half_hull(pts), half_hull(pts[::-1])
    return np.array(lower[:-1] + upper[:-1])


def _slab(p: Params, n_lo: float, n_hi: float) -> Solid:
    pts = [(float(x), float(y)) for x, y in _outline_points(p)]
    face = make_face(Polyline(*pts, close=True))
    return Pos(0, 0, n_lo) * extrude(face, n_hi - n_lo)


def _rod_space(p: Params) -> Solid:
    """Everything the rod may occupy: run slot, arm bore, corner pocket (full depth)."""
    rb = p.bore_d / 2
    run_v = (p.run_v_lo + p.run_v_hi) / 2
    slot = SlotCenterToCenter(p.run_v_hi - p.run_v_lo, p.bore_d)
    run = Plane.YZ.offset(-1) * Pos(run_v, 0) * extrude(slot, p.run_len + 5)  # along +u
    arm = Pos(0, -1, 0) * Rot(-90, 0, 0) * Cylinder(rb, p.arm_len + 5, align=C_MIN)
    lo = p.run_v_lo - rb
    pocket = Pos(-rb, lo, -rb) * Box(
        rb + p.bend_max, p.bend_max - lo, 2 * rb, align=(Align.MIN,) * 3
    )
    return run + arm + pocket


def _screw_cuts(p: Params, top: bool) -> Solid:
    cuts = None
    for su, sv in p.screws:
        c = Pos(su, sv, -p.t - 1) * Cylinder(p.clearance_d / 2, 2 * p.t + 2, align=C_MIN)
        if top:
            c += Pos(su, sv, p.t + 1) * Cylinder(p.cbore_d / 2, p.cbore_depth + 1, align=C_MAX)
        else:
            hexagon = RegularPolygon(p.nut_af / 2, 6, major_radius=False)
            c += Pos(su, sv, -p.t - 1) * extrude(hexagon, p.nut_depth + 1)
        cuts = c if cuts is None else cuts + c
    return cuts


def nuts_rod(p: Params) -> Solid:
    """The three M3 nuts in their pockets, rod frame."""
    nut = None
    for su, sv in p.screws:
        hexagon = RegularPolygon(5.5 / 2, 6, major_radius=False)
        n = Pos(su, sv, -p.t + p.nut_depth - 2.4) * extrude(hexagon, 2.4)
        n -= Pos(su, sv, -p.t) * Cylinder(1.25, p.t, align=C_MIN)
        nut = n if nut is None else nut + n
    return nut


def _cavity_floor(p: Params) -> float:
    """Lowest radio z for the cavity floor: floor_skin over the bore across its footprint."""
    o, u, v, n = rod_axes()
    tx, ty = tower_xy()
    n_min = p.bore_d / 2 + p.floor_skin
    h = p.m8_af / 2
    zs = []
    for x in (tx - h / np.cos(np.pi / 6), tx + p.tower_half_x + 2):
        for y in (ty - h, ty + h):
            # point (x, y, z) with n-coordinate n_min: solve for z
            rel = np.array([x - o[0], y - o[1], 0.0])
            zs.append(o[2] + (n_min - np.dot(rel, n)) / n[2])
    return float(max(zs))


def panel_top(p: Params) -> float:
    return _cavity_floor(p) + p.cavity_depth + p.panel_t


def _tower_radio(p: Params) -> tuple[Solid, Solid]:
    """Tower solid and its cuts, radio frame."""
    tx, ty = tower_xy()
    zf, zt = _cavity_floor(p), panel_top(p)
    z_lo = rod_axes()[0][2] - 30
    body = Pos(tx, ty, z_lo) * Box(2 * p.tower_half_x, 2 * p.tower_half_y, zt - z_lo, align=C_MIN)
    under = zt - p.panel_t
    cav = Pos(tx, ty, zf) * extrude(RegularPolygon(p.m8_af / 2, 6, major_radius=False), under - zf)
    cav += Pos(tx, ty - p.m8_af / 2, zf) * Box(
        p.tower_half_x + 2, p.m8_af, under - zf, align=(Align.MIN, Align.MIN, Align.MIN)
    )
    cav += Pos(tx, ty, under - 1) * Cylinder(p.hole_d / 2, p.panel_t + 2, align=C_MIN)
    cav += Pos(tx, ty, zt + 1) * Cylinder(p.key_d / 2, p.key_depth + 1, align=C_MAX)
    return body, cav


def top_rod(p: Params) -> Solid:
    """Top half, rod frame."""
    face = p.gap / 2
    part = _slab(p, face, p.t)
    body, cav = _tower_radio(p)
    keep = Pos(0, 0, face) * Box(400, 400, 200, align=C_MIN)
    part += (radio_to_rod() * body) & keep
    part -= radio_to_rod() * cav
    part -= _rod_space(p)
    part -= _screw_cuts(p, top=True)
    return part


def bottom_rod(p: Params) -> Solid:
    part = _slab(p, -p.t, -p.gap / 2)
    part -= _rod_space(p)
    part -= _screw_cuts(p, top=False)
    return part


def top_radio(p: Params) -> Solid:
    return rod_to_radio() * top_rod(p)


def bottom_radio(p: Params) -> Solid:
    return rod_to_radio() * bottom_rod(p)


def build(p: Params):
    shape = top_rod(p) if p.half == "top" else bottom_rod(p)
    return rod_to_part(p) * shape


def sections(p: Params) -> list[str]:
    off = rod_to_part(p).position.Z + (1.5 if p.half == "top" else -1.5)
    s1, s2, _ = p.screws
    return [f"x={p.run_len - 8}", f"y={p.arm_len - 6}", f"x={s1[0]}", f"y={s2[1]}", f"z={off}"]


# -------------------------------------------------------------------------- receptacle


def receptacle_radio(p: Params) -> Solid:
    """HR10A-7R-4S envelope standing on the panel, radio frame (catalog; flange, nut assumed)."""
    flange_t, nut_t = 1.5, 2.0
    body = Pos(0, 0, flange_t) * Cylinder(8.85 / 2, 6.3 - flange_t, align=C_MIN)
    flange = Cylinder(11.0 / 2, flange_t, align=C_MIN)
    thread = Cylinder(7.9 / 2, 7.7, align=C_MAX)
    key = Pos(4.45, 0, 0) * Box(1.1, 1.5, 0.7, align=C_MAX)
    nut = Pos(0, 0, -p.panel_t - nut_t) * extrude(
        RegularPolygon(10.0 / 2, 6, major_radius=False), nut_t
    )
    tx, ty = tower_xy()
    return Pos(tx, ty, panel_top(p)) * (body + flange + thread + key + nut)


def plug_radio(p: Params) -> Solid:
    """HR10A-7P-4S mated: Ø11.5 x 35 body (catalog p.6) plus 40 mm of Ø5 cable."""
    body = Pos(0, 0, 1.5) * Cylinder(11.5 / 2, 35.0, align=C_MIN)
    cable = Pos(0, 0, 36.5) * Cylinder(2.5, 40.0, align=C_MIN)
    tx, ty = tower_xy()
    return Pos(tx, ty, panel_top(p)) * (body + cable)


def mates(p: Params) -> dict:
    to_part = rod_to_part(p)
    other = bottom_rod(p) if p.half == "top" else top_rod(p)
    out = {"rod": to_part * rod(p), "other_half": to_part * other}
    if p.half == "top":
        out["receptacle"] = radio_to_part(p) * receptacle_radio(p)
        out["plug"] = radio_to_part(p) * plug_radio(p)
    return out


def expect(p: Params) -> dict:
    common = {
        "clearance": {"rod": 0.05, "other_half": p.gap - 0.05},
        "min_wall": 1.2,
    }
    if p.half == "top":
        common["holes"] = {p.hole_d: 1, p.key_d: 1, p.clearance_d: 3, p.cbore_d: 3}
        common["clearance"]["plug"] = 0.5
        return common
    common["holes"] = {p.clearance_d: 3}
    common["bbox"] = (None, None, p.t - p.gap / 2)
    return common
