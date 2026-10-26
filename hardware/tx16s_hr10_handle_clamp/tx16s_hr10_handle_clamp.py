"""TX16S MK2 HR10 handle clamp: holds an HR10A-7R-4S receptacle on the carry-handle bar.

The fastener-only alternative to ../tx16s_hr10_mount (VHB on the shell). Two printed
halves clamp the straight right end of the carry-handle bar with two M3 screws into
heat-set inserts; nothing is glued and no radio screw is touched. A tower cantilevers off
the clamp's right end over the bar's corner and stands the receptacle there, axis
vertical, so the plug and its cable come in from above. The AUX1 pigtail leaves the
tower's underside through a front notch, straight onto the right handle arm.

Spec
- Process: FDM PLA, both halves printed upright (radio +Z up): the top half on its split
  face, the bottom half on its underside. The bore crown and the hex cavity's ceiling
  print as bridges. The tower's floor is a one-sided 16 mm overhang 9 mm off the bed
  (check.py fdm.overhang WARN): enable slicer support under the tower only.
- Frames: geometry is built in the radio frame (../tx16s_hr10_mount/scan_patch.py: +X
  pilot's right, +Y away from pilot, +Z up). Each half is then shifted only, so its print
  bed is z = 0 and its origin sits at the clamp's centre. Params.half picks the half.
- Handle: fixed (user, 2026-10-25). Bar sections from the scan (bar_scan.py,
  tx16s_handle_bar_sections.npz) at x 58..80. The front face recedes from y 47.8 to 51.9
  across the clamp, so the bore is a ruled loft through those sections, not a prism.
- Bore tools: 0.2 mm radial clearance on the scan sections. Each half follows the bar's
  surface out from its widest points, and never within 0.15 mm of its split face; nearer
  the split the bore walls stand straight at the section's full width, so neither half
  has an undercut lip and the bar cannot reach past them. The last 1 mm at the right
  end is the x = 76 section opening 0.3 mm more toward the end face: the bar narrows
  toward +X, and a tapered bore meets that end face at under 70 deg. A 0.4 mm entry
  chamfer runs round the bore at both end faces.
- verify_scan.py against the raw scan: plug 14.8 mm clear; the bottom half's bore sits
  up to 0.18 mm into scan bumps the smoothed sections miss, which closes the 0.8 split
  gap to about 0.6 when tightened.
- Placement: clamp spans x 61..77, short of the corner bend at x 80. The tower spans
  x 77..93 over the corner (scan peak z -21.1 there) with a 1.5 mm gap under it. The arm
  drops forward from y 50 at x 83..88. The module bay is ahead of and below the bar
  (y < 25).
- Clamping: split at z -29, the section's widest point. The halves stand 0.8 apart on the
  bar; tightening the screws flexes the ears onto it.
- Fasteners: 2 x M3 x 14 SHCS from the top, counterbored, into M3 heat-set inserts
  (Ø4.0 hole, 5.7 long, fasteners.md) in the bottom half, at y 44 (front) and y 70 (back).
- Receptacle: same captive-nut scheme as the shell mount. M8x0.5 nut in a 10.1 AF hex
  cavity open at the tower's underside; the receptacle threads down into it and its key
  tab turns in a Ø10.6 x 0.8 counterbore under the flange.

Assembly
1. Press the two inserts into the bottom half with an iron.
2. Push the nut into the tower's hex cavity from below. Thread the receptacle in from
   the top until the flange seats, medium threadlocker on the thread.
3. Solder the pigtail through the cavity and lead it out of the front notch.
4. Put both halves on the bar, screws in, snug. Stop when the clamp no longer turns on
   the bar; the ears flex.

Open questions
- Grip: PLA on the bar's plastic with a scan-fit bore. If the clamp turns under a plug
  pull, add 0.5 mm silicone tape in the bore and reprint with gap 0.5.
- Key tab length and nut thickness are catalog gaps; same caveats as ../tx16s_hr10_mount.
"""

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np
from build123d import (
    Align,
    Box,
    Cylinder,
    Edge,
    GeomType,
    Part,
    Pos,
    RectangleRounded,
    RegularPolygon,
    Solid,
    Vector,
    Wire,
    chamfer,
    extrude,
)

PROCESS = "fdm"
MATERIAL = "pla"

SECTIONS = Path(__file__).resolve().parent / "tx16s_handle_bar_sections.npz"
C_MIN = (Align.CENTER, Align.CENTER, Align.MIN)
C_MAX = (Align.CENTER, Align.CENTER, Align.MAX)


@dataclass
class Params:
    half: str = "top"  # "top" or "bottom"
    # Clamp, radio frame
    x0: float = 61.0  # clamp left end
    x1: float = 77.0  # clamp right end, short of the corner bend at x 80
    split_z: float = -29.0  # widest point of the section
    gap: float = 0.8  # between the halves on the bar
    lead: float = 1.0  # constant-section bore at the right end
    bore_clear: float = 0.2  # radial, about the scan's own accuracy; the 0.8 gap still closes
    lead_open: float = 0.3  # the lead widens by this much more over its run to the end face
    wall_band: float = 0.15  # bore walls stand at full section width this far into each half
    bore_chamfer: float = 0.4  # entry chamfer at both end faces
    y_front: float = 38.5  # 2.25 wall outside the front counterbore
    y_back: float = 75.5  # 2.25 wall outside the back counterbore
    ear_top: float = -17.0  # 4.3 over the bar's crown (z -21.3)
    bottom: float = -37.5  # 3.8 under the bar's lowest point at x 61 (z -33.7)
    corner_r: float = 3.0
    # Tower, radio frame
    tower_x: float = 85.0  # over the bar's corner; arm at x 83..88
    tower_y: float = 58.0  # over the bar's crown (y 59.5)
    tower_half_x: float = 8.0  # hex flat 5.05 + 2.95 wall; tower meets the clamp at x 77
    tower_half_y: float = 7.83  # hex corner 5.83 + 2.0 wall
    tower_gap: float = 1.5  # over the corner's scan peak (z -21.1)
    corner_peak_z: float = -21.1  # scan, max z under the tower footprint
    # Receptacle, HR10 catalog (see ../tx16s_hr10_mount)
    panel_t: float = 2.0  # catalog max, for a 1.2 web under the counterbore
    hole_d: float = 8.3
    key_d: float = 10.6
    key_depth: float = 0.8
    nut_af: float = 10.1
    cavity_depth: float = 9.7  # panel underside to tower floor: 7.7 behind the panel + 2
    notch_w: float = 5.0  # wire notch through the tower's front wall; 3.5 OD cable
    notch_h: float = 4.5
    # Fasteners
    screw_y_front: float = 44.0  # Ø3.4 hole 2.3 clear of the bar front at x 61
    screw_y_back: float = 70.0  # head counterbore clear of the spine's back face
    clearance_d: float = 3.4  # M3 normal clearance, fasteners.md
    cbore_d: float = 6.5  # M3 SHCS counterbore, fasteners.md
    cbore_depth: float = 3.3
    insert_d: float = 4.0  # M3 heat-set insert hole, fasteners.md
    insert_depth: float = 6.7  # insert 5.7 + 1, fasteners.md


# ------------------------------------------------------------------------------- bar


@lru_cache(maxsize=1)
def _sections() -> tuple[np.ndarray, np.ndarray]:
    d = np.load(SECTIONS)
    return d["x"], d["loops"]


def _loop_wire(x: float, loop: np.ndarray) -> Wire:
    return Wire([Edge.make_spline([Vector(x, y, z) for y, z in loop], periodic=True)])


def bar() -> Solid:
    """The handle bar from x 58 to 80, radio frame, lofted through the scan sections.

    Ruled like the bore tools, so the two differ only by bore_clear; a smooth loft bulges
    up to 0.2 mm past the ruled one between stations.
    """
    xs, loops = _sections()
    wires = [_loop_wire(x, loop) for x, loop in zip(xs, loops, strict=True)]
    return Solid.make_loft(wires, ruled=True)


def _draft_wire(x: float, loop: np.ndarray, upper: bool, z_cut: float, z_far: float) -> Wire:
    """One half's bore profile at one station.

    The loop's arc between its widest points on this half's side (over the top for the top
    half), clipped to z_cut, then straight walls at the section's full width on to z_far.
    Each wall rises (or drops) from whichever is farther from the split, the widest point or
    z_cut: from the widest point there is no undercut, and below z_cut the bar cannot reach
    past a wall at full width however its widest point wanders between stations.
    """
    n = len(loop)
    i_back, i_front = int(np.argmax(loop[:, 0])), int(np.argmin(loop[:, 0]))
    y_back, y_front = float(loop[i_back, 0]), float(loop[i_front, 0])
    # Loops run counter-clockwise in (y, z) from angle -pi: back -> top -> front -> bottom.
    a, b = (i_back, i_front) if upper else (i_front, i_back)
    arc = loop[[(a + k) % n for k in range((b - a) % n + 1)]]
    inside = arc[:, 1] > z_cut if upper else arc[:, 1] < z_cut
    keep = np.nonzero(inside)[0]
    lo, hi = int(keep[0]), int(keep[-1])

    def on_cut(p: np.ndarray, q: np.ndarray) -> np.ndarray:
        t = (z_cut - p[1]) / (q[1] - p[1])
        return p + t * (q - p)

    head = [on_cut(arc[lo - 1], arc[lo])] if lo > 0 else []
    tail = [on_cut(arc[hi], arc[hi + 1])] if hi < len(arc) - 1 else []
    chain = np.vstack(head + [arc[lo : hi + 1]] + tail)
    pts = [Vector(x, y, z) for y, z in chain]
    y_start, y_end = (y_back, y_front) if upper else (y_front, y_back)
    step = -0.05 if upper else 0.05  # keeps the step edge non-degenerate when unclipped
    # Steps out to full width at the chain's ends: horizontal when clipped at z_cut,
    # a short collinear piece of wall when the chain ends on the widest point.
    s_end = Vector(x, y_end, pts[-1].Z + (step if not tail else 0))
    s_start = Vector(x, y_start, pts[0].Z + (step if not head else 0))
    f_end, f_start = Vector(x, y_end, z_far), Vector(x, y_start, z_far)
    return Wire(
        [
            Edge.make_spline(pts),
            Edge.make_line(pts[-1], s_end),
            Edge.make_line(s_end, f_end),
            Edge.make_line(f_end, f_start),
            Edge.make_line(f_start, s_start),
            Edge.make_line(s_start, pts[0]),
        ]
    )


def _grow(loop: np.ndarray, d: float) -> np.ndarray:
    """Move each loop point d outward from the section centroid."""
    c = loop.mean(axis=0)
    r = loop - c
    return c + r * (1 + d / np.linalg.norm(r, axis=1))[:, None]


def bore_tool(p: Params, upper: bool) -> Solid:
    """The bar as one half sees it, radio frame: no undercut, prism lead at the right end."""
    xs, loops = _sections()
    loops = np.stack([_grow(loop, p.bore_clear) for loop in loops])
    face = p.split_z + p.gap / 2 if upper else p.split_z - p.gap / 2
    z_cut = face + p.wall_band if upper else face - p.wall_band
    z_far = p.split_z - 10 if upper else p.split_z + 10
    keep = xs <= p.x1 - p.lead + 1e-6
    stations = [(x, loop) for x, loop in zip(xs[keep], loops[keep], strict=True)]
    # The lead widens toward the end: the bar's x 78 section pokes 0.1 past the x 76 one
    # in places, and a bore that widens toward an end face meets it at an obtuse angle.
    last = stations[-1][1]
    stations.append((p.x1 + 3, _grow(last, p.lead_open)))
    wires = [_draft_wire(x, loop, upper, z_cut, z_far) for x, loop in stations]
    return Solid.make_loft(wires, ruled=True)


# ------------------------------------------------------------------------------ halves


def clamp_x(p: Params) -> float:
    return (p.x0 + p.x1) / 2


def tower_floor(p: Params) -> float:
    return p.corner_peak_z + p.tower_gap


def panel_top(p: Params) -> float:
    return tower_floor(p) + p.cavity_depth + p.panel_t


def _slab(p: Params, z_lo: float, z_hi: float, y_lo: float, y_hi: float) -> Part:
    sketch = RectangleRounded(p.x1 - p.x0, y_hi - y_lo, p.corner_r)
    return Pos(clamp_x(p), (y_lo + y_hi) / 2, z_lo) * extrude(sketch, z_hi - z_lo)


def _break_bore_ends(p: Params, part: Part) -> Part:
    """Entry chamfer on the bore's edges at both end faces.

    The bar's crown runs nearly parallel to the clamp, so the bore meets each end face at a
    grazing angle there; the chamfer also eases the clamp onto the bar.
    """

    def bore_arc_at_end(e) -> bool:
        c = e.center()
        flat = all(abs(v.X - c.X) < 1e-4 for v in e.vertices())
        at_end = min(abs(c.X - p.x0), abs(c.X - p.x1)) < 1e-4
        # The arc along the bar's surface; the short step pieces beside it take no chamfer.
        return flat and at_end and e.geom_type == GeomType.BSPLINE and e.length > 5

    edges = part.edges().filter_by(bore_arc_at_end)
    assert len(edges) == 2, len(edges)
    return chamfer(edges, p.bore_chamfer)


def top_radio(p: Params) -> Part:
    z_lo = p.split_z + p.gap / 2
    ztop = panel_top(p)
    tx, ty = p.tower_x, p.tower_y
    y_lo, y_hi = ty - p.tower_half_y, ty + p.tower_half_y
    part = _slab(p, z_lo, p.ear_top, p.y_front, p.y_back)
    # Spine over the clamp, up to the panel, carrying the tower.
    part += _slab(p, z_lo, ztop, y_lo, y_hi)
    tower = RectangleRounded(2 * p.tower_half_x, 2 * p.tower_half_y, p.corner_r)
    part += Pos(tx, ty, tower_floor(p)) * extrude(tower, ztop - tower_floor(p))
    part -= bore_tool(p, upper=True)
    part = _break_bore_ends(p, part)

    # Captive-nut cavity, open at the tower's floor. Flats face +-X.
    hexagon = RegularPolygon(p.nut_af / 2, 6, major_radius=False, rotation=30)
    part -= Pos(tx, ty, tower_floor(p) - 1) * extrude(
        hexagon, ztop - p.panel_t - tower_floor(p) + 1
    )
    part -= Pos(tx, ty, ztop - p.panel_t - 1) * Cylinder(p.hole_d / 2, p.panel_t + 2, align=C_MIN)
    part -= Pos(tx, ty, ztop + 1) * Cylinder(p.key_d / 2, p.key_depth + 1, align=C_MAX)
    # Wire notch through the tower's front wall, onto the arm.
    part -= Pos(tx, y_lo - 1, tower_floor(p) - 1) * Box(
        p.notch_w, ty - y_lo + 1, p.notch_h + 1, align=(Align.CENTER, Align.MIN, Align.MIN)
    )

    # Screw holes and head counterbores.
    for y in (p.screw_y_front, p.screw_y_back):
        part -= Pos(clamp_x(p), y, z_lo - 1) * Cylinder(
            p.clearance_d / 2, p.ear_top - z_lo + 2, align=C_MIN
        )
        part -= Pos(clamp_x(p), y, p.ear_top + 1) * Cylinder(
            p.cbore_d / 2, p.cbore_depth + 1, align=C_MAX
        )
    return part


def bottom_radio(p: Params) -> Part:
    z_hi = p.split_z - p.gap / 2
    part = _slab(p, p.bottom, z_hi, p.y_front, p.y_back)
    part -= bore_tool(p, upper=False)
    part = _break_bore_ends(p, part)
    for y in (p.screw_y_front, p.screw_y_back):
        part -= Pos(clamp_x(p), y, z_hi + 1) * Cylinder(
            p.insert_d / 2, p.insert_depth + 1, align=C_MAX
        )
    return part


def radio_to_part(p: Params, half: str | None = None) -> Pos:
    """Radio frame to a half's print frame: bed at z 0, origin at the clamp centre."""
    half = half or p.half
    bed = p.split_z + p.gap / 2 if half == "top" else p.bottom
    return Pos(-clamp_x(p), -(p.y_front + p.y_back) / 2, -bed)


def build(p: Params) -> Part:
    if p.half not in ("top", "bottom"):
        raise ValueError(f"half must be 'top' or 'bottom', got {p.half!r}")
    shape = top_radio(p) if p.half == "top" else bottom_radio(p)
    return radio_to_part(p) * shape


def sections(p: Params) -> list[str]:
    mid_y = (p.y_front + p.y_back) / 2
    return [
        "x=0",
        f"x={p.tower_x - clamp_x(p)}",
        f"y={p.tower_y - mid_y}",
        f"y={p.screw_y_front - mid_y}",
        f"y={p.screw_y_back - mid_y}",
    ]


# ----------------------------------------------------------------------------- mates


def receptacle_radio(p: Params) -> Part:
    """HR10A-7R-4S envelope standing on the panel, radio frame (catalog; flange, nut assumed)."""
    flange_t, nut_t = 1.5, 2.0
    body = Pos(0, 0, flange_t) * Cylinder(8.85 / 2, 6.3 - flange_t, align=C_MIN)
    flange = Cylinder(11.0 / 2, flange_t, align=C_MIN)
    thread = Cylinder(7.9 / 2, 7.7, align=C_MAX)
    key = Pos(4.45, 0, 0) * Box(1.1, 1.5, 0.7, align=C_MAX)
    nut = Pos(0, 0, -p.panel_t - nut_t) * extrude(
        RegularPolygon(10.0 / 2, 6, major_radius=False, rotation=30), nut_t
    )
    return Pos(p.tower_x, p.tower_y, panel_top(p)) * (body + flange + thread + key + nut)


def plug_radio(p: Params) -> Part:
    """HR10A-7P-4S mated: Ø11.5 x 35 body (catalog p.6) plus 40 mm of Ø5 cable."""
    body = Pos(0, 0, 1.5) * Cylinder(11.5 / 2, 35.0, align=C_MIN)
    cable = Pos(0, 0, 36.5) * Cylinder(2.5, 40.0, align=C_MIN)
    return Pos(p.tower_x, p.tower_y, panel_top(p)) * (body + cable)


def mates(p: Params) -> dict:
    to_part = radio_to_part(p)
    other = bottom_radio(p) if p.half == "top" else top_radio(p)
    out = {"bar": to_part * bar(), "other_half": to_part * other}
    if p.half == "top":
        out["receptacle"] = to_part * receptacle_radio(p)
        out["plug"] = to_part * plug_radio(p)
    return out


def expect(p: Params) -> dict:
    if p.half == "top":
        return {
            "holes": {p.hole_d: 1, p.key_d: 1, p.clearance_d: 2, p.cbore_d: 2},
            "clearance": {"other_half": p.gap - 0.05, "plug": 0.5},
            "min_wall": 1.2,
        }
    return {
        "holes": {p.insert_d: 2},
        "bbox": (p.x1 - p.x0, p.y_back - p.y_front, p.split_z - p.gap / 2 - p.bottom),
        "clearance": {"other_half": p.gap - 0.05},
        "min_wall": 1.2,
    }
