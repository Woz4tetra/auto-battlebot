"""TX16S MK2 HR10 trainer-port mount: holds an HR10A-7R-4S receptacle on top of the radio.

The Jetson trainer link runs on the radio's AUX1 UART (bottom front face). A pigtail from
AUX1 runs up the outside of the shell to this block, which stands the receptacle on top of
the right shoulder block, tilted back so the mating plug and its cable lead away from the
pilot. The block is bonded with VHB tape to a contour taken from a 3D scan of the radio:
the right shoulder block has no screws to borrow, and the module bay, which may change,
is the only other flat area on top.

Spec
- Process: FDM PLA, panel face on the bed, contoured base up. No supports.
- Frame: origin at the receptacle's panel-hole centre on the panel face; +Z runs into the
  block toward the radio (print direction). Radio frame (scan_patch.py: +X pilot's right,
  +Y away from pilot, +Z up) maps here by radio_to_part().
- Placement: receptacle axis meets the shell at radio (95.3, -10.5), tilted 20 deg back
  about radio X. Footprint stays in x 87.8..102.8: ahead of the carry-handle hinge arm
  (x 81..88, only behind y 12) and inboard of the two outer shoulder toggles. Toggle A's
  boss flank starts at x 102 for y -2..6, so the back-right corner tapers in. The front
  wall is vertical at radio y = -24, ahead of the dome's front roll-off. The module bay
  (x < 70) is 18 mm away. verify_scan.py checks all of this against the raw scan.
- Interfaces:
  - Shell: base is the scan height map (tx16s_top_right_heightmap.npz) offset 1.1 mm
    along its normal for 3M VHB 5952 (1.1 mm, made for painted and low-energy plastics).
  - HR10A-7R-4S(73), Hirose HR10 catalog p.7 and p.20: panel hole 8.1 +0.1, key slot
    1.6 +0.1 to 5.1 from centre, panel 0.7 to 2.0 thick, M8x0.5 nut 10 AF.
  - Captive nut: the M8x0.5 nut sits in a 10.1 AF hex cavity under the panel, open to
    the base, and the receptacle threads down into it. A nut under a keyed panel needs a
    10 mm socket (Ø15) or pliers to turn it, and neither fits a block this narrow. The
    key tab turns with the receptacle, so it rides in a Ø10.6 x 0.8 counterbore under
    the flange instead of a slot (tab length <= 0.7, the catalog's thinnest panel).
  - Hole Ø8.3: catalog 8.1 +0.1, +0.2 FDM undersize (fdm.md).
  - Wire exit: a groove the full width of the hex flat (5.83) from the cavity out the
    back wall. Flat roof, at least 4.5 mm over the shell along its run.
- Critical dimensions: panel 2.0 thick (1.2 web under the counterbore); hole Ø8.3;
  counterbore Ø10.6 x 0.8; hex cavity 10.1 AF, 11 deep from the tape face at the axis,
  a flat toward the groove; walls 1.67 at the hex corners (+-X); 0.5 chamfer on every
  base edge.
- Edge geometry: the walls run along the tilted axis and the shell slopes away under
  them, so every base edge, groove edge and corner here was placed to keep check.py's
  1.2 mm minimum wall. The chamfers are fragile: base_chamfer 0.45 or 0.55, or
  taper_u 4.5, fails to build.

Assembly
1. Solder nothing yet. Push the nut into the hex cavity from the base and hold it up
   against the panel with a pencil.
2. Thread the receptacle in from the top by its body until the flange seats. A drop of
   medium threadlocker on the thread.
3. Solder the pigtail to the cups through the base, lay it in the groove.
4. Clean the shell with IPA, VHB 5952 on the base, press 30 s, wait 24 h before loading.

Open questions
- Handle swing: the scan shows the handle up and back. If it folds forward over the top,
  check the plug clears its arm (verify_scan.py reports the gap to the scanned arm only).
- Key tab length and the flange's bearing ring: the counterbore leaves the flange 0.2 mm
  of PLA to sit on (r 5.3 to 5.5). Print to 3 mm height first and seat a receptacle; if
  the tab is longer than 0.8, raise key_depth (and panel_t cannot follow past 2.0).
- Receptacle flange thickness and nut thickness are not in the catalog; the mate models
  them as 1.5 and 2.0.
- AUX1 connector type and pin order: meter the port before wiring the pigtail.
"""

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np
from build123d import (
    Align,
    Axis,
    Box,
    Cylinder,
    Face,
    GeomType,
    Location,
    Part,
    Polygon,
    Pos,
    RegularPolygon,
    Rot,
    Solid,
    Vector,
    chamfer,
    extrude,
    fillet,
)

PROCESS = "fdm"
MATERIAL = "pla"

HEIGHTMAP = Path(__file__).resolve().parent / "tx16s_top_right_heightmap.npz"
C_MIN = (Align.CENTER, Align.CENTER, Align.MIN)


@dataclass
class Params:
    # Placement, radio frame (scan_patch.py)
    axis_x: float = 95.3  # between hinge arm and toggle A's boss (from x 102)
    axis_y: float = -10.5  # nearer toggle B (y -20, flank x 106) than A (y +2, flank x 102)
    tilt_deg: float = 20.0  # plug leans back, away from the pilot
    front_y: float = -24.0  # vertical front wall, ahead of the dome roll-off at y -26
    # Footprint across the axis: u along radio X, v toward the back
    u_half: float = 7.5  # hex 11.66 across corners + 1.67 walls
    v_front: float = -19.0  # front trim crosses the bed here, so the bed edge stays square
    v_back: float = 14.0  # back edge, short of the shell's back drop at y = 11
    taper_v: float = -2.0  # back-right taper starts here...
    taper_u: float = 5.0  # ...and ends here at v_back: toggle A's boss flank reaches x 102
    r_corner: float = 3.0
    r_taper_corner: float = 0.6  # back-right: arc starts outboard of the groove wall (x 2.92)
    trim_corner_r: float = 1.5  # front trim's side edges; 2.0 leaves the base chamfer no room
    # Bond
    tape: float = 1.1  # 3M VHB 5952 thickness
    base_chamfer: float = 0.5  # breaks the acute edges where the tilted base meets the walls
    # Receptacle, HR10 catalog
    panel_t: float = 2.0  # catalog 0.7 to 2.0; the max, for a 1.2 web under the counterbore
    hole_d: float = 8.3  # 8.1 +0.1 catalog, +0.2 FDM undersize (fdm.md)
    key_d: float = 10.6  # key tab tip r 5.1, +0.1 catalog, +0.1 FDM
    key_depth: float = 0.8  # tab <= 0.7, the catalog's thinnest panel
    nut_af: float = 10.1  # 10 AF nut, snug sliding fit (fdm.md)
    cavity_depth: float = 11.0  # tape face to panel: 7.7 behind the panel + wire bend
    # Wire exit
    groove_h: float = 4.5  # least clear height over the shell along the groove; 3.5 OD cable
    edge_chamfer: float = 0.5  # elephant's foot on the bed edge, fdm.md


# ---------------------------------------------------------------------------- frames


def axis_frame(p: Params) -> np.ndarray:
    """Rows: the block's u (radio X), v (back, along the shell) and w (receptacle axis)."""
    t = np.radians(p.tilt_deg)
    return np.array([[1.0, 0.0, 0.0], [0.0, np.cos(t), -np.sin(t)], [0.0, np.sin(t), np.cos(t)]])


@lru_cache(maxsize=1)
def _heightmap() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    d = np.load(HEIGHTMAP)
    return d["x"], d["y"], d["z"]


def shell_z(x: float, y: float) -> float:
    """Bilinear height of the scanned shell top, radio frame."""
    xs, ys, z = _heightmap()
    fx = np.interp(x, xs, np.arange(len(xs)))
    fy = np.interp(y, ys, np.arange(len(ys)))
    i, j = int(fx), int(fy)
    a, b = fx - i, fy - j
    return float(
        (1 - a) * (1 - b) * z[i, j]
        + a * (1 - b) * z[i + 1, j]
        + (1 - a) * b * z[i, j + 1]
        + a * b * z[i + 1, j + 1]
    )


def axis_origin(p: Params) -> np.ndarray:
    """Where the receptacle axis meets the scanned shell, radio frame."""
    return np.array([p.axis_x, p.axis_y, shell_z(p.axis_x, p.axis_y)])


def panel_top(p: Params) -> float:
    """Panel face height along the axis, measured from the shell point."""
    return p.tape + p.cavity_depth + p.panel_t


def radio_to_part(p: Params) -> Location:
    """Rigid transform taking radio-frame geometry into this part's frame."""
    r = axis_frame(p)
    o = axis_origin(p)
    t = -r @ o
    from OCP.gp import gp_Trsf

    trsf = gp_Trsf()
    trsf.SetValues(*r[0], t[0], *r[1], t[1], *r[2], t[2])
    to_axis = Location(trsf)
    # Axis frame -> part frame: flip so +Z points into the block, origin on the panel face.
    return Pos(0, 0, panel_top(p)) * Rot(180, 0, 0) * to_axis


# Height map region the base follows. Outside it the slab continues flat from its edge, so
# the tilted prism always ends inside the slab; a B-spline through the real back-edge drop
# and toggle bosses overshoots by centimetres. Inside, the spline is within 0.3 mm of the
# map except the y = -27.5 and 8.5 edge rows (0.5 to 0.9 mm above it: a gap, never a clash).
SURFACE_X = (86.5, 107.0)
SURFACE_Y = (-28.0, 10.0)


def shell_slab(p: Params, lift: float = 0.0, depth: float = 40.0) -> Part:
    """The scanned shell top under the block as a solid slab, radio frame.

    `lift` offsets the top along its normal, so the tape gap holds on the slopes too.
    """
    xs = np.arange(SURFACE_X[0] - 8, SURFACE_X[1] + 8.01, 1.0)
    ys = np.arange(SURFACE_Y[0] - 8, SURFACE_Y[1] + 8.01, 1.0)
    rows = [[Vector(x, y, _clamped_z(x, y)) for y in ys] for x in xs]
    top = Face.make_surface_from_array_of_points(rows, tol=0.05)
    slab = Solid.extrude(top, Vector(0, 0, -depth))
    if lift:
        upper = slab.faces().sort_by(Axis.Z)[-1]
        slab = Solid.extrude(upper.offset(lift), Vector(0, 0, -depth - lift))
    return Part(slab.wrapped)


def _clamped_z(x: float, y: float) -> float:
    return shell_z(float(np.clip(x, *SURFACE_X)), float(np.clip(y, *SURFACE_Y)))


# ------------------------------------------------------------------------------ part


def footprint(p: Params):
    """Cross-section across the axis. The back-right tapers in to clear toggle A's boss."""
    corners = [
        (-p.u_half, p.v_front),
        (p.u_half, p.v_front),
        (p.u_half, p.taper_v),
        (p.taper_u, p.v_back),
        (-p.u_half, p.v_back),
    ]
    sk = Polygon(*corners, align=None)
    assert len(sk.vertices()) == 5
    # The back-right corner gets a tight radius so its arc starts outboard of the wire
    # groove's wall (x 2.92); a wall that clips the arc leaves a knife edge.
    tight = sk.vertices().filter_by(lambda v: abs(v.X - p.taper_u) < 1e-6)
    assert len(tight) == 1
    sk = fillet(tight, p.r_taper_corner)
    rest = sk.vertices().filter_by(lambda v: abs(abs(v.X) - p.u_half) < 1e-6)
    assert len(rest) == 4, len(rest)
    return fillet(rest, p.r_corner)


def build(p: Params) -> Part:
    h = panel_top(p)
    below = (
        10.0  # prism runs this far under the shell point: past the 6 mm dome drop, inside the slab
    )
    to_part = radio_to_part(p)
    # Prism along the axis (axis frame: w up the receptacle axis), then into the part frame.
    prism = Pos(0, 0, -below) * extrude(footprint(p), h + below)
    block = Pos(0, 0, h) * Rot(180, 0, 0) * prism

    # Bed edge (panel face, z = 0 in the part frame) before any cut reaches it.
    bed = block.edges().filter_by_position(Axis.Z, -1e-6, 1e-6)
    assert len(bed) == 10, len(bed)  # 5 sides + 5 corner arcs
    block = chamfer(bed, p.edge_chamfer)

    # Vertical front wall in the radio frame. It leans 20 deg from the print axis, toward
    # the part, so it prints without support.
    front = Pos(p.axis_x, p.front_y, -60) * Box(
        80, 80, 80, align=(Align.CENTER, Align.MAX, Align.CENTER)
    )
    block -= to_part * front
    # Round the trim's two side edges: square ones meet the tilted base in a sharp
    # three-way corner.
    trim_dir = (to_part * Pos(0, -1, 0)).position - (to_part * Pos(0, 0, 0)).position
    trim = block.faces().filter_by(lambda f: f.normal_at(f.center()).dot(trim_dir) > 0.99)
    assert len(trim) == 1, len(trim)
    sides = trim[0].edges().filter_by(lambda e: abs(e.tangent_at(0.5).dot(trim_dir)) < 0.1)
    sides = sides.filter_by(lambda e: abs(e.center().X) > p.u_half - 0.1)
    assert len(sides) == 2, len(sides)
    block = fillet(sides, p.trim_corner_r)

    # Contour the base: everything below the shell plus the tape gap goes.
    block -= to_part * shell_slab(p, lift=p.tape)

    # Captive-nut cavity under the panel, open through the base. A flat faces the wire
    # groove (part -Y): a groove leaving through a hex corner leaves knife edges.
    hexagon = RegularPolygon(p.nut_af / 2, 6, major_radius=False)
    block -= Pos(0, 0, p.panel_t) * extrude(hexagon, h + below)
    # Thread hole, and the counterbore the key tab turns in.
    block -= Pos(0, 0, -1) * Cylinder(p.hole_d / 2, p.panel_t + 2, align=C_MIN)
    block -= Pos(0, 0, -1) * Cylinder(p.key_d / 2, p.key_depth + 1, align=C_MIN)

    # Wire groove from the cavity out the back wall. The roof is flat in the part frame,
    # square to the hex flat it leaves through (a roof following the shell meets that flat
    # at an acute angle), and sits at the lowest roof that keeps groove_h over the shell
    # along the whole run. The groove is as wide as the hex flat, so its walls carry on
    # from the neighbouring hex faces with no short edges between.
    o = axis_origin(p)
    roof = min(
        (to_part * Pos(o[0], y, shell_z(o[0], y) + p.groove_h)).position.Z
        for y in np.arange(o[1], SURFACE_Y[1], 0.5)
    )
    flat = p.nut_af / np.sqrt(3)
    block -= Pos(0, 0, roof) * Box(flat, 40, 40, align=(Align.CENTER, Align.MAX, Align.MIN))

    # Edge break around the contoured base. The walls run along the tilted axis and the
    # shell slopes away under them, so the base meets them at down to 55 deg: a feather
    # edge that tears off the tape. Chamfer every edge of the base face, cavity rim included.
    # The base is the largest B-spline face; the trim fillets come out as B-splines too.
    base = block.faces().filter_by(GeomType.BSPLINE).sort_by(lambda f: f.area)[-1]
    assert base.area > 150, base.area
    return chamfer(base.edges(), p.base_chamfer)


def sections(p: Params) -> list[str]:
    return ["x=0", "y=0", f"z={p.key_depth / 2}", f"z={p.panel_t + 1}"]


# ----------------------------------------------------------------------------- mates


def receptacle(p: Params) -> Part:
    """HR10A-7R-4S envelope in the part frame: catalog diameters, flange and nut assumed."""
    flange_t, nut_t = 1.5, 2.0
    front = Pos(0, 0, -flange_t) * Cylinder(
        8.85 / 2, 6.3 - flange_t, align=(Align.CENTER, Align.CENTER, Align.MAX)
    )
    flange = Cylinder(11.0 / 2, flange_t, align=(Align.CENTER, Align.CENTER, Align.MAX))
    thread = Cylinder(7.9 / 2, 7.7, align=C_MIN)
    # Key tab: r 3.9 to 5.0, 1.5 wide, 0.7 long; it ends up wherever the thread stops.
    key = Pos(0, -4.45, 0) * Box(1.5, 1.1, 0.7, align=C_MIN)
    nut = Pos(0, 0, p.panel_t) * extrude(RegularPolygon(10.0 / 2, 6, major_radius=False), nut_t)
    return front + flange + thread + key + nut


def plug(p: Params) -> Part:
    """HR10A-7P-4S mated: Ø11.5 x 35 body (catalog p.6) plus 40 mm of Ø5 cable."""
    body = Pos(0, 0, -1.5) * Cylinder(11.5 / 2, 35.0, align=(Align.CENTER, Align.CENTER, Align.MAX))
    cable = Pos(0, 0, -36.5) * Cylinder(2.5, 40.0, align=(Align.CENTER, Align.CENTER, Align.MAX))
    return body + cable


def mates(p: Params) -> dict:
    return {
        "shell": radio_to_part(p) * shell_slab(p),
        "receptacle": receptacle(p),
        "plug": plug(p),
    }


def expect(p: Params) -> dict:
    return {
        "holes": {p.hole_d: 1, p.key_d: 1},
        "clearance": {"shell": p.tape - 0.05, "plug": 0.5},
        "allow_interference": [],
        "min_wall": 1.2,  # fdm.md minimum
    }
