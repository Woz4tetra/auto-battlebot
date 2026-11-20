"""TX16S MK2 HR10 trainer-port mount bonded on the right shoulder."""

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
    axis_x: float = 95.3
    axis_y: float = -10.5
    tilt_deg: float = 20.0
    front_y: float = -24.0
    u_half: float = 7.5
    v_front: float = -19.0
    v_back: float = 14.0
    taper_v: float = -2.0
    taper_u: float = 5.0
    r_corner: float = 3.0
    r_taper_corner: float = 0.6
    trim_corner_r: float = 1.5
    tape: float = 1.1
    base_chamfer: float = 0.5
    panel_t: float = 2.0
    hole_d: float = 8.3
    key_d: float = 10.6
    key_depth: float = 0.8
    nut_af: float = 10.1
    cavity_depth: float = 11.0
    groove_h: float = 4.5
    edge_chamfer: float = 0.5


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
    return Pos(0, 0, panel_top(p)) * Rot(180, 0, 0) * to_axis


SURFACE_X = (86.5, 107.0)
SURFACE_Y = (-28.0, 10.0)


def shell_slab(p: Params, lift: float = 0.0, depth: float = 40.0) -> Part:
    """Scanned shell top under the block as a solid slab, offset by lift along its normal."""
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
    tight = sk.vertices().filter_by(lambda v: abs(v.X - p.taper_u) < 1e-6)
    assert len(tight) == 1
    sk = fillet(tight, p.r_taper_corner)
    rest = sk.vertices().filter_by(lambda v: abs(abs(v.X) - p.u_half) < 1e-6)
    assert len(rest) == 4, len(rest)
    return fillet(rest, p.r_corner)


PRISM_BELOW_SHELL = 10.0
FDM_MIN_WALL = 1.2


def _chamfer_bed_edge(block, p: Params):
    bed = block.edges().filter_by_position(Axis.Z, -1e-6, 1e-6)
    assert len(bed) == 10, len(bed)
    return chamfer(bed, p.edge_chamfer)


def _contour_base(block, p: Params, to_part):
    return block - to_part * shell_slab(p, lift=p.tape)


def build(p: Params) -> Part:
    h = panel_top(p)
    below = PRISM_BELOW_SHELL
    to_part = radio_to_part(p)
    prism = Pos(0, 0, -below) * extrude(footprint(p), h + below)
    block = Pos(0, 0, h) * Rot(180, 0, 0) * prism

    block = _chamfer_bed_edge(block, p)

    front = Pos(p.axis_x, p.front_y, -60) * Box(
        80, 80, 80, align=(Align.CENTER, Align.MAX, Align.CENTER)
    )
    block -= to_part * front
    trim_dir = (to_part * Pos(0, -1, 0)).position - (to_part * Pos(0, 0, 0)).position
    trim = block.faces().filter_by(lambda f: f.normal_at(f.center()).dot(trim_dir) > 0.99)
    assert len(trim) == 1, len(trim)
    sides = trim[0].edges().filter_by(lambda e: abs(e.tangent_at(0.5).dot(trim_dir)) < 0.1)
    sides = sides.filter_by(lambda e: abs(e.center().X) > p.u_half - 0.1)
    assert len(sides) == 2, len(sides)
    block = fillet(sides, p.trim_corner_r)

    block = _contour_base(block, p, to_part)

    hexagon = RegularPolygon(p.nut_af / 2, 6, major_radius=False)
    block -= Pos(0, 0, p.panel_t) * extrude(hexagon, h + below)
    block -= Pos(0, 0, -1) * Cylinder(p.hole_d / 2, p.panel_t + 2, align=C_MIN)
    block -= Pos(0, 0, -1) * Cylinder(p.key_d / 2, p.key_depth + 1, align=C_MIN)

    o = axis_origin(p)
    roof = min(
        (to_part * Pos(o[0], y, shell_z(o[0], y) + p.groove_h)).position.Z
        for y in np.arange(o[1], SURFACE_Y[1], 0.5)
    )
    flat = p.nut_af / np.sqrt(3)
    block -= Pos(0, 0, roof) * Box(flat, 40, 40, align=(Align.CENTER, Align.MAX, Align.MIN))

    base = block.faces().filter_by(GeomType.BSPLINE).sort_by(lambda f: f.area)[-1]
    assert base.area > 150, base.area
    return chamfer(base.edges(), p.base_chamfer)


def sections(p: Params) -> list[str]:
    return ["x=0", "y=0", f"z={p.key_depth / 2}", f"z={p.panel_t + 1}"]


def receptacle(p: Params) -> Part:
    """HR10A-7R-4S envelope in the part frame: catalog diameters, flange and nut assumed."""
    flange_t, nut_t = 1.5, 2.0
    front = Pos(0, 0, -flange_t) * Cylinder(
        8.85 / 2, 6.3 - flange_t, align=(Align.CENTER, Align.CENTER, Align.MAX)
    )
    flange = Cylinder(11.0 / 2, flange_t, align=(Align.CENTER, Align.CENTER, Align.MAX))
    thread = Cylinder(7.9 / 2, 7.7, align=C_MIN)
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
        "min_wall": FDM_MIN_WALL,
    }
