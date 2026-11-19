"""TX16S MK2 HR10 handle clamp, two printed halves on the carry-handle bar."""

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
THREAD_LEN = 7.7
CABLE_OD = 3.5
INSERT_LEN = 5.7
MATERIAL = "pla"

SECTIONS = Path(__file__).resolve().parent / "tx16s_handle_bar_sections.npz"
C_MIN = (Align.CENTER, Align.CENTER, Align.MIN)
C_MAX = (Align.CENTER, Align.CENTER, Align.MAX)


@dataclass
class Params:
    half: str = "top"
    x0: float = 61.0
    x1: float = 77.0
    split_z: float = -29.0
    gap: float = 0.8
    lead: float = 1.0
    bore_clear: float = 0.2
    lead_open: float = 0.3
    wall_band: float = 0.15
    bore_chamfer: float = 0.4
    y_front: float = 38.5
    y_back: float = 75.5
    ear_top: float = -17.0
    bottom: float = -37.5
    corner_r: float = 3.0
    tower_x: float = 85.0
    tower_y: float = 58.0
    tower_half_x: float = 8.0
    tower_half_y: float = 7.83
    tower_gap: float = 1.5
    corner_peak_z: float = -21.1
    panel_t: float = 2.0
    hole_d: float = 8.3
    key_d: float = 10.6
    key_depth: float = 0.8
    nut_af: float = 10.1
    cavity_depth: float = THREAD_LEN + 2.0
    notch_w: float = CABLE_OD + 1.5
    notch_h: float = 4.5
    screw_y_front: float = 44.0
    screw_y_back: float = 70.0
    m3_clearance_dia: float = 3.4
    m3_cbore_dia: float = 6.5
    cbore_depth: float = 3.3
    insert_hole_dia: float = 4.0
    insert_depth: float = INSERT_LEN + 1.0


@lru_cache(maxsize=1)
def _sections() -> tuple[np.ndarray, np.ndarray]:
    d = np.load(SECTIONS)
    return d["x"], d["loops"]


def _loop_wire(x: float, loop: np.ndarray) -> Wire:
    return Wire([Edge.make_spline([Vector(x, y, z) for y, z in loop], periodic=True)])


def bar() -> Solid:
    """Handle bar from x 58 to 80, lofted through the scan sections."""
    xs, loops = _sections()
    wires = [_loop_wire(x, loop) for x, loop in zip(xs, loops, strict=True)]
    return Solid.make_loft(wires, ruled=True)


def _draft_wire(x: float, loop: np.ndarray, upper: bool, z_cut: float, z_far: float) -> Wire:
    """One half's bore profile at one station."""
    n = len(loop)
    i_back, i_front = int(np.argmax(loop[:, 0])), int(np.argmin(loop[:, 0]))
    y_back, y_front = float(loop[i_back, 0]), float(loop[i_front, 0])
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
    last = stations[-1][1]
    stations.append((p.x1 + 3, _grow(last, p.lead_open)))
    wires = [_draft_wire(x, loop, upper, z_cut, z_far) for x, loop in stations]
    return Solid.make_loft(wires, ruled=True)


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
    """Entry chamfer where the bore meets the end faces at a grazing angle."""

    def bore_arc_at_end(e) -> bool:
        c = e.center()
        flat = all(abs(v.X - c.X) < 1e-4 for v in e.vertices())
        at_end = min(abs(c.X - p.x0), abs(c.X - p.x1)) < 1e-4
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
    part += _slab(p, z_lo, ztop, y_lo, y_hi)
    tower = RectangleRounded(2 * p.tower_half_x, 2 * p.tower_half_y, p.corner_r)
    part += Pos(tx, ty, tower_floor(p)) * extrude(tower, ztop - tower_floor(p))
    part -= bore_tool(p, upper=True)
    part = _break_bore_ends(p, part)

    hexagon = RegularPolygon(p.nut_af / 2, 6, major_radius=False, rotation=30)
    part -= Pos(tx, ty, tower_floor(p) - 1) * extrude(
        hexagon, ztop - p.panel_t - tower_floor(p) + 1
    )
    part -= Pos(tx, ty, ztop - p.panel_t - 1) * Cylinder(p.hole_d / 2, p.panel_t + 2, align=C_MIN)
    part -= Pos(tx, ty, ztop + 1) * Cylinder(p.key_d / 2, p.key_depth + 1, align=C_MAX)
    part -= Pos(tx, y_lo - 1, tower_floor(p) - 1) * Box(
        p.notch_w, ty - y_lo + 1, p.notch_h + 1, align=(Align.CENTER, Align.MIN, Align.MIN)
    )

    for y in (p.screw_y_front, p.screw_y_back):
        part -= Pos(clamp_x(p), y, z_lo - 1) * Cylinder(
            p.m3_clearance_dia / 2, p.ear_top - z_lo + 2, align=C_MIN
        )
        part -= Pos(clamp_x(p), y, p.ear_top + 1) * Cylinder(
            p.m3_cbore_dia / 2, p.cbore_depth + 1, align=C_MAX
        )
    return part


def bottom_radio(p: Params) -> Part:
    z_hi = p.split_z - p.gap / 2
    part = _slab(p, p.bottom, z_hi, p.y_front, p.y_back)
    part -= bore_tool(p, upper=False)
    part = _break_bore_ends(p, part)
    for y in (p.screw_y_front, p.screw_y_back):
        part -= Pos(clamp_x(p), y, z_hi + 1) * Cylinder(
            p.insert_hole_dia / 2, p.insert_depth + 1, align=C_MAX
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
            "holes": {p.hole_d: 1, p.key_d: 1, p.m3_clearance_dia: 2, p.m3_cbore_dia: 2},
            "clearance": {"other_half": p.gap - 0.05, "plug": 0.5},
            "min_wall": 1.2,
        }
    return {
        "holes": {p.insert_hole_dia: 2},
        "bbox": (p.x1 - p.x0, p.y_back - p.y_front, p.split_z - p.gap / 2 - p.bottom),
        "clearance": {"other_half": p.gap - 0.05},
        "min_wall": 1.2,
    }
