"""Cable connector envelopes for the cage pack assembly, traced from datasheet drawings.

Exact vendor models are used wherever one could be downloaded (vendor/: KiCad's 3D library
for the PCBA parts, Mini-Box's OpenUPS STEP, Stereolabs' ZED Box Mini and ZED X One S). The
parts below have no free model, so each is an envelope from its datasheet's drawing, close
enough for clearance and cable routing, not for mating geometry.

Every function returns a part in a local frame: origin on the mating plane at the contact
axis, +Z pointing away from the mating partner (out of a panel, up off a header, back along
the cable). `place(shape, origin, z_dir, x_dir)` puts one in the assembly.

| Part | Source |
| --- | --- |
| GX16-8 panel receptacle, cable plug | generic GX16 drawing (M16 x 1, flange Ø19, plug Ø19 x 45) |
| GX12-2 panel receptacle, cable plug | generic GX12 drawing (assembly uses vendor/grabcad) |
| CIT AH power button | CIT AH datasheet, latching (bezel Ø21.8 x 2.0, M19 body, 31.8 deep) |
| Same Sky TBP01P1W-508-02GR | datasheet p.2: 10.16 wide, 15.00 tall, 18.00 deep, flanges 15.24 |
| Mini-Fit Jr receptacle housings | 4.2 pitch, 9.6 x 9.6 (2x2), 9.6 x 5.4 (1x2), 13.6 tall |
| JST PH / GH housings | JST catalog outlines |
| USB plugs (A 90°, C 90°, mini-B 90°) | generic overmolds |
| FAKRA-Z plug (Stereolabs GMSL2 cable) | generic FAKRA housing 9.6 square |
| Lever suction cup with 1/4-20 stud | generic 4.5" (114 mm) cup, e.g. Delvie's Plus |
"""

from build123d import (
    Align,
    Box,
    Circle,
    Cone,
    Cylinder,
    FilletPolyline,
    Location,
    Part,
    Plane,
    Polyline,
    Pos,
    RegularPolygon,
    Rot,
    Transition,
    extrude,
    sweep,
)

C_MIN = (Align.CENTER, Align.CENTER, Align.MIN)
C_MAX = (Align.CENTER, Align.CENTER, Align.MAX)


def place(shape, origin, z_dir, x_dir):
    return Location(Plane(origin=origin, x_dir=x_dir, z_dir=z_dir)) * shape


def _hex(af: float, t: float, z0: float) -> Part:
    return Pos(0, 0, z0) * extrude(RegularPolygon(af / 2, 6, major_radius=False), t)


# Panel connectors. Frame: origin on the panel's outer face, +Z OUT of the enclosure.
def gx16_panel(wall: float) -> Part:
    flange = Cylinder(19.0 / 2, 2.0, align=C_MIN)
    shroud = Pos(0, 0, 2.0) * Cylinder(16.0 / 2, 6.0, align=C_MIN)
    body = Cylinder(15.8 / 2, 18.0, align=C_MAX)
    nut = _hex(20.0, 3.0, -wall - 3.0)
    cups = Pos(0, 0, -18.0) * Cylinder(12.0 / 2, 6.0, align=C_MAX)
    return flange + shroud + body + nut + cups


def gx16_plug(cable_d: float = 7.0) -> Part:
    """Mated on gx16_panel: coupling nut over the shroud, from the flange outward."""
    body = Pos(0, 0, 2.0) * Cylinder(19.0 / 2, 45.0, align=C_MIN)
    boot = Pos(0, 0, 47.0) * Cone(10.0 / 2, cable_d / 2, 8.0, align=C_MIN)
    return body + boot


def gx12_panel(wall: float) -> Part:
    flange = Cylinder(15.0 / 2, 2.0, align=C_MIN)
    shroud = Pos(0, 0, 2.0) * Cylinder(12.0 / 2, 5.0, align=C_MIN)
    body = Cylinder(11.8 / 2, 15.0, align=C_MAX)
    nut = _hex(16.0, 2.5, -wall - 2.5)
    cups = Pos(0, 0, -15.0) * Cylinder(9.0 / 2, 5.0, align=C_MAX)
    return flange + shroud + body + nut + cups


def gx12_plug(cable_d: float = 5.5) -> Part:
    """GX12 cable plug mated on gx12_panel: Ø15 x 30 body over the shroud, then a boot down
    to the cable. Generic GX12 drawing, about +-1 mm."""
    body = Pos(0, 0, 2.0) * Cylinder(15.0 / 2, 30.0, align=C_MIN)
    boot = Pos(0, 0, 32.0) * Cone(12.0 / 2, cable_d / 2, 10.0, align=C_MIN)
    return body + boot


def dupont_2p() -> Part:
    """2-pin 0.1" (2.54 mm) crimp housing on a vertical pin header: 5.1 x 2.6 x 14, starting
    2.5 above the board (the header's plastic base)."""
    return Pos(0, 0, 2.5) * Box(5.1, 2.6, 14.0, align=C_MIN)


def gx12_cap() -> Part:
    """Dust cap on the charge input, which is empty while worn."""
    return Pos(0, 0, 2.0) * Cylinder(15.0 / 2, 9.0, align=C_MIN)


def cit_ah_button(wall: float) -> Part:
    """CIT AH 19 mm anti-vandal pushbutton, latching, quick-connect tabs (AH1LBBG12).

    Frame: origin on the wall's outer face at the hole's centre, +Z out through the wall.
    Bezel Ø21.8 x 2.0 with the Ø15.4 actuator flush in it; M19 body 31.8 deep from the
    bezel's face, the last 8.0 being the solder tabs. Nut 3.0 thick (not on the datasheet).
    """
    import cage_layout as cl

    bd, bt = cl.SWITCH_BEZEL
    nd, nt = cl.SWITCH_NUT
    bezel = Cylinder(bd / 2, bt, align=C_MIN)
    cap = Pos(0, 0, bt) * Cylinder(cl.SWITCH_CAP / 2, 0.5, align=C_MIN)
    body = Cylinder(19.0 / 2, cl.SWITCH_DEPTH - 8.0, align=C_MAX)
    tabs = Pos(0, 0, -(cl.SWITCH_DEPTH - 8.0)) * Box(12.0, 12.0, 8.0, align=C_MAX)
    nut = Pos(0, 0, -wall) * Cylinder(nd / 2, nt, align=C_MAX)
    return bezel + cap + body + tabs + nut


# Wire-to-board and wire-to-device plugs. Frame: origin on the mating face, +Z away from
# the partner (the direction the plug comes off).
def tbp_plug_2p(insert: float = 8.0) -> Part:
    """Same Sky TBP01P1W-508-02GR: 2-pole 5.08 mm pluggable plug with locking flanges.
    X along the pole pitch, Y the 15.00 height, Z from the mating face back to the wire
    entry (18.00). `insert` of it sits inside the header shroud."""
    body = Pos(0, 0, -insert) * Box(10.16, 15.0, 18.0, align=C_MIN)
    ears = Pos(0, -3.3, -insert) * Box(15.24 + 5.0, 8.4, 6.0, align=C_MIN)
    screws = [
        Pos(sx * (15.24 / 2 + 1.27), -3.3, -insert + 6.0) * Cylinder(2.2, 2.0, align=C_MIN)
        for sx in (-1, 1)
    ]
    return body + ears + screws[0] + screws[1]


def minifit_plug(cols: int, rows: int) -> Part:
    """Molex Mini-Fit Jr receptacle housing over a vertical header: 4.2 mm pitch, starts 3.0
    above the board (the header's base), 13.6 tall."""
    w, d = cols * 4.2 + 1.2, rows * 4.2 + 1.2
    housing = Pos(0, 0, 3.0) * Box(w, d, 13.6, align=C_MIN)
    latch = Pos(0, d / 2, 8.0) * Box(3.0, 2.0, 6.0, align=(Align.CENTER, Align.MIN, Align.MIN))
    return housing + latch


def ph_plug(n: int) -> Part:
    """JST PHR housing on a vertical PH header: 2.0 pitch, starts 1.5 above the board."""
    return Pos(0, 0, 1.5) * Box(n * 2.0 + 1.9, 4.5, 6.0, align=C_MIN)


def gh_plug(n: int) -> Part:
    """JST GHR housing: 1.25 pitch, 4.25 deep along its mating axis."""
    return Box(n * 1.25 + 1.75, 3.0, 6.5, align=C_MIN)


def usb_a_plug_90() -> Part:
    """USB-A plug, cable leaving at 90°: shell 12 into the receptacle, 22 head, cable +X."""
    shell = Box(12.0, 4.5, 12.0, align=C_MAX)
    head = Box(16.0, 9.0, 22.0, align=C_MIN)
    return shell + head


def usb_c_plug_90() -> Part:
    shell = Box(8.3, 2.5, 6.5, align=C_MAX)
    head = Box(12.0, 6.0, 14.0, align=C_MIN)
    return shell + head


def mini_b_plug_90() -> Part:
    shell = Box(7.0, 3.5, 6.0, align=C_MAX)
    head = Box(9.0, 7.0, 12.0, align=C_MIN)
    return shell + head


def fakra_plug() -> Part:
    """FAKRA-Z plug over the jack: housing from the jack's base face outward, boot, cable."""
    housing = Box(9.6, 9.6, 26.0, align=C_MIN)
    boot = Pos(0, 0, 26.0) * Cone(4.5, 3.0, 8.0, align=C_MIN)
    return housing + boot


def suction_cup() -> Part:
    """Lever suction cup, generic 4.5" (114 mm): cup face on the glass at Z = 0, 1/4-20
    male stud on top along +Z. Total 52 tall to the stud's tip."""
    cup = Cone(114.0 / 2, 52.0 / 2, 12.0, align=C_MIN)
    housing = Pos(0, 0, 12.0) * Cylinder(60.0 / 2, 18.0, align=C_MIN)
    lever = (
        Pos(18.0, 0, 30.0)
        * Rot(0, -8, 0)
        * Box(70.0, 22.0, 6.0, align=(Align.MIN, Align.CENTER, Align.MIN))
    )
    boss = Pos(0, 0, 30.0) * Cylinder(14.0 / 2, 12.0, align=C_MIN)
    stud = Pos(0, 0, 42.0) * Cylinder(6.35 / 2, 10.0, align=C_MIN)
    return cup + housing + lever + boss + stud


# Cables: a circle swept along a polyline with rounded corners.
def tube(points: list[tuple[float, float, float]], d: float, bend_r: float | None = None) -> Part:
    """Cable centreline through `points`, corners rounded to `bend_r` (default 3 x d), cut
    down to fit the shortest segment."""
    pts = [p for i, p in enumerate(points) if i == 0 or p != points[i - 1]]
    last = len(pts) - 1
    pts = [
        p for i, p in enumerate(pts) if i in (0, last) or not _collinear(pts[i - 1], p, pts[i + 1])
    ]
    segs = [sum((b[k] - a[k]) ** 2 for k in range(3)) ** 0.5 for a, b in zip(pts, pts[1:])]
    r = min(bend_r or 3.0 * d, 0.45 * min(segs))
    path = None
    while path is None and len(pts) > 2 and r > 0.5:
        try:
            path = FilletPolyline(*pts, radius=r)
        except (ValueError, IndexError):  # IndexError: collinear points, no corner to round
            r *= 0.6
    if path is None:
        path = Polyline(*pts)
    start = path.edges()[0]
    plane = Plane(origin=start.position_at(0), z_dir=start.tangent_at(0))
    return sweep(plane * Circle(d / 2), path=path, transition=Transition.ROUND)


def _collinear(a, b, c) -> bool:
    u = [b[k] - a[k] for k in range(3)]
    v = [c[k] - b[k] for k in range(3)]
    cx = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
    return sum(x * x for x in cx) < 1e-9 * (sum(x * x for x in u) * sum(x * x for x in v) + 1e-12)
