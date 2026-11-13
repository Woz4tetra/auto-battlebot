"""Hirose HR10 4-pin envelopes: HR10A-7R-4S panel receptacle and HR10A-7P-4P cable plug.

Same catalog numbers as hardware/tx16s_hr10_mount/tx16s_hr10_mount.py (Hirose HR10 catalog
p.6, p.7, p.20). Hirose gates its STEP downloads behind a login, so these are traced from the
catalog drawings, not exact models.

Frame: origin on the panel's front face at the receptacle axis, +Z into the enclosure (the
thread and nut side), -Z out toward the mating plug.
"""

from build123d import Align, Box, Cylinder, Part, Pos, RegularPolygon, extrude

C_MIN = (Align.CENTER, Align.CENTER, Align.MIN)
C_MAX = (Align.CENTER, Align.CENTER, Align.MAX)

PANEL_HOLE = 8.1  # catalog 8.1 +0.1
KEY_R = 5.1  # key slot to centre, catalog
KEY_W = 1.6  # catalog 1.6 +0.1
PANEL_T_MAX = 2.0  # catalog 0.7 to 2.0
FRONT_LEN = 6.3  # receptacle in front of the panel, catalog
THREAD_LEN = 7.7  # M8 x 0.5 thread behind the panel, catalog
PLUG_D = 11.5  # HR10A-7P plug body, catalog p.6
PLUG_LEN = 35.0  # plug body with strain relief, catalog p.6
FLANGE_T = 1.5  # not in the catalog, assumed
NUT_T = 2.0  # not in the catalog, assumed


def receptacle(panel_t: float = PANEL_T_MAX) -> Part:
    """HR10A-7R-4S, flange on the panel's front face, nut on the back."""
    front = Pos(0, 0, -FLANGE_T) * Cylinder(8.85 / 2, FRONT_LEN - FLANGE_T, align=C_MAX)
    flange = Cylinder(11.0 / 2, FLANGE_T, align=C_MAX)
    thread = Cylinder(7.9 / 2, THREAD_LEN, align=C_MIN)
    nut = Pos(0, 0, panel_t) * extrude(RegularPolygon(10.0 / 2, 6, major_radius=False), NUT_T)
    # Solder cups behind the thread: 4 cups inside a Ø6 envelope, 4 mm long.
    cups = Pos(0, 0, THREAD_LEN) * Cylinder(3.0, 4.0, align=C_MIN)
    return Pos(0, 0, -0.0) * (front + flange + thread + nut + cups)


def plug(cable_len: float = 60.0, cable_d: float = 5.0) -> Part:
    """HR10A-7P-4P mated on the receptacle: body from the flange outward, then the cable."""
    body = Pos(0, 0, -FLANGE_T) * Cylinder(PLUG_D / 2, PLUG_LEN, align=C_MAX)
    cable = Pos(0, 0, -FLANGE_T - PLUG_LEN) * Cylinder(cable_d / 2, cable_len, align=C_MAX)
    return body + cable


def panel_cut(panel_t: float) -> Part:
    """Panel hole plus the anti-rotation key slot, as a tool to subtract (+Z through panel)."""
    hole = Cylinder(PANEL_HOLE / 2 + 0.1, panel_t, align=C_MIN)  # +0.1 FDM undersize
    key = Pos(0, -(KEY_R + 0.05) / 2, 0) * Box(KEY_W + 0.2, KEY_R + 0.05, panel_t, align=C_MIN)
    return hole + key
