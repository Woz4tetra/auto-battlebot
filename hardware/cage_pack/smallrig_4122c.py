"""SmallRig 4122C suction cup envelope."""

import cage_layout as cl
from build123d import Align, Box, Cone, Cylinder, Part, Pos, Rot

C_MIN = (Align.CENTER, Align.CENTER, Align.MIN)
C_MAX = (Align.CENTER, Align.CENTER, Align.MAX)

MASS_G = 264.0
RATED_PULL_N = 50.0 * 9.81
RATED_SHEAR_N = 35.0 * 9.81
THREAD_DEPTH = 10.0
TAP_38 = 8.4


TAB_W_L_H = (14.0, 12.0, 9.0)


def build() -> Part:
    rubber = Cone(cl.CUP_SKIRT_D / 2, 92.0 / 2, 6.0, align=C_MIN)
    housing = Pos(0, 0, 6.0) * Cone(88.0 / 2, 64.0 / 2, 18.0, align=C_MIN)
    block = Pos(0, 0, 24.0) * Box(50.0, 40.0, cl.CUP_H - 24.0, align=C_MIN)
    rail = Pos(0, 0, cl.CUP_H) * Box(2 * cl.CUP_RAIL_HALF_LEN, 21.0, 4.0, align=C_MAX)
    pump = Pos(0, 0, 33.0) * Rot(0, -90, 0) * Cylinder(9.0, cl.CUP_REACH, align=C_MIN)
    part = rubber + housing + block + rail + pump
    w, ln, h = TAB_W_L_H
    for s in (-1, 1):
        a = 180.0 + s * cl.CUP_TAB_ANGLE
        r = cl.CUP_SKIRT_D / 2 + 2.0 - ln / 2
        part += Rot(0, 0, a) * Pos(r, 0, 0) * Box(ln, w, h, align=C_MIN)
    part -= Pos(0, 0, cl.CUP_H) * Cylinder(TAP_38 / 2, THREAD_DEPTH, align=C_MAX)
    return part
