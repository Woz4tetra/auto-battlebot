"""SmallRig 4122C 4" suction cup camera mount: an envelope traced from the B&H page.

SmallRig publishes no CAD. Sources (saved to ~/Downloads by the user, 2026-11-07):
- B&H spec table: 139.7 x 106.7 x 50.8 mm overall, 264 g, rated 50 kg horizontal mounting
  (pull straight off the surface) and 35 kg vertical mounting (cup on a wall, load in shear).
- Dimension photo: Ø101.5 cup, 48.2 mm glass to the top of the rail.
- Close-up of the top: a NATO rail with a 3/8-16 ARRI locating hole in the middle (four
  small pin holes around it), one 1/4-20 toward the pump, and two 1/4-20 on the side
  shoulder. Only the central 3/8-16 is used here; the rest are left out of the model.
- Product photos: two release tabs stand on the rubber's rim, opposite each other, about 90°
  each side of the pump. Lifting one breaks the seal.

Frame: cup face on the glass at Z = 0, axis along +Z, rail and pump along X, pump toward -X.

Open questions
- Thread depth of the central 3/8-16: assumed 10 mm. Measure before buying screws.
- Body profile between the rubber and the rail block is traced from photos, +-3 mm.
- Release tab angle from the pump (cage_layout.CUP_TAB_ANGLE): +-20° from the photos; tab
  size 14 wide x 12 radial x 9 tall, guessed.
"""

import cage_layout as cl
from build123d import Align, Box, Cone, Cylinder, Part, Pos, Rot

C_MIN = (Align.CENTER, Align.CENTER, Align.MIN)
C_MAX = (Align.CENTER, Align.CENTER, Align.MAX)

MASS_G = 264.0  # B&H spec
RATED_PULL_N = 50.0 * 9.81  # "horizontal mounting": load normal to the surface
RATED_SHEAR_N = 35.0 * 9.81  # "vertical mounting": load along the surface
THREAD_DEPTH = 10.0  # 3/8-16, assumed: a 9/16" screw through 4.83 of plate engages 9.5
TAP_38 = 8.4  # 3/8-16 tap drill (5/16"), fasteners.md


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
