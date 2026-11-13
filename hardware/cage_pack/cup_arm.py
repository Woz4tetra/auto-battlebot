"""Cup arm: a printed PETG arm that carries one suction cup off a side wall of the tray.

The tray's panel sits CUP_H off the glass, level with the cups' rail tops. Each arm's plate
lies on its cup's rail beside the tray, with one 3/8-16 screw and a load washer down into
the cup's centre thread. Its tongue reaches under the panel and takes two M3 screws up into
heat-set inserts pressed into the panel's glass side. A step between the two hugs the side
wall's outside face. The cup's whole rim, release tabs included, is outside the tray.

Both arms are this part: the -X arm is the +X arm turned 180° about the glass normal, since
the profile is symmetric about the cup's axis along Y. Print two.

Spec
- Process: FDM, black PETG, printed on its side: the profile below is the bed face, the
  arm's width rises from it, so every face of the profile is a wall and the screw holes
  run across the layers. PROCESS = "fdm".
- Frame (print frame): X = out from the side wall's outside face, Y = along the glass
  normal from the cups' rail tops (Y = 0 is Z = CUP_H in the assembly), Z = across the arm,
  0 to ARM_W. ASSEMBLY_AT places it on either side.
- Profile: tongue 36 under the panel, 10 thick; beam 18 deep (tongue + plate) to 8 mm
  past the wall, sloping up to the plate 2 mm short of the cup's rail; plate 8 thick to 30
  past the cup's axis. 40 wide.
- Holes:
  - cup screw: Ø10.3 (3/8" normal clearance, fasteners.md) at the cup's axis, CUP_OUT
    from the wall, in a Ø27 x 3.0 washer spot face. That leaves 5.0 under the washer, so a
    3/8-16 x 5/8" screw engages the cup's thread by 9.4, as the old 1/2" screw did through
    the tray's 1.6.
  - panel screws: 2 x Ø3.4 (M3 normal clearance) 28 in from the wall, 24 apart, each
    under a Ø6.5 x 3.5 counterbore for an M3 socket head from the glass side. M3 x 12
    engages the panel's 6.4 mm inserts by 5.5 (86 %).
- Model: features() (recipe.py), so the checked solid and the Onshape Part Studio are the
  same eight features: profile, arm, foot chamfer, cup screw sketch, hole, spot face,
  panel screw sketch, holes and counterbores.

Load path: the cup pulls the plate; the beam carries it to the wall as a lever, the panel
screws in tension 28 in from the wall and the panel's underside bearing on the tongue
at the wall. With the cup 45 out, a cup pull F puts about 2.6 F into the two screws.

Open questions
- Release tab positions are from photos (smallrig_4122c.py): check them on the real cups.
- The inserts pull out toward the side they were pressed from. Pull test a printed coupon
  before trusting the arm with a pack on the glass.
"""

from dataclasses import dataclass

import b123d_fix  # noqa: F401
import cage_layout as cl
import recipe  # build123d-part/scripts, on the path when run through scripts/run
from build123d import Part, Pos, Rot

PROCESS = "fdm"
MATERIAL = "petg"

W = cl.ARM_W
_BASE = Pos(0, W / 2, 0) * Rot(90, 0, 0)  # print frame -> wall frame, cup axis at y = 0
ASSEMBLY_AT = {
    "-x": Pos(cl.X_OUT[0], cl.CUP_Y, cl.CUP_H) * Rot(0, 0, 180) * _BASE,
    "+x": Pos(cl.X_OUT[1], cl.CUP_Y, cl.CUP_H) * _BASE,
}


@dataclass
class Params:
    cup_hole: float = 10.3  # 3/8" normal clearance, fasteners.md
    clear: float = 3.4  # M3 normal clearance, fasteners.md
    rail_gap: float = 2.0  # beam end to the cup's rail
    slope: float = 6.0  # beam's slope up to the plate, along X
    plate_past: float = 30.0  # plate beyond the cup's axis; the rail ends 28 past it
    foot_chamfer: float = 0.5  # elephant's foot, fdm.md


def profile(p: Params) -> list[tuple[float, float]]:
    """The arm's side profile in the print frame (X out from the wall, Y from the rail tops)."""
    lt, tt, tp = cl.ARM_TONGUE, cl.ARM_TONGUE_T, cl.ARM_PLATE_T
    beam = cl.CUP_OUT - 28.0 - p.rail_gap  # the rail is 56 long, centred on the cup
    end = cl.CUP_OUT + p.plate_past
    return [(-lt, -tt), (beam - p.slope, -tt), (beam, 0.0), (end, 0.0), (end, tp),
            (0.0, tp), (0.0, 0.0), (-lt, 0.0)]  # fmt: skip


def features(p: Params) -> list[dict]:
    """Profile on Top extruded across the arm, then the screw holes from the Front plane
    (sketch x = X, y = Z; normal -Y, so an extrude's start is minus the Y it begins at)."""
    poly = profile(p)
    tt, tp = cl.ARM_TONGUE_T, cl.ARM_PLATE_T
    zm = W / 2
    cup = (cl.CUP_OUT, zm)
    screws = [(-cl.ARM_SCREW_IN, zm + s * cl.ARM_SCREW_DY) for s in (-1, 1)]
    spot_ring = (cl.CUP_OUT + (p.cup_hole + cl.ARM_SPOT[0]) / 4, zm)
    cb_d, cb_t = cl.ARM_CBORE
    cb_ring = [(x + (p.clear + cb_d) / 4, z) for x, z in screws]
    # Bed-face edges for the foot chamfer: the midpoint of each profile side at Z = 0.
    sides = [((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, 0.0) for a, b in zip(poly, poly[1:] + poly[:1])]
    return [
        {"op": "sketch", "name": "Arm profile", "plane": "Top", "profiles": [{"polygon": poly}]},
        {"op": "extrude", "name": "Arm", "sketch": "Arm profile",
         "regions": [(-cl.ARM_TONGUE / 2, -tt / 2)], "depth": W},
        {"op": "chamfer", "name": "Elephant's foot chamfer", "size": p.foot_chamfer,
         "edges": sides},
        {"op": "sketch", "name": "Cup screw", "plane": "Front", "profiles": [
            {"circle": (*cup, p.cup_hole)}, {"circle": (*cup, cl.ARM_SPOT[0])}]},
        {"op": "extrude", "name": "Cup screw hole", "sketch": "Cup screw", "regions": [cup],
         "start": -(tp + 1.0), "depth": tp + 2.0, "mode": "remove"},
        {"op": "extrude", "name": "Washer spot face", "sketch": "Cup screw",
         "regions": [spot_ring], "start": -tp, "depth": cl.ARM_SPOT[1], "mode": "remove"},
        {"op": "sketch", "name": "Panel screws", "plane": "Front", "profiles": [
            *[{"circle": (*xz, p.clear)} for xz in screws],
            *[{"circle": (*xz, cb_d)} for xz in screws]]},
        {"op": "extrude", "name": "Panel screw holes", "sketch": "Panel screws",
         "regions": screws, "start": -1.0, "depth": tt + 2.0, "mode": "remove"},
        {"op": "extrude", "name": "Counterbores", "sketch": "Panel screws", "regions": cb_ring,
         "start": tt - cb_t, "depth": cb_t + 1.0, "mode": "remove"},
    ]  # fmt: skip


def build(p: Params) -> Part:
    return recipe.build(features(p))


def sections(p: Params) -> list[str]:
    return [f"z={W / 2}", f"x={cl.CUP_OUT}", f"x={-cl.ARM_SCREW_IN}"]


def expect(p: Params) -> dict:
    return {
        "bbox": (cl.ARM_TONGUE + cl.CUP_OUT + p.plate_past, cl.ARM_TONGUE_T + cl.ARM_PLATE_T, W),
        "holes": {p.cup_hole: 1, cl.ARM_SPOT[0]: 1, p.clear: 2, cl.ARM_CBORE[0]: 2},
        "hole_at": {p.cup_hole: [{"x": cl.CUP_OUT, "z": W / 2}]},
    }
