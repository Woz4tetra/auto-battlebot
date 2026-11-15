"""Cup arm that carries one suction cup off a tray side wall."""

from dataclasses import dataclass

import b123d_fix  # noqa: F401
import cage_layout as cl
import recipe
from build123d import Part, Pos, Rot

PROCESS = "fdm"
MATERIAL = "petg"

W = cl.ARM_W
PRINT_TO_WALL = Pos(0, W / 2, 0) * Rot(90, 0, 0)
ASSEMBLY_AT = {
    "-x": Pos(cl.X_OUT[0], cl.CUP_Y, cl.CUP_H) * Rot(0, 0, 180) * PRINT_TO_WALL,
    "+x": Pos(cl.X_OUT[1], cl.CUP_Y, cl.CUP_H) * PRINT_TO_WALL,
}


@dataclass
class Params:
    cup_clearance_dia: float = 10.3
    m3_clearance_dia: float = 3.4
    beam_to_rail_gap: float = 2.0
    slope: float = 6.0
    plate_past: float = 30.0
    foot_chamfer: float = 0.5


def profile(p: Params) -> list[tuple[float, float]]:
    """The arm's side profile in the print frame (X out from the wall, Y from the rail tops)."""
    lt, tt, tp = cl.ARM_TONGUE, cl.ARM_TONGUE_T, cl.ARM_PLATE_T
    beam = cl.CUP_OUT - cl.CUP_RAIL_HALF_LEN - p.beam_to_rail_gap
    end = cl.CUP_OUT + p.plate_past
    return [(-lt, -tt), (beam - p.slope, -tt), (beam, 0.0), (end, 0.0), (end, tp),
            (0.0, tp), (0.0, 0.0), (-lt, 0.0)]  # fmt: skip


def _bed_edge_midpoints(poly: list[tuple[float, float]]) -> list[tuple[float, float, float]]:
    return [((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, 0.0) for a, b in zip(poly, poly[1:] + poly[:1])]


def features(p: Params) -> list[dict]:
    """Arm profile and screw holes as CAD features."""
    poly = profile(p)
    tt, tp = cl.ARM_TONGUE_T, cl.ARM_PLATE_T
    zm = W / 2
    cup = (cl.CUP_OUT, zm)
    screws = [(-cl.ARM_SCREW_IN, zm + s * cl.ARM_SCREW_DY) for s in (-1, 1)]
    spot_ring = (cl.CUP_OUT + (p.cup_clearance_dia + cl.ARM_SPOT[0]) / 4, zm)
    cb_d, cb_t = cl.ARM_CBORE
    cb_ring = [(x + (p.m3_clearance_dia + cb_d) / 4, z) for x, z in screws]
    sides = _bed_edge_midpoints(poly)
    return [
        {"op": "sketch", "name": "Arm profile", "plane": "Top", "profiles": [{"polygon": poly}]},
        {"op": "extrude", "name": "Arm", "sketch": "Arm profile",
         "regions": [(-cl.ARM_TONGUE / 2, -tt / 2)], "depth": W},
        {"op": "chamfer", "name": "Elephant's foot chamfer", "size": p.foot_chamfer,
         "edges": sides},
        {"op": "sketch", "name": "Cup screw", "plane": "Front", "profiles": [
            {"circle": (*cup, p.cup_clearance_dia)}, {"circle": (*cup, cl.ARM_SPOT[0])}]},
        {"op": "extrude", "name": "Cup screw hole", "sketch": "Cup screw", "regions": [cup],
         "start": -(tp + 1.0), "depth": tp + 2.0, "mode": "remove"},
        {"op": "extrude", "name": "Washer spot face", "sketch": "Cup screw",
         "regions": [spot_ring], "start": -tp, "depth": cl.ARM_SPOT[1], "mode": "remove"},
        {"op": "sketch", "name": "Panel screws", "plane": "Front", "profiles": [
            *[{"circle": (*xz, p.m3_clearance_dia)} for xz in screws],
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
        "holes": {
            p.cup_clearance_dia: 1,
            cl.ARM_SPOT[0]: 1,
            p.m3_clearance_dia: 2,
            cl.ARM_CBORE[0]: 2,
        },
        "hole_at": {p.cup_clearance_dia: [{"x": cl.CUP_OUT, "z": W / 2}]},
    }
