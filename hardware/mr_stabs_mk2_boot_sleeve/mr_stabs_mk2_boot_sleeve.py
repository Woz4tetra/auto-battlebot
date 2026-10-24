"""Mr Stabs Mk2 BOOT/RST sleeve: insulating cap over the H1 1x3 jumper header (IO0 / GND / EN).

The header stands on the board's wedge face on the left ear, at board frame (-21.0, -9.0)
(pcba/gen_placement.py). A loose wire, a screw or swarf across GND and EN resets the ESP32 mid
match, and across IO0 and GND at power-up it boots into the ROM loader. The cap pushes onto the
header's plastic body, covers all three pins, and pulls off by hand to fit the jumper.

Spec
- Process: FDM PETG, closed end on the bed, opening up. No supports.
- Frame: origin at H1's footprint centre on the board's wedge face. +Z points from the pin tips
  toward the board, pins run along X. In the board frame (../mr_stabs_mk2_pcb) the part sits at
  Pos(-21.0, -9.0, -1.6), no rotation.
- Interfaces:
  - Header body (HX PZ2.54-1x3P-TP-YQ STEP): 7.62 x 2.50, from 0.70 to 3.20 off the board.
    Pocket at +0.10 per axis grips its top 2.0 mm (press, fdm.md; FDM holes print 0.1 to 0.2
    under, so this lands at a light interference).
  - Pins: 0.64 square (0.91 across corners), 6.0 past the body, tips 9.20 off the board. Each
    gets its own Ø1.30 blind hole, 0.30 past the tip, so a pin never touches another pin's hole.
  - SMD leads bend out to y ±2.45 within 0.70 of the board. The skirt stops 1.20 off the
    board: 0.50 above them.
  - Chassis (pcba/cad/stl/chassis.stl, board frame): one guide face runs parallel to the
    board's install direction (y = -4.337 + 0.3478 z, 19.2 deg off Z). It is the front of the
    wall at x > -16.3 and the slope of the lip under the ear, whose tip is at y -6.268,
    z -5.552. The rear-inboard corner is relieved 0.3 off that face, and the rear face below
    the tip steps in to 0.3 off it. Cut parallel to the install direction, so the sleeve stays
    0.3 clear while the board slides in.
- Install: push the sleeve on before the board goes in. With the board seated the guide face
  overhangs the pins' axis, so the sleeve cannot come off or go on along it.
- Model: features() (recipe.py), so the checked solid and the Onshape Part Studio are the
  same 10 features. The relief is two Right-plane polygon cuts.
- Critical dimensions: pocket 7.72 x 2.60 x 2.0 deep; 3 x Ø1.30 holes at x = -2.54, 0, 2.54;
  walls 1.5 (1.2 fdm.md minimum after the 0.3 lead-in); outside 10.72 x 5.60 x 9.50.

Open questions
- Grip is friction on the body only. If it walks off in a match, print with pocket_clear 0.0,
  or add a dab of hot glue on one end.
- PETG assumed. TPU would grip harder but holds Ø1.3 holes poorly.
"""

import math
from dataclasses import dataclass
from pathlib import Path

import recipe  # build123d-part/scripts, on the path when run through scripts/run
from build123d import Align, Box, Part, Pos, Rot, import_step

PROCESS = "fdm"
MATERIAL = "petg"

HEADER_STEP = (
    Path(__file__).resolve().parent.parent
    / "mr_stabs_mk2_pcb/pcba/lib/mk2.3dshapes/HDR-SMD_3P-P2.54-V-M_HX-PZ2.54-1X3P-TP-YQ.step"
)
C_MIN = (Align.CENTER, Align.CENTER, Align.MIN)
C_MAX = (Align.CENTER, Align.CENTER, Align.MAX)
MIN_Y = (Align.CENTER, Align.MIN, Align.CENTER)


@dataclass
class Params:
    body_x: float = 7.62  # header STEP
    body_y: float = 2.50  # header STEP
    body_top: float = 3.20  # body top off the board, header STEP
    pin_tip: float = 9.20  # pin tip off the board, header STEP
    pitch: float = 2.54  # header STEP
    pocket_clear: float = 0.10  # press, fdm.md
    grip: float = 2.0  # body length inside the pocket
    pin_hole: float = 1.30  # 0.91 pin diagonal + free clearance (0.4), fdm.md
    tip_gap: float = 0.30  # hole floor past the pin tip
    roof: float = 1.2  # closed end, minimum wall (fdm.md)
    wall: float = 1.5  # 1.2 minimum (fdm.md) plus the 0.3 mouth chamfer
    corner_r: float = 1.0  # vertical edges
    foot_chamfer: float = 0.4  # elephant's foot, fdm.md
    mouth_chamfer: float = 0.3  # lead-in on the pocket
    # Placement and chassis, board frame (../mr_stabs_mk2_pcb.py)
    at_x: float = -21.0  # H1, pcba/gen_placement.py
    at_y: float = -9.0  # H1, pcba/gen_placement.py
    board_t: float = 1.6  # wedge face at z -1.6
    guide_y0: float = -4.337  # chassis guide face y at z 0, chassis.stl
    guide_slope: float = 0.3478  # dy/dz, = INSTALL_DIR in ../mr_stabs_mk2_pcb.py
    wall_x: float = -16.3  # guide wall's outboard face, chassis.stl
    lip_tip_y: float = -6.268  # chassis.stl
    lip_tip_z: float = -5.552  # chassis.stl
    relief_gap: float = 0.3  # FDM tolerance on a 0.6 mm-class print, fdm.md


def z_bottom(p: Params) -> float:
    return -(p.pin_tip + p.tip_gap + p.roof)


def z_top(p: Params) -> float:
    return -(p.body_top - p.grip)


def features(p: Params) -> list[dict]:
    """The sleeve as 10 CAD features (recipe.py), replayed by build() and by the onshape skill."""
    px, py = p.body_x + p.pocket_clear, p.body_y + p.pocket_clear
    ox, oy = px + 2 * p.wall, py + 2 * p.wall
    pins = [(x, 0.0) for x in (-p.pitch, 0.0, p.pitch)]
    ring, pocket = (0.0, (oy + py) / 4), (p.pitch / 2, 0.0)

    # Chassis relief, sketched on the Right plane (sketch x = part Y, sketch y = part Z): the
    # guide face moved relief_gap toward the sleeve is the line y = y0 + slope z. Material on
    # its rear side goes beside the wall (x past wall_x) and behind the lip tip.
    y0, slope = guide_line(p, p.relief_gap)
    wall_x, tip_y, _ = to_part(p, p.wall_x, p.lip_tip_y, 0)
    lo, hi, far = z_bottom(p) - 1.0, 0.0, oy  # sketch bounds past the sleeve
    tip_z = (tip_y - p.relief_gap - y0) / slope  # where the face crosses the tip's step
    assert lo < tip_z < hi
    beside_wall = [(y0 + slope * lo, lo), (far, lo), (far, hi), (y0 + slope * hi, hi)]
    behind_tip = [
        (tip_y - p.relief_gap, lo),
        (far, lo),
        (far, hi),
        (y0 + slope * hi, hi),
        (tip_y - p.relief_gap, tip_z),
    ]

    return [
        {
            "op": "sketch",
            "name": "outline",
            "profiles": [
                {"rect": (0, 0, ox, oy), "r": p.corner_r},
                {"rect": (0, 0, px, py)},
                *[{"circle": (x, y, p.pin_hole)} for x, y in pins],
            ],
        },
        {
            "op": "extrude",
            "name": "body",
            "sketch": "outline",
            "regions": [ring, pocket, *pins],
            "start": z_bottom(p),
            "depth": z_top(p) - z_bottom(p),
        },
        {
            "op": "chamfer",
            "name": "foot",
            "edges": [(0, -oy / 2, z_bottom(p))],
            "size": p.foot_chamfer,
        },
        {
            "op": "extrude",
            "name": "pocket",
            "sketch": "outline",
            "regions": [pocket, *pins],
            "start": z_top(p),
            "depth": p.grip,
            "flip": True,
            "mode": "remove",
        },
        {
            "op": "chamfer",
            "name": "mouth",
            "size": p.mouth_chamfer,
            "tangent": False,
            "edges": [
                (0, py / 2, z_top(p)),
                (0, -py / 2, z_top(p)),
                (px / 2, 0, z_top(p)),
                (-px / 2, 0, z_top(p)),
            ],
        },
        {
            "op": "extrude",
            "name": "pin holes",
            "sketch": "outline",
            "regions": pins,
            "start": -p.body_top,
            "depth": p.pin_tip - p.body_top + p.tip_gap,
            "flip": True,
            "mode": "remove",
        },
        {
            "op": "sketch",
            "name": "wall relief",
            "plane": "Right",
            "profiles": [{"polygon": beside_wall}],
        },
        {
            "op": "extrude",
            "name": "wall relief",
            "sketch": "wall relief",
            "regions": [(far - 0.5, hi - 0.5)],
            "start": wall_x - p.relief_gap,
            "depth": ox,
            "mode": "remove",
        },
        {
            "op": "sketch",
            "name": "lip relief",
            "plane": "Right",
            "profiles": [{"polygon": behind_tip}],
        },
        {
            "op": "extrude",
            "name": "lip relief",
            "sketch": "lip relief",
            "regions": [(far - 0.5, lo + 0.5)],
            "start": -ox,
            "depth": 2 * ox,
            "mode": "remove",
        },
    ]


def build(p: Params) -> Part:
    return recipe.build(features(p))


def to_part(p: Params, x: float, y: float, z: float) -> tuple[float, float, float]:
    """Board frame -> this part's frame."""
    return x - p.at_x, y - p.at_y, z + p.board_t


def guide_line(p: Params, offset: float) -> tuple[float, float]:
    """The guide face moved `offset` toward the sleeve, as y = y0 + slope * z (part frame)."""
    _, y0, _ = to_part(p, 0, p.guide_y0, 0)
    tilt = math.atan(p.guide_slope)
    return y0 - p.guide_slope * p.board_t - offset / math.cos(tilt), p.guide_slope


def guide_half(p: Params, offset: float) -> Part:
    """Everything behind the chassis guide face moved `offset` toward the sleeve (part frame)."""
    y0, _ = guide_line(p, offset)
    tilt = math.degrees(math.atan(p.guide_slope))
    return Pos(0, y0, 0) * Rot(-tilt, 0, 0) * Box(80, 80, 80, align=MIN_Y)


def chassis_proxy(p: Params) -> Part:
    """The guide wall and the lip above its tip, as solids in this part's frame."""
    wall_x, _, tip_z = to_part(p, p.wall_x, 0, p.lip_tip_z)
    wall = guide_half(p, 0) & Pos(wall_x, 0, -30) * Box(
        20, 40, 30, align=(Align.MIN, Align.CENTER, Align.MIN)
    )
    lip = guide_half(p, 0) & Pos(-20, 0, tip_z) * Box(
        20 + wall_x, 40, -tip_z, align=(Align.MIN, Align.CENTER, Align.MIN)
    )
    return wall + lip


def sections(p: Params) -> list[str]:
    return ["y=0", f"x={p.pitch}", "x=0", "x=4.9"]


def header_in_part() -> Part:
    """The header STEP as KiCad places it (model offset), flipped so its pins point -Z."""
    return Rot(180, 0, 0) * Pos(-2.54, -0.413, 0.7) * import_step(HEADER_STEP)


def mates(p: Params) -> dict:
    return {
        "header": header_in_part(),
        # board slab around the footprint, solder side down at z 0
        "board": Pos(0, 0, 0) * Box(20, 12, 1.6, align=C_MIN),
        "chassis": chassis_proxy(p),
    }


def expect(p: Params) -> dict:
    return {
        "bbox": (
            p.body_x + p.pocket_clear + 2 * p.wall,
            p.body_y + p.pocket_clear + 2 * p.wall,
            p.pin_tip + p.tip_gap + p.roof - p.body_top + p.grip,
        ),
        "holes": {p.pin_hole: 3},
        "hole_at": {p.pin_hole: [{"x": x, "y": 0} for x in (-p.pitch, 0.0, p.pitch)]},
        "clearance": {"board": 1.0, "chassis": p.relief_gap},
        "chamfers_min": 8,
        "min_wall": 1.2,
    }
