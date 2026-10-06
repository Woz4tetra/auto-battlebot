"""INNOREL CP10 cheese plate (Amazon B0DRYFL595): 200 x 100 x 10 mm 6061-T6 plate.

A reference model for the adapter assembly, not a part to make. Hole positions were measured
from INNOREL's dimensioned drawing (Amazon image 71V4ad30vkL), scaled to the 200 x 100
outline at 6.13 px/mm, then made symmetric about both axes. After symmetrizing, the counts
match the drawing exactly: 106 x 1/4-20, 27 x 3/8-16, 44 locating holes.

Dimensions printed on the drawing, which the model uses directly: slot width 6.4, slot
counterbore 10.3, center slot pitch 24.7 / 74.3 (so center slots at x = ±49.5), center slot
rows 42.3 apart, end slots 27 long on centers 23.5 + 27 apart, edge 3/8-16 holes at 0 and
±76.7, edge hole pitch 9.

Coordinates: origin at the center of the top face, X along the 200 mm length, +Z up. Threaded
holes are modeled at their tap drill size (1/4-20: Ø5.1, 3/8-16: Ø8.4).

Open questions
- The slot counterbore depth is not on the drawing; INNOREL's store has the same drawing
  (Size-details-of-CP10.jpg) and no datasheet or CAD turned up. 6.5 mm comes from the close-up
  photo (Amazon image 71aXTwjp85L), where the counterbore wall is about twice the height of the
  through lip below it. Measure it with a caliper's depth rod.
- Edge hole depth (8.0) and corner radius (5.0) are not on the drawing either.
- Positions without a printed dimension carry about ±0.3 mm of image-scaling error.
"""

from dataclasses import dataclass

from build123d import (
    Align,
    Axis,
    Box,
    Cylinder,
    Part,
    Pos,
    Rot,
    SlotCenterToCenter,
    extrude,
    fillet,
)

PROCESS = "none"
MATERIAL = "al6061"

TAP_1_4_20 = 5.1  # #7 drill, fasteners.md
TAP_3_8_16 = 8.4  # 5/16" drill
LOCATING = 4.0  # measured from the drawing
EDGE_SMALL = 3.8  # unlabeled on the drawing, measured

# First-quadrant hole centers (x, y) in mm; each mirrors about both axes.
QUARTER_20 = [
    (5.5, 10.0), (5.5, 18.45), (5.5, 26.95), (5.5, 35.45), (5.5, 44.15),
    (10.25, 0.0),
    (27.5, 0.0), (27.5, 9.95), (27.5, 18.45), (27.5, 26.9), (27.5, 35.45), (27.5, 44.15),
    (38.55, 11.85), (38.55, 30.85),
    (49.5, 11.85), (49.5, 21.35), (49.5, 30.85),
    (60.5, 11.85), (60.5, 30.85),
    (71.2, 7.75), (71.2, 15.7), (71.2, 34.75), (71.2, 42.9),
    (82.2, 7.75), (82.2, 15.7), (82.2, 34.75), (82.2, 42.9),
    (88.25, 0.0),
]  # fmt: skip
THREE_EIGHTHS_16 = [
    (0.0, 0.0), (16.5, 7.55), (16.5, 22.45), (16.5, 37.45),
    (38.5, 21.35), (60.5, 21.35), (76.75, 25.2), (76.75, 0.0),
]  # fmt: skip
LOCATING_HOLES = [
    (0.0, 7.55), (9.0, 22.45), (16.5, 0.0), (16.5, 15.0), (16.5, 29.95), (16.5, 44.95),
    (24.0, 22.45), (69.3, 25.25), (76.75, 7.55), (76.75, 17.7), (76.75, 32.75), (84.2, 25.2),
]  # fmt: skip
# Edge holes: offset from the edge midpoint, and kind (q = 1/4-20, t = 3/8-16, s = small).
LONG_EDGE = [
    (0.0, "t"), (7.45, "s"), (15.85, "q"), (24.85, "q"), (33.85, "q"), (42.9, "q"),
    (51.85, "q"), (60.8, "q"), (69.2, "s"), (76.7, "t"), (84.2, "s"), (90.0, "q"),
]  # fmt: skip
SHORT_EDGE = [(0.0, "t"), (7.55, "s"), (13.95, "q"), (23.05, "q"), (31.95, "q"), (40.9, "q")]
EDGE_DIA = {"q": TAP_1_4_20, "t": TAP_3_8_16, "s": EDGE_SMALL}


def mirrored(quadrant: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """All four mirror images of each first-quadrant point, without duplicates on the axes."""
    out = {(sx * x, sy * y) for x, y in quadrant for sx in (-1, 1) for sy in (-1, 1)}
    return sorted((x + 0.0, y + 0.0) for x, y in out)


def mirrored_1d(edge: list[tuple[float, str]]) -> list[tuple[float, str]]:
    return sorted({(s * v + 0.0, k) for v, k in edge for s in (-1, 1)})


@dataclass
class Params:
    length: float = 200.0  # drawing
    width: float = 100.0  # drawing
    thickness: float = 10.0  # drawing
    corner_r: float = 5.0  # not on the drawing
    slot_w: float = 6.4  # drawing
    slot_cbore_w: float = 10.3  # drawing
    slot_cbore_depth: float = 6.5  # estimated from photo 71aXTwjp85L, leaves a 3.5 mm lip
    center_slot_x: float = 49.5  # drawing: 74.3 / 2 + 24.7 / 2
    center_slot_pitch: float = 24.7  # drawing
    center_slot_row: float = 42.3  # drawing
    end_slot_x: float = 92.3  # measured
    end_slot_y: float = 25.25  # drawing: 23.5 / 2 + 27 / 2
    end_slot_pitch: float = 27.0  # drawing
    edge_hole_depth: float = 8.0  # not on the drawing


def _slot(
    cx: float, cy: float, pitch: float, width: float, depth: float, z_top: float, vertical: bool
) -> Part:
    sk = SlotCenterToCenter(pitch, width, rotation=90 if vertical else 0)
    return Pos(cx, cy, z_top - depth) * extrude(sk, depth)


def build(p: Params) -> Part:
    t = p.thickness
    part = Box(p.length, p.width, t, align=(Align.CENTER, Align.CENTER, Align.MAX))
    corners = part.edges().filter_by(Axis.Z)
    assert len(corners) == 4, f"expected 4 vertical corners, got {len(corners)}"
    part = fillet(corners, p.corner_r)

    def through(d: float) -> Part:
        return Cylinder(d / 2, t, align=(Align.CENTER, Align.CENTER, Align.MAX))

    for pts, d in (
        (QUARTER_20, TAP_1_4_20),
        (THREE_EIGHTHS_16, TAP_3_8_16),
        (LOCATING_HOLES, LOCATING),
    ):
        for x, y in mirrored(pts):
            part -= Pos(x, y, 0) * through(d)

    slots = [
        (sx * p.center_slot_x, y, False)
        for sx in (-1, 1)
        for y in (-p.center_slot_row, 0.0, p.center_slot_row)
    ]
    slots += [(sx * p.end_slot_x, sy * p.end_slot_y, True) for sx in (-1, 1) for sy in (-1, 1)]
    for cx, cy, vertical in slots:
        pitch = p.end_slot_pitch if vertical else p.center_slot_pitch
        part -= _slot(cx, cy, pitch, p.slot_w, t, 0, vertical)
        part -= _slot(cx, cy, pitch, p.slot_cbore_w, p.slot_cbore_depth, 0, vertical)

    zc = -t / 2
    for v, k in mirrored_1d(LONG_EDGE):
        for sy in (-1, 1):
            hole = Cylinder(
                EDGE_DIA[k] / 2, p.edge_hole_depth, align=(Align.CENTER, Align.CENTER, Align.MAX)
            )
            # Rot(-90, 0, 0) maps +Z to +Y, so the hole runs inward from the +Y edge.
            part -= Pos(v, sy * p.width / 2, zc) * Rot(-90 * sy, 0, 0) * hole
    for v, k in mirrored_1d(SHORT_EDGE):
        for sx in (-1, 1):
            hole = Cylinder(
                EDGE_DIA[k] / 2, p.edge_hole_depth, align=(Align.CENTER, Align.CENTER, Align.MAX)
            )
            # Rot(0, 90, 0) maps +Z to +X.
            part -= Pos(sx * p.length / 2, v, zc) * Rot(0, 90 * sx, 0) * hole
    return part


def sections(p: Params) -> list[str]:
    return ["y=30.85", f"x={p.center_slot_x}", "x=49.5"]


def expect(p: Params) -> dict:
    return {
        "bbox": (200.0, 100.0, 10.0),
        "holes": {TAP_1_4_20: 106, TAP_3_8_16: 27, LOCATING: 44},
        "waive": {"spec.holes": "edge holes add more 1/4-20 and 3/8-16 holes along X and Y"},
    }


if __name__ == "__main__":
    print(build(Params()).bounding_box())
