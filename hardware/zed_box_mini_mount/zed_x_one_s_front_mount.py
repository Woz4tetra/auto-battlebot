"""Stereolabs ZED X One S front mount (ACC-151300): 50 x 30 x 4 mm aluminum plate.

Reference model for the camera bracket assembly, not a part to make. Dimensions come from the
"ZED X One S - Front mount, Mounting Guide" page of the ZED X One S datasheet Rev1.8
(vendor/zed_x_one_s_datasheet.pdf, page 11):

- 50 x 30 x 4 mm, 4 x R4 corners
- 2 x Ø5.4 slots at x = ±18 (36 mm pitch), 20 mm center to center along the 30 mm side
- 4 x Ø2.4 thru on the camera's 21 mm square, counterbored Ø4.4 x 2 from the front
- Ø20.5 bore on the optical axis for the lens ring

The camera's front face sits on the back face; its 4 x M2 x 0.4 front holes take the screws
from the counterbored side. The store page lists the plate as 27.22 x 55 x 4 mm, which matches
neither the drawing nor the product photo, so the drawing wins.

Coordinates: origin on the optical axis in the back face (the camera side). +Y is the viewing
direction (the plate runs y = 0..4), X along the 50 mm side, +Z up.

Open questions
- The side notches on the 30 mm edges are drawn but not dimensioned; modeled 9 x 1.5 mm from
  the drawing's scale. They touch nothing in this assembly.
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


@dataclass
class Params:
    length: float = 50.0  # datasheet p.11
    height: float = 30.0  # datasheet p.11
    thickness: float = 4.0  # datasheet p.11
    corner_r: float = 4.0  # datasheet p.11, 4X R4
    slot_x: float = 18.0  # datasheet p.11, 36 mm pitch
    slot_w: float = 5.4  # datasheet p.11, 2X Ø5.4
    slot_pitch: float = 20.0  # datasheet p.11, 2X 20
    cam_hole: float = 2.4  # datasheet p.11, M2 clearance
    cam_cbore: float = 4.4  # datasheet p.11
    cam_cbore_depth: float = 2.0  # datasheet p.11
    cam_pitch: float = 21.0  # datasheet p.11 and camera drawing p.8
    bore: float = 20.5  # datasheet p.11
    notch_h: float = 9.0  # drawing scale, not dimensioned
    notch_d: float = 1.5  # drawing scale, not dimensioned


def build(p: Params) -> Part:
    t = p.thickness
    # Profile in XZ, extruded along +Y: Rot(-90, 0, 0) maps +Z to +Y.
    plate = Box(p.length, p.height, t, align=(Align.CENTER, Align.CENTER, Align.MIN))
    corners = plate.edges().filter_by(Axis.Z)
    assert len(corners) == 4, f"expected 4 corners, got {len(corners)}"
    plate = fillet(corners, p.corner_r)
    for sx in (-1, 1):
        plate -= Pos(sx * (p.length / 2), 0, 0) * Box(
            2 * p.notch_d, p.notch_h, t, align=(Align.CENTER, Align.CENTER, Align.MIN)
        )
        plate -= Pos(sx * p.slot_x, 0, 0) * extrude(
            SlotCenterToCenter(p.slot_pitch, p.slot_w, rotation=90), t
        )
        for sz in (-1, 1):
            at = Pos(sx * p.cam_pitch / 2, sz * p.cam_pitch / 2, 0)
            plate -= at * Cylinder(p.cam_hole / 2, t, align=(Align.CENTER, Align.CENTER, Align.MIN))
            plate -= (
                at
                * Pos(0, 0, t - p.cam_cbore_depth)
                * Cylinder(
                    p.cam_cbore / 2,
                    p.cam_cbore_depth,
                    align=(Align.CENTER, Align.CENTER, Align.MIN),
                )
            )
    plate -= Cylinder(p.bore / 2, t, align=(Align.CENTER, Align.CENTER, Align.MIN))
    # Sketch Y becomes world -Z under Rot(-90, 0, 0); the part is symmetric in Z, so it stays put.
    return Rot(-90, 0, 0) * plate


def sections(p: Params) -> list[str]:
    return [f"x={p.slot_x}", f"x={p.cam_pitch / 2}"]


def expect(p: Params) -> dict:
    return {
        "bbox": (50.0, 4.0, 30.0),
        "holes": {2.4: 4, 4.4: 4, 20.5: 1},
    }


if __name__ == "__main__":
    print(build(Params()).bounding_box())
