"""Stereolabs ZED X One S front mount (ACC-151300) reference model."""

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
    # DATASHEET: ZED X One S datasheet rev 1.8, p. 11: front mount 50 x 30 x 4 mm, 4X R4 corners
    # DATASHEET: ZED X One S datasheet rev 1.8, p. 11: 2X slot 5.4 wide at x +-18, 20 long
    # DATASHEET: ZED X One S datasheet rev 1.8, p. 11: 4X 2.4 thru, 4.4 x 2.0 cbore, M2 clearance
    # DATASHEET: ZED X One S datasheet rev 1.8, p. 11: 20.5 bore on the optical axis
    # DATASHEET: ZED X One S datasheet rev 1.8, p. 11: camera holes on a 21 mm square
    # DATASHEET: ZED X One S datasheet rev 1.8, p. 8: camera drawing, holes on a 21 mm square
    length: float = 50.0
    height: float = 30.0
    thickness: float = 4.0
    corner_r: float = 4.0
    slot_x: float = 18.0
    slot_w: float = 5.4
    slot_pitch: float = 20.0
    cam_hole: float = 2.4
    cam_cbore: float = 4.4
    cam_cbore_depth: float = 2.0
    cam_pitch: float = 21.0
    bore: float = 20.5
    notch_h: float = 9.0
    notch_d: float = 1.5


def build(p: Params) -> Part:
    t = p.thickness
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
