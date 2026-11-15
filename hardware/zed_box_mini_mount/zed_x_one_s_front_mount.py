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
