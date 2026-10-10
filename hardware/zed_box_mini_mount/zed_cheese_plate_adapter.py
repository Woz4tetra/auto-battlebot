"""ZED Box Mini to cheese plate adapter: a printed plate sandwiched between the ZED Box Mini
and an INNOREL CP10 cheese plate (200 x 100 x 10 mm, 6061).

Spec
- Process and orientation: FDM PLA, 0.4 mm nozzle. Bottom face (cheese plate side) on the
  bed, so the counterbores and insert holes open upward and nothing overhangs.
- Coordinates: origin at the center of the bottom face. X runs along the cheese plate's
  200 mm length and the ZED's long axis, Y along the 100 mm width, +Z up toward the ZED.
- ZED Box Mini: the bottom is a 1 mm sheet with two side ears (104 mm across, 60 mm long);
  each ear has 2 x Ø3.6 holes. Measured from Stereolabs' STEP ("ZED Box Mini Wifi with
  fan.step", stereolabs.com/3dmodels): holes at (±24.0, ±47.3), a 48 x 94.55 mm pattern. The
  product page says "92 x 45 mm", which does not match the CAD. Body 143 x 86 mm.
- ZED to adapter: 4 x M3 x 6 mm heat-set inserts, Ø5 knurl, from the shop's stock ("M3x6x5").
  They sit in Ø4.2 through holes, flush with the top. Screws: M3 x 4 to M3 x 6 down through
  the ears. An M3 x 6 ends 1.7 mm above the bottom face; an M3 x 8 would stick out 0.3 mm
  and jack the adapter off the cheese plate.
- Adapter to cheese plate: 4 x 1/4-20 x 1/2" flanged button head (80/20 3342, the screw in
  McMaster 47065T142 without its T-nut), in counterbores under the ZED body. Install them before the ZED: the ZED's bottom sheet covers the heads.
  The 2.7 mm floor under the head puts the screw tip flush with the cheese plate's bottom
  face, so all 10 mm of the plate's thread is engaged.
- Cheese plate hole positions come from INNOREL's dimensioned Amazon image
  (m.media-amazon.com/images/I/71V4ad30vkL), scaled to the 200 x 100 outline at 6.13 px/mm.
  X = ±49.5 matches the drawing's own slot dimensions (74.3 / 2 + 24.7 / 2); Y = ±30.85 is
  scaled from the image only.

Open questions
- Measure the cheese plate's 1/4-20 Y spacing (61.7 mm here) before printing. The clearance
  holes are Ø7.0 so ±0.3 mm of image error still fits, and `plate_hole_y` is a Param.
- The flange size is unverified: 80/20's drawing would not load. The counterbore assumes
  a flange up to Ø0.60" (15.2 mm) and a head up to 0.15" (3.8 mm). Measure one; if it is
  bigger, raise `cbore_d` / `cbore_depth` and `thickness` together to keep the 2.7 mm floor.
- Insert hole Ø4.2 is a guess for a generic Ø5 knurl insert. Print a test hole if there is
  time; go to 4.0 if the insert drops in loose.
- 1/4-20 engagement is the plate's full 10 mm (1.57 D), over the 1.5 D rule for aluminum.
  The 2.7 mm of PLA under each flange is the weak link.
"""


from dataclasses import dataclass

from build123d import Align, Axis, Box, Cylinder, Part, Pos, chamfer, fillet

PROCESS = "fdm"
MATERIAL = "pla"

INCH = 25.4
C_MIN = (Align.CENTER, Align.CENTER, Align.MIN)

# Mating hardware, used only by mates().
PLATE_L, PLATE_W, PLATE_T = 200.0, 100.0, 10.0  # INNOREL CP10
ZED_BODY_L, ZED_BODY_W, ZED_BODY_H = 143.0, 86.0, 41.5  # ZED Box Mini STEP, main heatsink
ZED_SHEET_T = 1.0  # bottom sheet and ears, STEP
ZED_EAR_L, ZED_EAR_W = 60.0, 103.8  # ear span along X, total width across ears, STEP
ZED_EAR_HOLE = 3.6  # STEP
FBHCS_14_HEAD_D, FBHCS_14_HEAD_H = 0.560 * INCH, 0.132 * INCH  # 80/20 3342, unverified
FBHCS_14_LEN = 0.5 * INCH  # 80/20 3342
SHCS_M3_HEAD_D, SHCS_M3_HEAD_H, SHCS_M3_LEN = 5.5, 3.0, 6.0  # fasteners.md; longest that fits


@dataclass
class Params:
    length: float = 143.0  # X: matches the ZED body length (STEP)
    width: float = 103.8  # Y: matches the ZED ears (STEP); overhangs the 100 mm plate 1.9/side
    thickness: float = 6.7  # Z: counterbore + 2.7 floor; the screw tip lands flush under the plate
    corner_r: float = 5.0  # vertical corners
    bed_chamfer: float = 0.5  # elephant's foot relief, dfm/fdm.md

    # ZED Box Mini ear holes, STEP
    zed_hole_x: float = 24.0  # ±, along the ZED's long axis (48 mm pitch)
    zed_hole_y: float = 47.3  # ±, across (94.55 mm pitch in the STEP, rounded)
    # Shop-stock M3 x 6 x Ø5 heat-set insert
    insert_hole: float = 4.2  # generic Ø5 knurl; print a test hole
    insert_len: float = 6.0
    insert_od: float = 5.0

    # INNOREL CP10 1/4-20 holes used, from the plate drawing
    plate_hole_x: float = 49.5  # ±, drawing: 74.3 / 2 + 24.7 / 2
    plate_hole_y: float = 30.85  # ±, scaled from the drawing image; measure
    # 1/4-20 x 1/2 flanged button head, 80/20 3342
    bolt_clear: float = 7.0  # 6.75 free clearance (fasteners.md) + 0.25 FDM undersize
    cbore_d: float = 16.0  # flange up to 15.2 (unverified) + 0.8 clearance and FDM undersize
    cbore_depth: float = 4.0  # head up to 3.8 (unverified) + 0.2 below the ZED sheet


def build(p: Params) -> Part:
    part = Box(p.length, p.width, p.thickness, align=C_MIN)

    corners = part.edges().filter_by(Axis.Z)
    assert len(corners) == 4, f"expected 4 vertical corners, got {len(corners)}"
    part = fillet(corners, p.corner_r)

    # Elephant's foot relief on the bed outline, before the holes so it touches only the outline.
    bed = (
        part.edges()
        .filter_by(lambda e: e.center().Z < 1e-6)
        .filter_by(lambda e: abs(e.start_point().Z) < 1e-6)
    )
    assert len(bed) == 8, f"expected 4 sides + 4 corner arcs on the bed, got {len(bed)}"
    part = chamfer(bed, p.bed_chamfer)

    top = p.thickness
    for sx in (-1, 1):
        for sy in (-1, 1):
            # M3 heat-set insert holes for the ZED ears, through so the screw length is free;
            # the insert goes in from the top.
            part -= Pos(sx * p.zed_hole_x, sy * p.zed_hole_y, 0) * Cylinder(
                p.insert_hole / 2, p.thickness, align=C_MIN
            )
            # 1/4-20 clearance through hole with a counterbore from the top.
            xy = Pos(sx * p.plate_hole_x, sy * p.plate_hole_y, 0)
            part -= xy * Cylinder(p.bolt_clear / 2, p.thickness, align=C_MIN)
            part -= (
                xy
                * Pos(0, 0, top)
                * Cylinder(
                    p.cbore_d / 2, p.cbore_depth, align=(Align.CENTER, Align.CENTER, Align.MAX)
                )
            )
    return part


def sections(p: Params) -> list[str]:
    """Through one insert column and one bolt column, and across both bolt rows."""
    return [f"x={p.zed_hole_x}", f"x={p.plate_hole_x}", f"y={p.plate_hole_y}", f"y={p.zed_hole_y}"]


def mates(p: Params) -> dict:
    top = p.thickness
    plate = Pos(0, 0, 0) * Box(
        PLATE_L, PLATE_W, PLATE_T, align=(Align.CENTER, Align.CENTER, Align.MAX)
    )

    zed = Pos(0, 0, top) * Box(ZED_EAR_L, ZED_EAR_W, ZED_SHEET_T, align=C_MIN)
    zed += Pos(0, 0, top) * Box(ZED_BODY_L, ZED_BODY_W, ZED_BODY_H, align=C_MIN)

    bolts = None
    m3 = None
    inserts = None
    for sx in (-1, 1):
        for sy in (-1, 1):
            zxy = Pos(sx * p.zed_hole_x, sy * p.zed_hole_y, 0)
            zed -= zxy * Pos(0, 0, top) * Cylinder(ZED_EAR_HOLE / 2, ZED_SHEET_T, align=C_MIN)
            # M3 x 8 SHCS: head on the ear, shank down into the insert.
            s = (
                zxy
                * Pos(0, 0, top + ZED_SHEET_T)
                * (
                    Cylinder(SHCS_M3_HEAD_D / 2, SHCS_M3_HEAD_H, align=C_MIN)
                    + Cylinder(1.5, SHCS_M3_LEN, align=(Align.CENTER, Align.CENTER, Align.MAX))
                )
            )
            m3 = s if m3 is None else m3 + s
            # Installed insert: brass melts into the hole wall, so it overlaps the part.
            ins = (
                zxy
                * Pos(0, 0, top)
                * Cylinder(
                    p.insert_od / 2,
                    p.insert_len,
                    align=(Align.CENTER, Align.CENTER, Align.MAX),
                )
            )
            inserts = ins if inserts is None else inserts + ins
            # 1/4-20 x 1/2 flanged button head seated on the counterbore floor.
            b = Pos(sx * p.plate_hole_x, sy * p.plate_hole_y, top - p.cbore_depth) * (
                Cylinder(FBHCS_14_HEAD_D / 2, FBHCS_14_HEAD_H, align=C_MIN)
                + Cylinder(
                    0.25 * INCH / 2, FBHCS_14_LEN, align=(Align.CENTER, Align.CENTER, Align.MAX)
                )
            )
            bolts = b if bolts is None else bolts + b
    return {
        "cheese_plate": plate,
        "zed_box_mini": zed,
        "m3_shcs": m3,
        "m3_insert": inserts,
        "fbhcs_1_4_20": bolts,
    }


def expect(p: Params) -> dict:
    return {
        "bbox": (143.0, 103.8, 6.7),
        "holes": {p.insert_hole: 4, p.bolt_clear: 4, p.cbore_d: 4},
        "hole_at": {
            4.2: [{"x": sx * 24.0, "y": sy * 47.3} for sx in (-1, 1) for sy in (-1, 1)],
            7.0: [{"x": sx * 49.5, "y": sy * 30.85} for sx in (-1, 1) for sy in (-1, 1)],
        },
        "allow_interference": ["m3_insert"],
        "flush": {"fbhcs_1_4_20": "+z"},
        "gap": {"fbhcs_1_4_20": 0.2},
    }


if __name__ == "__main__":
    print(build(Params()).bounding_box())
