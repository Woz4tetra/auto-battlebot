"""ZED X One S bracket: holds the camera by its Stereolabs front mount on a 1/4-20 ball head.

Function: the camera hangs from its front mount (ACC-151300, zed_x_one_s_front_mount.py),
and the front mount bolts to this bracket's two cheeks through its 2 x Ø5.4 slots. The
bracket's bottom face seats on a SmallRig 2948 ball head's platform
(smallrig_2948_ball_head.py), which threads into any 1/4-20 hole on the INNOREL CP10 cheese
plate.

- Process: FDM, PLA, printed bottom face down. No supports: the cheeks rise straight off the
  floor, and the only horizontal holes are the Ø5.2 insert holes.
- Front mount: its back face (camera side) bears on the cheek fronts at y = plate_y, and its
  bottom edge rests on the floor's front ledge, so the camera height repeats. The slots still
  allow ±5 mm for the screws, which only clamp.
- Front mount screws: 4 x McMaster 91292A117 (M4 x 0.7 x 12 mm 18-8 SHCS) from the front,
  2 per slot, each on a McMaster 93475A230 washer (M4, 9 mm OD, 0.8 mm), into 4 x McMaster
  94180A353 (M4 x 0.7 brass tapered heat-set, 7.9 mm long, for a 0.208" = 5.28 mm max hole).
  The washer matters: a bare Ø7 head bears on only 0.8 mm each side of the Ø5.4 slot. The
  4 mm plate and washer leave 7.2 mm of thread, 91% of the insert, with the tip 1.8 mm off
  the hole floor.
- Ball head: 1 x McMaster 93365A160 (1/4-20 brass tapered heat-set, 0.300" = 7.62 mm long,
  for a 0.349" = 8.86 mm max hole), pressed from the top so the platform's clamp load
  wedges it into its taper. The 5.5 mm stud then engages 5.1 mm of it.
- Camera field of view (110° x 80°, wide lens) is clear: every part of the bracket sits
  behind the plate, and the screw heads sit behind the lens's FOV cone vertex.

Coordinates: origin on the 1/4-20 axis in the bottom face, +Z up (print direction), +Y the
camera's viewing direction, X across. The optical axis runs along y at (0, axis_z).

Open questions
- The ball head's dimensions are photo-scaled (smallrig_2948_ball_head.py). Its platform is
  about Ø26, which the 50 x 32 bottom face covers with margin either way.
- Insert holes are McMaster's maximum hole sizes, rounded down; test-fit an insert before
  printing the bracket, since printed holes run 0.1 to 0.2 mm under.
"""

from dataclasses import dataclass
from pathlib import Path

import smallrig_2948_ball_head as ballhead
import zed_x_one_s_front_mount as front_mount
from build123d import (
    Align,
    Axis,
    Box,
    Compound,
    Cylinder,
    Location,
    Part,
    Pos,
    Rot,
    chamfer,
    export_step,
    fillet,
    import_step,
)

PROCESS = "fdm"
MATERIAL = "pla"
HERE = Path(__file__).parent
C_MIN = (Align.CENTER, Align.CENTER, Align.MIN)
CAMERA_STEP = HERE / "vendor" / "zed_x_one_s_wide.step"
CAMERA_BODY_STEP = HERE / "vendor" / "zed_x_one_s_wide_body.step"
# Camera STEP frame: lens toward -Y, front face at Y = -76.2706, front M2 square centered on
# (X, Z) = (0.5142, -17.6945). Measured from the STEP; matches datasheet p.8 (25 x 25 body, 21 mm
# square). Solid 9 of the STEP is the FOV frustum surface, not hardware.
CAM_FRONT_Y = -76.2705954
CAM_AXIS_X, CAM_AXIS_Z = 0.5142, -17.6945
WASHER_ID, WASHER_OD, WASHER_T = 4.3, 9.0, 0.8  # McMaster 93475A230


@dataclass
class Params:
    width: float = 50.0  # matches the front mount's 50 mm length
    floor_t: float = 8.0  # 1/4-20 insert 7.62 + 0.38 skin under its tip (through hole)
    floor_back: float = -16.0  # rear edge; 32 mm floor deep covers the Ø26 platform
    plate_y: float = 12.0  # front mount back face; puts the camera's mass over the stud
    plate_t: float = 4.0  # front mount, datasheet p.11
    plate_h: float = 30.0  # front mount, datasheet p.11
    cheek_inner: float = 13.2  # camera half width 12.5 + 0.7 clearance
    cheek_back: float = -6.0  # cheek depth 18 mm behind the plate
    cheek_top: float = 33.0  # covers the top insert (axis_z + 5 + 4.2 boss radius)
    slot_x: float = 18.0  # front mount slots, datasheet p.11
    screw_dz: float = 5.0  # 2 screws per slot at axis_z ± 5, inside the 20 mm slot
    m4_insert_hole: float = 5.2  # 94180A353: 0.208" max hole
    m4_insert_len: float = 7.9  # 94180A353
    m4_hole_depth: float = 9.0  # insert + 1 mm, fasteners.md
    q_insert_hole: float = 8.8  # 93365A160: 0.349" max hole, drill S = 0.348"
    q_insert_len: float = 7.62  # 93365A160: 0.300"
    q_insert_od: float = 9.0  # knurl OD, McMaster drawing range
    outer_r: float = 3.0  # floor's vertical corners
    cheek_r: float = 2.0  # cheek back edges
    bed_chamfer: float = 0.5  # elephant's foot, dfm/fdm.md

    @property
    def axis_z(self) -> float:
        """Optical axis height: the plate's bottom edge sits on the floor."""
        return self.floor_t + self.plate_h / 2


def build(p: Params) -> Part:
    half = p.width / 2
    depth = p.plate_y + p.plate_t - p.floor_back
    floor = Pos(0, p.floor_back, 0) * Box(
        p.width, depth, p.floor_t, align=(Align.CENTER, Align.MIN, Align.MIN)
    )
    part = floor
    cheek_w = half - p.cheek_inner
    for sx in (-1, 1):
        part += Pos(sx * (p.cheek_inner + cheek_w / 2), p.cheek_back, 0) * Box(
            cheek_w,
            p.plate_y - p.cheek_back,
            p.cheek_top,
            align=(Align.CENTER, Align.MIN, Align.MIN),
        )
    vertical = part.edges().filter_by(Axis.Z)
    floor_corners = vertical.filter_by(
        lambda e: abs(abs(e.center().Y - (p.floor_back + depth / 2)) - depth / 2) < 1e-6
    )
    assert len(floor_corners) == 4, f"expected 4 floor corners, got {len(floor_corners)}"
    part = fillet(floor_corners, p.outer_r)
    # Cheek back edges only; the cheek fronts stay sharp where the plate seats.
    cheek_backs = (
        part.edges().filter_by(Axis.Z).filter_by(lambda e: abs(e.center().Y - p.cheek_back) < 1e-6)
    )
    assert len(cheek_backs) == 4, f"expected 4 cheek back edges, got {len(cheek_backs)}"
    part = fillet(cheek_backs, p.cheek_r)
    bed = part.edges().filter_by(lambda e: e.center().Z < 1e-6 and e.length > 1)
    part = chamfer(bed, p.bed_chamfer)

    # 1/4-20 insert from the top, hole through.
    part -= Cylinder(p.q_insert_hole / 2, p.floor_t, align=C_MIN)
    # M4 inserts in the cheek fronts, running -Y.
    for sx in (-1, 1):
        for sz in (-1, 1):
            part -= (
                Pos(sx * p.slot_x, p.plate_y, p.axis_z + sz * p.screw_dz)
                * Rot(90, 0, 0)
                * Cylinder(p.m4_insert_hole / 2, p.m4_hole_depth, align=C_MIN)
            )
    return part


def camera_body_step() -> Path:
    """Stereolabs' STEP minus the FOV frustum, written once next to it in the STEP's frame."""
    if not CAMERA_BODY_STEP.exists():
        step = import_step(str(CAMERA_STEP))
        keep = [s for s in step.solids() if s.bounding_box().size.X < 40]
        assert len(keep) == len(step.solids()) - 1, "expected to drop exactly the FOV frustum"
        export_step(Compound(keep), str(CAMERA_BODY_STEP))
    return CAMERA_BODY_STEP


def camera_location(p: Params) -> Location:
    """Camera STEP frame -> bracket frame: lens along +Y, front face on the plate."""
    # Rot(0, 0, 180) maps (x, y) to (-x, -y); the Pos then lands the axis at (0, axis_z).
    return Pos(CAM_AXIS_X, p.plate_y + CAM_FRONT_Y, p.axis_z - CAM_AXIS_Z) * Rot(0, 0, 180)


def camera(p: Params):
    return camera_location(p) * import_step(str(camera_body_step()))


def placed_front_mount(p: Params) -> Part:
    return Pos(0, p.plate_y, p.axis_z) * front_mount.build(front_mount.Params())


def placed_ball_head(p: Params, bp: ballhead.Params | None = None) -> Part:
    """Upright ball head with its platform's top face on the bracket's bottom face."""
    bp = bp or ballhead.Params()
    return ballhead.top_location(bp).inverse() * ballhead.build(bp)


def inserts(p: Params) -> dict[str, Part]:
    m4 = []
    for sx in (-1, 1):
        for sz in (-1, 1):
            m4.append(
                Pos(sx * p.slot_x, p.plate_y, p.axis_z + sz * p.screw_dz)
                * Rot(90, 0, 0)
                * (
                    Cylinder(2.8, p.m4_insert_len, align=C_MIN)
                    - Cylinder(1.65, p.m4_insert_len, align=C_MIN)
                )
            )
    q = Pos(0, 0, p.floor_t - p.q_insert_len) * (
        Cylinder(p.q_insert_od / 2, p.q_insert_len, align=C_MIN)
        - Cylinder(2.55, p.q_insert_len, align=C_MIN)
    )
    return {"m4_inserts_94180A353": Compound(m4), "insert_1_4_20_93365A160": q}


def washers(p: Params) -> Part:
    """4 x McMaster 93475A230 on the front mount's front face, under the M4 heads."""
    face_y = p.plate_y + p.plate_t
    out = []
    for sx in (-1, 1):
        for sz in (-1, 1):
            out.append(
                Pos(sx * p.slot_x, face_y, p.axis_z + sz * p.screw_dz)
                * Rot(-90, 0, 0)
                * (
                    Cylinder(WASHER_OD / 2, WASHER_T, align=C_MIN)
                    - Cylinder(WASHER_ID / 2, WASHER_T, align=C_MIN)
                )
            )
    return Compound(out)


def mates(p: Params) -> dict:
    return {
        "front_mount": placed_front_mount(p),
        "camera": camera(p),
        "ball_head": placed_ball_head(p),
        **inserts(p),
    }


def sections(p: Params) -> list[str]:
    return [f"x={p.slot_x}", "x=0", f"z={p.axis_z + p.screw_dz}", "y=0"]


def expect(p: Params) -> dict:
    return {
        "bbox": (50.0, 32.0, 33.0),
        "holes": {5.2: 4, 8.8: 1},
        "hole_at": {
            5.2: [{"x": sx * 18.0, "z": 23.0 + sz * 5.0} for sx in (-1, 1) for sz in (-1, 1)],
            8.8: [{"x": 0.0, "y": 0.0}],
        },
        "clearance": {"camera": 0.5},
        "allow_interference": ["m4_inserts_94180A353", "insert_1_4_20_93365A160"],
        "mass_g": (None, 40),
    }


if __name__ == "__main__":
    print(build(Params()).bounding_box())
