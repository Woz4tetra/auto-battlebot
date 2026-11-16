"""ZED X One S bracket on a 1/4-20 ball head."""

from dataclasses import dataclass
from pathlib import Path

import recipe
import smallrig_2948_ball_head as ballhead
import zed_x_one_s_front_mount as front_mount
from build123d import (
    Align,
    Compound,
    Cylinder,
    Location,
    Part,
    Pos,
    Rot,
    export_step,
    import_step,
)

PROCESS = "fdm"
MATERIAL = "pla"
HERE = Path(__file__).parent
C_MIN = (Align.CENTER, Align.CENTER, Align.MIN)
CAMERA_STEP = HERE / "vendor" / "zed_x_one_s_wide.step"
CAMERA_BODY_STEP = HERE / "vendor" / "zed_x_one_s_wide_body.step"
CAM_FRONT_Y = -76.2705954
CAM_AXIS_X, CAM_AXIS_Z = 0.5142, -17.6945
WASHER_93475A230 = (4.3, 9.0, 0.8)
WASHER_ID, WASHER_OD, WASHER_T = WASHER_93475A230


@dataclass
class Params:
    width: float = 50.0
    floor_t: float = 8.0
    floor_back: float = -16.0
    plate_y: float = 12.0
    plate_t: float = 4.0
    plate_h: float = 30.0
    cheek_inner: float = 13.2
    cheek_back: float = -6.0
    cheek_top: float = 33.0
    slot_x: float = 18.0
    screw_dz: float = 5.0
    m4_insert_hole: float = 5.2
    m4_insert_len: float = 7.9
    m4_hole_depth: float = 9.0
    q_insert_hole: float = 8.8
    q_insert_len: float = 7.62
    q_insert_od: float = 9.0
    outer_r: float = 3.0
    cheek_r: float = 2.0
    bed_chamfer: float = 0.5

    @property
    def axis_z(self) -> float:
        """Optical axis height: the plate's bottom edge sits on the floor."""
        return self.floor_t + self.plate_h / 2


def features(p: Params) -> list[dict]:
    """Floor, cheeks, edge finishes, then the inserts' holes: 11 CAD features (recipe.py)."""
    half = p.width / 2
    y1 = p.plate_y + p.plate_t
    depth = y1 - p.floor_back
    yc = p.floor_back + depth / 2
    cheek_w = half - p.cheek_inner
    cheek_x = p.cheek_inner + cheek_w / 2
    cheek_d = p.plate_y - p.cheek_back
    cheek_yc = p.cheek_back + cheek_d / 2
    cheek_mid_z = (p.floor_t + p.cheek_top) / 2
    m4 = [(sx * p.slot_x, p.axis_z + sz * p.screw_dz) for sx in (-1, 1) for sz in (-1, 1)]
    return [
        {"op": "sketch", "name": "Floor", "plane": "Top", "profiles": [
            {"rect": (0, yc, p.width, depth)}]},
        {"op": "extrude", "name": "Floor", "sketch": "Floor", "regions": [(0, yc)],
         "depth": p.floor_t},
        {"op": "sketch", "name": "Cheeks", "plane": "Top", "profiles": [
            {"rect": (sx * cheek_x, cheek_yc, cheek_w, cheek_d)} for sx in (-1, 1)]},
        {"op": "extrude", "name": "Cheeks", "sketch": "Cheeks", "mode": "add",
         "regions": [(sx * cheek_x, cheek_yc) for sx in (-1, 1)], "depth": p.cheek_top},
        {"op": "fillet", "name": "Floor corners", "size": p.outer_r, "tangent": False,
         "edges": [(sx * half, y, p.floor_t / 2) for sx in (-1, 1) for y in (p.floor_back, y1)]},
        {"op": "fillet", "name": "Cheek backs", "size": p.cheek_r, "tangent": False,
         "edges": [(sx * x, p.cheek_back, cheek_mid_z)
                   for sx in (-1, 1) for x in (p.cheek_inner, half)]},
        {"op": "chamfer", "name": "Elephant's foot chamfer", "size": p.bed_chamfer,
         "edges": [(0, p.floor_back, 0)]},
        {"op": "sketch", "name": "Ball head insert", "plane": "Top", "profiles": [
            {"circle": (0, 0, p.q_insert_hole)}]},
        {"op": "extrude", "name": "Ball head insert hole", "sketch": "Ball head insert",
         "regions": [(0, 0)], "depth": p.floor_t, "mode": "remove"},
        {"op": "sketch", "name": "Front mount inserts", "plane": "Front", "profiles": [
            {"circle": (x, z, p.m4_insert_hole)} for x, z in m4]},
        {"op": "extrude", "name": "Front mount insert holes", "sketch": "Front mount inserts",
         "regions": m4, "depth": p.m4_hole_depth, "start": -p.plate_y, "mode": "remove"},
    ]  # fmt: skip


def build(p: Params) -> Part:
    return recipe.build(features(p))


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
