"""ZED Box Mini on the INNOREL CP10 cheese plate, with the ZED X One S on a ball head.

Extends the ZED Box Mini stack (zed_cheese_plate_adapter.py, assembly_page.py) with the
camera: ZED X One S -> Stereolabs front mount (ACC-151300) -> printed bracket
(zed_x_one_s_bracket.py) -> SmallRig 2948 ball head -> any 1/4-20 hole on the cheese plate.

The ball head takes a 1/4-20 x 1/2" set screw (McMaster 92311A537) as a double-ended stud:
6.35 mm into the plate's tapped hole, 6.35 mm into the head's bottom thread. Every option
below uses a hole the adapter and ZED Box Mini leave open. Pick one with MOUNT_OPTION:

| Option | Hole | Head | Camera |
| --- | --- | --- | --- |
| top_end (default) | top face, (88.25, 0) | upright, 15° tilt | upright, looking +X, 15° down |
| underside | bottom face, (88.25, 0) | hanging, 30° into the notch | inverted, +X, 30° down |
| long_edge | long edge, x = 90, tapped 8 mm deep | 80° into the notch | upright, +Y, 10° down |
| short_edge | short edge, y = 13.95, 8 mm deep | 70° into the notch | upright, +X, 20° down |

The underside option images upside down; set the camera's flip in the ZED SDK.

    MOUNT_OPTION=long_edge scripts/run check_assembly.py zed_x_one_s_assembly.py \
        --out out/assembly_long_edge

Coordinates: the adapter frame, the same as assembly_page.py. Origin at the center of the
cheese plate's top face, X along its 200 mm length, +Z up toward the ZED Box Mini.

The ZED Box Mini comes from out/step/zed_box_mini.step, which assembly_page.py writes in
this frame from Stereolabs' 85 MB STEP (kept out of git). The adapter's own screws and
inserts are checked on that page and left out here.

Ball head poses come from solving pan and spin numerically so the camera points where the
table says; `pan` also keeps the wing knob off the adapter and ZED Box Mini.
"""

import os

import smallrig_2948_ball_head as ballhead
import zed_x_one_s_bracket as bracket
from build123d import Align, Cylinder, Location, Pos, Rot

INCH = 25.4
PLATE_T = 10.0  # innorel_cp10.Params.thickness

OPTIONS = {
    "top_end": (Pos(88.25, 0, 0), ballhead.Params(pan=180, tilt=15, tilt_dir=180, spin=90)),
    "underside": (
        Pos(88.25, 0, -PLATE_T) * Rot(180, 0, 0),
        ballhead.Params(pan=180, tilt=30, spin=90),
    ),
    "long_edge": (
        Pos(90.0, 50.0, -PLATE_T / 2) * Rot(-90, 0, 0),
        ballhead.Params(pan=270, tilt=80, spin=90),
    ),
    "short_edge": (
        Pos(100.0, 13.95, -PLATE_T / 2) * Rot(0, 90, 0),
        ballhead.Params(pan=180, tilt=70, spin=90),
    ),
}
OPTION = os.environ.get("MOUNT_OPTION", "top_end")
assert OPTION in OPTIONS, f"MOUNT_OPTION must be one of {sorted(OPTIONS)}"
BASE, BALL = OPTIONS[OPTION]
BP = bracket.Params()


def bracket_location() -> Location:
    return BASE * ballhead.top_location(BALL)


def _world(loc: Location, v: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple((loc * Pos(*v)).position)


def _world_dir(loc: Location, v: tuple[float, float, float]) -> tuple[float, float, float]:
    o, p = _world(loc, (0, 0, 0)), _world(loc, v)
    return tuple(b - a for a, b in zip(o, p))


def components():
    bl = bracket_location()
    ins = bracket.inserts(BP)
    stud = Cylinder(0.125 * INCH, 0.5 * INCH, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    return {
        "cheese_plate": {
            "part": "innorel_cp10.py",
            "color": "#9aa3ad",
            "description": "INNOREL CP10 cheese plate, 6061",
        },
        "adapter": {
            "part": "zed_cheese_plate_adapter.py",
            "color": "#6fb07f",
            "description": "ZED Box Mini adapter, PLA",
        },
        "zed_box_mini": {
            "step": "out/step/zed_box_mini.step",
            "min_volume": 200,
            "color": "#3a3f47",
            "description": "Stereolabs ZED Box Mini",
        },
        "set_screw": {
            "shape": BASE * stud,
            "color": "#c0c4c8",
            "qty": 1,
            "part_number": "92311A537",
            "vendor": "McMaster-Carr",
            "material": "steel",
            "description": '1/4-20 x 1/2" 18-8 cup-point set screw, ball head stud',
        },
        "ball_head": {
            "shape": BASE * ballhead.build(BALL),
            "color": "#2b2f36",
            "description": "SmallRig 2948 mini ball head, cold shoe removed",
        },
        "bracket": {
            "part": "zed_x_one_s_bracket.py",
            "at": bl,
            "color": "#e0a03a",
            "description": "ZED X One S bracket, PLA",
        },
        "front_mount": {
            "shape": bl * bracket.placed_front_mount(BP),
            "color": "#5b8fd9",
            "description": "Stereolabs ZED X One S front mount ACC-151300",
        },
        "camera": {
            "step": str(bracket.camera_body_step().relative_to(bracket.HERE)),
            "at": bl * bracket.camera_location(BP),
            "color": "#20242a",
            "description": "Stereolabs ZED X One S, wide lens",
        },
        "m4_inserts": {
            "shape": bl * ins["m4_inserts_94180A353"],
            "color": "#c8a24a",
            "qty": 4,
            "part_number": "94180A353",
            "vendor": "McMaster-Carr",
            "description": "M4 x 0.7 brass tapered heat-set insert, 7.9 mm",
        },
        "washers": {
            "shape": bl * bracket.washers(BP),
            "color": "#c0c4c8",
            "qty": 4,
            "part_number": "93475A230",
            "vendor": "McMaster-Carr",
            "material": "steel",
            "description": "M4 18-8 washer, 9 mm OD, 0.8 mm",
        },
        "quarter_insert": {
            "shape": bl * ins["insert_1_4_20_93365A160"],
            "color": "#c8a24a",
            "part_number": "93365A160",
            "vendor": "McMaster-Carr",
            "description": '1/4-20 brass tapered heat-set insert, 0.300"',
        },
    }


def fasteners():
    bl = bracket_location()
    head_y = BP.plate_y + BP.plate_t + bracket.WASHER_T
    seats = [
        _world(bl, (sx * BP.slot_x, head_y, BP.axis_z + sz * BP.screw_dz))
        for sx in (-1, 1)
        for sz in (-1, 1)
    ]
    return [
        {
            "name": "m4_front_mount",
            "size": "M4",
            "length": 12,
            "head": "shcs",
            "dir": _world_dir(bl, (0, -1, 0)),
            "at": seats,
            "into": "m4_inserts",
            "thread": "insert",
            "insert_len": BP.m4_insert_len,
            "mcmaster": "91292A117",
        }
    ]


def sections():
    bl = bracket_location()
    x, y, _ = _world(bl, (BP.slot_x, 0, 0))
    bx, by, _ = _world(BASE, (0, 0, 0))
    return [f"y={by:.2f}", f"x={bx:.2f}"]


EXPECT = {
    "allow_interference": [
        ("m4_inserts", "bracket"),
        ("quarter_insert", "bracket"),
        ("set_screw", "cheese_plate"),
        ("set_screw", "ball_head"),
        # The stud is 1/4-20 major diameter; the insert is modeled at its minor bore.
        ("quarter_insert", "ball_head"),
    ],
}
