"""ZED Box Mini on the CP10 with the ZED X One S on a ball head."""

import os

import innorel_cp10
import smallrig_2948_ball_head as ballhead
import zed_cheese_plate_adapter as adapter
import zed_x_one_s_bracket as bracket
from build123d import Align, Cylinder, Location, Pos, Rot

INCH = 25.4
PLATE_T = innorel_cp10.Params().thickness

OPTIONS = {
    "top_end": (
        Pos(-88.25, 0, 0) * Rot(0, 0, 180),
        ballhead.Params(pan=180, tilt=15, tilt_dir=180, platform_spin_deg=90),
    ),
    "underside": (
        Pos(88.25, 0, -PLATE_T) * Rot(180, 0, 0),
        ballhead.Params(pan=180, tilt=30, platform_spin_deg=90),
    ),
    "long_edge": (
        Pos(90.0, 50.0, -PLATE_T / 2) * Rot(-90, 0, 0),
        ballhead.Params(pan=270, tilt=80, platform_spin_deg=90),
    ),
    "short_edge": (
        Pos(100.0, 13.95, -PLATE_T / 2) * Rot(0, 90, 0),
        ballhead.Params(pan=180, tilt=70, platform_spin_deg=90),
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
            "at": Pos(0, 0, adapter.Params().thickness),
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
        ("quarter_insert", "ball_head"),
    ],
}
