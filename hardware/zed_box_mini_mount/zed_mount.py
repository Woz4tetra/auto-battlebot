"""ZED Box Mini on an INNOREL CP10 cheese plate through the printed adapter."""

import zed_cheese_plate_adapter as adapter
from build123d import Align, Cylinder, Pos
from zed_cheese_plate_adapter import Params

TITLE = "ZED BOX MINI MOUNT"

M3_MINOR_DIA = 2.5
P = Params()
T = P.thickness
C_MAX = (Align.CENTER, Align.CENTER, Align.MAX)
ZED_XY = [(sx * P.zed_hole_x, sy * P.zed_hole_y) for sx in (-1, 1) for sy in (-1, 1)]
BOLT_XY = [(sx * P.plate_hole_x, sy * P.plate_hole_y) for sx in (-1, 1) for sy in (-1, 1)]


def components():
    inserts = None
    for x, y in ZED_XY:
        ins = Pos(x, y, T) * Cylinder(P.insert_od / 2, P.insert_len, align=C_MAX)
        ins -= Pos(x, y, T) * Cylinder(M3_MINOR_DIA / 2, P.insert_len, align=C_MAX)
        inserts = ins if inserts is None else inserts + ins
    return {
        "plate": {
            "part": "innorel_cp10.py",
            "title": "INNOREL CP10 CHEESE PLATE",
            "description": "INNOREL CP10 cheese plate",
        },
        "adapter": {
            "part": "zed_cheese_plate_adapter.py",
            "title": "ZED BOX MINI ADAPTER PLATE",
            "description": "Printed PLA, 6 mm",
        },
        "zed": {
            "step": "out/step/zed_box_mini.step",
            "min_volume": 200,
            "at": Pos(0, 0, T),
            "title": "ZED BOX MINI",
            "mate_to": "adapter",
            "description": "Stereolabs ZED Box Mini",
        },
        "inserts": {
            "shape": inserts,
            "qty": 4,
            "title": "M3 X 6 HEAT-SET INSERT",
            "mate_to": "adapter",
            "material": "brass",
            "description": "M3 x 6 x Ø5 heat-set insert, shop stock",
        },
    }


def fasteners():
    return [
        {
            "name": "m3_zed",
            "size": "M3",
            "length": 6,
            "head": "shcs",
            "dir": "-Z",
            "at": [(x, y, T + adapter.ZED_SHEET_T) for x, y in ZED_XY],
            "into": "inserts",
            "thread": "insert",
            "insert_len": P.insert_len,
        },
        {
            "name": "bolt_plate",
            "title": "1/4-20 X 1/2 FLANGED BUTTON HEAD SCREW",
            "size": "1/4-20",
            "length": 12.7,
            "head": "button",
            "dir": "-Z",
            "at": [(x, y, T - P.cbore_depth) for x, y in BOLT_XY],
            "into": "plate",
            "thread": "tapped",
        },
    ]


EXPECT = {
    "allow_interference": [("inserts", "adapter")],
    "gap": {("zed", "plate"): 1.0},
}


def sections():
    return [f"x={P.plate_hole_x}", f"y={P.plate_hole_y}", f"x={P.zed_hole_x}"]
