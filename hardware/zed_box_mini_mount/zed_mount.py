"""ZED Box Mini on an INNOREL CP10 cheese plate through the printed adapter.

- Cheese plate: innorel_cp10.py, top face at z = 0.
- Adapter: zed_cheese_plate_adapter.py on the plate, top face at z = 6.7.
- ZED Box Mini: Stereolabs STEP as placed by the previous 12 mm adapter (sheet at z = 12),
  dropped 6 mm onto the new top face.
- 4 x M3 x 6 heat-set inserts in the adapter, 4 x M3 x 6 SHCS through the ZED ears.
- 4 x 1/4-20 x 1/2" flanged button heads (80/20 3342) in counterbores under the ZED body.
"""

from build123d import Align, Cylinder, Pos

from zed_cheese_plate_adapter import Params

P = Params()
T = P.thickness
C_MAX = (Align.CENTER, Align.CENTER, Align.MAX)
ZED_XY = [(sx * P.zed_hole_x, sy * P.zed_hole_y) for sx in (-1, 1) for sy in (-1, 1)]
BOLT_XY = [(sx * P.plate_hole_x, sy * P.plate_hole_y) for sx in (-1, 1) for sy in (-1, 1)]


def components():
    inserts = None
    for x, y in ZED_XY:
        ins = Pos(x, y, T) * Cylinder(P.insert_od / 2, P.insert_len, align=C_MAX)
        ins -= Pos(x, y, T) * Cylinder(2.5 / 2, P.insert_len, align=C_MAX)  # M3 minor dia
        inserts = ins if inserts is None else inserts + ins
    return {
        "plate": {"part": "innorel_cp10.py", "description": "INNOREL CP10 cheese plate"},
        "adapter": {"part": "zed_cheese_plate_adapter.py", "description": "Printed PLA, 6 mm"},
        "zed": {
            "step": "out/step/zed_box_mini.step",
            "min_volume": 200,
            "at": Pos(0, 0, T - 12.0),
            "description": "Stereolabs ZED Box Mini",
        },
        "inserts": {
            "shape": inserts,
            "qty": 4,
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
            "at": [(x, y, T + 1.0) for x, y in ZED_XY],  # on the 1 mm ear
            "into": "inserts",
            "thread": "insert",
            "insert_len": P.insert_len,
        },
        {
            "name": "bolt_plate",
            "size": "1/4-20",
            "length": 12.7,
            "head": "button",
            "dir": "-Z",
            "at": [(x, y, T - P.cbore_depth) for x, y in BOLT_XY],  # counterbore floor
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
