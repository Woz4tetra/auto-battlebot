"""Write cad/keepout.json: what each board face must clear, in the placeholder's board frame.

Frame (../../mr_stabs_mk2_pcb.py): origin at the hole-pattern centre on the roof face, +Z into the
chassis, +Y toward the rear wall. KiCad F side is the roof face (z 0 up), B side the wedge face
(z -1.6 down). cad/stl/ holds the robot parts already moved into that frame (export from the
placeholder's robot_in_board_frame() and esc_shapes()).

    ~/.local/share/pcba-design/venv/bin/python cad/make_keepout.py

Keys: "F" union of every section from z 0.1 to 3.78 (roof face, low parts only);
"B_low" sections from z -1.7 to -4 (anything on the wedge face: ESCs, the ledge ridge,
washers); "B_tall" sections down to -12 (USB-C body 8.5 mm, headers 8.5 mm plus solder).
"""

import glob
import json
import os

import numpy as np
import trimesh
from shapely.geometry import Point, Polygon, box, mapping
from shapely.ops import unary_union

HERE = os.path.dirname(os.path.abspath(__file__))
meshes = {
    os.path.basename(f)[:-4]: trimesh.load(f) for f in glob.glob(os.path.join(HERE, "stl", "*.stl"))
}


def section(z):
    polys = []
    for m in meshes.values():
        s = m.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
        if s is None:
            continue
        acc = Polygon()
        for loop in s.discrete:  # even-odd fill, so holes in a section stay holes
            if len(loop) >= 3:
                acc = acc.symmetric_difference(Polygon(loop[:, :2]).buffer(0))
        polys.append(acc)
    return unary_union(polys)


# The user's printed countersunk washers under each hole: 9.0 mm OD, 3.0 thick, D-cut at |x| 13.
washers = unary_union(
    [
        Point(sx * 9.8, sy * 9.8).buffer(4.5).intersection(box(-13, -50, 13, 50))
        for sx in (-1, 1)
        for sy in (-1, 1)
    ]
)
out = {
    "F": unary_union([section(z) for z in np.arange(0.1, 3.8, 0.25)]),
    "B_low": unary_union([section(z) for z in np.arange(-1.7, -4.0, -0.25)] + [washers]),
    "B_tall": unary_union([section(z) for z in np.arange(-1.7, -12.0, -0.5)] + [washers]),
}
json.dump(
    {k: mapping(v.simplify(0.02)) for k, v in out.items()},
    open(os.path.join(HERE, "keepout.json"), "w"),
)
for k, v in out.items():
    print(k, round(v.area, 1), "mm2")
