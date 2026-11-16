"""Write cad/keepout.json for each board face."""

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
HOLE_OFFSET = 9.8
WASHER_OD = 9.0
WASHER_FLAT_X = 13.0


def _even_odd_fill(loops):
    acc = Polygon()
    for loop in loops:
        if len(loop) >= 3:
            acc = acc.symmetric_difference(Polygon(loop[:, :2]).buffer(0))
    return acc


def section(z):
    polys = []
    for m in meshes.values():
        s = m.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
        if s is None:
            continue
        polys.append(_even_odd_fill(s.discrete))
    return unary_union(polys)


washers = unary_union(
    [
        Point(sx * HOLE_OFFSET, sy * HOLE_OFFSET)
        .buffer(WASHER_OD / 2)
        .intersection(box(-WASHER_FLAT_X, -50, WASHER_FLAT_X, 50))
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
