"""Turn the silkscreen images into polygons in mm: the Mk2 line renders and the BWBots logo.

    ~/.local/share/pcba-design/venv/bin/python art/vectorize.py

Inputs:
    robot: render_robot.py's renderer, re-run here at board resolution (view az 145, el 28),
        once from below and once from above.
    logo: logo_dark_h240.png from logo/gen_logo_icon.py --height 240 --out art/logo_dark_h240.png,
        the dashboard's dark-background variant (black half-disc drawn as an outline).

Writes robot_silk.json, robot_top_silk.json and logo_silk.json:
{"size": [w, h], "polys": [[exterior, hole, ...]]},
points in mm with the origin at the image centre, u right and v up as the image reads. silk.py
draws robot_top_silk.json on F.SilkS and the others on B.SilkS, mirrored so they read from below.
"""

import json
import math
import os

import numpy as np
import render_robot
import trimesh
from PIL import Image
from shapely.geometry import MultiPolygon, Polygon, box
from shapely.ops import unary_union

HERE = os.path.dirname(os.path.abspath(__file__))
MIN_LINE = 0.18  # mm; JLCPCB silk minimum is 0.15


def dilate(mask, r):
    out = mask.copy()
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy <= r * r:
                out |= np.roll(np.roll(mask, dy, 0), dx, 1)
    return out


def to_polys(mask, mm_per_px):
    """Bitmap (rows top-down) -> shapely geometry in mm, centred, v up."""
    h, w = mask.shape
    boxes = []
    for y in range(h):
        row = mask[y]
        edges = np.flatnonzero(np.diff(np.concatenate([[0], row.astype(np.int8), [0]])))
        for x0, x1 in zip(edges[::2], edges[1::2]):
            boxes.append(box(x0, h - y - 1, x1, h - y))
    g = unary_union(boxes)
    g = g.simplify(0.6, preserve_topology=True)
    from shapely import affinity

    g = affinity.translate(g, -w / 2, -h / 2)
    return affinity.scale(g, mm_per_px, mm_per_px, origin=(0, 0)), (w * mm_per_px, h * mm_per_px)


def dump(g, size, name, min_area):
    polys = []
    for p in g.geoms if isinstance(g, MultiPolygon) else [g]:
        if p.area < min_area:
            continue
        rings = [list(p.exterior.coords)] + [
            list(r.coords) for r in p.interiors if Polygon(r).area > min_area
        ]
        polys.append([[[round(x, 3), round(y, 3)] for x, y in ring] for ring in rings])
    json.dump(
        {"size": [round(v, 2) for v in size], "polys": polys}, open(os.path.join(HERE, name), "w")
    )
    print(f"{name}: {len(polys)} polygons, {size[0]:.1f} x {size[1]:.1f} mm")


# --- Robot, twice: seen from above for F silk and from below for B silk. The unflipped view
# (CAD +z toward the viewer) shows the robot's top; FLIP spins it 180 deg about y to show the
# underside. Each is
# fit inside its free box on the board at 25 px per board mm.
mesh = render_robot.load()
FLIP = trimesh.transformations.rotation_matrix(math.pi, [0, 1, 0])[:3, :3]
px_per_mm_board = 25.0


def robot(rot, box_w, box_h, name):
    v = mesh.vertices @ rot.T
    span = v[:, :2].max(0) - v[:, :2].min(0)
    scale = min(box_w / span[0], box_h / span[1])  # board mm per model mm
    px = scale * px_per_mm_board
    depth, nimg = render_robot.render(mesh, rot, px)
    lines = render_robot.lines(depth, nimg, px)
    lines = dilate(lines, int(MIN_LINE * px_per_mm_board / 2))
    g, size = to_polys(lines, 1 / px_per_mm_board)
    dump(g, size, name, min_area=0.02)


# Bottom: the robot seen from below, on the right ear tip's wedge face, x 35.5 to 46.3.
robot(render_robot.view_matrix(145, 28) @ FLIP, 10.4, 5.4, "robot_silk.json")
# Top: the robot seen from above, on the left ear tip of the roof face, x -46.4 to -35,
# y -17.1 to -11.3.
robot(render_robot.view_matrix(145, 28), 10.4, 5.4, "robot_top_silk.json")

# --- Logo: 5.4 mm tall. Its outline stroke is about 0.07 mm at that size, so every stroke is
# grown by a little under half the silk minimum on each side.
LOGO_H = 4.0  # stem, between the two headers' captions
im = np.array(Image.open(os.path.join(HERE, "logo_dark_h240.png")).convert("RGBA")).astype(float)
lum = im[..., :3].mean(-1) * im[..., 3] / 255
mask = lum > 128
mm_px = LOGO_H / mask.shape[0]
mask = dilate(mask, max(1, round(0.05 / mm_px)))
g, size = to_polys(mask, mm_px)
dump(g, size, "logo_silk.json", min_area=0.01)
