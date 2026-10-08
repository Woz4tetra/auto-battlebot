"""Line-art render of the Mr Stabs Mk2 CAD for the silkscreen icon.

Software z-buffer: rasterize the assembly's triangles into depth and normal images, then mark
pixels where depth jumps (silhouettes) or the normal turns sharply (creases). Writes
robot_lines.png (white lines on black) for silk.py to vectorize.

    ~/.local/share/pcba-design/venv/bin/python art/render_robot.py [--preview]
"""

import glob
import math
import os
import sys

import numpy as np
import trimesh
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
STL = os.path.join(HERE, "robot_stl")
MOTOR_Y, MOTOR_Z = -9.04, 8.93  # drive axis in the assembly frame (placeholder's TILT and SHIFT)
WHEEL_X = 61.0  # wheel centre plane, outboard of the motor bores (x 16.3 to 54.5)


def load():
    parts = []
    for f in glob.glob(os.path.join(STL, "*.stl")):
        name = os.path.basename(f)
        if any(k in name for k in ("Apriltag", "Battery", "ESC", "Clamp Hub")):
            continue
        m = trimesh.load(f)
        if "Wheel" in name:
            side = -1 if "(1)" in name else 1
            m.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
            m.apply_translation([side * WHEEL_X, MOTOR_Y, MOTOR_Z])
        parts.append(m)
    return trimesh.util.concatenate(parts)


def view_matrix(az, el):
    a, e = math.radians(az), math.radians(el)
    rz = trimesh.transformations.rotation_matrix(a, [0, 0, 1])[:3, :3]
    rx = trimesh.transformations.rotation_matrix(-e, [1, 0, 0])[:3, :3]
    return rx @ rz


def render(mesh, rot, px):
    v = mesh.vertices @ rot.T
    tri = v[mesh.faces]
    n = mesh.face_normals @ rot.T
    lo, hi = v[:, :2].min(0), v[:, :2].max(0)
    w, h = int((hi[0] - lo[0]) * px) + 9, int((hi[1] - lo[1]) * px) + 9
    depth = np.full((h, w), -np.inf)
    nimg = np.zeros((h, w, 3))
    xy = (tri[:, :, :2] - lo) * px + 4
    for t in range(len(tri)):
        p = xy[t]
        x0, y0 = np.floor(p.min(0)).astype(int)
        x1, y1 = np.ceil(p.max(0)).astype(int) + 1
        xs, ys = np.meshgrid(np.arange(x0, x1) + 0.5, np.arange(y0, y1) + 0.5)
        (ax, ay), (bx, by), (cx, cy) = p
        d = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
        if abs(d) < 1e-12:
            continue
        w0 = ((by - cy) * (xs - cx) + (cx - bx) * (ys - cy)) / d
        w1 = ((cy - ay) * (xs - cx) + (ax - cx) * (ys - cy)) / d
        w2 = 1 - w0 - w1
        inside = (w0 >= 0) & (w1 >= 0) & (w2 >= 0)
        if not inside.any():
            continue
        z = w0 * tri[t, 0, 2] + w1 * tri[t, 1, 2] + w2 * tri[t, 2, 2]
        yy, xx = np.nonzero(inside)
        yy, xx = yy + y0, xx + x0
        zz = z[inside]
        closer = zz > depth[yy, xx]
        depth[yy[closer], xx[closer]] = zz[closer]
        nimg[yy[closer], xx[closer]] = n[t] * (1 if n[t, 2] >= 0 else -1)
    return depth, nimg


def lines(depth, nimg, px):
    fg = np.isfinite(depth)
    d = np.where(fg, depth, depth[fg].min() - 50)
    edge = np.zeros_like(fg)
    for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):
        a = np.roll(np.roll(d, dy, 0), dx, 1)
        na = np.roll(np.roll(nimg, dy, 0), dx, 1)
        fa = np.roll(np.roll(fg, dy, 0), dx, 1)
        jump = np.abs(d - a) > 1.2  # mm: silhouette and occlusion edges
        crease = (fg & fa) & ((nimg * na).sum(-1) < math.cos(math.radians(35)))
        edge |= jump | crease
    return edge[::-1]  # image rows top-down


if __name__ == "__main__":
    mesh = load()
    PX = 6.0  # pixels per mm of model; silk.py scales the result to the board
    depth, nimg = render(mesh, view_matrix(az=-35, el=28), PX)
    img = lines(depth, nimg, PX)
    Image.fromarray((img * 255).astype(np.uint8)).save(os.path.join(HERE, "robot_lines.png"))
    print("robot_lines.png", img.shape)
    if "--preview" in sys.argv:
        shade = np.where(np.isfinite(depth), 80 + 170 * np.abs(nimg[..., 2]), 255)[::-1]
        Image.fromarray(shade.astype(np.uint8)).save(os.path.join(HERE, "robot_shaded.png"))
