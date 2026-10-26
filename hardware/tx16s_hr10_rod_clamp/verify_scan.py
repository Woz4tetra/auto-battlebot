"""Check the rod clamp halves, the plug and a hex driver against the raw TX16S scan.

The scan still has the handle sheath on, so vertices inside the sheath's envelope (rod
frame: the run within v -6..10.5, the arm above v 16, both within 5.6 of the rod plane)
are dropped; everything else (the bare arm, the radio body) must stay outside the parts.
The driver is a 2.5 hex bit (Ø2.5) in each counterbore and a Ø7 x 80 shaft up the screw
axis from the top face, which must miss the tower and the radio.

    ~/.local/share/build123d-part/venv/bin/python \
        hardware/tx16s_hr10_rod_clamp/verify_scan.py \
        ~/Downloads/radiomaster-tx16-1.snapshot.2.zip
"""

import argparse
import importlib.util
from pathlib import Path

import numpy as np
import pyvista as pv
from build123d import Cylinder, Pos

HERE = Path(__file__).resolve().parent


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


scan_patch = _load("scan_patch", HERE.parent / "tx16s_hr10_mount" / "scan_patch.py")
clamp = _load("tx16s_hr10_rod_clamp", HERE / "tx16s_hr10_rod_clamp.py")


def to_mesh(shape) -> pv.PolyData:
    verts, tris = shape.tessellate(0.02, 0.2)
    pts = np.array([(v.X, v.Y, v.Z) for v in verts])
    faces = np.hstack([np.full((len(tris), 1), 3), np.array(tris)]).ravel()
    return pv.PolyData(pts, faces).clean().triangulate()


def signed_gaps(mesh: pv.PolyData, pts: np.ndarray) -> np.ndarray:
    """Distance from each point to the mesh surface; negative inside."""
    return np.asarray(pv.PolyData(pts).compute_implicit_distance(mesh)["implicit_distance"])


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("scan_zip", type=Path)
    args = ap.parse_args()

    p = clamp.Params()
    top = clamp.top_radio(p)
    drivers = None
    for su, sv in p.screws:
        d = Pos(su, sv, p.t - p.cbore_depth + 0.05) * Cylinder(
            1.25, p.cbore_depth, align=clamp.C_MIN
        )
        d += Pos(su, sv, p.t + 0.05) * Cylinder(3.5, 80, align=clamp.C_MIN)
        drivers = d if drivers is None else drivers + d
    drivers = clamp.rod_to_radio() * drivers
    meshes = {
        "top": to_mesh(top),
        "bottom": to_mesh(clamp.bottom_radio(p)),
        "plug": to_mesh(clamp.plug_radio(p)),
        "drivers": to_mesh(drivers),
    }
    v = scan_patch.read_vertices(args.scan_zip) @ scan_patch.radio_rotation().T
    lo = np.min([m.bounds[::2] for m in meshes.values()], axis=0) - 15
    hi = np.max([m.bounds[1::2] for m in meshes.values()], axis=0) + 15
    near = v[np.all((v > lo) & (v < hi), axis=1)]

    o, u, vv, n = clamp.rod_axes()
    w = (near - o) @ np.stack([u, vv, n], axis=1)
    flat = np.abs(w[:, 2]) < 5.6
    run_sheath = flat & (w[:, 0] > -3) & (w[:, 1] > -6) & (w[:, 1] < 10.5)
    arm_sheath = flat & (w[:, 1] <= 16) & (w[:, 0] > -3) & (w[:, 0] < 5.6)
    keep = ~(run_sheath | arm_sheath)
    pts = near[keep]
    print(f"{len(near)} scan vertices near the clamp, {keep.sum()} outside the old sheath")

    for name, mesh in meshes.items():
        g = signed_gaps(mesh, pts)
        i = int(np.argmin(g))
        print(
            f"{name}: min gap {g[i]:.2f} mm at radio {np.round(pts[i], 1)}, "
            f"{int((g < 0).sum())} vertices inside"
        )
    bare = w[keep][:, 1] > 16
    g = signed_gaps(meshes["top"], pts[bare])
    gb = signed_gaps(meshes["bottom"], pts[bare])
    print(
        f"bare arm vs bores: top min {g.min():.2f} p1 {np.percentile(g, 1):.2f}, "
        f"bottom min {gb.min():.2f} p1 {np.percentile(gb, 1):.2f} mm (scan fit RMS 0.11)"
    )
    blocked = drivers & top
    print(f"drivers through the top half: {blocked.volume:.2f} mm^3")


if __name__ == "__main__":
    main()
