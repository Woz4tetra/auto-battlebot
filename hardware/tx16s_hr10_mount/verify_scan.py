"""Check the HR10 mount and a mated plug against the raw TX16S scan, not the height map.

check.py measures against a smoothed slab of the top surface, which cannot hold the
handle arm, the toggle levers, or anything the plug passes. This places the block and the
plug envelope in the radio frame and measures every scan vertex near them.

    ~/.local/share/build123d-part/venv/bin/python hardware/tx16s_hr10_mount/verify_scan.py \
        ~/Downloads/radiomaster-tx16-1.snapshot.2.zip
"""

import argparse
import importlib.util
from pathlib import Path

import numpy as np
import pyvista as pv

HERE = Path(__file__).resolve().parent


def _load(name: str):
    spec = importlib.util.spec_from_file_location(name, HERE / f"{name}.py")
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


scan_patch = _load("scan_patch")
mount = _load("tx16s_hr10_mount")


def to_mesh(shape) -> pv.PolyData:
    verts, tris = shape.tessellate(0.02, 0.2)
    pts = np.array([(v.X, v.Y, v.Z) for v in verts])
    faces = np.hstack([np.full((len(tris), 1), 3), np.array(tris)]).ravel()
    return pv.PolyData(pts, faces).clean().triangulate()


def signed_gaps(mesh: pv.PolyData, pts: np.ndarray) -> np.ndarray:
    """Distance from each point to the mesh surface; negative inside."""
    cloud = pv.PolyData(pts)
    return np.asarray(cloud.compute_implicit_distance(mesh)["implicit_distance"])


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("scan_zip", type=Path)
    args = ap.parse_args()

    p = mount.Params()
    to_radio = mount.radio_to_part(p).inverse()
    block = to_mesh(to_radio * mount.build(p))
    plug = to_mesh(to_radio * mount.plug(p))

    v = scan_patch.read_vertices(args.scan_zip) @ scan_patch.radio_rotation().T
    lo, hi = np.array(block.bounds[::2]) - 15, np.array(block.bounds[1::2]) + 15
    lo = np.minimum(lo, np.array(plug.bounds[::2]) - 15)
    hi = np.maximum(hi, np.array(plug.bounds[1::2]) + 15)
    near = v[np.all((v > lo) & (v < hi), axis=1)]
    print(f"{len(near)} scan vertices within 15 mm of the block or plug")

    for name, mesh in (("block", block), ("plug", plug)):
        g = signed_gaps(mesh, near)
        i = int(np.argmin(g))
        inside = int((g < 0).sum())
        where = np.round(near[i], 1)
        print(f"{name}: min gap {g[i]:.2f} mm at radio {where}, {inside} vertices inside")

    # Bond line: scan points within 2.5 mm of the block are the ones under the base; their
    # gap is what the tape fills.
    g = signed_gaps(block, near)
    under = near[(g > 0) & (g < 2.5)]
    gu = g[(g > 0) & (g < 2.5)]
    if len(under):
        q = np.percentile(gu, [1, 50, 99])
        gaps = f"p1 {q[0]:.2f} p50 {q[1]:.2f} p99 {q[2]:.2f}"
        print(f"bond line: {len(under)} scan vertices within 2.5 mm, gap {gaps}")


if __name__ == "__main__":
    main()
