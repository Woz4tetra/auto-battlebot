"""Check both clamp halves and a mated plug against the raw TX16S scan.

check.py and check_assembly.py measure against the lofted bar from x 58 to 80. The bar's
corner under the tower, the right arm, and everything the plug passes are only in the
scan, so this measures every scan vertex near the clamp.

    ~/.local/share/build123d-part/venv/bin/python \
        hardware/tx16s_hr10_handle_clamp/verify_scan.py \
        ~/Downloads/radiomaster-tx16-1.snapshot.2.zip
"""

import argparse
import importlib.util
from pathlib import Path

import numpy as np
import pyvista as pv

HERE = Path(__file__).resolve().parent


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


scan_patch = _load("scan_patch", HERE.parent / "tx16s_hr10_mount" / "scan_patch.py")
clamp = _load("tx16s_hr10_handle_clamp", HERE / "tx16s_hr10_handle_clamp.py")


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
    meshes = {
        "top": to_mesh(clamp.top_radio(p)),
        "bottom": to_mesh(clamp.bottom_radio(p)),
        "plug": to_mesh(clamp.plug_radio(p)),
    }
    v = scan_patch.read_vertices(args.scan_zip) @ scan_patch.radio_rotation().T
    lo = np.min([m.bounds[::2] for m in meshes.values()], axis=0) - 15
    hi = np.max([m.bounds[1::2] for m in meshes.values()], axis=0) + 15
    near = v[np.all((v > lo) & (v < hi), axis=1)]
    print(f"{len(near)} scan vertices within 15 mm of the clamp or plug")

    on_corner = near[:, 0] > p.x1  # bar corner, arm: past the clamp's right end
    for name, mesh in meshes.items():
        g = signed_gaps(mesh, near)
        i = int(np.argmin(g))
        print(
            f"{name}: min gap {g[i]:.2f} mm at radio {np.round(near[i], 1)}, "
            f"{int((g < 0).sum())} vertices inside"
        )
        if name == "top":
            gc = g[on_corner]
            j = int(np.argmin(gc))
            where = np.round(near[on_corner][j], 1)
            print(f"  tower over the corner: min gap {gc[j]:.2f} at radio {where}")


if __name__ == "__main__":
    main()
