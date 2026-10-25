"""Cut the TX16S MK2 top-right height map out of the full-radio scan.

The scan is a 3MF (mm) of the whole radio, 3.0M vertices, posed upside down and rolled in
the scanner frame. This rotates it into the radio frame the mount uses and writes the top
surface over the right shoulder block as a 0.5 mm height map.

Radio frame: +X the pilot's right, +Y away from the pilot (the front face normal is -Y),
+Z up. Origin is the scanner's, so only directions mean anything.

Frame fit, done once on the full scan:
- Front face normal: area-weighted mean of face normals within 4 deg of the screen plane.
- Up: the four side walls cluster at 73.5 / 163.5 / 253.5 / 343.5 deg in the scanner YZ
  plane; 253.5 deg points out of the top (the handle side).

    venv/bin/python hardware/tx16s_hr10_mount/scan_patch.py \
        ~/Downloads/radiomaster-tx16-1.snapshot.2.zip
"""

import argparse
import re
import zipfile
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt, median_filter

FRONT_NORMAL = np.array([0.99840818, 0.05007949, -0.02594522])  # scanner frame
UP_ROLL_DEG = 253.5  # side-wall cluster pointing out of the top, scanner YZ plane
# Top-right block window, radio frame (mm). Covers the mount footprint, the two outer
# toggles, the handle hinge and the back edge drop.
WINDOW_X = (80.0, 116.0)
WINDOW_Y = (-34.0, 16.0)
CELL = 0.5

VERTEX = re.compile(rb'<vertex x="([^"]+)" y="([^"]+)" z="([^"]+)"')


def radio_rotation() -> np.ndarray:
    """Rows are the radio X, Y, Z axes in scanner coordinates."""
    front = FRONT_NORMAL / np.linalg.norm(FRONT_NORMAL)
    a = np.radians(UP_ROLL_DEG)
    up = np.array([0.0, np.cos(a), np.sin(a)])
    up -= front * (up @ front)
    up /= np.linalg.norm(up)
    right = np.cross(up, front)
    return np.stack([right, -front, up])


def read_vertices(zip_path: Path) -> np.ndarray:
    with zipfile.ZipFile(zip_path) as outer:
        name = next(n for n in outer.namelist() if n.endswith(".3mf"))
        with outer.open(name) as f3mf, zipfile.ZipFile(f3mf) as model_zip:
            with model_zip.open("3D/3DModel.model") as f:
                rows = [m.groups() for line in f if (m := VERTEX.search(line))]
    return np.array(rows, dtype=np.float64)


def height_map(v: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Top surface z over the window: max z per cell, then a 3x3 median against scan noise."""
    nx = round((WINDOW_X[1] - WINDOW_X[0]) / CELL)
    ny = round((WINDOW_Y[1] - WINDOW_Y[0]) / CELL)
    ix = np.floor((v[:, 0] - WINDOW_X[0]) / CELL).astype(int)
    iy = np.floor((v[:, 1] - WINDOW_Y[0]) / CELL).astype(int)
    ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    z = np.full(nx * ny, -np.inf)
    np.maximum.at(z, ix[ok] * ny + iy[ok], v[ok, 2])
    z = z.reshape(nx, ny)
    # Cells past the shell edge at the window corners have no points. Fill them from the
    # nearest measured cell; the mount footprint must have none (main() checks).
    empty = ~np.isfinite(z)
    _, (jx, jy) = distance_transform_edt(empty, return_indices=True)
    z = z[jx, jy]
    z = median_filter(z, size=3, mode="nearest")
    xs = WINDOW_X[0] + CELL * (np.arange(nx) + 0.5)
    ys = WINDOW_Y[0] + CELL * (np.arange(ny) + 0.5)
    return xs, ys, z, empty


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("scan_zip", type=Path, help="zip holding the radio .3mf")
    ap.add_argument(
        "--out",
        type=Path,
        default=Path(__file__).resolve().parent / "tx16s_top_right_heightmap.npz",
    )
    args = ap.parse_args()

    v = read_vertices(args.scan_zip) @ radio_rotation().T
    xs, ys, z, empty = height_map(v)
    ex, ey = np.nonzero(empty)
    in_footprint = (np.abs(xs[ex] - 97.5) < 12) & (np.abs(ys[ey] + 9) < 20)
    if in_footprint.any():
        raise SystemExit(
            f"{in_footprint.sum()} empty cells under the mount; the scan has a hole there"
        )
    np.savez_compressed(args.out, x=xs, y=ys, z=z)
    print(
        f"{args.out}: {z.shape[0]} x {z.shape[1]} cells, {empty.sum()} filled at the window edge, "
        f"z {z.min():.1f} to {z.max():.1f}"
    )


if __name__ == "__main__":
    main()
