"""Render the HR10 mount, receptacle and mated plug on the full scanned TX16S.

Writes context_*.png into the check.py output directory and adds them to its report.json,
so page.py embeds them with the other renders. Run check.py first.

    ~/.local/share/build123d-part/venv/bin/python hardware/tx16s_hr10_mount/render_context.py \
        ~/Downloads/radiomaster-tx16-1.snapshot.2.zip
"""

import argparse
import importlib.util
import json
import re
import zipfile
from pathlib import Path

import numpy as np
import pyvista as pv

HERE = Path(__file__).resolve().parent
TRIANGLE = re.compile(rb'<triangle v1="(\d+)" v2="(\d+)" v3="(\d+)"')


def _load(name: str):
    spec = importlib.util.spec_from_file_location(name, HERE / f"{name}.py")
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


scan_patch = _load("scan_patch")
mount = _load("tx16s_hr10_mount")

# name, camera direction from the focus, focus, distance, zoom target
VIEWS = [
    ("context_pilot", "pilot's view, front right", (0.55, -1.0, 0.75), "radio", 560),
    ("context_back", "back right", (0.85, 1.0, 0.65), "radio", 560),
    ("context_right", "right side", (1.0, 0.0, 0.1), "radio", 720),
    ("context_top", "top, pilot at the bottom", (0.0, -0.25, 1.0), "radio", 520),
    ("context_closeup", "close-up, front right", (0.8, -1.0, 0.6), "block", 150),
]


def read_triangles(zip_path: Path) -> np.ndarray:
    with zipfile.ZipFile(zip_path) as outer:
        name = next(n for n in outer.namelist() if n.endswith(".3mf"))
        with outer.open(name) as f3mf, zipfile.ZipFile(f3mf) as model_zip:
            with model_zip.open("3D/3DModel.model") as f:
                rows = [m.groups() for line in f if (m := TRIANGLE.search(line))]
    return np.array(rows, dtype=np.int64)


def radio_mesh(zip_path: Path) -> pv.PolyData:
    v = scan_patch.read_vertices(zip_path) @ scan_patch.radio_rotation().T
    t = read_triangles(zip_path)
    faces = np.hstack([np.full((len(t), 1), 3), t]).ravel()
    return pv.PolyData(v, faces).decimate_pro(0.7)


def to_mesh(shape) -> pv.PolyData:
    verts, tris = shape.tessellate(0.03, 0.2)
    pts = np.array([(v.X, v.Y, v.Z) for v in verts])
    faces = np.hstack([np.full((len(tris), 1), 3), np.array(tris)]).ravel()
    return pv.PolyData(pts, faces)


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("scan_zip", type=Path)
    ap.add_argument("--out", type=Path, default=HERE / "out" / "tx16s_hr10_mount")
    args = ap.parse_args()

    p = mount.Params()
    to_radio = mount.radio_to_part(p).inverse()
    block = to_mesh(to_radio * mount.build(p))
    receptacle = to_mesh(to_radio * mount.receptacle(p))
    plug = to_mesh(to_radio * mount.plug(p))
    radio = radio_mesh(args.scan_zip)

    focus = {
        "radio": np.array(radio.center) + [0, 0, 25],
        "block": np.array(block.center) + [0, 0, 12],
    }
    written = []
    for name, label, direction, target, dist in VIEWS:
        pl = pv.Plotter(off_screen=True, window_size=(1400, 1100))
        pl.set_background("white")
        pl.add_mesh(radio, color="#3a3f47", smooth_shading=True, specular=0.2)
        pl.add_mesh(block, color="#e8a33d", smooth_shading=True, specular=0.3)
        pl.add_mesh(receptacle, color="#c9ccd1", smooth_shading=True, specular=0.6)
        pl.add_mesh(plug, color="#9aa3ad", smooth_shading=True, opacity=0.6)
        d = np.array(direction, float)
        d /= np.linalg.norm(d)
        f = focus[target]
        up = (0, 1, 0) if name == "context_top" else (0, 0, 1)
        pl.camera_position = [tuple(f + d * dist), tuple(f), up]
        pl.add_text(label, font_size=11, color="black")
        path = args.out / f"{name}.png"
        pl.screenshot(path)
        pl.close()
        written.append(path.name)
        print(path)

    report_path = args.out / "report.json"
    report = json.loads(report_path.read_text())
    report["files"] = written + [n for n in report["files"] if n not in written]
    report_path.write_text(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
