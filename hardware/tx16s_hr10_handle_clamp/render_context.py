"""Render the HR10 handle clamp, receptacle and mated plug on the full scanned TX16S.

Writes context_*.png into the check_assembly.py output directory and adds them to its
report.json, so page_assembly.py embeds them with the other drawings. Run
check_assembly.py first.

    ~/.local/share/build123d-part/venv/bin/python \
        hardware/tx16s_hr10_handle_clamp/render_context.py \
        ~/Downloads/radiomaster-tx16-1.snapshot.2.zip
"""

import argparse
import importlib.util
import json
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


clamp = _load("tx16s_hr10_handle_clamp", HERE / "tx16s_hr10_handle_clamp.py")
mount_render = _load("mount_render_context", HERE.parent / "tx16s_hr10_mount" / "render_context.py")

# name, label, camera direction from the focus, focus target, distance
VIEWS = [
    ("context_pilot", "pilot's view, front right", (0.55, -1.0, 0.75), "radio", 600),
    ("context_back", "back right", (0.85, 1.0, 0.65), "radio", 600),
    ("context_right", "right side", (1.0, 0.0, 0.1), "radio", 760),
    ("context_top", "top, pilot at the bottom", (0.0, -0.25, 1.0), "radio", 560),
    ("context_closeup", "close-up, back right", (0.9, 1.0, 0.6), "clamp", 170),
]


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("scan_zip", type=Path)
    ap.add_argument("--out", type=Path, default=HERE / "out" / "tx16s_hr10_handle_clamp_assembly")
    args = ap.parse_args()

    p = clamp.Params()
    top = mount_render.to_mesh(clamp.top_radio(p))
    bottom = mount_render.to_mesh(clamp.bottom_radio(p))
    receptacle = mount_render.to_mesh(clamp.receptacle_radio(p))
    plug = mount_render.to_mesh(clamp.plug_radio(p))
    radio = mount_render.radio_mesh(args.scan_zip)

    focus = {
        "radio": np.array(radio.center) + [0, 0, 25],
        "clamp": np.array(top.center) + [0, 0, 5],
    }
    written = []
    for name, label, direction, target, dist in VIEWS:
        pl = pv.Plotter(off_screen=True, window_size=(1400, 1100))
        pl.set_background("white")
        pl.add_mesh(radio, color="#3a3f47", smooth_shading=True, specular=0.2)
        pl.add_mesh(top, color="#e8a33d", smooth_shading=True, specular=0.3)
        pl.add_mesh(bottom, color="#d98a1f", smooth_shading=True, specular=0.3)
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
