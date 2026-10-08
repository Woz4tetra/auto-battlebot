"""Render the assembled board: every part's 3D model, plus the parts KiCad has none for.

    SKILL/scripts/kicad python3 render_3d.py      (after make_fab.sh; reads mk2_routed.kicad_pcb)

Stock models come from SKILL/scripts/fetch_3dmodels.py; the rest are build123d models in
art/models/ (checked with the build123d skill's check.py, STEPs copied to lib/mk2.3dshapes/):
    L1   FNR4030S100MT inductor, the footprint KiCad ships without a model
    H2   the Crossfire Nano RX standing on its header
    ESP1 the U.FL plug and Molex 146153-0050 lead on the module's socket
Writes render_3d_{top,bottom,iso_top,iso_bottom}.png. The fab board is not modified.
"""

import os
import subprocess

import pcbnew

board = pcbnew.LoadBoard("mk2_routed.kicad_pcb")
SHAPES = "${KIPRJMOD}/lib/mk2.3dshapes/"


def add_model(fp, name, offset=(0, 0, 0)):
    m = pcbnew.FP_3DMODEL()
    m.m_Filename = SHAPES + name
    m.m_Offset = pcbnew.VECTOR3D(*offset)
    fp.Models().push_back(m)


for fp in board.GetFootprints():
    v = fp.GetValue()
    if v in ("10uH", "15uH"):
        fp.Models().clear()
        add_model(fp, "fnr4030_inductor.step")
    elif v == "NANO_RX":
        add_model(fp, "nano_rx.step")
    elif v.startswith("ESP32-S3-MINI-1U"):
        # U.FL socket centre, from the module footprint's own model: (-4.6, +5.1) mm, y up.
        add_model(fp, "ufl_plug.step", (-4.6, 5.1, 0))

# Drill origin at the hole-pattern centre (KiCad 100, 100), the board frame of
# ../mr_stabs_mk2_pcb.py and the chassis CAD, so the STEP drops into the robot assembly.
board.GetDesignSettings().SetAuxOrigin(pcbnew.VECTOR2I(pcbnew.FromMM(100), pcbnew.FromMM(100)))
out = "mk2_render.kicad_pcb"  # beside the project, so KIPRJMOD resolves
board.Save(out)
views = {
    "top": ["--side", "top"],
    "bottom": ["--side", "bottom"],
    "iso_top": ["--side", "top", "--rotate", "-40,0,25"],
    "iso_bottom": ["--side", "bottom", "--rotate", "-40,0,-25"],
}
# Plan views fill the frame at 1.9; the tilted views need 1.0 to keep the ear tips in.
ZOOM = {"top": "1.9", "bottom": "1.9", "iso_top": "1.05", "iso_bottom": "1.05"}
for name, args in views.items():
    subprocess.run(
        [
            "kicad-cli",
            "pcb",
            "render",
            *args,
            "--width",
            "2400",
            "--height",
            "1400",
            "--quality",
            "high",
            "--zoom",
            ZOOM[name],
            "--floor",
            "-o",
            f"render_3d_{name}.png",
            out,
        ],
        check=True,
        capture_output=True,
    )
    print(f"render_3d_{name}.png")
os.makedirs("step", exist_ok=True)
subprocess.run(
    [
        "kicad-cli",
        "pcb",
        "export",
        "step",
        "--subst-models",
        "--include-pads",
        "--include-silkscreen",
        "--drill-origin",
        "-o",
        "step/mr_stabs_mk2_pcba.step",
        out,
    ],
    check=True,
    capture_output=True,
)
print("step/mr_stabs_mk2_pcba.step")
