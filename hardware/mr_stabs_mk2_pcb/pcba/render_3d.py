"""Render the assembled Mk2 board with custom models for parts KiCad lacks."""

import os
import subprocess

import pcbnew

board = pcbnew.LoadBoard("mk2_routed.kicad_pcb")
SHAPES = "${KIPRJMOD}/lib/mk2.3dshapes/"


def add_model(fp, name, offset=(0, 0, 0), rotate=(0, 0, 0)):
    m = pcbnew.FP_3DMODEL()
    m.m_Filename = SHAPES + name
    m.m_Offset = pcbnew.VECTOR3D(*offset)
    m.m_Rotation = pcbnew.VECTOR3D(*rotate)
    fp.Models().push_back(m)


NANO_RX_OFFSET = (0, -6.65, -2.45)
NANO_RX_ROTATION = (0, 0, 180)
UFL_SOCKET_OFFSET = (-4.6, 5.1, 0)
PLAN_ZOOM = "1.9"
ISO_ZOOM = "1.05"

for fp in board.GetFootprints():
    v = fp.GetValue()
    if v in ("10uH", "15uH"):
        fp.Models().clear()
        add_model(fp, "fnr4030_inductor.step")
    elif v == "NANO_RX":
        add_model(fp, "nano_rx.step", NANO_RX_OFFSET, NANO_RX_ROTATION)
    elif v.startswith("ESP32-S3-MINI-1U"):
        add_model(fp, "ufl_plug.step", UFL_SOCKET_OFFSET)

board.GetDesignSettings().SetAuxOrigin(pcbnew.VECTOR2I(pcbnew.FromMM(100), pcbnew.FromMM(100)))
out = "mk2_render.kicad_pcb"  # beside the project, so KIPRJMOD resolves
board.Save(out)
views = {
    "top": ["--side", "top"],
    "bottom": ["--side", "bottom"],
    "iso_top": ["--side", "top", "--rotate", "-40,0,25"],
    "iso_bottom": ["--side", "bottom", "--rotate", "-40,0,-25"],
}
ZOOM = {
    "top": PLAN_ZOOM,
    "bottom": PLAN_ZOOM,
    "iso_top": ISO_ZOOM,
    "iso_bottom": ISO_ZOOM,
}
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
