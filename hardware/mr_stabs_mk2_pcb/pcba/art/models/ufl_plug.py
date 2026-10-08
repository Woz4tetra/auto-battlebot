"""Mated U.FL plug with its 1.13 mm cable for the ESP32-S3-MINI-1U antenna: render model only.

Origin at the module's U.FL socket centre on the host board, +Z up. The Molex 146153-0050
lead runs forward (-Y) over the module's shield, between the front bosses and past the board's
front edge into the wire bay. It is the only exit on the roof face: the bosses close the stem's
front corners and the rear cross member sits 0.1 mm above the board.

Spec
- Process: none (render proxy, not manufactured).
- Interfaces: the plug sits on the module's socket, about 0.8 mm above the host board (module PCB).
- Critical dimensions: plug cap 2.0 mm across; mated height 2.5 mm max (Hirose U.FL series),
  so its top is at 3.3 mm, inside the 3.78 mm roof headroom. The 1.13 mm cable over the 2.55 mm
  shield tops out at 3.68 mm: 0.10 mm to the roof.

Open questions
- The socket's height on the module board (0.8 mm) is an estimate from the module drawing's
  2.55 mm shield height.
- 0.10 mm over the shield is not a working clearance; a 0.81 mm cable antenna leaves 0.42 mm.
"""

from dataclasses import dataclass

from build123d import Align, Cylinder, Part, Pos, Rot, Sphere  # noqa: F401

PROCESS = "none"
MATERIAL = "brass"


@dataclass
class Params:
    socket_z: float = 0.8  # module PCB top above the host board, estimate
    mated_h: float = 2.5  # Hirose U.FL mated height, max
    cap_dia: float = 2.0  # plug cap
    cable_dia: float = 1.13  # Molex 146153 cable
    shield_top: float = 2.55  # ESP32-S3-MINI-1U module height
    run_y: float = 28.4  # socket at board y +5.1 to 3 mm past the front edge at y -20.3


def build(p: Params) -> Part:
    c = (Align.CENTER, Align.CENTER, Align.MIN)
    plug = Pos(0, 0, p.socket_z) * Cylinder(p.cap_dia / 2, p.mated_h, align=c)
    zc = p.shield_top + p.cable_dia / 2  # resting on the shield
    plug += Pos(0, 0, zc) * Sphere(p.cable_dia / 2)
    plug += Pos(0, 0, zc) * (Rot(90, 0, 0) * Cylinder(p.cable_dia / 2, p.run_y, align=c))
    return plug


SECTIONS = ["x", "y"]


def expect(p: Params) -> dict:
    return {"bbox": (None, None, 2.88)}  # plug base at z 0.8 to the cable top at 3.68
