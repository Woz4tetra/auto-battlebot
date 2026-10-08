"""TBS Crossfire Nano RX standing on the board's 1x4 header (H2): render model only.

Origin at the header footprint origin on the board's wedge face, +Z away from the board, the pin
row along X. The RX's front (2.54 mm) pad row slides over the header pins and rests on the
header's 2.5 mm plastic, so the module stands 18 mm tall above it, its 11 mm edge along X.

Spec
- Process: none (render proxy, not manufactured).
- Interfaces: 4 pads on the RX's short edge over the header pins
  (TBS quickstart: GND, 5V, Ch1, Ch2).
- Critical dimensions: 18 x 11 module (TBS), 1.0 mm board, components 1.2 mm on one face,
  base 2.5 mm above the board (header plastic, HX PZ2.54 datasheet).

Open questions
- Board thickness and component height are estimates; TBS gives only 18 x 11.
"""

from dataclasses import dataclass

from build123d import Align, Box, Cylinder, Part, Pos, Rot

PROCESS = "none"
MATERIAL = "pc"  # density stand-in for FR4


@dataclass
class Params:
    width: float = 11.0  # TBS
    length: float = 18.0  # TBS
    board_t: float = 1.0  # estimate
    base_z: float = 2.5  # header plastic height
    chip: float = 7.0  # the RF SoC under its shield, estimate
    comp_h: float = 1.2  # estimate
    ufl_dia: float = 2.0  # antenna U.FL on the RX
    pitch: float = 2.54


def build(p: Params) -> Part:
    c = (Align.CENTER, Align.CENTER, Align.MIN)
    rx = Pos(0, 0, p.base_z) * Box(p.width, p.board_t, p.length, align=c)
    # Shielded SoC and the U.FL on the +Y face.
    rx += Pos(0, p.board_t / 2, p.base_z + 8.0) * Box(
        p.chip, p.comp_h, p.chip, align=(Align.CENTER, Align.MIN, Align.MIN)
    )
    rx += Pos(3.0, p.board_t / 2, p.base_z + 16.0) * (
        Rot(-90, 0, 0) * Cylinder(p.ufl_dia / 2, p.comp_h, align=c)
    )
    return rx


SECTIONS = ["x", "y"]


def expect(p: Params) -> dict:
    return {"bbox": (11.0, None, 18.0)}  # module size; it starts at z 2.5
