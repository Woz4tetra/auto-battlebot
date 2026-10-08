"""HX PZ2.54-1x4P WT right-angle SMD pin header (H2, LCSC C46061677): render model only.

EasyEDA has no 3D model for it. Origin at the footprint origin on the board surface, footprint
x along the pin row, model -Y the direction the mating pins point (footprint-local +y, y down),
+Z up from the board.

Spec
- Process: none (render proxy, not manufactured).
- Interfaces: four 1.02 x 3.20 mm SMD feet at 2.54 mm pitch (footprint pads at y = 0).
- Critical dimensions (Hanxia drawing, C46061677): 0.64 mm square pins, 2.54 mm pitch;
  insulator 2.5 x 2.5 mm, length B - 0.2 = 9.96 mm; mating pins 6.00 mm past the insulator;
  tails 4.80 mm from the insulator to the end of the foot.

Open questions
- The insulator's offset from the pads (3.6 mm, from EasyEDA's courtyard) is not dimensioned on
  the drawing; pins at 1.25 mm, the insulator's centre, assume it sits on the board.
"""

from dataclasses import dataclass

from build123d import Align, Box, Part, Pos

PROCESS = "none"
MATERIAL = "brass"


@dataclass
class Params:
    n: int = 4
    pitch: float = 2.54  # drawing
    pin: float = 0.64  # drawing, square
    body: float = 2.5  # insulator section, drawing
    body_len: float = 9.96  # B - 0.2 for 4 pins, drawing table
    body_y0: float = 3.6  # insulator's near face from the pads, EasyEDA courtyard
    mate: float = 6.0  # mating pin past the insulator, drawing
    foot_y0: float = -1.2  # foot end: tail 4.8 mm from the insulator (drawing)
    foot_t: float = 0.3  # foot thickness on the pad


def build(p: Params) -> Part:
    zc = p.body / 2  # pin centreline
    body = Pos(0, -(p.body_y0 + p.body / 2), 0) * Box(
        p.body_len, p.body, p.body, align=(Align.CENTER, Align.CENTER, Align.MIN)
    )
    part = body
    for i in range(p.n):
        x = (i - (p.n - 1) / 2) * p.pitch
        y_tip = -(p.body_y0 + p.body + p.mate)
        y_back = -p.body_y0
        # Mating pin through the insulator and out the far side.
        part += Pos(x, (y_tip + y_back) / 2, zc) * Box(p.pin, y_back - y_tip, p.pin)
        # Tail: down from the pin line to the board at the insulator's near face, then the foot
        # along the pad to its end (model y = -foot_y0, footprint-local y is down).
        y_foot = -p.foot_y0
        part += Pos(x, y_back + p.pin / 2, zc / 2) * Box(p.pin, p.pin, zc)
        part += Pos(x, (y_back + y_foot) / 2, p.foot_t / 2) * Box(p.pin, y_foot - y_back, p.foot_t)
    return part


SECTIONS = ["x", "y"]


def expect(p: Params) -> dict:
    # x: insulator length; y: foot end (-1.2 -> +1.2 in model y) to pin tip (12.1)
    return {"bbox": (9.96, 13.3, 2.5)}
