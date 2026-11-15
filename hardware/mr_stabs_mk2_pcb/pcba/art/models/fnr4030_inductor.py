"""FNR4030S100MT 10 uH shielded power inductor (L1, LCSC C167879): render model only.

KiCad's library has no 3D model for Inductor_SMD:L_Taiyo-Yuden_NR-40xx, so the board render showed
bare pads. Origin at the footprint origin on the board surface, +Z up from the board.

Spec
- Process: none (render proxy, not manufactured).
- Interfaces: sits on the footprint's two pads, centred.
- Critical dimensions: body 4.0 x 4.0, 3.0 tall (LCSC package "SMD,4x4mm"; 4030 = 4.0 x 3.0 max).

Open questions
- Height is the series maximum, 3.0 mm; the roof headroom over it is 0.78 mm.
"""

from dataclasses import dataclass

from build123d import Align, Axis, Box, Cylinder, Part, Pos, chamfer

PROCESS = "none"
MATERIAL = "steel"


@dataclass
class Params:
    body_side: float = 4.0
    body_max_height: float = 3.0
    top_chamfer: float = 0.3
    winding_opening_dia: float = 3.0
    core_depth: float = 0.15


def build(p: Params) -> Part:
    body = Box(
        p.body_side, p.body_side, p.body_max_height, align=(Align.CENTER, Align.CENTER, Align.MIN)
    )
    top = body.edges().filter_by(Axis.Z, reverse=True).group_by(Axis.Z)[-1]
    assert len(top) == 4
    body = chamfer(top, p.top_chamfer)
    body -= Pos(0, 0, p.body_max_height - p.core_depth) * Cylinder(
        p.winding_opening_dia / 2, p.core_depth, align=(Align.CENTER, Align.CENTER, Align.MIN)
    )
    return body


SECTIONS = ["x", "y"]


def expect(p: Params) -> dict:
    return {"bbox": (4.0, 4.0, 3.0)}
