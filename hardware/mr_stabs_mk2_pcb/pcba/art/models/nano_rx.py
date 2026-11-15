"""TBS Crossfire Nano RX render proxy on H2."""

from dataclasses import dataclass

from build123d import Align, Box, Cylinder, Part, Pos, Rot

PROCESS = "none"
MATERIAL = "pc"


@dataclass
class Params:
    width_tbs: float = 11.0
    length_tbs: float = 18.0
    board_t_est: float = 1.0
    header_plastic_h: float = 2.5
    chip_est: float = 7.0
    comp_h_est: float = 1.2
    ufl_dia: float = 2.0
    pitch: float = 2.54


def build(p: Params) -> Part:
    c = (Align.CENTER, Align.CENTER, Align.MIN)
    rx = Pos(0, 0, p.header_plastic_h) * Box(p.width_tbs, p.board_t_est, p.length_tbs, align=c)
    rx += Pos(0, p.board_t_est / 2, p.header_plastic_h + 8.0) * Box(
        p.chip_est, p.comp_h_est, p.chip_est, align=(Align.CENTER, Align.MIN, Align.MIN)
    )
    rx += Pos(3.0, p.board_t_est / 2, p.header_plastic_h + 16.0) * (
        Rot(-90, 0, 0) * Cylinder(p.ufl_dia / 2, p.comp_h_est, align=c)
    )
    return rx


SECTIONS = ["x", "y"]


def expect(p: Params) -> dict:
    return {"bbox": (11.0, None, 18.0)}
