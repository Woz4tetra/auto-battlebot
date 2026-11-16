"""HX PZ2.54-1x4P right-angle header render proxy."""

from dataclasses import dataclass

from build123d import Align, Box, Part, Pos

PROCESS = "none"
MATERIAL = "brass"


@dataclass
class Params:
    n: int = 4
    pitch: float = 2.54
    pin: float = 0.64
    body: float = 2.5
    body_len: float = 9.96
    body_y0: float = 3.6
    mate: float = 6.0
    foot_y0: float = -1.2
    foot_t: float = 0.3


def build(p: Params) -> Part:
    pin_zc = p.body / 2
    body = Pos(0, -(p.body_y0 + p.body / 2), 0) * Box(
        p.body_len, p.body, p.body, align=(Align.CENTER, Align.CENTER, Align.MIN)
    )
    part = body
    for i in range(p.n):
        x = (i - (p.n - 1) / 2) * p.pitch
        y_tip = -(p.body_y0 + p.body + p.mate)
        y_back = -p.body_y0
        part += Pos(x, (y_tip + y_back) / 2, pin_zc) * Box(p.pin, y_back - y_tip, p.pin)
        y_foot = -p.foot_y0
        part += Pos(x, y_back + p.pin / 2, pin_zc / 2) * Box(p.pin, p.pin, pin_zc)
        part += Pos(x, (y_back + y_foot) / 2, p.foot_t / 2) * Box(p.pin, y_foot - y_back, p.foot_t)
    return part


SECTIONS = ["x", "y"]


def expect(p: Params) -> dict:
    return {"bbox": (p.body_len, -p.foot_y0 + p.body_y0 + p.body + p.mate, p.body)}
