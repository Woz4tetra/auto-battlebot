"""Custom ESP32-S3 PCBA space claim for the cage pack."""

from dataclasses import dataclass
from functools import cache
from typing import NamedTuple

import b123d_fix  # noqa: F401
import cage_layout as cl
import recipe
from build123d import Align, Box, Compound, Part, Pos, Rot, Shape, import_step

PROCESS = "none"
MATERIAL = "nylon"
C_MIN = (Align.CENTER, Align.CENTER, Align.MIN)


@dataclass
class Params:
    m3_close_clearance_dia: float = 3.2


def holes() -> list[tuple[float, float]]:
    return [
        (cl.PCBA_X0 + 13.5, cl.PCBA_Y0 + 3.5),
        (cl.PCBA_X0 + 13.5, cl.PCBA_Y1 - 3.5),
        (cl.PCBA_X1 - 3.5, cl.PCBA_Y0 + 3.5),
        (cl.PCBA_X1 - 3.5, cl.PCBA_Y1 - 3.5),
    ]


def features(p: Params) -> list[dict]:
    """The bare board: outline with its four mounting holes, one extrude."""
    w, h = cl.PCBA_X1 - cl.PCBA_X0, cl.PCBA_Y1 - cl.PCBA_Y0
    xc, yc = (cl.PCBA_X0 + cl.PCBA_X1) / 2, (cl.PCBA_Y0 + cl.PCBA_Y1) / 2
    return [
        {"op": "sketch", "name": "Board outline", "plane": "Top", "profiles": [
            {"rect": (xc, yc, w, h)},
            *[{"circle": (x, y, p.m3_close_clearance_dia)} for x, y in holes()]]},
        {"op": "extrude", "name": "Board", "sketch": "Board outline", "regions": [(xc, yc)],
         "depth": cl.PCBA_T, "start": cl.PCBA_Z},
    ]  # fmt: skip


def build(p: Params) -> Part:
    return recipe.build(features(p))


TOP = cl.PCBA_Z + cl.PCBA_T
X0, Y0 = cl.PCBA_X0, cl.PCBA_Y0


class BoxEnvelope(NamedTuple):
    w: float
    d: float
    h: float


class Placement(NamedTuple):
    model: str | BoxEnvelope
    rot_deg: float
    x: float
    y: float


GH4V = "JST_GH_BM04B-GHS-TBT_1x04-1MP_P1.25mm_Vertical.step"
GH2V = "JST_GH_BM02B-GHS-TBT_1x02-1MP_P1.25mm_Vertical.step"
GH6V = "JST_GH_BM06B-GHS-TBT_1x06-1MP_P1.25mm_Vertical.step"
GH13V = BoxEnvelope(19.5, 4.95, 4.2)
SOP4 = BoxEnvelope(4.4, 3.8, 2.1)
SOT23 = BoxEnvelope(2.9, 2.4, 1.1)
SOT363 = BoxEnvelope(2.1, 2.0, 1.0)
WROOM_1U = "envelope:wroom1u"
PLACEMENT = {
    "U1_esp32s3_wroom1u": Placement(WROOM_1U, 0, X0 + 27.0, Y0 + 22.0),
    "U2_cp2102n": Placement("QFN-24-1EP_4x4mm_P0.5mm_EP2.6x2.6mm.step", 0, X0 + 10.0, Y0 + 21.5),
    "U3_ldo": Placement("SOT-223.step", 0, X0 + 7.0, Y0 + 27.5),
    "U4_usb_switch": Placement("SOT-23-5.step", 0, X0 + 15.5, Y0 + 28.0),
    "J1_usb_c": Placement(
        "USB_C_Receptacle_GCT_USB4105-xx-A_16P_TopMnt_Horizontal.step",
        90,
        cl.PCBA_X1 - 3.2,
        Y0 + 22.0,
    ),
    "J2_gh4_ups_usb": Placement(GH4V, 0, X0 + 5.0, Y0 + 4.0),
    "J3_gh4_tx_uart": Placement(GH4V, 0, X0 + 22.0, Y0 + 4.0),
    "J4_gh2_ntc": Placement(GH2V, 0, X0 + 30.0, Y0 + 4.0),
    "J5_gh4_button": Placement(GH4V, 0, X0 + 5.5, Y0 + 35.0),
    "J6_gh6_ups_ctrl": Placement(GH6V, 0, X0 + 23.0, Y0 + 35.0),
    "J7_gh13_display": Placement(GH13V, 0, X0 + 27.0, Y0 + 42.5),
    "SW1_boot": Placement("SW_SPST_TL3342.step", 0, X0 + 6.5, Y0 + 13.0),
    "SW2_reset": Placement("SW_SPST_TL3342.step", 0, X0 + 13.0, Y0 + 13.0),
    "D1_status": Placement("LED_WS2812B_PLCC4_5.0x5.0mm_P3.2mm.step", 0, X0 + 40.5, Y0 + 12.5),
    "U5_photomos_j8": Placement(SOP4, 0, X0 + 20.0, Y0 + 30.0),
    "U6_opto_j10": Placement(SOP4, 0, X0 + 26.0, Y0 + 30.0),
    "Q1_led_fet": Placement(SOT23, 0, X0 + 32.0, Y0 + 30.0),
    "Q2_start_oneshot": Placement(SOT363, 0, X0 + 36.0, Y0 + 30.0),
}
BOTTOM = {"U5_photomos_j8", "U6_opto_j10", "Q1_led_fet", "Q2_start_oneshot"}


@cache
def _model(name: str) -> Shape:
    return import_step(cl.VENDOR / name)


def _wroom_1u() -> Shape:
    body = Box(18.0, 19.2, 3.2, align=C_MIN)
    ufl = Pos(6.0, 7.5, 3.2) * Box(3.0, 3.0, 1.25, align=C_MIN)
    return body + ufl


def parts() -> dict[str, Shape]:
    def model(name: str | BoxEnvelope) -> Shape:
        if isinstance(name, BoxEnvelope):
            return Box(*name, align=C_MIN)
        if name == WROOM_1U:
            return _wroom_1u()
        return _model(name)

    def at(ref: str, rot: float, x: float, y: float) -> Pos:
        if ref in BOTTOM:
            return Pos(x, y, cl.PCBA_Z) * Rot(180, 0, rot)
        return Pos(x, y, TOP) * Rot(0, 0, rot)

    return {ref: at(ref, rot, x, y) * model(name) for ref, (name, rot, x, y) in PLACEMENT.items()}


def parts_compound() -> Compound:
    return Compound(list(parts().values()))


def sections(p: Params) -> list[str]:
    return [f"y={Y0 + 24.0}", f"x={X0 + 27.5}"]


def mates(p: Params) -> dict:
    out = {}
    for ref, s in parts().items():
        bb = s.bounding_box()
        out[ref] = Pos(*bb.center()) * Box(bb.size.X, bb.size.Y, bb.size.Z)
    return out


def expect(p: Params) -> dict:
    return {
        "bbox": (cl.PCBA_X1 - cl.PCBA_X0, cl.PCBA_Y1 - cl.PCBA_Y0, cl.PCBA_T),
        "holes": {p.m3_close_clearance_dia: 4},
        "allow_interference": list(PLACEMENT),
    }


if __name__ == "__main__":
    bbs = {k: v.bounding_box() for k, v in parts().items()}
    ks = list(bbs)
    clash = [
        (a, b)
        for i, a in enumerate(ks)
        for b in ks[i + 1 :]
        if (a in BOTTOM) == (b in BOTTOM)
        and bbs[a].min.X < bbs[b].max.X
        and bbs[b].min.X < bbs[a].max.X
        and bbs[a].min.Y < bbs[b].max.Y
        and bbs[b].min.Y < bbs[a].max.Y
    ]
    clash += [
        (k, f"hole {h}")
        for k, bb in bbs.items()
        for h in holes()
        if bb.min.X - 3 < h[0] < bb.max.X + 3 and bb.min.Y - 3 < h[1] < bb.max.Y + 3
    ]
    print("\n".join(f"OVERLAP {a} / {b}" for a, b in clash) or "no overlaps")
