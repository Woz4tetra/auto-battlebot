"""Custom PCBA for the cage pack: an ESP32-S3 board with the USB bridge, the power button's
start/stop logic and GH leads, in the printed enclosure.

Spec
- Process: 1.6 mm FR4, 4 layers, JLCPCB. PROCESS = "none".
- Envelope: 44 x 46. Grown 8 mm toward +Y (from 44 x 38) for J7; the +Y edge stops 2.2
  short of the ZED Box Mini's body.
- Frame: cage_layout.py. On four 8 mm standoffs, at PCBA_X0..X1, PCBA_Y0..Y1.
- Holes: 4 x Ø3.2 on 8 mm printed bosses with M3 inserts in the enclosure panel. The -X
  pair sits 13.5 in from the edge, which leaves the -Y edge clear for the GH connectors.
- U1 is an ESP32-S3-WROOM-1U (u.FL) so the antenna sits on the enclosure wall, clear of
  the ZED Box Mini and the battery.
- USB-C (J1) on the +X edge, facing +X; its 90° plug turns the cable toward +Y and out of
  the pod's top wall notch to the ZED Box Mini's USB-A.
- The three JST GH sockets (OpenUPS USB, transmitter UART, pack NTC) sit along the -Y
  edge, toward the OpenUPS and the -X panel connectors, either side of the -X hole.
- Touch display on J7, a vertical JST GH 13-pin (BM13B-GHS-TBT), centred on the +Y edge
  between the two +Y holes. Pinout follows the Waveshare 2.8inch Capacitive Touch LCD's
  13-pin header so a straight GH-to-GH lead works: VCC (3.3 V), GND, MISO (no connect),
  MOSI, SCLK, LCD_CS, LCD_DC, LCD_RST, LCD_BL (PWM), TP_SDA, TP_SCL, TP_INT, TP_RST.
  10 ESP32-S3 GPIOs; MOSI and SCLK on SPI2's IO_MUX pins (IO11, IO12) so the clock is not
  limited by the GPIO matrix (40 MHz there). The backlight's current is not published:
  measure it before sizing U3. Firmware keeps the display off the command path: commands
  on their own high-priority task, the display on the other core at low priority with DMA
  transfers.
- Power button (CIT AH1LBBG12, latching, SPST-NO + SPST-NC, 12 V LED ring) on J5: GND (both
  contacts' commons and LED -), NO, NC, LED +. The button carries signal current only; the
  battery stays wired to the OpenUPS through the 10 A fuse.
- OpenUPS control on J6: J8 (start/stop) pair, J10 (motherboard on/off) pair, VBAT through
  a polyfuse, GND. OpenUPS set to OPENUPSMODE = 1, so input power charges but never starts
  the output on its own.
  - Button pressed in (NO closes): a battery-powered RC one-shot (Q2, dual N-FET) drives
    photoMOS U5 across J8 for ~0.5 s, which starts the output. It runs with the ESP32 off.
  - Button released (NC closes): the ESP32 sees NC, tells the Jetson over USB to shut down,
    then holds U5 for 6 s (a long J8 press) once the Jetson confirms or a timeout passes.
    The OpenUPS cuts its output UPS_HARDOFF_TOUT later.
  - The Jetson can ask for the same long press over USB to power the pack off itself.
  - J10's off pulse (low battery or a fault) reaches the ESP32 through optocoupler U6,
    which tells the Jetson to shut down before the OpenUPS cuts power.
  - Q1 switches the button's LED ring from VBAT through a series resistor, so the ESP32
    can show running, shutting down and fault states.
  U5, U6, Q1 and Q2 sit on the bottom side; the board stands 8 mm off the panel.
- Model: features() (recipe.py) for the bare board, so the Onshape Part Studio matches it;
  the components are placed separately (parts()).
- No part sits over a mounting hole's 3 mm radius: `python pcba_cage.py` lists any part
  pair on the same side, or part-hole overlap, in plan view.
"""

from dataclasses import dataclass
from functools import cache

import b123d_fix  # noqa: F401
import cage_layout as cl
import recipe  # build123d-part/scripts, on the path when run through scripts/run
from build123d import Align, Box, Compound, Part, Pos, Rot, Shape, import_step

PROCESS = "none"
MATERIAL = "nylon"  # density stand-in for FR4
C_MIN = (Align.CENTER, Align.CENTER, Align.MIN)


@dataclass
class Params:
    hole: float = 3.2  # M3 close clearance, fasteners.md


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
            {"rect": (xc, yc, w, h)}, *[{"circle": (x, y, p.hole)} for x, y in holes()]]},
        {"op": "extrude", "name": "Board", "sketch": "Board outline", "regions": [(xc, yc)],
         "depth": cl.PCBA_T, "start": cl.PCBA_Z},
    ]  # fmt: skip


def build(p: Params) -> Part:
    return recipe.build(features(p))


TOP = cl.PCBA_Z + cl.PCBA_T
X0, Y0 = cl.PCBA_X0, cl.PCBA_Y0
# (model, rotation about Z, x, y): KiCad frames, mating face toward -Y, top +Z.
GH4V = "JST_GH_BM04B-GHS-TBT_1x04-1MP_P1.25mm_Vertical.step"
GH2V = "JST_GH_BM02B-GHS-TBT_1x02-1MP_P1.25mm_Vertical.step"
GH6V = "JST_GH_BM06B-GHS-TBT_1x06-1MP_P1.25mm_Vertical.step"
# KiCad's 9.0 library has no BM13B model here: the outline from the BM02-06 models' rule,
# 1.25 n + 3.25 wide, 4.95 deep, 4.2 tall.
GH13V = "box:19.5x4.95x4.2"
# Datasheet outlines where KiCad has no model: "box:W x D x H".
SOP4 = "box:4.4x3.8x2.1"  # Toshiba TLP172A photoMOS / TLP293 optocoupler, SO4
SOT23 = "box:2.9x2.4x1.1"
SOT363 = "box:2.1x2.0x1.0"  # 2N7002DW dual N-FET
# ESP32-S3-WROOM-1U: KiCad 9.0 has no model, so this is the datasheet outline (18.0 x 19.2 x
# 3.2, shield can) with the u.FL receptacle near the +Y end. Its pigtail runs to an RP-SMA
# bulkhead in the enclosure's antenna wall.
WROOM_1U = "envelope:wroom1u"
PLACEMENT = {
    "U1_esp32s3_wroom1u": (WROOM_1U, 0, X0 + 27.0, Y0 + 22.0),
    "U2_cp2102n": ("QFN-24-1EP_4x4mm_P0.5mm_EP2.6x2.6mm.step", 0, X0 + 10.0, Y0 + 21.5),
    "U3_ldo": ("SOT-223.step", 0, X0 + 7.0, Y0 + 27.5),
    "U4_usb_switch": ("SOT-23-5.step", 0, X0 + 15.5, Y0 + 28.0),
    "J1_usb_c": (
        "USB_C_Receptacle_GCT_USB4105-xx-A_16P_TopMnt_Horizontal.step",
        90,
        cl.PCBA_X1 - 3.2,
        Y0 + 22.0,
    ),
    "J2_gh4_ups_usb": (GH4V, 0, X0 + 5.0, Y0 + 4.0),
    "J3_gh4_tx_uart": (GH4V, 0, X0 + 22.0, Y0 + 4.0),
    "J4_gh2_ntc": (GH2V, 0, X0 + 30.0, Y0 + 4.0),
    "J5_gh4_button": (GH4V, 0, X0 + 5.5, Y0 + 35.0),
    "J6_gh6_ups_ctrl": (GH6V, 0, X0 + 23.0, Y0 + 35.0),
    "J7_gh13_display": (GH13V, 0, X0 + 27.0, Y0 + 42.5),
    "SW1_boot": ("SW_SPST_TL3342.step", 0, X0 + 6.5, Y0 + 13.0),
    "SW2_reset": ("SW_SPST_TL3342.step", 0, X0 + 13.0, Y0 + 13.0),
    "D1_status": ("LED_WS2812B_PLCC4_5.0x5.0mm_P3.2mm.step", 0, X0 + 40.5, Y0 + 12.5),
    "U5_photomos_j8": (SOP4, 0, X0 + 20.0, Y0 + 30.0),
    "U6_opto_j10": (SOP4, 0, X0 + 26.0, Y0 + 30.0),
    "Q1_led_fet": (SOT23, 0, X0 + 32.0, Y0 + 30.0),
    "Q2_start_oneshot": (SOT363, 0, X0 + 36.0, Y0 + 30.0),
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
    def model(name: str) -> Shape:
        if name == WROOM_1U:
            return _wroom_1u()
        if name.startswith("box:"):
            w, d, h = (float(v) for v in name[4:].split("x"))
            return Box(w, d, h, align=C_MIN)
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
        "holes": {p.hole: 4},
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
