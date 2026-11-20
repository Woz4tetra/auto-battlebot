"""Add labels, art and USB-C shell paste to the routed Mk2 board."""

import json
import os
import sys

import pcbnew

IN = sys.argv[1]
OUT = sys.argv[2] if len(sys.argv) > 2 else IN
board = pcbnew.LoadBoard(IN)


def _clear_board_silk(board):
    for item in list(board.GetDrawings()):
        if item.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS):
            board.Delete(item)


_clear_board_silk(board)
MM = pcbnew.FromMM
VERSION = "v1.0"
SILK_TEXT_H_MM = 1.0
SILK_LINE_W_MM = 0.15
RX_LABEL_DY = -2.5
STAGGER_LABEL_DY = 2.6
ROBOT_BOTTOM_ART_XY = (40.9, -14.2)
ROBOT_TOP_ART_XY = (-40.7, -14.2)
LOGO_ART_XY = (-31.67, -8.25)


def kpt(x, y):
    return pcbnew.VECTOR2I(MM(100 + x), MM(100 - y))


def text(s, x, y, side="F", h=SILK_TEXT_H_MM, angle=0, bold=False):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(s)
    t.SetPosition(kpt(x, y))
    t.SetLayer(pcbnew.F_SilkS if side == "F" else pcbnew.B_SilkS)
    t.SetTextSize(pcbnew.VECTOR2I(MM(h), MM(h)))
    t.SetTextThickness(MM(0.2 if bold else SILK_LINE_W_MM))
    t.SetTextAngleDegrees(angle)
    t.SetMirrored(side == "B")
    t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_CENTER)
    t.SetVertJustify(pcbnew.GR_TEXT_V_ALIGN_CENTER)
    board.Add(t)


def shape(kind, layer, width=0.2, filled=False):
    s = pcbnew.PCB_SHAPE(board, kind)
    s.SetLayer(layer)
    s.SetWidth(MM(width))
    s.SetFilled(filled)
    board.Add(s)
    return s


def pads_by_value():
    """Wire pad / header value -> (x, y, side, pad half-height in y) in the board frame."""
    out = {}
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            p = pad.GetPosition()
            bb = pad.GetBoundingBox()
            out.setdefault(fp.GetValue(), []).append(
                (
                    pcbnew.ToMM(p.x) - 100,
                    100 - pcbnew.ToMM(p.y),
                    "B" if fp.IsFlipped() else "F",
                    pcbnew.ToMM(bb.GetHeight()) / 2,
                    pad.GetNumber(),
                )
            )
    return out


P = pads_by_value()


def behind(value, label, side=None, gap=0.85):
    """Label behind a front-edge wire pad; gap = half text height + text margin + mask clearance."""
    x, y, s, half, _ = P[value][0]
    text(label, x, y + half + gap, side or s)


def _label_roof_pads():
    for value, label in (
        ("PACK_A-", "A-"),
        ("PACK_A+", "A+"),
        ("PACK_B-", "B-"),
        ("PACK_B+", "B+"),
        ("SW_OUT", "SW"),
        ("SW_BACK", "SW"),
    ):
        behind(value, label)


def _label_wedge_pads():
    for value, label in (
        ("ESC_L-", "L-"),
        ("ESC_L+", "L+"),
        ("ESC_R+", "R+"),
        ("ESC_R-", "R-"),
    ):
        behind(value, label)
    for value, label in (
        ("DSHOT_L", "S"),
        ("SIG_GND_L", "G"),
        ("SIG_GND_R", "G"),
        ("DSHOT_R", "S"),
    ):
        behind(value, label, gap=0.9)


def _caption_esc_pairs():
    for side in "LR":
        sx = sum(P[v][0][0] for v in (f"DSHOT_{side}", f"SIG_GND_{side}")) / 2
        text(
            f"ESC {side}",
            sx,
            P[f"DSHOT_{side}"][0][1] + P[f"DSHOT_{side}"][0][3] + 2.0,
            "B",
            h=0.8,
            bold=True,
        )
    for sign, refs in (("+", ("ESC_L+", "ESC_R+")), ("-", ("ESC_L-", "ESC_R-"))):
        px = sum(P[v][0][0] for v in refs) / 2
        text(f"ESC {sign}", px, P[refs[0]][0][1] + P[refs[0]][0][3] + 2.0, "B", h=0.8, bold=True)


def _label_header_pins():
    for value, names, caption in (
        ("NANO_RX", {"1": "G", "2": "5V", "3": "C1", "4": "C2"}, "NANO RX"),
        ("BOOT/RST", {"1": "BT", "2": "G", "3": "RS"}, "BT-G BOOT  RS-G RESET"),
    ):
        pins = P[value]
        row_y = sum(p[1] for p in pins) / len(pins)
        for x, y, s, half, num in pins:
            if value == "NANO_RX":
                ly = row_y + RX_LABEL_DY
            else:
                ly = row_y - STAGGER_LABEL_DY if y > row_y else row_y + STAGGER_LABEL_DY
            text(names[num], x, ly, "B", h=0.8)
        text(caption, sum(p[0] for p in pins) / len(pins), row_y - 4.3, "B", h=0.8)


_label_roof_pads()
_label_wedge_pads()
_caption_esc_pairs()
_label_header_pins()


text("MR STABS MK2", -36.0, -13.6, "B", h=0.9, bold=True)
text(f"{VERSION} BEN W 2026", -36.0, -15.1, "B", h=0.8)

for fp in board.GetFootprints():
    if fp.GetValue() in ("NANO_RX", "BOOT/RST", "TYPE-C-31-D-06"):
        fp.Reference().SetVisible(False)

for fp in board.GetFootprints():
    fp.Reference().SetVisible(False)

ART = os.path.join(os.path.dirname(os.path.abspath(__file__)), "art")


def draw_json(name, cx, cy, side="B"):
    """Draw vectorized art polygons, mirrored on B."""
    sx = -1 if side == "B" else 1
    for rings in json.load(open(os.path.join(ART, name)))["polys"]:
        ps = pcbnew.SHAPE_POLY_SET()
        for i, ring in enumerate(rings):
            if i == 0:
                ps.NewOutline()
            else:
                ps.NewHole()
            for u, v in ring[:-1]:
                p = kpt(cx + sx * u, cy + v)
                hole_index = -1 if i == 0 else i - 1
                ps.Append(p.x, p.y, 0, hole_index)
        sh = shape(pcbnew.SHAPE_T_POLY, pcbnew.B_SilkS if side == "B" else pcbnew.F_SilkS, 0, True)
        sh.SetPolyShape(ps)


draw_json("robot_silk.json", *ROBOT_BOTTOM_ART_XY)
draw_json("robot_top_silk.json", *ROBOT_TOP_ART_XY, "F")
draw_json("logo_silk.json", *LOGO_ART_XY)

board.Save(OUT)
print(f"silk: labels and icon written to {OUT}")
