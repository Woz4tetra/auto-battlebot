"""Finish the routed board: wire-pad and pin labels, and the Mk2 and BWBots art.

    SKILL/scripts/kicad python3 silk.py mk2_routed.kicad_pcb        (in place; or IN OUT)

Run it on the routed board after flow.sh and before check.sh / fab.sh: flow.sh rebuilds the
board from the netlist on every try, so anything added before routing is lost. Silk does not
touch copper, so the route is unchanged. Labels are placed from the pads' real positions, found
by the wire pads' values (the net labels in design.py), so they follow placement changes.

Board frame (gen_placement.py): x right, y toward the rear wall, mm; KiCad X = 100 + x,
Y = 100 - y. B-side items are drawn as seen from below (x mirrored) so they read correctly
when the board is turned over.
"""

import json
import os
import sys

import pcbnew

IN = sys.argv[1]
OUT = sys.argv[2] if len(sys.argv) > 2 else IN
board = pcbnew.LoadBoard(IN)
# Rerunnable: every board-level silk item is ours (footprint silk lives in the footprints).
for item in list(board.GetDrawings()):
    if item.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS):
        board.Delete(item)
MM = pcbnew.FromMM
TEXT_H = 1.0  # JLCPCB minimum is 0.8 mm
LINE_W = 0.15  # JLCPCB minimum silk line width


def kpt(x, y):
    return pcbnew.VECTOR2I(MM(100 + x), MM(100 - y))


def text(s, x, y, side="F", h=TEXT_H, angle=0, bold=False):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(s)
    t.SetPosition(kpt(x, y))
    t.SetLayer(pcbnew.F_SilkS if side == "F" else pcbnew.B_SilkS)
    t.SetTextSize(pcbnew.VECTOR2I(MM(h), MM(h)))
    t.SetTextThickness(MM(0.2 if bold else LINE_W))
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
    """Label just behind a front-edge wire pad (toward the board's interior). gap is pad edge to
    text centre: half the text height plus the text box's margin plus solder-mask clearance."""
    x, y, s, half, _ = P[value][0]
    text(label, x, y + half + gap, side or s)


# --- Roof face (F): the six 12 AWG XT60 pigtail pads along the front edge.
for value, label in (
    ("PACK_A-", "A-"),
    ("PACK_A+", "A+"),
    ("PACK_B-", "B-"),
    ("PACK_B+", "B+"),
    ("SW_OUT", "SW"),
    ("SW_BACK", "SW"),
):
    behind(value, label)

# --- Wedge face (B): ESC leads, signal pads, connectors.
for value, label in (
    ("ESC_L-", "L-"),
    ("ESC_L+", "L+"),
    ("ESC_R+", "R+"),
    ("ESC_R-", "R-"),
):
    behind(value, label)
for value, label in (("DSHOT_L", "S"), ("SIG_GND_L", "G"), ("SIG_GND_R", "G"), ("DSHOT_R", "S")):
    behind(value, label, gap=0.9)
# Captions: the signal pairs on the stem, the power pairs on each ear.
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

# Pin labels for the two headers: one label per pin, beside the row on the side away from the
# pin's staggered pad.
for value, names, caption in (
    ("NANO_RX", {"1": "G", "2": "5V", "3": "C1", "4": "C2"}, "NANO RX"),
    ("BOOT/RST", {"1": "BT", "2": "G", "3": "RS"}, "BT-G BOOT  RS-G RESET"),
):
    pins = P[value]
    row_y = sum(p[1] for p in pins) / len(pins)
    for x, y, s, half, num in pins:
        # Staggered pads sit 1.6 mm off the row: label on the row's other side. The right-angle
        # RX header's pads are one row along the rear edge: label between them and its body.
        if value == "NANO_RX":
            ly = row_y - 2.5
        else:
            ly = row_y - 2.6 if y > row_y else row_y + 2.6
        text(names[num], x, ly, "B", h=0.8)
    text(caption, sum(p[0] for p in pins) / len(pins), row_y - 4.3, "B", h=0.8)

usb = P["TYPE-C-31-M-06"]
text("USB", sum(p[0] for p in usb) / len(usb), max(p[1] for p in usb) + 2.6, "B", bold=True)

# Legend for the roof-face pads, on the left ear's bare wedge-face strip (over the ESC).
for i, line in enumerate(("XT60 PADS, TOP:", "A-/A+ PACK A", "B-/B+ PACK B", "SW/SW SWITCH")):
    text(line, -36.0, -12.9 - 1.15 * i, "B", h=0.8)

# Refdes on the B-side connectors sit on top of the labels above; JLCPCB places from the CPL.
for fp in board.GetFootprints():
    if fp.GetValue() in ("NANO_RX", "BOOT/RST", "TYPE-C-31-M-06"):
        fp.Reference().SetVisible(False)

# Hide every refdes: on a board this dense they collide with parts and labels, and JLCPCB
# assembles from the CPL. The labels above say what each connection is.
for fp in board.GetFootprints():
    fp.Reference().SetVisible(False)
    # The EasyEDA LED footprint's polarity circle sits on its own pad 1; the chamfer marks it too.
    if fp.GetValue() == "XL-2020RGBC-WS2812B":
        for item in list(fp.GraphicalItems()):
            if item.GetLayer() == pcbnew.F_SilkS and item.GetShape() == pcbnew.SHAPE_T_CIRCLE:
                fp.Remove(item)

# --- Art: the Mk2 CAD as line art (art/render_robot.py), its underside on the wedge face and its
# top on the roof face, and the BWBots logo from the dashboard (logo/gen_logo_icon.py), all
# vectorized by art/vectorize.py.
ART = os.path.join(os.path.dirname(os.path.abspath(__file__)), "art")


def draw_json(name, cx, cy, side="B"):
    """Polygons in (u, v) mm as the image reads; B silk is mirrored in x because it is read from
    below."""
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
                # Append(x, y, outline, hole): hole -1 is the outline itself, else a hole index.
                ps.Append(p.x, p.y, 0, -1 if i == 0 else i - 1)
        sh = shape(pcbnew.SHAPE_T_POLY, pcbnew.B_SilkS if side == "B" else pcbnew.F_SilkS, 0, True)
        sh.SetPolyShape(ps)


draw_json("robot_silk.json", 40.9, -14.2)  # seen from below, on the bottom (right ear tip)
draw_json("robot_top_silk.json", -40.7, -14.2, "F")  # seen from above, on the top
draw_json("logo_silk.json", -5.8, 3.2)  # stem, between the headers' captions, clear of the washer
text("MR STABS MK2", 4.6, 3.2, "B", h=0.9, bold=True)  # beside the logo

board.Save(OUT)
print(f"silk: labels and icon written to {OUT}")
