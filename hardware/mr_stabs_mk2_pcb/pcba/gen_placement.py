"""Write placement.json for the Mr Stabs Mk2 board from board-frame coordinates.

Board frame (../mr_stabs_mk2_pcb.py): origin at the centre of the 19.6 mm hole pattern on the roof
face, x right (toward the right ESC), y toward the rear wall, mm. KiCad F side is the roof face
(3.78 mm to the chassis roof), B side the wedge face. Parts are found by value and nets, not
refdes, so a regenerated netlist keeps its placement.

    SKILL/scripts/run gen_placement.py   (needs info.jsonl and cad/keepout.json)

Floorplan:
    centre, F       ESP32-S3-MINI-1U at (0, 0): the only spot a 15.4 mm square fits on the roof
                    face (0.47 mm to the front bosses and the rear cross member). USB and CRSF
                    pins face the front, I2C the right, 3V3 / IO0 / DShot the left.
    front centre, F BNO055 block between the front bosses, I2C pull-ups.
    ear front edges, F
                    12 AWG lap pads, wires laid flat into the wire bay ahead (40 mm clear at
                    |x| 14 to 26). Left: PACK_A-, PACK_A+, PACK_B-. Right: SW_BACK, SW_OUT,
                    PACK_B+. Each switch lead and pack A stay on one side; pack B splits.
    right ear, F    INA228 above the shunt, buck block and its three output caps outboard.
    left ear, F     LDO, EN RC, USB ESD / CC / VBUS diode.
    stem rear, F    NeoPixel behind the module, under the TPU roof so it shows through.
    right ear, B    shunt and its Kelvin resistors, ESC+ lap pads on its VBATT pour, 100 uF bulk.
    stem front, B   ESC- lap pads beside PACK_A-, then DShot and signal-ground pads, below the
                    washers.
    left ear, B     vertical USB-C, plugged with the wedge plate off; ESD and CC on F above it.
    stem rear, B    Nano RX 1x4 right-angle SMD header, pins forward; the RX stands on them.
    stem centre, B  BOOT/RST 1x3 SMD jumper header, in front of the RX's pin tips.
"""

import json
import sys

from legalize import Board, Layout
from shapely.geometry import shape

b = Board("info.jsonl")
find, pin_net, nets = b.find, b.pin_net, b.nets
info = b.parts
values = {d["ref"]: d["value"] for d in info}

MCU, IMU, INA = find("ESP32-S3-MINI-1U-N4R2"), find("BNO055"), find("INA228")
USB, LED = find("TYPE-C-31-M-06"), find("XL-2020RGBC-WS2812B")
holes_l = sorted(d["ref"] for d in info if d["value"] == "MountingHole_L")
holes_r = sorted(d["ref"] for d in info if d["value"] == "MountingHole_R")
holes = holes_l + holes_r  # (-9.8, -9.8), (-9.8, 9.8), (9.8, -9.8), (9.8, 9.8)
c100_3v3 = [d["ref"] for d in info if d["value"] == "100nF" and nets(d["ref"]) == {"+3V3", "GND"}]
assert len(c100_3v3) == 4, c100_3v3  # INA VS, MCU, IMU VDD, IMU VDDIO
r10k_imu = [
    d["ref"]
    for d in info
    if d["value"] == "10k" and "+3V3" in nets(d["ref"]) and not ({"EN", "IO0"} & nets(d["ref"]))
]
assert len(r10k_imu) == 2, r10k_imu


class SidedLayout(Layout):
    """Layout plus parts fixed on the B side (legalize.fix places on F only)."""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.fixed_b = {}

    def fix_bottom(self, ref, x, y, rot=0, through=False):
        self.fixed_b[ref] = (x, y, rot, through)

    def _b_shapes(self, side):
        return [
            self.rect(r, x, y, rot)
            for r, (x, y, rot, through) in self.fixed_b.items()
            if side == "B" or through
        ]

    def legal(self, shape_, placed, keepout_ok=False, side="F"):
        if any(shape_.buffer(self.gap).intersects(s) for s in self._b_shapes(side)):
            return False
        return super().legal(shape_, placed, keepout_ok, side)

    def solve(self, step=0.1):
        out = super().solve(step)
        for r, (x, y, rot, through) in self.fixed_b.items():
            s = self.rect(r, x, y, rot)
            if not self.region_fixed.contains(s):
                self.problems.append(f"fixed {r} leaves the board")
            ko = under_low if self.info[r]["value"] in ("0.5m", "10") else self.bottom_keepout
            if s.intersects(ko):
                self.problems.append(f"fixed {r} hits the B-side keep-out")
            out[r] = (x, y, rot, "B")
        return out


P = dict(x_half=12.7, y_max=14.7, y_min=-20.3, ear_y=-5.19, lip_x=35.2, tip_y=-11.01, tip_x=46.62)
P.update(wall_x=26.5, ear_front_y=-17.37)
right = [
    (P["x_half"], P["y_max"]),
    (P["x_half"], P["ear_y"]),
    (P["lip_x"], P["ear_y"]),
    (P["lip_x"], P["tip_y"]),
    (P["tip_x"], P["tip_y"]),
    (P["tip_x"], P["ear_front_y"]),
    (P["wall_x"], P["ear_front_y"]),
    (P["wall_x"], P["y_min"]),
]
outline_b = right + [(-x, y) for x, y in reversed(right)]  # ../mr_stabs_mk2_pcb.py outline()

keep = json.load(open("cad/keepout.json"))
roof = shape(keep["F"]).buffer(0.3)  # bosses and rear cross member on the roof face
under = shape(keep["B_tall"]).buffer(0.3)  # ESCs, ledge ridge, washers on the wedge face
under_low = shape(keep["B_low"]).buffer(0.3)  # what a part under 2.4 mm tall must clear

lay = SidedLayout("info.jsonl", outline_b, bottom_keepout=under, edge=0.25)
lay.keepout = roof
# EasyEDA courtyards that miss their own pads (footprint_audit.py).
lay.size(LED, (-1.25, 1.25), (-1.2, 1.2))
lay.size(USB, (-4.6, 4.6), (-2.95, 2.95))
# Feet to pin tips and the RX's 11 mm width. legalize.py does not mirror bottom-side courtyards
# and KiCad does, so this one is given mirrored (local y -12.35 to 1.85, not -1.85 to 12.35).
lay.size(find("NANO_RX"), (-5.8, 5.8), (-12.35, 1.85))
lay.size(find("BOOT/RST"), (-3.81, 3.81), (-3.4, 3.4))
# The screw head sits on the B side inside the printed washer (in the B keep-out); on the F side
# the hole only needs its own 3.7 mm plus 0.15 mm.
for h in holes:
    lay.size(h, (-1.85, 1.85), (-1.85, 1.85))
    lay.accept(
        h,
        MCU,
        "round 3.7 mm hole, box courtyard: the real hole edge is 1.05 mm from the module corner",
    )

# The module overhangs nothing but sits 0.47 mm from the roof keep-out; it is fixed, so it is
# checked against the board edge only, and the keep-out margin is checked here.
assert lay.rect(MCU, 0, 0, 0).distance(shape(keep["F"])) > 0.3

PAD_Y = -17.3  # lap pads: courtyard 0.25 mm inside the front edge at y -20.3; labels behind
fixed = {
    MCU: (0.0, 0.0, 0),
    holes[0]: (-9.8, -9.8, 0),
    holes[1]: (-9.8, 9.8, 0),
    holes[2]: (9.8, -9.8, 0),
    holes[3]: (9.8, 9.8, 0),
    find("PACK_A-"): (-14.9, PAD_Y, 0),
    find("PACK_A+"): (-19.5, PAD_Y, 0),
    find("PACK_B-"): (-24.1, PAD_Y, 0),
    find("SW_BACK"): (14.9, PAD_Y, 0),
    find("SW_OUT"): (19.5, PAD_Y, 0),
    find("PACK_B+"): (24.1, PAD_Y, 0),
    # Shunt along y right above SW_BACK: pad 1 BAT_IN toward the pad, pad 2 VBATT toward the rear.
    # VBUS OR-ing diode, pinned at 0 deg: at 90 deg its stock silk touches its own pad 1.
    find("B5819W", "VBUS_USB"): (-22.9, -10.4, 0),
}
THROUGH = {
    "MountingHole_L",
    "MountingHole_R",
    "SW_BACK",
    "PACK_A-",
    "ESC_L-",
    "ESC_R-",
}  # holes block both sides
for ref, (x, y, rot) in fixed.items():
    lay.fix(ref, x, y, rot, through=values[ref] in THROUGH)

# silk.py writes a label 1.4 mm deep just behind each lap pad; keep parts off those strips.
from shapely.geometry import box as _box

LABEL_STRIPS = [
    _box(x - 1.8, PAD_Y + 2.5, x + 1.8, PAD_Y + 3.9)
    for ref, (x, y, rot) in fixed.items()
    if values[ref] in ("PACK_A-", "PACK_A+", "PACK_B-", "PACK_B+", "SW_OUT", "SW_BACK")
]

# B side, fixed: USB-C on the right ear's rear strip; the headers on the left ear's.
B_FIXED = {
    USB: (-29.0, -9.0, 0),
    # Shunt on the wedge face: BAT_IN end toward SW_BACK, VBATT end toward the B-side ESC bus.
    # The 20 A crosses layers once, in the stitched BAT_IN pour between it and SW_BACK.
    find("0.5m"): (
        15.4,
        -9.95,
        90,
    ),  # courtyard clears the ear-edge ledge (B_low) and the D-cut washer
    # Kelvin resistors on the shunt's side, beside its pads, so each tap leaves from the pad
    # itself (Vishay's sensing-trace drawing); SENSE_P / SENSE_N then via up to the INA228.
    find("10", "SENSE_P"): (18.6, -12.4, 90),
    find("10", "SENSE_N"): (18.6, -10.2, 90),
    # 100 uF bulk on the right ear's wedge face (8 mm tall; 18-21 mm free there), + pad inboard
    # toward the VBATT pour.
    find("100uF 35V"): (30.6, -10.0, 90),  # pads along y: at 0 deg the GND pad crossed the ear edge
    # ESC power and DShot lap pads on the stem's front strip, below the washers (y < -14.3).
    # Leads run forward into the wire bay, then out to each ESC's inner end at |x| 14.
    # + outboard so the B-side VBATT pour reaches each one from the bus above the washers.
    # ESC+ on the shunt's VBATT pour (right ear), ESC- beside the pack's GND entry (left ear):
    # the 20 A never crosses the 0.5 oz inner planes (sim/plane_ir.py). Leads run from each ESC's
    # inner end at |x| 14 under the ears.
    find("ESC_L+"): (21.0, -11.8, 0),
    find("ESC_R+"): (24.6, -11.8, 0),
    # ESC- on the stem's front strip, 4 to 8 mm from PACK_A- (the GND entry): their via grids
    # carry the return to PACK_A-'s grid. The left ear's roof face is full of the LDO block.
    find("ESC_L-"): (-10.6, -17.1, 0),
    find("ESC_R-"): (-6.8, -17.1, 0),
    find("DSHOT_L"): (-3.8, -18.1, 0),
    find("SIG_GND_L"): (-1.6, -18.1, 0),
    find("SIG_GND_R"): (1.6, -18.1, 0),
    find("DSHOT_R"): (3.8, -18.1, 0),
    # Right-angle header, pins pointing forward: the RX stands on them at y 4.2-6.6, 18 mm deep
    # toward the wedge, where 22 mm is free (cad/, sectioned).
    # 180: KiCad mirrors bottom footprints (its pins would point rearward at 0); see lay.size.
    find("NANO_RX"): (0.0, 12.6, 180),
    find("BOOT/RST"): (0.0, -3.4, 0),  # clear of the RX header's pin tips at y 0.25
}
for ref, (x, y, rot) in B_FIXED.items():
    # Via-grid wire pads go through the board: they block the roof face above them too.
    lay.fix_bottom(ref, x, y, rot, through=values[ref] in THROUGH)
# Only their through-hole legs reach the F side: block those, not the whole body.
from shapely import affinity
from shapely.geometry import Point
from shapely.ops import unary_union

legs = []
for ref, (x, y, rot) in B_FIXED.items():
    d = lay.info[ref]
    for name, x0, x1, y0, y1 in d["pad_boxes"]:
        if name in ("25", ""):
            if ref == USB and name == "":
                continue  # locating pegs are non-plated holes under the body, not legs
            c = Point((x0 + x1) / 2, -(y0 + y1) / 2)  # footprint-local y down -> y up
            c = affinity.translate(affinity.rotate(c, rot, origin=(0, 0)), x, y)
            legs.append(c.buffer(max(x1 - x0, y1 - y0) / 2 + 0.3))
lay.keepout = unary_union([lay.keepout, *legs, *LABEL_STRIPS])

wanted = {
    # INA228 on the shunt's Kelvin lines, VBUS pin toward VBATT.
    # The sense taps see a little pour and via resistance besides the shunt: a fixed gain error,
    # calibrated out with a known load.
    # 270: pins 6-10 (GND, VBATT, SENSE) face the shunt; at 90 they faced the ear's rear edge
    # and pin 7 had no room for its GND via.
    INA: (15.2, -8.6, 270),
    find("100nF", "SENSE_P", "SENSE_N"): (18.4, -12.2, 90),  # off the BAT_IN pour
    c100_3v3[0]: (15.2, -6.1, 0),
    # Buck on the right ear outboard: VIN from VBATT near ESC_R+, SW into the inductor.
    find("TPS54202DDC"): (30.0, -14.2, 0),
    find("10uF", "VBATT", nth=0): (27.0, -10.0, 90),
    find("10uF", "VBATT", nth=1): (25.0, -10.0, 90),
    find("100nF", "VBATT"): (28.4, -16.4, 0),
    find("100nF", "SW"): (32.5, -15.8, 0),
    find("100k", "V5_BUCK"): (33.0, -12.6, 0),
    find("12k", "FB"): (33.0, -13.8, 0),
    find("15uH"): (40.0, -14.0, 0),
    find("47pF"): (33.0, -11.4, 0),  # feed-forward across R_top
    find("22uF", nth=0): (44.0, -14.6, 90),
    find("22uF", nth=1): (37.5, -11.8, 0),
    find("22uF", nth=2): (44.0, -12.4, 0),  # third output cap, sim/buck_loop.py
    find("B5819W", "V5_BUCK"): (31.0, -7.0, 0),
    find("10uF", "+5V"): (28.5, -10.0, 90),
    # USB ESD, CC resistors and the VBUS OR-ing diode over the connector, which sits under the
    # left ear's rear strip on B.
    find("USBLC6-2SC6"): (-29.0, -13.0, 0),
    find("5.1k", pin_net(USB, "A5")): (-33.0, -12.0, 0),
    find("5.1k", pin_net(USB, "B5")): (-33.0, -13.4, 0),
    # LDO block and EN RC on the left ear, by the module's 3V3 / EN side.
    find("AP2112K-3.3"): (-17.0, -9.0, 0),
    find("1uF", "+5V"): (-19.6, -9.0, 90),
    find("10uF", "+3V3", nth=0): (-14.6, -9.0, 90),
    find("10uF", "+3V3", nth=1): (-8.6, 2.0, 90),
    c100_3v3[1]: (-8.6, -1.6, 90),
    find("10k", "EN"): (-22.5, -7.0, 0),
    find("1uF", "EN"): (-22.5, -8.4, 0),
    find("10k", "IO0"): (-25.5, -7.0, 0),
    # NeoPixel behind the module on the roof face, facing the TPU roof so it shows through.
    LED: (0.0, 13.0, 0),
    find("100", pin_net(LED, "3")): (3.2, 13.0, 90),
    find("100nF", "+5V"): (-3.2, 13.0, 90),  # LED VDD
    # DShot and CRSF series resistors by their pads and header.
    find("100", "DSHOT_L"): (-8.6, -6.0, 90),
    find("100", "DSHOT_R"): (8.6, -6.0, 90),
    find("100", pin_net(find("NANO_RX"), "3")): (9.0, -6.0, 90),
    find("100", pin_net(find("NANO_RX"), "4")): (-9.0, 6.0, 90),
    # IMU between the front bosses, rotated so every LGA side has room to escape.
    IMU: (0.0, -13.0, 90),
    find("32.768kHz"): (-4.0, -13.0, 90),
    find("22pF", "XIN32"): (-5.4, -15.6, 0),
    find("22pF", "XOUT32"): (-5.4, -10.4, 0),
    c100_3v3[2]: (3.0, -10.4, 0),
    c100_3v3[3]: (3.0, -15.6, 0),
    r10k_imu[0]: (4.6, -12.0, 90),
    r10k_imu[1]: (4.6, -14.6, 90),
    find("4.7k", "SDA1"): (5.0, -18.0, 0),
    find("4.7k", "SCL1"): (2.6, -18.0, 0),
}
# BNO055 CAP pin cap.
cap_ref = [find("1uF", "IMU_CAP")]
wanted[cap_ref[0]] = (-2.0, -17.6, 0)

REACH = {find("22uF", nth=2): 12.0}  # the ear tip is full; take the nearest free spot
for ref, (x, y, rot) in wanted.items():
    lay.want(ref, x, y, rot, reach=REACH.get(ref, 6.0), bottom_ok=False)
solved = lay.solve()
if "--plot" in sys.argv:
    import debug_plot

    debug_plot.plot(lay, solved, roof, under, outline_b)
k = lay.to_kicad

spec = {
    "inherit": "jlcpcb_2layer",  # JLCPCB's 2-layer minimums are also safe on its 4-layer process
    # Both inner layers are GND (the default fanout covers them). stitch_pitch 1.0 packs the
    # BAT_IN and VBATT pours' F/B overlaps with vias.
    "flow": {
        "tries": 10,
        "fr_passes": 50,
        "fanout_nets": [],
        "stitch_nets": ["BAT_IN", "VBATT"],
        "stitch_pitch": 1.0,
    },
    # 4 layers: F signal, In1 and In2 solid GND (the ESC return), B signal. VBATT lives only on
    # the right ear's B pour, shunt to ESC+ pads (sim/copper_ir.py).
    "order": {"assembled": 2},  # 5 PCBs, 2 assembled (intake)
    "layers": 4,
    "plane_layers": ["In1.Cu", "In2.Cu"],
    # Worst-case net voltages for rating_check.py: 4S LiHV is 17.4 V full; SW swings to VIN;
    # BOOT rides about 5.6 V above SW.
    "net_voltage": {
        "PACK_MID": [0, 8.7],
        "PACK+": [0, 17.4],
        "BAT_IN": [0, 17.4],
        "VBATT": [0, 17.4],
        "SENSE_P": [0, 17.4],
        "SENSE_N": [0, 17.4],
        "SW": [0, 17.4],
        "BUCK_BOOT": [0, 23.0],
        "V5_BUCK": [0, 5.6],
        "+5V": [0, 5.6],
        "VBUS_USB": [0, 5.5],
        "+3V3": [0, 3.6],
    },
    "differential": [["SENSE_P", "SENSE_N", 0.1], ["BUCK_BOOT", "SW", 5.6]],
    "board": {"outline": [k(x, y) for x, y in outline_b]},
    "netclasses": {
        # Tracks only feed pins; the zones carry the current.
        "Battery": {"track_width": 0.3, "nets": ["BAT_IN", "VBATT", "PACK+", "PACK_MID"]},
        "Power": {"track_width": 0.3, "nets": ["+5V", "V5_BUCK", "VBUS_USB", "SW"]},
        # 0.2 mm: the BNO055's LGA pads are 0.25 mm wide at 0.5 mm pitch. Under 0.4 A.
        "Rail": {"track_width": 0.2, "nets": ["+3V3"]},
        "USB": {"track_width": 0.2, "clearance": 0.15, "nets": ["USB_DP", "USB_DM"]},
    },
    "accepted": [],
    "parts": lay.kicad_parts(solved),
    "courtyard_overrides": lay.overrides,
    "zones": [
        {"net": "GND", "layers": ["F.Cu", "B.Cu"]},
        {"net": "GND", "layers": ["In1.Cu"]},
        {"net": "GND", "layers": ["In2.Cu"]},
        # Pack series join: PACK_A+ to PACK_B-, both on the left ear's front edge.
        {
            "net": "PACK_MID",
            "layers": ["F.Cu", "B.Cu"],
            "priority": 2,
            "outline": [
                k(x, y) for x, y in [(-26.2, -20.0), (-17.3, -20.0), (-17.3, -13.4), (-26.2, -13.4)]
            ],
        },
        # Switch loop out: PACK_B+ to SW_OUT, the full pad height.
        {
            "net": "PACK+",
            "layers": ["F.Cu"],
            "priority": 2,
            "outline": [
                k(x, y) for x, y in [(17.3, -20.0), (26.2, -20.0), (26.2, -14.6), (17.3, -14.6)]
            ],
        },
        # Switch loop back: the F pour under SW_BACK, its vias, the B pour under the shunt.
        {
            "net": "BAT_IN",
            "layers": ["F.Cu"],
            "priority": 3,
            # Reaches up toward the shunt so the stitching vias in its overlap with the B pour
            # share the layer change with SW_BACK's grid (sim/copper_ir.py: its top row crowded).
            "outline": [
                k(x, y) for x, y in [(12.9, -20.0), (17.0, -20.0), (17.0, -11.4), (12.9, -11.4)]
            ],
        },
        {
            "net": "BAT_IN",
            "layers": ["B.Cu"],
            "priority": 3,
            "outline": [
                k(x, y) for x, y in [(13.0, -20.0), (17.4, -20.0), (17.4, -10.3), (13.0, -10.3)]
            ],
        },
        # Shunt VBATT end and its via field, on B.
        {
            "net": "VBATT",
            "layers": ["B.Cu"],
            "priority": 3,
            "outline": [
                k(x, y)
                for x, y in [
                    (12.9, -5.4),
                    (24.0, -5.4),
                    (24.0, -6.0),
                    (33.0, -6.0),
                    (33.0, -14.6),
                    (17.4, -14.6),
                    (17.4, -9.6),
                    (12.9, -9.6),
                ]
            ],
        },
    ],
}
json.dump(spec, open("placement.json", "w"), indent=1)
missing = sorted(set(d["ref"] for d in info) - set(solved))
print(f"{len(solved)} of {len(info)} placed; unplaced: {missing}")
sys.exit(1 if lay.problems else 0)
