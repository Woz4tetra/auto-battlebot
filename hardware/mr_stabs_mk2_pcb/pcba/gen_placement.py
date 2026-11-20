"""Write placement.json for the Mk2 board."""

import json
import sys

from legalize import Board, Layout
from shapely import affinity
from shapely.geometry import shape

b = Board("info.jsonl")
find, pin_net, nets = b.find, b.pin_net, b.nets
info = b.parts
values = {d["ref"]: d["value"] for d in info}

MCU, IMU, INA = find("ESP32-S3-MINI-1U-N4R2"), find("BNO055"), find("INA238")
USB, LED = find("TYPE-C-31-D-06"), find("XL-2020RGBC-WS2812B")
holes_l = sorted(d["ref"] for d in info if d["value"] == "MountingHole_L")
holes_r = sorted(d["ref"] for d in info if d["value"] == "MountingHole_R")
holes = holes_l + holes_r
HOLE_XY = [(-9.8, -9.8), (-9.8, 9.8), (9.8, -9.8), (9.8, 9.8)]
c100_3v3 = [d["ref"] for d in info if d["value"] == "100nF" and nets(d["ref"]) == {"+3V3", "GND"}]
assert len(c100_3v3) == 4, c100_3v3
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

    def rect(self, ref, x, y, rot):
        shape = super().rect(ref, x, y, rot)
        if ref not in self.fixed_b:
            return shape
        return affinity.scale(shape, yfact=-1, origin=(x, y))

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
            ko = (
                under_low_keepout
                if self.info[r]["value"] in ("0.5m", "10")
                else self.bottom_keepout
            )
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
outline_b = right + [(-x, y) for x, y in reversed(right)]

keep = json.load(open("cad/keepout.json"))
roof_keepout = shape(keep["F"]).buffer(0.3)
under_tall_keepout = shape(keep["B_tall"]).buffer(0.3)
under_low_keepout = shape(keep["B_low"]).buffer(0.3)

lay = SidedLayout("info.jsonl", outline_b, bottom_keepout=under_tall_keepout, edge=0.25)
lay.keepout = roof_keepout
lay.size(find("NANO_RX"), (-5.8, 5.8), (-1.85, 12.35))
lay.size(find("BOOT/RST"), (-3.81, 3.81), (-3.4, 3.4))
MCU_XY = (0.0, 0.0)
assert lay.rect(MCU, *MCU_XY, 0).distance(shape(keep["F"])) > 0.3

LAP_PAD_Y = -17.3
fixed = {
    MCU: (*MCU_XY, 0),
    **{h: (x, y, 0) for h, (x, y) in zip(holes, HOLE_XY)},
    find("GND_STITCH", nth=0): (11.6, -2.6, 0),
    find("GND_STITCH", nth=1): (-11.6, -2.6, 0),
    find("PACK_A-"): (-14.9, LAP_PAD_Y, 0),
    find("PACK_A+"): (-19.5, LAP_PAD_Y, 0),
    find("PACK_B-"): (-24.1, LAP_PAD_Y, 0),
    find("SW_BACK"): (14.9, LAP_PAD_Y, 0),
    find("SW_OUT"): (19.5, LAP_PAD_Y, 0),
    find("PACK_B+"): (24.1, LAP_PAD_Y, 0),
}
THROUGH_BOARD_VALUES = {
    "GND_STITCH",
    "MountingHole_L",
    "MountingHole_R",
    "SW_BACK",
    "PACK_A-",
    "ESC_L-",
    "ESC_R-",
}
for ref, (x, y, rot) in fixed.items():
    lay.fix(ref, x, y, rot, through=values[ref] in THROUGH_BOARD_VALUES)

from shapely.geometry import box as _box

LAP_PAD_LABEL_STRIPS = [
    _box(x - 1.8, LAP_PAD_Y + 2.5, x + 1.8, LAP_PAD_Y + 3.9)
    for ref, (x, y, rot) in fixed.items()
    if values[ref] in ("PACK_A-", "PACK_A+", "PACK_B-", "PACK_B+", "SW_OUT", "SW_BACK")
]

B_FIXED = {
    USB: (0.0, -11.1, 0),
    find("0.5m"): (
        15.4,
        -9.95,
        90,
    ),
    find("10", "SENSE_P"): (18.6, -12.4, 90),
    find("10", "SENSE_N"): (18.6, -10.2, 90),
    find("100uF 35V"): (30.6, -10.0, 90),
    find("ESC_L+"): (21.0, -11.8, 0),
    find("ESC_R+"): (24.6, -11.8, 0),
    find("ESC_L-"): (-10.6, -17.1, 0),
    find("ESC_R-"): (-6.8, -17.1, 0),
    find("DSHOT_L"): (-3.8, -18.1, 0),
    find("SIG_GND_L"): (-1.6, -18.1, 0),
    find("SIG_GND_R"): (1.6, -18.1, 0),
    find("DSHOT_R"): (3.8, -18.1, 0),
    find("NANO_RX"): (0.0, 12.6, 180),
    find("BOOT/RST"): (-21.0, -9.0, 0),
}
for ref, (x, y, rot) in B_FIXED.items():
    lay.fix_bottom(ref, x, y, rot, through=values[ref] in THROUGH_BOARD_VALUES)
from shapely.geometry import Point
from shapely.ops import unary_union

PIERCING_PAD_NAMES = ("25", "EP", "")


def _pad_center_board_frame(x0, x1, y0, y1, rot, x, y):
    c = Point((x0 + x1) / 2, -(y0 + y1) / 2)
    return affinity.translate(affinity.rotate(c, rot, origin=(0, 0)), x, y)


def _through_leg_keepouts(b_fixed):
    legs = []
    for ref, (x, y, rot) in b_fixed.items():
        d = lay.info[ref]
        for name, x0, x1, y0, y1 in d["pad_boxes"]:
            if name in PIERCING_PAD_NAMES:
                c = _pad_center_board_frame(x0, x1, y0, y1, rot, x, y)
                legs.append(c.buffer(max(x1 - x0, y1 - y0) / 2 + 0.3))
    return legs


lay.keepout = unary_union([lay.keepout, *_through_leg_keepouts(B_FIXED), *LAP_PAD_LABEL_STRIPS])

wanted = {
    INA: (15.4, -9.1, 270),
    find("100nF", "SENSE_P", "SENSE_N"): (18.4, -12.2, 90),
    c100_3v3[0]: (15.2, -6.1, 0),
    find("TPS54202DDC"): (30.0, -14.2, 0),
    find("10uF", "VBATT", nth=0): (27.0, -10.0, 90),
    find("10uF", "VBATT", nth=1): (25.0, -10.0, 90),
    find("100nF", "VBATT"): (27.23, -16.15, 270),
    find("100nF", "SW"): (32.5, -15.8, 0),
    find("100k", "V5_BUCK"): (33.0, -12.6, 0),
    find("12k", "FB"): (33.0, -13.8, 0),
    find("15uH"): (40.0, -14.0, 0),
    find("47pF"): (33.0, -11.4, 0),
    find("22uF", nth=0): (44.0, -14.6, 90),
    find("22uF", nth=1): (37.5, -11.8, 0),
    find("22uF", nth=2): (44.0, -12.4, 0),
    find("B5819W", "V5_BUCK"): (31.0, -7.0, 0),
    find("10uF", "+5V"): (28.5, -10.0, 90),
    find("USBLC6-2SC6"): (-6.4, -10.8, 90),
    find("5.1k", pin_net(USB, "A5")): (0.0, -10.6, 0),
    find("5.1k", pin_net(USB, "B5")): (0.0, -12.0, 0),
    find("AP2112K-3.3"): (-17.0, -9.0, 0),
    find("1uF", "+5V"): (-19.6, -9.0, 90),
    find("10uF", "+3V3", nth=0): (-14.6, -9.0, 90),
    find("10uF", "+3V3", nth=1): (-8.6, 2.0, 90),
    c100_3v3[1]: (-8.6, -1.6, 90),
    find("10k", "EN"): (-22.5, -7.0, 0),
    find("1uF", "EN"): (-22.5, -8.4, 0),
    find("10k", "IO0"): (-25.5, -7.0, 0),
    LED: (0.0, 13.0, 0),
    find("100", pin_net(LED, "3")): (3.2, 13.0, 90),
    find("100nF", "+5V"): (-3.2, 13.0, 90),
    find("100", "DSHOT_L"): (-8.6, -6.0, 90),
    find("100", "DSHOT_R"): (8.6, -6.0, 90),
    find("100", pin_net(find("NANO_RX"), "3")): (9.0, -6.0, 90),
    find("100", pin_net(find("NANO_RX"), "4")): (-9.0, 6.0, 90),
    IMU: (7.6, -16.4, 90),
    find("32.768kHz"): (3.6, -16.4, 90),
    find("22pF", "XOUT32"): (3.6, -14.0, 0),
    c100_3v3[2]: (11.4, -14.4, 90),
    c100_3v3[3]: (7.6, -12.6, 0),
    find("10k", "IMU_RST"): (5.0, -12.6, 0),
    find("10k", "IMU_BOOTLOAD"): (11.4, -18.0, 90),
    find("4.7k", "SDA1"): (-1.2, -17.6, 90),
    find("4.7k", "SCL1"): (1.2, -17.6, 90),
    find("B5819W", "VBUS_USB"): (-15.0, -12.6, 0),
}
imu_cap_ref = [find("1uF", "IMU_CAP")]

REACH = {
    find("22uF", nth=2): 12.0,
    find("USBLC6-2SC6"): 8.0,
}
for ref, (x, y, rot) in wanted.items():
    lay.want(ref, x, y, rot, reach=REACH.get(ref, 6.0), bottom_ok=False)
lay.want(imu_cap_ref[0], 7.6, -17.2, 0, reach=1.5, bottom_ok=True)
lay.want(find("22pF", "XIN32"), 3.1, -15.25, 0, reach=1.0, bottom_ok=True)
solved = lay.solve()
if "--plot" in sys.argv:
    import debug_plot

    debug_plot.plot(lay, solved, roof_keepout, under_tall_keepout, outline_b)
k = lay.to_kicad

spec = {
    "inherit": "jlcpcb_2layer",
    "rules": {"min_through_hole_diameter": 0.25},
    "flow": {
        "tries": 10,
        "fr_passes": 50,
        "fanout_nets": [],
        "stitch_nets": ["BAT_IN", "VBATT"],
        "stitch_pitch": 1.0,
    },
    "order": {"assembled": 2},
    "schematic": {
        "sheets": [
            ["Power", ["U2", "U3", "U1"]],
            ["MCU and USB", ["ESP1", "USB1", "U5", "LED1", "H1"]],
            ["IMU and radio", ["U4", "H2"]],
        ],
        "pads_sheet": "Power",
        "power_nets": ["VBATT"],
    },
    "layers": 4,
    "outer_copper_oz": 2,
    "plane_layers": ["In1.Cu", "In2.Cu"],
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
    "board": {"outline": [k(x, y) for x, y in outline_b], "corner_radius": 1.0},
    "netclasses": {
        "Default": {"track_width": 0.16, "via_diameter": 0.5, "via_drill": 0.25},
        "Ground": {"track_width": 0.2, "via_diameter": 0.6, "via_drill": 0.3, "nets": ["GND"]},
        "Battery": {
            "track_width": 0.3,
            "via_diameter": 0.6,
            "via_drill": 0.3,
            "nets": ["BAT_IN", "VBATT", "PACK+", "PACK_MID"],
        },
        "Power": {
            "track_width": 0.3,
            "via_diameter": 0.6,
            "via_drill": 0.3,
            "nets": ["+5V", "V5_BUCK", "VBUS_USB", "SW"],
        },
        "Rail": {"track_width": 0.2, "nets": ["+3V3"]},
        "USB": {"track_width": 0.2, "nets": ["USB_DP", "USB_DM"]},
    },
    "accepted": [],
    "parts": lay.kicad_parts(solved),
    "courtyard_overrides": lay.overrides,
    "zones": [
        {"net": "GND", "layers": ["F.Cu", "B.Cu"]},
        {"net": "GND", "layers": ["In1.Cu"]},
        {"net": "GND", "layers": ["In2.Cu"]},
        {
            "name": "pack series join",
            "net": "PACK_MID",
            "layers": ["F.Cu", "B.Cu"],
            "priority": 2,
            "outline": [
                k(x, y) for x, y in [(-26.2, -20.0), (-17.3, -20.0), (-17.3, -13.4), (-26.2, -13.4)]
            ],
        },
        {
            "name": "switch loop out",
            "net": "PACK+",
            "layers": ["F.Cu"],
            "priority": 2,
            "outline": [
                k(x, y) for x, y in [(17.3, -20.0), (26.2, -20.0), (26.2, -14.6), (17.3, -14.6)]
            ],
        },
        {
            "name": "switch loop back, F pour under SW_BACK",
            "net": "BAT_IN",
            "layers": ["F.Cu"],
            "priority": 3,
            "outline": [
                k(x, y) for x, y in [(12.9, -20.0), (17.0, -20.0), (17.0, -11.4), (12.9, -11.4)]
            ],
        },
        {
            "name": "switch loop back, B pour under the shunt",
            "net": "BAT_IN",
            "layers": ["B.Cu"],
            "priority": 3,
            "outline": [
                k(x, y) for x, y in [(13.0, -20.0), (17.4, -20.0), (17.4, -10.3), (13.0, -10.3)]
            ],
        },
        {
            "keepout": ["tracks", "vias"],
            "layers": ["B.Cu"],
            "outline": [
                k(x, y) for x, y in [(16.6, -5.6), (19.4, -5.6), (19.4, -8.8), (16.6, -8.8)]
            ],
        },
        {
            "keepout": ["tracks", "vias"],
            "layers": ["B.Cu"],
            "outline": [
                k(x, y) for x, y in [(18.75, -10.4), (19.6, -10.4), (19.6, -11.05), (18.75, -11.05)]
            ],
        },
        {
            "net": "VBATT",
            "layers": ["F.Cu"],
            "priority": 3,
            "outline": [
                k(x, y)
                for x, y in [
                    (23.9, -10.3),
                    (28.4, -10.3),
                    (28.4, -14.75),
                    (29.6, -14.75),
                    (29.6, -15.6),
                    (28.2, -15.6),
                    (28.2, -17.0),
                    (26.5, -17.0),
                    (26.5, -11.6),
                    (23.9, -11.6),
                ]
            ],
        },
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
                    (29.7, -14.6),
                    (29.7, -17.0),
                    (26.4, -17.0),
                    (26.4, -14.6),
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
