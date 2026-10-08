"""Mr Stabs Mk2 main PCB placeholder: the largest board that fits the chassis electronics bay.

Stands in for a single board that replaces the QT Py ESP32-S3, the BNO055 breakout, the Matek
I2C-INA-BM and the loose Crossfire Nano RX. Sized from the chassis STEP, laid out with
surface-mount keep-outs to show the parts fit. Not a routed design.

Two outlines, picked by `Params.wings`:
- wings = True (default): the stem between the bosses plus an ear into each ESC pocket,
  93.2 x 35.0, 1621 mm2 (the stem alone is 846). The ESCs stand on their long edge under
  the ears' front strip.
- wings = False: the stem only, 25.4 x 35.0, ESC pockets left empty.

Spec
- Process: 1.6 mm FR4, routed outline, 0.5 mm inside corner radius (1 mm router bit).
  No PCB process in check.py, so PROCESS = "none". MATERIAL is a density stand-in only
  (FR4 is about 1.85 g/cm3, not in the table).
- Frame: origin at the center of the 4-hole pattern on the board's top face, which seats
  against the chassis bosses. +Z points into the chassis (toward the bosses), +Y toward the
  rear wall. The frame is the Onshape assembly frame rotated -9.588 deg about X (the hole
  axes lean that far) and shifted by CHASSIS_SHIFT; `chassis_in_board_frame()` applies it.
- Install: wedge plate off, the board slides in along INSTALL_DIR, the plate's normal,
  19.2 deg off the board's axis. Straight along the board's axis is impossible: the rear
  cross member sits under the stem's last 8.4 mm. The ear outline is set by that sweep as
  much as by the walls at the seat. `python mr_stabs_mk2_pcb.py` checks the sweep and every
  keep-out against the chassis, plates, clamps and motors.
- Interfaces (0.3 mm gaps, edges from the chassis STEP):
  - Chassis (TPU): 4 bosses with Ø2.50 plastite pilot holes on a 19.6 mm square, flat
    faces at the board's top face. #6 flat-head plastite screws come up from below
    through 3D-printed countersunk washers (user's part), the board, into the bosses.
    The drivers reach all four along INSTALL_DIR.
  - Stem: side walls at x = ±13.0, rear wall at y = 15.0.
  - Ears, seated: ledge under the rear edge at y -4.89; clamp boss at y -10.71 past x 35.5.
  - Ears, on the way in: outer wall at x 46.92; the wing front wall's lip (z -1 to -8.5)
    limits the ear front to y -17.67 outboard of x 27.0.
  - Front edge stops short of the power-wire bay (user: no PCB there).
  - ESCs (ReadyToSky 35A, 26 x 12 x 5, wired, not soldered): on edge under each ear,
    x 14 to 40, y -20 to -15, top 0.5 mm below the board. The only rigid placement in the
    pocket: lying flat needs 26 mm along x and 12 along y, and the pocket narrows to 4.8 mm
    past x 35.5.
- Critical dimensions: 4 x Ø3.66 at (±9.8, ±9.8); 1.6 thick.

Part placement, wings layout (keep-out boxes in `mates()`, sizes from datasheets):
- Top side, under the chassis roof (3.78 mm headroom over stem and ears): Crossfire Nano RX
  18 x 11 x 3.0 between the boss rows; BNO055 block between the front bosses; wire pads at
  the front edge; ESP32-S3R2 block on the left ear with the chip antenna on the left tip
  (no copper past x -40.5); buck/LDO power block on the right ear.
- Bottom side: INA238 + 2512 shunt at the front by the wire pads; NeoPixel; ESC and USB
  connectors (Molex 505567 vertical, mated, 10 mm) on the ears' rear strip, where there is
  14 mm down to the wedge. Nothing over the ESCs.

Open questions
- ESP32-S3-MINI-1 (15.4 x 20.5) still does not fit: the left ear is 12.2 mm deep past
  x 26.5 and 15.1 inboard of it. The bare ESP32-S3R2 is laid out instead.
- The antenna is 2.6 mm past the end of the left ESC, which sits 0.5 mm below the board.
- ESC leads are not modeled: phase wires leave the outer end at x 40 toward the motor,
  power and signal leave the inner end at x 14 toward the wire bay. The ESCs go in after
  the board and are placed by hand, so no sweep covers them.
- The install sweep is a straight slide along the wedge normal; the closest pass is 0.27 mm
  at the inside corner where the ear front steps back. A flexing TPU chassis may allow more.
- Crossfire Nano RX height assumed 3.0 mm (TBS lists 18 x 11 only). Top headroom is 3.78.
- Countersunk washer assumed Ø9.0 x 3.0 with an 82 deg seat. The rear two are D-cut at
  |x| = 13.0 because the side walls step in to 13.3 below the board. The #6 head (Ø6.66)
  reaches x = 13.13 there, so the seat rim breaks through the flat by 0.13 mm. 3.0 thick puts
  a 3/8 screw tip at 4.93, short of the 5.0 pilot bore.
"""

import math
from dataclasses import dataclass
from pathlib import Path

from build123d import (
    Align,
    Axis,
    Box,
    Cone,
    Cylinder,
    Face,
    Part,
    Pos,
    Rot,
    Vector,
    Wire,
    extrude,
    fillet,
    import_step,
)

PROCESS = "none"
MATERIAL = "pc"  # density stand-in, see docstring

STEP_DIR = Path.home() / "Downloads" / "Mr Stabs Mk2"
CHASSIS_STEP = STEP_DIR / "Mr Stabs Mk2 - Mr Stabs Mk2 Chassis.step"
ESC_STEP = STEP_DIR / "Mr Stabs Mk2 - ReadyToSky 35A Brushless ESC.step"
# Robot parts the keep-outs are checked against. The wedge (top plate) is off during install.
ROBOT_STEPS = {
    "top_plate": STEP_DIR / "Mr Stabs Mk2 - Mr Stabs Mk2 Top Plate.step",
    "bottom_plate": STEP_DIR / "Mr Stabs Mk2 - Mr Stabs Mk2 Bottom Plate.step",
    "clamp_left": STEP_DIR / "Mr Stabs Mk2 - Mr Stabs Mk2 Motor Clamp Left.step",
    "clamp_right": STEP_DIR / "Mr Stabs Mk2 - Mr Stabs Mk2 Motor Clamp Right.step",
}

# Hole axes in the chassis STEP lean (0, -0.16656, 0.98603): 9.588 deg about X.
TILT_DEG = math.degrees(math.atan2(0.16655509, 0.98603215))
# Hole-pattern center on the boss faces, in the tilted frame (chassis STEP, measured).
CHASSIS_SHIFT = (0.0, 7.4276, -10.314653658)
# The board goes in through the wedge-plate opening along the plate's normal, 19.2 deg off
# the board's own axis (wedge plate STEP). Straight along -Z, the rear cross member under the
# stem's last 8.4 mm and the rear screws' driver path are both blocked.
INSTALL_DIR = (0.0, -0.328457, -0.944519)

C_MAX = (Align.CENTER, Align.CENTER, Align.MAX)
C_MIN = (Align.CENTER, Align.CENTER, Align.MIN)


@dataclass
class Params:
    wings: bool = True  # user: use the ESC pockets, as E3 did
    thickness: float = 1.6  # user
    x_half: float = 12.7  # side walls at ±13.0 (chassis STEP), 0.3 gap
    y_max: float = 14.7  # rear wall at 15.0 (chassis STEP), 0.3 gap
    y_min: float = -20.3  # front wall -20.62 (chassis STEP), 0.3 gap; wire bay beyond
    # Ear edges, measured by sweeping each edge along INSTALL_DIR (chassis STEP), 0.3 gap:
    ear_y: float = -5.19  # ledge under the board edge at y -4.89
    lip_x: float = 35.2  # motor clamp boss starts at x 35.5
    tip_y: float = -11.01  # clamp boss at board level reaches y -10.71
    tip_x: float = 46.62  # outer wall at x 46.92 along the install path
    wall_x: float = (
        26.5  # wing front wall starts at x 27.0; the 0.5 inside fillet needs the extra 0.2
    )
    ear_front_y: float = -17.37  # wing front wall lip, y -17.67 along the install path
    inside_r: float = 0.5  # 1 mm router bit
    rear_corner_r: float = 1.0  # rear corners pass the wall steps on the way in
    hole_pitch: float = 19.6  # boss pilot holes (chassis STEP)
    hole_dia: float = 3.66  # #6 close clearance, fasteners.md
    washer_od: float = 9.0  # assumed, user's printed countersunk washer
    washer_t: float = 3.0  # assumed; puts the 3/8 screw tip 0.1 short of the pilot floor
    washer_flat_x: float = 13.0  # wall below the board at |x| 13.3 (chassis STEP), 0.3 gap
    screw_len: float = 9.53  # #6 x 3/8 flat head plastite, overall (assembly STEP)
    screw_head_dia: float = 6.66  # assembly STEP
    screw_dia: float = 3.5  # #6 plastite major, approximate


def chassis_in_board_frame() -> Part:
    ch = import_step(CHASSIS_STEP)
    return Pos(*CHASSIS_SHIFT) * Rot(-TILT_DEG, 0, 0) * ch


def hole_xy(p: Params) -> list[tuple[float, float]]:
    h = p.hole_pitch / 2
    return [(sx * h, sy * h) for sx in (-1, 1) for sy in (-1, 1)]


def outline(p: Params) -> list[tuple[float, float]]:
    """Board outline: down the right side from the rear corner, back up the left."""
    if not p.wings:
        right = [(p.x_half, p.y_max), (p.x_half, p.y_min)]
    else:
        right = [
            (p.x_half, p.y_max),
            (p.x_half, p.ear_y),
            (p.lip_x, p.ear_y),
            (p.lip_x, p.tip_y),
            (p.tip_x, p.tip_y),
            (p.tip_x, p.ear_front_y),
            (p.wall_x, p.ear_front_y),
            (p.wall_x, p.y_min),
        ]
    return right + [(-x, y) for x, y in reversed(right)]


def build(p: Params) -> Part:
    pts = outline(p)
    face = Face(Wire.make_polygon([Vector(x, y, 0) for x, y in pts], close=True))
    board = extrude(face, amount=p.thickness, dir=(0, 0, -1))

    verticals = board.edges().filter_by(Axis.Z)
    assert len(verticals) == len(pts)
    others = verticals.filter_by(lambda e: e.center().Y < p.y_max - 0.01)
    assert len(others) == len(pts) - 2
    board = fillet(others, p.inside_r)
    rear = board.edges().filter_by(Axis.Z).filter_by(lambda e: e.center().Y > p.y_max - 0.01)
    assert len(rear) == 2
    board = fillet(rear, p.rear_corner_r)

    for x, y in hole_xy(p):
        board -= Pos(x, y, 0) * Cylinder(p.hole_dia / 2, p.thickness, align=C_MAX)
    return board


# (name, side, x, y, size_x, size_y, height). Side "top" sits on z = 0 and rises toward the
# roof; "bottom" hangs from z = -thickness.
STEM_COMPONENTS = [
    ("crossfire_nano_rx", "top", 0.0, 0.0, 18.0, 11.0, 3.0),  # TBS: 18 x 11, height assumed
    ("bno055", "top", 0.0, -11.6, 7.0, 7.0, 1.2),  # LGA-28 5.2 x 3.8 + 32 kHz crystal + caps
    ("pad_bat_pos", "top", -9.0, -18.2, 3.0, 3.0, 0.5),  # 16-18 AWG solder pads
    ("pad_bat_neg", "top", -3.0, -18.2, 3.0, 3.0, 0.5),
    ("pad_esc_pos", "top", 3.0, -18.2, 3.0, 3.0, 0.5),
    ("pad_esc_neg", "top", 9.0, -18.2, 3.0, 3.0, 0.5),
    ("esp32_s3", "bottom", 0.0, 0.0, 10.0, 10.0, 1.2),  # QFN-56 7 x 7 + flash + crystal
    ("antenna", "bottom", 0.0, 12.4, 3.2, 1.6, 1.1),  # 2.4 GHz chip antenna, rear edge
    ("shunt", "bottom", -6.5, -17.6, 6.4, 3.2, 0.7),  # 2512, 1 mohm
    ("ina238", "bottom", 0.0, -17.6, 4.0, 4.0, 1.1),  # VSSOP-10 3 x 3 + caps
    ("buck_ldo", "bottom", 7.6, -17.6, 5.5, 5.5, 2.0),  # TPS54202 + inductor + AP2112K
    ("esc_signal_pads", "bottom", 10.5, 0.0, 2.0, 4.0, 0.5),  # left/right DShot + GND
    ("esc_signal_pads_2", "bottom", -10.5, 0.0, 2.0, 4.0, 0.5),
    ("neopixel", "bottom", -7.3, 2.0, 3.5, 3.5, 1.0),  # SK6812 3535 status LED
    ("prog_pads", "bottom", 7.3, 2.0, 3.5, 3.5, 0.3),  # USB D+/D-/5V/GND pigtail pads
]

WING_COMPONENTS = [
    ("crossfire_nano_rx", "top", 0.0, 0.0, 18.0, 11.0, 3.0),  # TBS: 18 x 11, height assumed
    ("bno055", "top", 0.0, -11.6, 7.0, 7.0, 1.2),  # LGA-28 5.2 x 3.8 + 32 kHz crystal + caps
    ("pad_bat_pos", "top", -9.0, -18.3, 3.0, 3.0, 0.5),  # 16-18 AWG solder pads
    ("pad_bat_neg", "top", -3.0, -18.3, 3.0, 3.0, 0.5),
    ("pad_esc_pos", "top", 3.0, -18.3, 3.0, 3.0, 0.5),
    ("pad_esc_neg", "top", 9.0, -18.3, 3.0, 3.0, 0.5),
    ("esp32_s3", "top", -27.5, -11.28, 13.0, 11.5, 1.2),  # QFN-56 + flash + crystal + RF match
    ("antenna", "top", -44.5, -13.0, 1.6, 3.2, 1.1),  # 2.4 GHz chip antenna, no copper past x -40.5
    ("power", "top", 24.0, -12.0, 10.0, 8.0, 2.0),  # TPS54202 + inductor + caps + AP2112K
    ("shunt", "bottom", -6.5, -17.6, 6.4, 3.2, 0.7),  # 2512, 1 mohm
    ("ina238", "bottom", 0.0, -17.6, 4.0, 4.0, 1.1),  # VSSOP-10 3 x 3 + caps
    ("neopixel", "bottom", 0.0, 2.0, 3.5, 3.5, 1.0),  # SK6812 3535 status LED
    ("esc_connector", "bottom", 24.0, -9.6, 9.0, 5.0, 10.0),  # Molex 505567 vertical, mated
    ("usb_connector", "bottom", -24.0, -9.6, 9.0, 5.0, 10.0),  # same part, USB pigtail
]

ESC_X = 27.0  # ESC spans x 14 to 40: inner end by the wire bay, outer end toward the motor
ESC_Y = -17.5  # y -20 to -15: 0.62 to the front wall
ESC_GAP = 0.5  # below the board's underside


def component_shapes(p: Params) -> dict:
    out = {}
    for name, side, x, y, sx, sy, h in WING_COMPONENTS if p.wings else STEM_COMPONENTS:
        if side == "top":
            out[name] = Pos(x, y, 0) * Box(sx, sy, h, align=C_MIN)
        else:
            out[name] = Pos(x, y, -p.thickness) * Box(sx, sy, h, align=C_MAX)
    return out


def washer_shapes(p: Params) -> dict:
    """The user's printed countersunk washers, D-cut where the side walls drop below the board."""
    out = {}
    for i, (x, y) in enumerate(hole_xy(p)):
        w = Pos(x, y, -p.thickness) * Cylinder(p.washer_od / 2, p.washer_t, align=C_MAX)
        w &= Box(2 * p.washer_flat_x, 100, 100)
        cone_h = (p.screw_head_dia - p.screw_dia) / 2 / math.tan(math.radians(41))  # 82 deg head
        w -= Pos(x, y, -p.thickness - p.washer_t) * Cone(
            p.screw_head_dia / 2, p.screw_dia / 2, cone_h, align=C_MIN
        )
        w -= Pos(x, y, 0) * Cylinder(p.screw_dia / 2, 20)
        out[f"washer_{i}"] = w
    return out


def screw_shapes(p: Params) -> dict:
    """#6 flat head plastite, head flush with the washer's underside, simplified."""
    out = {}
    cone_h = (p.screw_head_dia - p.screw_dia) / 2 / math.tan(math.radians(41))
    z0 = -p.thickness - p.washer_t
    for i, (x, y) in enumerate(hole_xy(p)):
        head = Pos(x, y, z0) * Cone(p.screw_head_dia / 2, p.screw_dia / 2, cone_h, align=C_MIN)
        shank = Pos(x, y, z0 + cone_h) * Cylinder(
            p.screw_dia / 2, p.screw_len - cone_h, align=C_MIN
        )
        out[f"screw_{i}"] = head + shank
    return out


def esc_shapes(p: Params) -> dict:
    """ReadyToSky 35A ESCs on their long edge under each ear (STEP: 26 x 12 x 5, centered)."""
    if not p.wings:
        return {}
    esc = Rot(90, 0, 0) * import_step(ESC_STEP)  # 12 mm side now along Z
    z = -p.thickness - ESC_GAP - 6.0
    return {"esc_right": Pos(ESC_X, ESC_Y, z) * esc, "esc_left": Pos(-ESC_X, ESC_Y, z) * esc}


def robot_in_board_frame() -> dict:
    """Chassis, plates, clamps, and the drive motors as their Ø26 chassis bores."""
    out = {"chassis": chassis_in_board_frame()}
    for name, path in ROBOT_STEPS.items():
        out[name] = Pos(*CHASSIS_SHIFT) * Rot(-TILT_DEG, 0, 0) * import_step(path)
    for side, sx in (("right", 1), ("left", -1)):
        motor = Pos(sx * 35.4, 0, 0) * Rot(0, 90, 0) * Cylinder(13, 38.2)  # bores x 16.3 to 54.5
        out[f"motor_{side}"] = Pos(*CHASSIS_SHIFT) * Rot(-TILT_DEG, 0, 0) * motor
    return out


def fit_report(p: Params) -> list[str]:
    """Checks check.py cannot: keep-outs against every robot part, and the install sweep."""
    robot = robot_in_board_frame()
    lines = []
    things = {**component_shapes(p), **washer_shapes(p), **esc_shapes(p)}
    for name, shape in things.items():
        worst = min(
            ((-(shape & r).volume if (shape & r) else shape.distance_to(r)), rn)
            for rn, r in robot.items()
        )
        lines.append(
            f"{'FAIL' if worst[0] < 0.25 else 'PASS'}  {name:20s} {worst[0]:7.3f} mm to {worst[1]}"
        )
    # Install: the board comes in along INSTALL_DIR with the wedge plate off. The sweep starts
    # 0.5 mm short of the seat, where its top meets the boss faces; check.py's chassis gap
    # covers the seated board.
    top = build(p).faces().sort_by(Axis.Z)[-1]
    start = Vector(*INSTALL_DIR) * 0.5
    sweep = extrude(Pos(start.X, start.Y, start.Z) * top, amount=45, dir=INSTALL_DIR)
    worst = min((sweep.distance_to(r), rn) for rn, r in robot.items() if rn != "top_plate")
    verdict = "FAIL" if worst[0] < 0.25 else "PASS"
    lines.append(f"{verdict}  {'install sweep':20s} {worst[0]:7.3f} mm to {worst[1]}")
    return lines


def sections(p: Params) -> list[str]:
    out = [f"x={p.hole_pitch / 2}", f"y={p.hole_pitch / 2}", "y=0"]
    if p.wings:
        out += [f"x={ESC_X}", f"y={ESC_Y}", "x=-27.5"]
    return out


def mates(p: Params) -> dict:
    return {
        "chassis": chassis_in_board_frame(),
        **component_shapes(p),
        **washer_shapes(p),
        **screw_shapes(p),
        **esc_shapes(p),
    }


def expect(p: Params) -> dict:
    return {
        # Outline extents from the chassis edges (docstring), 0.3 mm gaps.
        "bbox": (93.24 if p.wings else 25.4, 35.0, 1.6),
        "holes": {3.66: 4},
        "hole_at": {3.66: [{"x": x, "y": y} for x, y in hole_xy(p)]},
        # The board seats on the bosses, so it touches the chassis; every other chassis
        # face must stay 0.3 mm away (side and rear walls).
        "gap": {"chassis": 0.25},
        # The plastite threads form in the TPU pilot holes, so the screws overlap the chassis.
        "allow_interference": [f"screw_{i}" for i in range(4)],
        "clearance": {"esc_left": ESC_GAP - 0.05, "esc_right": ESC_GAP - 0.05} if p.wings else {},
    }


if __name__ == "__main__":
    import sys

    params = Params(wings="--stem" not in sys.argv)
    report = fit_report(params)
    print("\n".join(report))
    sys.exit(any(line.startswith("FAIL") for line in report))
