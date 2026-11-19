"""Mr Stabs Mk2 main PCB placeholder outline and keep-outs."""

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, NamedTuple

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
ROBOT_STEPS = {
    "top_plate": STEP_DIR / "Mr Stabs Mk2 - Mr Stabs Mk2 Top Plate.step",
    "bottom_plate": STEP_DIR / "Mr Stabs Mk2 - Mr Stabs Mk2 Bottom Plate.step",
    "clamp_left": STEP_DIR / "Mr Stabs Mk2 - Mr Stabs Mk2 Motor Clamp Left.step",
    "clamp_right": STEP_DIR / "Mr Stabs Mk2 - Mr Stabs Mk2 Motor Clamp Right.step",
}

MOTOR_BORE_CENTER_X = 35.4
MOTOR_BORE_R = 13.0
MOTOR_BORE_LEN = 38.2
SWEEP_START_SHORT_OF_SEAT = 0.5
FLAT_HEAD_ANGLE_DEG = 82.0
ESC_EDGE_H = 12.0
TILT_DEG = math.degrees(math.atan2(0.16655509, 0.98603215))
CHASSIS_SHIFT = (0.0, 7.4276, -10.314653658)
INSTALL_DIR = (0.0, -0.328457, -0.944519)

C_MAX = (Align.CENTER, Align.CENTER, Align.MAX)
C_MIN = (Align.CENTER, Align.CENTER, Align.MIN)


@dataclass
class Params:
    wings: bool = True
    thickness: float = 1.6
    x_half: float = 12.7
    y_max: float = 14.7
    y_min: float = -20.3
    ear_y: float = -5.19
    lip_x: float = 35.2
    tip_y: float = -11.01
    tip_x: float = 46.62
    wall_x: float = 26.5
    ear_front_y: float = -17.37
    inside_r: float = 0.5
    rear_corner_r: float = 1.0
    hole_pitch: float = 19.6
    hole_dia: float = 3.66
    washer_od: float = 9.0
    washer_t: float = 3.0
    washer_flat_x: float = 13.0
    screw_len: float = 9.53
    screw_head_dia: float = 6.66
    screw_dia: float = 3.5


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


def outline_extents(p: Params) -> tuple[float, float]:
    pts = outline(p)
    xs, ys = [x for x, _ in pts], [y for _, y in pts]
    return max(xs) - min(xs), max(ys) - min(ys)


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


class Keepout(NamedTuple):
    name: str
    side: Literal["top", "bottom"]
    x: float
    y: float
    size_x: float
    size_y: float
    height: float


STEM_COMPONENTS = [
    Keepout("crossfire_nano_rx", "top", 0.0, 0.0, 18.0, 11.0, 3.0),
    Keepout("bno055", "top", 0.0, -11.6, 7.0, 7.0, 1.2),
    Keepout("pad_bat_pos", "top", -9.0, -18.2, 3.0, 3.0, 0.5),
    Keepout("pad_bat_neg", "top", -3.0, -18.2, 3.0, 3.0, 0.5),
    Keepout("pad_esc_pos", "top", 3.0, -18.2, 3.0, 3.0, 0.5),
    Keepout("pad_esc_neg", "top", 9.0, -18.2, 3.0, 3.0, 0.5),
    Keepout("esp32_s3", "bottom", 0.0, 0.0, 10.0, 10.0, 1.2),
    Keepout("antenna", "bottom", 0.0, 12.4, 3.2, 1.6, 1.1),
    Keepout("shunt", "bottom", -6.5, -17.6, 6.4, 3.2, 0.7),
    Keepout("ina238", "bottom", 0.0, -17.6, 4.0, 4.0, 1.1),
    Keepout("buck_ldo", "bottom", 7.6, -17.6, 5.5, 5.5, 2.0),
    Keepout("esc_signal_pads", "bottom", 10.5, 0.0, 2.0, 4.0, 0.5),
    Keepout("esc_signal_pads_2", "bottom", -10.5, 0.0, 2.0, 4.0, 0.5),
    Keepout("neopixel", "bottom", -7.3, 2.0, 3.5, 3.5, 1.0),
    Keepout("prog_pads", "bottom", 7.3, 2.0, 3.5, 3.5, 0.3),
]

WING_COMPONENTS = [
    Keepout("crossfire_nano_rx", "top", 0.0, 0.0, 18.0, 11.0, 3.0),
    Keepout("bno055", "top", 0.0, -11.6, 7.0, 7.0, 1.2),
    Keepout("pad_bat_pos", "top", -9.0, -18.3, 3.0, 3.0, 0.5),
    Keepout("pad_bat_neg", "top", -3.0, -18.3, 3.0, 3.0, 0.5),
    Keepout("pad_esc_pos", "top", 3.0, -18.3, 3.0, 3.0, 0.5),
    Keepout("pad_esc_neg", "top", 9.0, -18.3, 3.0, 3.0, 0.5),
    Keepout("esp32_s3", "top", -27.5, -11.28, 13.0, 11.5, 1.2),
    Keepout("antenna", "top", -44.5, -13.0, 1.6, 3.2, 1.1),
    Keepout("power", "top", 24.0, -12.0, 10.0, 8.0, 2.0),
    Keepout("shunt", "bottom", -6.5, -17.6, 6.4, 3.2, 0.7),
    Keepout("ina238", "bottom", 0.0, -17.6, 4.0, 4.0, 1.1),
    Keepout("neopixel", "bottom", 0.0, 2.0, 3.5, 3.5, 1.0),
    Keepout("esc_connector", "bottom", 24.0, -9.6, 9.0, 5.0, 10.0),
    Keepout("usb_connector", "bottom", -24.0, -9.6, 9.0, 5.0, 10.0),
]

ESC_CENTER_X = 27.0
ESC_CENTER_Y = -17.5
ESC_BELOW_BOARD_GAP = 0.5


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
        cone_h = (
            (p.screw_head_dia - p.screw_dia) / 2 / math.tan(math.radians(FLAT_HEAD_ANGLE_DEG / 2))
        )
        w -= Pos(x, y, -p.thickness - p.washer_t) * Cone(
            p.screw_head_dia / 2, p.screw_dia / 2, cone_h, align=C_MIN
        )
        w -= Pos(x, y, 0) * Cylinder(p.screw_dia / 2, 20)
        out[f"washer_{i}"] = w
    return out


def screw_shapes(p: Params) -> dict:
    """#6 flat head plastite, head flush with the washer's underside, simplified."""
    out = {}
    cone_h = (p.screw_head_dia - p.screw_dia) / 2 / math.tan(math.radians(FLAT_HEAD_ANGLE_DEG / 2))
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
    esc = Rot(90, 0, 0) * import_step(ESC_STEP)
    z = -p.thickness - ESC_BELOW_BOARD_GAP - ESC_EDGE_H / 2
    return {
        "esc_right": Pos(ESC_CENTER_X, ESC_CENTER_Y, z) * esc,
        "esc_left": Pos(-ESC_CENTER_X, ESC_CENTER_Y, z) * esc,
    }


def robot_in_board_frame() -> dict:
    """Chassis, plates, clamps, and the drive motors as their Ø26 chassis bores."""
    out = {"chassis": chassis_in_board_frame()}
    for name, path in ROBOT_STEPS.items():
        out[name] = Pos(*CHASSIS_SHIFT) * Rot(-TILT_DEG, 0, 0) * import_step(path)
    for side, sx in (("right", 1), ("left", -1)):
        motor = (
            Pos(sx * MOTOR_BORE_CENTER_X, 0, 0)
            * Rot(0, 90, 0)
            * Cylinder(MOTOR_BORE_R, MOTOR_BORE_LEN)
        )
        out[f"motor_{side}"] = Pos(*CHASSIS_SHIFT) * Rot(-TILT_DEG, 0, 0) * motor
    return out


def _install_sweep(p: Params):
    top = build(p).faces().sort_by(Axis.Z)[-1]
    start = Vector(*INSTALL_DIR) * SWEEP_START_SHORT_OF_SEAT
    return extrude(Pos(start.X, start.Y, start.Z) * top, amount=45, dir=INSTALL_DIR)


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
    sweep = _install_sweep(p)
    worst = min((sweep.distance_to(r), rn) for rn, r in robot.items() if rn != "top_plate")
    verdict = "FAIL" if worst[0] < 0.25 else "PASS"
    lines.append(f"{verdict}  {'install sweep':20s} {worst[0]:7.3f} mm to {worst[1]}")
    return lines


def sections(p: Params) -> list[str]:
    out = [f"x={p.hole_pitch / 2}", f"y={p.hole_pitch / 2}", "y=0"]
    if p.wings:
        out += [f"x={ESC_CENTER_X}", f"y={ESC_CENTER_Y}", "x=-27.5"]
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
        "bbox": (*outline_extents(p), 1.6),
        "holes": {3.66: 4},
        "hole_at": {3.66: [{"x": x, "y": y} for x, y in hole_xy(p)]},
        "gap": {"chassis": 0.25},
        "allow_interference": [f"screw_{i}" for i in range(4)],
        "clearance": {
            "esc_left": ESC_BELOW_BOARD_GAP - 0.05,
            "esc_right": ESC_BELOW_BOARD_GAP - 0.05,
        }
        if p.wings
        else {},
    }


if __name__ == "__main__":
    import sys

    params = Params(wings="--stem" not in sys.argv)
    report = fit_report(params)
    print("\n".join(report))
    sys.exit(any(line.startswith("FAIL") for line in report))
