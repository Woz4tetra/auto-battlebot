"""Shared layout constants for the cage pack."""

from pathlib import Path
from typing import NamedTuple

HERE = Path(__file__).parent
VENDOR = HERE / "vendor"
ZED_MOUNT = HERE.parent / "zed_box_mini_mount"
ZED_STEP = ZED_MOUNT / "out" / "step" / "zed_box_mini.step"
ZED_STEP_BASE_SHEET_Z = 12.0
OPENUPS_STEP = VENDOR / "openups.step"

CUP_D = 101.5
CUP_SKIRT_D = 106.7
CUP_H = 48.2
CUP_REACH = 139.7 - CUP_SKIRT_D / 2
CUP_TAB_ANGLE = 90.0
CUP_HEAD = (16.7, 5.2)
CUP_WASHER = (25.4, 1.5)
CP10_PITCH = 153.5
CP10_Y = (0.0, -CP10_PITCH)

BED = 256.0
BED_MARGIN = 2.0
PANEL_T = 3.0
WALL_T = 2.4
PANEL_Z0 = CUP_H
PANEL_Z1 = PANEL_Z0 + PANEL_T
CP10_SPOT = (27.0, 1.4)
DEPTH = 67.0
X_IN = (-112.0, 97.0)
Y_IN = (-183.0, 26.0)
X_OUT = (X_IN[0] - WALL_T, X_IN[1] + WALL_T)
Y_OUT = (Y_IN[0] - WALL_T, Y_IN[1] + WALL_T)
LID_UNDERSIDE_Z = PANEL_Z1 + DEPTH
OUTER_CORNER_R = 4.0  # outside vertical corners; inside ones are OUTER_CORNER_R - WALL_T
RIB_H = 6.0
RIB_T = 2.4
HR10_PANEL_T = 1.8


class SpotFace(NamedTuple):
    dia: float
    depth: float


CUP_Y = -70.0
CUP_OUT = 45.0
CUP_X_NEG_POS = (X_OUT[0] - CUP_OUT, X_OUT[1] + CUP_OUT)
CUP_ROT_NEG_POS = (0.0, 180.0)
ARM_WIDTH_Y = 40.0
ARM_PLATE_T = 8.0
ARM_TONGUE_T = 10.0
ARM_TONGUE = 36.0
ARM_SCREW_IN = 28.0
ARM_SCREW_DY = 12.0
ARM_SPOT = SpotFace(27.0, 3.0)
ARM_SEAT_Z = CUP_H + ARM_PLATE_T - ARM_SPOT.depth
ARM_CBORE = SpotFace(6.5, 3.5)
ARM_BOSS_H = 6.0


def arm_screws() -> list[tuple[float, float]]:
    """(x, y) of the four M3 screws that hold the arms to the panel, -X arm first."""
    xs = (X_OUT[0] + ARM_SCREW_IN, X_OUT[1] - ARM_SCREW_IN)
    return [(x, CUP_Y + s * ARM_SCREW_DY) for x in xs for s in (-1, 1)]


assert X_OUT[1] - X_OUT[0] <= BED - 2 * BED_MARGIN, "tray too wide for the P1S"
assert Y_OUT[1] - Y_OUT[0] <= BED - 2 * BED_MARGIN, "tray too tall for the P1S"

LID_T = 2.4
LID_LIP_DEPTH = 5.0
LID_LIP_T = 1.6
LID_LIP_WALL_GAP = 0.3
LID_BOSS_SIZE = 9.0
LID_BOSS_FULL_DEPTH = 10.0
PORT_Z_MAX = LID_UNDERSIDE_Z - LID_LIP_DEPTH - 1.0

BOSS_H = 8.0
BOSS_D = 8.0
INSERT_HOLE = 4.0
INSERT_LEN = 6.4
CUP_RAIL_HALF_LEN = 28.0
INSERT_BOTTOM_CLEARANCE = 1.0
INSERT_DEPTH = INSERT_LEN + INSERT_BOTTOM_CLEARANCE

ZED_XC = 0.0
ZED_YC = -50.0
ZED_DZ = PANEL_Z1 + BOSS_H - ZED_STEP_BASE_SHEET_Z
ZED_EAR_HOLES = [(ZED_XC + sx * 24.0, ZED_YC + sy * 47.3) for sx in (-1, 1) for sy in (-1, 1)]
ZED_Y0, ZED_Y1 = ZED_YC - 51.8, ZED_YC + 51.98
ZED_BODY_X0, ZED_BODY_X1 = ZED_XC - 71.9, ZED_XC + 71.5
ZED_TOP = 65.6 + ZED_DZ


def zed(x: float, y: float, z: float) -> tuple[float, float, float]:
    """ZED Box Mini STEP point -> this frame."""
    return (ZED_XC + x, ZED_YC + y, z + ZED_DZ)


ZED_RJ45_MOUTH = zed(-71.5, 14.01, 28.22)
ZED_POWER_MOUTH = zed(-71.9, -23.96, 25.11)
ZED_USB_A = zed(71.5, -11.60, 24.53)
ZED_FAKRA_A_TIP = zed(81.25, 6.85, 24.37)
ZED_WIFI_SMA_JACKS = [zed(x, 51.98, 30.0) for x in (-49.5, -29.5)]

PORT_Z = 88.0
SERVICE_PORTS_YZ = {
    "hr10": (-32.0, PORT_Z),
    "gx12": (-92.0, PORT_Z),
}
SWITCH_X, SWITCH_Z = 72.0, 84.0
SWITCH_HOLE = 19.3
SWITCH_BEZEL = (21.8, 2.0)
SWITCH_CAP = 15.4
SWITCH_DEPTH = 29.8
SWITCH_NUT = (23.4, 3.0)
RJ45_AT = (30.0, 93.0)

ANTENNAS_YZ = {
    "zed_wifi_1": (8.0, 100.0),
    "zed_wifi_2": (8.0, ZED_WIFI_SMA_JACKS[1][2]),
    "esp32": (-25.0, 60.0),
}

HOLDER_STEP = VENDOR / "mpd" / "BK-18650-PC8.step"
HOLDER = (78.84, 77.98, 21.54)
HOLDER_PIN = 3.2
CELL_XS = (-28.95, -9.65, 9.65, 28.95)
CELL_D, CELL_L = 18.4, 65.0
CELL_AXIS_DZ = 1.6 + CELL_D / 2
CARRIER_T = 1.6
CARRIER = (80.0, 80.0)
CARRIER_TAB = ((-18.0, -137.0), (-5.0, -124.0))
FOAM = 1.5
UPS_XC, UPS_YC = -58.0, -143.0
BRICK_XC, BRICK_YC = UPS_XC, -142.0
CARRIER_Z0 = PANEL_Z1 + FOAM
HOLDER_Z0 = CARRIER_Z0 + CARRIER_T
BRICK = (CARRIER[0], CARRIER[1], CARRIER_T + HOLDER[2])
UPS_BOSS_H = 25.0
CLAMP_T = 3.0
UPS_TUBE_H = 6.0
CLAMP_Z0 = PANEL_Z1 + UPS_BOSS_H
UPS_BOARD_Z0 = CLAMP_Z0 + CLAMP_T + UPS_TUBE_H


def ups(x: float, y: float, z: float) -> tuple[float, float, float]:
    """OpenUPS STEP point -> this frame (board plane is the STEP's XZ, components +y)."""
    return (UPS_XC - (x - 18.02), UPS_YC + (z - 102.5), UPS_BOARD_Z0 + (y - 60.54))


UPS_HOLES = [ups(x, 60.54, z)[:2] for x in (-27.98, 64.02) for z in (79.0, 126.0)]

PCBA_X0, PCBA_X1 = 12.0, 56.0
PCBA_Y0, PCBA_Y1 = -150.0, -104.0
PCBA_Z = PANEL_Z1 + BOSS_H
PCBA_T = 1.6

DISP_STEP = VENDOR / "waveshare" / "2_8inch-capacitive-touch-lcd.stp"
DISP_TOUCH_X, DISP_TOUCH_Y = 53.5, -140.0
DISP_GLASS_SIZE_XY = (73.06, 50.54)
DISP_GLASS_X_FROM_TOUCH = (-39.53, 33.53)
DISP_VA = (58.2, 43.8)
DISP_DEPTH = 10.0
DISP_FRONT_Z = LID_UNDERSIDE_Z
DISP_BACK_Z = DISP_FRONT_Z - DISP_DEPTH
DISP_STANDOFFS = [(x, y) for x in (-32.9, 27.1) for y in (-20.5, 20.5)]
DISP_EARS = [(x, y) for x in (-13.0, 13.0) for y in (-31.5, 31.5)]
DISP_BRACKET_T = 2.6
DISP_GH = (-29.5, -0.55)
DISP_GH_MOUTH_X = -32.42

CAM_Y = -150.0
CAM_AXIS_Z = 58.0
CAM_SPIN = 240.0
SHIELD_T = 3.0
CAM_SPACER = 9.0
HEAD_SEAT_X = X_OUT[1] + SHIELD_T + CAM_SPACER
CAM_PAD = (30.0, 4.0, 106.0)
SHIELD_SCREW_Z = 100.0
GMSL_HOLE = ZED_FAKRA_A_TIP[1:]
SWEEP_Z_MAX = 124.0
SHIELD_HOOD_Z0 = SWEEP_Z_MAX + 3.0
SHIELD_Y = (-198.0, -108.0)
SHIELD_REACH = 86.0
SHIELD_RIB = 8.0

GLASS_T = 6.35
GLASS_GAP = 25.4
CUP_BELOW_EDGE = 130.0
