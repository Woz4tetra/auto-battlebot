"""Shared layout for the cage pack: a printed enclosure on two SmallRig 4122C suction cups
on the outside of an NHRL cage wall.

Frame: the cage wall's outer polycarbonate face is Z = 0, +Z points out of the cage toward
the viewer, +Y up, X along the wall. Origin on the glass under the tray's upper CP10 hole.

Stack, glass outward:
- Two SmallRig 4122C cups beside the tray, one off each side wall at the same height, pump
  knobs pointing outward (-X, +X). Each cup's whole rim is outside the tray, so a thumb
  reaches its release tabs. A printed arm (cup_arm.py) carries each cup: its tongue sits
  under the back panel on two M3 screws, its plate on the cup's rail with one 3/8-16 screw
  and a load washer.
- Enclosure tray (enclosure.py): black PETG printed on a Bambu P1S, a 3 mm ribbed back panel
  and four 2.4 mm walls. The panel's glass side clears the glass by the cups' height. A flat
  printed lid (enclosure_lid.py) closes the front with six M3 screws into heat-set inserts.
- Inside, on printed bosses with heat-set inserts: the ZED Box Mini across the top, the
  OpenUPS over the 4S1P battery holder at the lower left, the custom PCBA at the lower middle.
- Service wall (-X): HR10 transmitter port, GX12 charge input. Bottom wall (-Y): the
  panel-mount RJ45 for the iPad beside the latching power button, both reachable from
  below with the pack on the glass. Antenna wall (+X): three RP-SMA bulkheads (two ZED
  WiFi, one ESP32).
- Camera outside the box on the +X wall, low: the ZED X One S on a SmallRig 2948 ball head,
  head axis along +X. Nothing round the camera but air, so it spins freely onto the head's
  stud and the head reaches its full range without the lid coming off. A removable black
  PETG light shield (camera_shield.py) is clamped under the head: a hood over the camera's
  back and top, toward the viewer, with lips down its two sides.
"""

from pathlib import Path

HERE = Path(__file__).parent
VENDOR = HERE / "vendor"
ZED_MOUNT = HERE.parent / "zed_box_mini_mount"
ZED_STEP = ZED_MOUNT / "out" / "step" / "zed_box_mini.step"
ZED_STEP_BASE_Z = 12.0  # base sheet underside in that STEP
OPENUPS_STEP = VENDOR / "openups.step"

# SmallRig 4122C (B&H page + dimension photo in ~/Downloads, 2026-11-07)
CUP_D = 101.5  # rubber face, dimension photo
CUP_SKIRT_D = 106.7  # overall width, B&H spec 139.7 x 106.7 x 50.8
CUP_H = 48.2  # glass to the top of the NATO rail, dimension photo
CUP_REACH = 139.7 - CUP_SKIRT_D / 2  # pump knob tip from the cup axis
CUP_TAB_ANGLE = 90.0  # release tabs on the rim, each side of the pump; photos, +-20
CUP_HEAD = (16.7, 5.2)  # 3/8-16 button head: diameter, height, fasteners.md
CUP_WASHER = (25.4, 1.5)  # McMaster 92141A051 under the head: OD, thickness
# The tray's old cup holes stay as a tripod mount: 153.5 apart on X = 0 matches the INNOREL
# CP10 cheese plate's 3/8-16 pair (zed_box_mini_mount/innorel_cp10.py).
CP10_PITCH = 153.5
CP10_Y = (0.0, -CP10_PITCH)

# Enclosure tray: black PETG, printed on a Bambu P1S (256 x 256 x 256) with the panel's
# glass-side face on the bed and the walls rising from it (dfm/fdm.md).
BED = 256.0  # P1S build plate, X and Y
BED_MARGIN = 2.0  # each side; leaves no room for a brim
PANEL_T = 3.0
WALL_T = 2.4  # six 0.4 perimeters
PANEL_Z0 = CUP_H  # back panel's glass-side face, level with the cups' rail tops
PANEL_Z1 = PANEL_Z0 + PANEL_T  # its inner face
CP10_SPOT = (27.0, 1.4)  # spot face for a tripod screw's washer: Ø, depth; 1.6 under it
# Inside, packed round the parts. What sets each face:
# - -X: the ZED's RJ45 and TBP power plugs, their cables and the RJ45 tail running down
#   beside the ZED Box Mini's -X face; also the battery, which must stay left of the lower
#   CP10 rib ring.
# - +X: the ZED's USB-A 90° plug (22 mm head) and the FAKRA plug, which passes through it.
# - +Y: the ZED's WiFi jacks with 90° RP-SMA plugs and their cables.
# - -Y: the 4S1P battery holder (78 mm) under the ZED.
# - Depth: the ZED Box Mini (61.6 over the panel) plus a 5.4 mm gap for its top fan intake.
DEPTH = 67.0  # panel inner face to the walls' front edge
X_IN = (-112.0, 97.0)  # inside faces of the -X and +X walls
Y_IN = (-183.0, 26.0)  # inside faces of the bottom and top walls
X_OUT = (X_IN[0] - WALL_T, X_IN[1] + WALL_T)
Y_OUT = (Y_IN[0] - WALL_T, Y_IN[1] + WALL_T)
WALL_EDGE_Z = PANEL_Z1 + DEPTH  # walls' front edge = lid's underside
CORNER_R = 4.0  # outside vertical corners; inside ones are CORNER_R - WALL_T
RIB_H = 6.0  # panel ribs, under the ZED Box Mini's 8 mm bosses
RIB_T = 2.4
HR10_PANEL_T = 1.8  # spot face around the HR10: catalog panel 0.7 to 2.0

# Cups beside the tray (cup_arm.py). The +X cup sits between the camera and its shield
# (y <= -108 below Z 70) and the RP-SMA antennas (y >= -30 from Z 55); the -X cup matches it.
# The cup's axis is CUP_OUT past the side wall's outside face: the arm's beam ends 2 mm short
# of the cup's rail, and each release tab, on the cup's axis line, sits clear of the antennas.
CUP_Y = -70.0
CUP_OUT = 45.0
CUP_X = (X_OUT[0] - CUP_OUT, X_OUT[1] + CUP_OUT)  # -X cup, +X cup
CUP_ROT = (0.0, 180.0)  # cup model's pump is on -X; these point it -X and +X
ARM_W = 40.0  # along Y
ARM_PLATE_T = 8.0  # on the cup's rail, Z = CUP_H up
ARM_TONGUE_T = 10.0  # under the panel, Z = CUP_H down
ARM_TONGUE = 36.0  # under the panel, from the side wall's outside face
ARM_SCREW_IN = 28.0  # panel screws in from the side wall's outside face
ARM_SCREW_DY = 12.0  # panel screws at CUP_Y +- this
ARM_SPOT = (27.0, 3.0)  # washer spot face: Ø, depth; 5.0 left under the washer
ARM_SEAT_Z = CUP_H + ARM_PLATE_T - ARM_SPOT[1]  # cup washer underside
ARM_CBORE = (6.5, 3.5)  # M3 head counterbore from the tongue's underside: Ø, depth
ARM_BOSS_H = 6.0  # panel bosses over the arm inserts; under the ZED Box Mini's 8 mm bosses


def arm_screws() -> list[tuple[float, float]]:
    """(x, y) of the four M3 screws that hold the arms to the panel, -X arm first."""
    xs = (X_OUT[0] + ARM_SCREW_IN, X_OUT[1] - ARM_SCREW_IN)
    return [(x, CUP_Y + s * ARM_SCREW_DY) for x in xs for s in (-1, 1)]


assert X_OUT[1] - X_OUT[0] <= BED - 2 * BED_MARGIN, "tray too wide for the P1S"
assert Y_OUT[1] - Y_OUT[0] <= BED - 2 * BED_MARGIN, "tray too tall for the P1S"

# Lid: flat 2.4 mm PETG plate on the walls' edges, printed outside face down, with a
# locating and light-sealing lip inside the walls; six M3 screws down into heat-set inserts
# in bosses hung from the walls' tops.
LID_T = 2.4
LID_LIP = 5.0  # lip depth below the walls' edge
LID_LIP_T = 1.6
LID_GAP = 0.3  # lip to the walls' inside faces
LID_BOSS = 9.0  # boss block, along and out from the wall
LID_BOSS_DEPTH = 10.0  # full-width part below the edge; a 50° gusset carries it below that
WALL_Z_MAX = WALL_EDGE_Z - LID_LIP - 1.0  # ports and vents stay below the lid's lip

# Printed bosses on the panel with M3 heat-set inserts (McMaster 94180A333, 6.4 long).
BOSS_H = 8.0  # clears the 3/8-16 button head + washer (6.8) under the ZED
BOSS_D = 8.0  # fasteners.md: M3 insert needs >= 7.0
INSERT_HOLE = 4.0  # 94180A333 in plastic
INSERT_LEN = 6.4
INSERT_DEPTH = INSERT_LEN + 1.0  # fasteners.md: 1 mm deeper than the insert

ZED_XC = 0.0
ZED_YC = -50.0
ZED_DZ = PANEL_Z1 + BOSS_H - ZED_STEP_BASE_Z
ZED_EAR_HOLES = [(ZED_XC + sx * 24.0, ZED_YC + sy * 47.3) for sx in (-1, 1) for sy in (-1, 1)]
ZED_Y0, ZED_Y1 = ZED_YC - 51.8, ZED_YC + 51.98
ZED_X0, ZED_X1 = ZED_XC - 71.9, ZED_XC + 71.5  # body; FAKRA jacks reach +81.25
ZED_TOP = 65.6 + ZED_DZ


def zed(x: float, y: float, z: float) -> tuple[float, float, float]:
    """ZED Box Mini STEP point -> this frame."""
    return (ZED_XC + x, ZED_YC + y, z + ZED_DZ)


ZED_RJ45 = zed(-71.5, 14.01, 28.22)  # jack mouth on the -X face
ZED_POWER = zed(-71.9, -23.96, 25.11)  # TBP header mouth; plug mates along +X
ZED_USB_A = zed(71.5, -11.60, 24.53)
ZED_FAKRA_A = zed(81.25, 6.85, 24.37)  # jack tip, port A
ZED_SMA = [zed(x, 51.98, 30.0) for x in (-49.5, -29.5)]  # WiFi jacks on the +Y face

# Service wall (-X) ports: (y, z) centres on the wall.
PORT_Z = 88.0
PORTS = {
    "hr10": (-32.0, PORT_Z),
    "gx12": (-92.0, PORT_Z),
}
# Power button: CIT AH1LBBG12 (19 mm anti-vandal, latching, black, green ring) through the
# bottom wall (-Y), in the lower +X corner beside the PCBA. Signal only: it tells the PCBA
# to start or stop the OpenUPS. Dimensions from CIT's AH datasheet (Rev J), latching,
# quick-connect version; the nut's thickness is not given (3 assumed).
SWITCH_X, SWITCH_Z = 72.0, 84.0
SWITCH_HOLE = 19.3  # datasheet cut-out 19.00, +0.3 for a printed hole (fdm.md)
SWITCH_BEZEL = (21.8, 2.0)  # front bezel, outside the wall: diameter, thickness
SWITCH_CAP = 15.4  # actuator (button) diameter
SWITCH_DEPTH = 29.8  # behind the bezel to the tab ends: 31.8 overall less the 2.0 bezel
SWITCH_NUT = (23.4, 3.0)  # across corners, thickness (assumed)
# RJ45 panel jack (ModDIY CO905) through the bottom wall beside the button: (x, z) of its
# body's centre. Its 37.3 flange ends 12 short of the button's bezel; the body (22 tall)
# stays under the display bracket (z 105.6) and its 33 mm inside clears the PCBA's -Y edge.
RJ45_AT = (30.0, 93.0)

# Antenna wall (+X): RP-SMA bulkheads, (y, z).
# Each folds its whip up (+Y), so they stack in Z where they share a Y.
ANTENNAS = {"zed_wifi_1": (8.0, 100.0), "zed_wifi_2": (8.0, ZED_SMA[1][2]), "esp32": (-25.0, 60.0)}

# Battery: four Molicel P30B in an MPD BK-18650-PC8 holder (no spot welding), cells along Y,
# soldered to a bare carrier PCB that puts them in series and carries the battery and
# balance pads. Carrier on a foam pad on the panel, under the OpenUPS; the OpenUPS's lower
# standoffs pin a clamp plate (battery_clamp.py) onto the holder's top rim.
# BK-18650-PC8 drawing (rev N, 2024-08-21) and MPD's STEP: 77.98 along the cells, 78.84
# across, 21.54 tall above the board, cells at ±9.65 and ±28.95 across, 3.2 mm PC pins.
HOLDER_STEP = VENDOR / "mpd" / "BK-18650-PC8.step"
HOLDER = (78.84, 77.98, 21.54)  # X (across cells), Y (along cells), Z
HOLDER_PIN = 3.2  # PC pin length below the holder; clipped to 1.0 below the carrier
CELL_XS = (-28.95, -9.65, 9.65, 28.95)  # from the holder's centre
CELL_D, CELL_L = 18.4, 65.0  # P30B, flat top
CELL_AXIS_DZ = 1.6 + CELL_D / 2  # holder base -> cell axis: cradle floor 1.6 under each cell
CARRIER_T = 1.6
CARRIER = (80.0, 80.0)  # X, Y under the holder
# Pad tab off the +X edge, between the OpenUPS bosses and clear of the lower CP10 ring:
# B-, B1, B2, B3, B+ solder pads for the battery lead and the OpenUPS J6 balance cable.
CARRIER_TAB = ((-18.0, -137.0), (-5.0, -124.0))  # (x0, y0), (x1, y1); 0.8 off the CP10 ring
FOAM = 1.5  # pad under the carrier
UPS_XC, UPS_YC = -58.0, -143.0
BRICK_XC, BRICK_YC = UPS_XC, -142.0
BRICK_Z0 = PANEL_Z1 + FOAM  # carrier underside
HOLDER_Z0 = BRICK_Z0 + CARRIER_T
BRICK = (CARRIER[0], CARRIER[1], CARRIER_T + HOLDER[2])  # X, Y, Z envelope
UPS_STANDOFF_LO = 25.0  # panel -> clamp plate: printed bosses with inserts
CLAMP_T = 3.0  # printed PETG plate
UPS_STANDOFF_HI = 6.0  # clamp plate -> OpenUPS: tubes printed on the clamp
CLAMP_Z0 = PANEL_Z1 + UPS_STANDOFF_LO
UPS_Z0 = CLAMP_Z0 + CLAMP_T + UPS_STANDOFF_HI  # board underside


def ups(x: float, y: float, z: float) -> tuple[float, float, float]:
    """OpenUPS STEP point -> this frame (board plane is the STEP's XZ, components +y)."""
    return (UPS_XC - (x - 18.02), UPS_YC + (z - 102.5), UPS_Z0 + (y - 60.54))


UPS_HOLES = [ups(x, 60.54, z)[:2] for x in (-27.98, 64.02) for z in (79.0, 126.0)]

PCBA_X0, PCBA_X1 = 12.0, 56.0
PCBA_Y0, PCBA_Y1 = -150.0, -104.0  # +Y edge 2.2 short of the ZED Box Mini's body
PCBA_Z = PANEL_Z1 + BOSS_H
PCBA_T = 1.6

# Touch display: Waveshare 2.8inch Capacitive Touch LCD (SKU 27579), in the lid, landscape,
# glass on the lid's underside, its GH1.25 13-pin end toward -X. Waveshare's STEP and 2D
# drawing: cover glass 50.54 x 73.06 with R2 corners, 10.0 from the glass front to the
# standoff ends; touch/visible area 58.2 x 43.8 centred on the STEP origin, which sits 3.0
# off the glass's centre toward the end away from the connector; four M2.5 threaded
# standoffs (Ø5.5) on 41 x 60, 4.0 proud of the PCB. STEP frame: glass front at z = 0.7,
# standoff ends at z = -9.3, connector end toward -y.
DISP_STEP = VENDOR / "waveshare" / "2_8inch-capacitive-touch-lcd.stp"
DISP_X, DISP_Y = 53.5, -140.0  # touch area centre: glass x 14.0..87.0, y -165.3..-114.7
DISP_GLASS = (73.06, 50.54)  # X, Y in this frame
DISP_GLASS_X = (-39.53, 33.53)  # glass ends from the touch area centre, connector end first
DISP_VA = (58.2, 43.8)  # touch and visible area, X, Y
DISP_DEPTH = 10.0  # glass front to standoff ends
DISP_FRONT_Z = WALL_EDGE_Z  # glass front on the lid's underside
DISP_BACK_Z = DISP_FRONT_Z - DISP_DEPTH
DISP_STANDOFFS = [(x, y) for x in (-32.9, 27.1) for y in (-20.5, 20.5)]  # from DISP_X, DISP_Y
DISP_EARS = [(x, y) for x in (-13.0, 13.0) for y in (-31.5, 31.5)]  # bracket screws, same
DISP_BRACKET_T = 2.6  # M2.5 x 6 then engages 3.3 of the standoffs (1.25 D brass: 3.12)
DISP_GH = (-29.5, -0.55)  # the module's GH1.25 13-pin (SM13B), centre; mouth faces -X
DISP_GH_MOUTH_X = -32.42  # its mating face, from DISP_X

# Camera on the +X wall's outside. The ball head's base sits on the shield's spacer boss,
# its axis along +X at (CAM_Y, CAM_AXIS_Z); a 1/4-20 screw from inside the tray holds both.
CAM_Y = -150.0
CAM_AXIS_Z = 58.0  # the head screw clears the panel; the aim sweep clears the glass by 12
CAM_SPIN = 240.0  # platform spin (knob toward +Z, tilt 0): lens into the glass, 30° down
SHIELD_T = 3.0  # shield flange, between the wall and the head
CAM_SPACER = 9.0  # boss on the flange: the head clears the wall at 90° into its notch
HEAD_SEAT_X = X_OUT[1] + SHIELD_T + CAM_SPACER  # ball head's base face
CAM_PAD = (30.0, 4.0, 106.0)  # inside pad on the +X wall: Y width, X depth, top Z
SHIELD_SCREW_Z = 100.0  # M3 anti-rotation screw through flange and wall into an insert
GMSL_HOLE = ZED_FAKRA_A[1:]  # (y, z): the FAKRA plug passes straight out through the +X wall
# Shield hood over the camera's back (+Z): its underside above the highest point the camera
# and its FAKRA plug reach anywhere in the head's range (camera_range.py). It covers the
# camera at the default aim; ribs on its top face stiffen it, out of the camera's way.
SWEEP_Z_MAX = 124.0  # camera_range.py's swept envelope, rounded up
SHIELD_HOOD_Z0 = SWEEP_Z_MAX + 3.0  # hood underside
SHIELD_Y = (-198.0, -108.0)  # hood's Y extent: the camera, bracket and front mount + 10
SHIELD_REACH = 86.0  # hood beyond the head's seat face: past the bracket's front + 10
SHIELD_RIB = 8.0  # rib height on the hood's top face

# Glass (NHRL wiki, Cage Design page, 3 lb cage): two 1/4" polycarbonate sheets with a
# 1" air gap, each wall one 4' x 8' sheet, outer sheet held by steel plates bolted to the
# welded 1" tube frame.
GLASS_T = 6.35
GLASS_GAP = 25.4
# Where the pack sits on the outer sheet: the frame origin is this far below the sheet's
# supported top edge (retainer plate ~50 mm + clearance), so the cups' centres are
# CUP_BELOW_EDGE - CUP_Y below it. Assumed.
CUP_BELOW_EDGE = 130.0
