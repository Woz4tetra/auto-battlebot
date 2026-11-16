# Mr Stabs Mk2 CAD

These notes were moved out of source comments and docstrings in `hardware/` at commit
`23e253c1`. Names, line numbers and numbers are as they were in the code then.

## BOOT/RST sleeve

### `mr_stabs_mk2_boot_sleeve/mr_stabs_mk2_boot_sleeve.py`

Docstring at line 1:

```text
Mr Stabs Mk2 BOOT/RST sleeve: insulating cap over the H1 1x3 jumper header (IO0 / GND / EN).

The header stands on the board's wedge face on the left ear, at board frame (-21.0, -9.0)
(pcba/gen_placement.py). A loose wire, a screw or swarf across GND and EN resets the ESP32 mid
match, and across IO0 and GND at power-up it boots into the ROM loader. The cap pushes onto the
header's plastic body, covers all three pins, and pulls off by hand to fit the jumper.

Spec
- Process: FDM PETG, closed end on the bed, opening up. No supports.
- Frame: origin at H1's footprint centre on the board's wedge face. +Z points from the pin tips
  toward the board, pins run along X. In the board frame (../mr_stabs_mk2_pcb) the part sits at
  Pos(-21.0, -9.0, -1.6), no rotation.
- Interfaces:
  - Header body (HX PZ2.54-1x3P-TP-YQ STEP): 7.62 x 2.50, from 0.70 to 3.20 off the board.
    Pocket at +0.10 per axis grips its top 2.0 mm (press, fdm.md; FDM holes print 0.1 to 0.2
    under, so this lands at a light interference).
  - Pins: 0.64 square (0.91 across corners), 6.0 past the body, tips 9.20 off the board. Each
    gets its own Ø1.30 blind hole, 0.30 past the tip, so a pin never touches another pin's hole.
  - SMD leads bend out to y ±2.45 within 0.70 of the board. The skirt stops 1.20 off the
    board: 0.50 above them.
  - Chassis (pcba/cad/stl/chassis.stl, board frame): one guide face runs parallel to the
    board's install direction (y = -4.337 + 0.3478 z, 19.2 deg off Z). It is the front of the
    wall at x > -16.3 and the slope of the lip under the ear, whose tip is at y -6.268,
    z -5.552. The rear-inboard corner is relieved 0.3 off that face, and the rear face below
    the tip steps in to 0.3 off it. Cut parallel to the install direction, so the sleeve stays
    0.3 clear while the board slides in.
- Install: push the sleeve on before the board goes in. With the board seated the guide face
  overhangs the pins' axis, so the sleeve cannot come off or go on along it.
- Model: features() (recipe.py), so the checked solid and the Onshape Part Studio are the
  same 10 features. The relief is two Right-plane polygon cuts.
- Critical dimensions: pocket 7.72 x 2.60 x 2.0 deep; 3 x Ø1.30 holes at x = -2.54, 0, 2.54;
  walls 1.5 (1.2 fdm.md minimum after the 0.3 lead-in); outside 10.72 x 5.60 x 9.50.

Open questions
- Grip is friction on the body only. If it walks off in a match, print with pocket_clear 0.0,
  or add a dab of hot glue on one end.
- PETG assumed. TPU would grip harder but holds Ø1.3 holes poorly.
```

## BOOT/RST sleeve dimension sources

### `mr_stabs_mk2_boot_sleeve/mr_stabs_mk2_boot_sleeve.py`

Line 61, `body_x: float = 7.62`:

```text
header STEP
header STEP
body top off the board, header STEP
pin tip off the board, header STEP
header STEP
press, fdm.md
body length inside the pocket
0.91 pin diagonal + free clearance (0.4), fdm.md
hole floor past the pin tip
closed end, minimum wall (fdm.md)
1.2 minimum (fdm.md) plus the 0.3 mouth chamfer
vertical edges
elephant's foot, fdm.md
lead-in on the pocket
Placement and chassis, board frame (../mr_stabs_mk2_pcb.py)
H1, pcba/gen_placement.py
H1, pcba/gen_placement.py
wedge face at z -1.6
chassis guide face y at z 0, chassis.stl
dy/dz, = INSTALL_DIR in ../mr_stabs_mk2_pcb.py
guide wall's outboard face, chassis.stl
chassis.stl
chassis.stl
FDM tolerance on a 0.6 mm-class print, fdm.md
```

## Main PCB fit study

### `mr_stabs_mk2_pcb/mr_stabs_mk2_pcb.py`

Docstring at line 1:

```text
Mr Stabs Mk2 main PCB placeholder: the largest board that fits the chassis electronics bay.

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
```

## Chassis STEP measurements

### `mr_stabs_mk2_pcb/mr_stabs_mk2_pcb.py`

Line 102, `above TILT_DEG = math.degrees(math.atan2(0.16655509, 0.98603215))`:

```text
Hole axes in the chassis STEP lean (0, -0.16656, 0.98603): 9.588 deg about X.
```

Line 104, `above CHASSIS_SHIFT = (0.0, 7.4276, -10.314653658)`:

```text
Hole-pattern center on the boss faces, in the tilted frame (chassis STEP, measured).
```

Line 117, `wings: bool = True`:

```text
user: use the ESC pockets, as E3 did
user
side walls at ±13.0 (chassis STEP), 0.3 gap
rear wall at 15.0 (chassis STEP), 0.3 gap
front wall -20.62 (chassis STEP), 0.3 gap; wire bay beyond
Ear edges, measured by sweeping each edge along INSTALL_DIR (chassis STEP), 0.3 gap:
ledge under the board edge at y -4.89
motor clamp boss starts at x 35.5
clamp boss at board level reaches y -10.71
outer wall at x 46.92 along the install path
```

Line 128, `26.5`:

```text
wing front wall starts at x 27.0; the 0.5 inside fillet needs the extra 0.2
```

Line 130, `ear_front_y: float = -17.37`:

```text
wing front wall lip, y -17.67 along the install path
1 mm router bit
rear corners pass the wall steps on the way in
boss pilot holes (chassis STEP)
#6 close clearance, fasteners.md
assumed, user's printed countersunk washer
assumed; puts the 3/8 screw tip 0.1 short of the pilot floor
wall below the board at |x| 13.3 (chassis STEP), 0.3 gap
#6 x 3/8 flat head plastite, overall (assembly STEP)
assembly STEP
#6 plastite major, approximate
```

## Install path

### `mr_stabs_mk2_pcb/mr_stabs_mk2_pcb.py`

Line 106, `above INSTALL_DIR = (0.0, -0.328457, -0.944519)`:

```text
The board goes in through the wedge-plate opening along the plate's normal, 19.2 deg off
the board's own axis (wedge plate STEP). Straight along -Z, the rear cross member under the
stem's last 8.4 mm and the rear screws' driver path are both blocked.
```

## Keep-out sizes

### `mr_stabs_mk2_pcb/mr_stabs_mk2_pcb.py`

Line 193, `("crossfire_nano_rx", "top", 0.0, 0.0, 18.0, 11.0, 3.0),`:

```text
TBS: 18 x 11, height assumed
LGA-28 5.2 x 3.8 + 32 kHz crystal + caps
16-18 AWG solder pads
```

Line 199, `("esp32_s3", "bottom", 0.0, 0.0, 10.0, 10.0, 1.2),`:

```text
QFN-56 7 x 7 + flash + crystal
2.4 GHz chip antenna, rear edge
2512, 1 mohm
VSSOP-10 3 x 3 + caps
TPS54202 + inductor + AP2112K
left/right DShot + GND
```

Line 206, `("neopixel", "bottom", -7.3, 2.0, 3.5, 3.5, 1.0),`:

```text
SK6812 3535 status LED
USB D+/D-/5V/GND pigtail pads
```

Line 211, `("crossfire_nano_rx", "top", 0.0, 0.0, 18.0, 11.0, 3.0),`:

```text
TBS: 18 x 11, height assumed
LGA-28 5.2 x 3.8 + 32 kHz crystal + caps
16-18 AWG solder pads
```

Line 217, `("esp32_s3", "top", -27.5, -11.28, 13.0, 11.5, 1.2),`:

```text
QFN-56 + flash + crystal + RF match
2.4 GHz chip antenna, no copper past x -40.5
TPS54202 + inductor + caps + AP2112K
2512, 1 mohm
VSSOP-10 3 x 3 + caps
SK6812 3535 status LED
Molex 505567 vertical, mated
same part, USB pigtail
```

## Board mount: plastite pilots, expected interference

### `mr_stabs_mk2_pcb/mr_stabs_mk2_pcb.py`

Line 342, `above "allow_interference": [f"screw_{i}" for i in range(4)],`:

```text
The plastite threads form in the TPU pilot holes, so the screws overlap the chassis.
```

## Render models

### `mr_stabs_mk2_pcb/pcba/art/models/fnr4030_inductor.py`

Docstring at line 1:

```text
FNR4030S100MT 10 uH shielded power inductor (L1, LCSC C167879): render model only.

KiCad's library has no 3D model for Inductor_SMD:L_Taiyo-Yuden_NR-40xx, so the board render showed
bare pads. Origin at the footprint origin on the board surface, +Z up from the board.

Spec
- Process: none (render proxy, not manufactured).
- Interfaces: sits on the footprint's two pads, centred.
- Critical dimensions: body 4.0 x 4.0, 3.0 tall (LCSC package "SMD,4x4mm"; 4030 = 4.0 x 3.0 max).

Open questions
- Height is the series maximum, 3.0 mm; the roof headroom over it is 0.78 mm.
```

### `mr_stabs_mk2_pcb/pcba/art/models/nano_rx.py`

Docstring at line 1:

```text
TBS Crossfire Nano RX standing on the board's 1x4 header (H2): render model only.

Origin at the header footprint origin on the board's wedge face, +Z away from the board, the pin
row along X. The RX's front (2.54 mm) pad row slides over the header pins and rests on the
header's 2.5 mm plastic, so the module stands 18 mm tall above it, its 11 mm edge along X.

Spec
- Process: none (render proxy, not manufactured).
- Interfaces: 4 pads on the RX's short edge over the header pins
  (TBS quickstart: GND, 5V, Ch1, Ch2).
- Critical dimensions: 18 x 11 module (TBS), 1.0 mm board, components 1.2 mm on one face,
  base 2.5 mm above the board (header plastic, HX PZ2.54 datasheet).

Open questions
- Board thickness and component height are estimates; TBS gives only 18 x 11.
```

### `mr_stabs_mk2_pcb/pcba/art/models/ra_header_1x4.py`

Docstring at line 1:

```text
HX PZ2.54-1x4P WT right-angle SMD pin header (H2, LCSC C46061677): render model only.

EasyEDA has no 3D model for it. Origin at the footprint origin on the board surface, footprint
x along the pin row, model -Y the direction the mating pins point (footprint-local +y, y down),
+Z up from the board.

Spec
- Process: none (render proxy, not manufactured).
- Interfaces: four 1.02 x 3.20 mm SMD feet at 2.54 mm pitch (footprint pads at y = 0).
- Critical dimensions (Hanxia drawing, C46061677): 0.64 mm square pins, 2.54 mm pitch;
  insulator 2.5 x 2.5 mm, length B - 0.2 = 9.96 mm; mating pins 6.00 mm past the insulator;
  tails 4.80 mm from the insulator to the end of the foot.

Open questions
- The insulator's offset from the pads (3.6 mm, from EasyEDA's courtyard) is not dimensioned on
  the drawing; pins at 1.25 mm, the insulator's centre, assume it sits on the board.
```

Line 30, `pitch: float = 2.54`:

```text
drawing
drawing, square
insulator section, drawing
B - 0.2 for 4 pins, drawing table
insulator's near face from the pads, EasyEDA courtyard
mating pin past the insulator, drawing
foot end: tail 4.8 mm from the insulator (drawing)
foot thickness on the pad
```

### `mr_stabs_mk2_pcb/pcba/art/models/ufl_plug.py`

Docstring at line 1:

```text
Mated U.FL plug with its 1.13 mm cable for the ESP32-S3-MINI-1U antenna: render model only.

Origin at the module's U.FL socket centre on the host board, +Z up. The Molex 146153-0050
lead runs forward (-Y) over the module's shield, between the front bosses and past the board's
front edge into the wire bay. It is the only exit on the roof face: the bosses close the stem's
front corners and the rear cross member sits 0.1 mm above the board.

Spec
- Process: none (render proxy, not manufactured).
- Interfaces: the plug sits on the module's socket, about 0.8 mm above the host board (module PCB).
- Critical dimensions: plug cap 2.0 mm across; mated height 2.5 mm max (Hirose U.FL series),
  so its top is at 3.3 mm, inside the 3.78 mm roof headroom. The 1.13 mm cable over the 2.55 mm
  shield tops out at 3.68 mm: 0.10 mm to the roof.

Open questions
- The socket's height on the module board (0.8 mm) is an estimate from the module drawing's
  2.55 mm shield height.
- 0.10 mm over the shield is not a working clearance; a 0.81 mm cable antenna leaves 0.42 mm.
```

Line 31, `socket_z: float = 0.8`:

```text
module PCB top above the host board, estimate
Hirose U.FL mated height, max
plug cap
Molex 146153 cable
ESP32-S3-MINI-1U module height
socket at board y +5.1 to 3 mm past the front edge at y -20.3
```

## Silkscreen art

### `mr_stabs_mk2_pcb/pcba/art/render_robot.py`

Docstring at line 1:

```text
Line-art render of the Mr Stabs Mk2 CAD for the silkscreen icon.

Software z-buffer: rasterize the assembly's triangles into depth and normal images, then mark
pixels where depth jumps (silhouettes) or the normal turns sharply (creases). Writes
robot_lines.png (white lines on black) for silk.py to vectorize.

    ~/.local/share/pcba-design/venv/bin/python art/render_robot.py [--preview]
```

### `mr_stabs_mk2_pcb/pcba/art/vectorize.py`

Docstring at line 1:

```text
Turn the silkscreen images into polygons in mm: the Mk2 line renders and the BWBots logo.

    ~/.local/share/pcba-design/venv/bin/python art/vectorize.py

Inputs:
    robot: render_robot.py's renderer, re-run here at board resolution (view az 145, el 28),
        once from below and once from above.
    logo: logo_dark_h240.png from logo/gen_logo_icon.py --height 240 --out art/logo_dark_h240.png,
        the dashboard's dark-background variant (black half-disc drawn as an outline).

Writes robot_silk.json, robot_top_silk.json and logo_silk.json:
{"size": [w, h], "polys": [[exterior, hole, ...]]},
points in mm with the origin at the image centre, u right and v up as the image reads. silk.py
draws robot_top_silk.json on F.SilkS and the others on B.SilkS, mirrored so they read from below.
```

Line 29, `MIN_LINE = 0.18`:

```text
mm; JLCPCB silk minimum is 0.15
```

## Keep-outs

### `mr_stabs_mk2_pcb/pcba/cad/make_keepout.py`

Docstring at line 1:

```text
Write cad/keepout.json: what each board face must clear, in the placeholder's board frame.

Frame (../../mr_stabs_mk2_pcb.py): origin at the hole-pattern centre on the roof face, +Z into the
chassis, +Y toward the rear wall. KiCad F side is the roof face (z 0 up), B side the wedge face
(z -1.6 down). cad/stl/ holds the robot parts already moved into that frame (export from the
placeholder's robot_in_board_frame() and esc_shapes()).

    ~/.local/share/pcba-design/venv/bin/python cad/make_keepout.py

Keys: "F" union of every section from z 0.1 to 3.78 (roof face, low parts only);
"B_low" sections from z -1.7 to -4 (anything on the wedge face: ESCs, the ledge ridge,
washers); "B_tall" sections down to -12 (USB-C body 8.5 mm, headers 8.5 mm plus solder).
```
