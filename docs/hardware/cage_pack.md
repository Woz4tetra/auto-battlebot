# Cage pack

These notes were moved out of source comments and docstrings in `hardware/` at commit
`23e253c1`. Names, line numbers and numbers are as they were in the code then.

## Battery clamp section

### `cage_pack/battery_clamp.py`

Docstring at line 1:

```text
Battery clamp: a printed PETG plate that holds the battery holder and cells down and
carries the OpenUPS on four standoff tubes.

The plate sits on the tray's four 25 mm bosses. Four M3 x 16 screws go down through the
OpenUPS, the tubes and the plate into the bosses' heat-set inserts, so one set of screws
holds the board, the plate and the battery. Taking the OpenUPS off frees the plate and the
cells.

Spec
- Process: FDM, black PETG, printed plate-down with the tubes rising from it (the build
  frame is the print frame). PROCESS = "fdm".
- Frame: cage_layout.py, underside at Z = CLAMP_Z0.
- Holes: 4 x Ø3.4 through plate and tubes on the OpenUPS's 92 x 47 pattern
  (cage_layout.UPS_HOLES).
- Size: the hole pattern + 6 mm margin, 5 mm corners, 3 mm thick. Tubes Ø8 x 6 tall. The
  battery holder (78.84 wide) sits between the boss columns. The plate clears the holder's
  walls by 0.4 and the cells by 2.1; a 2 mm foam strip stuck under the plate holds the
  cells' middle 59 mm down in their cradles.
- Model: features() (recipe.py), so the checked solid and the Onshape Part Studio are the
  same four features: plate sketch and plate, tube sketch and tubes.
```

## Frame and stack-up

### `cage_pack/cage_layout.py`

Docstring at line 1:

```text
Shared layout for the cage pack: a printed enclosure on two SmallRig 4122C suction cups
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
```

## Dimension sources

### `cage_pack/cage_layout.py`

Line 38, `above CP10_PITCH = 153.5`:

```text
SmallRig 4122C (B&H page + dimension photo in ~/Downloads, 2026-11-07)
rubber face, dimension photo
overall width, B&H spec 139.7 x 106.7 x 50.8
glass to the top of the NATO rail, dimension photo
pump knob tip from the cup axis
release tabs on the rim, each side of the pump; photos, +-20
3/8-16 button head: diameter, height, fasteners.md
McMaster 92141A051 under the head: OD, thickness
The tray's old cup holes stay as a tripod mount: 153.5 apart on X = 0 matches the INNOREL
CP10 cheese plate's 3/8-16 pair (zed_box_mini_mount/innorel_cp10.py).
```

Line 77, `HR10_PANEL_T = 1.8`:

```text
spot face around the HR10: catalog panel 0.7 to 2.0
```

### `cage_pack/cage_pack_assembly.py`

Line 109, `above UPS_TOP = cl.UPS_Z0 + 1.6`:

```text
OpenUPS connectors (STEP solid centres, inspect_step.py), on the board's top face.
```

Line 131, `above FAKRA_PLUG_LEN = 26.0`:

```text
FAKRA plug (Rosenberger 59Z114-000Z): 26 long from its nose, which sits FAKRA_ENGAGE back
from the jack's tip, so the cable leaves 26 - FAKRA_ENGAGE past the tip.
```

Line 135, `above FAKRA_CABLE_IN = 6.0`:

```text
The STEP is the housing alone, bored Ø6.70 through; the cable's crimp ferrule sits this far
inside its back end.
```

Line 138, `above CAM_FAKRA_TIP = (3.30, -42.72, -23.75)`:

```text
ZED X One S FAKRA jack in the camera STEP's frame (inspect_step.py, solid 6): axis along
the STEP's +Y (away from the lens) at (x, z) = (3.30, -23.75), 2.8 and 6.1 off the lens
axis, tip at y = -42.72, 13.3 behind the body's back face.
```

Line 384, `above UPS_J10 = cl.ups(6.2, 0.0, 77.5)[:2]`:

```text
OpenUPS J10 (motherboard on/off) and J8 (start/stop) 2-pin headers, located from Mini-Box's
connection diagram against the J5 and J7 connectors in the STEP (about +-1 mm).
```

### `cage_pack/connectors.py`

Docstring at line 1:

```text
Cable connector envelopes for the cage pack assembly, traced from datasheet drawings.

Exact vendor models are used wherever one could be downloaded (vendor/: KiCad's 3D library
for the PCBA parts, Mini-Box's OpenUPS STEP, Stereolabs' ZED Box Mini and ZED X One S). The
parts below have no free model, so each is an envelope from its datasheet's drawing, close
enough for clearance and cable routing, not for mating geometry.

Every function returns a part in a local frame: origin on the mating plane at the contact
axis, +Z pointing away from the mating partner (out of a panel, up off a header, back along
the cable). `place(shape, origin, z_dir, x_dir)` puts one in the assembly.

| Part | Source |
| --- | --- |
| GX16-8 panel receptacle, cable plug | generic GX16 drawing (M16 x 1, flange Ø19, plug Ø19 x 45) |
| GX12-2 panel receptacle, cable plug | generic GX12 drawing (assembly uses vendor/grabcad) |
| CIT AH power button | CIT AH datasheet, latching (bezel Ø21.8 x 2.0, M19 body, 31.8 deep) |
| Same Sky TBP01P1W-508-02GR | datasheet p.2: 10.16 wide, 15.00 tall, 18.00 deep, flanges 15.24 |
| Mini-Fit Jr receptacle housings | 4.2 pitch, 9.6 x 9.6 (2x2), 9.6 x 5.4 (1x2), 13.6 tall |
| JST PH / GH housings | JST catalog outlines |
| USB plugs (A 90°, C 90°, mini-B 90°) | generic overmolds |
| FAKRA-Z plug (Stereolabs GMSL2 cable) | generic FAKRA housing 9.6 square |
| Lever suction cup with 1/4-20 stud | generic 4.5" (114 mm) cup, e.g. Delvie's Plus |
```

### `cage_pack/hr10.py`

Docstring at line 1:

```text
Hirose HR10 4-pin envelopes: HR10A-7R-4S panel receptacle and HR10A-7P-4P cable plug.

Same catalog numbers as hardware/tx16s_hr10_mount/tx16s_hr10_mount.py (Hirose HR10 catalog
p.6, p.7, p.20). Hirose gates its STEP downloads behind a login, so these are traced from the
catalog drawings, not exact models.

Frame: origin on the panel's front face at the receptacle axis, +Z into the enclosure (the
thread and nut side), -Z out toward the mating plug.
```

### `cage_pack/pcba_cage.py`

Line 98, `above GH13V = "box:19.5x4.95x4.2"`:

```text
KiCad's 9.0 library has no BM13B model here: the outline from the BM02-06 models' rule,
1.25 n + 3.25 wide, 4.95 deep, 4.2 tall.
```

Line 104, `SOT363 = "box:2.1x2.0x1.0"`:

```text
2N7002DW dual N-FET
ESP32-S3-WROOM-1U: KiCad 9.0 has no model, so this is the datasheet outline (18.0 x 19.2 x
3.2, shield can) with the u.FL receptacle near the +Y end. Its pigtail runs to an RP-SMA
bulkhead in the enclosure's antenna wall.
```

### `cage_pack/pcba_cage.py` placement table

- `Placement(model, rot_deg, x, y)`: KiCad frames, mating face toward -Y, top +Z.
- `BoxEnvelope(w, d, h)` stands in for a KiCad model where there is none, using the datasheet
  outline: `SOP4` is the Toshiba TLP172A photoMOS and TLP293 optocoupler, SO4.
- `MATERIAL = "fr4"` (1.85 g/cm3 in the skill density table).

### `cage_pack/smallrig_4122c.py`

Docstring at line 1:

```text
SmallRig 4122C 4" suction cup camera mount: an envelope traced from the B&H page.

SmallRig publishes no CAD. Sources (saved to ~/Downloads by the user, 2026-11-07):
- B&H spec table: 139.7 x 106.7 x 50.8 mm overall, 264 g, rated 50 kg horizontal mounting
  (pull straight off the surface) and 35 kg vertical mounting (cup on a wall, load in shear).
- Dimension photo: Ø101.5 cup, 48.2 mm glass to the top of the rail.
- Close-up of the top: a NATO rail with a 3/8-16 ARRI locating hole in the middle (four
  small pin holes around it), one 1/4-20 toward the pump, and two 1/4-20 on the side
  shoulder. Only the central 3/8-16 is used here; the rest are left out of the model.
- Product photos: two release tabs stand on the rubber's rim, opposite each other, about 90°
  each side of the pump. Lifting one breaks the seal.

Frame: cup face on the glass at Z = 0, axis along +Z, rail and pump along X, pump toward -X.

Open questions
- Thread depth of the central 3/8-16: assumed 10 mm. Measure before buying screws.
- Body profile between the rubber and the rail block is traced from photos, +-3 mm.
- Release tab angle from the pump (cage_layout.CUP_TAB_ANGLE): +-20° from the photos; tab
  size 14 wide x 12 radial x 9 tall, guessed.
```

Line 28, `MASS_G = 264.0`:

```text
B&H spec
"horizontal mounting": load normal to the surface
"vertical mounting": load along the surface
3/8-16, assumed: a 9/16" screw through 4.83 of plate engages 9.5
3/8-16 tap drill (5/16"), fasteners.md
```

## Printing

### `cage_pack/cage_layout.py`

Line 51, `above PANEL_T = 3.0`:

```text
Enclosure tray: black PETG, printed on a Bambu P1S (256 x 256 x 256) with the panel's
glass-side face on the bed and the walls rising from it (dfm/fdm.md).
P1S build plate, X and Y
each side; leaves no room for a brim
```

## Tray sizing

### `cage_pack/cage_layout.py`

Line 56, `WALL_T = 2.4`:

```text
six 0.4 perimeters
back panel's glass-side face, level with the cups' rail tops
its inner face
spot face for a tripod screw's washer: Ø, depth; 1.6 under it
Inside, packed round the parts. What sets each face:
- -X: the ZED's RJ45 and TBP power plugs, their cables and the RJ45 tail running down
  beside the ZED Box Mini's -X face; also the battery, which must stay left of the lower
  CP10 rib ring.
- +X: the ZED's USB-A 90° plug (22 mm head) and the FAKRA plug, which passes through it.
- +Y: the ZED's WiFi jacks with 90° RP-SMA plugs and their cables.
- -Y: the 4S1P battery holder (78 mm) under the ZED.
- Depth: the ZED Box Mini (61.6 over the panel) plus a 5.4 mm gap for its top fan intake.
panel inner face to the walls' front edge
inside faces of the -X and +X walls
inside faces of the bottom and top walls
```

## Bosses and cup arm constants

### `cage_pack/cage_layout.py`

- Bosses on the panel take M3 heat-set inserts (McMaster 94180A333, 6.4 mm long). `INSERT_HOLE` is
  the 94180A333 hole in plastic.
- `BOSS_H = 8.0` clears the 3/8-16 button head plus washer (6.7 mm) under the ZED.
  `test_zed_bosses_clear_tripod_screw_head_and_washer` holds it.
- `BOSS_D = 8.0`: fasteners.md says an M3 insert needs at least 7.0.
- `PCBA_Y1` leaves 2.2 mm to the ZED Box Mini body; `test_pcba_clears_zed_body` requires 2.0.
- `CUP_X_NEG_POS` holds the -X and +X cup positions. `CUP_ROT_NEG_POS` rotates the cup model, whose
  pump is on -X, so the pumps point -X and +X.
- `ARM_WIDTH_Y` is the arm width along Y.
- `ARM_PLATE_T` is the plate on the cup's rail, Z = `CUP_H` up. `ARM_TONGUE_T` is the tongue under the
  panel, Z = `CUP_H` down.
- `ARM_TONGUE` is the tongue length under the panel from the side wall's outside face.
- `ARM_SCREW_IN` is where the panel screws go in, from the side wall's outside face. `ARM_SCREW_DY`
  puts them at `CUP_Y` +- that.
- `ARM_SPOT = SpotFace(27.0, 3.0)` is the washer spot face, diameter and depth, leaving 5.0 mm of the
  plate under the washer. `ARM_SEAT_Z` is the cup washer underside.
- `ARM_CBORE = SpotFace(6.5, 3.5)` is the M3 head counterbore from the tongue's underside.
- `ARM_BOSS_H = 6.0`: panel bosses over the arm inserts, under the ZED Box Mini's 8 mm bosses.

## Cup placement

### `cage_pack/cage_layout.py`

Line 79, `above CUP_Y = -70.0`:

```text
Cups beside the tray (cup_arm.py). The +X cup sits between the camera and its shield
(y <= -108 below Z 70) and the RP-SMA antennas (y >= -30 from Z 55); the -X cup matches it.
The cup's axis is CUP_OUT past the side wall's outside face: the arm's beam ends 2 mm short
of the cup's rail, and each release tab, on the cup's axis line, sits clear of the antennas.
```

## Lid

### `cage_pack/cage_layout.py`

Line 108, `above LID_T = 2.4`:

```text
Lid: flat 2.4 mm PETG plate on the walls' edges, printed outside face down, with a
locating and light-sealing lip inside the walls; six M3 screws down into heat-set inserts
in bosses hung from the walls' tops.
```

### `cage_pack/enclosure_lid.py`

Docstring at line 1:

```text
Cage pack enclosure lid: a flat black PETG plate over the tray's open front.

Spec
- Process: FDM, black PETG, printed outside face down on a Bambu P1S, the lip rising from
  it. `build()` returns it in that print frame: outside face on Z = 0, turned over about X.
  `ASSEMBLY_AT` puts it back on the tray. PROCESS = "fdm".
- Frame: cage_layout.py. Underside on the tray walls' edges (Z = WALL_EDGE_Z); outline equal
  to the tray's, so it fits the same 256 x 256 plate.
- Interfaces:
  - 6 x Ø3.4 over the tray's lid bosses (enclosure.lid_screws()), M3 x 8 button heads into
    M3 x 6.4 heat-set inserts.
  - Lip: 1.6 thick, 5 deep, 0.3 inside the walls. It locates the lid and closes the light
    path at the joint; notched 0.5 clear round each boss.
- Vents: 3 x 30 slots over the ZED Box Mini's top face, which carries its fan intake.
- Touch display (cage_layout.DISP_*): the Waveshare 2.8" module's glass sits on the
  underside, landscape, in the lower right, clear of the ZED Box Mini and the OpenUPS's
  plugs. The lid is its bezel: a window 0.4 larger than the 58.2 x 43.8 touch area each
  side, R1 corners, a 0.8 chamfer on the outside edge for a fingertip. A 1.0 tall, 1.2
  wide rim 0.3 round the glass locates it. Four Ø8 bosses hang to the display's
  standoff plane with M3 x 6.4 inserts for display_bracket.py, which clamps the glass up
  against the lid. The plate round the window is left full thickness (no pockets).
- Weight: pocketed from the inside to a 1.2 mm skin between 3 mm lands, a land round the
  lip's inside and each screw boss; print_mass.py slices it at 86 g (143 g unpocketed).
  Printed face down, so the pockets face up and need no support.
- Critical dimensions: outline 250.8 x 213.8 with R4 corners; lip outside 245.4 x 208.4;
  display bosses on cage_layout.DISP_EARS, their end faces in the standoffs' plane.
- Model: features() (recipe.py) in the print frame, so the checked solid and the Onshape
  Part Studio are the same 17 features, in print order: plate, foot chamfer, lip, pockets,
  display window and its chamfer, glass rim, display bosses and insert holes, then the
  screw holes and vents.
```

Docstring at line 72:

```text
Plate thinned to p.skin from the inside between 3 mm lands, on the tray panel's
grid except x = 4.5, which falls between two vent slots where the spine's x = 0 would
cut through one; a land runs round the inside of the lip, each screw boss and the
display.
```

## Power button

### `cage_pack/cage_layout.py`

Line 153, `above SWITCH_X, SWITCH_Z = 72.0, 84.0`:

```text
Power button: CIT AH1LBBG12 (19 mm anti-vandal, latching, black, green ring) through the
bottom wall (-Y), in the lower +X corner beside the PCBA. Signal only: it tells the PCBA
to start or stop the OpenUPS. Dimensions from CIT's AH datasheet (Rev J), latching,
quick-connect version; the nut's thickness is not given (3 assumed).
```

## Power button, RJ45

### `cage_pack/cage_layout.py`

Line 158, `SWITCH_HOLE = 19.3`:

```text
datasheet cut-out 19.00, +0.3 for a printed hole (fdm.md)
front bezel, outside the wall: diameter, thickness
actuator (button) diameter
behind the bezel to the tab ends: 31.8 overall less the 2.0 bezel
across corners, thickness (assumed)
RJ45 panel jack (ModDIY CO905) through the bottom wall beside the button: (x, z) of its
body's centre. Its 37.3 flange ends 12 short of the button's bezel; the body (22 tall)
stays under the display bracket (z 105.6) and its 33 mm inside clears the PCBA's -Y edge.
```

## Battery

### `cage_pack/cage_layout.py`

Line 172, `above HOLDER_STEP = VENDOR / "mpd" / "BK-18650-PC8.step"`:

```text
Battery: four Molicel P30B in an MPD BK-18650-PC8 holder (no spot welding), cells along Y,
soldered to a bare carrier PCB that puts them in series and carries the battery and
balance pads. Carrier on a foam pad on the panel, under the OpenUPS; the OpenUPS's lower
standoffs pin a clamp plate (battery_clamp.py) onto the holder's top rim.
BK-18650-PC8 drawing (rev N, 2024-08-21) and MPD's STEP: 77.98 along the cells, 78.84
across, 21.54 tall above the board, cells at ±9.65 and ±28.95 across, 3.2 mm PC pins.
```

Line 179, `HOLDER = (78.84, 77.98, 21.54)`:

```text
X (across cells), Y (along cells), Z
PC pin length below the holder; clipped to 1.0 below the carrier
from the holder's centre
P30B, flat top
holder base -> cell axis: cradle floor 1.6 under each cell
```

Line 185, `CARRIER = (80.0, 80.0)`:

```text
X, Y under the holder
Pad tab off the +X edge, between the OpenUPS bosses and clear of the lower CP10 ring:
B-, B1, B2, B3, B+ solder pads for the battery lead and the OpenUPS J6 balance cable.
(x0, y0), (x1, y1); 0.8 off the CP10 ring
pad under the carrier
```

## Display

### `cage_pack/cage_layout.py`

Line 215, `above DISP_STEP = VENDOR / "waveshare" / "2_8inch-capacitive-touch-lcd.stp"`:

```text
Touch display: Waveshare 2.8inch Capacitive Touch LCD (SKU 27579), in the lid, landscape,
glass on the lid's underside, its GH1.25 13-pin end toward -X. Waveshare's STEP and 2D
drawing: cover glass 50.54 x 73.06 with R2 corners, 10.0 from the glass front to the
standoff ends; touch/visible area 58.2 x 43.8 centred on the STEP origin, which sits 3.0
off the glass's centre toward the end away from the connector; four M2.5 threaded
standoffs (Ø5.5) on 41 x 60, 4.0 proud of the PCB. STEP frame: glass front at z = 0.7,
standoff ends at z = -9.3, connector end toward -y.
```

### `cage_pack/cage_layout.py` display constants

- `DISP_STANDOFFS` and `DISP_EARS` are offsets from `DISP_TOUCH_X`, `DISP_TOUCH_Y`. The ears are the
  bracket screws.
- `DISP_BRACKET_T = 2.6`: an M2.5 x 6 screw engages 3.4 mm of the brass standoff, over the 1.25 D
  (3.125 mm) minimum. `test_display_screw_engages_brass_standoff_thread` holds the bracket to it.
- `DISP_GH = (-29.5, -0.55)`: the module's GH1.25 13-pin connector (SM13B), centre; the mouth faces -X.
- `DISP_GH_MOUTH_X = -32.42`: its mating face, from `DISP_TOUCH_X`.

## Camera

### `cage_pack/cage_layout.py`

Line 236, `above CAM_Y = -150.0`:

```text
Camera on the +X wall's outside. The ball head's base sits on the shield's spacer boss,
its axis along +X at (CAM_Y, CAM_AXIS_Z); a 1/4-20 screw from inside the tray holds both.
```

### `cage_pack/cage_pack_assembly.py`

Line 98, `above HEAD = Location(`:

```text
Ball head on the shield's spacer boss outside the +X wall, axis +X; its knob (head -X)
faces +Z, toward the viewer, and its drop notch the glass. Neck straight out (tilt 0);
the platform's spin sets the aim.
```

### `cage_pack/cage_layout.py` camera, shield and hood

- `CAM_AXIS_Z = 58.0`: the head screw clears the panel, and the aim sweep clears the glass by 12 mm.
- `CAM_SPIN = 240.0`: platform spin with the knob toward +Z and tilt 0 puts the lens into the glass,
  30 degrees down.
- `SHIELD_T = 3.0`: shield flange thickness, between the wall and the head.
- `CAM_SPACER = 9.0`: boss on the flange; the head clears the wall at 90 degrees into its notch.
- `HEAD_SEAT_X`: the ball head's base face.
- `CAM_PAD = (30.0, 4.0, 106.0)`: inside pad on the +X wall, Y width, X depth, top Z.
- `SHIELD_SCREW_Z = 100.0`: M3 anti-rotation screw through the flange and wall into an insert.
- `GMSL_HOLE`: (y, z) of the FAKRA plug, which passes straight out through the +X wall.
- Hood over the camera's back (+Z): its underside sits above the highest point the camera and its
  FAKRA plug reach anywhere in the head's range (`camera_range.py`). It covers the camera at the
  default aim. Ribs on its top face stiffen it, out of the camera's way.
- `SWEEP_Z_MAX = 124.0` is the top of the swept envelope from `camera_range.py`, rounded up. `sweep("aim")` gave an envelope top
  of 114.9 mm on 2026-11-19. The hood underside is `SWEEP_Z_MAX + 3.0`, 12 mm above that.
  `test_shield_hood_above_camera_sweep` fails when the sweep rises above the constant.
- `SHIELD_Y = (-198.0, -108.0)`: hood Y extent, the camera, bracket and front mount plus 10 mm.
- `SHIELD_REACH = 86.0`: hood beyond the head's seat face, past the bracket's front plus 10 mm.
- `SHIELD_RIB = 8.0`: rib height on the hood's top face.

## Cage glass

### `cage_pack/cage_layout.py`

Line 256, `above GLASS_T = 6.35`:

```text
Glass (NHRL wiki, Cage Design page, 3 lb cage): two 1/4" polycarbonate sheets with a
1" air gap, each wall one 4' x 8' sheet, outer sheet held by steel plates bolted to the
welded 1" tube frame.
```

Line 261, `above CUP_BELOW_EDGE = 130.0`:

```text
Where the pack sits on the outer sheet: the frame origin is this far below the sheet's
supported top edge (retainer plate ~50 mm + clearance), so the cups' centres are
CUP_BELOW_EDGE - CUP_Y below it. Assumed.
```

## Overview, wiring

### `cage_pack/cage_pack_assembly.py`

Docstring at line 1:

```text
Cage pack: ZED Box Mini, battery and PCBA in a printed PETG enclosure on two SmallRig
4122C suction cups on an NHRL cage wall.

    scripts/run check_assembly.py cage_pack_assembly.py      (build123d-part skill)

The box is printed on a Bambu P1S. It shades the camera, takes a fall, and is the one part
carrying every load to the cups.

Frame: cage_layout.py. Z = 0 on the outer polycarbonate face, +Z out of the cage, +Y up.

Load path: every part is screwed to the enclosure tray (enclosure.py). Two printed arms
(cup_arm.py) screw to the panel's underside, two M3 each, and carry the cups beside the
side walls, one 3/8-16 screw each. Nothing else bears on the glass except the two cup faces.

What goes where
- Cups beside the -X and +X walls at y = -70, 45 past each wall, pump knobs pointing
  outward. Each cup's rim, and so its release tabs, is outside the tray.
- ZED Box Mini across the top on 8 mm printed bosses, over the upper CP10 tripod hole.
- Lower left: four P30B cells in an MPD BK-18650-PC8 holder on a carrier PCB that wires
  them in series (no spot welding), held by battery_clamp.py, under the OpenUPS.
- Lower middle: the custom PCBA (ESP32-S3-WROOM-1U).
- Lid, lower right: a Waveshare 2.8" capacitive touch display (SPI + I2C) for boot
  progress, faults and stats, glass on the lid's underside behind a window, clamped by
  display_bracket.py. A GH 13-pin lead runs down the -X side of it to the PCBA's J7.
- -X wall: HR10 transmitter port, GX12 DC input to the OpenUPS
  VIN, from a Mean Well GST120A24 (24 V 5 A) whose barrel plug is swapped for a GX12 plug.
- Bottom wall: latching power button (CIT AH1LBBG12), signal only, wired to the PCBA (J5),
  and beside it the RJ45 panel coupler (iPad), both reachable from below on the glass.
  The battery stays on the OpenUPS through the 10 A fuse. The PCBA (J6) presses the OpenUPS
  J8 to start or stop it and reads J10's off pulse; with OPENUPSMODE = 1, DC input charges
  the battery without starting the output, so the pack charges when off and runs and
  charges when on. See pcba_cage.py.
- +X wall: three RP-SMA bulkheads, two for the ZED's AX210 WiFi, one for the ESP32.
- Outside the +X wall: the ZED X One S on a SmallRig 2948 ball head, nothing round it but
  air, so it spins onto the head's stud and reaches the head's full range with the lid on.
  camera_shield.py hoods its back against light from behind the camera.
- Safety tether: 1/8" steel lanyard from the top wall's -X hole to the cage frame.
```

## Vendor STEP frames

### `cage_pack/cage_pack_assembly.py`

Line 153, `above HOLDER_AT = Location(`:

```text
MPD's STEP has the cells along its X, height along Y from the pin tips (holder base at
Y = 3.2), across the cells along Z. Placed with the cells along Y, base on the carrier.
```

Line 277, `above def _gx12_jack() -> Compound:`:

```text
GrabCAD "GX12 Aviation Connectors" (Nelson Stoldt), axis along the STEP's X, mating
toward -X. connectors.py frame: origin on the wall's outer face, +Z outward.
```

Docstring at line 280:

```text
GX12-2 panel jack: front shroud and flange outside, nut and solder cups inside. The
model's gasket face (x = 1.0) sits on the wall; its nut leaves a 2.0 panel gap.
```

Line 309, `above RJ45_FLANGE_BACK = -11.55`:

```text
ModDIY panel jack (GrabCAD "Panel Mount RJ45", traced from their CAB105): STEP axis X,
jack mouth toward +X, flange back face at x = -11.55, cable stub out to x = -44.5.
```

Docstring at line 316:

```text
ModDIY panel-mount RJ45 jack: 7 mm flange outside the bottom wall, 21.4 x 22 body and
the cable's moulded stub inside, two screws at 27.5 along X.
```

Docstring at line 325:

```text
Molex Mini-Fit Jr receptacle housing (Molex's STEP): mating face at its z = -10.8,
cavities centred on y = -5.0, latch +Y. connectors.minifit_plug frame: origin on the
header's base, +Z off the board, housing from 3.0 up.
```

Docstring at line 332:

```text
Same Sky TBP01P1W-508-02GR (Same Sky's STEP): mates along its +X (pin end at
x = 17.5), poles along its Z at 0 and 5.08, 15 tall along Y. connectors.tbp_plug_2p
frame: X along the poles, Y height, mating face at z = -8.
```

Docstring at line 341:

```text
Rosenberger 59Z114-000Z FAKRA Z-code plug housing (Rosenberger's STEP), 26 long,
coded nose at its z = -0.2. connectors.fakra_plug frame: +Z outward from the jack.
```

Docstring at line 349:

```text
L-com HG2402RD-RSF tilt-swivel RP-SMA antenna (L-com's STEP), plug onto the bulkhead
4 mm past the wall, whip folded 90° to +Y at its hinge (STEP x = 36.25).
```

Docstring at line 364:

```text
The extension's 90° plug in the ZED's jack: GrabCAD "RJ45_Ethernet_Plug" head (22.17
long, mouth at its z = 22.17, latch +Y), 15 in the jack with the latch up, plus the
moulded right-angle boot turning the cable +Z.
```

Docstring at line 395:

```text
Mean Well GST120A (Mean Well's GS-120 case STEP): case x 65.53 to 233.11 in the
STEP, DC cord end at +X, 67 x 35. The STEP's 100 mm cord stub is cut off at the case;
charger_cord carries on from there.
```

## Cable routing

### `cage_pack/cage_pack_assembly.py`

Line 411, `above out["usb_a_plug"] = cn.place(cn.usb_a_plug_90(), cl.ZED_USB_A, (1, 0, 0), (0, -1, 0))`:

```text
Head along +X, cable leaving -Y (away from the FAKRA plug above it).
```

## Cables

### `cage_pack/cage_pack_assembly.py`

Line 634, `above fx, fy, fz = cl.ZED_FAKRA_A`:

```text
GMSL: straight out of the ZED's FAKRA jack through the +X wall, up and over toward
the camera under the shield's hood, and into the plug on the camera's back, which
points up (+Z) and +Y at the default aim. About 90 mm of the 0.3 m cable is slack
for re-aiming.
```

### `cage_pack/cage_pack_assembly.py` cable specs

- `CABLES` rows are `CableSpec(od, min_bend_r, source)`: envelope diameter, minimum bend radius, and
  where the radius comes from. The radii are static: installed once, not flexed in service.
- `_run` places every corner at the cable's minimum bend radius. Unlike `cn.tube` it never shrinks the
  radius to fit: a corner too tight for it raises `ValueError`, so a route that builds meets the bend
  radius. `tests/python/test_cage_pack_cables.py` covers this.

## Materials for the thread check

### `cage_pack/cage_pack_assembly.py`

Line 876, `"material": "brass",`:

```text
the M2.5 standoffs' threads
```

Line 906, `"material": "al6061",`:

```text
aluminium head: 1.5 D thread rule
```

## Interference waivers

### `cage_pack/cage_pack_assembly.py`

Line 1221, `("battery_clamp", "openups"),`:

```text
board seated on the clamp's tubes
contact springs drawn uncompressed, cradle snap lips
```

## Camera range check

### `cage_pack/camera_range.py`

Docstring at line 1:

```text
Camera range check: sweeps the ZED X One S through the SmallRig 2948 ball head's whole
range and measures how close it comes to everything fixed around it.

    PYTHONPATH=$SKILL/scripts ~/.local/share/build123d-part/venv/bin/python camera_range.py

$SKILL is the build123d-part skill's directory: the part files import its recipe.py.

What moves: the camera body, the bracket, the front mount and the FAKRA plug in the
camera's back, as points sampled off their tessellated surfaces (0.5 mm tolerance), carried
by the head's pose. The GMSL cable is not swept; it hangs slack.

Three checks, from the head's range (smallrig_2948_ball_head._check_pose):
- mount: the camera and bracket spun all the way round on the stud at the default pan and
  tilt, plug off, as when screwing the camera on. Must clear.
- aim: pan and spin all the way round (15° steps), the neck tilted up to the socket's 20°
  in eight directions, plug on, keeping the poses that look into the cage: the optical
  axis within AIM_CONE of the glass normal. Must clear. (Spun to look away from the glass,
  the plug in the camera's back swings into the glass; nobody aims it there.)
- notch: the neck dropped into the notch, 15° steps to 90°, at every pan and spin, plug
  on. Reported as the deepest drop that clears at each pan; not a pass/fail check, since
  a full 90° drop folds the camera back through the wall or the glass at most pans.

Fixed: the glass, the tray and lid (one box), the shield's flange, hood and ribs, and the
three RP-SMA antennas on the +X wall (boxes round each hinge and whip). Each is a box; the
clearance is the smallest distance from any sampled point to it over all poses, negative
inside it. Exits 1 when any clearance is under MIN_CLEAR.
```

## Camera shield

### `cage_pack/camera_shield.py`

Docstring at line 1:

```text
Camera light shield: a black PETG hood over the back of the ZED X One S, which hangs off
the enclosure's +X wall on a SmallRig 2948 ball head.

Light from behind the camera reflects off the cage glass into the lens; the hood shades
the camera's back (+Z, toward the viewer) and takes a knock before the camera does. It
stays out of the camera's way: the hood's underside sits above the highest point the
camera and its FAKRA plug reach anywhere in the head's range, and its stiffening ribs are
on its top face (camera_range.py checks the sweep against keep_out()).

Spec
- Process: FDM, black PETG, printed flange-down (the flange's wall face on the bed), so the
  hood, ribs and spacer boss all rise straight from it. `build()` returns the print frame;
  `ASSEMBLY_AT` puts it on the wall. PROCESS = "fdm".
- Frame: cage_layout.py. Flange on the +X wall's outside face (X = X_OUT[1]).
- Interfaces:
  - Ball head: spacer boss Ø22 x CAM_SPACER on the flange, under the head's Ø23 base. A
    1/4-20 x 1" socket head from inside the tray goes through the wall pad, the flange and
    the boss (Ø6.53 close clearance) into the head's base thread, clamping all three.
  - Anti-rotation: M3 x 8 button head through Ø3.4 in the flange into an M3 x 6.4 insert in
    the wall at (CAM_Y, SHIELD_SCREW_Z).
- Critical dimensions: hood underside SHIELD_HOOD_Z0, 3 mm over the swept envelope; hood
  SHIELD_Y wide, to HEAD_SEAT_X + SHIELD_REACH.
- Model: features() (recipe.py) in the print frame, so the checked solid and the Onshape
  Part Studio are the same ten features: flange sketch (flange, boss and both holes),
  flange, spacer boss, hood sketch, hood, hood corner fillets, rib sketch, three ribs.

Open questions
- The hood is a 3 mm cantilever of about 95 mm off a clamp under one 1/4-20 screw and one
  M3. A knock on its far end levers the head screw; check it by hand before trusting it.
```

## Cup arm

### `cage_pack/cup_arm.py`

Docstring at line 1:

```text
Cup arm: a printed PETG arm that carries one suction cup off a side wall of the tray.

The tray's panel sits CUP_H off the glass, level with the cups' rail tops. Each arm's plate
lies on its cup's rail beside the tray, with one 3/8-16 screw and a load washer down into
the cup's centre thread. Its tongue reaches under the panel and takes two M3 screws up into
heat-set inserts pressed into the panel's glass side. A step between the two hugs the side
wall's outside face. The cup's whole rim, release tabs included, is outside the tray.

Both arms are this part: the -X arm is the +X arm turned 180° about the glass normal, since
the profile is symmetric about the cup's axis along Y. Print two.

Spec
- Process: FDM, black PETG, printed on its side: the profile below is the bed face, the
  arm's width rises from it, so every face of the profile is a wall and the screw holes
  run across the layers. PROCESS = "fdm".
- Frame (print frame): X = out from the side wall's outside face, Y = along the glass
  normal from the cups' rail tops (Y = 0 is Z = CUP_H in the assembly), Z = across the arm,
  0 to ARM_W. ASSEMBLY_AT places it on either side.
- Profile: tongue 36 under the panel, 10 thick; beam 18 deep (tongue + plate) to 8 mm
  past the wall, sloping up to the plate 2 mm short of the cup's rail; plate 8 thick to 30
  past the cup's axis. 40 wide.
- Holes:
  - cup screw: Ø10.3 (3/8" normal clearance, fasteners.md) at the cup's axis, CUP_OUT
    from the wall, in a Ø27 x 3.0 washer spot face. That leaves 5.0 under the washer, so a
    3/8-16 x 5/8" screw engages the cup's thread by 9.4, as the old 1/2" screw did through
    the tray's 1.6.
  - panel screws: 2 x Ø3.4 (M3 normal clearance) 28 in from the wall, 24 apart, each
    under a Ø6.5 x 3.5 counterbore for an M3 socket head from the glass side. M3 x 12
    engages the panel's 6.4 mm inserts by 5.5 (86 %).
- Model: features() (recipe.py), so the checked solid and the Onshape Part Studio are the
  same eight features: profile, arm, foot chamfer, cup screw sketch, hole, spot face,
  panel screw sketch, holes and counterbores.

Load path: the cup pulls the plate; the beam carries it to the wall as a lever, the panel
screws in tension 28 in from the wall and the panel's underside bearing on the tongue
at the wall. With the cup 45 out, a cup pull F puts about 2.6 F into the two screws.

Open questions
- Release tab positions are from photos (smallrig_4122c.py): check them on the real cups.
- The inserts pull out toward the side they were pressed from. Pull test a printed coupon
  before trusting the arm with a pack on the glass.
```

## Display bracket

### `cage_pack/display_bracket.py`

Docstring at line 1:

```text
Display bracket: a printed PETG frame behind the touch display that clamps its glass
against the lid's underside.

Four M2.5 x 6 screws go up through the frame into the display's threaded standoffs, and
four M3 x 8 screws go up through its ears into heat-set inserts in the lid's display
bosses. The display comes away with the lid; unplug its GH lead first.

Spec
- Process: FDM, black PETG, printed flat (the build frame is the print frame).
  PROCESS = "fdm".
- Frame: cage_layout.py. Top face on the display's standoff ends (DISP_BACK_Z), the
  bosses' end faces in the same plane.
- Holes: 4 x Ø2.9 (M2.5 normal clearance, fasteners.md) on the display's 60 x 41
  standoff pattern; 4 x Ø3.4 (M3 normal clearance) at the lid bosses (DISP_EARS).
- Size: frame 71 x 50 round the standoffs, ears out to ±36 in Y, 2.6 thick. A window
  through the middle, and a notch at the -X end round the display's GH connector, whose
  back face sits in the frame's top plane; its lead leaves through the notch.
- Critical dimensions: the standoff pattern (from Waveshare's STEP, inspect_step.py),
  the ear pattern (shared with enclosure_lid.py through cage_layout).
- Model: features() (recipe.py), so the checked solid and the Onshape Part Studio are the
  same five features: outline sketch, plate, corner fillets, cut-out sketch, cut.
```

## Enclosure tray

### `cage_pack/enclosure.py`

Docstring at line 1:

```text
Cage pack enclosure tray: the one part that carries everything, printed in black PETG.

A 3 mm back panel with four 2.4 mm walls rising toward the viewer, printed on a Bambu P1S
with the panel's glass-side face on the bed. Two printed arms (cup_arm.py) screw to the
panel's underside and carry a suction cup each, beside the side walls; everything inside
screws into heat-set inserts in printed bosses. The lid (enclosure_lid.py) closes the front.

Spec
- Process: FDM, black PETG, 0.4 nozzle, 0.2 layers, printed panel-down (the build frame is
  the print frame). Outside 250.8 x 213.8 x 73.0 on the P1S's 256 x 256 plate: 2.6 mm each
  side, so no brim; PETG over ASA for that reason (less warp) and for its impact toughness.
  PROCESS = "fdm".
- Frame: cage_layout.py. Panel's glass-side face at Z = CUP_H (level with the cups' rail
  tops).
- Stiffness: a cross of 6 mm ribs on the panel, a spine along X = 0 between the two CP10
  holes and a cross rib along Y = -50 under the ZED Box Mini wall to wall, with a ring
  around each CP10 hole. The 70 mm walls close the box.
- Weight: the panel is pocketed from the inside down to a 1.2 mm floor (six solid layers),
  leaving 3 mm full-thickness lands along the ribs, at x = -56, 48.5 and y = -116, round
  every boss and CP10 ring, and round the panel's edge. print_mass.py slices it at 282 g
  (317 g unpocketed).
- Interfaces:
  - Cup arms: 4 x Ø4.0 x 7.4 insert holes up from the panel's glass side, 28 in from each
    side wall at y = CUP_Y +- 12, for M3 x 6.4 heat-set inserts pressed from below. Each
    sits under a Ø8 x 6 boss on the panel's inside; the +X pair is under the ZED Box
    Mini's edge, 2 mm below its underside. M3 x 12 socket heads come up through each arm's
    tongue.
  - Tripod (CP10): 2 x Ø10.3 (3/8" normal clearance) at (0, 0) and (0, -153.5), where the
    cups used to bolt on, each in a Ø27 x 1.4 washer spot face. A 3/8-16 x 1/2" button head
    over a 1" washer from inside reaches a CP10 cheese plate under the panel.
  - Bosses Ø8 with Ø4.0 x 7.4 holes for M3 x 6.4 heat-set inserts (94180A333): four 8 mm
    under the ZED Box Mini's ears, four 8 mm under the PCBA, four 25 mm under the battery
    clamp and OpenUPS.
  - Camera (+X wall, outside): a 1/4-20 x 1" socket head from inside through a 4 mm pad on
    the wall's inside face (Ø6.53 close clearance), the shield's flange and spacer boss,
    into the ball head's base thread. An M3 insert through the pad (from inside) takes the
    shield's anti-rotation screw. A Ø14 pointed-roof hole passes the GMSL cable's FAKRA plug.
  - Service wall (-X): HR10 Ø8.2 + key slot in a 1.8 mm spot face (catalog panel 0.7 to
    2.0). GX12 hole with a flat-topped teardrop roof, under its nut.
  - Bottom wall: Ø19.3 power button hole (CIT AH, 19.00 cut-out) with a teardrop roof that
    stays under the button's Ø21.8 bezel. Beside it at cage_layout.RJ45_AT, the ModDIY RJ45
    panel jack cut-out 21.8 x 21.6 with a 0.2 mm roof that stays under its 22 mm flange (a
    21.4 mm bridge), 2 x Ø3.4 on 27.5 mm.
  - Antenna wall (+X): 3 x Ø6.5 for RP-SMA bulkheads.
  - Lid: six 9 x 9 bosses hung from the walls' tops, each with an M3 insert hole along -Z,
    carried by 50° gussets so they print without support.
  - Tether: Ø6.5 through the top wall at the -X end, in a pad that makes the wall 6 mm thick
    there.
- Vents: 3 x 30 slots with pointed roofs in the top and bottom walls.
- Critical dimensions: inside 209 x 209 x 70; CP10 holes 153.5 apart on X = 0, the INNOREL
  CP10's 3/8-16 pair, so the tray bolts to a CP10 on a tripod; arm insert holes on
  cage_layout.arm_screws(); fits the P1S plate with BED_MARGIN each side.

Open questions
- The arm inserts are pressed from the glass side and pull out toward it under load; pull
  test a printed coupon. PETG creeps under the arm screws' heads: check them before each
  event.
- Tether pull-out of the 6 mm pad is about 3 kN in shear at 30 MPa: estimate only. Pull test
  a printed coupon before trusting it.
- Panel stiffness against the aluminium tray is lower; dynamics.py assumes a stiff tray.
- The 1.2 mm pocket floors carry the glass-side face; a hit through the glass at a pocket is
  unchecked. Raise `floor` to 1.6 for about +10 g if a test print feels soft.
```

## Dimension sources, HR10

### `cage_pack/hr10.py`

Line 16, `PANEL_HOLE = 8.1`:

```text
catalog 8.1 +0.1
key slot to centre, catalog
catalog 1.6 +0.1
catalog 0.7 to 2.0
receptacle in front of the panel, catalog
M8 x 0.5 thread behind the panel, catalog
HR10A-7P plug body, catalog p.6
plug body with strain relief, catalog p.6
not in the catalog, assumed
not in the catalog, assumed
```

## PCBA

### `cage_pack/pcba_cage.py`

Docstring at line 1:

```text
Custom PCBA for the cage pack: an ESP32-S3 board with the USB bridge, the power button's
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
```

## Printing

### `cage_pack/petg_p1s.ini`

Lines 1-3:

```text
PrusaSlicer overrides for print_mass.py: black PETG on the Bambu P1S, 0.4 nozzle.
Match these in Bambu Studio when printing, or the slicer masses no longer hold.
Keys left out take PrusaSlicer's defaults.
```

## Workarounds

All unverified unless noted: the CAD builds are too heavy to rerun for a comment purge.

- `cage_pack/cage_layout.py:OUTER_CORNER_R`: the outside vertical corners. The inside ones are
  `OUTER_CORNER_R - WALL_T`.
- `cage_pack/cage_pack_assembly.py:_hr10_plug`: the GrabCAD "Hirose HR10 6 pin plug end" STEP, the
  same size-7 shell as the 4-pin HR10A-7P-4P. The body's front face sits on the receptacle's 7.8 mm
  front, plus 40 mm of cable, in the `hr10.py` frame (+Z into the enclosure). Only the body is
  placed: the nose and pins sit inside the receptacle.
- `cage_pack/camera_shield.py:_rib_profile`: the rib ends one corner radius short of the hood's end.
- `cage_pack/dynamics.py`: the rotational inertia `J` is that of a box about its own CG, roughly
  160 x 100 x 100 mm.
- `cage_pack/enclosure.py:TOUCHING_LAND_GROWTH`: a land circle that only touches a cell's edge
  leaves a pocket whose boundary meets itself at a point, which Onshape's kernel will not extrude
  (the PCBA's (25.5, -107.5) boss land touched the y = -114.5 cell edge). Such a land grows by
  0.1 mm so it crosses.
- `cage_pack/enclosure.py:BOSS_WALL_OVERLAP`: a lid boss is a 9 x 9 block under the wall's edge,
  its underside a 50 degree gusset back into the wall. The profile starts 0.2 mm inside the wall
  so the block fuses, and the gusset drops 0.2 mm extra. Top and bottom walls sketch on Right
  (Y, Z), the +X and -X walls on Front (X, Z). Not verified without the overlap.
