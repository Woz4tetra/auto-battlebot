# Mr Stabs Mk2 main board

These notes were moved out of source comments and docstrings in `hardware/` at commit
`23e253c1`. Names, line numbers and numbers are as they were in the code then.

## Silkscreen art pipeline usage

### `mr_stabs_mk2_pcb/pcba/art/export_robot.py`

Docstring at line 1:

```text
Export the Mr Stabs Mk2 assembly parts to STL for the silkscreen line render (render_robot.py).

~/.local/share/build123d-part/venv/bin/python art/export_robot.py
```

## Spec, open questions

### `mr_stabs_mk2_pcb/pcba/design.py`

Docstring at line 1:

```text
Mr Stabs Mk2 main board: ESP32-S3, BNO055, INA238, Crossfire Nano RX and the 4S power path.

Spec (references/spec_template.md, agreed with the user on 2026-10-08):
    Function: one board in the Mk2 chassis electronics bay replacing the QT Py ESP32-S3, the
        BNO055 breakout, the Matek I2C-INA-BM and the loose Crossfire Nano RX.
    Replaces: QT Py ESP32-S3 N4R2 (firmware/mr_stabs_mk2, board adafruit_qtpy_esp32s3_n4r2).
        Pin map kept:
            IO8  left ESC DShot (A3)        IO9  right ESC DShot (A2)
            IO17 CRSF TX (TXD1)             IO18 CRSF RX (RXD1)
            IO41 SDA1, IO40 SCL1 (Wire1): BNO055 at 0x28, INA238 at 0x45
            IO39 NeoPixel data              IO19 / IO20 USB D- / D+
        Firmware changes (user allowed them for substitutions): SHUNT_OHMS 0.0002 -> 0.0005
        (0.5 mohm 6 W shunt), and the INA238 in place of the INA228 (DEVICE_ID 3Fh die 0x238;
        16-bit VSHUNT at 1.25 uV/LSB with ADCRANGE 1, +-40.96 mV, which holds the 35 mV of a 70 A
        peak; VBUS 3.125 mV/LSB: SLYS025B tables 6-8, 6-9, 6-21).
    Power: two 2S packs in series (4S, 12.8 to 16.8 V), each on an XT60 pigtail. A FingerTech
        switch on a third XT60 pigtail loops pack+ out and back. Switched pack+ -> 20 vias in the
        SW_BACK pad -> 0.5 mohm shunt (B side) -> B VBATT pour -> ESC+ pads (B, beside it).
        70 A peak, 20 A continuous. 5 V from a TPS54202 buck (28 V rated), 3.3 V
        from an AP2112K, buck and USB VBUS OR-ed through Schottkys.
    Interfaces (all wire pads unless noted):
        Pack A -/+, pack B -/+, switch out/back: 12 AWG pigtails lap-soldered flat on
            3.6 x 5.0 mm roof-face edge pads; insulation starts past the edge in the wire bay.
        ESC L/R power +: 16 AWG lap pads 3.0 x 5.0 mm on the wedge face of the right ear, on the
            shunt's VBATT pour; ESC L/R power -: the same on the stem's front strip, beside
            the pack's GND entry (PACK_A-). ESC current never crosses the board's
            inner planes (sim/copper_ir.py).
        ESC L/R DShot + GND: lap pads 1.6 x 3.0 mm on the stem's front strip.
        Crossfire Nano RX: 1x4 2.54 mm right-angle SMD male header (HX PZ2.54-1x4P WT) on the
            wedge face at the stem's rear, pins pointing forward (GND, 5V, Ch1 = RX's CRSF TX,
            Ch2 = RX's CRSF RX, per the TBS quickstart). The RX slides onto the pins and stands
            perpendicular to the board, 18 mm deep against 22+ mm free (cad/, sectioned).
        USB: TYPE-C-31-D-06 vertical 16-pin receptacle, ESD on D+/D-, mates with the wedge plate
            off.
        BOOT / RESET: 1x3 2.54 mm vertical SMD jumper header IO0 / GND / EN on the wedge face
            (buttons get pressed in impacts).
        Wi-Fi: ESP32-S3-MINI-1U U.FL socket to a Molex 146153-0050 flex dipole (50 mm cable).
    Mechanical: outline from ../mr_stabs_mk2_pcb.py (wings, 93.24 x 35.0), 4 x 3.7 mm holes at
        (+-9.8, +-9.8) for #6 flat-head plastite screws from the wedge side through 9 mm printed
        washers. Roof face (on the bosses): 3.78 mm headroom, low SMD only. Wedge face: 16 to
        25 mm under the stem, 18 to 21 mm under the ears' rear strip, 0.6 mm over the ESCs.
    Fab: JLCPCB PCBA, 4 layers (F sig / In1 GND / In2 GND / B sig), 1.6 mm, 2 oz outer,
        HASL, double-sided assembly. 5 PCBs, 2 assembled.
    Substitution policy: firmware changes allowed.
    Assumptions the user agreed to: BNO055 X along board +X, Z normal to the board; NeoPixel
        kept on IO39; GPIO map and I2C addresses kept.

Open questions:
    1. YAW_RATE_SIGN and upside-down detection against the BNO055's new orientation.
    2. The mated U.FL plug tops out at about 3.3 mm (socket 0.8 mm up on the module, 2.5 mm
        mated height); check the module drawing's socket height against the 3.78 mm headroom.
    3. The standing Nano RX (x +-5.8) leaves about 1 mm to a driver on the rear screws at
        x +-9.8, depending on the driver's shank diameter.
    4. Holes are 3.7 mm; the placeholder drew 3.66 mm #6 close clearance.
    5. Firmware: set SHUNT_OHMS to 0.0005 and read the INA238 (vbat_sensor.cpp rejects its
        device ID and uses INA228 register scales). Then calibrate it once against a known load:
        the sense taps include some pour and via resistance, a fixed gain error.
    6. Antenna lead route: the bosses close the stem's front corners and the rear cross member
        sits 0.1 mm above the board, so the only exit on the roof face is forward over the module
        shield and between the front bosses (render_3d.png). A 1.13 mm cable over the 2.55 mm
        shield leaves 0.10 mm to the roof; an antenna on 0.81 mm cable leaves 0.42 mm. Pick one.
    7. Current path: 20 vias in SW_BACK, 18 + 2 at the shunt's VBATT end, 15 into each ESC+ pad,
        13 to 15 at each GND pad, about 1.2 A per via. Sized for 20 A continuous; not measured.
    8. Two GND fanout vias sit in capacitor pads, untented: C14 (10 uF on the LDO, the whole drill
        inside its GND pad) and C10 (one of three 22 uF buck outputs, half). Solder can wick down
        them; order filled and capped vias, or accept it (other caps share each job).

Run (from this directory): SKILL/scripts/kicad python3 design.py   -> mk2.net
```

## Wire pads

### `mr_stabs_mk2_pcb/pcba/design.py`

Line 85, `PAD_12AWG = "mk2:WirePad_Lap_3.6x5.0mm"`:

```text
SMD, wire laid flat: 3.78 mm roof headroom
```

## Power path

### `mr_stabs_mk2_pcb/pcba/design.py`

Line 127, `above gnd += pad("PACK_A-", "mk2:WirePad_Lap_3.6x5.0mm_Vias", "p_pack_a_neg")[1]`:

```text
--- Packs in series: A- is ground, A+ joins B-, B+ goes out to the switch and comes back as
BAT_IN. Pulling the switch's XT60 or turning it off leaves the whole board dead.
PACK_A-, SW_BACK and every ESC power pad carry a grid of 0.4 mm vias: those are where the
current changes layers, and sim/copper_ir.py found the vias there crowded.
```

Line 136, `above bat_in += pad("SW_BACK", "mk2:WirePad_Lap_3.6x5.0mm_Vias", "p_sw_back")[1]`:

```text
SW_BACK carries 20 vias in the pad: the pack current crosses to the B-side shunt there.
```

Line 145, `above for side in ("l", "r"):`:

```text
The ESC+ pads sit on the shunt's VBATT pour on the same side: the ESC current never changes
layers. VBATT has no inner plane (sim/copper_ir.py: a parallel inner path crowded its vias).
```

## Current sense

### `mr_stabs_mk2_pcb/pcba/design.py`

Line 139, `above shunt = part("Device", "R", "mk2:R_Yezhan_ASR3_2512", "0.5m", "C469426", tag="r_shunt")`:

```text
BAT_IN -> 1 mohm shunt -> VBATT, the ESC bus.
0.5 mohm, 6 W manganin (Yezhan ASR-M-3-0.5F): stalls can hold 70 A for seconds, 2.45 W here,
41% of its rating; the 2 W 1 mohm part would have run at 4.9 W. Datasheet land pattern.
```

Line 151, `above ina = part(`:

```text
INA238 high-side monitor at 0x45 (A0 = A1 = VS), as the Matek board was: JLCPCB assembly had no
INA228, and the INA238 has its pinout at 16 bits. Kelvin sense lines are their own nets so
placement can run them from the shunt pads, not the high-current copper.
```

Line 163, `above bat_in += ina["VBUS"]`:

```text
VBUS on the pack side of the shunt: the pin sits over the F BAT_IN pour, while VBATT is
across the shunt body on B (Freerouting failed that link in 3 of 5 near-clean tries). It reads
the shunt's drop high, at most 35 mV at a 70 A stall; VBUS current never passes the 10 ohms.
```

## Hot-plug

### `mr_stabs_mk2_pcb/pcba/design.py`

Line 174, `above bulk = part(`:

```text
Bulk on the ESC bus at the board: damps the ring when a pack is hot-plugged into ceramic-only
VBATT (TPS54202 7.3: it could overshoot the 30 V absolute max). The electrolytic's ESR is the
damping, so not a polymer. 8 mm tall: wedge face only.
```

### `mr_stabs_mk2_pcb/pcba/sim/hotplug.py`

Docstring at line 1:

```text
Hot-plug transient in ngspice: peak VBATT when a charged pack closes onto the board.

    SKILL/scripts/kicad python3 sim/hotplug.py      -> sim/hotplug.json, sim/hotplug_*.csv

Pack 17.4 V (4S LiHV full), 40 mohm internal. Leads: two 10 cm 12 AWG pigtails plus the
switch loop, taken as 0.4 uH and 15 mohm. Board VBATT ceramics: 2 x 10 uF 25 V X5R 0805 at about
3 uF each under 17 V DC bias, plus 100 nF. With bulk: the 100 uF 35 V Rubycon TZV, ESR taken as
its 0.34 ohm maximum impedance at 100 kHz (a lower real ESR damps less), 3 nH ESL. The ESCs' own
input caps are left out, the worst case for the board. TPS54202 absolute maximum VIN is 30 V.
```

## 5 V rail

### `mr_stabs_mk2_pcb/pcba/design.py`

Line 183, `above buck = part(`:

```text
--- 5 V: TPS54202 buck from VBATT (4S, up to 17.4 V LiHV; part is rated 28 V).
```

Line 189, `buck["EN"] += NC`:

```text
internal pull-up enables it
```

Line 192, `vbatt & c("100nF", "C307331", "c_buck_hf") & gnd`:

```text
50 V: the 16 V C1525 sat below 17.4 V LiHV
```

Line 196, `above ind = part("Device", "L", "Inductor_SMD:L_Taiyo-Yuden_NR-40xx", "15uH", "C167881", tag="l_buck")`:

```text
15 uH: TPS54202 table 7-2 for 5 V out (10 uH was below it). Isat 1.8 A vs 1.1 A peak.
```

Line 203, `above fb = Net("FB")`:

```text
FB: 0.596 V x (1 + 100k / 12k) = 5.56 V, about 5.2 V after the OR-ing Schottky.
```

Line 212, `above d_buck = part("Device", "D_Schottky", "Diode_SMD:D_SOD-123", "B5819W", "C8598", tag="d_buck")`:

```text
5 V rail = buck OR USB, each through a Schottky. The buck's high-side body diode would
otherwise backfeed USB 5 V into VBATT and the ESCs when the packs are off.
```

## 5 V rail loop

### `mr_stabs_mk2_pcb/pcba/design.py`

Line 199, `above for i in (1, 2, 3):`:

```text
Three 22 uF: two derate to about 20 uF at 5.6 V, which put crossover near 100 kHz with the
feed-forward cap (sim/buck_loop.py); three bring it to about 33 kHz, under TI's 40 kHz.
```

Line 207, `above v5_buck & c("47pF", "C1567", "c_ff") & fb`:

```text
Feed-forward across R_top: TPS54202 7.2.3.5.3 for all-ceramic output caps (table 7-2 lists 75 pF;
47 pF is the basic part, zero at 34 kHz near the estimated crossover).
```

### `mr_stabs_mk2_pcb/pcba/sim/buck_loop.py`

Docstring at line 1:

```text
TPS54202 loop gain: crossover and phase margin, with and without the feed-forward cap.

    ~/.local/share/pcba-design/venv/bin/python sim/buck_loop.py   -> sim/buck_loop.{png,json}

TI does not publish the internal compensation, so the model is built from what the datasheet
does give:
- Crossover without C6, equation 14 (p.16): fo = 3.95 / (VOUT x COUT), a -1 slope through fo.
- C6 across R2 adds a zero at 1 / (2 pi R2 C6) and a pole at 1 / (2 pi (R2 || R3) C6).
- Peak current mode's sampling double pole at fsw / 2 (500 kHz typical, p.5), Q = 2 / pi.
Effective COUT is the board's three 22 uF 25 V X5R 0805 after DC bias at 5.6 V (about 10 uF each,
from datasheets.json U2), the previous two, and TI's 44 uF for comparison. This is a small-signal
estimate, not TI's transistor model: it says whether the margin is comfortable, not its exact value.
```

## 3.3 V rail

### `mr_stabs_mk2_pcb/pcba/design.py`

Line 222, `above ldo = part(`:

```text
--- 3.3 V: AP2112K, 600 mA (ESP32-S3 Wi-Fi TX peaks about 350 mA).
```

## MCU

### `mr_stabs_mk2_pcb/pcba/design.py`

Line 255, `above j_boot = part(`:

```text
BOOT and RESET on a 1x3 jumper header, not buttons: buttons get pressed in impacts.
IO0 - GND jumpered at power-up enters the ROM bootloader; touching GND to EN resets.
```

## Status LED

### `mr_stabs_mk2_pcb/pcba/design.py`

Line 269, `above led = part(`:

```text
NeoPixel on IO39, powered from +5V: the XL-2020RGBC needs 3.5 V minimum (p.4), so the QT Py's
3V3 supply was out of spec. Its DIN threshold is 0.5 x VDD, under the ESP32's 3.3 V high.
```

## IMU

### `mr_stabs_mk2_pcb/pcba/design.py`

Line 280, `above imu = part(`:

```text
--- BNO055 at 0x28 (COM3 low), I2C mode (PS0 = PS1 = low), 32.768 kHz crystal because the
firmware calls setExtCrystalUse(true).
```

Line 291, `imu["CAP"] & c("1uF", "C52923", "c_imu_cap") & gnd`:

```text
1 uF: Bosch figures 9-11
```

## Receiver

### `mr_stabs_mk2_pcb/pcba/design.py`

Line 324, `above j_rx = part(`:

```text
--- Crossfire Nano RX on a 1x4 male header, its front connector in TBS order: GND (square
pad), 5V, Ch1 = CRSF TX (to ESP RX, IO18), Ch2 = CRSF RX (from ESP TX, IO17).
Right-angle SMD header: its pins run parallel to the board, so the RX slides on and stands
perpendicular to it. A vertical header would stack the RX flat against the board.
```

## USB

### `mr_stabs_mk2_pcb/pcba/design.py`

Line 341, `above usb = part("mk2", "TYPE-C-31-D-06", "mk2:USB-C-SMD_TYPE-C-31-D-06", lcsc="C2689964", tag="usb")`:

```text
--- USB-C, vertical: flashing and BLHeli passthrough with the wedge plate off. 5.1k on each CC
makes it a sink; USBLC6 clamps D+/D- because a person plugs this one in.
16-pin USB 2.0 receptacle (JLCPCB stock 2,310; JLCPCB flagged the 24-pin TYPE-C-31-M-06 as hard
to source).
```

## Mechanical

### `mr_stabs_mk2_pcb/pcba/design.py`

Line 375, `above for i, side in enumerate("LLRR"):`:

```text
--- Mounting: four 3.7 mm holes for #6 flat-head plastite screws into the chassis bosses.
The washers are D-cut on their outboard side, so each side has its own footprint.
```

## Floorplan

### `mr_stabs_mk2_pcb/pcba/gen_placement.py`

Docstring at line 1:

```text
Write placement.json for the Mr Stabs Mk2 board from board-frame coordinates.

Board frame (../mr_stabs_mk2_pcb.py): origin at the centre of the 19.6 mm hole pattern on the roof
face, x right (toward the right ESC), y toward the rear wall, mm. KiCad F side is the roof face
(3.78 mm to the chassis roof), B side the wedge face. Parts are found by value and nets, not
refdes, so a regenerated netlist keeps its placement.

    SKILL/scripts/run gen_placement.py   (needs info.jsonl and cad/keepout.json)

Floorplan:
    centre, F       ESP32-S3-MINI-1U at (0, 0): the only spot a 15.4 mm square fits on the roof
                    face (0.47 mm to the front bosses and the rear cross member). USB and CRSF
                    pins face the front, I2C the right, 3V3 / IO0 / DShot the left.
    front centre, F BNO055 block front-right between the bosses, clear of the USB-C legs; USB
                    ESD and CC beside the connector.
    ear front edges, F
                    12 AWG lap pads, wires laid flat into the wire bay ahead (40 mm clear at
                    |x| 14 to 26). Left: PACK_A-, PACK_A+, PACK_B-. Right: SW_BACK, SW_OUT,
                    PACK_B+. Each switch lead and pack A stay on one side; pack B splits.
    right ear, F    INA238 above the shunt, buck block and its three output caps outboard.
    left ear, F     LDO, EN RC.
    stem rear, F    NeoPixel behind the module, under the TPU roof so it shows through.
    right ear, B    shunt and its Kelvin resistors, ESC+ lap pads and the 100 uF bulk on its
                    VBATT pour.
    stem, B         USB-C between the bosses, away from the motors.
    stem front, B   ESC- lap pads beside PACK_A-, then DShot and signal-ground pads, below the
                    washers.
    left ear, B     BOOT/RST 1x3 SMD jumper header.
    stem rear, B    Nano RX 1x4 right-angle SMD header, pins forward; the RX stands on them.
```

## Placement and routing log

### `mr_stabs_mk2_pcb/pcba/gen_placement.py`

Line 177, `above USB: (0.0, -11.1, 0),`:

```text
USB-C in the stem, away from the motors under the ears: a dislodged motor can reach the
ears, not the stem between the bosses. USB between the front washers (0.15 mm to each),
BOOT/RST out on the left ear where the USB was.
```

Line 181, `above find("0.5m"): (`:

```text
Shunt on the wedge face: BAT_IN end toward SW_BACK, VBATT end toward the B-side ESC bus.
The 20 A crosses layers once, in the stitched BAT_IN pour between it and SW_BACK.
```

Line 187, `),`:

```text
courtyard clears the ear-edge ledge (B_low) and the D-cut washer
Kelvin resistors on the shunt's side, beside its pads, so each tap leaves from the pad
itself (Vishay's sensing-trace drawing); SENSE_P / SENSE_N then via up to the INA238.
```

Line 192, `above find("ESC_L+"): (21.0, -11.8, 0),`:

```text
100 uF bulk on the right ear's wedge face (8 mm tall; 18-21 mm free there), on the shunt's
VBATT pour beside the ESC+ pads. It sat in the stem for motor-impact protection, but its
feed crossed the USB-C and BNO055 fan-out, and at 2 oz spacing no Freerouting try (26 of
them) routed it; a prewired track there walled off the stem's signals.
pads along y: at 0 deg the GND pad crossed the ear edge
ESC power and DShot lap pads on the stem's front strip, below the washers (y < -14.3).
Leads run forward into the wire bay, then out to each ESC's inner end at |x| 14.
+ outboard so the B-side VBATT pour reaches each one from the bus above the washers.
ESC+ on the shunt's VBATT pour (right ear), ESC- beside the pack's GND entry (left ear):
the 20 A never crosses the 0.5 oz inner planes (sim/plane_ir.py). Leads run from each ESC's
inner end at |x| 14 under the ears.
```

Line 205, `above find("ESC_L-"): (-10.6, -17.1, 0),`:

```text
ESC- on the stem's front strip, 4 to 8 mm from PACK_A- (the GND entry): their via grids
carry the return to PACK_A-'s grid. The left ear's roof face is full of the LDO block.
```

Line 213, `above find("NANO_RX"): (0.0, 12.6, 180),`:

```text
Right-angle header, pins pointing forward: the RX stands on them at y 4.2-6.6, 18 mm deep
toward the wedge, where 22 mm is free (cad/, sectioned).
180: KiCad mirrors bottom footprints (its pins would point rearward at 0); see lay.size.
```

Line 238, `above INA: (15.4, -9.1, 270),`:

```text
INA238 on the shunt's Kelvin lines, VBUS pin over the F BAT_IN pour.
The sense taps see a little pour and via resistance besides the shunt: a fixed gain error,
calibrated out with a known load.
270: pins 6-10 (GND, VBATT, SENSE) face the shunt; at 90 they faced the ear's rear edge
and pin 7 had no room for its GND via.
y -9.1: at -8.6 its pins 1-5 sat 0.68 mm from the rear edge and SDA squeezed past SCL's
pad; lower also seats VBUS (pin 8) on the F BAT_IN pour.
```

Line 252, `above find("100nF", "VBATT"): (27.23, -16.15, 270),`:

```text
270: VBATT pad up, against the F VBATT patch; at 90 its GND pad faced the patch and the
GND fanout track cut the VBATT pad off it.
```

Line 289, `above IMU: (7.6, -16.4, 90),`:

```text
IMU between the front bosses, rotated so every LGA side has room to escape.
IMU block front-right of the USB-C's shell legs (they pierce the top at (+-2.4, -8.95) and
(+-2.4, -13.25)), rotated 90 with room on every side: at the front edge 3 nets could not
escape the LGA.
```

Line 313, `above lay.want(cap_ref[0], 7.6, -17.2, 0, reach=1.5, bottom_ok=True)`:

```text
The CAP pin's cap goes straight under the IMU on the wedge face (the roof face around it is
full; 6 mm away on top it would not route): a short reach so it never lands far off on top.
```

Line 316, `above lay.want(find("22pF", "XIN32"), 3.1, -15.25, 0, reach=1.0, bottom_ok=True)`:

```text
XIN's load cap on the wedge face under the crystal, between the USB-C and the signal pads: at
the front edge on top its GND pad had no room for a via in any of 20 routing tries.
```

Line 327, `"inherit": "jlcpcb_2layer",`:

```text
JLCPCB's 2-layer minimums are also safe on its 4-layer process
Signal vias are 0.5 / 0.25 mm (netclasses below); JLCPCB drills 0.2 mm on 4 layers.
```

Line 330, `above "flow": {`:

```text
Both inner layers are GND (the default fanout covers them). stitch_pitch 1.0 packs the
BAT_IN and VBATT pours' F/B overlaps with vias.
```

Line 339, `above "schematic": {`:

```text
4 layers: F signal, In1 and In2 solid GND (the ESC return), B signal. VBATT lives only on
the right ear's B pour, shunt to ESC+ pads (sim/copper_ir.py).
5 PCBs, 2 assembled (intake)
Schematic sheets by function (SKILL/scripts/schematic.py); passives follow their IC.
```

Line 353, `above "outer_copper_oz": 2,`:

```text
2 oz outer for the ESC current: JLCPCB's 2 oz minimums, 0.16 mm track and 0.20 mm space.
```

Line 356, `above "net_voltage": {`:

```text
Worst-case net voltages for rating_check.py: 4S LiHV is 17.4 V full; SW swings to VIN;
BOOT rides about 5.6 V above SW.
```

Line 373, `above "board": {"outline": [k(x, y) for x, y in outline_b], "corner_radius": 1.0},`:

```text
Convex corners filleted (pcb.py); the notch corners stay sharp for the chassis bay.
```

Line 376, `above "Default": {"track_width": 0.16, "via_diameter": 0.5, "via_drill": 0.25},`:

```text
Signals at JLCPCB's 2 oz minimum width on 0.5 / 0.25 mm vias: at 0.2 mm clearance, 0.2
then 0.16 mm tracks on 0.6 mm vias left every Freerouting try 4 to 18 errors or unrouted
links in the stem (crystal, SDA, DSHOT_R, the INA238 sense tap). They carry milliamps.
```

Line 380, `above "Ground": {"track_width": 0.2, "via_diameter": 0.6, "via_drill": 0.3, "nets": ["GND"]},`:

```text
Classes below inherit Default's vias; anything carrying current keeps 0.6 / 0.3.
GND's fanout and stitching vias also join the outer pours that return ESC current.
```

Line 396, `above "Rail": {"track_width": 0.2, "nets": ["+3V3"]},`:

```text
0.2 mm: the BNO055's LGA pads are 0.25 mm wide at 0.5 mm pitch. Under 0.4 A.
```

Line 430, `above "outline": [`:

```text
Reaches up toward the shunt so the stitching vias in its overlap with the B pour
share the layer change with SW_BACK's grid (sim/copper_ir.py: its top row crowded).
```

Line 464, `above {`:

```text
Buck input on F: C4, C5 and C6's VBATT pads to U2's VIN, over the B VBATT pour, so
VBATT stitching drops it to B. Routed, that drop failed in some tries. Clear of the
PACK+ pour (x < 26.2) and the +5V pad of C12.
```

Line 487, `above {`:

```text
Shunt VBATT end on B, under the ESC+ pads, the bulk cap and the whole F buck patch: other
nets' tracks (+5V, SW) cut that patch, and a keepout there boxed in U2's SW pin, so
each piece instead gets its own VBATT stitching vias down to this pour.
```

## Placement rationale

### `mr_stabs_mk2_pcb/pcba/gen_placement.py`

Line 246, `find("100nF", "SENSE_P", "SENSE_N"): (18.4, -12.2, 90),`:

```text
off the BAT_IN pour
```

Line 310, `}`:

```text
the ear tip is full; take the nearest free spot
```

## Netclasses

### `mr_stabs_mk2_pcb/pcba/gen_placement.py`

Line 383, `above "Battery": {`:

```text
Tracks only feed pins; the zones carry the current.
```

## Fab pipeline: render_3d usage and custom models

### `mr_stabs_mk2_pcb/pcba/render_3d.py`

Docstring at line 1:

```text
Render the assembled board: every part's 3D model, plus the parts KiCad has none for.

    SKILL/scripts/kicad python3 render_3d.py      (after make_fab.sh; reads mk2_routed.kicad_pcb)

Stock models come from SKILL/scripts/fetch_3dmodels.py; the rest are build123d models in
art/models/ (checked with the build123d skill's check.py, STEPs copied to lib/mk2.3dshapes/):
    L1   FNR4030S100MT inductor, the footprint KiCad ships without a model
    H2   the Crossfire Nano RX on its right-angle header
    ESP1 the U.FL plug and Molex 146153-0050 lead on the module's socket
Writes render_3d_{top,bottom,iso_top,iso_bottom}.png. The fab board is not modified.
```

## Board frame

### `mr_stabs_mk2_pcb/pcba/render_3d.py`

Line 43, `above board.GetDesignSettings().SetAuxOrigin(pcbnew.VECTOR2I(pcbnew.FromMM(100), pcbnew.FromMM(100)))`:

```text
Drill origin at the hole-pattern centre (KiCad 100, 100), the board frame of
../mr_stabs_mk2_pcb.py and the chassis CAD, so the STEP drops into the robot assembly.
```

## Fab pipeline

### `mr_stabs_mk2_pcb/pcba/silk.py`

Docstring at line 1:

```text
Finish the routed board: wire-pad and pin labels, the Mk2 and BWBots art, USB-C shell paste.

    SKILL/scripts/kicad python3 silk.py mk2_routed.kicad_pcb        (in place; or IN OUT)

Run it on the routed board after flow.sh and before check.sh / fab.sh: flow.sh rebuilds the
board from the netlist on every try, so anything added before routing is lost. Silk does not
touch copper, so the route is unchanged. Labels are placed from the pads' real positions, found
by the wire pads' values (the net labels in design.py), so they follow placement changes.

Board frame (gen_placement.py): x right, y toward the rear wall, mm; KiCad X = 100 + x,
Y = 100 - y. B-side items are drawn as seen from below (x mirrored) so they read correctly
when the board is turned over.
```

Line 155, `above for fp in board.GetFootprints():`:

```text
Hide every refdes: on a board this dense they collide with parts and labels, and JLCPCB
assembles from the CPL. The labels above say what each connection is.
```

## Silkscreen layout

### `mr_stabs_mk2_pcb/pcba/silk.py`

Line 145, `above text("MR STABS MK2", -36.0, -13.6, "B", h=0.9, bold=True)`:

```text
Name, revision and signature on the left ear's bare wedge-face strip (over the ESC). The
roof-face pads carry their own labels, so the strip needs no pad legend.
```

## Copper IR sim

### `mr_stabs_mk2_pcb/pcba/sim/copper_ir.py`

Docstring at line 1:

```text
DC current flow through each high-current net, every copper layer at once.

    ~/.local/share/pcba-design/venv/bin/python sim/copper_ir.py    (after export_geometry.py)

Finite volume on a 0.15 mm grid per layer: each cell couples to its neighbours by 1 / R_sheet;
each via or plated hole couples the cells it passes through on every layer by the barrel
resistance between those layers. A wire pad is a terminal: its cells are tied to one node (the
wire and its solder are far better conductors than the board). Current enters and leaves at
the terminals, split evenly between the ESCs. Reports, per net and load: the voltage drop,
the copper loss, the busiest via's current, and the 99.9th-percentile current density;
writes sim/copper_ir.json and sim/copper_ir_<NET>.png (20 A).

Copper: 2 oz outer (as ordered), 0.5 oz inner (JLCPCB's 4-layer default; this design does not
order 1 oz). Stack positions for a 1.6 mm JLCPCB 4-layer board: F 0, In1 0.21, In2 1.39, B 1.6.
This is DC and electrical only: no temperature.
```

## Fab pipeline

### `mr_stabs_mk2_pcb/pcba/make_fab.sh`

Lines 2-9:

```text
Routed board to JLCPCB upload files. Run from this directory after SKILL/scripts/flow.sh mk2.
  ./make_fab.sh
1. silk.py: labels, Mk2 and BWBots art, via-field tenting (flow.sh rebuilds the board each try,
   so these go on after routing).
2. check.sh: DRC, exits 1 on any error or unconnected item.
3. fab.sh: fab/gerbers.zip, fab/bom.csv, fab/cpl.csv.
4. Drop BOM rows with no LCSC number: wire pads and the via field are copper, not parts.
5. schematic.py (connectivity checked against mk2.net) and render_3d.py, then the review page.
```

## Workarounds

All unverified: SKiDL and the routing flow were not run for this purge.

- `mr_stabs_mk2_pcb/mr_stabs_mk2_pcb.py:MATERIAL`: `"pc"` is a density stand-in. check.py has no
  PCB process, so `PROCESS = "none"`.
- `mr_stabs_mk2_pcb/pcba/design.py:NC`: SKiDL 2.3 injects its no-connect net into `builtins`
  instead of exporting it, so `from skidl import NC` fails.
- `mr_stabs_mk2_pcb/pcba/design.py:GND_STITCH`: two GND stitching vias in the stem's sides. The
  tracks to the stem's USB-C and bulk cap box the bottom GND pour into fragments the flow's
  stitching found no via spot in.
- `mr_stabs_mk2_pcb/pcba/gen_placement.py:lay.size` for the LED and USB-C: the EasyEDA courtyards
  miss their own pads (`footprint_audit.py`), so hand-measured boxes replace them before
  legalizing.
- `mr_stabs_mk2_pcb/pcba/gen_placement.py:lay.size` for NANO_RX: the box runs feet to pin tips and
  covers the RX's 11 mm width. `legalize.py` does not mirror bottom-side courtyards and KiCad does,
  so this one is given mirrored (local y -12.35 to 1.85, not -1.85 to 12.35).
- `mr_stabs_mk2_pcb/pcba/gen_placement.py:holes`: the screw head sits on the B side inside the
  printed washer (in the B keep-out). On the F side the hole needs only its own 3.7 mm plus 0.15 mm.
- `mr_stabs_mk2_pcb/pcba/gen_placement.py:assert lay.rect(MCU...)`: the MCU module overhangs
  nothing but sits 0.47 mm from the roof keep-out. It is fixed, so legalizing checks it against the
  board edge only, and the assert checks the keep-out margin.
- `mr_stabs_mk2_pcb/pcba/gen_placement.py` B.Cu keep-out at (16.6..19.4, -8.8..-5.6): keeps signals
  out of the B VBATT pour's neck, shunt VBATT end to ESC_L+. Two routing tries cut the neck with
  tracks and left the 20 A ESC current on a track. It clears the INA238's +3V3 pins (vias go
  through B) and R3's SENSE_N pad, whose track leaves downward.
- `mr_stabs_mk2_pcb/pcba/gen_placement.py` B.Cu keep-out at (18.75..19.6, -11.05..-10.4): keeps
  signals out of the strip between R3's VBATT pad and ESC_L+, so the pour always joins the Kelvin
  tap. SENSE_N and SENSE_P tracks had boxed that pad in. With R3 turned to face the pad, it no
  longer fit between the shunt and ESC_L+.
- `mr_stabs_mk2_pcb/pcba/render_3d.py`: `mk2_render.kicad_pcb` is written beside the project so
  `${KIPRJMOD}` resolves in the 3D model paths.
- `mr_stabs_mk2_pcb/pcba/silk.py`: the EasyEDA LED footprint puts its polarity circle on its own
  pad 1, so the F.SilkS circle is deleted from every XL-2020RGBC-WS2812B on the routed board. The
  chamfer marks pad 1 as well.
- `mr_stabs_mk2_pcb/pcba/silk.py`: paste is added to the USB-C's shell-leg slots (pad `EP`) so
  reflow solders them. The EasyEDA pads had none. The routed board carries the footprint as
  kinet2pcb built it, so the paste is set there as well.
