# Mr Stabs Mk2 main board

## Silkscreen art pipeline usage

```text
Export the Mr Stabs Mk2 assembly parts to STL for the silkscreen line render (render_robot.py).

~/.local/share/build123d-part/venv/bin/python art/export_robot.py
```

## Spec, open questions

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

## Hot-plug

```text
Hot-plug transient in ngspice: peak VBATT when a charged pack closes onto the board.

    SKILL/scripts/kicad python3 sim/hotplug.py      -> sim/hotplug.json, sim/hotplug_*.csv

Pack 17.4 V (4S LiHV full), 40 mohm internal. Leads: two 10 cm 12 AWG pigtails plus the
switch loop, taken as 0.4 uH and 15 mohm. Board VBATT ceramics: 2 x 10 uF 25 V X5R 0805 at about
3 uF each under 17 V DC bias, plus 100 nF. With bulk: the 100 uF 35 V Rubycon TZV, ESR taken as
its 0.34 ohm maximum impedance at 100 kHz (a lower real ESR damps less), 3 nH ESL. The ESCs' own
input caps are left out, the worst case for the board. TPS54202 absolute maximum VIN is 30 V.
```

## 5 V rail loop

```text
TPS54202 loop gain: crossover and phase margin, with and without the feed-forward cap.

    ~/.local/share/pcba-design/venv/bin/python sim/buck_loop.py   -> sim/buck_loop.{png,json}

TI does not publish the internal compensation, so the model is built from what the datasheet
does give:
- Crossover without C6, equation 14 (p.16): fo = 3.95 / (VOUT x COUT), a -1 slope through fo.
- C6 across R2 adds a zero at 1 / (2 pi R2 C6) and a pole at 1 / (2 pi (R2 || R3) C6).
- Peak current mode's sampling double pole at fsw / 2 (500 kHz typical, p.5), Q = 2 / pi.
Effective COUT is the board's three 22 uF 25 V X5R 0805 after DC bias at 5.6 V (about 10 uF each,
from datasheets.json U2), and TI's 44 uF for comparison. This is a small-signal
estimate, not TI's transistor model: it says whether the margin is comfortable, not its exact value.
```

## Floorplan

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

## Render the assembled board

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

## Silkscreen finish

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

## Copper IR sim

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

## Fab files

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
