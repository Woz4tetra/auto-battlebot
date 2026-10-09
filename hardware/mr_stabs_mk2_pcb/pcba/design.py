"""Mr Stabs Mk2 main board: ESP32-S3, BNO055, INA238, Crossfire Nano RX and the 4S power path.

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
"""

import builtins
import os

from skidl import ERC, KICAD9, Net, Part, generate_netlist, lib_search_paths, set_default_tool

set_default_tool(KICAD9)
NC = builtins.NC  # SKiDL 2.3 injects its no-connect net into builtins instead of exporting it
HERE = os.path.dirname(os.path.abspath(__file__))
lib_search_paths[KICAD9].append(os.path.join(HERE, "lib"))

R0402 = "Resistor_SMD:R_0402_1005Metric"
C0402 = "Capacitor_SMD:C_0402_1005Metric"
C0603 = "Capacitor_SMD:C_0603_1608Metric"
C0805 = "Capacitor_SMD:C_0805_2012Metric"
PAD_12AWG = "mk2:WirePad_Lap_3.6x5.0mm"  # SMD, wire laid flat: 3.78 mm roof headroom
PAD_16AWG = "mk2:WirePad_Lap_3.0x5.0mm"
PAD_SIG = "mk2:WirePad_Lap_1.6x3.0mm"


def part(lib, name, fp, value=None, lcsc=None, *, tag):
    # tag gives a stable identity, so footprints keep their placement when the netlist regenerates
    p = Part(lib, name, footprint=fp, tag=tag)
    if value:
        p.value = value
    if lcsc:
        p.fields["LCSC"] = lcsc
    return p


def r(value, lcsc, tag):
    return part("Device", "R", R0402, value, lcsc, tag=tag)


def c(value, lcsc, tag, fp=C0402):
    return part("Device", "C", fp, value, lcsc, tag=tag)


def pad(name, fp, tag):
    """A wire solder pad: one plated hole, no part to buy."""
    p = Part("Connector", "TestPoint", footprint=fp, tag=tag)
    p.value = name
    return p


pack_mid, pack_pos, bat_in, vbatt, gnd = (
    Net("PACK_MID"),
    Net("PACK+"),
    Net("BAT_IN"),
    Net("VBATT"),
    Net("GND"),
)
v5_buck, vbus_usb, v5, v3v3 = Net("V5_BUCK"), Net("VBUS_USB"), Net("+5V"), Net("+3V3")
for n in (pack_mid, pack_pos, bat_in, vbatt, gnd, v5_buck, vbus_usb, v5, v3v3):
    n.drive = 3  # power nets, so SKiDL ERC accepts passive-only loads
sda, scl = Net("SDA1"), Net("SCL1")

# --- Packs in series: A- is ground, A+ joins B-, B+ goes out to the switch and comes back as
# BAT_IN. Pulling the switch's XT60 or turning it off leaves the whole board dead.
# PACK_A-, SW_BACK and every ESC power pad carry a grid of 0.4 mm vias: those are where the
# current changes layers, and sim/copper_ir.py found the vias there crowded.
gnd += pad("PACK_A-", "mk2:WirePad_Lap_3.6x5.0mm_Vias", "p_pack_a_neg")[1]
pack_mid += pad("PACK_A+", PAD_12AWG, "p_pack_a_pos")[1]
pack_mid += pad("PACK_B-", PAD_12AWG, "p_pack_b_neg")[1]
pack_pos += pad("PACK_B+", PAD_12AWG, "p_pack_b_pos")[1]
pack_pos += pad("SW_OUT", PAD_12AWG, "p_sw_out")[1]
# SW_BACK carries 20 vias in the pad: the pack current crosses to the B-side shunt there.
bat_in += pad("SW_BACK", "mk2:WirePad_Lap_3.6x5.0mm_Vias", "p_sw_back")[1]

# BAT_IN -> 1 mohm shunt -> VBATT, the ESC bus.
# 0.5 mohm, 6 W manganin (Yezhan ASR-M-3-0.5F): stalls can hold 70 A for seconds, 2.45 W here,
# 41% of its rating; the 2 W 1 mohm part would have run at 4.9 W. Datasheet land pattern.
shunt = part("Device", "R", "mk2:R_Yezhan_ASR3_2512", "0.5m", "C469426", tag="r_shunt")
bat_in += shunt[1]
vbatt += shunt[2]
# The ESC+ pads sit on the shunt's VBATT pour on the same side: the ESC current never changes
# layers. VBATT has no inner plane (sim/copper_ir.py: a parallel inner path crowded its vias).
for side in ("l", "r"):
    vbatt += pad(f"ESC_{side.upper()}+", PAD_16AWG, f"p_esc_{side}_pos")[1]
    gnd += pad(f"ESC_{side.upper()}-", "mk2:WirePad_Lap_3.0x5.0mm_Vias", f"p_esc_{side}_neg")[1]

# INA238 high-side monitor at 0x45 (A0 = A1 = VS), as the Matek board was: JLCPCB assembly had no
# INA228, and the INA238 has its pinout at 16 bits. Kelvin sense lines are their own nets so
# placement can run them from the shunt pads, not the high-current copper.
ina = part(
    "Sensor_Energy", "INA238", "Package_SO:VSSOP-10_3x3mm_P0.5mm", lcsc="C2868250", tag="ina"
)
sense_p, sense_n = Net("SENSE_P"), Net("SENSE_N")
sense_p += ina["Vin+"]
sense_n += ina["Vin-"]
bat_in & r("10", "C25077", "r_sense_p") & sense_p
vbatt & r("10", "C25077", "r_sense_n") & sense_n
sense_p & c("100nF", "C1525", "c_sense") & sense_n
# VBUS on the pack side of the shunt: the pin sits over the F BAT_IN pour, while VBATT is
# across the shunt body on B (Freerouting failed that link in 3 of 5 near-clean tries). It reads
# the shunt's drop high, at most 35 mV at a 70 A stall; VBUS current never passes the 10 ohms.
bat_in += ina["VBUS"]
v3v3 += ina["VS"], ina["A0"], ina["A1"]
gnd += ina["GND"]
sda += ina["SDA"]
scl += ina["SCL"]
ina["~{Alert}"] += NC
v3v3 & c("100nF", "C1525", "c_ina") & gnd

# Bulk on the ESC bus at the board: damps the ring when a pack is hot-plugged into ceramic-only
# VBATT (TPS54202 7.3: it could overshoot the 30 V absolute max). The electrolytic's ESR is the
# damping, so not a polymer. 8 mm tall: wedge face only.
bulk = part(
    "Device", "C_Polarized", "Capacitor_SMD:CP_Elec_6.3x7.7", "100uF 35V", "C88744", tag="c_bulk"
)
vbatt += bulk[1]
gnd += bulk[2]

# --- 5 V: TPS54202 buck from VBATT (4S, up to 17.4 V LiHV; part is rated 28 V).
buck = part(
    "Regulator_Switching", "TPS54202DDC", "Package_TO_SOT_SMD:SOT-23-6", lcsc="C191884", tag="buck"
)
vbatt += buck["VIN"]
gnd += buck["GND"]
buck["EN"] += NC  # internal pull-up enables it
for i in (1, 2):
    vbatt & c("10uF", "C15850", f"c_buck_in{i}", C0805) & gnd
vbatt & c("100nF", "C307331", "c_buck_hf") & gnd  # 50 V: the 16 V C1525 sat below 17.4 V LiHV
sw = Net("SW")
sw += buck["SW"]
buck["BOOT"] & c("100nF", "C1525", "c_boot") & sw
# 15 uH: TPS54202 table 7-2 for 5 V out (10 uH was below it). Isat 1.8 A vs 1.1 A peak.
ind = part("Device", "L", "Inductor_SMD:L_Taiyo-Yuden_NR-40xx", "15uH", "C167881", tag="l_buck")
sw & ind & v5_buck
# Three 22 uF: two derate to about 20 uF at 5.6 V, which put crossover near 100 kHz with the
# feed-forward cap (sim/buck_loop.py); three bring it to about 33 kHz, under TI's 40 kHz.
for i in (1, 2, 3):
    v5_buck & c("22uF", "C45783", f"c_buck_out{i}", C0805) & gnd
# FB: 0.596 V x (1 + 100k / 12k) = 5.56 V, about 5.2 V after the OR-ing Schottky.
fb = Net("FB")
fb += buck["FB"]
v5_buck & r("100k", "C25741", "r_fb_top") & fb
# Feed-forward across R_top: TPS54202 7.2.3.5.3 for all-ceramic output caps (table 7-2 lists 75 pF;
# 47 pF is the basic part, zero at 34 kHz near the estimated crossover).
v5_buck & c("47pF", "C1567", "c_ff") & fb
fb & r("12k", "C25752", "r_fb_bot") & gnd

# 5 V rail = buck OR USB, each through a Schottky. The buck's high-side body diode would
# otherwise backfeed USB 5 V into VBATT and the ESCs when the packs are off.
d_buck = part("Device", "D_Schottky", "Diode_SMD:D_SOD-123", "B5819W", "C8598", tag="d_buck")
d_usb = part("Device", "D_Schottky", "Diode_SMD:D_SOD-123", "B5819W", "C8598", tag="d_usb")
v5_buck += d_buck["A"]
v5 += d_buck["K"]
vbus_usb += d_usb["A"]
v5 += d_usb["K"]
v5 & c("10uF", "C19702", "c_5v", C0603) & gnd

# --- 3.3 V: AP2112K, 600 mA (ESP32-S3 Wi-Fi TX peaks about 350 mA).
ldo = part(
    "Regulator_Linear", "AP2112K-3.3", "Package_TO_SOT_SMD:SOT-23-5", lcsc="C51118", tag="ldo"
)
v5 += ldo["VIN"], ldo["EN"]
v3v3 += ldo["VOUT"]
gnd += ldo["GND"]
v5 & c("1uF", "C52923", "c_ldo_in") & gnd
v3v3 & c("10uF", "C19702", "c_ldo_out", C0603) & gnd

# --- ESP32-S3-MINI-1U-N4R2: the QT Py's chip, flash and PSRAM, with a U.FL socket on the module.
mcu = part(
    "mk2",
    "ESP32-S3-MINI-1U-N4R2",
    "mk2:BULETM-SMD_ESPRESSIF_ESP32-S3-MINI-1U-N8",
    lcsc="C22356044",
    tag="mcu",
)
v3v3 += mcu["3V3"]
gnd += mcu["GND"]
v3v3 & c("10uF", "C19702", "c_mcu_bulk", C0603) & gnd
v3v3 & c("100nF", "C1525", "c_mcu") & gnd
en, io0 = Net("EN"), Net("IO0")
en += mcu["EN"]
io0 += mcu["IO0"]
v3v3 & r("10k", "C25744", "r_en") & en
en & c("1uF", "C52923", "c_en") & gnd
v3v3 & r("10k", "C25744", "r_io0") & io0
sda += mcu["IO41"]
scl += mcu["IO40"]
v3v3 & r("4.7k", "C25900", "r_sda") & sda
v3v3 & r("4.7k", "C25900", "r_scl") & scl

# BOOT and RESET on a 1x3 jumper header, not buttons: buttons get pressed in impacts.
# IO0 - GND jumpered at power-up enters the ROM bootloader; touching GND to EN resets.
j_boot = part(
    "mk2",
    "HXPZ2.54-1X3PTP-YQ",
    "mk2:HDR-SMD_3P-P2.54-V-M_HX-PZ2.54-1X3P-TP-YQ",
    "BOOT/RST",
    "C41417360",
    tag="j_boot",
)
j_boot[1] += io0
j_boot[2] += gnd
j_boot[3] += en

# NeoPixel on IO39, powered from +5V: the XL-2020RGBC needs 3.5 V minimum (p.4), so the QT Py's
# 3V3 supply was out of spec. Its DIN threshold is 0.5 x VDD, under the ESP32's 3.3 V high.
led = part(
    "mk2", "XL-2020RGBC-WS2812B", "mk2:LED-SMD_4P-L2.0-W2.0-BR-1", lcsc="C5349955", tag="led"
)
v5 += led["VDD"]
gnd += led["GND"]
mcu["IO39"] & r("100", "C25076", "r_led") & led["DI"]
led["DO"] += NC
v5 & c("100nF", "C1525", "c_led") & gnd

# --- BNO055 at 0x28 (COM3 low), I2C mode (PS0 = PS1 = low), 32.768 kHz crystal because the
# firmware calls setExtCrystalUse(true).
imu = part(
    "Sensor_Motion", "BNO055", "Package_LGA:LGA-28_5.2x3.8mm_P0.5mm", lcsc="C93216", tag="imu"
)
v3v3 += imu["VDD"], imu["VDDIO"]
gnd += imu["GND"], imu["GNDIO"], imu["PS0"], imu["PS1"], imu["COM2"], imu["COM3"]
sda += imu["COM0"]
scl += imu["COM1"]
for p_name, tag in (("~{RESET}", "r_imu_rst"), ("~{BOOT_LOAD_PIN}", "r_imu_boot")):
    v3v3 & r("10k", "C25744", tag) & imu[p_name]
imu["CAP"] & c("1uF", "C52923", "c_imu_cap") & gnd  # 1 uF: Bosch figures 9-11
v3v3 & c("100nF", "C1525", "c_imu_vdd") & gnd
v3v3 & c("100nF", "C1525", "c_imu_vddio") & gnd
imu["INT"] += NC
imu["BL_IND"] += NC
for p_name in (
    "PIN1",
    "PIN7",
    "PIN8",
    "PIN12",
    "PIN13",
    "PIN15",
    "PIN16",
    "PIN21",
    "PIN22",
    "PIN23",
    "PIN24",
):
    imu[p_name] += NC
xtal = part(
    "Device",
    "Crystal",
    "Crystal:Crystal_SMD_3215-2Pin_3.2x1.5mm",
    "32.768kHz",
    "C32346",
    tag="xtal",
)
x_in, x_out = Net("XIN32"), Net("XOUT32")
x_in += imu["XIN32"], xtal[1]
x_out += imu["XOUT32"], xtal[2]
x_in & c("22pF", "C1555", "c_xin") & gnd
x_out & c("22pF", "C1555", "c_xout") & gnd

# --- Crossfire Nano RX on a 1x4 male header, its front connector in TBS order: GND (square
# pad), 5V, Ch1 = CRSF TX (to ESP RX, IO18), Ch2 = CRSF RX (from ESP TX, IO17).
# Right-angle SMD header: its pins run parallel to the board, so the RX slides on and stands
# perpendicular to it. A vertical header would stack the RX flat against the board.
j_rx = part(
    "mk2",
    "HXPZ2.54-1X4PWT",
    "mk2:CONN-SMD_HX-PZ2.54-1X4P-WT",
    "NANO_RX",
    "C46061677",
    tag="j_rx",
)
j_rx[1] += gnd
j_rx[2] += v5
mcu["IO18"] & r("100", "C25076", "r_crsf_rx") & j_rx[3]
mcu["IO17"] & r("100", "C25076", "r_crsf_tx") & j_rx[4]

# --- USB-C, vertical: flashing and BLHeli passthrough with the wedge plate off. 5.1k on each CC
# makes it a sink; USBLC6 clamps D+/D- because a person plugs this one in.
# 16-pin USB 2.0 receptacle (JLCPCB stock 2,310; JLCPCB flagged the 24-pin TYPE-C-31-M-06 as hard
# to source).
usb = part("mk2", "TYPE-C-31-D-06", "mk2:USB-C-SMD_TYPE-C-31-D-06", lcsc="C2689964", tag="usb")
dm, dp = Net("USB_DM"), Net("USB_DP")
vbus_usb += usb["A4"], usb["A9"], usb["B4"], usb["B9"]
gnd += usb["A1"], usb["A12"], usb["B1"], usb["B12"], usb["EP"]
dp += usb["A6"], usb["B6"], mcu["IO20"]
dm += usb["A7"], usb["B7"], mcu["IO19"]
for cc, tag in (("A5", "r_cc1"), ("B5", "r_cc2")):
    usb[cc] & r("5.1k", "C25905", tag) & gnd
for name in ("A8", "B8"):
    usb[name] += NC  # SBU
esd = part(
    "Power_Protection", "USBLC6-2SC6", "Package_TO_SOT_SMD:SOT-23-6", lcsc="C2687116", tag="esd"
)
dp += esd["I/O1"]  # both pins of each flow-through pair
dm += esd["I/O2"]
gnd += esd["GND"]
vbus_usb += esd["VBUS"]

# --- ESC signal pads: DShot through 100 ohm, a ground beside each.
for side, io in (("l", "IO8"), ("r", "IO9")):
    sig = Net(f"DSHOT_{side.upper()}")
    mcu[io] & r("100", "C25076", f"r_dshot_{side}") & sig
    sig += pad(f"DSHOT_{side.upper()}", PAD_SIG, f"p_dshot_{side}")[1]
    gnd += pad(f"SIG_GND_{side.upper()}", PAD_SIG, f"p_sig_gnd_{side}")[1]

# GND stitching vias in the stem's sides: the tracks to the stem's USB-C and bulk cap box the
# bottom GND pour there into fragments the flow's stitching found no via spot in.
for i in range(2):
    gnd += pad("GND_STITCH", "mk2:StitchVia_0.6mm", f"v_stitch{i}")[1]

# --- Mounting: four 3.7 mm holes for #6 flat-head plastite screws into the chassis bosses.
# The washers are D-cut on their outboard side, so each side has its own footprint.
for i, side in enumerate("LLRR"):
    h = Part(
        "Mechanical",
        "MountingHole",
        footprint=f"mk2:MountingHole_3.7mm_Washer9mm_Dcut_{side}",
        tag=f"h{i}",
    )
    h.value = f"MountingHole_{side}"

used = {
    "3V3",
    "GND",
    "EN",
    "IO0",
    "IO41",
    "IO40",
    "IO39",
    "IO17",
    "IO18",
    "IO8",
    "IO9",
    "IO19",
    "IO20",
}
for pin in mcu.pins:
    if pin.name not in used:
        pin += NC

# Name the nets SKiDL left as N$n, so the schematic and the board read by function: a net on an
# MCU pin takes the pin's name, the rest are named here.
NAMES = {
    (buck, "BOOT"): "BUCK_BOOT",
    (imu, "~{RESET}"): "IMU_RST",
    (imu, "~{BOOT_LOAD_PIN}"): "IMU_BOOTLOAD",
    (imu, "CAP"): "IMU_CAP",
    (led, "DI"): "LED_DIN",
    (usb, "A5"): "USB_CC1",
    (usb, "B5"): "USB_CC2",
    (j_rx, 3): "CRSF_RX_TX",
    (j_rx, 4): "CRSF_RX_RX",
}
for (p, pin), name in NAMES.items():
    p[pin].net.name = name
for pin in mcu.pins:
    if pin.is_connected() and pin.net.name.startswith("N$"):
        pin.net.name = pin.name

ERC()
generate_netlist(file_="mk2.net")
