"""Mr Stabs Mk2 main board netlist (SKiDL)."""

import builtins
import os

from skidl import ERC, KICAD9, Net, Part, generate_netlist, lib_search_paths, set_default_tool

set_default_tool(KICAD9)
NC = builtins.NC
HERE = os.path.dirname(os.path.abspath(__file__))
POWER_DRIVE = 3
SBU_PINS = ("A8", "B8")
lib_search_paths[KICAD9].append(os.path.join(HERE, "lib"))

R0402 = "Resistor_SMD:R_0402_1005Metric"
C0402 = "Capacitor_SMD:C_0402_1005Metric"
C0603 = "Capacitor_SMD:C_0603_1608Metric"
C0805 = "Capacitor_SMD:C_0805_2012Metric"
PAD_12AWG = "mk2:WirePad_Lap_3.6x5.0mm"
PAD_16AWG = "mk2:WirePad_Lap_3.0x5.0mm"
PAD_SIG = "mk2:WirePad_Lap_1.6x3.0mm"


def part(lib, name, fp, value=None, lcsc=None, *, tag):
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
    n.drive = POWER_DRIVE
sda, scl = Net("SDA1"), Net("SCL1")

gnd += pad("PACK_A-", "mk2:WirePad_Lap_3.6x5.0mm_Vias", "p_pack_a_neg")[1]
pack_mid += pad("PACK_A+", PAD_12AWG, "p_pack_a_pos")[1]
pack_mid += pad("PACK_B-", PAD_12AWG, "p_pack_b_neg")[1]
pack_pos += pad("PACK_B+", PAD_12AWG, "p_pack_b_pos")[1]
pack_pos += pad("SW_OUT", PAD_12AWG, "p_sw_out")[1]
bat_in += pad("SW_BACK", "mk2:WirePad_Lap_3.6x5.0mm_Vias", "p_sw_back")[1]

shunt = part("Device", "R", "mk2:R_Yezhan_ASR3_2512", "0.5m", "C469426", tag="r_shunt")
bat_in += shunt[1]
vbatt += shunt[2]
for side in ("l", "r"):
    vbatt += pad(f"ESC_{side.upper()}+", PAD_16AWG, f"p_esc_{side}_pos")[1]
    gnd += pad(f"ESC_{side.upper()}-", "mk2:WirePad_Lap_3.0x5.0mm_Vias", f"p_esc_{side}_neg")[1]

ina = part(
    "Sensor_Energy", "INA238", "Package_SO:VSSOP-10_3x3mm_P0.5mm", lcsc="C2868250", tag="ina"
)
sense_p, sense_n = Net("SENSE_P"), Net("SENSE_N")
sense_p += ina["Vin+"]
sense_n += ina["Vin-"]
bat_in & r("10", "C25077", "r_sense_p") & sense_p
vbatt & r("10", "C25077", "r_sense_n") & sense_n
sense_p & c("100nF", "C1525", "c_sense") & sense_n
bat_in += ina["VBUS"]
v3v3 += ina["VS"], ina["A0"], ina["A1"]
gnd += ina["GND"]
sda += ina["SDA"]
scl += ina["SCL"]
ina["~{Alert}"] += NC
v3v3 & c("100nF", "C1525", "c_ina") & gnd

bulk = part(
    "Device", "C_Polarized", "Capacitor_SMD:CP_Elec_6.3x7.7", "100uF 35V", "C88744", tag="c_bulk"
)
vbatt += bulk[1]
gnd += bulk[2]

buck = part(
    "Regulator_Switching", "TPS54202DDC", "Package_TO_SOT_SMD:SOT-23-6", lcsc="C191884", tag="buck"
)
vbatt += buck["VIN"]
gnd += buck["GND"]
buck["EN"] += NC
for i in (1, 2):
    vbatt & c("10uF", "C15850", f"c_buck_in{i}", C0805) & gnd
vbatt & c("100nF", "C307331", "c_buck_hf") & gnd
sw = Net("SW")
sw += buck["SW"]
buck["BOOT"] & c("100nF", "C1525", "c_boot") & sw
ind = part("Device", "L", "Inductor_SMD:L_Taiyo-Yuden_NR-40xx", "15uH", "C167881", tag="l_buck")
sw & ind & v5_buck
for i in (1, 2, 3):
    v5_buck & c("22uF", "C45783", f"c_buck_out{i}", C0805) & gnd
fb = Net("FB")
fb += buck["FB"]
v5_buck & r("100k", "C25741", "r_fb_top") & fb
v5_buck & c("47pF", "C1567", "c_ff") & fb
fb & r("12k", "C25752", "r_fb_bot") & gnd

d_buck = part("Device", "D_Schottky", "Diode_SMD:D_SOD-123", "B5819W", "C8598", tag="d_buck")
d_usb = part("Device", "D_Schottky", "Diode_SMD:D_SOD-123", "B5819W", "C8598", tag="d_usb")
v5_buck += d_buck["A"]
v5 += d_buck["K"]
vbus_usb += d_usb["A"]
v5 += d_usb["K"]
v5 & c("10uF", "C19702", "c_5v", C0603) & gnd

ldo = part(
    "Regulator_Linear", "AP2112K-3.3", "Package_TO_SOT_SMD:SOT-23-5", lcsc="C51118", tag="ldo"
)
v5 += ldo["VIN"], ldo["EN"]
v3v3 += ldo["VOUT"]
gnd += ldo["GND"]
v5 & c("1uF", "C52923", "c_ldo_in") & gnd
v3v3 & c("10uF", "C19702", "c_ldo_out", C0603) & gnd

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

led = part(
    "mk2", "XL-2020RGBC-WS2812B", "mk2:LED-SMD_4P-L2.0-W2.0-BR-1", lcsc="C5349955", tag="led"
)
v5 += led["VDD"]
gnd += led["GND"]
mcu["IO39"] & r("100", "C25076", "r_led") & led["DI"]
led["DO"] += NC
v5 & c("100nF", "C1525", "c_led") & gnd

imu = part(
    "Sensor_Motion", "BNO055", "Package_LGA:LGA-28_5.2x3.8mm_P0.5mm", lcsc="C93216", tag="imu"
)
v3v3 += imu["VDD"], imu["VDDIO"]
gnd += imu["GND"], imu["GNDIO"], imu["PS0"], imu["PS1"], imu["COM2"], imu["COM3"]
sda += imu["COM0"]
scl += imu["COM1"]
for p_name, tag in (("~{RESET}", "r_imu_rst"), ("~{BOOT_LOAD_PIN}", "r_imu_boot")):
    v3v3 & r("10k", "C25744", tag) & imu[p_name]
imu["CAP"] & c("1uF", "C52923", "c_imu_cap") & gnd
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

usb = part("mk2", "TYPE-C-31-D-06", "mk2:USB-C-SMD_TYPE-C-31-D-06", lcsc="C2689964", tag="usb")
dm, dp = Net("USB_DM"), Net("USB_DP")
vbus_usb += usb["A4"], usb["A9"], usb["B4"], usb["B9"]
gnd += usb["A1"], usb["A12"], usb["B1"], usb["B12"], usb["EP"]
dp += usb["A6"], usb["B6"], mcu["IO20"]
dm += usb["A7"], usb["B7"], mcu["IO19"]
for cc, tag in (("A5", "r_cc1"), ("B5", "r_cc2")):
    usb[cc] & r("5.1k", "C25905", tag) & gnd
for name in SBU_PINS:
    usb[name] += NC
esd = part(
    "Power_Protection", "USBLC6-2SC6", "Package_TO_SOT_SMD:SOT-23-6", lcsc="C2687116", tag="esd"
)
dp += esd["I/O1"]
dm += esd["I/O2"]
gnd += esd["GND"]
vbus_usb += esd["VBUS"]

for side, io in (("l", "IO8"), ("r", "IO9")):
    sig = Net(f"DSHOT_{side.upper()}")
    mcu[io] & r("100", "C25076", f"r_dshot_{side}") & sig
    sig += pad(f"DSHOT_{side.upper()}", PAD_SIG, f"p_dshot_{side}")[1]
    gnd += pad(f"SIG_GND_{side.upper()}", PAD_SIG, f"p_sig_gnd_{side}")[1]

for i in range(2):
    gnd += pad("GND_STITCH", "mk2:StitchVia_0.6mm", f"v_stitch{i}")[1]

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


def _name_unnamed_nets():
    for (p, pin), name in NAMES.items():
        p[pin].net.name = name
    for pin in mcu.pins:
        if pin.is_connected() and pin.net.name.startswith("N$"):
            pin.net.name = pin.name


_name_unnamed_nets()

ERC()
generate_netlist(file_="mk2.net")
