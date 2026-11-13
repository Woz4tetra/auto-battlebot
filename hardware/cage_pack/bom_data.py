"""Purchase data for the cage pack BOM: names, links and unit prices, checked 2026-11-08.

Prices are USD, quantity 1 unless the note says otherwise. Pack items (McMaster screws,
Amazon kits) carry the per-unit price so the BOM subtotal counts only what the build uses;
the note gives the pack size and pack price, which is what the order actually costs.
Owned parts have no price, so the BOM total is the cost to buy what is missing.

Sources: Digi-Key and McMaster product pages, 18650BatteryStore, Stereolabs store, Amazon
search results (first relevant listing; Amazon prices move daily). Mouser blocks automated
browsing and Mini-Box shows no OpenUPS price without a cart, so neither is priced here.
"""

AMZ = "https://www.amazon.com/dp/"
DK = "https://www.digikey.com/en/products/detail/"
MC = "https://www.mcmaster.com/"

# Keyed by assembly component name.
COMPONENTS: dict[str, dict] = {
    "cup": {
        "name": 'SmallRig 4122C Suction Cup Camera Mount (4")',
        "vendor": "B&H",
        "part_number": "SM4122C",
        "url": "https://www.bhphotovideo.com/c/product/1923062-REG/smallrig_4122c_suction_cup_camera_mount.html",
        "price": 59.99,
    },
    "zed_box_mini": {
        "name": "Stereolabs ZED Box Mini, Orin NX 16GB with WiFi",
        "vendor": "Stereolabs",
        "part_number": "ZBX-450110",
        "url": "https://www.stereolabs.com/store/products/zed-box-mini",
        "note": "Owned. From $999 at Stereolabs",
    },
    "camera": {
        "name": "Stereolabs ZED X One S",
        "vendor": "Stereolabs",
        "url": "https://www.stereolabs.com/store/products/zed-x-one-s",
        "note": "Owned. $429 at Stereolabs",
    },
    "front_mount": {
        "name": "Stereolabs Camera Mount for ZED X One S, Front",
        "vendor": "Stereolabs",
        "part_number": "ACC-151300",
        "url": "https://www.stereolabs.com/store/products/camera-mount-for-zed-x-one-s",
        "note": "Owned. $19 at Stereolabs",
    },
    "c11_gmsl": {
        "name": "Stereolabs GMSL2 FAKRA cable F-F, 0.3 m",
        "vendor": "Stereolabs",
        "part_number": "CBL-320100",
        "url": "https://www.stereolabs.com/store/products/gmsl2-fakra-cables",
        "note": "Owned (the short one from the box setup). $15 at Stereolabs",
    },
    "openups": {
        "name": "Mini-Box OpenUPS 6-30 V UPS, 1-6S balancing charger",
        "vendor": "Mini-Box",
        "part_number": "PWR-OpenUPS",
        "url": "https://www.mini-box.com/OpenUPS",
        "note": "Owned. Includes the J4 battery and J6 balance cables",
    },
    "ball_head": {
        "name": "SmallRig 2948B Mini Ball Head, 2-pack",
        "vendor": "Amazon",
        "part_number": "B09G9PFQCN",
        "url": AMZ + "B09G9PFQCN",
        "price": 18.90,
        "note": "One head used, one spare",
    },
    "power_button": {
        "name": "CIT AH1LBBG12 19 mm latching pushbutton, black, green 12 V ring",
        "vendor": "Digi-Key",
        "part_number": "2449-AH1LBBG12-ND",
        "url": DK + "cit-relay-and-switch/AH1LBBG12/17127303",
        "price": 25.95,
        "note": "SPST-NO + SPST-NC, signal only. 4 in stock 2026-11-09, else 9 weeks",
    },
    "charger": {
        "name": "Mean Well GST120A24-P1M, 24 V 5 A desktop supply",
        "vendor": "Digi-Key",
        "part_number": "1866-2040-ND",
        "url": DK + "mean-well-usa-inc/GST120A24-P1M/7703595",
        "price": 44.70,
        "note": "Cut the barrel plug off, fit the GX12 plug. Not carried on the pack",
    },
    "battery": {
        "name": "Molicel P30B 18650, 3000 mAh 36 A",
        "vendor": "18650BatteryStore",
        "part_number": "P30B",
        "url": "https://www.18650batterystore.com/products/molicel-p30b-18650-3000mah-36a-battery",
        "price": 8.99,
        "qty": 4,
        "note": "4S1P, 43.2 Wh, in the BK-18650-PC8 holder. $8.00 each at 6+",
    },
    "battery_holder": {
        "name": "MPD BK-18650-PC8, 4 x 18650 holder, PC pins",
        "vendor": "Digi-Key",
        "part_number": "BK-18650-PC8-ND",
        "url": DK + "mpd-memory-protection-devices/BK-18650-PC8/2330515",
        "price": 6.22,
        "note": "952 in stock 2026-11-10. Cells side by side; the carrier PCB wires them in series",
    },
    "display": {
        "name": "Waveshare 2.8inch Capacitive Touch LCD, 320 x 240, ST7789 SPI + I2C touch",
        "vendor": "Waveshare",
        "part_number": "27579",
        "url": "https://www.waveshare.com/2.8inch-Capacitive-Touch-LCD.htm",
        "price": 16.99,
        "note": "Checked 2026-11-11. V1 touch IC CST328, V2 CST3530 (V2 label on the back)",
    },
    "battery_carrier": {
        "name": "Battery carrier PCB, 80 x 80 2-layer bare board",
        "vendor": "JLCPCB",
        "url": "https://jlcpcb.com/",
        "price": 2.0,
        "note": "Estimate: add to the PCBA order. Series links, B- B1 B2 B3 B+ pads, 2 oz copper",
    },
}

# Cabling and panel parts that have no solid of their own in the assembly.
EXTRAS: list[dict] = [
    {
        "name": "Inline ATC fuse holder 16 AWG, 4-pack with fuses (fit 10 A)",
        "vendor": "Amazon",
        "part_number": "B0F6NPV287",
        "url": AMZ + "B0F6NPV287",
        "price": 7.99,
    },
    {
        "name": "XT30U male + female, 5 pairs",
        "vendor": "Amazon",
        "part_number": "B0DB5DPDB8",
        "url": AMZ + "B0DB5DPDB8",
        "price": 5.99,
    },
    {
        "name": "Battery straps 20 x 200 mm hook-and-loop, 5-pack",
        "vendor": "Amazon",
        "part_number": "B019QWZYU8",
        "url": AMZ + "B019QWZYU8",
        "price": 8.99,
    },
    {
        "name": "16 AWG silicone zip cord red/black, 25 ft",
        "vendor": "Amazon",
        "part_number": "B077B5HFXN",
        "url": AMZ + "B077B5HFXN",
        "price": 19.98,
        "note": "OpenUPS VOUT to ZED power",
    },
    {
        "name": "GX12 2-pin panel connector + plug",
        "vendor": "Amazon",
        "part_number": "B089YT6LFN",
        "url": AMZ + "B089YT6LFN",
        "price": 9.99,
        "note": "Charge input",
    },
    {
        "name": "Mean Well YP12_YC12 mains cord, NEMA 5-15P to IEC C13, 1.8 m",
        "vendor": "Digi-Key",
        "url": "https://www.digikey.com/en/products/result?keywords=YP12_YC12",
        "price": 7.11,
        "note": "For the charger; listed beside it on Digi-Key",
    },
    {
        "name": "2-pin 0.1 in crimp housings + Dupont pins",
        "vendor": "Amazon",
        "url": "https://www.amazon.com/s?k=dupont+connector+kit+2.54",
        "note": "J6 lead onto the OpenUPS J8 and J10 headers. Price not checked",
    },
    {
        "name": "Hirose HR10A-7R-4S(73) receptacle, 4p female",
        "vendor": "Digi-Key",
        "part_number": "HR1590-ND",
        "url": DK + "hirose-electric-co-ltd/HR10A-7R-4S-73/1095448",
        "price": 10.55,
        "note": "Transmitter port",
    },
    {
        "name": "Hirose HR10A-7P-4P(73) cable plug, 4p male",
        "vendor": "Digi-Key",
        "url": DK + "hirose-electric-co-ltd/HR10A-7P-4P-73/1095440",
        "price": 19.62,
        "note": "Transmitter bridge cable",
    },
    {
        "name": "USB-A to USB-C, 90°, 25 cm",
        "vendor": "Amazon",
        "part_number": "B0B8N8DDPW",
        "url": AMZ + "B0B8N8DDPW",
        "price": 9.98,
        "note": "ZED to PCBA. Check which end is angled before ordering",
    },
    {
        "name": "JST GH 1.25 pre-crimped cable kit",
        "vendor": "Amazon",
        "part_number": "B07PBHN7TM",
        "url": AMZ + "B07PBHN7TM",
        "price": 18.99,
        "note": "PCBA J2-J6 leads (GH 4 and 6 pin)",
    },
    {
        "name": "u.FL to RP-SMA female bulkhead pigtail, 8 in",
        "vendor": "Amazon",
        "part_number": "B0CS6KBDJJ",
        "url": AMZ + "B0CS6KBDJJ",
        "price": 7.99,
        "note": "ESP32-S3-WROOM-1U antenna through the enclosure wall",
    },
    {
        "name": "2.4 GHz RP-SMA antenna, 2-pack",
        "vendor": "Amazon",
        "part_number": "B01KBU61S8",
        "url": AMZ + "B01KBU61S8",
        "price": 8.59,
        "note": "ESP32 antenna",
    },
    {
        "name": "JST GHR-13V-S housing, 2 + GH pre-crimped 28 AWG leads, 13",
        "vendor": "Digi-Key",
        "url": "https://www.digikey.com/en/products/result?keywords=GHR-13V-S",
        "note": "Display to PCBA J7, pin 1 to pin 1. Price not checked",
    },
    {
        "name": '1/8" stainless cable lanyard, eye loops, vinyl coated',
        "vendor": "Amazon",
        "part_number": "B07X1J4FTF",
        "url": AMZ + "B07X1J4FTF",
        "price": 13.99,
        "note": "Safety tether to the cage frame. Pick the 18 in length",
    },
]

# The custom PCBA as one line, priced from the last JLCPCB order rather than part by part:
# the Mr Stabs Mk2 board (W2026100921070917, 2026-10-09) cost $452.79 delivered for 5 bare
# PCBs with 2 assembled ($289.78 boards and assembly, $39.46 DHL, $105.43 duty, $18.12 tax),
# about $226 per assembled board. That board had 30 BOM lines; this one has about 20, so
# expect a little less. Re-quote at JLCPCB once the board is laid out.
PCBA: dict = {
    "name": "Custom PCBA, 4-layer, assembled (JLCPCB), estimate",
    "vendor": "JLCPCB",
    "url": "https://jlcpcb.com/",
    "price": 226.0,
    "note": "Estimate: one of two assembled boards from the Mk2 order's $452.79 total",
}

# McMaster, keyed by part number: (per-unit price, pack size, pack price).
MCMASTER: dict[str, tuple[float, int, float]] = {
    "91255A619": (18.43 / 50, 50, 18.43),  # 3/8-16 x 1/2" alloy button head, black oxide
    "92311A537": (13.28 / 100, 100, 13.28),  # 1/4-20 x 1/2" 18-8 cup-point set screw
    "91292A111": (11.58 / 100, 100, 11.58),  # M3 x 6 18-8 SHCS
    "91292A112": (6.46 / 100, 100, 6.46),  # M3 x 8 18-8 SHCS
    "91292A115": (8.44 / 100, 100, 8.44),  # M3 x 16 18-8 SHCS
    "92095A181": (8.80 / 100, 100, 8.80),  # M3 x 8 18-8 button head
    "92095A182": (8.64 / 100, 100, 8.64),  # M3 x 10 18-8 button head
    "94180A331": (20.44 / 100, 100, 20.44),  # M3 brass tapered heat-set, 3.8 mm
    "94180A333": (27.84 / 100, 100, 27.84),  # M3 brass tapered heat-set, 6.4 mm
    "93365A160": (15.76 / 25, 25, 15.76),  # 1/4-20 brass tapered heat-set, 0.300"
    "92141A051": (9.25 / 100, 100, 9.25),  # 3/8" 18-8 oversized washer, 1" OD, 0.040-0.060"
}


def mcmaster(pn: str) -> dict:
    """BOM keys for a McMaster item placed as a component (a component's own "mcmaster" key
    would make the skill load its STEP instead)."""
    if pn not in MCMASTER:  # not priced yet
        return {
            "vendor": "McMaster-Carr",
            "part_number": pn,
            "url": MC + pn,
            "note": "Price not checked",
        }
    unit, pack, pack_price = MCMASTER[pn]
    return {
        "vendor": "McMaster-Carr",
        "part_number": pn,
        "url": MC + pn,
        "price": round(unit, 3),
        "note": f"Pack of {pack}, ${pack_price:.2f}",
    }
