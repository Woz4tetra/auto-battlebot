"""Onshape names and part properties for the cage pack, keyed by assembly component or
fastener group (cage_pack_assembly.ONSHAPE_PARTS).

Each entry sets the Part Studio tab, part and (for a purchased part) document name, `title`,
upper case the Onshape way, and the part's properties: Part number (the manufacturer's
part number), Vendor (the manufacturer) and Description (what it is and where it was
bought). `home` is "own" for an off-the-shelf part, which lives in its own document, and
"main" for a part designed here. `bom: False` sets "Exclude from all BOMs" (the cage wall,
the PCBA's parts, which are priced with the board).

Components with congruent shapes (the two cups, every M3 insert) share one entry's title
and become one Part Studio with several instances.

Sources: manufacturer pages, Digi-Key and McMaster listings, and for Amazon items the
listing's brand (from search snippets, checked 2026-11-11). Where a maker gives no part
number the Amazon ASIN stands in, and the description says so.
"""

AMZ = "No manufacturer part number; the part number is the Amazon ASIN."


def _mc(pn: str, title: str, desc: str) -> dict:
    return {"title": f"MCMASTER {pn} {title}", "vendor": "McMaster-Carr", "part_number": pn,
            "description": desc, "home": "own"}  # fmt: skip


def _own(title: str, vendor: str, pn: str, desc: str) -> dict:
    return {"title": title, "vendor": vendor, "part_number": pn, "description": desc, "home": "own"}


def _main(title: str, desc: str = "", bom: bool = True) -> dict:
    return {"title": title, "description": desc, "home": "main", "bom": bom}


_insert = _mc(
    "94180A333",
    "M3 HEAT-SET INSERT",
    "Brass tapered heat-set insert for plastic, M3 x 0.5, 6.4 mm long.",
)
_cup = _own(
    "SMALLRIG 4122C SUCTION CUP MOUNT",
    "SmallRig",
    "4122C",
    "4 in suction cup camera mounting support. Bought from B&H (SM4122C).",
)
_fakra = _own(
    "ROSENBERGER 59Z114-000Z FAKRA Z PLUG",
    "Rosenberger",
    "59Z114-000Z",
    "FAKRA Z-code plug housing, as on the Stereolabs CBL-320100 GMSL2 cable.",
)
_mf4 = _own(
    "MOLEX 39012040 MINI-FIT JR 4-PIN RECEPTACLE",
    "Molex",
    "39012040",
    "Mini-Fit Jr receptacle housing, dual row, 4 circuits.",
)
_dupont = _own(
    "DUPONT 2-PIN 2.54 MM HOUSING",
    "Generic",
    "1x2 2.54 mm crimp housing",
    "2-pin 0.1 in crimp housing from a Dupont kit. No manufacturer part number.",
)
_gh4 = _own("JST GHR-04V-S GH 4-PIN HOUSING", "JST", "GHR-04V-S", "GH 1.25 mm housing, 4 circuits.")
_gh13 = _own(
    "JST GHR-13V-S GH 13-PIN HOUSING", "JST", "GHR-13V-S", "GH 1.25 mm housing, 13 circuits."
)
_usb = ("GELRHONR", "GELRHONR-025M-90A-C")
_antenna = _own(
    "L-COM HG2402RD-RSF 2.4 GHZ ANTENNA",
    "L-com",
    "HG2402RD-RSF",
    "Tilt-swivel 2.4 GHz RP-SMA antenna. Stand-in model for the HIGHFINE 2.4GHz 6dBi "
    "antenna bought (Amazon B01KBU61S8), which has no published model.",
)

PARTS: dict[str, dict] = {
    # Designed here
    "enclosure": _main("ENCLOSURE", "Enclosure tray, black PETG, Bambu P1S."),
    "enclosure_lid": _main("ENCLOSURE LID", "Enclosure lid, black PETG."),
    "battery_clamp": _main("BATTERY CLAMP", "Battery clamp and OpenUPS standoffs, black PETG."),
    "camera_shield": _main("CAMERA SHIELD", "Camera light shield, black PETG."),
    "pcba": _main("PCBA", "Custom PCBA, JLCPCB (space claim)."),
    "display_bracket": _main("DISPLAY BRACKET", "Display bracket, black PETG."),
    "camera_bracket": _main("CAMERA BRACKET", "ZED X One S bracket, PLA."),
    "cup_arm_left": _main("CUP ARM", "Cup arm, black PETG, printed on its side. Two per pack."),
    "cup_arm_right": _main("CUP ARM", "Cup arm, black PETG, printed on its side. Two per pack."),
    "battery_carrier": _main("BATTERY CARRIER PCB", "80 x 80 2-layer bare board, JLCPCB."),
    "pcba_parts": _main(
        "PCBA COMPONENTS", "Parts on the custom PCBA, priced with the board.", bom=False
    ),
    "glass_outer": _main(
        "CAGE SHEET", "NHRL cage wall, 1/4 in polycarbonate. Reference only.", bom=False
    ),
    "glass_inner": _main(
        "CAGE SHEET", "NHRL cage wall, 1/4 in polycarbonate. Reference only.", bom=False
    ),
    "retainer": _main(
        "CAGE RETAINER PLATE", "Cage wall retainer (assumed). Reference only.", bom=False
    ),
    "c4_zed_power": _main("ZED POWER LEAD", "OpenUPS VOUT to ZED Box Mini power, 2 x 16 AWG."),
    "battery_lead": _main("BATTERY LEAD", "Battery to OpenUPS BAT (J4): XT30, 10 A fuse."),
    "button_lead": _main("BUTTON LEAD", "Power button to PCBA J5, 28 AWG."),
    "ups_ctrl_lead": _main("UPS CONTROL LEAD", "PCBA J6 to OpenUPS J8, plus VBAT and GND."),
    "ups_ctrl_j10": _main("UPS CONTROL J10 BRANCH", "J6 lead's branch to OpenUPS J10."),
    "charge_lead": _main("CHARGE LEAD", "GX12 to OpenUPS VIN (J1), 2 x 16 AWG."),
    "display_lead": _main("DISPLAY LEAD", "Display GH 13-pin to PCBA J7, 28 AWG."),
    "charger_cord": _main(
        "CHARGER DC CORD", "Mean Well DC cord, barrel plug cut off, GX12 plug fitted."
    ),
    # Bought
    "cup_left": _cup,
    "cup_right": _cup,
    "tray_inserts": _insert,
    "lid_inserts": _insert,
    "shield_insert": _insert,
    "cup_washers": _mc(
        "92141A051",
        "3/8 IN OVERSIZED WASHER",
        "18-8 stainless oversized washer for 3/8 in screws, 1 in OD.",
    ),
    "zed_box_mini": _own(
        "STEREOLABS ZBX-450110 ZED BOX MINI",
        "Stereolabs",
        "ZBX-450110",
        "ZED Box Mini, Jetson Orin NX 16GB with WiFi.",
    ),
    "battery": _own(
        "MOLICEL INR-18650-P30B CELL",
        "Molicel",
        "INR-18650-P30B",
        "18650 Li-ion cell, 3000 mAh, 36 A. Bought from 18650BatteryStore.",
    ),
    "battery_holder": _own(
        "MPD BK-18650-PC8 BATTERY HOLDER",
        "MPD",
        "BK-18650-PC8",
        "4 x 18650 holder, PC pins. Digi-Key BK-18650-PC8-ND.",
    ),
    "openups": _own(
        "MINI-BOX PWR-OPENUPS UPS",
        "Mini-Box",
        "PWR-OpenUPS",
        "OpenUPS 6-30 V UPS with 1-6S balancing charger.",
    ),
    "display": _own(
        "WAVESHARE 27579 2.8IN CAPACITIVE TOUCH LCD",
        "Waveshare",
        "27579",
        "2.8inch Capacitive Touch LCD, 320 x 240, ST7789 SPI and I2C touch.",
    ),
    "charger": _own(
        "MEAN WELL GST120A24-P1M POWER SUPPLY",
        "Mean Well",
        "GST120A24-P1M",
        "24 V 5 A desktop supply. Digi-Key 1866-2040-ND.",
    ),
    "ball_head": _own(
        "SMALLRIG 2948B MINI BALL HEAD",
        "SmallRig",
        "2948B",
        "Mini ball-head mounting support kit, 2 pcs, one used. Amazon B09G9PFQCN.",
    ),
    "front_mount": _own(
        "STEREOLABS ACC-151300 ZED X ONE S FRONT MOUNT",
        "Stereolabs",
        "ACC-151300",
        "Camera Mount for ZED X One S, Front.",
    ),
    "camera": _own(
        "STEREOLABS ZED-412012 ZED X ONE S", "Stereolabs", "ZED-412012", "ZED X One S, wide lens."
    ),
    "m4_inserts": _mc(
        "94180A353",
        "M4 HEAT-SET INSERT",
        "Brass tapered heat-set insert for plastic, M4 x 0.7, 7.9 mm long.",
    ),
    "m4_washers": _mc(
        "93475A230", "M4 WASHER", "18-8 stainless washer for M4 screws, 9 mm OD, 0.8 mm thick."
    ),
    "quarter_inserts": _mc(
        "93365A160",
        "1/4-20 HEAT-SET INSERT",
        "Brass tapered heat-set insert for plastic, 1/4-20, 0.300 in long.",
    ),
    "tbp_plug": _own(
        "SAME SKY TBP01P1W-508-02GR TERMINAL BLOCK PLUG",
        "Same Sky",
        "TBP01P1W-508-02GR",
        "Pluggable terminal block plug, 2 position, 5.08 mm, locking flanges. Mates the "
        "ZED Box Mini's DC input header.",
    ),
    "usb_a_plug": _own(
        "GELRHONR USB-A 90 DEG PLUG",
        *_usb,
        "USB-A plug end of the 0.25 m right-angle USB-A to USB-C cable (Amazon B0B8N8DDPW).",
    ),
    "usb_c_plug": _own(
        "GELRHONR USB-C 90 DEG PLUG",
        *_usb,
        "USB-C plug end of the 0.25 m right-angle USB-A to USB-C cable (Amazon B0B8N8DDPW).",
    ),
    "c6_usb": _own(
        "GELRHONR USB-A TO USB-C 90 DEG CABLE",
        *_usb,
        "0.25 m right-angle USB-A to USB-C cable (Amazon B0B8N8DDPW), routed ZED to PCBA.",
    ),
    "fakra_plug_zed": _fakra,
    "fakra_plug_cam": _fakra,
    "c11_gmsl": _own(
        "STEREOLABS CBL-320100 GMSL2 FAKRA CABLE",
        "Stereolabs",
        "CBL-320100",
        "GMSL2 Fakra cable F-F, 0.3 m.",
    ),
    "rj45_plug": _own(
        "MODDIY CO905 RJ45 90 DEG PLUG",
        "ModDIY",
        "CO905",
        "Right-angle RJ45 plug end of the CO905 panel-mount extension.",
    ),
    "rj45_coupler": _own(
        "MODDIY CO905 RJ45 PANEL JACK",
        "ModDIY",
        "CO905",
        "Panel-mount RJ45 jack end of the CO905 extension (modelled from ModDIY's CAB105).",
    ),
    "lan_tail": _own(
        "MODDIY CO905 RJ45 EXTENSION CABLE",
        "ModDIY",
        "CO905",
        "Right-angled RJ45 Ethernet extension cable with panel mount, 30 cm.",
    ),
    "gx12_dc_in": _own(
        "DIYHZ GX12 2-PIN PANEL SOCKET",
        "DIYhz",
        "B089YT6LFN",
        f"GX12 2-pin aviation panel socket. {AMZ}",
    ),
    "gx12_charger_plug": _own(
        "DIYHZ GX12 2-PIN CABLE PLUG",
        "DIYhz",
        "B089YT6LFN",
        f"GX12 2-pin aviation cable plug. {AMZ}",
    ),
    "power_button": _own(
        "CIT AH1LBBG12 PUSHBUTTON",
        "CIT Relay and Switch",
        "AH1LBBG12",
        "19 mm latching anti-vandal pushbutton, green 12 V ring. Digi-Key 2449-AH1LBBG12-ND.",
    ),
    "hr10_rcpt": _own(
        "HIROSE HR10A-7R-4S(73) RECEPTACLE",
        "Hirose",
        "HR10A-7R-4S(73)",
        "HR10 4-pin female panel receptacle. Digi-Key HR1590-ND.",
    ),
    "hr10_tx_plug": _own(
        "HIROSE HR10A-7P-4P(73) PLUG",
        "Hirose",
        "HR10A-7P-4P(73)",
        "HR10 4-pin male cable plug, transmitter bridge cable.",
    ),
    "sma_bulkheads": _own(
        "RYDOCYEE RP-SMA FEMALE BULKHEAD",
        "Rydocyee",
        "B0CS6KBDJJ",
        f"RP-SMA female bulkhead end of a u.FL or RP-SMA pigtail. {AMZ}",
    ),
    **{f"antenna_{n}": _antenna for n in ("zed_wifi_1", "zed_wifi_2", "esp32")},
    "wifi_plugs": _own(
        "RP-SMA MALE 90 DEG PLUG",
        "Not sourced",
        "Not sourced",
        "90 deg RP-SMA plug of the ZED WiFi extensions. Not sourced yet.",
    ),
    "wifi_pigtails": _own(
        "RP-SMA EXTENSION CABLE 15 CM",
        "Not sourced",
        "Not sourced",
        "RP-SMA male to RP-SMA female bulkhead extension, 15 cm. Not sourced yet.",
    ),
    "esp_pigtail": _own(
        "RYDOCYEE U.FL TO RP-SMA BULKHEAD PIGTAIL",
        "Rydocyee",
        "B0CS6KBDJJ",
        f"u.FL to RP-SMA female bulkhead pigtail, 8 in. {AMZ}",
    ),
    "tether": _own(
        "BYTIYAR 1/8 IN CABLE LANYARD",
        "Bytiyar",
        "B07X1J4FTF",
        f"1/8 in stainless cable lanyard, eye loops, vinyl coated. {AMZ}",
    ),
    "minifit_vout": _mf4,
    "minifit_vin": _mf4,
    "minifit_bat": _own(
        "MOLEX 39012020 MINI-FIT JR 2-PIN RECEPTACLE",
        "Molex",
        "39012020",
        "Mini-Fit Jr receptacle housing, dual row, 2 circuits.",
    ),
    "dupont_J10": _dupont,
    "dupont_J8": _dupont,
    "mini_b_plug": _own(
        "USB MINI-B 90 DEG PLUG",
        "Generic",
        "Mini-B 90 deg overmold",
        "Right-angle USB mini-B plug on the OpenUPS USB lead. No manufacturer part number.",
    ),
    "gh_J2": _gh4,
    "gh_J3": _gh4,
    "gh_J5": _gh4,
    "gh_J4": _own(
        "JST GHR-02V-S GH 2-PIN HOUSING", "JST", "GHR-02V-S", "GH 1.25 mm housing, 2 circuits."
    ),
    "gh_J6": _own(
        "JST GHR-06V-S GH 6-PIN HOUSING", "JST", "GHR-06V-S", "GH 1.25 mm housing, 6 circuits."
    ),
    "gh_J7": _gh13,
    "gh_display": _gh13,
    # Fasteners (the M3 x 6 under the ZED is Onshape Standard Content)
    "cup_screws": _mc(
        "91255A621",
        "3/8-16 X 5/8 BUTTON HEAD SCREW",
        "Black-oxide alloy steel button head hex drive screw, 3/8-16 x 5/8 in. "
        "Part number from a distributor listing: confirm on mcmaster.com.",
    ),
    "m3_cup_arms": _mc(
        "91292A114",
        "M3 X 12 SOCKET HEAD SCREW",
        "18-8 stainless socket head screw, M3 x 0.5 x 12 mm.",
    ),
    "m3_pcba": _mc(
        "91292A112",
        "M3 X 8 SOCKET HEAD SCREW",
        "18-8 stainless socket head screw, M3 x 0.5 x 8 mm.",
    ),
    "m3_display_bracket": _mc(
        "91292A112",
        "M3 X 8 SOCKET HEAD SCREW",
        "18-8 stainless socket head screw, M3 x 0.5 x 8 mm.",
    ),
    "m3_openups": _mc(
        "91292A115",
        "M3 X 16 SOCKET HEAD SCREW",
        "18-8 stainless socket head screw, M3 x 0.5 x 16 mm.",
    ),
    "m3_lid": _mc(
        "92095A181",
        "M3 X 8 BUTTON HEAD SCREW",
        "18-8 stainless button head hex drive screw, M3 x 0.5 x 8 mm.",
    ),
    "m3_shield": _mc(
        "92095A181",
        "M3 X 8 BUTTON HEAD SCREW",
        "18-8 stainless button head hex drive screw, M3 x 0.5 x 8 mm.",
    ),
    "m25_display": _own(
        "ISO 4762 M2.5 X 6 SOCKET HEAD SCREW",
        "Any",
        "ISO 4762 M2.5x6 A2",
        "Stainless socket head cap screw, M2.5 x 0.45 x 6 mm. McMaster number not picked yet.",
    ),
    "head_screw": _mc(
        "91251A542",
        "1/4-20 X 1 SOCKET HEAD SCREW",
        "Black-oxide alloy steel socket head screw, 1/4-20 x 1 in.",
    ),
    "m4_front_mount": _mc(
        "91292A117",
        "M4 X 12 SOCKET HEAD SCREW",
        "18-8 stainless socket head screw, M4 x 0.7 x 12 mm.",
    ),
}
