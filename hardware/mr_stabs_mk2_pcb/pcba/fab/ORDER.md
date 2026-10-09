# Ordering the Mr Stabs Mk2 board (JLCPCB)

Regenerate first if anything changed: `../make_fab.sh` (it refuses to write these files unless
the rating, datasheet and stock gates pass).

## Upload

| File | Where |
| --- | --- |
| `gerbers.zip` | PCB order page |
| `bom.csv` | Assembly, BOM step |
| `cpl.csv` | Assembly, placement step |

## PCB options

| Option | Value | Why |
| --- | --- | --- |
| Layers | 4 | F signal, In1 GND, In2 GND, B signal |
| Thickness | 1.6 mm | The USB-C drawing assumes 0.8 mm; its shell legs stop 0.65 mm short of the far side, with paste in the slots |
| Outer copper | 2 oz | 20 A continuous, 70 A stalls (`../sim/copper_ir.py`) |
| Inner copper | 0.5 oz (default) | ESC current no longer uses the inner layers |
| Min via | 0.25 mm hole, 0.5 mm pad | Signal vias only; power and GND vias are 0.3 / 0.6. Check the quote for a small-hole surcharge |
| Surface finish | HASL lead-free | |
| Quantity | 5 boards, 2 assembled | |

## Assembly options

- **Gerbers:** `gerbers.zip` must hold 4 copper files (F, In1, In2, B). `fab.sh` stops if it does not.
- **PCBA Qty: 2.** The BOM is per board; JLCPCB multiplies it by this number, which defaults to
  the PCB quantity (5).
- Standard PCBA, **both sides**: C3, C19, C22, H1, H2, R1, R2, R3 and USB1 are on the bottom.
- `cpl.csv` rotations are fitted to JLCPCB's own footprint for each LCSC number
  (`cpl_check.csv` lists KiCad's angle beside the fitted one; 11 differ). Still compare the
  placement preview against it, and tick **Confirm Parts Placement** so JLCPCB sends placement
  images before building: this is the first order on fitted angles.
- 30 BOM lines, 13 extended parts. Parts about $24.60 per board at the last stock check
  (`../stock_report.json`, JLCPCB assembly stock). The ESP32-S3-MINI-1U had only 140 left.

## Bought separately

- Molex 146153-0050 Wi-Fi antenna (U.FL, 50 mm cable), one per board.
- XT60 pigtails (3 per board), 12 AWG; 16 AWG ESC leads.

## Before power-up

1. Resistance from VBATT, +5V and +3V3 to GND: no shorts.
2. Bench supply at 12 V, 100 mA limit: +5V and +3V3 come up.
3. Flash over USB-C (jumper BT to G at power-up for the ROM bootloader).
4. Diagnostics page I2C scan: BNO055 at 0x28, INA238 at 0x45. Its VBUS reads the pack side of
   the shunt, up to 35 mV above VBATT at a 70 A stall.
5. Load step on +5V (buck stability), then a current-limited stall test with a thermocouple on
   R1 and the SW_BACK pad.
