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
| Thickness | 1.6 mm | Shell legs of the USB-C are sized for it |
| Outer copper | 2 oz | 20 A continuous, 70 A stalls (`../sim/copper_ir.py`) |
| Inner copper | 0.5 oz (default) | ESC current no longer uses the inner layers |
| Surface finish | HASL lead-free | |
| Quantity | 5 boards, 2 assembled | |

## Assembly options

- Standard PCBA, **both sides**: C3, H1, H2, R1, R2, R3 and USB1 are on the bottom.
- Check every part's rotation in JLCPCB's placement preview, bottom side especially.
- 30 BOM lines, 13 extended parts. Parts about $23.60 per board at the last stock check.

## Bought separately

- Molex 146153-0050 Wi-Fi antenna (U.FL, 50 mm cable), one per board.
- XT60 pigtails (3 per board), 12 AWG; 16 AWG ESC leads.

## Before power-up

1. Resistance from VBATT, +5V and +3V3 to GND: no shorts.
2. Bench supply at 12 V, 100 mA limit: +5V and +3V3 come up.
3. Flash over USB-C (jumper BT to G at power-up for the ROM bootloader).
4. Diagnostics page I2C scan: BNO055 at 0x28, INA228 at 0x45.
5. Load step on +5V (buck stability), then a current-limited stall test with a thermocouple on
   R1 and the SW_BACK pad.
