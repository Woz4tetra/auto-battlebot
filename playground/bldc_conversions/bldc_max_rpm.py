"""Theoretical max RPM calculator for the drivetrain BLDC motors.

Hardware:
- 2x Repeat Compact 1806, 2300 Kv, 22.6:1 gearbox
  https://repeat-robotics.com/products/repeat-compact-1806
- 2x BLHeli ESC
- 2x GNB 2S 300mAh LiHV battery wired in series -> 4S bus
  https://www.racedayquads.com/products/gaoneng-gnb-3-8v-2s-300mah-80c-lihv-whoop-micro-battery-w-cabled-connector-xt30

Theoretical no-load speed is just Kv times the applied voltage. Kv is the
back-EMF constant: at N rpm the motor generates N/Kv volts of back-EMF, so the
rotor spins up until back-EMF equals the supply and no more current flows. Real
top speed is a few percent lower (winding resistance, iron/friction losses,
ESC timing), but this is the ceiling the ESC can command.

The 2300 Kv figure is validated by the manufacturer's own output-shaft numbers:
  2300 * 14.8 V (4S nominal) / 22.6 = 1506 rpm  (spec sheet says 1500 rpm)
  2300 * 11.1 V (3S nominal) / 22.6 = 1130 rpm  (spec sheet says 1130 rpm)

Usage:
    python playground/bldc_max_rpm.py
"""

from __future__ import annotations

# --- Hardware constants ---------------------------------------------------

MOTOR_KV = 2300.0  # rpm per volt, Repeat Compact 1806
GEAR_RATIO = 22.6  # motor turns per output-shaft turn

# GNB LiHV cell. LiHV charges to 4.35 V/cell (vs 4.20 for a plain LiPo) and
# sags to ~3.8 V/cell nominal.
CELL_NOMINAL_V = 3.8
CELL_CHARGED_V = 4.35

# Two 2S packs in series.
PACKS_IN_SERIES = 2
CELLS_PER_PACK = 2
SERIES_CELLS = PACKS_IN_SERIES * CELLS_PER_PACK  # 4S


def motor_rpm(voltage: float, kv: float = MOTOR_KV) -> float:
    """No-load motor-shaft rpm at a given bus voltage."""
    return kv * voltage


def output_rpm(voltage: float, kv: float = MOTOR_KV, gear_ratio: float = GEAR_RATIO) -> float:
    """No-load output-shaft rpm (after the gearbox)."""
    return motor_rpm(voltage, kv) / gear_ratio


def _report(label: str, cell_v: float) -> None:
    bus_v = SERIES_CELLS * cell_v
    motor = motor_rpm(bus_v)
    output = output_rpm(bus_v)
    print(f"{label:<14} {cell_v:.2f} V/cell -> {bus_v:5.1f} V bus")
    print(f"    motor shaft : {motor:8.0f} rpm  ({motor / 60:6.1f} rev/s)")
    print(f"    output shaft: {output:8.0f} rpm  ({output / 60:6.1f} rev/s)")


def main() -> None:
    print(f"Repeat Compact 1806, {MOTOR_KV:.0f} Kv, {GEAR_RATIO}:1 gearbox")
    print(f"Battery: {PACKS_IN_SERIES}x {CELLS_PER_PACK}S LiHV in series = {SERIES_CELLS}S\n")

    _report("Nominal", CELL_NOMINAL_V)
    print()
    _report("Full charge", CELL_CHARGED_V)

    print("\nNote: no-load ceiling (Kv x V). Real top speed runs a few percent")
    print("lower under drivetrain load. Both motors see the same 4S bus, so")
    print("per-motor rpm is identical; the second motor doubles current, not speed.")


if __name__ == "__main__":
    main()
