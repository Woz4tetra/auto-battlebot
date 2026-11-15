"""Theoretical max RPM calculator for the drivetrain BLDC motors."""

from __future__ import annotations

REPEAT_1806_KV_RPM_PER_V = 2300.0
GEAR_RATIO_MOTOR_PER_OUTPUT = 22.6

CELL_NOMINAL_V = 3.8
CELL_CHARGED_V = 4.35

PACKS_IN_SERIES = 2
CELLS_PER_PACK = 2
SERIES_CELLS = PACKS_IN_SERIES * CELLS_PER_PACK


def motor_rpm(voltage: float, kv: float = REPEAT_1806_KV_RPM_PER_V) -> float:
    """No-load motor-shaft rpm at a given bus voltage."""
    return kv * voltage


def output_rpm(
    voltage: float,
    kv: float = REPEAT_1806_KV_RPM_PER_V,
    gear_ratio: float = GEAR_RATIO_MOTOR_PER_OUTPUT,
) -> float:
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
    print(
        f"Repeat Compact 1806, {REPEAT_1806_KV_RPM_PER_V:.0f} Kv, "
        f"{GEAR_RATIO_MOTOR_PER_OUTPUT}:1 gearbox"
    )
    print(f"Battery: {PACKS_IN_SERIES}x {CELLS_PER_PACK}S LiHV in series = {SERIES_CELLS}S\n")

    _report("Nominal", CELL_NOMINAL_V)
    print()
    _report("Full charge", CELL_CHARGED_V)

    print("\nNote: no-load ceiling (Kv x V). Real top speed runs a few percent")
    print("lower under drivetrain load. Both motors see the same 4S bus, so")
    print("per-motor rpm is identical; the second motor doubles current, not speed.")


if __name__ == "__main__":
    main()
