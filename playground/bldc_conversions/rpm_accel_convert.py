"""Convert between output-shaft acceleration (RPM/s) and wheel-surface linear"""

from __future__ import annotations

import argparse
import math

# --- Defaults (match config/_common.toml [transmitter]) -------------------

DEFAULT_WHEEL_DIAMETER_M = 0.05
SLIP_FLIP_LIMIT_MPS2 = 8.5


def rpm_per_sec_to_mps2(rpm_per_sec: float, wheel_diameter: float) -> float:
    """Output-shaft angular acceleration (RPM/s) -> wheel-surface linear accel (m/s^2)."""
    return rpm_per_sec * math.pi * wheel_diameter / 60.0


def mps2_to_rpm_per_sec(mps2: float, wheel_diameter: float) -> float:
    """Wheel-surface linear accel (m/s^2) -> output-shaft angular acceleration (RPM/s)."""
    return mps2 * 60.0 / (math.pi * wheel_diameter)


def _reference_table(wheel_diameter: float) -> None:
    print(f"wheel diameter: {wheel_diameter:.3f} m (radius {wheel_diameter / 2:.3f} m)\n")
    print(f"{'m/s^2':>10}   {'RPM/s':>10}")
    print(f"{'-' * 10}   {'-' * 10}")
    for mps2 in (2.0, 4.0, 6.0, SLIP_FLIP_LIMIT_MPS2, 10.0, 12.0):
        marker = "  <- slip/flip limit" if mps2 == SLIP_FLIP_LIMIT_MPS2 else ""
        print(f"{mps2:>10.2f}   {mps2_to_rpm_per_sec(mps2, wheel_diameter):>10.1f}{marker}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--wheel-diameter",
        type=float,
        default=DEFAULT_WHEEL_DIAMETER_M,
        metavar="M",
        help=f"output-shaft wheel diameter in meters (default {DEFAULT_WHEEL_DIAMETER_M})",
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--mps2", type=float, metavar="A", help="convert this m/s^2 value to RPM/s")
    group.add_argument(
        "--rpm-per-sec", type=float, metavar="R", help="convert this RPM/s value to m/s^2"
    )
    args = parser.parse_args()

    diameter: float = args.wheel_diameter

    if args.mps2 is not None:
        rpm_per_sec = mps2_to_rpm_per_sec(args.mps2, diameter)
        print(f"{args.mps2:.3f} m/s^2  ->  {rpm_per_sec:.1f} RPM/s  (wheel {diameter:.3f} m)")
    elif args.rpm_per_sec is not None:
        mps2 = rpm_per_sec_to_mps2(args.rpm_per_sec, diameter)
        print(f"{args.rpm_per_sec:.1f} RPM/s  ->  {mps2:.3f} m/s^2  (wheel {diameter:.3f} m)")
    else:
        _reference_table(diameter)


if __name__ == "__main__":
    main()
