#!/usr/bin/env python3
"""Basic DS3231 RTC hardware test for Jetson.

Usage examples:
    python playground/test_ds3231_rtc.py
    python playground/test_ds3231_rtc.py --i2c-bus 7 --i2c-address 0x68
    python playground/test_ds3231_rtc.py --set-now
    python playground/test_ds3231_rtc.py --compare-system-seconds 10
"""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime, timedelta, timezone

from smbus2 import SMBus


def bcd_to_int(value: int) -> int:
    return ((value >> 4) * 10) + (value & 0x0F)


def int_to_bcd(value: int) -> int:
    return ((value // 10) << 4) | (value % 10)


def read_registers(bus: SMBus, address: int, start: int, length: int) -> list[int]:
    return bus.read_i2c_block_data(address, start, length)


def write_registers(bus: SMBus, address: int, start: int, values: list[int]) -> None:
    bus.write_i2c_block_data(address, start, values)


def decode_datetime_utc(regs: list[int]) -> datetime:
    seconds = bcd_to_int(regs[0] & 0x7F)
    minutes = bcd_to_int(regs[1] & 0x7F)

    hour_raw = regs[2]
    if hour_raw & 0x40:
        # 12-hour mode
        hour = bcd_to_int(hour_raw & 0x1F)
        pm = bool(hour_raw & 0x20)
        hour = (hour % 12) + (12 if pm else 0)
    else:
        # 24-hour mode
        hour = bcd_to_int(hour_raw & 0x3F)

    day_of_month = bcd_to_int(regs[4] & 0x3F)
    month_raw = regs[5]
    month = bcd_to_int(month_raw & 0x1F)
    year = 2000 + bcd_to_int(regs[6])

    return datetime(year, month, day_of_month, hour, minutes, seconds, tzinfo=timezone.utc)


def encode_datetime_utc(now_utc: datetime) -> list[int]:
    weekday = now_utc.isoweekday()  # 1..7 (Mon..Sun), accepted by DS3231 DOW register
    return [
        int_to_bcd(now_utc.second),
        int_to_bcd(now_utc.minute),
        int_to_bcd(now_utc.hour),  # 24-hour mode
        int_to_bcd(weekday),
        int_to_bcd(now_utc.day),
        int_to_bcd(now_utc.month),
        int_to_bcd(now_utc.year % 100),
    ]


def read_temperature_c(bus: SMBus, address: int) -> float:
    msb, lsb = read_registers(bus, address, 0x11, 2)
    if msb & 0x80:
        msb -= 256
    return msb + ((lsb >> 6) * 0.25)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test DS3231 RTC hardware over I2C.")
    parser.add_argument("--i2c-bus", type=int, default=7, help="I2C bus number (default: 7)")
    parser.add_argument(
        "--i2c-address",
        type=lambda x: int(x, 0),
        default=0x68,
        help="I2C device address in decimal or hex (default: 0x68)",
    )
    parser.add_argument(
        "--set-now",
        action="store_true",
        help="Write current UTC system time into DS3231, then verify ticking.",
    )
    parser.add_argument(
        "--compare-system-seconds",
        type=float,
        default=None,
        help="Fail if |RTC - system UTC| exceeds this threshold in seconds.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print(f"Opening /dev/i2c-{args.i2c_bus} address 0x{args.i2c_address:02x}")

    try:
        with SMBus(args.i2c_bus) as bus:
            # Probe basic communication by reading seconds register.
            _ = bus.read_byte_data(args.i2c_address, 0x00)
            print("I2C ACK: OK")

            initial_regs = read_registers(bus, args.i2c_address, 0x00, 7)
            rtc_initial = decode_datetime_utc(initial_regs)
            print(f"RTC time (UTC): {rtc_initial.isoformat()}")

            temp_c = read_temperature_c(bus, args.i2c_address)
            print(f"RTC temperature: {temp_c:.2f} C")

            if args.set_now:
                now_utc = datetime.now(timezone.utc).replace(microsecond=0)
                write_registers(bus, args.i2c_address, 0x00, encode_datetime_utc(now_utc))
                print(f"Wrote RTC time from system UTC: {now_utc.isoformat()}")

                time.sleep(2.0)
                verify_regs = read_registers(bus, args.i2c_address, 0x00, 7)
                rtc_verify = decode_datetime_utc(verify_regs)
                print(f"RTC time after write/verify: {rtc_verify.isoformat()}")

                if rtc_verify < now_utc + timedelta(seconds=1):
                    print(
                        "ERROR: RTC did not appear to tick after write verification.",
                        file=sys.stderr,
                    )
                    return 1

            # Always check ticking behavior in read-only manner.
            before = decode_datetime_utc(read_registers(bus, args.i2c_address, 0x00, 7))
            time.sleep(2.0)
            after = decode_datetime_utc(read_registers(bus, args.i2c_address, 0x00, 7))
            print(f"Tick check: before={before.isoformat()} after={after.isoformat()}")
            if after <= before:
                print("ERROR: RTC is not advancing.", file=sys.stderr)
                return 1

            if args.compare_system_seconds is not None:
                system_utc = datetime.now(timezone.utc).replace(microsecond=0)
                drift_sec = abs((after - system_utc).total_seconds())
                print(f"RTC vs system UTC drift: {drift_sec:.1f}s")
                if drift_sec > args.compare_system_seconds:
                    print(
                        "ERROR: RTC/system drift exceeded threshold.",
                        file=sys.stderr,
                    )
                    return 1

    except FileNotFoundError:
        print(f"ERROR: /dev/i2c-{args.i2c_bus} does not exist.", file=sys.stderr)
        return 1
    except PermissionError:
        print(
            f"ERROR: Permission denied opening /dev/i2c-{args.i2c_bus}. "
            "Use sudo or add your user to the i2c group.",
            file=sys.stderr,
        )
        return 1
    except OSError as exc:
        print(f"ERROR: I2C communication failed: {exc}", file=sys.stderr)
        return 1

    print("DS3231 basic hardware test passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
