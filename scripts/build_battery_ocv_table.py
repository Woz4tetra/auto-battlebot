#!/usr/bin/env python3
"""Capture INA219 discharge data and generate a static OCV table.

Example:
  python scripts/build_battery_ocv_table.py --i2c-bus 7 --i2c-address 0x41 \\
      --cutoff-voltage 9.0 --sample-interval 2.0
"""

from __future__ import annotations

import argparse
import csv
import os
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import tomli_w
from smbus2 import SMBus


INA219_REG_CONFIG = 0x00
INA219_REG_BUSVOLTAGE = 0x02
INA219_REG_POWER = 0x03
INA219_REG_CURRENT = 0x04
INA219_REG_CALIBRATION = 0x05

INA219_CALIBRATION_16V_5A = 26868
INA219_CONFIG_16V_5A_CONTINUOUS = 0x0EEF
INA219_CURRENT_LSB_A = 0.04096 / (INA219_CALIBRATION_16V_5A * 0.01)
INA219_POWER_LSB_W = 20.0 * INA219_CURRENT_LSB_A


@dataclass
class Sample:
    t: float
    bus_voltage_v: float
    current_a: float
    power_w: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build static OCV table from full discharge run")
    parser.add_argument("--i2c-bus", type=int, default=7)
    parser.add_argument("--i2c-address", type=lambda x: int(x, 0), default=0x41)
    parser.add_argument("--sample-interval", type=float, default=2.0)
    parser.add_argument(
        "--checkpoint-interval-sec",
        type=float,
        default=30.0,
        help="Periodically save CSV/table while capture is running",
    )
    parser.add_argument("--cutoff-voltage", type=float, default=9.0)
    parser.add_argument("--point-count", type=int, default=7)
    parser.add_argument(
        "--output-csv",
        type=Path,
        default=Path("data/battery_discharge_capture.csv"),
    )
    parser.add_argument(
        "--output-table",
        type=Path,
        default=Path("data/battery_ocv_table.toml"),
    )
    parser.add_argument(
        "--discharge-current-negative",
        action="store_true",
        help="Interpret negative INA219 current as discharge current",
    )
    return parser.parse_args()


def write_reg_u16(bus: SMBus, addr: int, reg: int, value: int) -> None:
    bus.write_i2c_block_data(addr, reg, [(value >> 8) & 0xFF, value & 0xFF])


def read_reg_u16(bus: SMBus, addr: int, reg: int) -> int:
    data = bus.read_i2c_block_data(addr, reg, 2)
    return (data[0] << 8) | data[1]


def signed_u16(value: int) -> int:
    return value - 65536 if value > 32767 else value


def read_sample(bus: SMBus, addr: int) -> Sample:
    raw_bus = read_reg_u16(bus, addr, INA219_REG_BUSVOLTAGE)
    raw_current = read_reg_u16(bus, addr, INA219_REG_CURRENT)
    raw_power = read_reg_u16(bus, addr, INA219_REG_POWER)
    current_a = signed_u16(raw_current) * INA219_CURRENT_LSB_A
    power_w = signed_u16(raw_power) * INA219_POWER_LSB_W
    bus_v = (raw_bus >> 3) * 0.004
    return Sample(t=time.time(), bus_voltage_v=bus_v, current_a=current_a, power_w=power_w)


def compute_soc_trace(samples: list[Sample], discharge_current_positive: bool) -> tuple[np.ndarray, np.ndarray]:
    if len(samples) < 2:
        raise ValueError("Need at least 2 samples to compute discharge curve")

    discharged_ah = [0.0]
    total_discharge_ah = 0.0
    for i in range(1, len(samples)):
        dt_h = max(0.0, samples[i].t - samples[i - 1].t) / 3600.0
        current = samples[i].current_a if discharge_current_positive else -samples[i].current_a
        discharge_current = max(current, 0.0)
        total_discharge_ah += discharge_current * dt_h
        discharged_ah.append(total_discharge_ah)

    total_discharge_ah = max(total_discharge_ah, 1e-6)
    soc = [max(0.0, min(100.0, 100.0 * (1.0 - (x / total_discharge_ah)))) for x in discharged_ah]
    voltage = [s.bus_voltage_v for s in samples]
    return np.asarray(soc, dtype=float), np.asarray(voltage, dtype=float)


def build_table_from_trace(soc: np.ndarray, voltage: np.ndarray, point_count: int) -> tuple[list[float], list[float]]:
    order = np.argsort(soc)
    soc_sorted = soc[order]
    voltage_sorted = voltage[order]

    unique_soc, unique_idx = np.unique(soc_sorted, return_index=True)
    unique_voltage = voltage_sorted[unique_idx]

    targets = np.linspace(0.0, 100.0, num=max(2, point_count))
    interp_voltage = np.interp(targets, unique_soc, unique_voltage)
    return [round(float(v), 4) for v in interp_voltage], [round(float(s), 2) for s in targets]


def write_capture_csv(path: Path, samples: list[Sample], soc_trace: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "bus_voltage_v", "current_a", "power_w", "soc_est_percent"])
        for sample, soc in zip(samples, soc_trace):
            writer.writerow([sample.t, sample.bus_voltage_v, sample.current_a, sample.power_w, soc])
        f.flush()
        os.fsync(f.fileno())


def write_table_toml(path: Path, voltage_v: list[float], soc_percent: list[float]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "discharge": {"voltage_v": voltage_v, "soc_percent": soc_percent},
        "charge": {"voltage_v": voltage_v, "soc_percent": soc_percent},
    }
    with path.open("wb") as f:
        f.write(tomli_w.dumps(data).encode("utf-8"))
        f.flush()
        os.fsync(f.fileno())


def checkpoint_outputs(args: argparse.Namespace, samples: list[Sample]) -> None:
    if len(samples) < 2:
        return
    discharge_current_positive = not args.discharge_current_negative
    soc_trace, voltage_trace = compute_soc_trace(samples, discharge_current_positive)
    table_voltage, table_soc = build_table_from_trace(soc_trace, voltage_trace, args.point_count)
    write_capture_csv(args.output_csv, samples, soc_trace)
    write_table_toml(args.output_table, table_voltage, table_soc)


def main() -> int:
    args = parse_args()
    keep_running = True

    def stop_handler(_sig: int, _frame: object) -> None:
        nonlocal keep_running
        keep_running = False

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    samples: list[Sample] = []
    last_checkpoint_t = 0.0
    with SMBus(args.i2c_bus) as bus:
        write_reg_u16(bus, args.i2c_address, INA219_REG_CALIBRATION, INA219_CALIBRATION_16V_5A)
        write_reg_u16(bus, args.i2c_address, INA219_REG_CONFIG, INA219_CONFIG_16V_5A_CONTINUOUS)
        print(f"Capturing discharge data on i2c-{args.i2c_bus} addr {hex(args.i2c_address)}")
        while keep_running:
            sample = read_sample(bus, args.i2c_address)
            samples.append(sample)
            print(
                f"V={sample.bus_voltage_v:.3f}V I={sample.current_a:.3f}A P={sample.power_w:.3f}W "
                f"(n={len(samples)})"
            )
            now = time.time()
            if now - last_checkpoint_t >= max(1.0, args.checkpoint_interval_sec):
                checkpoint_outputs(args, samples)
                last_checkpoint_t = now
                print(f"Checkpoint saved: {args.output_csv} and {args.output_table}")
            if sample.bus_voltage_v <= args.cutoff_voltage:
                print(f"Reached cutoff voltage {args.cutoff_voltage:.3f}V; stopping capture.")
                break
            time.sleep(max(0.1, args.sample_interval))

    if len(samples) < 2:
        print("Not enough samples captured to build table.", file=sys.stderr)
        return 1

    discharge_current_positive = not args.discharge_current_negative
    soc_trace, voltage_trace = compute_soc_trace(samples, discharge_current_positive)
    table_voltage, table_soc = build_table_from_trace(soc_trace, voltage_trace, args.point_count)
    write_capture_csv(args.output_csv, samples, soc_trace)
    write_table_toml(args.output_table, table_voltage, table_soc)

    print(f"Wrote capture CSV: {args.output_csv}")
    print(f"Wrote OCV table:  {args.output_table}")
    print("Discharge table:")
    for v, s in zip(table_voltage, table_soc):
        print(f"  {v:>7.4f} V -> {s:>6.2f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
