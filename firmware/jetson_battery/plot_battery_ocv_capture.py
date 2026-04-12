#!/usr/bin/env python3
"""Plot battery capture CSV and save figure as PNG.

Example:
  python scripts/plot_battery_ocv_capture.py \
      --input-csv data/battery_discharge_capture.csv \
      --output-png data/battery_discharge_capture.png
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot battery discharge capture CSV")
    parser.add_argument(
        "--input-csv",
        type=Path,
        default=Path("data/battery_discharge_capture.csv"),
    )
    parser.add_argument(
        "--output-png",
        type=Path,
        default=Path("data/battery_discharge_capture.png"),
    )
    return parser.parse_args()


def load_capture(path: Path) -> dict[str, list[float]]:
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        raise ValueError(f"No rows found in capture file: {path}")

    t0 = float(rows[0]["timestamp"])
    elapsed_min = [(float(row["timestamp"]) - t0) / 60.0 for row in rows]
    voltage_v = [float(row["bus_voltage_v"]) for row in rows]
    current_a = [float(row["current_a"]) for row in rows]
    power_w = [float(row["power_w"]) for row in rows]
    soc_est_percent = [float(row["soc_est_percent"]) for row in rows]
    return {
        "elapsed_min": elapsed_min,
        "voltage_v": voltage_v,
        "current_a": current_a,
        "power_w": power_w,
        "soc_est_percent": soc_est_percent,
    }


def main() -> int:
    args = parse_args()
    data = load_capture(args.input_csv)

    args.output_png.parent.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle("Battery Discharge Capture")

    axes[0].plot(data["elapsed_min"], data["voltage_v"])
    axes[0].set_ylabel("Voltage (V)")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(data["elapsed_min"], data["current_a"])
    axes[1].set_ylabel("Current (A)")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(data["elapsed_min"], data["power_w"])
    axes[2].set_ylabel("Power (W)")
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(data["elapsed_min"], data["soc_est_percent"])
    axes[3].set_ylabel("SoC (%)")
    axes[3].set_xlabel("Elapsed Time (min)")
    axes[3].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(args.output_png, dpi=150)
    print(f"Wrote plot: {args.output_png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
