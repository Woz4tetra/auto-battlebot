#!/usr/bin/env python3
"""Plot and summarize a velocity-jig LOG file.

Usage: plot_log.py data/downloads/LOG-1.TXT [--show] [--out plot.png]

Reads the scale factors and sample rate from the file header, converts the raw
int16 IMU columns to dps / g, prints summary stats, and saves a 3-panel plot
(gyro, accel, encoder count) next to the input file.
"""
import argparse
import os
import re
import sys

import numpy as np


def parse_header(path):
    gyro, accel, rate = 0.070, 0.000244, None
    with open(path) as f:
        for line in f:
            if not line.startswith("#"):
                break
            for key, cast in (("gyro_dps_per_lsb", float),
                              ("accel_g_per_lsb", float),
                              ("sample_rate_hz", int)):
                m = re.search(key + r"=([\d.]+)", line)
                if m:
                    val = cast(m.group(1))
                    if key.startswith("gyro"):
                        gyro = val
                    elif key.startswith("accel"):
                        accel = val
                    else:
                        rate = val
    return gyro, accel, rate


def main():
    ap = argparse.ArgumentParser(description="plot + summarize a velocity-jig log")
    ap.add_argument("file")
    ap.add_argument("--show", action="store_true", help="show a window, not just save")
    ap.add_argument("--out", help="output PNG (default: <file>.png)")
    args = ap.parse_args()

    if not os.path.exists(args.file):
        print("no such file:", args.file)
        sys.exit(1)

    gyro_scale, accel_scale, hdr_rate = parse_header(args.file)
    data = np.loadtxt(args.file, comments="#", delimiter=",")
    if data.ndim != 2 or data.shape[0] < 2:
        print("not enough data rows")
        sys.exit(1)

    t_us = data[:, 0]
    count = data[:, 1]
    gyro = data[:, 2:5] * gyro_scale     # dps
    accel = data[:, 5:8] * accel_scale   # g
    t = (t_us - t_us[0]) / 1e6           # seconds

    n = len(t)
    dur = t[-1]
    dt = np.diff(t_us) / 1e6             # seconds
    rate = 1.0 / dt.mean() if dt.mean() > 0 else 0.0
    gmag = np.linalg.norm(gyro, axis=1)
    amag = np.linalg.norm(accel, axis=1)

    def row(label, a, fmt):
        print(("  %-4s " + fmt * 4) % (label, a.mean(), a.std(), a.min(), a.max()))

    print("\n=== %s ===" % os.path.basename(args.file))
    print("samples      : %d" % n)
    print("duration     : %.3f s" % dur)
    print("sample rate  : %.1f Hz (header %s)" % (rate, hdr_rate))
    gap = "   <- GAP" if dt.max() > 3 * dt.mean() else ""
    print("dt mean/max  : %.3f / %.3f ms%s" % (dt.mean() * 1e3, dt.max() * 1e3, gap))
    const = "   (constant - encoder likely unplugged)" if np.ptp(count) == 0 else ""
    print("encoder count: start %.0f  end %.0f  delta %.0f  range [%.0f, %.0f]%s"
          % (count[0], count[-1], count[-1] - count[0], count.min(), count.max(), const))

    print("\n  axis      mean      std      min      max   (gyro, dps)")
    for i, ax in enumerate("xyz"):
        row("g" + ax, gyro[:, i], "%9.2f")
    row("|w|", gmag, "%9.2f")
    print("\n  axis      mean      std      min      max   (accel, g)")
    for i, ax in enumerate("xyz"):
        row("a" + ax, accel[:, i], "%9.3f")
    row("|a|", amag, "%9.3f")
    print("  (|a| at rest should be ~1.0 g)")

    import matplotlib
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (a1, a2, a3) = plt.subplots(3, 1, sharex=True, figsize=(11, 8))
    for i, ax in enumerate("xyz"):
        a1.plot(t, gyro[:, i], lw=0.7, label="g" + ax)
    a1.set_ylabel("gyro (dps)")
    a1.legend(loc="upper right", ncol=3)
    a1.grid(alpha=0.3)
    a1.set_title(os.path.basename(args.file))
    for i, ax in enumerate("xyz"):
        a2.plot(t, accel[:, i], lw=0.7, label="a" + ax)
    a2.set_ylabel("accel (g)")
    a2.legend(loc="upper right", ncol=3)
    a2.grid(alpha=0.3)
    a3.plot(t, count, lw=0.8, color="tab:purple")
    a3.set_ylabel("enc count")
    a3.set_xlabel("time (s)")
    a3.grid(alpha=0.3)
    fig.tight_layout()

    out = args.out or os.path.splitext(args.file)[0] + ".png"
    fig.savefig(out, dpi=120)
    print("\nplot -> %s" % out)
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
