#!/usr/bin/env python3
"""USB-serial TUI for the velocity jig: list / download / delete / live-stream.

Talks to the firmware's idle-only console, so nothing here can run while the jig
is recording (commands sent during a recording return BUSY). Run via the
`console` wrapper, or: venv/bin/python console.py [-p PORT] [-o OUTDIR]
"""

import argparse
import glob
import os
import select
import sys
import time

from serial.tools import list_ports

import serial

BAUD = 921600  # nominal only; USB CDC ignores baud (transfers run at USB speed)
GYRO_DPS_PER_LSB = 0.070
ACCEL_G_PER_LSB = 0.000244


def find_port(preferred=None):
    if preferred:
        return preferred
    acms = sorted(glob.glob("/dev/ttyACM*"))
    if acms:
        return acms[0]
    for p in list_ports.comports():
        if "ACM" in p.device or "usbmodem" in p.device:
            return p.device
    return None


def read_line(ser, timeout=3.0):
    """Read one newline-terminated line, decoded and stripped. '' on timeout."""
    deadline = time.time() + timeout
    buf = bytearray()
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b == b"\n":
            return buf.decode("ascii", "replace").rstrip("\r")
        buf += b
    return buf.decode("ascii", "replace").rstrip("\r")


def send(ser, cmd):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    ser.flush()


def do_list(ser):
    send(ser, "LIST")
    files = []
    while True:
        line = read_line(ser, 3.0)
        if line in ("END", ""):
            break
        if line == "BUSY":
            print("  device is recording; stop it first")
            return None
        if line.startswith("F "):
            name, _, size = line[2:].rpartition(" ")
            if name and size.isdigit():
                files.append((name, int(size)))
    return files


def do_get(ser, name, outdir):
    send(ser, "GET " + name)
    line = read_line(ser, 3.0)
    if line == "BUSY":
        print("  device is recording")
        return
    if line.startswith("ERR"):
        print("  " + line)
        return
    if not line.startswith("SIZE "):
        print("  unexpected reply: %r" % line)
        return
    size = int(line.split()[1])
    data = bytearray()
    t0 = time.time()
    while len(data) < size and time.time() - t0 < 60:
        chunk = ser.read(size - len(data))
        if chunk:
            data += chunk
            t0 = time.time()
    read_line(ser, 1.0)  # trailing blank
    read_line(ser, 1.0)  # END
    os.makedirs(outdir, exist_ok=True)
    path = os.path.join(outdir, name)
    with open(path, "wb") as f:
        f.write(data)
    print("  saved %d/%d bytes -> %s" % (len(data), size, path))


def do_del(ser, name):
    send(ser, "DEL " + name)
    print("  " + (read_line(ser, 3.0) or "no response"))


def do_stream(ser):
    send(ser, "STREAM")
    ack = read_line(ser, 2.0)
    if ack == "BUSY":
        print("  device is recording")
        return
    print("  streaming ~10 Hz. Press Enter to stop.")
    print(
        "  %-15s %-7s %8s %8s %8s %7s %7s %7s"
        % ("t_us", "count", "gx_dps", "gy_dps", "gz_dps", "ax_g", "ay_g", "az_g")
    )
    try:
        while True:
            r, _, _ = select.select([ser.fileno(), sys.stdin], [], [], 0.5)
            if sys.stdin in r:
                sys.stdin.readline()
                ser.write(b"\n")  # tell the firmware to stop
                break
            if ser.fileno() in r:
                line = read_line(ser, 0.5)
                if not line or line == "END":
                    continue
                if line.startswith("S "):
                    try:
                        v = line[2:].split(",")
                        t, c = int(v[0]), int(v[1])
                        g = [int(x) * GYRO_DPS_PER_LSB for x in v[2:5]]
                        a = [int(x) * ACCEL_G_PER_LSB for x in v[5:8]]
                        print(
                            "  %-15d %-7d %8.2f %8.2f %8.2f %7.3f %7.3f %7.3f"
                            % (t, c, g[0], g[1], g[2], a[0], a[1], a[2])
                        )
                    except (ValueError, IndexError):
                        pass
    except KeyboardInterrupt:
        ser.write(b"\n")
    t0 = time.time()  # drain until END
    while time.time() - t0 < 1.0:
        if read_line(ser, 0.3) == "END":
            break


def parse_index(cmd, files):
    if len(cmd) < 2 or not cmd[1].lstrip("-").isdigit():
        print("  need a file number")
        return None
    idx = int(cmd[1])
    if not (0 <= idx < len(files)):
        print("  no file %d (run 'l' to list)" % idx)
        return None
    return idx


def main():
    ap = argparse.ArgumentParser(description="velocity jig USB console")
    ap.add_argument("-p", "--port", help="serial port (default: first /dev/ttyACM*)")
    ap.add_argument("-o", "--outdir", default="data/downloads", help="download directory")
    args = ap.parse_args()

    port = find_port(args.port)
    if not port:
        print("No serial port found. Is the jig plugged in and not in BOOTSEL?")
        sys.exit(1)
    try:
        ser = serial.Serial(port, BAUD, timeout=0.2)
    except serial.SerialException as e:
        print("Could not open %s: %s" % (port, e))
        sys.exit(1)
    time.sleep(0.3)
    ser.reset_input_buffer()
    print("velocity jig console on %s  (downloads -> %s/)" % (port, args.outdir))

    files = []
    while True:
        print("\n[l]ist  [d]ownload N  [a]ll  [x]delete N  [s]tream  [q]uit")
        try:
            raw = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not raw:
            continue
        cmd = raw.split()
        op = cmd[0].lower()
        if op == "q":
            break
        elif op == "l":
            files = do_list(ser) or []
            for i, (n, sz) in enumerate(files):
                print("  %d: %-16s %10d B" % (i, n, sz))
            if not files:
                print("  (no files)")
        elif op == "d":
            files = files or do_list(ser) or []
            idx = parse_index(cmd, files)
            if idx is not None:
                do_get(ser, files[idx][0], args.outdir)
        elif op == "a":
            files = do_list(ser) or []
            for n, _ in files:
                do_get(ser, n, args.outdir)
        elif op == "x":
            files = files or do_list(ser) or []
            idx = parse_index(cmd, files)
            if idx is not None:
                do_del(ser, files[idx][0])
                files = do_list(ser) or []
        elif op == "s":
            do_stream(ser)
        else:
            print("  ?")
    ser.close()


if __name__ == "__main__":
    main()
