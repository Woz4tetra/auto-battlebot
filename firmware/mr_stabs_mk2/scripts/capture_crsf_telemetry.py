#!/usr/bin/env python3
"""Capture the raw CRSF telemetry frames an EdgeTX radio receives, and decode the sensors.

Needs the custom EdgeTX build in ~/edgetx, whose `telemetry on` CLI command mirrors every byte
from the external module to USB serial. Plug the radio in over USB and pick "USB Serial (VCP)".

Prints each battery and attitude frame as raw bytes and decoded values, then a count of every
frame type seen, and writes the same to a log file.
"""

import argparse
import time
from collections import Counter
from pathlib import Path

from serial.tools.list_ports import comports

import serial

EDGETX_VID = 0x0483
EDGETX_PID = 0x5740

FRAME_NAMES = {
    0x02: "gps",
    0x07: "vario",
    0x08: "battery",
    0x09: "baro_altitude",
    0x0B: "heartbeat",
    0x14: "link_statistics",
    0x1E: "attitude",
    0x21: "flight_mode",
    0x29: "device_info",
}


def crc8_dvb_s2(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def find_radio() -> str:
    for port in comports():
        if port.vid == EDGETX_VID and port.pid == EDGETX_PID:
            return port.device
    raise SystemExit("No EdgeTX radio found. Plug it in and pick USB Serial (VCP).")


def decode(frame_type: int, payload: bytes) -> str:
    if frame_type == 0x08 and len(payload) >= 8:
        volts = int.from_bytes(payload[0:2], "big") / 10
        amps = int.from_bytes(payload[2:4], "big") / 10
        mah = int.from_bytes(payload[4:7], "big")
        percent = payload[7]
        return f"RxBt={volts:.1f} V  Curr={amps:.1f} A  Capa={mah} mAh  Bat%={percent}"
    if frame_type == 0x1E and len(payload) >= 6:
        pitch, roll, yaw = (
            int.from_bytes(payload[i : i + 2], "big", signed=True) / 10000 for i in (0, 2, 4)
        )
        return (
            f"Ptch={pitch:+.4f} rad ({pitch * 57.2958:+.1f} deg)  "
            f"Roll={roll:+.4f} rad ({roll * 57.2958:+.1f} deg)  "
            f"Yaw={yaw:+.4f} rad ({yaw * 57.2958:+.1f} deg)"
        )
    return ""


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--log", type=Path, default=Path.home() / "crsf_telemetry_capture.log")
    args = parser.parse_args()

    device = serial.Serial(find_radio(), 115200, timeout=0.1)
    device.write(b"telemetry on\r\n")
    buffer = bytearray()
    counts: Counter[int] = Counter()
    lines: list[str] = []
    end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < end:
            buffer.extend(device.read(256))
            # A CRSF frame is [address][length][type][payload][crc]; length covers type..crc.
            while len(buffer) >= 4:
                length = buffer[1]
                if not 2 <= length <= 62:
                    del buffer[0]
                    continue
                if len(buffer) < length + 2:
                    break
                frame = bytes(buffer[: length + 2])
                if crc8_dvb_s2(frame[2:-1]) != frame[-1]:
                    del buffer[0]
                    continue
                del buffer[: length + 2]
                frame_type = frame[2]
                counts[frame_type] += 1
                if frame_type in (0x08, 0x1E):
                    text = decode(frame_type, frame[3:-1])
                    name = FRAME_NAMES[frame_type]
                    line = f"addr=0x{frame[0]:02X} {name:8s} {frame.hex(' ')}  {text}"
                    print(line)
                    lines.append(line)
    finally:
        device.write(b"telemetry off\r\n")
        device.close()

    summary = ["", "frame counts:"] + [
        f"  0x{t:02X} {FRAME_NAMES.get(t, '?'):16s} {n}" for t, n in sorted(counts.items())
    ]
    print("\n".join(summary))
    args.log.write_text("\n".join(lines + summary) + "\n")
    print(f"wrote {args.log}")


if __name__ == "__main__":
    main()
