#!/usr/bin/env python3
"""Flash mr_stabs_mk2 over ESP-NOW through the USB dongle.

Run it on the machine the dongle is plugged into, normally the ZED Box (scripts/ota does that over
ssh). The robot must be disarmed. The transfer is go-back-N over the 0x10-0x14 messages in
docs/plans/espnow_link_plan.md, then the script waits for the rebooted robot to report the new
image valid and checks its MD5.

    venv/bin/python firmware/mr_stabs_mk2/scripts/espnow_ota.py firmware.bin --manage-service
"""

from __future__ import annotations

import argparse
import hashlib
import struct
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import serial

SERIAL_MAGIC = b"\x7b\xb0"
MSG_OTA_BEGIN = 0x10
MSG_OTA_CHUNK = 0x11
MSG_OTA_END = 0x12
MSG_OTA_ABORT = 0x13
MSG_OTA_STATUS = 0x14

OTA_IDLE, OTA_RECEIVING, OTA_DONE, OTA_VALIDATED, OTA_ERROR = range(5)
STATE_NAMES = {0: "idle", 1: "receiving", 2: "done", 3: "validated", 4: "error"}
ERROR_NAMES = {
    0: "ok",
    1: "robot is armed; disarm it and retry",
    2: "Update.begin refused the size (image larger than the OTA slot?)",
    3: "flash write failed",
    4: "image failed verification (MD5 or ESP-IDF image check)",
    5: "aborted on the robot (armed, host abort, or 5 s of silence)",
    6: "robot has no transfer in progress",
}

CHUNK = 240  # espnow_protocol::OTA_CHUNK_DATA
WINDOW = 16  # chunks in flight past the last acknowledged offset
PROGRESS_TIMEOUT_S = 0.5  # above a ~150 ms block erase, which stalls acknowledgements
MAX_TIMEOUTS = 10
VALIDATE_TIMEOUT_S = 30.0  # reboot plus the robot's 10 s validation hold
SERVICE = "auto-battlebot"


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(msg_type: int, payload: bytes = b"") -> bytes:
    body = bytes([msg_type, len(payload)]) + payload
    return SERIAL_MAGIC + body + struct.pack("<H", crc16_ccitt(body))


@dataclass
class OtaStatus:
    state: int
    next_offset: int
    error: int
    running_md5: bytes

    @classmethod
    def decode(cls, payload: bytes) -> OtaStatus:
        state, next_offset, error = struct.unpack_from("<BIB", payload)
        return cls(state, next_offset, error, payload[6:22])

    def describe(self) -> str:
        state = STATE_NAMES.get(self.state, str(self.state))
        return f"{state}: {ERROR_NAMES.get(self.error, str(self.error))}"


class FrameReader:
    """Streaming parser for the dongle's serial frames. Skips text and CRC failures."""

    def __init__(self, port: serial.Serial) -> None:
        self.port = port
        self.buffer = bytearray()

    def statuses(self) -> list[OtaStatus]:
        self.buffer += self.port.read(self.port.in_waiting or 1)
        found = []
        while True:
            start = self.buffer.find(SERIAL_MAGIC)
            if start < 0:
                del self.buffer[:-1]
                return found
            del self.buffer[:start]
            if len(self.buffer) < 4:
                return found
            length = self.buffer[3]
            size = 6 + length
            if len(self.buffer) < size:
                return found
            body = bytes(self.buffer[2 : 4 + length])
            (crc,) = struct.unpack_from("<H", self.buffer, 4 + length)
            if crc != crc16_ccitt(body):
                del self.buffer[:1]
                continue
            if body[0] == MSG_OTA_STATUS and length == 22:
                found.append(OtaStatus.decode(body[2:]))
            del self.buffer[:size]


class OtaError(RuntimeError):
    pass


def wait_for_status(reader: FrameReader, timeout_s: float, states: set[int]) -> OtaStatus | None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for status in reader.statuses():
            if status.state == OTA_ERROR:
                raise OtaError(status.describe())
            if status.state in states:
                return status
    return None


def begin_transfer(port: serial.Serial, reader: FrameReader, image: bytes, md5: bytes) -> None:
    begin = encode_frame(MSG_OTA_BEGIN, struct.pack("<I", len(image)) + md5)
    for attempt in range(5):
        port.write(begin)
        if wait_for_status(reader, 1.0, {OTA_RECEIVING}) is not None:
            return
        print(f"no answer to begin (attempt {attempt + 1}/5)")
    raise OtaError("robot never answered: is it powered, in range, and on the dongle's channel?")


class Transfer:
    """Go-back-N sender: at most WINDOW chunks past the robot's acknowledged offset."""

    def __init__(self, port: serial.Serial, reader: FrameReader, image: bytes) -> None:
        self.port = port
        self.reader = reader
        self.image = image
        self.acked = 0
        self.send_offset = 0
        self.last_progress = time.monotonic()
        self.timeouts = 0
        self.last_goback: tuple[int, float] = (-1, 0.0)

    def fill_window(self) -> None:
        end = min(len(self.image), self.acked + WINDOW * CHUNK)
        while self.send_offset < end:
            data = self.image[self.send_offset : self.send_offset + CHUNK]
            frame = encode_frame(MSG_OTA_CHUNK, struct.pack("<I", self.send_offset) + data)
            self.port.write(frame)
            self.send_offset += len(data)

    def apply(self, status: OtaStatus) -> None:
        if status.state == OTA_ERROR:
            raise OtaError(status.describe())
        if status.next_offset > self.acked:
            self.acked = status.next_offset
            self.last_progress = time.monotonic()
            self.timeouts = 0
        if status.next_offset < self.send_offset:
            # The robot dropped a chunk. Every chunk behind it in flight draws the same answer, so
            # only go back once per offset in a short window.
            now = time.monotonic()
            offset, when = self.last_goback
            if offset != status.next_offset or now - when > 0.1:
                self.send_offset = status.next_offset
                self.last_goback = (status.next_offset, now)

    def check_timeout(self) -> None:
        if time.monotonic() - self.last_progress <= PROGRESS_TIMEOUT_S:
            return
        self.timeouts += 1
        if self.timeouts >= MAX_TIMEOUTS:
            self.port.write(encode_frame(MSG_OTA_ABORT))
            raise OtaError(f"no progress after {MAX_TIMEOUTS} timeouts at offset {self.acked}")
        self.send_offset = self.acked
        self.last_progress = time.monotonic()

    def run(self) -> None:
        last_print = 0.0
        while self.acked < len(self.image):
            self.fill_window()
            for status in self.reader.statuses():
                self.apply(status)
            self.check_timeout()
            if time.monotonic() - last_print > 1.0:
                last_print = time.monotonic()
                size = len(self.image)
                print(f"  {100 * self.acked / size:5.1f}%  {self.acked}/{size}")


def finish_transfer(port: serial.Serial, reader: FrameReader, md5: bytes) -> None:
    for _ in range(5):
        port.write(encode_frame(MSG_OTA_END))
        if wait_for_status(reader, 1.0, {OTA_DONE}) is not None:
            break
    else:
        raise OtaError("robot never confirmed the image after the last chunk")

    print("robot rebooting into the new image; waiting for it to validate")
    status = wait_for_status(reader, VALIDATE_TIMEOUT_S, {OTA_VALIDATED})
    if status is None:
        raise OtaError(
            f"no validation within {VALIDATE_TIMEOUT_S:.0f} s: the new image is installed but did "
            "not report running for 10 s with the dongle in range. There is no rollback; if the "
            "robot is not driving, recover over USB with scripts/upload."
        )
    if status.running_md5 != md5:
        raise OtaError(f"robot validated an image with md5 {status.running_md5.hex()}, not ours")
    print(f"validated, running md5 {status.running_md5.hex()}")


def flash(port: serial.Serial, image: bytes) -> None:
    md5 = hashlib.md5(image).digest()
    reader = FrameReader(port)
    port.reset_input_buffer()
    begin_transfer(port, reader, image, md5)
    print(f"sending {len(image)} bytes, md5 {md5.hex()}")
    started = time.monotonic()
    Transfer(port, reader, image).run()
    print(f"transferred in {time.monotonic() - started:.1f} s, verifying")
    finish_transfer(port, reader, md5)


def systemctl(action: str) -> None:
    subprocess.run(["sudo", "systemctl", action, SERVICE], check=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("image", type=Path, help="firmware.bin from pio run")
    parser.add_argument("--port", default="/dev/espnow_dongle", help="dongle serial device")
    parser.add_argument(
        "--manage-service",
        action="store_true",
        help=f"stop {SERVICE} to free the dongle, and start it again afterward, even on failure",
    )
    args = parser.parse_args()

    image = args.image.read_bytes()
    stopped = False
    try:
        if args.manage_service:
            print(f"stopping {SERVICE}")
            systemctl("stop")
            stopped = True
        # DTR and RTS stay high, as Linux raises them on open. Setting them low one at a time
        # passes through DTR low with RTS high, which resets an ESP32-S3 USB-Serial/JTAG port,
        # and two resets within ~500 ms drop an Adafruit board into its TinyUF2 bootloader.
        port = serial.Serial(timeout=0.01, exclusive=True)
        port.port = args.port
        port.baudrate = 921600
        port.dtr = True
        port.rts = True
        with port:  # opens it
            flash(port, image)
    except OtaError as error:
        print(f"OTA failed: {error}", file=sys.stderr)
        return 1
    except serial.SerialException as error:
        print(f"cannot use {args.port}: {error}", file=sys.stderr)
        if not args.manage_service:
            print(f"is {SERVICE} holding it? pass --manage-service", file=sys.stderr)
        return 1
    finally:
        if stopped:
            print(f"starting {SERVICE}")
            systemctl("start")
    return 0


if __name__ == "__main__":
    sys.exit(main())
