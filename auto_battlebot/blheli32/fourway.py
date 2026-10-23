"""BLHeli 4-way interface client over a serial passthrough (firmware/BlHeli-Passthrough).

Framing, from the passthrough's Check_4Way and a usbmon capture of BLHeliSuite32xl 1.0.4.4:

    request:  0x2F cmd addr_hi addr_lo n params[n or 256] crc_hi crc_lo
    response: 0x2E cmd addr_hi addr_lo n params[n or 256] ack crc_hi crc_lo

n = 0 means 256 bytes. CRC is CRC16-XMODEM over everything before it. The serial link first
switches from MSP to 4-way with MSP_SET_4WAY_IF (0xF5), as BLHeliSuite does. The Mk2 robot's
tuning-mode relay (http_port.HttpPort) speaks 4-way only, so it skips that step.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import Protocol

START_REQUEST = 0x2F
START_RESPONSE = 0x2E

CMD_TEST_ALIVE = 0x30
CMD_PROTOCOL_GET_VERSION = 0x31
CMD_INTERFACE_GET_NAME = 0x32
CMD_INTERFACE_GET_VERSION = 0x33
CMD_INTERFACE_EXIT = 0x34
CMD_DEVICE_RESET = 0x35
CMD_DEVICE_INIT_FLASH = 0x37
CMD_DEVICE_PAGE_ERASE = 0x39
CMD_DEVICE_READ = 0x3A
CMD_DEVICE_WRITE = 0x3B
CMD_INTERFACE_SET_MODE = 0x3F

MODE_ARM_BLB = 0x04
ACK_OK = 0x00

MSP_SET_4WAY_IF = 0xF5


class SerialPort(Protocol):
    """The slice of pyserial's Serial that this module uses."""

    def write(self, data: bytes) -> int | None: ...
    def read(self, size: int = 1) -> bytes: ...
    def reset_input_buffer(self) -> None: ...


class FourWayError(RuntimeError):
    pass


@dataclass(frozen=True)
class Response:
    cmd: int
    address: int
    params: bytes
    ack: int


def crc16_xmodem(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_request(cmd: int, address: int = 0, params: bytes = b"\x00") -> bytes:
    if not 1 <= len(params) <= 256:
        raise ValueError(f"4-way params must be 1..256 bytes, got {len(params)}")
    body = bytes([START_REQUEST, cmd, address >> 8, address & 0xFF, len(params) & 0xFF]) + params
    return body + struct.pack(">H", crc16_xmodem(body))


def parse_response(frame: bytes) -> Response:
    if len(frame) < 8 or frame[0] != START_RESPONSE:
        raise FourWayError(f"not a 4-way response: {frame[:8].hex()}")
    n = frame[4] or 256
    if len(frame) != 5 + n + 3:
        raise FourWayError(f"response length {len(frame)} does not match n={n}")
    body, crc = frame[:-2], struct.unpack(">H", frame[-2:])[0]
    if crc16_xmodem(body) != crc:
        raise FourWayError("response CRC mismatch")
    return Response(frame[1], (frame[2] << 8) | frame[3], frame[5 : 5 + n], frame[5 + n])


def msp_request(cmd: int, payload: bytes = b"") -> bytes:
    checksum = len(payload) ^ cmd
    for b in payload:
        checksum ^= b
    return b"$M<" + bytes([len(payload), cmd]) + payload + bytes([checksum])


class FourWay:
    """One 4-way session. ESC indices are the passthrough's pin order (0 = A2, 1 = A3).

    msp=False skips the MSP switch, for transports that speak 4-way from the start.
    """

    def __init__(self, port: SerialPort, timeout_s: float = 2.0, msp: bool = True) -> None:
        self._port = port
        self._timeout_s = timeout_s
        self._msp = msp

    def _read_exact(self, n: int) -> bytes:
        out = bytearray()
        deadline = time.monotonic() + self._timeout_s
        while len(out) < n:
            chunk = self._port.read(n - len(out))
            if chunk:
                out += chunk
            elif time.monotonic() > deadline:
                raise FourWayError(f"timeout: got {len(out)} of {n} bytes")
        return bytes(out)

    def enter(self) -> None:
        """Switch the passthrough from MSP to 4-way mode."""
        self._port.reset_input_buffer()
        if not self._msp:
            return
        self._port.write(msp_request(MSP_SET_4WAY_IF))
        header = self._read_exact(5)
        if header[:3] != b"$M>":
            raise FourWayError(f"MSP_SET_4WAY_IF got {header.hex()}")
        self._read_exact(header[3] + 1)

    def command(self, cmd: int, address: int = 0, params: bytes = b"\x00") -> Response:
        self._port.write(build_request(cmd, address, params))
        header = self._read_exact(5)
        n = header[4] or 256
        resp = parse_response(header + self._read_exact(n + 3))
        if resp.cmd != cmd:
            raise FourWayError(f"sent 0x{cmd:02X}, got a reply to 0x{resp.cmd:02X}")
        if resp.ack != ACK_OK:
            raise FourWayError(
                f"command 0x{cmd:02X} at 0x{address:04X} failed, ack 0x{resp.ack:02X}"
            )
        return resp

    def interface_name(self) -> str:
        return self.command(CMD_INTERFACE_GET_NAME).params.decode("ascii", "replace")

    def set_mode_arm_blb(self) -> None:
        self.command(CMD_INTERFACE_SET_MODE, params=bytes([MODE_ARM_BLB]))

    def init_esc(self, esc: int) -> bytes:
        """Reset the ESC into its bootloader and connect. Returns the 4 signature bytes."""
        self.command(CMD_DEVICE_RESET, params=bytes([esc]))
        return self.command(CMD_DEVICE_INIT_FLASH, params=bytes([esc])).params

    def release_esc(self, esc: int) -> None:
        self.command(CMD_DEVICE_RESET, params=bytes([esc]))

    def read(self, address: int, length: int) -> bytes:
        data = self.command(CMD_DEVICE_READ, address, bytes([length & 0xFF])).params
        if len(data) != length:
            raise FourWayError(f"read 0x{address:04X}: asked {length} bytes, got {len(data)}")
        return data

    def write(self, address: int, data: bytes) -> None:
        self.command(CMD_DEVICE_WRITE, address, data)

    def exit(self) -> None:
        self.command(CMD_INTERFACE_EXIT)
