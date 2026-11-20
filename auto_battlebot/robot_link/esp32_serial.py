"""Open an ESP32-S3 USB-Serial/JTAG port without resetting the chip."""

from __future__ import annotations

import serial


def open_esp32_serial(
    path: str, baudrate: int, timeout: float, exclusive: bool = True
) -> serial.Serial:
    port = serial.Serial(timeout=timeout, exclusive=exclusive)
    port.port = path
    port.baudrate = baudrate
    port.dtr = True
    port.rts = True
    port.open()
    return port
