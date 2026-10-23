"""Read and write BLHeli_32 settings through the 4-way passthrough.

The read sequence is BLHeliSuite32xl's, from a usbmon capture: per ESC, DeviceReset(n),
DeviceInitFlash(n), read 256 bytes at 0xF800 (settings), 16 at 0xFA00 (input frame counters),
16 at 0xFD90, then DeviceReset(n) to release it.

Write Setup, from a capture against a passthrough that acknowledged writes without forwarding
them: DeviceReset(n), DeviceInitFlash(n), read the three regions, one DeviceWrite of the
256-byte encrypted page at 0xF800 (no DevicePageErase; the bootloader erases), read the three
regions back to verify, DeviceReset(n). Every payload byte BLHeliSuite does not edit, reserved
ones included, is written back exactly as read.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

from auto_battlebot.blheli32 import cipher, layout
from auto_battlebot.blheli32.fourway import FourWay, SerialPort

STATS_ADDRESS = 0xFA00
INFO_ADDRESS = 0xFD90


@dataclass(frozen=True)
class EscSnapshot:
    index: int
    signature: bytes
    page: bytes  # encrypted, exactly as read
    payload: bytes  # decrypted 192-byte record
    stats: bytes
    info: bytes

    # 0xFA00 stats: byte 3 is the detected input protocol (5 on DShot; other codes unmapped),
    # then good and bad input frame counters as little-endian u32.
    @property
    def input_protocol(self) -> int:
        return self.stats[3]

    @property
    def good_frames(self) -> int:
        return int(struct.unpack("<I", self.stats[4:8])[0])

    @property
    def bad_frames(self) -> int:
        return int(struct.unpack("<I", self.stats[8:12])[0])


class VerifyError(RuntimeError):
    """The ESC acknowledged the write but reads back different settings."""


class Session:
    def __init__(self, port: SerialPort, msp: bool = True) -> None:
        self.link = FourWay(port, msp=msp)

    def open(self) -> str:
        self.link.enter()
        name = self.link.interface_name()
        self.link.set_mode_arm_blb()
        return name

    def read(self, esc: int) -> EscSnapshot:
        signature = self.link.init_esc(esc)
        try:
            page = self.link.read(cipher.PAGE_ADDRESS, cipher.PAGE_SIZE)
            stats = self.link.read(STATS_ADDRESS, 16)
            info = self.link.read(INFO_ADDRESS, 16)
        finally:
            self.link.release_esc(esc)
        return EscSnapshot(esc, signature, page, cipher.decode_page(page), stats, info)

    def write(self, esc: int, payload: bytes, expect_current: bytes | None = None) -> EscSnapshot:
        """Write a 192-byte payload to one ESC and verify it by reading it back.

        expect_current, when given, must equal what the ESC holds now: the edit was made from
        that read, so anything else means the backup is stale or names the wrong ESC.
        """
        layout.check_layout(payload)
        page = cipher.encode_page(payload)
        self.link.init_esc(esc)
        try:
            current = cipher.decode_page(self.link.read(cipher.PAGE_ADDRESS, cipher.PAGE_SIZE))
            layout.check_layout(current)
            if expect_current is not None and current != expect_current:
                raise VerifyError(f"ESC {esc} no longer matches the backup this edit came from")
            if current != payload:
                self.link.write(cipher.PAGE_ADDRESS, page)
            after = cipher.decode_page(self.link.read(cipher.PAGE_ADDRESS, cipher.PAGE_SIZE))
        finally:
            self.link.release_esc(esc)
        if after != payload:
            changed = [i for i in range(len(payload)) if after[i] != payload[i]]
            raise VerifyError(f"ESC {esc}: read-back differs at payload bytes {changed}")
        return self.read(esc)

    def close(self) -> None:
        self.link.exit()
