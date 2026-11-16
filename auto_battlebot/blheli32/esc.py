"""Read and write BLHeli_32 settings through the 4-way passthrough."""

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
    encrypted_page: bytes
    decrypted_payload: bytes
    stats: bytes
    info: bytes

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
    def __init__(self, port: SerialPort, switch_from_msp: bool = True) -> None:
        self.link = FourWay(port, switch_from_msp=switch_from_msp)

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
        """Write a payload to one ESC and verify by reading back."""
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
