"""BLHeli_32 settings page encryption."""

from __future__ import annotations

import os
import struct
from collections.abc import Callable

PAGE_ADDRESS = 0xF800
PAGE_SIZE = 256
BLOCK = 8
PAYLOAD_PER_BLOCK = 6
PAYLOAD_SIZE = PAGE_SIZE // BLOCK * PAYLOAD_PER_BLOCK
TWEAK_64K = 0xF800

_KEY = (0x81FBC617, 0x9E81C901, 0x29A1FA54, 0x318234B4)
_DELTA = 0x9E3779B9
_MASK = 0xFFFFFFFF


class PageError(ValueError):
    pass


def _mix(v: int) -> int:
    return ((((v << 4) & _MASK) ^ (v >> 5)) + v) & _MASK


def decrypt_block(block: bytes, tweak: int = TWEAK_64K) -> bytes:
    v0, v1 = struct.unpack("<II", block)
    s = (32 * _DELTA) & _MASK
    for _ in range(32):
        v1 = (v1 - (_mix(v0) ^ ((s + _KEY[(s >> 11) & 3] + tweak) & _MASK))) & _MASK
        s = (s - _DELTA) & _MASK
        v0 = (v0 - (_mix(v1) ^ ((s + _KEY[s & 3] + tweak) & _MASK))) & _MASK
    return struct.pack("<II", v0, v1)


def encrypt_block(block: bytes, tweak: int = TWEAK_64K) -> bytes:
    v0, v1 = struct.unpack("<II", block)
    s = 0
    for _ in range(32):
        v0 = (v0 + (_mix(v1) ^ ((s + _KEY[s & 3] + tweak) & _MASK))) & _MASK
        s = (s + _DELTA) & _MASK
        v1 = (v1 + (_mix(v0) ^ ((s + _KEY[(s >> 11) & 3] + tweak) & _MASK))) & _MASK
    return struct.pack("<II", v0, v1)


def decode_page(page: bytes, tweak: int = TWEAK_64K) -> bytes:
    """Encrypted 256-byte page -> 192-byte settings payload (nonces dropped)."""
    if len(page) != PAGE_SIZE:
        raise PageError(f"page is {len(page)} bytes, expected {PAGE_SIZE}")
    return b"".join(
        decrypt_block(page[i : i + BLOCK], tweak + i)[2:] for i in range(0, PAGE_SIZE, BLOCK)
    )


def page_nonces(page: bytes, tweak: int = TWEAK_64K) -> list[bytes]:
    return [decrypt_block(page[i : i + BLOCK], tweak + i)[:2] for i in range(0, PAGE_SIZE, BLOCK)]


def esc_style_nonces(seed: bytes | None = None) -> Callable[[int], bytes]:
    """Nonces shaped like the ESC's: byte 0 rises by 8 per block, byte 1 falls by 8."""
    b0, b1 = seed if seed is not None else os.urandom(2)
    return lambda n: bytes([(b0 + 8 * n) & 0xFF, (b1 - 8 * n) & 0xFF])


def encode_page(
    payload: bytes, tweak: int = TWEAK_64K, nonce: Callable[[int], bytes] | None = None
) -> bytes:
    """Encrypt a 192-byte settings payload into a 256-byte page."""
    nonce = nonce or (lambda _: os.urandom(2))
    if len(payload) != PAYLOAD_SIZE:
        raise PageError(f"payload is {len(payload)} bytes, expected {PAYLOAD_SIZE}")
    page = bytearray()
    for n in range(PAGE_SIZE // BLOCK):
        chunk = payload[n * PAYLOAD_PER_BLOCK : (n + 1) * PAYLOAD_PER_BLOCK]
        page += encrypt_block(nonce(n) + chunk, tweak + n * BLOCK)
    return bytes(page)
