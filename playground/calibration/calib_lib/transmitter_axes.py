"""Read-only reader for the OpenTX transmitter's streamed stick axes (channels).

apriltag_track.py records what the driver's sticks were doing alongside each camera frame, so the
drivetrain fit can correlate the commanded input against the AprilTag ground-truth motion. This
module only READS the radio's channel feed (the same `telemetry on` + `channels on` stream that
channel_streaming.py decodes); it never sends trainer commands, so it is safe to run while a human
drives the robot manually.

The OpenTX radio streams a custom channel packet: sync 0xA3 0xA4 0xA5, a phase byte (0 = channels
1..16, 1 = channels 17..32), a length byte (= 33), 16 little-endian int16 channel values, then a
checksum equal to phase ^ length ^ (the 32 data bytes). Values are roughly [-1024, 1024]; the stick
axes are the low channels. A daemon thread drains the serial port and keeps the latest full
32-channel snapshot, which the capture loop samples at each image timestamp.
"""

from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass

from serial.tools.list_ports import comports

import serial

# Same OpenTX USB-CDC device as drive_protocol.py / channel_streaming.py (VID=0x0483, PID=0x5740).
OPENTX_VID = 0x0483
OPENTX_PID = 0x5740

_SYNC = bytes([0xA3, 0xA4, 0xA5])
_CHANNELS_PER_PACKET = 16
_NUM_CHANNELS = 32
# Bytes after the phase+length pair: 16 int16 channel values + 1 checksum byte.
_PACKET_LEN = _CHANNELS_PER_PACKET * 2 + 1


@dataclass
class ChannelSample:
    """A decoded channel snapshot. `t` is the CLOCK_MONOTONIC time it was decoded (staleness)."""

    t: float
    channels: list[int]


def find_transmitter_port() -> str | None:
    """Auto-detect the OpenTX radio by USB VID/PID (shared by the read-only reader and --drive)."""
    for p in comports():
        if p.vid == OPENTX_VID and p.pid == OPENTX_PID:
            return p.device
    return None


class _ChannelDecoder:
    """Incremental, silent decoder for the OpenTX channel stream (corrupt/partial packets dropped).

    feed() returns the (phase, 16-channel) sets fully present in the bytes seen so far; partial
    packets are buffered until the rest arrives. Unlike channel_streaming.ChannelStreamingParser it
    prints nothing, so it does not spam the capture console on a noisy link.
    """

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[tuple[int, list[int]]]:
        self._buf.extend(data)
        out: list[tuple[int, list[int]]] = []
        while True:
            i = self._buf.find(_SYNC)
            if i < 0:
                # No sync in buffer; keep the last 2 bytes (a 3-byte sync may straddle the read).
                if len(self._buf) > 2:
                    del self._buf[: len(self._buf) - 2]
                break
            if i > 0:
                del self._buf[:i]  # discard junk before the sync
            # Need sync(3) + phase(1) + length(1) + payload(_PACKET_LEN) before a packet can parse.
            if len(self._buf) < len(_SYNC) + 2 + _PACKET_LEN:
                break
            phase = self._buf[len(_SYNC)]
            length = self._buf[len(_SYNC) + 1]
            if length != _PACKET_LEN:
                del self._buf[:1]  # bad framing; drop one byte and resync on the next sync
                continue
            start = len(_SYNC) + 2
            payload = bytes(self._buf[start : start + _PACKET_LEN])
            del self._buf[: start + _PACKET_LEN]
            chk = phase ^ length
            for b in payload[:-1]:
                chk ^= b
            if chk != payload[-1] or phase not in (0, 1):
                continue  # corrupt or unknown phase; drop it
            channels = [
                struct.unpack_from("<h", payload, j)[0]
                for j in range(0, _CHANNELS_PER_PACKET * 2, 2)
            ]
            out.append((phase, channels))
        return out


class TransmitterReader:
    """Streams the OpenTX channel feed on a daemon thread, holding the latest decoded snapshot.

    Read-only: primes the radio with `telemetry on` + `channels on` and decodes the channel packets,
    but never issues trainer commands. latest() returns the most recent ChannelSample (or None until
    the first full packet), which the capture loop stamps with the image timestamp.
    """

    def __init__(self, port: str) -> None:
        self._serial = serial.Serial(port, baudrate=115200, timeout=0.1)
        self._decoder = _ChannelDecoder()
        self._channels = [0] * _NUM_CHANNELS  # last value seen per channel, updated half at a time
        self._latest: ChannelSample | None = None
        self._packets = 0
        self._stop = threading.Event()
        # Re-prime the OpenTX streams, like channel_streaming.py / OpenTxTransmitter::initialize().
        self._serial.write(b"telemetry on\r\n")
        self._serial.write(b"channels on\r\n")
        self._thread = threading.Thread(target=self._run, name="transmitter-reader", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                n = self._serial.in_waiting or 1  # read(1) blocks up to the 0.1s timeout, no spin
                data = self._serial.read(n)
            except Exception:
                break
            if not data:
                continue
            for phase, channels in self._decoder.feed(data):
                base = phase * _CHANNELS_PER_PACKET
                self._channels[base : base + _CHANNELS_PER_PACKET] = channels
                # Snapshot a copy so the consumer never sees a half-updated list (ref assign atomic).
                self._latest = ChannelSample(time.monotonic(), list(self._channels))
                self._packets += 1

    def latest(self) -> ChannelSample | None:
        return self._latest

    @property
    def packets(self) -> int:
        return self._packets

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=0.5)
        try:
            self._serial.close()
        except Exception:
            pass
