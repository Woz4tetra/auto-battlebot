"""USB console client for the velocity jig (`firmware/velocity_jig`)."""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterator

import numpy as np
from serial.tools.list_ports import comports

import serial
from auto_battlebot.calibration.jig.velocity_jig import ClockProbe

ADAFRUIT_FEATHER_RP2040_USB_ID = (0x239A, 0x80F1)
RPI_RP2040_CDC_USB_ID = (0x2E8A, 0x000A)
JIG_USB_IDS = frozenset({ADAFRUIT_FEATHER_RP2040_USB_ID, RPI_RP2040_CDC_USB_ID})
TRANSMITTER_USB_IDS = frozenset({(0x0483, 0x5740)})

JIG_BAUD = 921600
GYRO_DPS_PER_LSB = 0.070
ACCEL_G_PER_LSB = 0.000244


class JigBusyError(RuntimeError):
    """The jig is recording, so it answered BUSY. Only TIME works in that state."""


@dataclass(frozen=True)
class PortInfo:
    device: str
    vid: int | None
    pid: int | None
    serial_number: str | None
    product: str | None
    manufacturer: str | None

    @property
    def usb_id(self) -> tuple[int, int] | None:
        return None if self.vid is None or self.pid is None else (self.vid, self.pid)

    def describe(self) -> str:
        ids = "----:----" if self.usb_id is None else f"{self.vid:04x}:{self.pid:04x}"
        return (
            f"{self.device:<20} {ids}  sn={self.serial_number or '-':<20} "
            f"{self.manufacturer or '-'} / {self.product or '-'}"
        )


def list_ports() -> list[PortInfo]:
    return [
        PortInfo(p.device, p.vid, p.pid, p.serial_number, p.product, p.manufacturer)
        for p in comports()
    ]


def find_jig_port(preferred: str | None = None) -> PortInfo | None:
    """Locate the jig, never the transmitter."""
    ports = list_ports()
    if preferred:
        for p in ports:
            if p.device == preferred:
                return p
        return PortInfo(preferred, None, None, None, None, None)

    for p in ports:
        if p.usb_id in JIG_USB_IDS:
            return p
    for p in ports:
        if p.usb_id in TRANSMITTER_USB_IDS:
            continue
        if "ACM" in p.device or "usbmodem" in p.device:
            return p
    return None


def find_port_by_serial(serial_number: str) -> PortInfo | None:
    """Re-find a port after a replug."""
    for p in list_ports():
        if p.serial_number and p.serial_number == serial_number:
            return p
    return None


@dataclass(frozen=True)
class ClockSample:
    """One TIME round trip, in host seconds and jig seconds."""

    t_send: float
    t_recv: float
    rx_us: int
    tx_us: int

    @property
    def rtt(self) -> float:
        return self.t_recv - self.t_send

    @property
    def host_mid_ms(self) -> float:
        return 0.5 * (self.t_send + self.t_recv) * 1e3

    @property
    def jig_mid_ms(self) -> float:
        return 0.5 * (self.rx_us + self.tx_us) * 1e-3


@dataclass(frozen=True)
class StreamRow:
    t_us: int
    count: int
    gyro_dps: tuple[float, float, float]
    accel_g: tuple[float, float, float]


class JigLink:
    """Serial console client. One command at a time, unsolicited lines latched."""

    def __init__(self, port: str, *, serial_number: str | None = None, timeout: float = 0.1):
        self.port = port
        self.serial_number = serial_number
        self._serial = serial.Serial(port, JIG_BAUD, timeout=timeout)
        self._buf = bytearray()
        self._pending: list[str] = []
        self._latched_recording: str | None = None
        self._latched_stopped: tuple[int, int] | None = None
        self.recording = False

    def close(self) -> None:
        try:
            self._serial.close()
        except Exception:
            pass

    def __enter__(self) -> JigLink:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def _pump(self) -> None:
        """Drain the port into whole lines, routing unsolicited ones to their latches."""
        try:
            waiting = self._serial.in_waiting
        except OSError as err:
            raise ConnectionError(f"{self.port}: {err}") from err
        data = self._serial.read(waiting or 1)
        if not data:
            return
        self._buf.extend(data)
        while b"\n" in self._buf:
            raw, _, rest = self._buf.partition(b"\n")
            self._buf = bytearray(rest)
            line = raw.decode("ascii", "replace").rstrip("\r")
            if not line:
                continue
            if line.startswith("recording "):
                self._latched_recording = line.split(None, 1)[1].strip()
                self.recording = True
                continue
            if line.startswith("stopped,"):
                self._latched_stopped = _parse_stopped(line)
                self.recording = False
                continue
            self._pending.append(line)

    def _read_line(self, timeout: float = 4.0) -> str | None:
        deadline = time.monotonic() + timeout
        while True:
            if self._pending:
                return self._pending.pop(0)
            if time.monotonic() >= deadline:
                return None
            self._pump()

    def _send(self, cmd: str) -> None:
        self._serial.write((cmd + "\n").encode("ascii"))
        self._serial.flush()

    def _command(self, cmd: str, timeout: float = 4.0) -> str | None:
        self._send(cmd)
        line = self._read_line(timeout)
        if line == "BUSY":
            raise JigBusyError(f"{cmd!r} refused: the jig is recording")
        return line

    def list_files(self, timeout: float = 4.0) -> list[tuple[str, int]]:
        self._send("LIST")
        out: list[tuple[str, int]] = []
        while True:
            line = self._read_line(timeout)
            if line is None or line == "END":
                return out
            if line == "BUSY":
                raise JigBusyError("LIST refused: the jig is recording")
            if line.startswith("F "):
                parts = line.split()
                if len(parts) >= 3:
                    out.append((parts[1], int(parts[2])))

    def get(self, name: str, timeout: float = 600.0) -> bytes:
        """Download one log, consuming the trailing newline and END."""
        self._send(f"GET {name}")
        header = self._read_line(10.0)
        if header == "BUSY":
            raise JigBusyError("GET refused: the jig is recording")
        if header is None or not header.startswith("SIZE "):
            raise OSError(f"GET {name}: expected SIZE, got {header!r}")
        size = int(header.split()[1])

        data = bytearray(self._buf)
        self._buf = bytearray()
        deadline = time.monotonic() + timeout
        while len(data) < size and time.monotonic() < deadline:
            chunk = self._serial.read(min(65536, size - len(data)))
            if chunk:
                data.extend(chunk)
                deadline = time.monotonic() + timeout
        if len(data) != size:
            raise OSError(f"GET {name}: read {len(data)} of {size} bytes")

        end = self._read_line(5.0)
        if end != "END":
            raise OSError(f"GET {name}: expected END, got {end!r}")
        return bytes(data)

    def download(self, name: str, dest: Path | str) -> int:
        dest = Path(dest)
        payload = self.get(name)
        dest.write_bytes(payload)
        return len(payload)

    def delete(self, name: str) -> bool:
        return self._command(f"DEL {name}") == "OK"

    def time_probe(self, timeout: float = 1.0) -> ClockSample | None:
        t0 = time.monotonic()
        self._send("TIME")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self._read_line(max(0.0, deadline - time.monotonic()))
            if line is None:
                return None
            if line.startswith("TIME "):
                t1 = time.monotonic()
                parts = line.split()
                if len(parts) >= 3:
                    return ClockSample(t0, t1, int(parts[1]), int(parts[2]))
                return None
        return None

    def stream(self, seconds: float) -> Iterator[StreamRow]:
        """Live sensor rows at ~10 Hz. Nothing is recorded; safe while idle only."""
        self._send("STREAM")
        deadline = time.monotonic() + seconds
        try:
            while time.monotonic() < deadline:
                line = self._read_line(max(0.05, deadline - time.monotonic()))
                if line is None:
                    continue
                if line == "BUSY":
                    raise JigBusyError("STREAM refused: the jig is recording")
                row = _parse_stream_row(line)
                if row is not None:
                    yield row
        finally:
            self._stop_stream()

    def _stop_stream(self) -> None:
        self._send("")
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            if self._read_line(0.3) == "END":
                return

    def take_recording(self) -> str | None:
        self._pump()
        name, self._latched_recording = self._latched_recording, None
        return name

    def take_stopped(self) -> tuple[int, int] | None:
        self._pump()
        got, self._latched_stopped = self._latched_stopped, None
        return got

    def wait_recording(self, timeout: float = 300.0, tick: Callable[[], None] | None = None) -> str:
        """Block until button A is pressed. Honors a press that already happened."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            name = self.take_recording()
            if name:
                return name
            if tick:
                tick()
            time.sleep(0.05)
        raise TimeoutError("timed out waiting for button A (no 'recording' line)")

    def wait_stopped(
        self, timeout: float = 300.0, tick: Callable[[], None] | None = None
    ) -> tuple[int, int]:
        """Block until button B is pressed."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            got = self.take_stopped()
            if got:
                return got
            if tick:
                tick()
            time.sleep(0.05)
        raise TimeoutError("timed out waiting for button B (no 'stopped' line)")


def _parse_stopped(line: str) -> tuple[int, int]:
    samples = dropped = 0
    for token in line.replace(",", " ").split():
        if token.startswith("n="):
            samples = int(token[2:])
        elif token.startswith("dropped="):
            dropped = int(token[8:])
    return samples, dropped


def _parse_stream_row(line: str) -> StreamRow | None:
    if not line.startswith("S "):
        return None
    try:
        v = [int(x) for x in line[2:].split(",")]
    except ValueError:
        return None
    if len(v) != 8:
        return None
    return StreamRow(
        t_us=v[0],
        count=v[1],
        gyro_dps=(v[2] * GYRO_DPS_PER_LSB, v[3] * GYRO_DPS_PER_LSB, v[4] * GYRO_DPS_PER_LSB),
        accel_g=(v[5] * ACCEL_G_PER_LSB, v[6] * ACCEL_G_PER_LSB, v[7] * ACCEL_G_PER_LSB),
    )


def probe_clock(link: JigLink, n: int = 200, *, min_kept: int = 8) -> ClockProbe:
    """Fit the host-to-jig clock offset from a burst of TIME round trips."""
    samples = [s for s in (link.time_probe() for _ in range(n)) if s is not None]
    if len(samples) < min_kept:
        raise OSError(f"clock probe: only {len(samples)} of {n} probes answered")

    samples.sort(key=lambda s: s.rtt)
    keep = samples[: max(4, -(-len(samples) // 10))]
    offsets = np.array([s.host_mid_ms - s.jig_mid_ms for s in keep])
    median = float(np.median(offsets))
    residual = float(np.sqrt(np.mean((offsets - median) ** 2)))
    fastest = keep[0]
    return ClockProbe(
        offset_ms=median,
        residual_ms=residual,
        at_host_ms=fastest.host_mid_ms,
        at_jig_ms=fastest.jig_mid_ms,
        kept=len(keep),
        total=n,
    )


def skew_ppm(pre: ClockProbe, post: ClockProbe) -> float:
    """Crystal drift across the run in ppm, 0.0 when the board restarted between probes."""
    span_ms = post.at_jig_ms - pre.at_jig_ms
    if span_ms <= 1e-6:
        return 0.0
    return (post.offset_ms - pre.offset_ms) / span_ms * 1e6
