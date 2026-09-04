"""USB console client for the velocity jig (`firmware/velocity_jig`).

The jig is a Feather RP2040 Adalogger that rides on the robot and logs a wheel encoder plus
an ISM330DHCX IMU to microSD at 1 kHz. It speaks a line protocol over native USB CDC:

    LIST            -> "F <name> <size>" per file, then "END"
    GET <name>      -> "SIZE <n>", exactly n raw bytes, then "\\nEND\\n"
    DEL <name>      -> "OK" or "ERR"
    TIME            -> "TIME <rx_us> <tx_us>"
    STREAM          -> "STREAM", then "S t_us,count,gx,gy,gz,ax,ay,az" at 10 Hz until any
                       byte arrives, then "END"

Three firmware behaviors shape this client:

1. **Blind during recording.** Every command except `TIME` answers `BUSY`. `TIME` is the
   exception on purpose: it touches neither the SD card nor the capture path, so a mid-run
   probe measures skew across the run window instead of extrapolating into it.
2. **Recording starts and stops on physical buttons.** There is no `START`. Button A emits
   an unsolicited `recording <name>` and button B a `stopped, n=... dropped=...`, so the
   client latches those lines as they arrive rather than only while something is waiting.
   Operators press A during the clock probe burst, and an unlatched press hangs the CLI.
3. **`STREAM` stops on any byte** and can exit into a recording: pressing A mid-stream
   prints `END` and then `recording <name>`, so `END` is not necessarily terminal.

Baud is nominal. This is native USB CDC, so transfers run at USB Full Speed regardless.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterator

import numpy as np
from serial.tools.list_ports import comports

import serial
from auto_battlebot.velocity_jig import ClockProbe

# The jig's USB ids under the arduino-pico core. CONFIRM ON HARDWARE with --list-ports:
# the core takes these from the board variant, and a wrong guess here silently falls back
# to the scan below. The important half is TRANSMITTER_USB_IDS: without excluding it, a
# first-ttyACM scan will happily open the radio and then time out waiting for "TIME".
JIG_USB_IDS = frozenset(
    {
        (0x239A, 0x80F1),  # Adafruit vendor id, Feather RP2040 variant
        (0x2E8A, 0x000A),  # Raspberry Pi vendor id, generic RP2040 CDC
    }
)
TRANSMITTER_USB_IDS = frozenset({(0x0483, 0x5740)})  # OpenTX/EdgeTX radio

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
    """Locate the jig, never the transmitter.

    Exact USB id match first. Falling back to "any ACM port that is not the radio" keeps the
    tool usable before JIG_USB_IDS has been confirmed, but it warns through the caller so
    the constant gets fixed rather than quietly relied on.
    """
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
    """Re-find a port after a replug.

    Matching on the device path does not survive a replug: /dev/ttyACM0 can come back as
    ttyACM1 if anything else enumerated in between. The RP2040's serial number is its flash
    id, so it is stable across the unplug the run card asks for.
    """
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

    # -- plumbing ----------------------------------------------------------

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
        except OSError as err:  # the cable was pulled
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

    # -- console commands --------------------------------------------------

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
        """Download one log. Consumes the trailing newline and END, which the web tool did
        not, leaving a stray blank line in front of the next reply."""
        self._send(f"GET {name}")
        header = self._read_line(10.0)
        if header == "BUSY":
            raise JigBusyError("GET refused: the jig is recording")
        if header is None or not header.startswith("SIZE "):
            raise OSError(f"GET {name}: expected SIZE, got {header!r}")
        size = int(header.split()[1])

        # Whatever is already buffered is payload, not lines.
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
            # END is not necessarily terminal: pressing A mid-stream prints END and then
            # starts a recording, so keep pumping until the latches settle.
            if self._read_line(0.3) == "END":
                return

    # -- button handshakes -------------------------------------------------

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
        """Block until button B is pressed. Returns (samples, dropped).

        Must be called with the cable plugged in: the stop summary goes to the wire
        unbuffered, so a press while unplugged loses the counts for good.
        """
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
    """Fit the host-to-jig clock offset from a burst of TIME round trips.

    The jig counts microseconds since its own boot, so cross-correlating command edges
    against measured acceleration only ever recovers `clock_offset + transport_delay`.
    Measuring the offset separately is what makes the fitted transport delay a real number
    rather than a real number plus however far apart the two clocks happen to be.

    Only the fastest decile is kept. USB Full Speed polls in 1 ms frames, so most of the
    spread is host scheduling; the fastest round trips are the ones whose transport was
    most nearly symmetric, which is the assumption the midpoint pairing rests on.
    """
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
    """Crystal drift across the run, in parts per million.

    The RP2040 crystal runs around 30 ppm, which is about 0.9 ms over a 30 s run: small,
    but the same order as the residual the run card gates on, so worth removing.

    Returns 0.0 when the span is not positive. The jig counts from its own boot, so a
    post-probe at or below the pre-probe is a board that restarted, and dividing by that
    span is arithmetic across two epochs. LOG-119 did exactly this and produced -1048410
    ppm, which reads as a 105% clock error and would shift a 218 s run by 228 s if anything
    applied it. `ClockFit.from_probes` already guarded against it; this did not, so the
    number reached the sidecar where a reader picking `skew_ppm` up directly would trust it.
    """
    span_ms = post.at_jig_ms - pre.at_jig_ms
    if span_ms <= 1e-6:
        return 0.0
    return (post.offset_ms - pre.offset_ms) / span_ms * 1e6
