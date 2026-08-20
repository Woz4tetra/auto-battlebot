"""Trainer link to the OpenTX/EdgeTX radio: writes commands, reads back what was sent.

Mirrors the trainer protocol in `src/transmitter/opentx_transmitter.cpp`. The radio is a USB
CDC device (VID 0x0483, PID 0x5740) primed once with `telemetry on` + `channels on`, then
driven with `trainer <channel> <value>` at 50 Hz, value in [-500, 500].

**One port, both directions.** The same serial port that accepts `trainer` writes also
streams the radio's mixer output back once primed. Reading that back matters because
trainer mode *adds* to the human driver's sticks: the mixer output is the command the robot
actually received, not the one this process asked for. Two consequences:

- A hand-driven run has a real command log. Without the readback it has a column of zeros
  and is unusable for fitting.
- A scripted run gets a contamination check. If the driver's stick is not centered, measured
  diverges from commanded and the run can be flagged instead of quietly poisoning the fit.

Commands are sent RAW. The deployed transmitter applies `DifferentialDriveProcessor`'s
lifted and zero deadzones; this does not, because the physical deadzone is exactly what is
being measured and pre-compensating it here would measure the compensation.

SAFETY
- The robot moves fast. Run in a clear, bounded space with guard plates on.
- Keep the human driver's sticks centered: in trainer mode the radio ADDS stick input.
- Zero the channels and disarm on every exit path: normal return, Ctrl-C, exception, and a
  hard wall-clock timeout. `armed()` is a context manager that does this.
"""

from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Iterator

from serial.tools.list_ports import comports

# Matches kChannelMax / kTrainerMax in opentx_transmitter.cpp.
TRAINER_MAX = 500
OPENTX_VID = 0x0483
OPENTX_PID = 0x5740
TRANSMITTER_USB_IDS = frozenset({(OPENTX_VID, OPENTX_PID)})

# The two trainer channels this writes. What they MEAN depends on the mix: under "direct"
# they are linear and angular, under "tank" they are left and right wheel.
CHANNEL_A = 0
CHANNEL_B = 1

# Radio channel-stream framing: sync, phase byte (0 = channels 1..16, 1 = 17..32), length
# byte, 16 little-endian int16, then a checksum equal to phase ^ length ^ every data byte.
_SYNC = bytes([0xA3, 0xA4, 0xA5])
_CHANNELS_PER_PACKET = 16
_NUM_CHANNELS = 32
_PACKET_LEN = _CHANNELS_PER_PACKET * 2 + 1


def find_transmitter_port() -> str | None:
    for p in comports():
        if p.vid == OPENTX_VID and p.pid == OPENTX_PID:
            return p.device
    return None


def to_trainer(value: float) -> int:
    """Normalized [-1, 1] to the radio's integer range, clamped."""
    return max(-TRAINER_MAX, min(TRAINER_MAX, round(value * TRAINER_MAX)))


@dataclass(frozen=True)
class MixConfig:
    """How body command maps onto the two trainer channels, and how to read it back.

    `mode` is "tank" when the robot runs TankDriveProcessor and the radio must carry left
    and right wheel, or "direct" when the robot's own processor mixes and the channels carry
    linear and angular. Getting this backwards makes the robot arc on a straight command and
    spin on a turn, which is why the polarity check exists as a session abort gate.
    """

    mode: str = "tank"
    reverse_angular: bool = True
    channel_scale: float = 1024.0  # full scale of the readback stream
    read_linear: int = 0
    read_angular: int = 1
    read_arm: int = 4
    # Per-channel sign on the READ path only. A radio with servo-reverse set on one drive
    # channel reports it negated while the ESC on that side compensates, so the robot moves
    # correctly and only the readback disagrees. Without these, a straight command un-mixes
    # into a pure turn and the contamination gate fires on a perfectly good run. Find them
    # with `velocity_jig_drive.py --check-radio`.
    read_invert_a: bool = False
    read_invert_b: bool = False

    def to_channels(self, linear: float, angular: float) -> tuple[int, int]:
        ang = -angular if self.reverse_angular else angular
        if self.mode != "tank":
            return to_trainer(linear), to_trainer(ang)
        # Angular has priority: saturate it first, then give the forward command whatever
        # authority is left. A turn that gets clipped instead is a turn the fit sees as
        # commanded but never delivered.
        ang = max(-1.0, min(1.0, ang))
        room = 1.0 - abs(ang)
        lin = max(-room, min(room, linear))
        return to_trainer(lin + ang), to_trainer(lin - ang)

    def from_channels(self, a: float, b: float) -> tuple[float, float]:
        """Inverse of `to_channels`, so the log is in body units whatever the mix."""
        if self.read_invert_a:
            a = -a
        if self.read_invert_b:
            b = -b
        if self.mode != "tank":
            lin, ang = a, b
        else:
            lin, ang = 0.5 * (a + b), 0.5 * (a - b)
        return lin, (-ang if self.reverse_angular else ang)


@dataclass
class CommandSample:
    """One tick: what was asked for, what was sent, and what the radio said it sent."""

    t: float  # time.monotonic()
    linear: float
    angular: float
    trim: float
    channel_a: int
    channel_b: int
    # As reported by the radio. The channels are the measurement; linear and angular are
    # derived from them through the mix, and are kept only for convenience.
    meas_ch_a: float = float("nan")
    meas_ch_b: float = float("nan")
    meas_linear: float = float("nan")
    meas_angular: float = float("nan")
    meas_arm: float = float("nan")
    label: str = ""


class _ChannelDecoder:
    """Frames the radio's channel stream. Silent on corruption: a noisy link should not
    spam the console during a run."""

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[tuple[int, list[int]]]:
        self._buf.extend(data)
        out: list[tuple[int, list[int]]] = []
        while True:
            i = self._buf.find(_SYNC)
            if i < 0:
                # Keep the last two bytes: a three-byte sync can straddle two reads.
                if len(self._buf) > 2:
                    del self._buf[: len(self._buf) - 2]
                break
            if i > 0:
                del self._buf[:i]
            if len(self._buf) < len(_SYNC) + 2 + _PACKET_LEN:
                break
            phase = self._buf[len(_SYNC)]
            length = self._buf[len(_SYNC) + 1]
            if length != _PACKET_LEN:
                del self._buf[:1]  # bad framing; resync on the next sync word
                continue
            start = len(_SYNC) + 2
            payload = bytes(self._buf[start : start + _PACKET_LEN])
            del self._buf[: start + _PACKET_LEN]
            chk = phase ^ length
            for b in payload[:-1]:
                chk ^= b
            if chk != payload[-1] or phase not in (0, 1):
                continue
            out.append(
                (
                    phase,
                    [
                        struct.unpack_from("<h", payload, j)[0]
                        for j in range(0, _CHANNELS_PER_PACKET * 2, 2)
                    ],
                )
            )
        return out


class TrainerLink:
    """Serial link to the radio. Writes trainer commands, decodes the channel stream."""

    def __init__(self, port: str, mix: MixConfig | None = None, *, read_back: bool = True):
        import serial  # lazy so --dry-run needs no pyserial

        self.mix = mix or MixConfig()
        self._serial = serial.Serial(port, baudrate=115200, timeout=0.1)
        self._channels = [0] * _NUM_CHANNELS
        self._packets = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        # Re-prime, same as OpenTxTransmitter::initialize().
        self._serial.write(b"telemetry on\r\n")
        self._serial.write(b"channels on\r\n")
        self._reader: threading.Thread | None = None
        if read_back:
            self._reader = threading.Thread(target=self._run, name="trainer-reader", daemon=True)
            self._reader.start()

    def _run(self) -> None:
        decoder = _ChannelDecoder()
        while not self._stop.is_set():
            try:
                n = self._serial.in_waiting or 1
                data = self._serial.read(n)
            except Exception:
                break
            if not data:
                continue
            for phase, channels in decoder.feed(data):
                base = phase * _CHANNELS_PER_PACKET
                with self._lock:
                    self._channels[base : base + _CHANNELS_PER_PACKET] = channels
                    self._packets += 1

    @property
    def packets(self) -> int:
        with self._lock:
            return self._packets

    def measured_raw(self) -> list[float]:
        """Every channel, normalized by channel_scale and otherwise untouched.

        `measured()` applies the mix and the read inversions; this is what they are inferred
        from, so it must stay raw.
        """
        with self._lock:
            if self._packets == 0:
                return []
            raw = list(self._channels)
        scale = self.mix.channel_scale or 1.0
        return [v / scale for v in raw]

    def measured(self) -> tuple[float, float, float]:
        """Latest (channel a, channel b, arm) from the radio, normalized.

        Deliberately the two raw drive channels rather than body units. The radio measures
        channels; linear and angular are an interpretation that depends on the mix and the
        per-channel signs, and getting those wrong should be fixable offline rather than
        costing a re-record. `to_body()` applies the interpretation when one is wanted.
        """
        with self._lock:
            if self._packets == 0:
                return float("nan"), float("nan"), float("nan")
            raw = list(self._channels)
        scale = self.mix.channel_scale or 1.0
        return (
            raw[self.mix.read_linear] / scale,
            raw[self.mix.read_angular] / scale,
            raw[self.mix.read_arm] / scale,
        )

    def to_body(self, a: float, b: float) -> tuple[float, float]:
        """Two drive channels to (linear, angular), under the configured mix."""
        if a != a or b != b:  # NaN in, NaN out
            return float("nan"), float("nan")
        return self.mix.from_channels(a, b)

    def send(self, linear: float, angular: float) -> tuple[int, int]:
        a, b = self.mix.to_channels(linear, angular)
        self._serial.write(f"trainer {CHANNEL_A} {a}\r\n".encode())
        self._serial.write(f"trainer {CHANNEL_B} {b}\r\n".encode())
        return a, b

    def disarm(self) -> None:
        try:
            self._serial.write(f"trainer {CHANNEL_A} 0\r\n".encode())
            self._serial.write(f"trainer {CHANNEL_B} 0\r\n".encode())
            self._serial.flush()
        except Exception:
            pass

    def close(self) -> None:
        self.disarm()
        self._stop.set()
        if self._reader:
            self._reader.join(timeout=0.5)
        try:
            self._serial.close()
        except Exception:
            pass


@dataclass
class PlayResult:
    commands: list[CommandSample] = field(default_factory=list)
    completed: bool = False
    aborted_reason: str = ""

    # Ticks to ignore after the command changes. The radio reports its mixer output on its
    # own schedule, so `measured` lags `commanded` by a tick or two. At a step edge that lag
    # makes the two differ by the full step height, which has nothing to do with the driver.
    SETTLE_TICKS = 4

    @property
    def contamination(self) -> float:
        """How far the radio's own report of what it sent diverges from what we asked for.

        Trainer mode sums the scripted command with the human's sticks, so a persistent gap
        means a second, unlogged input is in the run. Anything above roughly 0.05 is worth
        discarding over.

        Measured over held stretches only, and reported as a high percentile rather than the
        maximum. Both matter: comparing sample-by-sample across a step edge measures the
        radio's reporting lag, and a single outlying tick is not evidence of a leaning stick.
        """
        diffs = []
        settle = 0
        prev: tuple[float, float] | None = None
        for c in self.commands:
            target = (c.linear, c.angular + c.trim)
            if prev is not None and target != prev:
                settle = self.SETTLE_TICKS
            prev = target
            if settle > 0:
                settle -= 1
                continue
            if c.meas_linear != c.meas_linear:  # NaN: no readback yet
                continue
            diffs.append(max(abs(c.meas_linear - target[0]), abs(c.meas_angular - target[1])))
        if not diffs:
            return 0.0
        import numpy as np

        return float(np.percentile(diffs, 95))


def play(
    link: TrainerLink,
    program,
    *,
    rate_hz: float = 50.0,
    trim: float = 0.0,
    stop_event: threading.Event | None = None,
    timeout_s: float | None = None,
    on_tick: Callable[[CommandSample], None] | None = None,
) -> PlayResult:
    """Play one excitation program, logging every tick on CLOCK_MONOTONIC.

    `trim` is added to the angular channel and recorded in its own column. It is never a
    hidden offset: a trimmed straight run is a two-input excitation, and a fit that does not
    know about the second input attributes its effect to the first.

    The caller owns disarm. This returns on completion, on `stop_event`, or on the hard
    timeout, and in every case the last thing it sends is a zero command.
    """
    result = PlayResult()
    period = 1.0 / rate_hz
    t0 = time.monotonic()
    deadline = t0 + (timeout_s if timeout_s is not None else program.duration_s + 10.0)
    try:
        while True:
            now = time.monotonic()
            elapsed = now - t0
            if elapsed >= program.duration_s:
                result.completed = True
                break
            if now >= deadline:
                result.aborted_reason = "hard wall-clock timeout"
                break
            if stop_event is not None and stop_event.is_set():
                result.aborted_reason = "stop requested"
                break

            linear, angular = program.at(elapsed)
            a, b = link.send(linear, angular + trim)
            ch_a, ch_b, meas_arm = link.measured()
            meas_lin, meas_ang = link.to_body(ch_a, ch_b)
            sample = CommandSample(
                t=now,
                linear=linear,
                angular=angular,
                trim=trim,
                channel_a=a,
                channel_b=b,
                meas_ch_a=ch_a,
                meas_ch_b=ch_b,
                meas_linear=meas_lin,
                meas_angular=meas_ang,
                meas_arm=meas_arm,
                label=_label_at(program, elapsed),
            )
            result.commands.append(sample)
            if on_tick:
                on_tick(sample)
            time.sleep(max(0.0, period - (time.monotonic() - now)))
    finally:
        link.send(0.0, 0.0)
    return result


def _label_at(program, t: float) -> str:
    if not getattr(program, "segments", None):
        return program.kind
    for seg in program.segments:
        if seg.t0 <= t < seg.t1:
            return seg.label
    return "idle"


class armed:
    """Context manager that guarantees a disarm.

    Every exit path goes through __exit__: normal return, exception, and Ctrl-C, since
    KeyboardInterrupt is an exception. The runbook asks the operator to verify this once per
    session with a deliberate Ctrl-C before trusting the rest of the day.
    """

    def __init__(self, link: TrainerLink) -> None:
        self._link = link

    def __enter__(self) -> TrainerLink:
        return self._link

    def __exit__(self, *exc: object) -> None:
        self._link.disarm()


def stream_measured(link: TrainerLink, seconds: float, rate_hz: float = 50.0) -> Iterator[
    tuple[float, float, float, float, float, float]
]:
    """(t, ch_a, ch_b, linear, angular, arm) from the radio alone, nothing commanded.

    This is how a hand-driven run gets a command log: the tool sends nothing, the operator
    drives, and the radio reports what it sent.
    """
    period = 1.0 / rate_hz
    t0 = time.monotonic()
    while True:
        now = time.monotonic()
        if now - t0 >= seconds:
            return
        ch_a, ch_b, arm = link.measured()
        lin, ang = link.to_body(ch_a, ch_b)
        yield now, ch_a, ch_b, lin, ang, arm
        time.sleep(max(0.0, period - (time.monotonic() - now)))
