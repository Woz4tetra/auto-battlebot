"""Scripted excitation driver for the Mrs Buff MK3 drivetrain characterization.

This is the write counterpart to transmitter_axes.py (which only reads the radio). apriltag_track.py
--drive uses it to play a deterministic command sequence through the OpenTX trainer link while the same
process records the overhead camera and the issued commands, so the command log and the AprilTag
ground-truth share one CLOCK_MONOTONIC and need no time alignment.

It mirrors the trainer protocol in src/transmitter/opentx_transmitter.cpp: a USB CDC serial device (the
same VID=0x0483, PID=0x5740 that transmitter_axes.find_transmitter_port() auto-detects), primed with
`telemetry on` + `channels on`, commanded `trainer <channel> <value>` where channel 0 = linear, channel
1 = angular, value in [-500, 500].

Unlike the deployed transmitter, it sends RAW commands and does NOT apply the DifferentialDriveProcessor
deadzone (lifted_deadzone_percent / zero_deadzone_percent): the physical deadzone is exactly what we are
measuring, so it must not be pre-compensated here.

SAFETY
- The robot moves fast. Run in a clear, bounded space with guard plates on (see am32_tuning.md).
- Keep the human driver's sticks centered: in trainer mode the radio ADDS stick input to the command.
- apriltag_track.py zeroes the channels and disarms on every exit path (normal, Ctrl-C, exception, hard
  wall-clock timeout) and gates the run behind an explicit arm.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

# Matches kChannelMax / kTrainerMax in opentx_transmitter.cpp.
TRAINER_MAX = 500
LINEAR_CHANNEL = 0
ANGULAR_CHANNEL = 1


@dataclass
class Segment:
    """Hold (linear, angular) normalized command for `duration` seconds.

    `linear` and `angular` are in [-1, 1]; they map to trainer values via round(x * TRAINER_MAX).
    `label` tags the maneuver so the fitter can slice the log by phase.
    """

    duration: float
    linear: float
    angular: float
    label: str


@dataclass
class CommandSample:
    """One issued command, stamped with the CLOCK_MONOTONIC time it was sent.

    DriveRunner hands these to apriltag_track.py's capture loop (the sole MCAP writer) via a queue, so the
    issued commands land in the recording on the same clock as the camera frames.
    """

    t: float
    cmd_lin: float
    cmd_ang: float
    trainer_lin: int
    trainer_ang: int
    label: str


def build_protocol() -> list[Segment]:
    """The excitation sequence (see docs/experiments/control_improvement and the calibration plan).

    Pure and deterministic so it can be inspected with --dry-run and unit-tested without hardware.
    """
    seg: list[Segment] = []

    def hold(duration: float, lin: float, ang: float, label: str) -> None:
        seg.append(Segment(duration, lin, ang, label))

    # 8. Sync twitch (start): a sharp, short yaw pulse to cross-correlate the command log against the
    #    AprilTag capture (recorded in the same process, so the two already share a clock).
    hold(0.15, 0.0, 0.8, "sync_start")
    hold(0.15, 0.0, -0.8, "sync_start")
    hold(1.0, 0.0, 0.0, "sync_start")

    # 1. Idle baseline: drift + ground-truth noise floor.
    hold(3.0, 0.0, 0.0, "idle")

    # 2. Linear deadzone staircase: creep the command up until the wheels break static friction.
    for sign, tag in ((1.0, "fwd"), (-1.0, "rev")):
        for frac in (0.04, 0.08, 0.12, 0.16, 0.20, 0.24, 0.28):
            hold(0.8, sign * frac, 0.0, f"lin_deadzone_{tag}")
        hold(1.0, 0.0, 0.0, f"lin_deadzone_{tag}")

    # 3. Linear steps: rise to steady state, then step to zero to capture coast (decel) tau.
    for amp in (0.25, 0.5, 0.75, 1.0):
        for sign in (1.0, -1.0):
            hold(1.2, sign * amp, 0.0, "lin_step")
            hold(1.2, 0.0, 0.0, "lin_step")

    # 4. Angular deadzone staircase + yaw steps, both directions.
    for sign, tag in ((1.0, "left"), (-1.0, "right")):
        for frac in (0.04, 0.08, 0.12, 0.16, 0.20, 0.24, 0.28):
            hold(0.8, 0.0, sign * frac, f"ang_deadzone_{tag}")
        hold(0.8, 0.0, 0.0, f"ang_deadzone_{tag}")
    for amp in (0.25, 0.5, 0.75, 1.0):
        for sign in (1.0, -1.0):
            hold(1.0, 0.0, sign * amp, "ang_step")
            hold(1.0, 0.0, 0.0, "ang_step")

    # 5. Sustained max: steady-state gain (forward + spin). Keep short; the arena is small.
    hold(1.5, 1.0, 0.0, "lin_max")
    hold(1.5, 0.0, 0.0, "lin_max")
    hold(2.0, 0.0, 1.0, "ang_max")
    hold(1.5, 0.0, 0.0, "ang_max")

    # 6. Steer-brake grid: forward speed loss as a function of |yaw command|.
    for ang in (0.0, 0.25, 0.5, 0.75):
        hold(1.5, 0.7, ang, "steer_brake")
        hold(0.8, 0.0, 0.0, "steer_brake")

    # 7. Latency battery: many sharp +/- steps for actuation-lag cross-correlation.
    for _ in range(8):
        hold(0.3, 0.8, 0.0, "latency")
        hold(0.3, -0.8, 0.0, "latency")

    # 8. Sync twitch (end).
    hold(1.0, 0.0, 0.0, "sync_end")
    hold(0.15, 0.0, 0.8, "sync_end")
    hold(0.15, 0.0, -0.8, "sync_end")
    return seg


def protocol_duration(protocol: list[Segment]) -> float:
    return sum(s.duration for s in protocol)


def to_trainer(value: float) -> int:
    """Normalized [-1, 1] command -> trainer integer, clamped, matching to_trainer_value()."""
    return max(-TRAINER_MAX, min(TRAINER_MAX, round(value * TRAINER_MAX)))


class TrainerLink:
    """Thin wrapper over the OpenTX USB CDC serial link. Sends raw trainer channel commands."""

    def __init__(self, port: str, reverse_angular: bool) -> None:
        import serial  # lazy import so --dry-run needs no pyserial

        self._serial = serial.Serial(port, baudrate=115200, timeout=0.1)
        self._reverse_angular = reverse_angular
        # Re-prime the OpenTX-side streams, same as OpenTxTransmitter::initialize().
        self._serial.write(b"telemetry on\r\n")
        self._serial.write(b"channels on\r\n")

    def send(self, linear: float, angular: float) -> tuple[int, int]:
        # reverse_angular_channel = true in main.toml; keep the same convention so "+angular" turns the
        # robot the same way it does in deployment.
        ang = -angular if self._reverse_angular else angular
        lin_val = to_trainer(linear)
        ang_val = to_trainer(ang)
        self._serial.write(f"trainer {LINEAR_CHANNEL} {lin_val}\r\n".encode())
        self._serial.write(f"trainer {ANGULAR_CHANNEL} {ang_val}\r\n".encode())
        return lin_val, ang_val

    def disarm(self) -> None:
        try:
            self._serial.write(f"trainer {LINEAR_CHANNEL} 0\r\n".encode())
            self._serial.write(f"trainer {ANGULAR_CHANNEL} 0\r\n".encode())
            self._serial.flush()
        except Exception:
            pass

    def close(self) -> None:
        self.disarm()
        try:
            self._serial.close()
        except Exception:
            pass


class DriveRunner(threading.Thread):
    """Plays the excitation protocol on a daemon thread, independent of the camera capture loop.

    Each tick sends one command at `rate_hz` and hands a CommandSample (stamped with the send time) to
    `sink`, which apriltag_track.py wires to the queue its capture loop drains; that loop is the sole MCAP
    writer, so the commands are recorded without a second thread touching the writer. `stop_event` aborts
    the run (sending a zero command first); `finished` is set on any exit and `completed` only when the
    protocol ran to its end. The runner never closes the link: the caller owns it and disarms on exit.
    """

    def __init__(
        self,
        link: TrainerLink,
        protocol: list[Segment],
        rate_hz: float,
        sink,
        stop_event: threading.Event,
    ) -> None:
        super().__init__(name="drive-runner", daemon=True)
        self._link = link
        self._protocol = protocol
        self._period = 1.0 / rate_hz
        self._sink = sink
        self._stop_event = stop_event
        self.finished = threading.Event()
        self.completed = False

    def run(self) -> None:
        # Timestamps are absolute time.monotonic() (CLOCK_MONOTONIC), the same clock the camera frames are
        # stamped with in this process, so the command log and the truth poses need no alignment.
        try:
            for s in self._protocol:
                seg_end = time.monotonic() + s.duration
                while time.monotonic() < seg_end:
                    if self._stop_event.is_set():
                        self._link.send(
                            0.0, 0.0
                        )  # cut output immediately; caller still disarms on exit
                        return
                    lin_val, ang_val = self._link.send(s.linear, s.angular)
                    self._sink(
                        CommandSample(
                            time.monotonic(), s.linear, s.angular, lin_val, ang_val, s.label
                        )
                    )
                    time.sleep(self._period)
            self.completed = True
        finally:
            self.finished.set()
