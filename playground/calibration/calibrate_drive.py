"""Standalone excitation driver for Mrs Buff MK3 drivetrain characterization.

Drives the OpenTX trainer link directly (no C++ binary) through a scripted command sequence, and logs
every issued command with a monotonic timestamp to CSV. Pair it with apriltag_track.py (overhead ground
truth) and feed both CSVs to fit_plant_calib.py.

This mirrors the trainer protocol in src/transmitter/opentx_transmitter.cpp: a USB CDC serial device
(VID=0x0483, PID=0x5740), primed with `telemetry on` + `channels on`, commanded with
`trainer <channel> <value>` where channel 0 = linear, channel 1 = angular, value in [-500, 500].

Unlike the deployed transmitter, this sends RAW commands and does NOT apply the DifferentialDriveProcessor
deadzone (lifted_deadzone_percent / zero_deadzone_percent). The physical deadzone is exactly what we are
measuring, so it must not be pre-compensated here.

SAFETY
  - The robot moves fast. Run in a clear, bounded space with the guard plates on (see am32_tuning.md).
  - Keep the human driver's sticks centered: in trainer mode the radio ADDS stick input to these commands.
  - The script zeroes the channels and disarms on normal exit, Ctrl-C, and any exception, and enforces a
    hard wall-clock timeout.

Usage:
    source scripts/activate_python.sh
    # Dry run: print the schedule, touch no hardware.
    python playground/calibration/calibrate_drive.py --dry-run
    # Real run (auto-detects the OpenTX device):
    python playground/calibration/calibrate_drive.py --out playground/calibration/out/cmd_log.csv
"""

from __future__ import annotations

import argparse
import csv
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path

# Matches kChannelMax / kTrainerMax and the device IDs in opentx_transmitter.cpp.
TRAINER_MAX = 500
OPENTX_VID = 0x0483
OPENTX_PID = 0x5740
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


def build_protocol() -> list[Segment]:
    """The excitation sequence (see docs/experiments/control_improvement and the calibration plan).

    Pure and deterministic so it can be inspected with --dry-run and unit-tested without hardware.
    """
    seg: list[Segment] = []

    def hold(duration: float, lin: float, ang: float, label: str) -> None:
        seg.append(Segment(duration, lin, ang, label))

    # 8. Sync twitch (start): a sharp, short yaw pulse to cross-correlate the command log against the
    #    AprilTag capture, which runs on its own clock.
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


def find_opentx_port() -> str | None:
    """Auto-detect the OpenTX radio by USB VID/PID, like find_opentx_device() on the C++ side."""
    from serial.tools import list_ports

    for p in list_ports.comports():
        if p.vid == OPENTX_VID and p.pid == OPENTX_PID:
            return p.device
    return None


def run(protocol: list[Segment], link: TrainerLink, rate_hz: float, writer: "csv._writer") -> None:
    # Timestamps are absolute time.monotonic() (CLOCK_MONOTONIC). On Linux that clock is system-wide, so
    # when apriltag_track.py runs on the same host the two logs share a zero and need no sync.
    period = 1.0 / rate_hz
    for s in protocol:
        seg_end = time.monotonic() + s.duration
        while time.monotonic() < seg_end:
            lin_val, ang_val = link.send(s.linear, s.angular)
            writer.writerow([f"{time.monotonic():.4f}", s.linear, s.angular, lin_val, ang_val, s.label])
            time.sleep(period)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("playground/calibration/out/cmd_log.csv"))
    parser.add_argument("--port", type=str, default=None, help="serial device; auto-detected if omitted")
    parser.add_argument("--rate", type=float, default=50.0, help="command send rate (Hz)")
    parser.add_argument(
        "--no-reverse-angular",
        action="store_true",
        help="disable the reverse_angular convention (default matches main.toml: enabled)",
    )
    parser.add_argument("--dry-run", action="store_true", help="print the schedule, touch no hardware")
    args = parser.parse_args()

    protocol = build_protocol()
    total = protocol_duration(protocol)

    if args.dry_run:
        t = 0.0
        for s in protocol:
            print(f"  {t:6.2f}s  +{s.duration:4.2f}s  lin={s.linear:+.2f} ang={s.angular:+.2f}  {s.label}")
            t += s.duration
        print(f"\nTotal: {total:.1f}s, {len(protocol)} segments. (dry run, no hardware touched)")
        return

    port = args.port or find_opentx_port()
    if not port:
        sys.exit("No OpenTX device found (VID=0x0483 PID=0x5740). Pass --port, or check the cable.")

    print(f"OpenTX device: {port}")
    print(f"Protocol: {total:.1f}s. Guard plates ON, clear space, driver sticks CENTERED.")
    if input("Type 'go' to arm and run: ").strip() != "go":
        sys.exit("Aborted.")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    link = TrainerLink(port, reverse_angular=not args.no_reverse_angular)

    # Disarm on any exit path: normal return, Ctrl-C, SIGTERM, exception, or the hard timeout.
    def _stop(*_: object) -> None:
        link.close()
        sys.exit("Stopped (disarmed).")

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGALRM, _stop)
    signal.alarm(int(total) + 5)  # hard timeout guard

    try:
        with open(args.out, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["t", "cmd_lin", "cmd_ang", "trainer_lin", "trainer_ang", "label"])
            run(protocol, link, args.rate, writer)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        link.close()
        print(f"Disarmed. Command log: {args.out}")


if __name__ == "__main__":
    main()
