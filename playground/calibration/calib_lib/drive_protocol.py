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

import math
import threading
import time
from collections.abc import Collection
from dataclasses import dataclass

# Matches kChannelMax / kTrainerMax in opentx_transmitter.cpp.
TRAINER_MAX = 500
LINEAR_CHANNEL = 0
ANGULAR_CHANNEL = 1

# Excitation sizing. A moving segment's duration is shrunk so the robot does not run out of the
# overhead camera's view (linear) or spin an unbounded number of turns (angular). Tuned as single
# knobs after the first real run.
SAFETY_FRACTION = 0.5  # fraction of view_size a translation phase may cross before it must stop
N_ROT = 3.0  # rotations a sustained/step spin is allowed to complete
TWITCH_ANGLE = 0.5  # rad a sync twitch rotates, regardless of robot (keeps the pulse crisp)
SLEW_DT = 0.05  # sub-step (s) used to approximate a slew-limited command ramp

# Selectable excitation phases, in run order, for build_protocol(phases=...) and --phases. A subset
# run still gets the in-place sync twitches at both ends: they cost no travel, and the fitter needs
# them to cross-correlate the command log against the AprilTag capture. Everything else is opt-in.
# lin_step alone yields both the accel (rise) and decel/coast (drop-to-zero) time constants.
PHASES: tuple[str, ...] = (
    "idle",
    "lin_deadzone",
    "lin_step",
    "ang_deadzone",
    "ang_step",
    "lin_max",
    "ang_max",
    "steer_brake",
    "latency",
)


@dataclass
class Segment:
    """Hold (linear, angular) normalized command for `duration` seconds.

    `linear` and `angular` are in [-1, 1]; they map to trainer values via round(x * TRAINER_MAX).
    `label` tags the maneuver so the fitter can slice the log by phase.

    A `checkpoint` segment is a zero-duration, zero-command marker between phases: DriveRunner stops
    the robot and blocks for the operator to reposition it (Enter to continue). It issues no command,
    so it never lands in the recording or the fitter's label slices.
    """

    duration: float
    linear: float
    angular: float
    label: str
    checkpoint: bool = False


@dataclass
class DriveSpecs:
    """Drivetrain geometry for one robot, the input to build_protocol().

    We take physical specs, not max velocities, because max velocity is exactly what the calibration
    measures: requiring it as input would be circular. The theoretical maxima are derived here purely
    to size the excitation so the robot stays in the overhead camera's view.
    """

    shaft_rpm: float  # output-shaft (wheel) max RPM
    wheel_diameter_m: float  # drive wheel diameter, metres
    wheel_base_m: float  # left-right wheel track, metres
    view_size_m: float  # usable camera footprint at the floor, shorter dimension, metres
    # Max forward acceleration in command units per second before the robot flips. Torque-happy bots
    # (e.g. Mr Stabs) backflip on a full-command launch; it is the acceleration, not the top speed, that
    # pitches them over. Measure it with find_flip_accel.py. When set, build_protocol slew-limits every
    # forward command INCREASE to this rate, so the robot reaches full command via a safe ramp instead
    # of a launch step. None = step instantly (robots that don't flip); the natural step response is
    # then measured directly.
    max_accel: float | None = None

    @property
    def v_max(self) -> float:
        """Theoretical max linear speed, m/s (both wheels at shaft_rpm)."""
        return math.pi * self.wheel_diameter_m * (self.shaft_rpm / 60.0)

    @property
    def omega_max(self) -> float:
        """Theoretical max yaw rate, rad/s (wheels counter-rotating, differential drive)."""
        return 2.0 * self.v_max / self.wheel_base_m


# Per-robot presets, selected with apriltag_track.py --robot NAME.
# TODO: measure and fill real specs for each robot (shaft RPM, wheel diameter, wheel base, and the
# usable camera footprint at the floor). The placeholders below only make --dry-run runnable.
ROBOTS: dict[str, DriveSpecs] = {
    "mr_stabs_mk2": DriveSpecs(
        shaft_rpm=1500.0,
        wheel_diameter_m=0.050,
        wheel_base_m=0.131,
        view_size_m=1.6,
        max_accel=2.16,  # measured with find_flip_accel.py: max safe ~8.5 m/s^2 before it backflips
    ),
    "mrs_buff_mk3": DriveSpecs(
        shaft_rpm=1500.0,
        wheel_diameter_m=0.050,
        wheel_base_m=0.195,
        view_size_m=1.6,
        # max_accel left at None: heavier, does not flip on launch. Set it if it ever does.
    ),
}


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


def build_protocol(specs: DriveSpecs, phases: Collection[str] | None = None) -> list[Segment]:
    """The excitation sequence (see docs/experiments/control_improvement and the calibration plan).

    Moving-segment durations are sized from `specs` so a full-speed maneuver stays inside the camera
    view (linear) or completes a bounded number of turns (angular), and a checkpoint (operator
    repositions the robot, Enter to continue) sits between phases that would otherwise drift the robot
    out of frame. Pure and deterministic so it can be inspected with --dry-run without hardware.

    `phases` (names from PHASES) restricts the run to a subset for focused re-tuning, e.g. just
    "lin_step" to iterate on accel/decel tau without driving the full battery. None runs everything.
    The sync twitches always bracket the output regardless of the filter. Each phase starts and ends
    at rest, so dropping phases never changes the ones that remain.
    """
    selected = None if phases is None else frozenset(phases)

    def want(name: str) -> bool:
        return selected is None or name in selected

    v_max = specs.v_max
    omega_max = specs.omega_max
    max_accel = specs.max_accel  # forward command/s slew limit (None = step instantly)
    travel_budget = specs.view_size_m * SAFETY_FRACTION  # metres a translation phase may cross
    rot_budget = N_ROT * 2.0 * math.pi  # radians a spin phase may sweep

    seg: list[Segment] = []
    prev_lin = 0.0  # last commanded forward value, for the slew limiter

    def hold(duration: float, lin: float, ang: float, label: str) -> None:
        seg.append(Segment(duration, lin, ang, label))

    def checkpoint(next_phase: str) -> None:
        seg.append(Segment(0.0, 0.0, 0.0, f"checkpoint:{next_phase}", checkpoint=True))

    def capped(nominal: float, command: float, speed: float, budget: float) -> float:
        """Shrink `nominal` so |command|*speed*duration <= budget (distance or angle covered).

        A rest/coast segment (command 0) keeps its nominal duration.
        """
        move = abs(command) * speed
        return nominal if move <= 0.0 else min(nominal, budget / move)

    def drive_to(target: float, hold_dur: float, label: str, ang: float = 0.0) -> None:
        """Move the forward command to `target`, then hold for `hold_dur` (with optional yaw `ang`).

        With max_accel set, an INCREASE in command magnitude is slew-limited to that rate so a
        torque-happy bot ramps up instead of launching into a wheelie; the ramp sub-steps are labeled
        "slew" so they stay out of the fitter's phase slices. A decrease (e.g. back to 0) steps
        instantly, preserving the natural coast/decel the fitter measures.
        """
        nonlocal prev_lin
        if max_accel is not None and abs(target) > abs(prev_lin) + 1e-9:
            delta = target - prev_lin
            n = max(1, round(abs(delta) / max_accel / SLEW_DT))
            for i in range(1, n + 1):
                hold(SLEW_DT, prev_lin + delta * i / n, 0.0, "slew")
        if hold_dur > 0.0:
            hold(hold_dur, target, ang, label)
        prev_lin = target

    # 8. Sync twitch (start): a sharp, short yaw pulse to cross-correlate the command log against the
    #    AprilTag capture (recorded in the same process, so the two already share a clock). In-place,
    #    so scale to a fixed small angle, floored so it stays long enough to register on both logs.
    twitch = max(0.1, TWITCH_ANGLE / (0.8 * omega_max))
    hold(twitch, 0.0, 0.8, "sync_start")
    hold(twitch, 0.0, -0.8, "sync_start")
    hold(1.0, 0.0, 0.0, "sync_start")

    # 1. Idle baseline: drift + ground-truth noise floor. Follows the in-place twitch, robot still
    #    centred, so no checkpoint before it.
    if want("idle"):
        hold(3.0, 0.0, 0.0, "idle")

    # 2. Linear deadzone staircase: creep the command up until the wheels break static friction. Each
    #    direction marches the robot the same way, so recentre before each and bound the *cumulative*
    #    crawl across the seven steps to one travel budget.
    if want("lin_deadzone"):
        step_budget = travel_budget / 7.0
        for sign, tag in ((1.0, "fwd"), (-1.0, "rev")):
            checkpoint(f"lin_deadzone_{tag}")
            for frac in (0.04, 0.08, 0.12, 0.16, 0.20, 0.24, 0.28):
                hold(capped(0.8, frac, v_max, step_budget), sign * frac, 0.0, f"lin_deadzone_{tag}")
            hold(1.0, 0.0, 0.0, f"lin_deadzone_{tag}")

    # 3. Linear steps: rise to steady state, then step to zero to capture coast (decel) tau. The
    #    rise is slew-limited (when max_accel is set) so a torque-happy bot does not wheelie; the
    #    drop to zero always steps so the natural coast is measured. Each step's post-drop coast is
    #    uncounted travel (that IS the decel tau), so at high amp the robot ends near the edge:
    #    checkpoint before EVERY step for the operator to recentre, giving each drive its full view.
    if want("lin_step"):
        for amp in (0.25, 0.5, 0.75, 1.0):
            for sign in (1.0, -1.0):
                checkpoint("lin_step")
                drive_to(sign * amp, capped(1.2, amp, v_max, travel_budget), "lin_step")
                drive_to(0.0, 1.2, "lin_step")

    # 4. Angular deadzone staircase + yaw steps, both directions. Spin-in-place stays centred, so one
    #    checkpoint before the whole angular block (no recentre between left and right) and the limit
    #    is rotations, not view.
    if want("ang_deadzone"):
        checkpoint("ang_deadzone")
        for sign, tag in ((1.0, "left"), (-1.0, "right")):
            for frac in (0.04, 0.08, 0.12, 0.16, 0.20, 0.24, 0.28):
                hold(
                    capped(0.8, frac, omega_max, rot_budget),
                    0.0,
                    sign * frac,
                    f"ang_deadzone_{tag}",
                )
            hold(0.8, 0.0, 0.0, f"ang_deadzone_{tag}")
    if want("ang_step"):
        checkpoint("ang_step")
        for amp in (0.25, 0.5, 0.75, 1.0):
            for sign in (1.0, -1.0):
                hold(capped(1.0, amp, omega_max, rot_budget), 0.0, sign * amp, "ang_step")
                hold(1.0, 0.0, 0.0, "ang_step")

    # 5. Sustained max speed: slew up to full command (no launch wheelie when max_accel is set), then
    #    hold for the steady-state gain. The slew sub-steps are labeled "slew" so they stay out of the
    #    fitter's lin_max/lin_step max-speed slice; only the steady hold carries the lin_max label.
    if want("lin_max"):
        checkpoint("lin_max")
        drive_to(1.0, capped(1.0, 1.0, v_max, travel_budget), "lin_max")
        drive_to(0.0, 1.5, "lin_max")
    if want("ang_max"):
        checkpoint("ang_max")
        hold(capped(2.0, 1.0, omega_max, rot_budget), 0.0, 1.0, "ang_max")
        hold(1.5, 0.0, 0.0, "ang_max")

    # 6. Steer-brake grid: forward speed loss as a function of |yaw command|. Each 0.7 forward pulse
    #    drives across the view, so checkpoint before EVERY pulse for the operator to recentre; each
    #    pulse then gets the full travel budget. The forward rise is slew-limited (when max_accel is
    #    set); the yaw is applied on the steady hold.
    if want("steer_brake"):
        for ang in (0.0, 0.25, 0.5, 0.75):
            checkpoint(f"steer_brake ang={ang:g}")
            drive_to(0.7, capped(1.5, 0.7, v_max, travel_budget), "steer_brake", ang=ang)
            drive_to(0.0, 0.8, "steer_brake")

    # 7. Latency battery: many sharp +/- steps for actuation-lag cross-correlation. Sharp edges are the
    #    point, so these are NOT slew-limited; instead a flip-prone bot (max_accel set) uses a small
    #    amplitude that is still crisp but stays under the wheelie threshold (0.35 still flipped).
    if want("latency"):
        checkpoint("latency")
        lat_amp = 0.2 if max_accel is not None else 0.8
        lat = capped(0.3, lat_amp, v_max, travel_budget)
        for _ in range(8):
            hold(lat, lat_amp, 0.0, "latency")
            hold(lat, -lat_amp, 0.0, "latency")

    # 8. Sync twitch (end).
    checkpoint("sync_end")
    hold(1.0, 0.0, 0.0, "sync_end")
    hold(twitch, 0.0, 0.8, "sync_end")
    hold(twitch, 0.0, -0.8, "sync_end")
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

    On a checkpoint segment the runner stops the robot, sets `pause_event`, and blocks on `resume_event`
    (still honoring `stop_event`). apriltag_track.py owns the operator prompt and the hard-timeout while
    paused, so this stays UI-agnostic.
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
        # Checkpoint handshake: the runner sets pause_event (with pause_label naming the upcoming
        # phase) and waits for the operator (via apriltag_track.py) to set resume_event.
        self.pause_event = threading.Event()
        self.resume_event = threading.Event()
        self.pause_label = ""

    def run(self) -> None:
        # Timestamps are absolute time.monotonic() (CLOCK_MONOTONIC), the same clock the camera frames are
        # stamped with in this process, so the command log and the truth poses need no alignment.
        try:
            for s in self._protocol:
                if s.checkpoint:
                    # Stop the robot and block until the operator repositions it and resumes. A stop
                    # during the pause still aborts cleanly.
                    self._link.send(0.0, 0.0)
                    self.pause_label = s.label
                    self.resume_event.clear()
                    self.pause_event.set()
                    while not self.resume_event.wait(timeout=0.1):
                        if self._stop_event.is_set():
                            self._link.send(0.0, 0.0)
                            return
                    self.pause_event.clear()
                    continue
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
