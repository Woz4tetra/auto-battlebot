#!/usr/bin/env python3
"""Find the max forward acceleration a robot tolerates before it flips (trainer link ONLY, no
tracking).

A torque-happy rammer (e.g. Mr Stabs) backflips when you slam the throttle from rest: it is the
acceleration (how fast the command rises), not the top speed, that pitches it over. This ramps the
forward command 0 -> target at an escalating slew rate and stops at the rate that flips it. Feed the
result is the safe forward command slew rate, so scripted excitation and the deployed
controller slew-limit every motion to stay upright, instead of blindly capping speed.

No camera, no MCAP, no AprilTag. The measurement is operator-in-the-loop: you watch the robot and
answer whether it flipped. Ctrl-C during a ramp aborts that trial and zeroes the link.

SAFETY
- Guard plates ON, competition battery, surface matched to the NHRL arena floor (plate friction is
  part of the plant, see am32_tuning.md).
- Clear, bounded lane several metres long: each trial ramps to full speed and coasts.
- Driver sticks CENTERED: in trainer mode the radio ADDS stick input to the command.
- The link is zeroed and disarmed on every exit (normal, Ctrl-C, exception, per-trial hard timeout).

Usage:
    source scripts/activate_python.sh
    python playground/calibration/find_flip_accel.py --robot mr_stabs_mk2
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

from auto_battlebot.calibration import drive_protocol as dp


class _TrialTimeoutError(Exception):
    """Raised by the SIGALRM handler to break a trial that overran its time budget."""


def run_ramp(
    link: dp.TrainerLink, rate_hz: float, target: float, accel: float, hold_s: float
) -> bool:
    """Ramp forward command 0 -> target at `accel` (command/s), hold `hold_s`, then cut to zero.

    Returns True if the trial completed, False if interrupted (Ctrl-C or the hard-timeout backstop).
    The link is disarmed on the way out either way.
    """
    dt = 1.0 / rate_hz
    ramp_time = target / accel
    try:
        t0 = time.monotonic()
        while True:
            elapsed = time.monotonic() - t0
            if elapsed >= ramp_time:
                break
            link.send(min(target, accel * elapsed), 0.0)
            time.sleep(dt)
        hold_start = time.monotonic()
        while time.monotonic() - hold_start < hold_s:
            link.send(target, 0.0)
            time.sleep(dt)
        return True
    except (KeyboardInterrupt, _TrialTimeoutError):
        return False
    finally:
        link.disarm()


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--transmitter-port", default=None, help="OpenTX serial device (auto-detected if omitted)"
    )
    parser.add_argument(
        "--v-max",
        type=float,
        default=None,
        help="top speed in m/s, to report the limit in m/s^2 as well as command units",
    )
    parser.add_argument(
        "--rate", type=float, default=50.0, help="command send rate in Hz (default 50)"
    )
    parser.add_argument(
        "--target",
        type=float,
        default=1.0,
        help="peak forward command each ramp reaches (default 1.0)",
    )
    parser.add_argument(
        "--start-accel",
        type=float,
        default=1.5,
        help="gentlest ramp rate in command/s (default 0.75 = reach full in ~1.3 s)",
    )
    parser.add_argument(
        "--factor",
        type=float,
        default=1.2,
        help="multiply the ramp rate by this each trial (default 1.4)",
    )
    parser.add_argument(
        "--max-accel",
        type=float,
        default=10.0,
        help="stop escalating past this ramp rate in command/s (default 10)",
    )
    parser.add_argument(
        "--hold", type=float, default=0.3, help="seconds to hold at the peak command after the ramp"
    )
    parser.add_argument(
        "--no-reverse-angular",
        action="store_true",
        help="disable the reverse_angular convention (default matches main.toml)",
    )
    args = parser.parse_args()

    if args.start_accel <= 0 or args.factor <= 1.0 or args.target <= 0:
        raise SystemExit("--start-accel and --target must be > 0 and --factor must be > 1.")
    if not sys.stdin.isatty():
        raise SystemExit("This tool is interactive; run it from a terminal.")

    v_max = args.v_max

    def in_ms2(accel_cmd: float) -> str:
        # d(speed)/dt = d(command)/dt * v_max, since speed ~= command * v_max (deadzone/lag aside).
        return f"  (~{accel_cmd * v_max:.1f} m/s^2)" if v_max is not None else ""

    tx_port = args.transmitter_port or dp.find_transmitter_port()
    if tx_port is None:
        raise SystemExit("No OpenTX transmitter found; pass --transmitter-port.")

    print("Find max forward acceleration before flip (trainer link only, NO tracking).")
    print("SAFETY: guard plates ON, clear multi-metre lane, driver sticks CENTERED.")
    print(f"transmitter: {tx_port}")
    if input("Type 'go' to ARM and begin: ").strip() != "go":
        raise SystemExit("Aborted (not armed).")

    signal.signal(signal.SIGALRM, lambda *_: (_ for _ in ()).throw(_TrialTimeoutError()))
    link = dp.TrainerLink(
        tx_port, dp.MixConfig(reverse_angular=not args.no_reverse_angular), read_back=False
    )

    last_safe: float | None = None
    flipped_at: float | None = None
    accel = args.start_accel
    try:
        while accel <= args.max_accel + 1e-9:
            ramp_time = args.target / accel
            print(
                f"\nTrial: ramp to {args.target:.2f} at {accel:.2f}/s{in_ms2(accel)}, reached in "
                f"{ramp_time:.2f}s."
            )
            resp = (
                input(
                    "Reset robot upright at the lane start, clear space, Enter to run ('q' to "
                    "stop): "
                )
                .strip()
                .lower()
            )
            if resp == "q":
                break
            signal.alarm(int(ramp_time + args.hold) + 3)  # backstop if a trial wedges
            completed = run_ramp(link, args.rate, args.target, accel, args.hold)
            signal.alarm(0)
            if not completed:
                print("  (trial interrupted; link zeroed)")
            ans = input("Did it FLIP or lift the front wheels? [y/N/q]: ").strip().lower()
            if ans == "q":
                break
            if ans == "y":
                flipped_at = accel
                break
            last_safe = accel
            accel *= args.factor
        else:
            print(f"\nNo flip up to {args.max_accel:.2f}/s; robot tolerated the full escalation.")
    finally:
        link.close()  # zeroes the channels and disarms

    print("\n=== result ===")
    if flipped_at is not None:
        print(f"flipped at   {flipped_at:.2f} command/s{in_ms2(flipped_at)}")
    if last_safe is not None:
        print(f"max safe     {last_safe:.2f} command/s{in_ms2(last_safe)}")
        print(f"Safe forward command slew: {last_safe:.2f} /s (leave headroom).")
    else:
        print("No safe ramp rate recorded (flipped on the gentlest trial? lower --start-accel).")


if __name__ == "__main__":
    main()
