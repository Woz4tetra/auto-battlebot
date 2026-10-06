#!/usr/bin/env python3
"""Log Mr Stabs over the diagnostics stream and check two causes of twitching while driving.

1. Spurious upside-down flips. With the flip switch DOWN, the firmware takes is_upside_down from
   the BNO055 gravity vector and negates the throttle when it changes. It flips upside down below
   accel_z = -1 m/s^2 and back above +1 m/s^2. A flip that reverts within --flip-window while the
   switch did not move is a false detection: the drive reversed under the driver.
2. DShot 3D reversals. A wheel command crossing from one side of the ESC deadzone to the other is
   a direction reversal, which the ESC handles with a stall. A reversal is blamed on heading hold
   when the driver's own stick mix for that wheel kept its sign.

Join the MR-STABS WiFi, arm, and drive. Ctrl-C stops the capture early. --csv analyzes a file
from an earlier capture or from the dashboard's Download CSV button instead.
"""

import argparse
import csv
import time
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from statistics import median

COLUMNS = [
    "timestamp_ms",
    "radio_connected",
    "armed",
    "a_percent",
    "b_percent",
    "button_state",
    "flip_switch",
    "left_cmd",
    "right_cmd",
    "accel_x",
    "accel_y",
    "accel_z",
    "is_upside_down",
    "loop_us",
    "wifi_clients",
    "orientation_x",
    "orientation_y",
    "orientation_z",
    "pid_setpoint",
    "pid_output",
    "vbat",
    "ibat",
]

FLIP_SWITCH_DOWN = 0  # crsf_bridge::DOWN, the auto upside-down and heading-hold position
UPSIDE_DOWN_BELOW = -1.0  # updown_sensor.h thresholds on the gravity z the diag reports
RIGHT_SIDE_UP_ABOVE = 1.0
TURN_THRESHOLD = 1.0  # |b_percent| above this is a manual turn, below it heading hold drives

Row = dict[str, float]


def http_get(host: str, path: str, timeout: float = 2.0) -> str:
    with urllib.request.urlopen(f"http://{host}{path}", timeout=timeout) as response:
        return str(response.read().decode())


def fetch_deadzones(host: str) -> tuple[float, float]:
    return (
        float(http_get(host, "/tune/left_esc_dz")),
        float(http_get(host, "/tune/right_esc_dz")),
    )


def capture(host: str, seconds: float, out: Path) -> None:
    """Stream /events in recording mode, which sends a row every control loop."""
    out.parent.mkdir(parents=True, exist_ok=True)
    http_get(host, "/record/start")
    rows = 0
    end = time.monotonic() + seconds
    print(f"capturing to {out} for {seconds:.0f} s, Ctrl-C to stop early")
    try:
        with (
            urllib.request.urlopen(f"http://{host}/events", timeout=5.0) as stream,
            out.open("w", newline="") as f,
        ):
            f.write(",".join(COLUMNS) + "\n")
            for raw in stream:
                if time.monotonic() > end:
                    break
                line = raw.decode(errors="replace").strip()
                # The connect greeting is also a data line; only full diag rows have every column.
                if not line.startswith("data:"):
                    continue
                data = line[len("data:") :].strip()
                if data.count(",") != len(COLUMNS) - 1:
                    continue
                f.write(data + "\n")
                rows += 1
                if rows % 500 == 0:
                    print(f"\r{rows} rows", end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            http_get(host, "/record/stop")
        except OSError as error:
            print(f"\ncould not stop recording mode: {error}")
    print(f"\r{rows} rows written to {out}")


def load(path: Path) -> list[Row]:
    rows: list[Row] = []
    with path.open() as f:
        for record in csv.DictReader(f):
            try:
                rows.append({key: float(record[key]) for key in COLUMNS})
            except (KeyError, TypeError, ValueError):
                continue
    return rows


def seconds(row: Row, start: Row) -> float:
    return (row["timestamp_ms"] - start["timestamp_ms"]) / 1000.0


def report_stream(rows: list[Row]) -> None:
    """The stream drops rows when WiFi falls behind, which can hide short events."""
    gaps = [b["timestamp_ms"] - a["timestamp_ms"] for a, b in zip(rows, rows[1:])]
    duration = seconds(rows[-1], rows[0])
    loop_us = [row["loop_us"] for row in rows]
    gravity_updates = sum(
        1
        for a, b in zip(rows, rows[1:])
        if (a["accel_x"], a["accel_y"], a["accel_z"]) != (b["accel_x"], b["accel_y"], b["accel_z"])
    )
    rate = len(rows) / max(duration, 1e-3)
    print(f"stream: {len(rows)} rows over {duration:.1f} s ({rate:.0f} Hz)")
    print(f"  row gap median {median(gaps):.0f} ms, max {max(gaps):.0f} ms")
    print(f"  loop_us median {median(loop_us):.0f}, max {max(loop_us):.0f}")
    print(
        f"  gravity vector changed {gravity_updates / max(duration, 1e-3):.0f} times/s "
        "(BNO055 sample rate lower bound; expect near 100 when moving)"
    )


def check_upside_down(rows: list[Row], flip_window: float) -> bool:
    """Returns True when a spurious flip was seen."""
    print("\n[3] upside-down auto-detection")
    auto = [row for row in rows if row["flip_switch"] == FLIP_SWITCH_DOWN]
    if not auto:
        print("  not exercised: flip switch was never DOWN while armed")
        return False

    # Distance from the threshold that would flip the current state, in m/s^2.
    margins = [
        row["accel_z"] - UPSIDE_DOWN_BELOW
        if row["is_upside_down"] == 0
        else RIGHT_SIDE_UP_ABOVE - row["accel_z"]
        for row in auto
    ]
    close = sum(1 for margin in margins if margin < 3.0)
    print(
        f"  {len(auto)} rows with the switch DOWN; smallest margin to the flip threshold "
        f"{min(margins):.1f} m/s^2, {100 * close / len(auto):.1f}% of rows within 3 m/s^2"
    )

    flips: list[tuple[Row, Row]] = []
    for a, b in zip(rows, rows[1:]):
        same_switch = a["flip_switch"] == b["flip_switch"] == FLIP_SWITCH_DOWN
        if same_switch and a["is_upside_down"] != b["is_upside_down"]:
            flips.append((a, b))

    spurious = 0
    for i, (before, after) in enumerate(flips):
        reverted_at = flips[i + 1][1] if i + 1 < len(flips) else None
        held = (reverted_at["timestamp_ms"] - after["timestamp_ms"]) / 1000 if reverted_at else None
        is_spurious = held is not None and held < flip_window
        spurious += is_spurious
        state = "upside down" if after["is_upside_down"] else "right side up"
        held_text = f"held {held:.2f} s" if held is not None else "held to end of log"
        print(
            f"  t={seconds(after, rows[0]):7.2f} s -> {state}, accel_z {before['accel_z']:+.1f} -> "
            f"{after['accel_z']:+.1f}, roll {after['orientation_y']:+.0f}, "
            f"pitch {after['orientation_z']:+.0f}, throttle {after['a_percent']:+.0f}%, "
            f"{held_text}{'  SPURIOUS' if is_spurious else ''}"
        )
    if spurious:
        print(f"  PROBLEM: {spurious} of {len(flips)} flips reverted within {flip_window} s")
    elif flips:
        print(f"  ok: {len(flips)} flips, all held longer than {flip_window} s")
    else:
        print("  ok: no flips with the switch DOWN")
    return spurious > 0


@dataclass
class Reversal:
    t: float
    wheel: str
    from_cmd: float
    to_cmd: float
    driver_kept_sign: bool
    pid_output: float
    throttle: float


def wheel_sign(command: float, deadzone: float) -> int:
    if abs(command) < deadzone:
        return 0
    return 1 if command > 0 else -1


def driver_mix(row: Row, wheel: str) -> float:
    """The wheel command from the sticks alone, as mix_motor_outputs builds it without the PID."""
    turn = row["b_percent"]
    if row["flip_switch"] == FLIP_SWITCH_DOWN and abs(turn) <= TURN_THRESHOLD:
        turn = 0.0
    sign = 1.0 if wheel == "left" else -1.0
    return -row["a_percent"] + sign * turn


def find_reversals(rows: list[Row], wheel: str, deadzone: float) -> tuple[list[Reversal], int]:
    """Reversals of one wheel, and how often it entered or left the deadzone while driving."""
    key = f"{wheel}_cmd"
    reversals: list[Reversal] = []
    stop_toggles = 0
    last_nonzero: Row | None = None
    previous = 0
    for row in rows:
        sign = wheel_sign(row[key], deadzone)
        if (sign == 0) != (previous == 0) and abs(row["a_percent"]) >= 5.0:
            stop_toggles += 1
        previous = sign
        if sign == 0:
            continue
        if last_nonzero is not None and wheel_sign(last_nonzero[key], deadzone) != sign:
            driver_before = wheel_sign(driver_mix(last_nonzero, wheel), deadzone)
            driver_after = wheel_sign(driver_mix(row, wheel), deadzone)
            reversals.append(
                Reversal(
                    t=seconds(row, rows[0]),
                    wheel=wheel,
                    from_cmd=last_nonzero[key],
                    to_cmd=row[key],
                    driver_kept_sign=driver_before == driver_after != 0,
                    pid_output=row["pid_output"],
                    throttle=row["a_percent"],
                )
            )
        last_nonzero = row
    return reversals, stop_toggles


def check_reversals(rows: list[Row], deadzones: tuple[float, float], chatter_window: float) -> bool:
    """Returns True when heading hold reversed a wheel or a wheel chattered."""
    print("\n[4] DShot 3D direction reversals")
    print(f"  deadzone left {deadzones[0]:.1f}%, right {deadzones[1]:.1f}%")
    left, left_toggles = find_reversals(rows, "left", deadzones[0])
    right, right_toggles = find_reversals(rows, "right", deadzones[1])
    reversals = left + right
    driving = [row for row in rows if abs(row["a_percent"]) >= 5.0]
    near_zero_rows = sum(
        1 for row in driving if min(abs(row["left_cmd"]), abs(row["right_cmd"])) < 10.0
    )

    reversals.sort(key=lambda r: r.t)
    pid_caused = [r for r in reversals if r.driver_kept_sign]
    chatter = sum(
        1
        for wheel_reversals in (left, right)
        for a, b in zip(wheel_reversals, wheel_reversals[1:])
        if b.t - a.t < chatter_window
    )

    for r in reversals:
        cause = "heading hold" if r.driver_kept_sign else "driver"
        print(
            f"  t={r.t:7.2f} s {r.wheel:5s} {r.from_cmd:+6.1f}% -> {r.to_cmd:+6.1f}%  "
            f"throttle {r.throttle:+4.0f}%  pid {r.pid_output:+6.2f}  ({cause})"
        )
    print(
        f"  {len(reversals)} reversals: {len(pid_caused)} by heading hold, "
        f"{len(reversals) - len(pid_caused)} by the sticks; "
        f"{chatter} came back within {chatter_window} s"
    )
    print(
        f"  deadzone stop/start toggles while driving: left {left_toggles}, right {right_toggles}"
    )
    if driving:
        print(
            f"  while driving, a wheel was under 10% for "
            f"{100 * near_zero_rows / len(driving):.1f}% of rows"
        )
    problem = bool(pid_caused) or chatter > 0
    if problem:
        print("  PROBLEM: heading hold reversed a wheel, or a wheel reversed back and forth")
    else:
        print("  ok: no heading-hold reversals and no back-and-forth reversals")
    return problem


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--host", default="192.168.4.1")
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument(
        "--out",
        type=Path,
        default=Path.home() / "mr_stabs_logs" / time.strftime("drive_%Y%m%d_%H%M%S.csv"),
    )
    parser.add_argument("--csv", type=Path, help="analyze this file instead of capturing")
    parser.add_argument(
        "--deadzone",
        type=float,
        nargs=2,
        metavar=("LEFT", "RIGHT"),
        help="ESC deadzones in percent; read from the robot when omitted, else 1.0",
    )
    parser.add_argument("--flip-window", type=float, default=0.5)
    parser.add_argument("--chatter-window", type=float, default=0.5)
    args = parser.parse_args()

    deadzones: tuple[float, float] = tuple(args.deadzone) if args.deadzone else (1.0, 1.0)
    if args.csv is None:
        if not args.deadzone:
            deadzones = fetch_deadzones(args.host)
        capture(args.host, args.seconds, args.out)
        path = args.out
    else:
        path = args.csv

    rows = [row for row in load(path) if row["armed"] == 1 and row["radio_connected"] == 1]
    if len(rows) < 2:
        raise SystemExit(f"{path}: fewer than two armed rows, nothing to analyze")
    report_stream(rows)
    flip_problem = check_upside_down(rows, args.flip_window)
    reversal_problem = check_reversals(rows, deadzones, args.chatter_window)
    print(
        f"\nsummary: [3] {'PROBLEM' if flip_problem else 'ok'}, "
        f"[4] {'PROBLEM' if reversal_problem else 'ok'}"
    )


if __name__ == "__main__":
    main()
