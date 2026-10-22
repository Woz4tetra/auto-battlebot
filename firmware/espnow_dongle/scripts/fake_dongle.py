#!/usr/bin/env python3
"""Simulated ESP-NOW dongle on a pty, for running espnow_diag.py without hardware.

Speaks the dongle's serial side: the boot banner, MSG_LINK_STATS every 100 ms, and robot
telemetry at 100 Hz, echoing the seq and jetson_tx_us of the last MSG_COMMAND it reads. Flags
inject the faults the diagnostics page should catch.

    venv/bin/python firmware/espnow_dongle/scripts/fake_dongle.py /tmp/fake_dongle &
    venv/bin/python firmware/espnow_dongle/scripts/espnow_diag.py --port /tmp/fake_dongle
"""

from __future__ import annotations

import argparse
import math
import os
import pty
import random
import time
from pathlib import Path

from espnow_diag import (
    COMMAND,
    LINK_STATS,
    MSG_COMMAND,
    MSG_LINK_STATS,
    MSG_PING,
    MSG_TELEMETRY,
    TELEMETRY,
    TELEMETRY_FIELDS,
    TELEMETRY_PERIOD_MS,
    UINT32_MAX,
    FrameParser,
    encode_frame,
)


def telemetry(
    t_ms: int, seq: int, args: argparse.Namespace, cmd: tuple[int, int, int, float, float]
) -> bytes:
    cmd_seq, cmd_tx_us, cmd_age_ms, cmd_a, cmd_b = cmd
    # Substitution as the firmware does it: SA on and a fresh command drive the robot.
    driving = args.autonomy and cmd_age_ms < 60
    phase = t_ms / 1000
    fields: dict[str, float | int] = {name: 0 for name in TELEMETRY_FIELDS}
    fields.update(
        timestamp_ms=t_ms & UINT32_MAX,
        loop_us=1800 + random.randint(-50, 50),
        accel_z=9.81,
        orientation_x=(phase * 20) % 360,
        vbat=15.8 - 0.1 * math.sin(phase),
        ibat=1.2,
        yaw_rate_cmd=math.nan,
        imu_calibration=0xFF,
        radio_connected=1,
        autonomy_enabled=int(args.autonomy),
        flip_switch=1,
        espnow_active=int(driving),
        a_percent=-100.0 * cmd_a if driving else 0.0,
        b_percent=100.0 * cmd_b if driving else 0.0,
        espnow_channel=args.robot_channel,
        espnow_age_ms=cmd_age_ms,
        espnow_seq=cmd_seq,
        jetson_tx_us=cmd_tx_us,
        espnow_lq=args.lq,
        telemetry_seq=seq,
    )
    return encode_frame(
        MSG_TELEMETRY,
        TELEMETRY.pack(*(float(v) if isinstance(v, float) else int(v) for v in fields.values())),
    )


class FakeDongle:
    """Dongle and robot state; step() returns the serial bytes due at a given time."""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.reader = FrameParser()
        self.start = time.monotonic()
        self.sent = self.acked = 0
        self.tlm_seq = 0
        self.cmd_seq = self.cmd_tx_us = 0
        self.cmd_a = self.cmd_b = 0.0
        self.cmd_s: float | None = None
        self.next_stats = self.next_tlm = self.start
        self.next_foreign = self.start + args.foreign_every
        self.next_crc = self.start + args.crc_every
        self.banner_sent = False
        self.robot_known = False

    def feed(self, data: bytes) -> None:
        frames, _ = self.reader.feed(data)
        for msg_type, payload in frames:
            if msg_type == MSG_COMMAND and len(payload) == COMMAND.size:
                self.cmd_seq, self.cmd_tx_us, self.cmd_a, self.cmd_b = COMMAND.unpack(payload)
                self.cmd_s = time.monotonic()

    def step(self, now: float) -> bytes:
        return self.banner(now) + self.link_stats(now) + self.robot(now) + self.faults(now)

    def banner(self, now: float) -> bytes:
        if self.banner_sent or now - self.start < self.args.banner_delay:
            return b""
        self.banner_sent = True
        return f"\nespnow_dongle mac={self.args.mac} channel={self.args.channel}\n".encode()

    def link_stats(self, now: float) -> bytes:
        if now < self.next_stats:
            return b""
        self.next_stats += 0.1
        # Commands repeat at 250 Hz while the host commands; pings fill 100 Hz otherwise.
        # Broadcast frames report success with no ACK behind them.
        commanding = self.cmd_s is not None and now - self.cmd_s < 0.1
        count = 25 if commanding else 10
        self.sent += count
        if self.robot_known:
            self.acked += sum(random.random() < self.args.ack for _ in range(count))
        else:
            self.acked += count
        failed = self.sent - self.acked
        payload = LINK_STATS.pack(
            self.sent, self.acked, failed, 850, self.args.channel, int(self.robot_known)
        )
        return encode_frame(MSG_LINK_STATS, payload)

    def robot(self, now: float) -> bytes:
        if self.args.no_robot or now < self.next_tlm:
            return b""
        self.next_tlm += TELEMETRY_PERIOD_MS / 1000
        self.tlm_seq += 1
        self.robot_known = True
        if random.random() < self.args.drop:
            return b""
        age = UINT32_MAX if self.cmd_s is None else int((now - self.cmd_s) * 1000)
        t_ms = int((now - self.start) * 1000) + 5000
        return telemetry(
            t_ms,
            self.tlm_seq,
            self.args,
            (self.cmd_seq, self.cmd_tx_us, age, self.cmd_a, self.cmd_b),
        )

    def faults(self, now: float) -> bytes:
        out = b""
        if self.args.foreign_every and now >= self.next_foreign:
            self.next_foreign += self.args.foreign_every
            out += encode_frame(MSG_PING)
        if self.args.crc_every and now >= self.next_crc:
            self.next_crc += self.args.crc_every
            bad = bytearray(encode_frame(MSG_PING))
            bad[-1] ^= 0xFF
            out += bytes(bad)
        return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("link", type=Path, help="symlink to create for the pty, the --port to read")
    parser.add_argument("--mac", default="d8:3b:da:8e:ed:2c", help="MAC in the boot banner")
    parser.add_argument("--channel", type=int, default=1, help="dongle channel")
    parser.add_argument("--robot-channel", type=int, default=1, help="0 reports sweeping")
    parser.add_argument("--no-robot", action="store_true", help="broadcast forever, no telemetry")
    parser.add_argument("--ack", type=float, default=0.99, help="unicast ACK probability")
    parser.add_argument("--drop", type=float, default=0.02, help="telemetry frame loss probability")
    parser.add_argument("--lq", type=int, default=97, help="robot espnow_lq percent")
    parser.add_argument("--autonomy", action="store_true", help="SA on: robot accepts commands")
    parser.add_argument(
        "--foreign-every", type=float, default=0.0, help="seconds between foreign PINGs, 0 for none"
    )
    parser.add_argument("--crc-every", type=float, default=0.0, help="seconds between bad frames")
    parser.add_argument(
        "--banner-delay", type=float, default=3.0, help="seconds after start before the banner"
    )
    args = parser.parse_args()

    master, slave = pty.openpty()
    os.set_blocking(master, False)
    if args.link.is_symlink() or args.link.exists():
        args.link.unlink()
    args.link.symlink_to(os.ttyname(slave))
    print(f"fake dongle on {args.link} -> {os.ttyname(slave)}")

    fake = FakeDongle(args)
    try:
        while True:
            try:
                fake.feed(os.read(master, 4096))
            except (BlockingIOError, OSError):
                pass
            out = fake.step(time.monotonic())
            if out:
                try:
                    os.write(master, out)
                except (BlockingIOError, OSError):
                    pass  # No reader: drop, as the dongle does.
            time.sleep(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        args.link.unlink(missing_ok=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
