#!/usr/bin/env python3
"""ESP-NOW link diagnostics: reads the dongle's serial stream and serves a live page.

Decodes every serial frame the dongle forwards (link stats, robot telemetry, OTA status, and any
frame some other host put on the channel), tracks ACK rate, telemetry drops, arrival gaps, and
command round trip, and runs a checklist that names the likely fault. Every frame lands in a
2-minute ring that can be saved after the fact, and CAPTURE writes the whole stream to JSONL.

The dongle allows one reader. Stop the app first, or pass --manage-service:

    venv/bin/python firmware/espnow_dongle/scripts/espnow_diag.py --manage-service
    # then open http://<host>:8090

SEND COMMANDS streams commands at --command-hz so the command path, sequence numbers, and round
trip can be measured. They start neutral (channel_a = channel_b = 0); the page's sliders set them,
and the space bar returns both to neutral. The page refreshes the values every 100 ms; after
COMMAND_DEADMAN_S without a refresh (tab closed, laptop asleep, network gone) they fall back to
neutral. The robot only drives from them while armed with SA on, so the handset keeps authority.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import struct
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from flask import Flask, Response, abort, jsonify, request, send_from_directory

import serial

SCRIPT_DIR = Path(__file__).resolve().parent
WEB_DIR = SCRIPT_DIR / "espnow_diag_web"
FIRMWARE_DIR = SCRIPT_DIR.parent.parent
# The robot's diagnostics page holds the shared stylesheet and the embedded fonts.
ROBOT_WEB_DIR = FIRMWARE_DIR / "mr_stabs_mk2" / "web"
SERVICE = "auto-battlebot"

# ---- wire format: firmware/shared/espnow_protocol/src/espnow_protocol.h -------------------------

SERIAL_MAGIC = b"\x7b\xb0"
SERIAL_OVERHEAD = 6
BAUD = 921600

MSG_COMMAND = 0x01
MSG_TELEMETRY = 0x02
MSG_TUNABLE_SET = 0x03
MSG_LINK_STATS = 0x04
MSG_PING = 0x05
MSG_OTA_BEGIN = 0x10
MSG_OTA_CHUNK = 0x11
MSG_OTA_END = 0x12
MSG_OTA_ABORT = 0x13
MSG_OTA_STATUS = 0x14

# (name, who sends it, expected payload size or None for variable)
MESSAGE_TYPES: dict[int, tuple[str, str, int | None]] = {
    MSG_COMMAND: ("COMMAND", "host", 16),
    MSG_TELEMETRY: ("TELEMETRY", "robot", 118),
    MSG_TUNABLE_SET: ("TUNABLE_SET", "host", 5),
    MSG_LINK_STATS: ("LINK_STATS", "dongle", 18),
    MSG_PING: ("PING", "dongle", 0),
    MSG_OTA_BEGIN: ("OTA_BEGIN", "host", 20),
    MSG_OTA_CHUNK: ("OTA_CHUNK", "host", None),
    MSG_OTA_END: ("OTA_END", "host", 0),
    MSG_OTA_ABORT: ("OTA_ABORT", "host", 0),
    MSG_OTA_STATUS: ("OTA_STATUS", "robot", 22),
}
# The dongle forwards every air frame carrying the session magic, whoever sent it. Only these
# two come from the robot; anything else read back from the air is another dongle or host on
# this channel.
ROBOT_TYPES = {MSG_TELEMETRY, MSG_OTA_STATUS}

COMMAND = struct.Struct("<IIff")
LINK_STATS = struct.Struct("<IIIIBB")
OTA_STATUS = struct.Struct("<BIB16s")
TELEMETRY = struct.Struct("<II21f9BIIIBI")
TELEMETRY_FIELDS = (
    "timestamp_ms loop_us a_percent b_percent left_cmd right_cmd accel_x accel_y accel_z "
    "orientation_x orientation_y orientation_z pid_setpoint pid_output vbat ibat yaw_rate "
    "yaw_rate_cmd linear_accel_x linear_accel_y linear_accel_z gyro_x gyro_y imu_calibration "
    "radio_connected armed autonomy_enabled button_state flip_switch is_upside_down "
    "espnow_active espnow_channel espnow_age_ms espnow_seq jetson_tx_us espnow_lq telemetry_seq"
).split()
assert COMMAND.size == 16 and LINK_STATS.size == 18 and OTA_STATUS.size == 22
assert TELEMETRY.size == 118 and len(TELEMETRY_FIELDS) == 37

OTA_STATES = {0: "IDLE", 1: "RECEIVING", 2: "DONE", 3: "VALIDATED", 4: "ERROR"}
OTA_ERRORS = {
    0: "OK",
    1: "ARMED",
    2: "BEGIN (image does not fit slot)",
    3: "WRITE",
    4: "VERIFY (MD5 or image check)",
    5: "ABORTED",
    6: "NOT_ACTIVE",
}

UINT32_MAX = 0xFFFFFFFF
TELEMETRY_PERIOD_MS = 10  # mr_stabs_mk2 espnow_link.h
LINK_STATS_PERIOD_MS = 100  # espnow_dongle main.cpp

RING_SECONDS = 120.0
SERIES_SECONDS = 60.0
SERIES_PERIOD_S = 0.2
GAP_EVENT_MS = 100.0
DONGLE_SILENT_S = 0.5
REOPEN_PERIOD_S = 1.0
COMMAND_DEADMAN_S = 0.3

BANNER = re.compile(r"espnow_dongle mac=([0-9A-Fa-f:]{17}) channel=(\d+)")


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def encode_frame(msg_type: int, payload: bytes = b"") -> bytes:
    body = bytes([msg_type, len(payload)]) + payload
    return SERIAL_MAGIC + body + struct.pack("<H", crc16_ccitt(body))


def monotonic_us() -> int:
    """Same clock shape as espnow_monotonic_us() in the C++ transmitter: u32 microseconds."""
    return (time.monotonic_ns() // 1000) & UINT32_MAX


def finite(value: float) -> float | None:
    """JSON has no NaN; the firmware uses it for 'sensor absent' and 'loop off'."""
    return value if math.isfinite(value) else None


def decode(msg_type: int, payload: bytes) -> dict[str, Any] | None:
    """Decoded fields, or None when the type is unknown or the size is wrong."""
    expected = MESSAGE_TYPES.get(msg_type, (None, None, -1))[2]
    if expected is not None and len(payload) != expected:
        return None
    if msg_type == MSG_TELEMETRY:
        values = TELEMETRY.unpack(payload)
        return {
            k: finite(v) if isinstance(v, float) else v for k, v in zip(TELEMETRY_FIELDS, values)
        }
    if msg_type == MSG_LINK_STATS:
        sent, acked, failed, rtt, channel, known = LINK_STATS.unpack(payload)
        return {
            "sent": sent,
            "acked": acked,
            "failed": failed,
            "last_rtt_us": rtt,
            "channel": channel,
            "robot_known": known,
        }
    if msg_type == MSG_OTA_STATUS:
        state, next_offset, error, md5 = OTA_STATUS.unpack(payload)
        return {
            "state": state,
            "state_name": OTA_STATES.get(state, f"?{state}"),
            "next_offset": next_offset,
            "error": error,
            "error_name": OTA_ERRORS.get(error, f"?{error}"),
            "running_md5": md5.hex(),
        }
    if msg_type == MSG_COMMAND:
        seq, tx_us, a, b = COMMAND.unpack(payload)
        return {"seq": seq, "jetson_tx_us": tx_us, "channel_a": finite(a), "channel_b": finite(b)}
    if msg_type == MSG_TUNABLE_SET:
        tunable, value = struct.unpack("<Bf", payload)
        return {"id": tunable, "value": finite(value)}
    if msg_type == MSG_OTA_BEGIN:
        size, md5 = struct.unpack("<I16s", payload)
        return {"size": size, "md5": md5.hex()}
    if msg_type == MSG_OTA_CHUNK and len(payload) >= 4:
        return {"offset": struct.unpack_from("<I", payload)[0], "data_len": len(payload) - 4}
    if msg_type in MESSAGE_TYPES:
        return {}
    return None


def check(name: str, level: str, value: str, hint: str = "") -> dict[str, str]:
    """One checklist row. The hint only shows when the row is not ok."""
    return {"name": name, "level": level, "value": value, "hint": "" if level == "ok" else hint}


def grade(value: float, ok_at: float, warn_at: float) -> str:
    return "ok" if value >= ok_at else "warn" if value >= warn_at else "error"


def percentile(sorted_values: list[float], q: float) -> float | None:
    if not sorted_values:
        return None
    index = min(len(sorted_values) - 1, int(round(q * (len(sorted_values) - 1))))
    return sorted_values[index]


def expand(record: dict[str, Any]) -> dict[str, Any]:
    """Frame records hold raw hex only, so the 2-minute ring stays small; decode on write."""
    if record.get("kind") != "frame":
        return record
    payload = bytes.fromhex(record["hex"])
    return {
        **record,
        "name": MESSAGE_TYPES.get(record["type"], (f"0x{record['type']:02X}",))[0],
        "len": len(payload),
        "decoded": decode(record["type"], payload),
    }


class FrameParser:
    """Python twin of espnow_protocol::SerialFrameParser, plus capture of the bytes between
    frames, which is where the dongle's plain-text boot banner lands."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.text = bytearray()
        self.crc_errors = 0
        self.junk_bytes = 0

    def feed(self, data: bytes) -> tuple[list[tuple[int, bytes]], list[str]]:
        self.buffer += data
        frames: list[tuple[int, bytes]] = []
        lines: list[str] = []
        while True:
            start = self.buffer.find(SERIAL_MAGIC)
            if start < 0:
                # Keep a trailing first magic byte; it may be the start of the next frame.
                keep = 1 if self.buffer.endswith(SERIAL_MAGIC[:1]) else 0
                self._skip(len(self.buffer) - keep, lines)
                break
            self._skip(start, lines)
            if len(self.buffer) < 4:
                break
            length = self.buffer[3]
            size = SERIAL_OVERHEAD + length
            if len(self.buffer) < size:
                break
            body = bytes(self.buffer[2 : 4 + length])
            (crc,) = struct.unpack_from("<H", self.buffer, 4 + length)
            if crc != crc16_ccitt(body):
                self.crc_errors += 1
                self.junk_bytes += 1
                del self.buffer[:1]
                continue
            frames.append((body[0], body[2:]))
            del self.buffer[:size]
        return frames, lines

    def _skip(self, count: int, lines: list[str]) -> None:
        if count <= 0:
            return
        skipped = bytes(self.buffer[:count])
        del self.buffer[:count]
        self.junk_bytes += count
        for byte in skipped:
            if byte == 0x0A:
                line = self.text.decode("ascii", "replace").strip()
                if line:
                    lines.append(line)
                self.text.clear()
            elif 0x20 <= byte < 0x7F or byte == 0x09:
                self.text.append(byte)
                if len(self.text) > 512:
                    self.text.clear()


@dataclass
class TypeStats:
    count: int = 0
    bytes: int = 0
    size_errors: int = 0
    last_s: float = 0.0
    arrivals: deque[float] = field(default_factory=lambda: deque(maxlen=4000))


class Diagnostics:
    """All link state. The serial thread writes it, Flask reads it, both under one lock."""

    def __init__(self, port_path: str, capture_dir: Path, command_hz: float) -> None:
        self.lock = threading.Lock()
        self.port_path = port_path
        self.capture_dir = capture_dir
        self.command_period_s = 1.0 / command_hz
        self.started_s = time.monotonic()
        self.reset_counters()

        # Port state survives a counter reset.
        self.banner: dict[str, Any] | None = None
        self.port: serial.Serial | None = None
        self.port_state = "CLOSED"
        self.port_error = ""
        self.opened_s: float | None = None
        self.opens = 0
        self.reset_requested = False

        self.events: deque[dict[str, Any]] = deque(maxlen=500)
        self.text_lines: deque[dict[str, Any]] = deque(maxlen=200)
        self.ring: deque[dict[str, Any]] = deque()
        self.capture_file: Any = None
        self.capture_path: Path | None = None
        self.capture_records = 0

    def reset_counters(self) -> None:
        self.parser = FrameParser()
        self.rx_bytes = 0
        self.rx_log: deque[tuple[float, int]] = deque()
        self.types: dict[int, TypeStats] = {}
        self.foreign_frames = 0

        self.link: dict[str, Any] | None = None
        self.link_s = 0.0
        self.link_history: deque[tuple[float, int, int, int]] = deque()
        self.link_session: tuple[int, int] | None = None  # (sent, acked) when robot_known went 1
        self.link_gap_max_ms = 0.0
        self.dongle_reboots = 0

        self.telemetry: dict[str, Any] | None = None
        self.telemetry_s = 0.0
        self.tlm_received = 0
        self.tlm_dropped = 0
        self.tlm_window: deque[tuple[float, int]] = deque()  # (time, frames lost before it)
        self.robot_reboots = 0
        self.host_gaps: deque[tuple[float, float]] = deque()  # (time, ms since previous)
        self.robot_gaps: deque[tuple[float, float]] = deque()  # robot clock ms between frames
        self.big_gaps = 0
        self.robot_clock_offset_ms: float | None = None
        self.robot_clock_drift_ms = 0.0

        self.ota: dict[str, Any] | None = None
        self.ota_s = 0.0

        self.sending = False
        self.cmd_a = 0.0  # channel_a, ch0: linear under DifferentialDriveProcessor
        self.cmd_b = 0.0  # channel_b, ch1: angular
        self.cmd_set_s = 0.0
        self.cmd_seq = 0
        self.cmd_sent = 0
        self.cmd_write_errors = 0
        self.cmd_next_s = 0.0
        self.cmd_tx: dict[int, int] = {}  # seq -> tx_us, recent sends only
        self.cmd_last_robot_seq = 0
        self.cmd_seq_jumps = 0
        self.cmd_foreign_echo = 0
        self.rtt: deque[tuple[float, float]] = deque()

        self.series: dict[str, deque[float | None]] = {}
        self.series_t: deque[float] = deque()
        self.series_next_s = 0.0
        self.interval_gap_ms = 0.0
        self.interval_rtt_ms: float | None = None

    # ---- events and capture ----------------------------------------------------------------

    def event(self, level: str, text: str) -> None:
        record = {"t": time.time(), "level": level, "text": text}
        self.events.append(record)
        self.record({"kind": "event", **record})

    def record(self, record: dict[str, Any]) -> None:
        record.setdefault("t", time.time())
        self.ring.append(record)
        cutoff = record["t"] - RING_SECONDS
        while self.ring and self.ring[0]["t"] < cutoff:
            self.ring.popleft()
        if self.capture_file is not None:
            self.capture_file.write(json.dumps(expand(record)) + "\n")
            self.capture_records += 1

    def start_capture(self) -> Path:
        self.capture_dir.mkdir(parents=True, exist_ok=True)
        path = self.capture_dir / time.strftime("espnow_capture_%Y%m%d_%H%M%S.jsonl")
        self.capture_file = path.open("w")
        self.capture_path = path
        self.capture_records = 0
        self.capture_file.write(json.dumps(self.header_record()) + "\n")
        self.event("info", f"capture started: {path}")
        return path

    def stop_capture(self) -> None:
        if self.capture_file is None:
            return
        self.capture_file.close()
        self.capture_file = None
        self.event("info", f"capture stopped: {self.capture_records} records")

    def save_ring(self) -> Path:
        self.capture_dir.mkdir(parents=True, exist_ok=True)
        path = self.capture_dir / time.strftime("espnow_last2min_%Y%m%d_%H%M%S.jsonl")
        with path.open("w") as out:
            out.write(json.dumps(self.header_record()) + "\n")
            for record in self.ring:
                out.write(json.dumps(expand(record)) + "\n")
        self.event("info", f"saved last {RING_SECONDS:.0f} s: {path}")
        return path

    def header_record(self) -> dict[str, Any]:
        return {
            "kind": "header",
            "t": time.time(),
            "port": self.port_path,
            "banner": self.banner,
            "telemetry_fields": TELEMETRY_FIELDS,
        }

    # ---- frame handling --------------------------------------------------------------------

    def on_bytes(self, data: bytes, now: float) -> None:
        self.rx_bytes += len(data)
        self.rx_log.append((now, len(data)))
        frames, lines = self.parser.feed(data)
        for line in lines:
            self.on_text(line)
        for msg_type, payload in frames:
            self.on_frame(msg_type, payload, now)

    def on_text(self, line: str) -> None:
        self.text_lines.append({"t": time.time(), "text": line})
        self.record({"kind": "text", "text": line})
        match = BANNER.search(line)
        if match:
            mac = match.group(1).lower()
            self.banner = {"mac": mac, "channel": int(match.group(2)), "t": time.time()}
            self.event("info", f"dongle booted: mac={mac} channel={match.group(2)}")

    def on_frame(self, msg_type: int, payload: bytes, now: float) -> None:
        stats = self.types.setdefault(msg_type, TypeStats())
        stats.count += 1
        stats.bytes += len(payload)
        stats.last_s = now
        stats.arrivals.append(now)
        decoded = decode(msg_type, payload)
        self.record({"kind": "frame", "type": msg_type, "hex": payload.hex()})
        if decoded is None:
            stats.size_errors += 1
            if stats.size_errors in (1, 10, 100, 1000):
                self.event(
                    "error",
                    f"type 0x{msg_type:02X} with {len(payload)}-byte payload could not be decoded "
                    f"({stats.size_errors} so far): mismatched firmware builds?",
                )
            return
        if msg_type == MSG_LINK_STATS:
            self.on_link_stats(decoded, now)
        elif msg_type == MSG_TELEMETRY:
            self.on_telemetry(decoded, now)
        elif msg_type == MSG_OTA_STATUS:
            self.ota = decoded
            self.ota_s = now
        if msg_type not in ROBOT_TYPES and msg_type != MSG_LINK_STATS:
            self.foreign_frames += 1
            if self.foreign_frames in (1, 10, 100, 1000, 10000):
                name = MESSAGE_TYPES.get(msg_type, (f"0x{msg_type:02X}",))[0]
                self.event(
                    "warn",
                    f"heard {name} from the air ({self.foreign_frames} foreign frames): another "
                    "dongle or host is on this channel",
                )

    def on_link_stats(self, s: dict[str, Any], now: float) -> None:
        prev = self.link
        if prev is not None:
            gap_ms = (now - self.link_s) * 1000
            self.link_gap_max_ms = max(self.link_gap_max_ms, gap_ms)
            if gap_ms > 3 * LINK_STATS_PERIOD_MS:
                self.event("warn", f"link stats gap {gap_ms:.0f} ms (USB stall or dongle hang)")
            if s["sent"] < prev["sent"]:
                self.dongle_reboots += 1
                self.link_history.clear()
                self.link_session = None
                self.event("warn", "dongle counters went backwards: dongle rebooted")
            if s["robot_known"] != prev["robot_known"]:
                if s["robot_known"]:
                    self.event("good", "dongle heard the robot: unicast with MAC-layer ACKs")
                else:
                    self.event("warn", "dongle lost the robot: back to broadcast")
            if s["channel"] != prev["channel"]:
                self.event("warn", f"dongle channel {prev['channel']} -> {s['channel']}")
        else:
            mode = "unicast" if s["robot_known"] else "broadcasting, robot not heard yet"
            self.event("info", f"first link stats: channel {s['channel']}, {mode}")
        if s["robot_known"] and self.link_session is None:
            self.link_session = (s["sent"], s["acked"])
        if not s["robot_known"]:
            self.link_session = None
        self.link = s
        self.link_s = now
        self.link_history.append((now, s["sent"], s["acked"], s["failed"]))
        while self.link_history and self.link_history[0][0] < now - 1.05:
            self.link_history.popleft()

    def on_telemetry(self, t: dict[str, Any], now: float) -> None:
        prev = self.telemetry
        lost = 0
        if prev is not None:
            gap_ms = (now - self.telemetry_s) * 1000
            self.host_gaps.append((now, gap_ms))
            self.interval_gap_ms = max(self.interval_gap_ms, gap_ms)
            if gap_ms > GAP_EVENT_MS:
                self.big_gaps += 1
                self.event("warn", f"telemetry gap {gap_ms:.0f} ms")
            delta = (t["telemetry_seq"] - prev["telemetry_seq"]) & UINT32_MAX
            if delta == 0 or delta > 0x7FFFFFFF:
                self.robot_reboots += 1
                self.robot_clock_offset_ms = None
                self.event(
                    "warn",
                    f"telemetry_seq {prev['telemetry_seq']} -> {t['telemetry_seq']}: "
                    "robot rebooted",
                )
            else:
                lost = delta - 1
                self.robot_gaps.append(
                    (now, ((t["timestamp_ms"] - prev["timestamp_ms"]) & UINT32_MAX) / delta)
                )
            if prev["espnow_channel"] != t["espnow_channel"]:
                ch = t["espnow_channel"] or "sweeping"
                self.event("info", f"robot ESP-NOW channel -> {ch}")
            for key, on, off in (
                ("armed", "robot armed", "robot disarmed"),
                ("autonomy_enabled", "autonomy switch (SA) on", "autonomy switch (SA) off"),
                ("espnow_active", "robot driving from ESP-NOW", "robot stopped using ESP-NOW"),
                ("radio_connected", "robot RC radio connected", "robot RC radio lost"),
            ):
                if prev[key] != t[key]:
                    self.event("info", on if t[key] else off)
        else:
            self.event("good", f"first robot telemetry, telemetry_seq {t['telemetry_seq']}")
        self.tlm_received += 1
        self.tlm_dropped += lost
        self.tlm_window.append((now, lost))

        # Robot clock against host clock. The minimum offset tracks the fastest delivery, so its
        # change over time is clock drift plus any growth in the transport floor.
        offset = now * 1000 - t["timestamp_ms"]
        if self.robot_clock_offset_ms is None or offset < self.robot_clock_offset_ms:
            if self.robot_clock_offset_ms is None:
                self.robot_clock_drift_ms = 0.0
            self.robot_clock_offset_ms = offset
        self.robot_clock_drift_ms = offset - self.robot_clock_offset_ms

        self.on_command_echo(t, now)
        self.telemetry = t
        self.telemetry_s = now

    def on_command_echo(self, t: dict[str, Any], now: float) -> None:
        robot_seq = t["espnow_seq"]
        if t["espnow_age_ms"] == UINT32_MAX or robot_seq == 0:
            return
        if robot_seq == self.cmd_last_robot_seq:
            return  # Same command echoed again: its second sample also holds a 10 ms slot wait.
        if self.cmd_last_robot_seq and robot_seq > self.cmd_last_robot_seq + 1:
            self.cmd_seq_jumps += robot_seq - self.cmd_last_robot_seq - 1
        self.cmd_last_robot_seq = robot_seq
        tx_us = self.cmd_tx.get(robot_seq)
        if tx_us is None or tx_us != t["jetson_tx_us"]:
            # Not one of ours: the app ran before us, or another host commands this robot.
            self.cmd_foreign_echo += 1
            return
        rtt_ms = ((monotonic_us() - tx_us) & UINT32_MAX) / 1000
        self.rtt.append((now, rtt_ms))
        if self.interval_rtt_ms is None or rtt_ms < self.interval_rtt_ms:
            self.interval_rtt_ms = rtt_ms

    # ---- periodic work ---------------------------------------------------------------------

    def next_command(self, now: float) -> bytes | None:
        if not self.sending or now < self.cmd_next_s:
            return None
        self.cmd_next_s = max(self.cmd_next_s + self.command_period_s, now)
        if (self.cmd_a or self.cmd_b) and now - self.cmd_set_s > COMMAND_DEADMAN_S:
            self.cmd_a = self.cmd_b = 0.0
            self.event("warn", "slider values went stale: back to neutral")
        self.cmd_seq += 1
        tx_us = monotonic_us()
        self.cmd_tx[self.cmd_seq] = tx_us
        self.cmd_tx.pop(self.cmd_seq - 1000, None)
        payload = COMMAND.pack(self.cmd_seq, tx_us, self.cmd_a, self.cmd_b)
        self.record(
            {
                "kind": "sent",
                "type": MSG_COMMAND,
                "seq": self.cmd_seq,
                "tx_us": tx_us,
                "a": self.cmd_a,
                "b": self.cmd_b,
            }
        )
        return encode_frame(MSG_COMMAND, payload)

    def trim(self, now: float) -> None:
        for log, horizon in (
            (self.rx_log, 1.0),
            (self.tlm_window, 5.0),
            (self.host_gaps, 10.0),
            (self.robot_gaps, 10.0),
            (self.rtt, 10.0),
        ):
            while log and log[0][0] < now - horizon:
                log.popleft()
        for stats in self.types.values():
            while stats.arrivals and stats.arrivals[0] < now - 1.0:
                stats.arrivals.popleft()

    def sample_series(self, now: float) -> None:
        if now < self.series_next_s:
            return
        self.series_next_s = now + SERIES_PERIOD_S
        m = self.metrics(now)
        sample = {
            "ack_pct": m["ack_pct_1s"],
            "sent_hz": m["sent_hz"],
            "failed_hz": m["failed_hz"],
            "send_cb_us": self.link["last_rtt_us"] if self.link else None,
            "tlm_hz": m["tlm_hz"],
            "delivery_pct": m["delivery_pct_5s"],
            "lq": self.telemetry["espnow_lq"] if self.telemetry else None,
            "gap_max_ms": self.interval_gap_ms if self.telemetry else None,
            "rtt_min_ms": self.interval_rtt_ms,
            "rx_kbps": m["rx_bytes_per_s"] * 8 / 1000,
        }
        self.interval_gap_ms = 0.0
        self.interval_rtt_ms = None
        self.series_t.append(time.time())
        for key, value in sample.items():
            self.series.setdefault(key, deque()).append(value)
        while self.series_t and self.series_t[0] < time.time() - SERIES_SECONDS:
            self.series_t.popleft()
            for values in self.series.values():
                values.popleft()

    # ---- derived numbers -------------------------------------------------------------------

    def metrics(self, now: float) -> dict[str, Any]:
        m: dict[str, Any] = {}
        m["rx_bytes_per_s"] = sum(n for _, n in self.rx_log)

        sent_hz = acked_hz = failed_hz = None
        ack_1s = None
        if len(self.link_history) >= 2:
            (t0, s0, a0, f0), (t1, s1, a1, f1) = self.link_history[0], self.link_history[-1]
            span = t1 - t0
            if span > 0:
                sent_hz, acked_hz, failed_hz = (s1 - s0) / span, (a1 - a0) / span, (f1 - f0) / span
            if s1 > s0:
                # An ACK can land in a later stats frame than its send, so a window can hold
                # more ACKs than sends.
                ack_1s = min(100.0, 100.0 * (a1 - a0) / (s1 - s0))
        m.update(sent_hz=sent_hz, acked_hz=acked_hz, failed_hz=failed_hz, ack_pct_1s=ack_1s)
        ack_session = None
        if self.link and self.link_session:
            ds = self.link["sent"] - self.link_session[0]
            if ds > 0:
                ack_session = 100.0 * (self.link["acked"] - self.link_session[1]) / ds
        m["ack_pct_session"] = ack_session
        m["link_age_s"] = now - self.link_s if self.link else None

        m["tlm_hz"] = len(self.types[MSG_TELEMETRY].arrivals) if MSG_TELEMETRY in self.types else 0
        got = len(self.tlm_window)
        lost = sum(n for _, n in self.tlm_window)
        m["delivery_pct_5s"] = 100.0 * got / (got + lost) if got else None
        total = self.tlm_received + self.tlm_dropped
        m["delivery_pct_session"] = 100.0 * self.tlm_received / total if total else None
        m["telemetry_age_s"] = now - self.telemetry_s if self.telemetry else None

        host = sorted(g for _, g in self.host_gaps)
        robot = sorted(g for _, g in self.robot_gaps)
        m["host_gap_ms"] = {
            "p50": percentile(host, 0.5),
            "p90": percentile(host, 0.9),
            "p99": percentile(host, 0.99),
            "max": host[-1] if host else None,
            "n": len(host),
        }
        m["robot_interval_ms"] = {
            "p50": percentile(robot, 0.5),
            "max": robot[-1] if robot else None,
        }
        edges = [0, 5, 10, 15, 20, 30, 50, 100, 250, 1e9]
        m["gap_histogram"] = [
            {
                "label": f"{lo:g}-{hi:g}" if hi < 1e9 else f">{lo:g}",
                "count": sum(1 for g in host if lo <= g < hi),
            }
            for lo, hi in zip(edges, edges[1:])
        ]
        rtts = sorted(r for _, r in self.rtt)
        m["rtt_ms"] = {
            "min": rtts[0] if rtts else None,
            "p50": percentile(rtts, 0.5),
            "p90": percentile(rtts, 0.9),
            "max": rtts[-1] if rtts else None,
            "n": len(rtts),
        }
        return m

    def checks(self, now: float, m: dict[str, Any]) -> list[dict[str, str]]:
        """Ordered link checklist. Each later check assumes the earlier ones pass, so the list
        stops at the first check that rules out the rest."""
        out: list[dict[str, str]] = []
        if self._check_reach(out, m):
            self._check_telemetry(out, m)
            self._check_rates(out, m)
            self._check_hygiene(out, m)
        return out

    def _check_reach(self, out: list[dict[str, str]], m: dict[str, Any]) -> bool:
        """Port, dongle, and robot reachable. False when a failure hides everything after it."""
        if self.port_state != "OPEN":
            hint = self.port_error or f"waiting for {self.port_path}"
            if "busy" in self.port_error.lower() or "lock" in self.port_error.lower():
                hint += f". Is {SERVICE} holding it? Restart with --manage-service."
            out.append(check("Serial port", "error", self.port_state, hint))
            return False
        out.append(check("Serial port", "ok", "OPEN"))

        if self.link is None or m["link_age_s"] > DONGLE_SILENT_S:
            hint = (
                "No MSG_LINK_STATS in 0.5 s. Wrong device, dongle firmware not flashed, or "
                "dongle hung (RESET DONGLE, or replug)."
            )
            out.append(check("Dongle link stats", "error", "SILENT", hint))
            return False
        hz = len(self.types[MSG_LINK_STATS].arrivals)
        out.append(check("Dongle link stats", "ok", f"{hz} Hz"))

        if not self.link["robot_known"]:
            hint = (
                f"Dongle on channel {self.link['channel']} has never received robot telemetry. "
                "Robot off, out of range, ESPNOW_ENABLED=false, on another channel, or this "
                "dongle not paired on the robot's page."
            )
            out.append(check("Robot heard", "error", "BROADCAST", hint))
            return False
        out.append(check("Robot heard", "ok", "UNICAST"))
        return True

    def _check_telemetry(self, out: list[dict[str, str]], m: dict[str, Any]) -> None:
        if self.telemetry is None or m["telemetry_age_s"] > DONGLE_SILENT_S:
            age = "never" if self.telemetry is None else f"{m['telemetry_age_s']:.1f} s ago"
            hint = (
                f"Last telemetry {age}. Robot rebooted, browned out, out of range, or this "
                "dongle was unpaired on the robot's page."
            )
            out.append(check("Robot telemetry", "error", "STALE", hint))
        else:
            hz = m["tlm_hz"]
            level = grade(hz, 80, 50)
            hint = f"Expected {1000 // TELEMETRY_PERIOD_MS} Hz."
            out.append(check("Robot telemetry", level, f"{hz} Hz", hint))
        if self.telemetry is None or self.link is None:
            return
        robot_ch = self.telemetry["espnow_channel"]
        if robot_ch == 0:
            hint = "Robot has not settled on a channel."
            out.append(check("Channel match", "warn", "ROBOT SWEEPING", hint))
        elif robot_ch != self.link["channel"]:
            hint = "Dongle and robot report different channels."
            value = f"{self.link['channel']} vs {robot_ch}"
            out.append(check("Channel match", "error", value, hint))
        else:
            out.append(check("Channel match", "ok", f"CH {robot_ch}"))

    def _check_rates(self, out: list[dict[str, str]], m: dict[str, Any]) -> None:
        ack = m["ack_pct_1s"]
        if ack is not None:
            hint = "MAC-layer ACK failures after retries: range, antenna, or a busy channel."
            out.append(check("Dongle to robot ACK", grade(ack, 95, 80), f"{ack:.1f} %", hint))
        delivery = m["delivery_pct_5s"]
        if delivery is not None:
            hint = "telemetry_seq gaps: frames lost on the way back."
            level = grade(delivery, 98, 90)
            out.append(check("Robot to dongle delivery", level, f"{delivery:.1f} %", hint))
        if self.telemetry:
            lq = self.telemetry["espnow_lq"]
            hint = "Robot's own ACK rate."
            out.append(check("Robot telemetry LQ", grade(lq, 95, 80), f"{lq} %", hint))

    def _check_hygiene(self, out: list[dict[str, str]], m: dict[str, Any]) -> None:
        if self.foreign_frames:
            hint = (
                "Frames with our session magic that the robot did not send: a second dongle or "
                "host is on this channel."
            )
            value = f"{self.foreign_frames} foreign"
            out.append(check("Channel to ourselves", "warn", value, hint))
        else:
            out.append(check("Channel to ourselves", "ok", "NO FOREIGN FRAMES"))

        crc = self.parser.crc_errors
        hint = "USB corruption, or two readers on the port."
        out.append(check("Serial CRC", "warn" if crc else "ok", f"{crc} errors", hint))

        if not self.telemetry:
            return
        name = "Robot accepts commands"
        if not self.telemetry["autonomy_enabled"]:
            hint = "The robot ignores ESP-NOW commands until the autonomy switch (SA) is on."
            out.append(check(name, "info", "SA OFF", hint))
        elif self.sending and m["rtt_ms"]["n"] == 0:
            hint = "Sending, but the robot never echoes our seq. Is this dongle paired?"
            out.append(check(name, "error", "NO ECHO", hint))
        elif self.sending:
            out.append(check(name, "ok", f"RTT min {m['rtt_ms']['min']:.1f} ms"))
        else:
            out.append(
                check(name, "info", "NOT TESTED", "SEND COMMANDS measures the command path.")
            )

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        m = self.metrics(now)
        types = []
        for msg_type, stats in sorted(self.types.items()):
            name, sender, expected = MESSAGE_TYPES.get(msg_type, (f"0x{msg_type:02X}", "?", None))
            types.append(
                {
                    "type": msg_type,
                    "name": name,
                    "sender": sender,
                    "expected": expected,
                    "count": stats.count,
                    "bytes": stats.bytes,
                    "size_errors": stats.size_errors,
                    "hz": len(stats.arrivals),
                    "age_s": now - stats.last_s,
                    "foreign": msg_type not in ROBOT_TYPES and msg_type != MSG_LINK_STATS,
                }
            )
        return {
            "now": time.time(),
            "uptime_s": now - self.started_s,
            "port": {
                "path": self.port_path,
                "state": self.port_state,
                "error": self.port_error,
                "open_s": now - self.opened_s if self.opened_s else None,
                "opens": self.opens,
                "rx_bytes": self.rx_bytes,
                "rx_bytes_per_s": m["rx_bytes_per_s"],
                "crc_errors": self.parser.crc_errors,
                "junk_bytes": self.parser.junk_bytes,
                "frames": sum(s.count for s in self.types.values()),
            },
            "banner": self.banner,
            "link": self.link,
            "link_metrics": {
                "sent_hz": m["sent_hz"],
                "acked_hz": m["acked_hz"],
                "failed_hz": m["failed_hz"],
                "ack_pct_1s": m["ack_pct_1s"],
                "ack_pct_session": m["ack_pct_session"],
                "age_s": m["link_age_s"],
                "gap_max_ms": self.link_gap_max_ms,
                "dongle_reboots": self.dongle_reboots,
                "hz": len(self.types[MSG_LINK_STATS].arrivals)
                if MSG_LINK_STATS in self.types
                else 0,
            },
            "telemetry": self.telemetry,
            "telemetry_metrics": {
                "hz": m["tlm_hz"],
                "age_s": m["telemetry_age_s"],
                "received": self.tlm_received,
                "dropped": self.tlm_dropped,
                "delivery_pct_5s": m["delivery_pct_5s"],
                "delivery_pct_session": m["delivery_pct_session"],
                "robot_reboots": self.robot_reboots,
                "big_gaps": self.big_gaps,
                "host_gap_ms": m["host_gap_ms"],
                "robot_interval_ms": m["robot_interval_ms"],
                "gap_histogram": m["gap_histogram"],
                "clock_drift_ms": self.robot_clock_drift_ms if self.telemetry else None,
            },
            "command": {
                "sending": self.sending,
                "a": self.cmd_a,
                "b": self.cmd_b,
                "hz": 1.0 / self.command_period_s,
                "seq": self.cmd_seq,
                "sent": self.cmd_sent,
                "write_errors": self.cmd_write_errors,
                "robot_seq": self.telemetry["espnow_seq"] if self.telemetry else None,
                "robot_age_ms": self.telemetry["espnow_age_ms"] if self.telemetry else None,
                "seq_jumps": self.cmd_seq_jumps,
                "foreign_echo": self.cmd_foreign_echo,
                "rtt_ms": m["rtt_ms"],
            },
            "ota": self.ota,
            "ota_age_s": now - self.ota_s if self.ota else None,
            "foreign_frames": self.foreign_frames,
            "types": types,
            "checks": self.checks(now, m),
            "events": list(self.events)[-150:][::-1],
            "text_lines": list(self.text_lines)[-50:][::-1],
            "capture": {
                "active": self.capture_file is not None,
                "path": str(self.capture_path) if self.capture_path else None,
                "records": self.capture_records,
                "ring_records": len(self.ring),
                "dir": str(self.capture_dir),
            },
        }


# ---- serial thread ---------------------------------------------------------------------------


def open_port(path: str) -> serial.Serial:
    # DTR and RTS go high before the open, as in espnow_ota.py: setting them low one at a time
    # passes through DTR low RTS high, which resets an ESP32-S3 on its USB-Serial/JTAG port.
    port = serial.Serial(timeout=0.002, exclusive=True)
    port.port = path
    port.baudrate = BAUD
    port.dtr = True
    port.rts = True
    port.open()
    return port


def reset_dongle(port: serial.Serial) -> None:
    """One chip reset through the USB-Serial/JTAG lines: DTR low with RTS high holds EN low, then
    RTS low releases it with GPIO0 free, so the dongle boots its firmware and prints the banner.
    One pulse only: two resets inside ~500 ms drop an Adafruit board into TinyUF2."""
    port.dtr = False
    time.sleep(0.2)
    port.rts = False
    time.sleep(0.05)
    port.dtr = True
    port.rts = True


def try_open(diag: Diagnostics) -> serial.Serial | None:
    try:
        port = open_port(diag.port_path)
    except (serial.SerialException, OSError) as e:
        with diag.lock:
            state = "MISSING" if not Path(diag.port_path).exists() else "BUSY"
            if diag.port_state != state or diag.port_error != str(e):
                diag.event("error", f"cannot open {diag.port_path}: {e}")
            diag.port_state, diag.port_error = state, str(e)
        return None
    with diag.lock:
        diag.port = port
        diag.port_state, diag.port_error = "OPEN", ""
        diag.opened_s = time.monotonic()
        diag.opens += 1
        diag.parser = FrameParser()
        diag.event("good", f"opened {diag.port_path}")
    return port


def service_port(diag: Diagnostics, port: serial.Serial) -> None:
    """One pass: read, write a due command, run a requested reset, update rates."""
    data = port.read(port.in_waiting or 1)
    with diag.lock:
        now = time.monotonic()
        if data:
            diag.on_bytes(data, now)
        frame = diag.next_command(now)
        reset = diag.reset_requested
        diag.reset_requested = False
    if frame is not None:
        port.write(frame)
        with diag.lock:
            diag.cmd_sent += 1
    if reset:
        reset_dongle(port)
    with diag.lock:
        diag.trim(now)
        diag.sample_series(now)


def serial_loop(diag: Diagnostics, stop: threading.Event) -> None:
    next_open = 0.0
    while not stop.is_set():
        with diag.lock:
            port = diag.port
        if port is None:
            if time.monotonic() < next_open:
                time.sleep(0.05)
                continue
            next_open = time.monotonic() + REOPEN_PERIOD_S
            port = try_open(diag)
            if port is None:
                continue
        try:
            service_port(diag, port)
        except (serial.SerialException, OSError) as e:
            with diag.lock:
                diag.event("error", f"port lost: {e}")
                diag.port = None
                diag.port_state, diag.port_error = "LOST", str(e)
                diag.sending = False
            try:
                port.close()
            except (serial.SerialException, OSError):
                pass
    with diag.lock:
        if diag.port is not None:
            diag.port.close()
        diag.stop_capture()


# ---- web -------------------------------------------------------------------------------------


def make_app(diag: Diagnostics) -> Flask:
    app = Flask(__name__, static_folder=None)
    add_page_routes(app, diag)
    add_command_routes(app, diag)
    add_control_routes(app, diag)
    add_capture_routes(app, diag)
    return app


def add_page_routes(app: Flask, diag: Diagnostics) -> None:
    @app.get("/")
    def index() -> Response:
        return send_from_directory(WEB_DIR, "index.html")

    @app.get("/style.css")
    def shared_style() -> Response:
        return send_from_directory(ROBOT_WEB_DIR, "style.css")

    @app.get("/fonts/<path:name>")
    def fonts(name: str) -> Response:
        return send_from_directory(ROBOT_WEB_DIR / "fonts", name)

    @app.get("/<any('app.js', 'diag.css'):name>")
    def assets(name: str) -> Response:
        return send_from_directory(WEB_DIR, name)

    @app.get("/api/state")
    def state() -> Response:
        with diag.lock:
            return jsonify(diag.snapshot())

    @app.get("/api/series")
    def series() -> Response:
        with diag.lock:
            t0 = time.time()
            return jsonify(
                {
                    "t": [t - t0 for t in diag.series_t],
                    "series": {k: list(v) for k, v in diag.series.items()},
                    "window_s": SERIES_SECONDS,
                }
            )

    @app.get("/api/snapshot")
    def snapshot_download() -> Response:
        with diag.lock:
            body = json.dumps(diag.snapshot(), indent=2)
        name = time.strftime("espnow_snapshot_%Y%m%d_%H%M%S.json")
        return Response(
            body,
            mimetype="application/json",
            headers={"Content-Disposition": f"attachment; filename={name}"},
        )


def add_command_routes(app: Flask, diag: Diagnostics) -> None:
    @app.post("/api/command")
    def command() -> Response:
        on = bool((request.get_json(silent=True) or {}).get("on"))
        with diag.lock:
            if on and diag.port_state != "OPEN":
                abort(409, "port not open")
            if on != diag.sending:
                diag.sending = on
                diag.cmd_a = diag.cmd_b = 0.0
                diag.cmd_next_s = time.monotonic()
                diag.rtt.clear()
                diag.event("info", "SEND COMMANDS on, neutral" if on else "SEND COMMANDS off")
        return jsonify({"sending": on})

    @app.post("/api/command/value")
    def command_value() -> Response:
        body = request.get_json(silent=True) or {}
        try:
            a = float(body.get("a", 0.0))
            b = float(body.get("b", 0.0))
        except (TypeError, ValueError):
            abort(400, "a and b must be numbers")
        if not (math.isfinite(a) and math.isfinite(b)):
            abort(400, "a and b must be finite")
        with diag.lock:
            if not diag.sending:
                abort(409, "not sending")
            diag.cmd_a = max(-1.0, min(1.0, a))
            diag.cmd_b = max(-1.0, min(1.0, b))
            diag.cmd_set_s = time.monotonic()
            return jsonify({"a": diag.cmd_a, "b": diag.cmd_b})


def add_control_routes(app: Flask, diag: Diagnostics) -> None:

    @app.post("/api/reset_dongle")
    def reset() -> Response:
        with diag.lock:
            if diag.port_state != "OPEN":
                abort(409, "port not open")
            diag.reset_requested = True
            diag.event("info", "resetting dongle")
        return jsonify({"ok": True})

    @app.post("/api/clear")
    def clear() -> Response:
        with diag.lock:
            diag.reset_counters()
            diag.event("info", "counters cleared")
        return jsonify({"ok": True})


def add_capture_routes(app: Flask, diag: Diagnostics) -> None:
    @app.post("/api/capture")
    def capture() -> Response:
        on = bool((request.get_json(silent=True) or {}).get("on"))
        with diag.lock:
            if on and diag.capture_file is None:
                diag.start_capture()
            elif not on:
                diag.stop_capture()
            return jsonify({"active": diag.capture_file is not None})

    @app.post("/api/save_ring")
    def save_ring() -> Response:
        with diag.lock:
            path = diag.save_ring()
        return jsonify({"name": path.name})

    @app.get("/api/captures")
    def captures() -> Response:
        files = sorted(diag.capture_dir.glob("espnow_*.jsonl"), reverse=True)[:20]
        return jsonify([{"name": f.name, "bytes": f.stat().st_size} for f in files])

    @app.get("/captures/<name>")
    def download(name: str) -> Response:
        if not re.fullmatch(r"espnow_[\w]+\.jsonl", name):
            abort(404)
        return send_from_directory(diag.capture_dir, name, as_attachment=True)


def systemctl(action: str) -> None:
    subprocess.run(["sudo", "systemctl", action, SERVICE], check=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--port", default="/dev/espnow_dongle", help="dongle serial device")
    parser.add_argument("--host", default="0.0.0.0", help="address the page listens on")
    parser.add_argument("--http-port", type=int, default=8090)
    parser.add_argument(
        "--capture-dir",
        type=Path,
        default=Path.home() / "espnow_diag",
        help="where CAPTURE and SAVE LAST 2 MIN write JSONL",
    )
    parser.add_argument(
        "--command-hz",
        type=float,
        default=50.0,
        help="SEND COMMANDS rate (the app sends 250 Hz)",
    )
    parser.add_argument(
        "--manage-service",
        action="store_true",
        help=f"stop {SERVICE} to free the dongle, start it again on exit",
    )
    args = parser.parse_args()

    diag = Diagnostics(args.port, args.capture_dir.expanduser(), args.command_hz)
    stop = threading.Event()
    stopped_service = False
    try:
        if args.manage_service:
            print(f"stopping {SERVICE}")
            systemctl("stop")
            stopped_service = True
        thread = threading.Thread(target=serial_loop, args=(diag, stop), daemon=True)
        thread.start()
        print(f"reading {args.port}; page at http://{args.host}:{args.http_port}")
        make_app(diag).run(host=args.host, port=args.http_port, threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        time.sleep(0.1)
        if stopped_service:
            print(f"starting {SERVICE}")
            systemctl("start")
    return 0


if __name__ == "__main__":
    sys.exit(main())
