#!/usr/bin/env python3
"""Analyze MCAP recordings for likely failure events.

This script scans:
  - Per-topic timing gaps (stalls / dropped publisher cadence)
  - /diagnostics WARN/ERROR statuses

It then selects a likely failure event timestamp and prints a concise summary.

Usage:
  python3 playground/analyze_failure_event.py data/recordings/foo.mcap
"""

from __future__ import annotations

import argparse
import statistics
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

from mcap.reader import make_reader


DIAGNOSTICS_TOPIC = "/diagnostics"


@dataclass
class DiagnosticEvent:
    ts_ns: int
    level: int
    name: str
    message: str
    hardware_id: str
    values: dict[str, str]


@dataclass
class TopicGapEvent:
    topic: str
    start_ns: int
    end_ns: int
    gap_s: float
    median_period_s: float


def _read_string(data: bytes, offset: int) -> tuple[str, int]:
    (length,) = struct.unpack_from("<I", data, offset)
    offset += 4
    value = data[offset : offset + length].decode("utf-8", errors="replace")
    return value, offset + length


def _read_uint32(data: bytes, offset: int) -> tuple[int, int]:
    (value,) = struct.unpack_from("<I", data, offset)
    return value, offset + 4


def _read_int8(data: bytes, offset: int) -> tuple[int, int]:
    (value,) = struct.unpack_from("<b", data, offset)
    return value, offset + 1


def decode_diagnostic_array(data: bytes) -> list[dict]:
    """Decode diagnostic_msgs/DiagnosticArray from ROS1 wire bytes."""
    off = 0
    # Header: seq, stamp.secs, stamp.nsecs, frame_id
    _seq, off = _read_uint32(data, off)
    _secs, off = _read_uint32(data, off)
    _nsecs, off = _read_uint32(data, off)
    _frame_id, off = _read_string(data, off)

    status_count, off = _read_uint32(data, off)
    out: list[dict] = []
    for _ in range(status_count):
        level, off = _read_int8(data, off)
        name, off = _read_string(data, off)
        message, off = _read_string(data, off)
        hardware_id, off = _read_string(data, off)
        value_count, off = _read_uint32(data, off)
        values: dict[str, str] = {}
        for _ in range(value_count):
            k, off = _read_string(data, off)
            v, off = _read_string(data, off)
            values[k] = v
        out.append(
            {
                "level": level,
                "name": name,
                "message": message,
                "hardware_id": hardware_id,
                "values": values,
            }
        )
    return out


def ns_to_s(ts_ns: int, t0_ns: int) -> float:
    return (ts_ns - t0_ns) / 1e9


def collect(path: Path) -> tuple[dict[str, list[int]], list[DiagnosticEvent], int, int]:
    topic_times: dict[str, list[int]] = {}
    diagnostics: list[DiagnosticEvent] = []
    first_ns: int | None = None
    last_ns: int | None = None

    with open(path, "rb") as f:
        reader = make_reader(f)
        for _schema, channel, message in reader.iter_messages():
            ts_ns = message.log_time
            if first_ns is None or ts_ns < first_ns:
                first_ns = ts_ns
            if last_ns is None or ts_ns > last_ns:
                last_ns = ts_ns

            topic = channel.topic
            if topic not in topic_times:
                topic_times[topic] = []
            topic_times[topic].append(ts_ns)

            if topic == DIAGNOSTICS_TOPIC:
                try:
                    statuses = decode_diagnostic_array(message.data)
                except Exception:
                    continue
                for status in statuses:
                    diagnostics.append(
                        DiagnosticEvent(
                            ts_ns=ts_ns,
                            level=int(status["level"]),
                            name=str(status["name"]),
                            message=str(status["message"]),
                            hardware_id=str(status["hardware_id"]),
                            values=dict(status["values"]),
                        )
                    )

    if first_ns is None or last_ns is None:
        raise RuntimeError("No messages found in MCAP.")
    return topic_times, diagnostics, first_ns, last_ns


def detect_topic_gaps(
    topic_times: dict[str, list[int]],
    *,
    min_samples: int = 20,
    min_gap_s: float = 2.0,
    gap_factor: float = 5.0,
) -> list[TopicGapEvent]:
    events: list[TopicGapEvent] = []

    for topic, times in topic_times.items():
        if len(times) < min_samples:
            continue
        times = sorted(times)
        deltas_s = [(times[i] - times[i - 1]) / 1e9 for i in range(1, len(times))]
        if not deltas_s:
            continue
        median_period = statistics.median(deltas_s)
        threshold = max(min_gap_s, median_period * gap_factor)

        for i in range(1, len(times)):
            gap_s = (times[i] - times[i - 1]) / 1e9
            if gap_s >= threshold:
                events.append(
                    TopicGapEvent(
                        topic=topic,
                        start_ns=times[i - 1],
                        end_ns=times[i],
                        gap_s=gap_s,
                        median_period_s=median_period,
                    )
                )
    events.sort(key=lambda e: e.gap_s, reverse=True)
    return events


def choose_failure_event(
    diagnostics: list[DiagnosticEvent], gaps: list[TopicGapEvent]
) -> tuple[int | None, str]:
    # Prefer explicit ERROR diagnostics, then WARN diagnostics, then biggest gap.
    error_events = [d for d in diagnostics if d.level >= 2]
    if error_events:
        event = min(error_events, key=lambda d: d.ts_ns)
        return event.ts_ns, "diagnostics_error"

    warn_events = [d for d in diagnostics if d.level == 1]
    if warn_events:
        event = min(warn_events, key=lambda d: d.ts_ns)
        return event.ts_ns, "diagnostics_warn"

    if gaps:
        return gaps[0].start_ns, "topic_stall_gap"

    return None, "none_found"


def print_summary(
    path: Path,
    topic_times: dict[str, list[int]],
    diagnostics: list[DiagnosticEvent],
    gaps: list[TopicGapEvent],
    first_ns: int,
    last_ns: int,
) -> None:
    duration_s = (last_ns - first_ns) / 1e9
    total_messages = sum(len(v) for v in topic_times.values())
    print(f"\nMCAP: {path}")
    print(f"Duration: {duration_s:.1f}s")
    print(f"Topics: {len(topic_times)}")
    print(f"Messages: {total_messages}")
    print(f"Diagnostics events (WARN/ERROR): {sum(1 for d in diagnostics if d.level >= 1)}")

    event_ns, reason = choose_failure_event(diagnostics, gaps)
    if event_ns is None:
        print("\nNo obvious failure event detected.")
        if gaps:
            top = gaps[0]
            print(
                f"Largest gap seen: {top.topic} gap={top.gap_s:.2f}s "
                f"(median {top.median_period_s:.3f}s)"
            )
        return

    t_event_s = ns_to_s(event_ns, first_ns)
    print("\nLikely failure event")
    print(f"- Time from start: {t_event_s:.2f}s")
    print(f"- Trigger source: {reason}")

    window_before_ns = int(10 * 1e9)
    window_after_ns = int(10 * 1e9)
    ws = event_ns - window_before_ns
    we = event_ns + window_after_ns

    nearby_diag = [d for d in diagnostics if ws <= d.ts_ns <= we and d.level >= 1]
    if nearby_diag:
        print("- Nearby diagnostics WARN/ERROR:")
        for d in nearby_diag[:8]:
            rel = ns_to_s(d.ts_ns, first_ns)
            level_name = "WARN" if d.level == 1 else "ERROR"
            name = d.name if d.name else "<root>"
            msg = d.message if d.message else "(no message)"
            print(f"  - t={rel:8.2f}s [{level_name}] {d.hardware_id}/{name}: {msg}")
    else:
        print("- Nearby diagnostics WARN/ERROR: none")

    nearby_gaps = [g for g in gaps if ws <= g.start_ns <= we or ws <= g.end_ns <= we]
    if nearby_gaps:
        print("- Nearby topic gaps:")
        for g in sorted(nearby_gaps, key=lambda x: x.gap_s, reverse=True)[:8]:
            rel = ns_to_s(g.start_ns, first_ns)
            print(
                f"  - t={rel:8.2f}s topic={g.topic} gap={g.gap_s:.2f}s "
                f"(median={g.median_period_s:.3f}s)"
            )
    else:
        print("- Nearby topic gaps: none")

    # Global top offenders for quick prioritization.
    if gaps:
        print("\nTop global topic stalls:")
        for g in gaps[:10]:
            rel = ns_to_s(g.start_ns, first_ns)
            print(
                f"  - t={rel:8.2f}s topic={g.topic} gap={g.gap_s:.2f}s "
                f"(median={g.median_period_s:.3f}s)"
            )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Analyze MCAP for likely runtime failure events."
    )
    parser.add_argument("file", type=Path, help="Path to MCAP recording")
    parser.add_argument(
        "--min-gap-s",
        type=float,
        default=2.0,
        help="Minimum absolute gap seconds to flag (default: 2.0)",
    )
    parser.add_argument(
        "--gap-factor",
        type=float,
        default=5.0,
        help="Gap multiplier vs median period to flag (default: 5.0)",
    )
    parser.add_argument(
        "--min-samples",
        type=int,
        default=20,
        help="Minimum messages on a topic before gap analysis (default: 20)",
    )
    args = parser.parse_args()

    if not args.file.exists():
        print(f"File not found: {args.file}", file=sys.stderr)
        sys.exit(1)

    topic_times, diagnostics, first_ns, last_ns = collect(args.file)
    gaps = detect_topic_gaps(
        topic_times,
        min_samples=args.min_samples,
        min_gap_s=args.min_gap_s,
        gap_factor=args.gap_factor,
    )
    print_summary(args.file, topic_times, diagnostics, gaps, first_ns, last_ns)


if __name__ == "__main__":
    main()
