#!/usr/bin/env python3
"""
Print the total size, average message size, and KB/s of each topic in an MCAP file.
"""

import argparse
import sys
from collections import defaultdict
from pathlib import Path

from mcap.reader import make_reader


def human_readable(size_bytes: int) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if size_bytes < 1024:
            return f"{size_bytes:.1f} {unit}"
        size_bytes /= 1024
    return f"{size_bytes:.1f} TB"


def analyze(path: Path) -> None:
    topic_bytes: dict[str, int] = defaultdict(int)
    topic_counts: dict[str, int] = defaultdict(int)
    topic_type: dict[str, str] = {}
    topic_min_ns: dict[str, int] = {}
    topic_max_ns: dict[str, int] = {}

    with open(path, "rb") as f:
        reader = make_reader(f)

        summary = reader.get_summary()
        if summary is None:
            print(
                "No summary found — scanning all messages (slower)...", file=sys.stderr
            )
        else:
            channels = summary.channels
            schemas = summary.schemas
            for channel in channels.values():
                schema = schemas.get(channel.schema_id)
                topic_type[channel.topic] = schema.name if schema else "unknown"
            for channel_id, count in summary.statistics.channel_message_counts.items():
                ch = channels[channel_id]
                topic_counts[ch.topic] += count

        # Iterate messages to accumulate byte sizes and time spans
        f.seek(0)
        reader = make_reader(f)
        for schema, channel, message in reader.iter_messages():
            topic = channel.topic
            size = (
                message.data_length
                if hasattr(message, "data_length")
                else len(message.data)
            )
            topic_bytes[topic] += size

            t = message.log_time  # nanoseconds
            if topic not in topic_min_ns or t < topic_min_ns[topic]:
                topic_min_ns[topic] = t
            if topic not in topic_max_ns or t > topic_max_ns[topic]:
                topic_max_ns[topic] = t

            if topic not in topic_type and schema:
                topic_type[topic] = schema.name
            topic_counts[topic]  # touch to ensure key exists

    if not topic_bytes:
        print("No messages found.")
        return

    total_bytes = sum(topic_bytes.values())
    total_msgs = sum(topic_counts.values())

    # Overall file time span (across all topics)
    all_min = min(topic_min_ns.values()) if topic_min_ns else 0
    all_max = max(topic_max_ns.values()) if topic_max_ns else 0
    total_duration_s = (all_max - all_min) / 1e9

    # Sort by size descending
    sorted_topics = sorted(topic_bytes.items(), key=lambda x: x[1], reverse=True)

    col_topic = max(len(t) for t, _ in sorted_topics)
    col_topic = max(col_topic, 5)

    header = (
        f"{'Topic':<{col_topic}}  {'Type':<45}  {'Messages':>10}"
        f"  {'Size':>10}  {'Avg/msg':>10}  {'KB/s':>8}  {'% Total':>8}"
    )
    print(f"\nFile: {path}")
    if total_duration_s > 0:
        print(f"Duration: {total_duration_s:.1f}s")
    print("-" * len(header))
    print(header)
    print("-" * len(header))

    for topic, size in sorted_topics:
        msg_type = topic_type.get(topic, "unknown")
        count = topic_counts.get(topic, 0)
        avg = size / count if count else 0
        pct = 100.0 * size / total_bytes if total_bytes else 0

        duration_s = (topic_max_ns.get(topic, 0) - topic_min_ns.get(topic, 0)) / 1e9
        kbps = (size / 1024) / duration_s if duration_s > 0 else 0

        print(
            f"{topic:<{col_topic}}  {msg_type:<45}  {count:>10,}"
            f"  {human_readable(size):>10}  {human_readable(int(avg)):>10}"
            f"  {kbps:>7.1f}  {pct:>7.1f}%"
        )

    total_avg = int(total_bytes / total_msgs) if total_msgs else 0
    total_kbps = (total_bytes / 1024) / total_duration_s if total_duration_s > 0 else 0
    print("-" * len(header))
    print(
        f"{'TOTAL':<{col_topic}}  {'':45}  {total_msgs:>10,}"
        f"  {human_readable(int(total_bytes)):>10}  {human_readable(total_avg):>10}"
        f"  {total_kbps:>7.1f}  {'100.0%':>8}"
    )
    print()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Print per-topic sizes in an MCAP file."
    )
    parser.add_argument("files", nargs="+", type=Path, help="MCAP file(s) to analyze")
    args = parser.parse_args()

    for path in args.files:
        if not path.exists():
            print(f"File not found: {path}", file=sys.stderr)
            sys.exit(1)
        analyze(path)


if __name__ == "__main__":
    main()
