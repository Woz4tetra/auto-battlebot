#!/usr/bin/env python3
"""
Print /rosout messages from an MCAP file in journalctl-style format.

Output format mirrors journalctl --no-pager:
  MMM DD HH:MM:SS.mmm <node> [LEVEL]: <message>
"""

import argparse
import sys
from datetime import datetime, timezone
from pathlib import Path

from mcap_ros1.reader import read_ros1_messages

# ROS1 rosgraph_msgs/Log level constants
_LEVEL_NAME = {
    1: "DEBUG",
    2: "INFO",
    4: "WARN",
    8: "ERROR",
    16: "FATAL",
}

# ANSI colour codes (disabled when not a tty)
_COLOURS = {
    "DEBUG": "\033[37m",     # grey
    "INFO": "",              # default
    "WARN": "\033[33m",      # yellow
    "ERROR": "\033[31m",     # red
    "FATAL": "\033[1;31m",   # bold red
    "RESET": "\033[0m",
}


def format_ts(secs: int, nsecs: int) -> str:
    dt = datetime.fromtimestamp(secs + nsecs / 1e9, tz=timezone.utc).astimezone()
    return dt.strftime("%b %d %H:%M:%S.") + f"{nsecs // 1_000_000:03d}"


def print_log(path: Path, use_colour: bool, min_level: int) -> None:
    try:
        for msg in read_ros1_messages(str(path), topics=["/rosout"]):
            m = msg.ros_msg
            if m.level < min_level:
                continue

            level_name = _LEVEL_NAME.get(m.level, f"LEVEL{m.level}")
            ts = format_ts(m.header.stamp.secs, m.header.stamp.nsecs)
            node = m.name or "?"

            line = f"{ts} {node} [{level_name}]: {m.msg}"

            if use_colour:
                colour = _COLOURS.get(level_name, "")
                reset = _COLOURS["RESET"] if colour else ""
                line = f"{colour}{line}{reset}"

            print(line)
    except Exception as e:
        print(f"warning: error reading {path.name}: {e}", file=sys.stderr)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Print /rosout from an MCAP file like journalctl."
    )
    parser.add_argument("files", nargs="+", type=Path, help="MCAP file(s) to read")
    parser.add_argument(
        "--level",
        choices=["debug", "info", "warn", "error", "fatal"],
        default="info",
        help="Minimum log level to display (default: info)",
    )
    parser.add_argument(
        "--no-colour",
        action="store_true",
        help="Disable ANSI colour output",
    )
    args = parser.parse_args()

    level_map = {"debug": 1, "info": 2, "warn": 4, "error": 8, "fatal": 16}
    min_level = level_map[args.level]
    use_colour = not args.no_colour and sys.stdout.isatty()

    for path in args.files:
        if not path.exists():
            print(f"File not found: {path}", file=sys.stderr)
            sys.exit(1)
        print_log(path, use_colour, min_level)


if __name__ == "__main__":
    main()
