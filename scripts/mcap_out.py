#!/usr/bin/env python3
"""
Print /log messages from an MCAP file in journalctl-style format.

Reads the ``foxglove.Log`` channel on ``/log``.

Output format mirrors journalctl --no-pager:
  MMM DD HH:MM:SS.mmm <node> [LEVEL]: <message>
"""

import argparse
import sys
from datetime import datetime, timezone
from pathlib import Path

from auto_battlebot.mcap_io import decode_log, iter_messages

_LEVEL_RANK = {"DEBUG": 0, "INFO": 1, "WARN": 2, "ERROR": 3, "FATAL": 4}

# ANSI colour codes (disabled when not a tty)
_COLOURS = {
    "DEBUG": "\033[37m",  # grey
    "INFO": "",  # default
    "WARN": "\033[33m",  # yellow
    "ERROR": "\033[31m",  # red
    "FATAL": "\033[1;31m",  # bold red
    "RESET": "\033[0m",
}


def format_ts(secs: int, nsecs: int) -> str:
    dt = datetime.fromtimestamp(secs + nsecs / 1e9, tz=timezone.utc).astimezone()
    return dt.strftime("%b %d %H:%M:%S.") + f"{nsecs // 1_000_000:03d}"


def print_log(path: Path, use_colour: bool, min_level: int) -> None:
    try:
        for _topic, _log_time, data in iter_messages(path, ["/log"]):
            m = decode_log(data)
            level_name = m.level
            if _LEVEL_RANK.get(level_name, 1) < min_level:
                continue

            ts = format_ts(m.stamp_ns // 1_000_000_000, m.stamp_ns % 1_000_000_000)
            node = m.name or "?"

            line = f"{ts} {node} [{level_name}]: {m.message}"

            if use_colour:
                colour = _COLOURS.get(level_name, "")
                reset = _COLOURS["RESET"] if colour else ""
                line = f"{colour}{line}{reset}"

            print(line)
    except Exception as e:
        print(f"warning: error reading {path.name}: {e}", file=sys.stderr)


def main() -> None:
    parser = argparse.ArgumentParser(description="Print /log from an MCAP file like journalctl.")
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

    min_level = _LEVEL_RANK[args.level.upper()]
    use_colour = not args.no_colour and sys.stdout.isatty()

    for path in args.files:
        if not path.exists():
            print(f"File not found: {path}", file=sys.stderr)
            sys.exit(1)
        print_log(path, use_colour, min_level)


if __name__ == "__main__":
    main()
