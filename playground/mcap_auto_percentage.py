#!/usr/bin/env python3
"""
Report what percentage of a match recording the robot spent in autonomous mode.

The match window runs from the first time values/15 == 1024 (auto engaged)
to the last auto->manual transition.  Auto mode is detected from the
diagnostics: opentx_transmitter / channels / values/15 == 1024.
"""

import argparse
import sys
from pathlib import Path

from auto_battlebot.diag_io import iter_diagnostic_statuses


def analyze(path: Path) -> None:
    # Collect all (timestamp_float, is_auto) channel samples from the file.
    # We need the full list before we can determine the match window boundaries.
    raw: list[tuple[float, bool]] = []

    try:
        for log_time_ns, status in iter_diagnostic_statuses(path):
            if status["hardware_id"] != "opentx_transmitter" or status["name"] != "channels":
                continue
            ch15 = status["values"].get("values/15")
            is_auto = ch15 == 1024
            raw.append((log_time_ns / 1e9, is_auto))

    except Exception as e:
        print(f"  warning: error reading {path.name}: {e}", file=sys.stderr)

    if not raw:
        print(f"File: {path}")
        print("  no opentx_transmitter/channels diagnostics found")
        return

    # Determine match boundaries.
    first_auto_ts = next((t for t, a in raw if a), None)
    last_auto_ts = next((t for t, a in reversed(raw) if a), None)

    last_switch_to_manual_ts: float | None = None
    for i in range(1, len(raw)):
        if raw[i - 1][1] and not raw[i][1]:
            last_switch_to_manual_ts = raw[i][0]

    # Match window: first auto -> last switch to manual (or last auto if still in auto at end).
    match_end = last_switch_to_manual_ts if last_switch_to_manual_ts is not None else last_auto_ts
    duration_s = (match_end - first_auto_ts) if (first_auto_ts and match_end) else 0.0

    # Count only samples inside the match window for the percentage.
    match_samples = [
        (t, a)
        for t, a in raw
        if first_auto_ts is not None and match_end is not None and first_auto_ts <= t <= match_end
    ]
    total_samples = len(match_samples)
    auto_samples = sum(1 for _, a in match_samples if a)

    pct = 100.0 * auto_samples / total_samples if total_samples else 0.0
    auto_s = pct / 100.0 * duration_s

    print(f"File:            {path}")
    print(f"Match duration:  {duration_s:.1f}s")
    print(f"Channel samples: {total_samples:,}")
    print(f"Auto samples:    {auto_samples:,}")
    print(f"Auto time:       {auto_s:.1f}s  ({pct:.1f}%)")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Report what percentage of a match the robot spent in autonomous mode."
    )
    parser.add_argument("files", nargs="+", type=Path, help="MCAP file(s) to analyze")
    args = parser.parse_args()

    for path in args.files:
        if not path.exists():
            print(f"File not found: {path}", file=sys.stderr)
            sys.exit(1)
        analyze(path)
        print()


if __name__ == "__main__":
    main()
