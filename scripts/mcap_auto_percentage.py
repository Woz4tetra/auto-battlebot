#!/usr/bin/env python3
"""
Report what percentage of a match recording the robot spent in autonomous mode.

The "match" is the period when the ZED SVO is actively recording.  Recording
starts when a /rosout INFO message contains "SVO recording started" and ends
when one contains "SVO recording stopped".  Auto mode is detected from the
/diagnostics topic: opentx_transmitter / channels / values/15 == "1024".

Diagnostics samples that fall outside any SVO recording window are ignored.

If a "started" message is missing (race condition at process startup), the
start time is inferred from the timestamp embedded in the SVO filename that
appears in the corresponding "stopped" message.
"""

import argparse
import re
import sys
from datetime import datetime
from pathlib import Path

from mcap_ros1.reader import read_ros1_messages

_SVO_START = "SVO recording started"
_SVO_STOP = "SVO recording stopped"
# Matches the ISO-ish timestamp in SVO filenames: 2026-04-30T00-00-50
_SVO_TS_RE = re.compile(r"(\d{4}-\d{2}-\d{2})T(\d{2})-(\d{2})-(\d{2})")


def _parse_svo_start_time(msg_text: str) -> datetime | None:
    """Extract the start datetime encoded in an SVO filename (e.g. ...T17-04-27.svo2)."""
    m = _SVO_TS_RE.search(msg_text)
    if not m:
        return None
    date_part, hh, mm, ss = m.groups()
    try:
        return datetime.fromisoformat(f"{date_part}T{hh}:{mm}:{ss}")
    except ValueError:
        return None


def _is_recording(ts: datetime, windows: list[tuple[datetime, datetime]]) -> bool:
    for start, stop in windows:
        if start <= ts <= stop:
            return True
    return False


def _build_windows(
    path: Path,
) -> tuple[list[tuple[datetime, datetime]], datetime | None]:
    """First pass: collect SVO recording windows from /rosout."""
    windows: list[tuple[datetime, datetime]] = []
    current_start: datetime | None = None

    for msg in read_ros1_messages(str(path), topics=["/rosout"]):
        text = msg.ros_msg.msg
        ts: datetime = msg.log_time

        if _SVO_START in text:
            current_start = ts
        elif _SVO_STOP in text:
            start = current_start
            if start is None:
                # "started" log message was lost; recover from the SVO filename
                start = _parse_svo_start_time(text)
            if start is not None:
                windows.append((start, ts))
            current_start = None

    # If recording is still open at end of file, treat end of file as stop
    # (use the last message's log_time — we just note it's open-ended)
    # We leave it open so diagnostics up to EOF are counted.
    return windows, current_start


def analyze(path: Path) -> None:
    try:
        windows, open_start = _build_windows(path)
    except Exception as e:
        print(f"  warning: error reading {path.name}: {e}", file=sys.stderr)
        return

    # Build a unified list including any still-open window (treated as unbounded)
    all_windows: list[tuple[datetime, datetime | None]] = [
        (s, e) for s, e in windows
    ]
    if open_start is not None:
        all_windows.append((open_start, None))

    if not all_windows:
        print(f"File: {path}")
        print("  no SVO recording windows found")
        return

    total_samples = 0
    auto_samples = 0
    first_counted_ts: float | None = None
    last_counted_ts: float | None = None

    try:
        for msg in read_ros1_messages(str(path), topics=["/diagnostics"]):
            diag = msg.ros_msg
            ts: datetime = msg.log_time

            for status in diag.status:
                if (
                    status.hardware_id != "opentx_transmitter"
                    or status.name != "channels"
                ):
                    continue

                in_window = any(
                    s <= ts and (e is None or ts <= e) for s, e in all_windows
                )
                if not in_window:
                    continue

                total_samples += 1
                ts_f = ts.timestamp()
                if first_counted_ts is None:
                    first_counted_ts = ts_f
                last_counted_ts = ts_f

                ch15 = next(
                    (kv.value for kv in status.values if kv.key == "values/15"),
                    None,
                )
                if ch15 is not None and ch15.strip() == "1024":
                    auto_samples += 1

    except Exception as e:
        print(f"  warning: error reading {path.name}: {e}", file=sys.stderr)

    if total_samples == 0:
        print(f"File: {path}")
        print("  no opentx_transmitter/channels diagnostics found during SVO recording")
        return

    pct = 100.0 * auto_samples / total_samples
    duration_s = (
        (last_counted_ts - first_counted_ts)
        if (first_counted_ts and last_counted_ts)
        else 0.0
    )
    auto_s = pct / 100.0 * duration_s

    print(f"File:             {path}")
    print(f"Recording windows:{len(all_windows):3d}")
    print(f"Match duration:   {duration_s:.1f}s")
    print(f"Channel samples:  {total_samples:,}")
    print(f"Auto samples:     {auto_samples:,}")
    print(f"Auto time:        {auto_s:.1f}s  ({pct:.1f}%)")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Report what percentage of a match the robot spent in autonomous mode. "
            "Only samples inside SVO recording windows are counted."
        )
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
