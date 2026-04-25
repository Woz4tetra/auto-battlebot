#!/usr/bin/env python3
"""Analyze syslog and extract possible failure events before reboot.

Designed for cases where a lockup required power cycling:
it detects reboot markers, then inspects the pre-reboot window for clues.

Example:
  python3 playground/analyze_syslog_reboot_failure.py /home/ben/Desktop/syslog_dump
"""

from __future__ import annotations

import argparse
import re
import statistics
from dataclasses import dataclass
from datetime import datetime, timedelta
from pathlib import Path


MONTHS = {
    "Jan": 1,
    "Feb": 2,
    "Mar": 3,
    "Apr": 4,
    "May": 5,
    "Jun": 6,
    "Jul": 7,
    "Aug": 8,
    "Sep": 9,
    "Oct": 10,
    "Nov": 11,
    "Dec": 12,
}

SYSLOG_TS_RE = re.compile(
    r"^(Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|Sep|Oct|Nov|Dec)\s+(\d{1,2})\s+"
    r"(\d{2}):(\d{2}):(\d{2})\s+"
)

BOOT_MARKERS: dict[str, list[re.Pattern[str]]] = {
    "booting_cpu": [
        re.compile(r"kernel: \[\s*0\.000000\]\s+Booting Linux on physical CPU", re.IGNORECASE),
    ],
    "linux_version": [
        re.compile(r"kernel: \[\s*0\.000000\]\s+Linux version", re.IGNORECASE),
    ],
    "both": [
        re.compile(r"kernel: \[\s*0\.000000\]\s+Booting Linux on physical CPU", re.IGNORECASE),
        re.compile(r"kernel: \[\s*0\.000000\]\s+Linux version", re.IGNORECASE),
    ],
}

FAILURE_SIGNATURES: list[tuple[str, int, re.Pattern[str]]] = [
    ("kernel_panic", 100, re.compile(r"kernel panic|not syncing", re.IGNORECASE)),
    ("hard_lockup", 95, re.compile(r"watchdog:.*hard lockup", re.IGNORECASE)),
    ("soft_lockup", 90, re.compile(r"watchdog:.*soft lockup", re.IGNORECASE)),
    ("rcu_stall", 88, re.compile(r"rcu.*stall", re.IGNORECASE)),
    ("hung_task", 85, re.compile(r"task .* blocked for more than .* seconds", re.IGNORECASE)),
    ("oops_bug", 82, re.compile(r"\bOops:|\bBUG:", re.IGNORECASE)),
    ("oom", 80, re.compile(r"out of memory|oom-killer|killed process .* out of memory", re.IGNORECASE)),
    ("nv_gpu_error", 70, re.compile(r"\bNVRM\b|Xid|gpu fault|gr3d", re.IGNORECASE)),
    ("usb_camera_error", 65, re.compile(r"usb .* reset|xusb|zed|camera.*(error|fail)", re.IGNORECASE)),
    ("app_error", 60, re.compile(r"auto_battlebot.*\[(error|fatal)\]", re.IGNORECASE)),
]

SHUTDOWN_CLEAN_PATTERNS = [
    re.compile(r"reboot: Restarting system", re.IGNORECASE),
    re.compile(r"Reached target (Reboot|Power-Off)", re.IGNORECASE),
    re.compile(r"systemd-shutdown", re.IGNORECASE),
]

SHUTDOWN_ACTIVITY_PATTERN = re.compile(
    r"systemd\[1\]:\s+(Stopping|Stopped|Removed slice|Reached target Shutdown|Shutting down)|"
    r"deactivated successfully",
    re.IGNORECASE,
)


@dataclass
class LogLine:
    idx: int
    raw: str
    dt: datetime | None
    ts_prefix: str | None = None


@dataclass
class Finding:
    category: str
    severity: int
    line: LogLine


@dataclass
class CycleSummary:
    cycle_idx: int
    boot_start_idx: int
    reboot_idx: int
    boot_start: LogLine
    reboot_marker: LogLine
    clean_shutdown: bool
    clean_reason: str
    shutdown_activity_count: int
    last_auto_battlebot: LogLine | None
    top_findings: list[Finding]


def parse_timestamp(line: str, year: int) -> datetime | None:
    m = SYSLOG_TS_RE.match(line)
    if not m:
        return None
    mon_s, day_s, hh_s, mm_s, ss_s = m.groups()
    return datetime(
        year=year,
        month=MONTHS[mon_s],
        day=int(day_s),
        hour=int(hh_s),
        minute=int(mm_s),
        second=int(ss_s),
    )


def extract_ts_prefix(line: str) -> str | None:
    m = SYSLOG_TS_RE.match(line)
    if not m:
        return None
    mon_s, day_s, hh_s, mm_s, ss_s = m.groups()
    return f"{mon_s} {int(day_s):02d} {hh_s}:{mm_s}:{ss_s}"


def load_lines(path: Path, base_year: int) -> list[LogLine]:
    lines: list[LogLine] = []
    current_year = base_year
    prev_dt: datetime | None = None

    with path.open("r", errors="replace") as f:
        for i, raw in enumerate(f, start=1):
            raw = raw.rstrip("\n")
            dt = parse_timestamp(raw, current_year)

            # Handle year rollover for multi-month dumps without year in syslog prefix.
            if dt is not None and prev_dt is not None and dt < prev_dt - timedelta(days=180):
                current_year += 1
                dt = parse_timestamp(raw, current_year)

            if dt is not None:
                prev_dt = dt

            lines.append(LogLine(idx=i, raw=raw, dt=dt))
            lines[-1].ts_prefix = extract_ts_prefix(raw)
    return lines


def is_reboot_marker(raw: str, markers: list[re.Pattern[str]]) -> bool:
    return any(rx.search(raw) for rx in markers)


def find_reboot_indices(lines: list[LogLine], markers: list[re.Pattern[str]]) -> list[int]:
    idxs: list[int] = []
    for i, ll in enumerate(lines):
        if is_reboot_marker(ll.raw, markers):
            idxs.append(i)

    # Deduplicate nearby markers in same boot event cluster.
    deduped: list[int] = []
    for i in idxs:
        if not deduped or i - deduped[-1] > 200:
            deduped.append(i)
    return deduped


def find_failures(lines: list[LogLine]) -> list[Finding]:
    out: list[Finding] = []
    for ll in lines:
        for category, severity, rx in FAILURE_SIGNATURES:
            if rx.search(ll.raw):
                out.append(Finding(category=category, severity=severity, line=ll))
                break
    out.sort(key=lambda f: (-f.severity, f.line.idx))
    return out


def find_clean_shutdown_reason(lines: list[LogLine]) -> tuple[bool, str, int]:
    for ll in lines:
        for rx in SHUTDOWN_CLEAN_PATTERNS:
            if rx.search(ll.raw):
                return True, f"explicit marker at line {ll.idx}", 0

    activity_count = sum(1 for ll in lines if SHUTDOWN_ACTIVITY_PATTERN.search(ll.raw))
    if activity_count >= 12:
        return True, f"shutdown activity burst ({activity_count} lines)", activity_count

    return False, "no shutdown markers detected", activity_count


def summarize_cycle(lines: list[LogLine], reboot_idxs: list[int], cycle_idx: int,
                    window_lines: int) -> CycleSummary:
    start = reboot_idxs[cycle_idx]
    end = reboot_idxs[cycle_idx + 1]
    cycle_lines = lines[start:end]
    if not cycle_lines:
        return CycleSummary(
            cycle_idx=cycle_idx,
            boot_start_idx=start,
            reboot_idx=end,
            boot_start=lines[start],
            reboot_marker=lines[end],
            clean_shutdown=False,
            clean_reason="empty cycle window",
            shutdown_activity_count=0,
            last_auto_battlebot=None,
            top_findings=[],
        )

    tail = cycle_lines[-min(len(cycle_lines), 5000):]
    clean, reason, activity_count = find_clean_shutdown_reason(tail)

    pre_reboot_window_start = max(start, end - window_lines)
    pre_reboot_window = lines[pre_reboot_window_start:end]
    top_findings = find_failures(pre_reboot_window)[:12]

    bot_lines = [ll for ll in pre_reboot_window if "auto_battlebot[" in ll.raw]
    last_bot = bot_lines[-1] if bot_lines else None

    return CycleSummary(
        cycle_idx=cycle_idx,
        boot_start_idx=start,
        reboot_idx=end,
        boot_start=lines[start],
        reboot_marker=lines[end],
        clean_shutdown=clean,
        clean_reason=reason,
        shutdown_activity_count=activity_count,
        last_auto_battlebot=last_bot,
        top_findings=top_findings,
    )


def print_cycle_summary(cycle: CycleSummary, *, show_findings: bool) -> None:
    print("\nCycle")
    print("-" * 5)
    print(
        f"cycle_index={cycle.cycle_idx} boot_start_line={cycle.boot_start.idx} "
        f"reboot_line={cycle.reboot_marker.idx}"
    )
    print(
        f"boot_start_ts={cycle.boot_start.ts_prefix or 'n/a'} "
        f"reboot_ts={cycle.reboot_marker.ts_prefix or 'n/a'}"
    )
    print(
        f"clean_shutdown={'yes' if cycle.clean_shutdown else 'no'} "
        f"reason={cycle.clean_reason}"
    )
    if cycle.last_auto_battlebot:
        print(
            f"last_auto_battlebot_line={cycle.last_auto_battlebot.idx} "
            f"ts={cycle.last_auto_battlebot.ts_prefix or 'n/a'}"
        )
    else:
        print("last_auto_battlebot_line=none")

    if show_findings:
        if cycle.top_findings:
            print("top_pre_reboot_findings:")
            for f in cycle.top_findings:
                print(
                    f"  - [{f.category}] score={f.severity} line={f.line.idx} "
                    f"ts={f.line.ts_prefix or 'n/a'}"
                )
                print(f"    {f.line.raw[:220]}")
        else:
            print("top_pre_reboot_findings: none")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Parse syslog dump and extract likely pre-reboot failure events."
    )
    parser.add_argument("file", type=Path, help="Path to syslog dump file")
    parser.add_argument(
        "--year",
        type=int,
        default=2025,
        help="Base year for syslog lines (default: 2025)",
    )
    parser.add_argument(
        "--window-lines",
        type=int,
        default=20000,
        help="How many lines before reboot marker to analyze (default: 20000)",
    )
    parser.add_argument(
        "--event-index",
        type=int,
        default=None,
        help=(
            "Optional cycle index to inspect (0 means between marker[0] and marker[1]). "
            "If omitted, script prints all cycles that do NOT end cleanly."
        ),
    )
    parser.add_argument(
        "--boot-marker",
        choices=["booting_cpu", "linux_version", "both"],
        default="booting_cpu",
        help=(
            "Reboot marker type (default: booting_cpu). "
            "'booting_cpu' matches 'Booting Linux on physical CPU...'"
        ),
    )
    args = parser.parse_args()

    if not args.file.exists():
        raise SystemExit(f"File not found: {args.file}")

    lines = load_lines(args.file, args.year)
    if not lines:
        raise SystemExit("No lines found in file.")

    markers = BOOT_MARKERS[args.boot_marker]
    reboot_idxs = find_reboot_indices(lines, markers)
    if not reboot_idxs:
        raise SystemExit(f"No reboot markers found for --boot-marker={args.boot_marker}.")
    if len(reboot_idxs) < 2:
        raise SystemExit("Need at least 2 reboot markers to analyze boot cycles.")

    print(f"Loaded {len(lines):,} lines from {args.file}")
    print(f"Detected {len(reboot_idxs)} reboot marker(s) using '{args.boot_marker}'")
    for i, idx in enumerate(reboot_idxs):
        ll = lines[idx]
        ts = ll.ts_prefix or "n/a"
        print(f"  [{i}] line={ll.idx} ts={ts} :: {ll.raw[:120]}")

    cycle_count = len(reboot_idxs) - 1
    cycles = [
        summarize_cycle(lines, reboot_idxs, i, args.window_lines) for i in range(cycle_count)
    ]

    if args.event_index is not None:
        if args.event_index < 0 or args.event_index >= cycle_count:
            raise SystemExit(f"Invalid --event-index {args.event_index}; valid range: 0..{cycle_count - 1}")
        print_cycle_summary(cycles[args.event_index], show_findings=True)
        return

    unclean_cycles = [c for c in cycles if not c.clean_shutdown]
    print(f"\nUnclean cycles: {len(unclean_cycles)} / {cycle_count}")
    if not unclean_cycles:
        print("All detected cycles appear to end with clean shutdown markers.")
        return

    for c in unclean_cycles:
        print_cycle_summary(c, show_findings=True)


if __name__ == "__main__":
    main()
