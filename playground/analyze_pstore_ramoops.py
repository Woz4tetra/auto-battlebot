#!/usr/bin/env python3
"""Analyze pstore/ramoops crash logs and summarize likely failure cause.

Examples:
  python3 playground/analyze_pstore_ramoops.py
  python3 playground/analyze_pstore_ramoops.py /sys/fs/pstore
  python3 playground/analyze_pstore_ramoops.py /var/lib/systemd/pstore
  python3 playground/analyze_pstore_ramoops.py /tmp/pstore_dump/*
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass
class Finding:
    category: str
    severity: int
    line_no: int
    line: str
    timestamp_s: float | None
    file: Path


SIGNATURES: list[tuple[str, int, re.Pattern[str]]] = [
    ("kernel_panic", 100, re.compile(r"kernel panic - not syncing", re.IGNORECASE)),
    ("hard_lockup", 95, re.compile(r"watchdog:\s+bug:\s+hard lockup", re.IGNORECASE)),
    ("soft_lockup", 90, re.compile(r"watchdog:\s+bug:\s+soft lockup", re.IGNORECASE)),
    ("rcu_stall", 88, re.compile(r"rcu.*stall", re.IGNORECASE)),
    ("hung_task", 85, re.compile(r"task .* blocked for more than .* seconds", re.IGNORECASE)),
    ("oops", 84, re.compile(r"\bOops:\b", re.IGNORECASE)),
    ("kernel_bug", 82, re.compile(r"\bBUG:\s+kernel\b", re.IGNORECASE)),
    ("general_protection_fault", 80, re.compile(r"general protection fault", re.IGNORECASE)),
    ("null_deref", 80, re.compile(r"unable to handle kernel (null )?pointer dereference", re.IGNORECASE)),
    ("oom_killer", 75, re.compile(r"out of memory|oom-killer|killed process .* out of memory", re.IGNORECASE)),
    ("machine_check", 72, re.compile(r"machine check|mce:", re.IGNORECASE)),
    ("tainted", 40, re.compile(r"tainted:", re.IGNORECASE)),
]

TS_RE = re.compile(r"\[\s*([0-9]+\.[0-9]+)\]")
CALL_TRACE_RE = re.compile(r"(?:\]|\s)([A-Za-z0-9_\.]+)\+0x[0-9a-fA-F]+")


def read_text(path: Path) -> str:
    try:
        return path.read_text(errors="replace")
    except Exception:
        return ""


def collect_inputs(paths: list[str]) -> list[Path]:
    if not paths:
        defaults = [Path("/sys/fs/pstore"), Path("/var/lib/systemd/pstore")]
        inputs: list[Path] = []
        for d in defaults:
            if d.is_dir():
                inputs.extend(sorted([p for p in d.iterdir() if p.is_file()]))
        return inputs

    out: list[Path] = []
    for raw in paths:
        p = Path(raw)
        if p.is_file():
            out.append(p)
        elif p.is_dir():
            out.extend(sorted([x for x in p.iterdir() if x.is_file()]))
    return out


def parse_timestamp(line: str) -> float | None:
    m = TS_RE.search(line)
    if not m:
        return None
    try:
        return float(m.group(1))
    except ValueError:
        return None


def analyze_file(path: Path) -> tuple[list[Finding], list[str]]:
    text = read_text(path)
    if not text:
        return [], []

    findings: list[Finding] = []
    lines = text.splitlines()

    for i, line in enumerate(lines, start=1):
        for category, severity, regex in SIGNATURES:
            if regex.search(line):
                findings.append(
                    Finding(
                        category=category,
                        severity=severity,
                        line_no=i,
                        line=line.strip(),
                        timestamp_s=parse_timestamp(line),
                        file=path,
                    )
                )

    # Collect likely call trace symbols near first "Call Trace:".
    funcs: list[str] = []
    for i, line in enumerate(lines):
        if "Call Trace:" not in line:
            continue
        for j in range(i + 1, min(i + 40, len(lines))):
            m = CALL_TRACE_RE.search(lines[j])
            if not m:
                continue
            name = m.group(1)
            if name not in funcs:
                funcs.append(name)
            if len(funcs) >= 10:
                break
        if funcs:
            break

    return findings, funcs


def category_title(category: str) -> str:
    return category.replace("_", " ").title()


def summarize(findings: list[Finding], traces: dict[Path, list[str]]) -> int:
    if not findings:
        print("No failure signatures found in provided pstore/ramoops files.")
        print("Check that files contain kernel logs (dmesg-ramoops, console-ramoops, etc).")
        return 1

    findings_sorted = sorted(findings, key=lambda f: (-f.severity, str(f.file), f.line_no))
    top = findings_sorted[0]

    print("Pstore/Ramoops Failure Summary")
    print("=" * 32)
    print(f"Likely failure type: {category_title(top.category)}")
    print(f"Confidence basis: matched highest-severity signature (score={top.severity})")
    print(f"Primary evidence file: {top.file}")
    if top.timestamp_s is not None:
        print(f"Approx kernel timestamp: {top.timestamp_s:.6f}s")
    print(f"Primary evidence line: {top.line}")

    per_file: dict[Path, list[Finding]] = {}
    for f in findings_sorted:
        per_file.setdefault(f.file, []).append(f)

    print("\nTop evidence by file")
    print("-" * 20)
    for path, items in per_file.items():
        best = items[0]
        print(f"{path.name}: {category_title(best.category)} (score={best.severity})")
        print(f"  line {best.line_no}: {best.line[:220]}")
        extra = [x for x in items[1:4]]
        for e in extra:
            print(f"  line {e.line_no}: {e.line[:220]}")
        trace = traces.get(path, [])
        if trace:
            print(f"  call_trace_top: {', '.join(trace[:6])}")

    # Global related signatures
    print("\nRelated signatures")
    print("-" * 18)
    seen: set[tuple[str, Path]] = set()
    for f in findings_sorted:
        key = (f.category, f.file)
        if key in seen:
            continue
        seen.add(key)
        stamp = f"{f.timestamp_s:.6f}s" if f.timestamp_s is not None else "n/a"
        print(f"- {category_title(f.category)} @ {f.file.name}:{f.line_no} (t={stamp})")
        if len(seen) >= 12:
            break

    return 0


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Analyze pstore/ramoops logs and summarize likely crash cause."
    )
    parser.add_argument(
        "paths",
        nargs="*",
        help="Files or directories to scan. Defaults: /sys/fs/pstore and /var/lib/systemd/pstore",
    )
    args = parser.parse_args()

    files = collect_inputs(args.paths)
    if not files:
        print("No pstore/ramoops files found.", file=sys.stderr)
        print("Pass a file/directory path explicitly, e.g. /sys/fs/pstore", file=sys.stderr)
        sys.exit(1)

    all_findings: list[Finding] = []
    traces: dict[Path, list[str]] = {}
    for p in files:
        findings, funcs = analyze_file(p)
        all_findings.extend(findings)
        if funcs:
            traces[p] = funcs

    rc = summarize(all_findings, traces)
    sys.exit(rc)


if __name__ == "__main__":
    main()
