#!/usr/bin/env python3
"""Merge per-export `validation_state.json` files into one at the dataset root."""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path


def collect(root: Path) -> tuple[dict[str, str], int, int]:
    """Return (root-relative verdicts, files read, verdicts dropped for a missing image)."""
    merged: dict[str, str] = {}
    files = dropped = 0
    for state in sorted(root.rglob("validation_state.json")):
        if state.parent == root:
            continue
        try:
            verdicts = json.loads(state.read_text())
        except (OSError, ValueError):
            print(f"  skipped unreadable {state.relative_to(root)}")
            continue
        files += 1
        for key, verdict in verdicts.items():
            image = state.parent / key
            if not image.is_file():
                dropped += 1
                continue
            merged[str(image.relative_to(root))] = verdict
    return merged, files, dropped


def apply_existing(merged: dict[str, str], existing: dict[str, str]) -> int:
    """Overlay the existing root verdicts onto `merged`; returns how many changed a verdict."""
    overridden = sum(1 for k, v in existing.items() if merged.get(k) not in (None, v))
    merged.update(existing)
    return overridden


def main() -> None:
    """Merge every per-export validation state under the root into a root-level one."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("root", type=Path, help="Dataset root holding the per-export directories")
    parser.add_argument("--dry-run", action="store_true", help="Report only, write nothing")
    args = parser.parse_args()

    out = args.root / "validation_state.json"
    merged, files, dropped = collect(args.root)
    if not files:
        raise SystemExit(f"no per-export validation_state.json under {args.root}")

    counts = Counter(merged.values())
    print(f"{files} per-export state files -> {len(merged)} verdicts")
    for verdict, n in sorted(counts.items()):
        print(f"  {verdict:12s} {n:6d}")
    if dropped:
        print(f"  {dropped} verdict(s) dropped: image no longer on disk")

    if out.is_file():
        existing = json.loads(out.read_text())
        overridden = apply_existing(merged, existing)
        print(f"  {len(existing)} verdict(s) already in {out.name} kept ({overridden} differed)")

    if args.dry_run:
        print("\ndry run: nothing written")
        return

    tmp = out.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(merged, indent=2))
    tmp.replace(out)
    print(f"\nwrote {out} ({len(merged)} verdicts)")


if __name__ == "__main__":
    main()
