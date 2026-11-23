"""Command line for the feature registry: check, draft, reach, and map."""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter

from auto_battlebot.features import check, draft, reach, registry
from auto_battlebot.paths import REPO_ROOT


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="python -m auto_battlebot.features")
    sub = parser.add_subparsers(dest="action", required=True)
    check_parser = sub.add_parser("check", help="Lint the registry against the tree")
    check_parser.add_argument("--quiet", action="store_true", help="Print only the summary line")
    sub.add_parser("draft", help=f"Append a stub to {registry.FEATURES_FILE} per unclaimed point")
    reach_parser = sub.add_parser("reach", help="List every discovered reach point")
    reach_parser.add_argument("--json", action="store_true")
    map_parser = sub.add_parser("map", help="Print the joined feature map as JSON")
    map_parser.add_argument("--no-git", action="store_true", help="Skip commit history")
    map_parser.add_argument(
        "--with-selection", action="store_true", help="Add what `--changed` would run"
    )
    return parser.parse_args(argv)


def run_check(quiet: bool) -> int:
    report = check.check(REPO_ROOT, registry.load(REPO_ROOT))
    if not quiet:
        for line in report.lines():
            print(line)
    kinds = Counter(p.kind for p in report.unclaimed)
    mode = "enforced" if report.enforce else "reporting gaps only"
    print(
        f"features: {len(report.errors)} errors, {len(report.gaps)} gaps "
        f"({len(report.unclaimed)} unclaimed reach points: "
        f"{', '.join(f'{k} {n}' for k, n in sorted(kinds.items())) or 'none'}); {mode}"
    )
    return 1 if report.failed() else 0


def run_draft() -> int:
    current = registry.load(REPO_ROOT)
    if current.errors:
        print("\n".join(current.errors), file=sys.stderr)
        return 1
    report = check.check(REPO_ROOT, current)
    count = draft.draft(REPO_ROOT, current, report.unclaimed)
    print(f"appended {count} draft entries to {registry.FEATURES_FILE}")
    return 0


def run_reach(as_json: bool) -> int:
    points = reach.discover(REPO_ROOT)
    if as_json:
        print(json.dumps([p.__dict__ for p in points], indent=2))
    else:
        for p in points:
            print(f"{p.id}\t{p.path}:{p.line}")
    return 0


def run_map(with_git: bool, with_selection: bool) -> int:
    from auto_battlebot.features.feature_map import build_map

    data = build_map(REPO_ROOT, with_git=with_git, with_selection=with_selection)
    print(json.dumps(data, indent=2, sort_keys=True))
    return 0


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.action == "check":
        return run_check(args.quiet)
    if args.action == "draft":
        return run_draft()
    if args.action == "reach":
        return run_reach(args.json)
    return run_map(not args.no_git, args.with_selection)


if __name__ == "__main__":
    sys.exit(main())
