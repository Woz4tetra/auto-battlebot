"""Command line for `scripts/verify`."""

from __future__ import annotations

import argparse
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

from auto_battlebot.verify import suites
from auto_battlebot.verify.selection import SUITES, Selection, everything, select
from auto_battlebot.verify.state import STATE_DIR

LATER_TIERS = {
    "playback": "phase 3 (WP-V10)",
    "realtime": "phase 3 (WP-V10)",
    "sim": "phase 3 (nav plan 3.5 smoke tier)",
    "hil": "phase 6 (WP-V15)",
}


def repo_root() -> Path:
    out = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        cwd=Path(__file__).resolve().parent,
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    return Path(out.strip())


def working_changes(root: Path) -> list[str]:
    """Modified, staged, and untracked paths in the working tree."""
    out = subprocess.run(
        ["git", "status", "--porcelain", "--untracked-files=all", "-z"],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    entries = out.split("\0")
    paths = []
    skip = False
    for entry in entries:
        if skip:
            skip = False
            continue
        if len(entry) < 4:
            continue
        paths.append(entry[3:])
        skip = entry[0] in "RC"
    return paths


def restrict(selection: Selection, only: list[str]) -> Selection:
    for name, scope in selection.scopes.items():
        if name not in only:
            scope.run = False
    return selection


def describe(selection: Selection) -> list[str]:
    lines = []
    for name, scope in selection.scopes.items():
        if not scope.run:
            continue
        detail = "all" if scope.everything else ", ".join(sorted(scope.items))
        lines.append(f"{name}: {detail}")
    return lines


@dataclass
class Outcome:
    """What a verify run did and whether it passed."""

    ok: bool
    results: list[suites.StepResult]
    selection: Selection

    def failures(self) -> list[suites.StepResult]:
        return [r for r in self.results if not r.ok]

    def summary(self) -> str:
        lines = [
            f"{'ok  ' if r.ok else 'FAIL'} {r.name} ({r.seconds:.0f} s) {r.log}"
            for r in self.results
        ]
        lines += [f"no test covers {path}" for path in self.selection.uncovered]
        return "\n".join(lines)


def verify(
    root: Path,
    files: list[str] | None,
    fast: bool,
    only: list[str] | None = None,
    log_dir: Path | None = None,
    echo: bool = False,
) -> Outcome:
    """Run lint (with --fast) and the suites a file list selects, or every suite."""
    selection = everything() if files is None else select(files, root)
    if only:
        selection = restrict(selection, only)
    before = [suites.lint_step(root)] if fast else []

    def report(result: suites.StepResult) -> None:
        if echo:
            mark = "ok  " if result.ok else "FAIL"
            print(f"{mark} {result.name} ({result.seconds:.0f} s)", flush=True)

    results = suites.run(
        suites.plan(root, selection, fast),
        log_dir or root / STATE_DIR / "verify-logs",
        before,
        report,
    )
    return Outcome(all(r.ok for r in results), results, selection)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="scripts/verify",
        description="Run every test suite, or the part of it a file list affects.",
    )
    parser.add_argument("files", nargs="*", help="Changed paths for --changed")
    parser.add_argument("--fast", action="store_true", help="Run lint --quick first")
    parser.add_argument(
        "--changed",
        action="store_true",
        help="Run only suites the file list affects (default list: git working changes)",
    )
    parser.add_argument("--files-from", type=Path, help="Read the file list from a file")
    parser.add_argument("--suite", action="append", choices=SUITES, help="Limit to these suites")
    parser.add_argument("--list", action="store_true", help="Print the selection and exit")
    parser.add_argument("--log-dir", type=Path, help="Where step logs go")
    for tier in LATER_TIERS:
        parser.add_argument(f"--{tier}", action="store_true", help=f"Lands in {LATER_TIERS[tier]}")
    return parser.parse_args(argv)


def file_list(args: argparse.Namespace, root: Path) -> list[str] | None:
    """The changed paths to select from, or None to run every suite."""
    explicit = bool(args.files or args.files_from)
    if not args.changed:
        if explicit:
            raise ValueError("a file list needs --changed")
        return None
    if not explicit:
        return working_changes(root)
    files = list(args.files)
    if args.files_from:
        files += args.files_from.read_text().splitlines()
    return [f.strip() for f in files if f.strip()]


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    for tier, phase in LATER_TIERS.items():
        if getattr(args, tier):
            print(f"--{tier} is not built yet; it lands in {phase}", file=sys.stderr)
            return 2
    root = repo_root()
    try:
        files = file_list(args, root)
    except ValueError as error:
        print(error, file=sys.stderr)
        return 2
    if args.list:
        selection = everything() if files is None else select(files, root)
        if args.suite:
            selection = restrict(selection, args.suite)
        print("\n".join(describe(selection)) or "nothing to run")
        for path in selection.uncovered:
            print(f"no test covers {path}")
        return 0
    outcome = verify(root, files, args.fast, args.suite, args.log_dir, echo=True)
    for failure in outcome.failures():
        print(f"\n==> {failure.name} failed; last lines of {failure.log}:")
        print(suites.tail(failure.log))
    for path in outcome.selection.uncovered:
        print(f"no test covers {path}")
    print("verify: " + ("PASS" if outcome.ok else "FAIL"))
    return 0 if outcome.ok else 1


if __name__ == "__main__":
    sys.exit(main())
