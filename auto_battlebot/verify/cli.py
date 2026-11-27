"""Command line for `scripts/verify`."""

from __future__ import annotations

import argparse
import subprocess
import sys
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path

from auto_battlebot.verify import suites
from auto_battlebot.verify.selection import SUITES, Selection, everything, select
from auto_battlebot.verify.state import STATE_DIR

PLAYBACK_TIERS = {"playback": "lockstep", "realtime": "realtime"}
LATER_TIERS = {
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


def restrict(selection: Selection, only: Iterable[str]) -> Selection:
    keep = set(only)
    for name, scope in selection.scopes.items():
        if name not in keep:
            scope.run = False
    return selection


def nightly_tiers(target: str | None, nightly: bool) -> tuple[str, ...]:
    """Playback tiers a nightly adds: real time on the Jetson only."""
    return ("realtime",) if nightly and target == "jetson" else ()


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
    playback: tuple[str, ...] = (),
    suites_too: bool = True,
    target: str | None = None,
    nightly: bool = False,
) -> Outcome:
    """Run lint (with --fast), the suites a file list selects or every suite, and playback tiers."""
    selection = everything() if files is None else select(files, root)
    if only:
        selection = restrict(selection, only)
    if target is not None:
        selection = restrict(selection, suites.TARGET_SUITES[target])
    if not suites_too:
        selection = restrict(selection, [])
    lint = suites.lint_step(root, target, nightly) if fast or nightly else None
    before = [lint] if lint else []
    gpu_ok = True
    if nightly and target == "x86":
        gpu_ok = suites.gpu_idle()
        if not gpu_ok:
            print("gpu tests deferred: the GPU is busy", flush=True)

    def report(result: suites.StepResult) -> None:
        if echo:
            mark = "ok  " if result.ok else "FAIL"
            print(f"{mark} {result.name} ({result.seconds:.0f} s)", flush=True)

    tiers = playback + tuple(t for t in nightly_tiers(target, nightly) if t not in playback)
    groups = suites.plan(root, selection, fast, target, nightly, gpu_ok) + [
        suites.playback_group(root, tier) for tier in tiers
    ]
    results = suites.run(
        groups,
        log_dir or root / STATE_DIR / "verify-logs",
        before,
        report,
        parallel=target != "jetson",
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
    parser.add_argument(
        "--playback", action="store_true", help="Replay the lockstep clips against their baseline"
    )
    parser.add_argument(
        "--realtime", action="store_true", help="Replay the clips in real time against A/A bounds"
    )
    parser.add_argument(
        "--target",
        choices=suites.TARGETS,
        help="CI box: run only its suites, lint check-only (default: every suite)",
    )
    parser.add_argument(
        "--nightly",
        action="store_true",
        help="With --target, add slow and gpu tests, real time, release and ASan builds",
    )
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


def print_selection(
    args: argparse.Namespace,
    root: Path,
    files: list[str] | None,
    playback: tuple[str, ...],
    suites_too: bool,
) -> None:
    selection = everything() if files is None else select(files, root)
    if args.suite:
        selection = restrict(selection, args.suite)
    if args.target:
        selection = restrict(selection, suites.TARGET_SUITES[args.target])
    if not suites_too:
        selection = restrict(selection, [])
    tiers = playback + nightly_tiers(args.target, args.nightly)
    lines = describe(selection) + [f"playback {tier}: every clip" for tier in tiers]
    print("\n".join(lines) or "nothing to run")
    for path in selection.uncovered:
        print(f"no test covers {path}")


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
    if args.nightly and args.target is None:
        print("--nightly needs --target", file=sys.stderr)
        return 2
    playback = tuple(tier for flag, tier in PLAYBACK_TIERS.items() if getattr(args, flag))
    suites_too = not playback or bool(
        args.fast or args.changed or args.suite or args.target or args.nightly
    )
    if args.list:
        print_selection(args, root, files, playback, suites_too)
        return 0
    outcome = verify(
        root,
        files,
        args.fast,
        args.suite,
        args.log_dir,
        echo=True,
        playback=playback,
        suites_too=suites_too,
        target=args.target,
        nightly=args.nightly,
    )
    for failure in outcome.failures():
        print(f"\n==> {failure.name} failed; last lines of {failure.log}:")
        print(suites.tail(failure.log))
    for path in outcome.selection.uncovered:
        print(f"no test covers {path}")
    print("verify: " + ("PASS" if outcome.ok else "FAIL"))
    return 0 if outcome.ok else 1


if __name__ == "__main__":
    sys.exit(main())
