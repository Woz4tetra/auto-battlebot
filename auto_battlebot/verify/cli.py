"""Command line for `scripts/verify`."""

from __future__ import annotations

import argparse
import subprocess
import sys
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

from auto_battlebot.verify import only, run_store, suites
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
    run_file: Path | None = None

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
    tier: str = "",
    store: bool = True,
    preselected: Selection | None = None,
    clips: Sequence[str] = (),
) -> Outcome:
    """Run lint (with --fast), the suites a file list selects or every suite, and playback tiers."""
    if preselected is not None:
        selection = preselected
    else:
        selection = everything() if files is None else select(files, root)
    if only:
        selection = restrict(selection, only)
    if not suites_too:
        selection = restrict(selection, [])
    before = [suites.lint_step(root)] if fast else []

    def report(result: suites.StepResult) -> None:
        if echo:
            mark = "ok  " if result.ok else "FAIL"
            print(f"{mark} {result.name} ({result.seconds:.0f} s)", flush=True)

    logs = log_dir or root / STATE_DIR / "verify-logs"
    groups = suites.plan(root, selection, fast, logs) + [
        suites.playback_group(root, name, logs, clips) for name in playback
    ]
    results = suites.run(groups, logs, before, report)
    run_file = store_run(root, results, tier or run_tier(files, fast, playback)) if store else None
    return Outcome(all(r.ok for r in results), results, selection, run_file)


def run_tier(files: list[str] | None, fast: bool, playback: tuple[str, ...]) -> str:
    parts = ["fast" if fast else "full", "changed" if files is not None else "all"]
    return "+".join([*parts, *playback])


def store_run(root: Path, results: list[suites.StepResult], tier: str) -> Path | None:
    """Write the run's per-proof results to the run store, if any step reported some."""
    proofs: list[run_store.ProofResult] = []
    for result in results:
        if result.report:
            proofs += run_store.read_report(result.report.kind, result.report.path, root)
    if not proofs:
        return None
    return run_store.write_run(root, tier, proofs)


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
    for tier in LATER_TIERS:
        parser.add_argument(f"--{tier}", action="store_true", help=f"Lands in {LATER_TIERS[tier]}")
    parser.add_argument(
        "--only", nargs="+", metavar="PROOF", help="Run exactly these proof ids (gtest:..., ...)"
    )
    parser.add_argument("--stale", action="store_true", help="Rerun every stale proof")
    parser.add_argument(
        "--coverage", action="store_true", help="Record which files each proof executes"
    )
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


def print_selection(selection: Selection, playback: tuple[str, ...], clips: Sequence[str]) -> int:
    which = ", ".join(clips) if clips else "every clip"
    lines = describe(selection) + [f"playback {tier}: {which}" for tier in playback]
    print("\n".join(lines) or "nothing to run")
    for path in selection.uncovered:
        print(f"no test covers {path}")
    return 0


def report_outcome(root: Path, outcome: Outcome) -> int:
    for failure in outcome.failures():
        print(f"\n==> {failure.name} failed; last lines of {failure.log}:")
        print(suites.tail(failure.log))
    for path in outcome.selection.uncovered:
        print(f"no test covers {path}")
    if outcome.run_file:
        print(f"results stored in {outcome.run_file.relative_to(root)}")
    print("verify: " + ("PASS" if outcome.ok else "FAIL"))
    return 0 if outcome.ok else 1


def stale_ids(root: Path) -> list[str]:
    from auto_battlebot.features.feature_map import build_map

    data = build_map(root, with_git=True)
    found = []
    for feature in data["features"]:
        entries = feature["proofs"] + feature["supporting"]
        entries += [e for variant in feature["variants"] for e in variant["proofs"]]
        found += [e["id"] for e in entries if e["state"] == "stale" and e["id"] not in found]
    return found


def run_only(root: Path, args: argparse.Namespace, ids: list[str]) -> int:
    plan = only.plan_only(ids)
    for problem in plan.problems:
        print(problem, file=sys.stderr)
    if args.list:
        return print_selection(plan.selection, plan.playback, plan.clips)
    if not ids:
        print("nothing is stale")
        return 0
    outcome = verify(
        root,
        None,
        args.fast,
        args.suite,
        args.log_dir,
        echo=True,
        playback=plan.playback,
        tier="only",
        preselected=plan.selection,
        clips=plan.clips,
    )
    code = report_outcome(root, outcome)
    return code or (2 if plan.problems else 0)


def run_coverage(root: Path, args: argparse.Namespace) -> int:
    from auto_battlebot.verify import coverage

    chosen = [s for s in (args.suite or []) if s in coverage.SUITES]
    path = coverage.run(chosen or list(coverage.SUITES))
    print(f"coverage written to {path.relative_to(root)}")
    return 0


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    for tier, phase in LATER_TIERS.items():
        if getattr(args, tier):
            print(f"--{tier} is not built yet; it lands in {phase}", file=sys.stderr)
            return 2
    root = repo_root()
    if args.coverage:
        return run_coverage(root, args)
    if args.only or args.stale:
        return run_only(root, args, list(args.only or []) + (stale_ids(root) if args.stale else []))
    try:
        files = file_list(args, root)
    except ValueError as error:
        print(error, file=sys.stderr)
        return 2
    playback = tuple(tier for flag, tier in PLAYBACK_TIERS.items() if getattr(args, flag))
    suites_too = not playback or bool(args.fast or args.changed or args.suite)
    if args.list:
        selection = everything() if files is None else select(files, root)
        if args.suite:
            selection = restrict(selection, args.suite)
        if not suites_too:
            selection = restrict(selection, [])
        return print_selection(selection, playback, ())
    outcome = verify(
        root,
        files,
        args.fast,
        args.suite,
        args.log_dir,
        echo=True,
        playback=playback,
        suites_too=suites_too,
    )
    return report_outcome(root, outcome)


if __name__ == "__main__":
    sys.exit(main())
