"""`scripts/verify --coverage`: record which files each proof executes, as its `touches` set."""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from pathlib import Path

from auto_battlebot.features import proofs
from auto_battlebot.paths import REPO_ROOT
from auto_battlebot.verify import gcda, playback, run_store
from auto_battlebot.verify.suites import FIRMWARE_DIR, build_jobs, memory_cap_prefix, pio_bin

COVERAGE_DIR = Path("data/verify/coverage")
BUILD_DIR = Path("build-coverage")
GTEST_BINARY = "auto_battlebot_test"
APP_BINARY = "auto_battlebot"
NO_TEST_FILTER = "NoSuchSuite.NoSuchCase"
CASE_TIMEOUT_S = 300.0
PYTHON_SOURCES = ("auto_battlebot", "scripts", "training", "simulation")
CONTEXT_PHASE = re.compile(r"\|(run|setup|teardown)$")
SUITES = ("gtest", "pytest", "firmware", "playback")


def configure_and_build(root: Path, target: str) -> Path:
    build = root / BUILD_DIR
    if not (build / "Makefile").exists():
        subprocess.run(
            [
                "cmake",
                "-S",
                str(root),
                "-B",
                str(build),
                "-DCMAKE_BUILD_TYPE=Debug",
                "-DBUILD_TESTING=ON",
                "-DBUILD_DEBUG=ON",
                "-DAUTO_BATTLEBOT_COVERAGE=ON",
            ],
            check=True,
        )
    subprocess.run(
        [*memory_cap_prefix(), "make", "-C", str(build), f"-j{build_jobs()}", target], check=True
    )
    return build / target


def gtest_cases(binary: Path) -> list[str]:
    """Full gtest names, instantiation prefixes and parameter suffixes included."""
    out = subprocess.run(
        [str(binary), "--gtest_list_tests"], capture_output=True, text=True, check=True
    ).stdout
    cases = []
    suite = ""
    for line in out.splitlines():
        if not line.strip():
            continue
        if not line.startswith(" "):
            suite = line.split("#", 1)[0].strip()
        else:
            cases.append(suite + line.split("#", 1)[0].strip())
    return cases


def run_prefixed(command: list[str], cwd: Path, prefix: Path) -> None:
    if prefix.exists():
        shutil.rmtree(prefix)
    prefix.mkdir(parents=True)
    env = {**os.environ, "GCOV_PREFIX": str(prefix)}
    subprocess.run(
        command,
        cwd=cwd,
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        timeout=CASE_TIMEOUT_S,
        check=False,
    )


def gtest_touches(
    root: Path, scratch: Path, index: gcda.FunctionIndex, limit: int | None
) -> dict[str, list[str]]:
    binary = configure_and_build(root, GTEST_BINARY)
    build = binary.parent
    run_prefixed([str(binary), f"--gtest_filter={NO_TEST_FILTER}"], build, scratch / "base")
    baseline = gcda.snapshot(scratch / "base")
    touches: dict[str, set[str]] = {}
    cases = gtest_cases(binary)[:limit] if limit else gtest_cases(binary)
    for i, case in enumerate(cases):
        run_prefixed([str(binary), f"--gtest_filter={case}"], build, scratch / "case")
        files = gcda.executed_files(gcda.snapshot(scratch / "case"), baseline, index)
        suite, _, name = case.partition(".")
        proof = run_store.gtest_case_id(suite, name)
        touches.setdefault(proof, set()).update(files)
        if (i + 1) % 100 == 0:
            print(f"gtest coverage: {i + 1}/{len(cases)}", flush=True)
            index.save()
    index.save()
    return {proof: sorted(files) for proof, files in touches.items()}


def context_proof(context: str) -> str | None:
    node = CONTEXT_PHASE.sub("", context)
    if "::" not in node:
        return None
    path, _, rest = node.partition("::")
    return f"pytest:{path}::{rest.split('[', 1)[0]}"


def pytest_touches(root: Path, scratch: Path) -> dict[str, list[str]]:
    import coverage

    data_file = scratch / ".coverage"
    command = [
        sys.executable,
        "-m",
        "pytest",
        "-q",
        "-p",
        "no:cacheprovider",
        "-m",
        "not slow",
        "-n",
        "8",
        *[f"--cov={source}" for source in PYTHON_SOURCES],
        "--cov-context=test",
        "--cov-report=",
    ]
    subprocess.run(
        command, cwd=root, env={**os.environ, "COVERAGE_FILE": str(data_file)}, check=False
    )
    data = coverage.CoverageData(basename=str(data_file))
    data.read()
    touches: dict[str, set[str]] = {}
    prefix = str(root.resolve()) + "/"
    for measured in data.measured_files():
        if not measured.startswith(prefix):
            continue
        rel = measured[len(prefix) :]
        for contexts in (data.contexts_by_lineno(measured) or {}).values():
            for context in contexts:
                proof = context_proof(context)
                if proof:
                    touches.setdefault(proof, set()).add(rel)
    return {proof: sorted(files) for proof, files in touches.items()}


def in_place_snapshot(build_dir: Path) -> dict[Path, gcda.FunctionBlocks]:
    return {g: gcda.read_blocks(g) for g in build_dir.rglob("*.gcda")}


def firmware_touches(root: Path, scratch: Path, index: gcda.FunctionIndex) -> dict[str, list[str]]:
    """Per test suite: every case in a suite shares the files the suite's program ran."""
    build_dir = scratch / "pio"
    cases_by_suite: dict[str, list[str]] = {}
    for proof in proofs.native_ids(root):
        suite, _, _ = proof.partition("::")
        cases_by_suite.setdefault(suite, []).append(proof)
    env = {
        **os.environ,
        "PLATFORMIO_BUILD_FLAGS": "--coverage",
        "PLATFORMIO_BUILD_DIR": str(build_dir),
    }
    touches: dict[str, list[str]] = {}
    for suite, cases in sorted(cases_by_suite.items()):
        for old in build_dir.rglob("*.gcda") if build_dir.exists() else []:
            old.unlink()
        subprocess.run(
            [pio_bin(root), "test", "-e", "native", "-f", suite],
            cwd=root / FIRMWARE_DIR,
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        files = sorted(gcda.executed_files(in_place_snapshot(build_dir), {}, index))
        for case in cases:
            touches[f"native:{case}"] = files
    index.save()
    return touches


@contextmanager
def gcov_prefix(prefix: Path) -> Iterator[None]:
    old = os.environ.get("GCOV_PREFIX")
    os.environ["GCOV_PREFIX"] = str(prefix)
    try:
        yield
    finally:
        if old is None:
            os.environ.pop("GCOV_PREFIX", None)
        else:
            os.environ["GCOV_PREFIX"] = old


def playback_touches(root: Path, scratch: Path, index: gcda.FunctionIndex) -> dict[str, list[str]]:
    """Each clip once through a coverage build of the app, in lockstep."""
    binary = configure_and_build(root, APP_BINARY)
    touches: dict[str, set[str]] = {}
    every: set[str] = set()
    for clip in playback.load_clips():
        prefix = scratch / f"clip_{clip.name}"
        if prefix.exists():
            shutil.rmtree(prefix)
        with gcov_prefix(prefix):
            playback.run_clip(clip, "lockstep", scratch / f"out_{clip.name}", binary)
        files = gcda.executed_files(gcda.snapshot(prefix), {}, index)
        every |= files
        for metric in proofs.PLAYBACK_METRICS:
            touches[f"playback:{clip.name}:{metric}"] = set(files)
    for proof in proofs.mcap_ids(root):
        touches[f"mcap:{proof}"] = every
    index.save()
    return {proof: sorted(files) for proof, files in touches.items()}


def write_coverage(
    root: Path, touches: dict[str, list[str]], seconds: dict[str, float], commit: str
) -> Path:
    out_dir = root / COVERAGE_DIR
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"{commit}.json"
    previous = json.loads(path.read_text()) if path.exists() else {}
    merged = {**previous.get("touches", {}), **touches}
    payload = {
        "commit": commit,
        "dirty": run_store.dirty_hash(root),
        "stamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "granularity": {"gtest": "case", "pytest": "case", "firmware": "suite", "playback": "clip"},
        "seconds": {**previous.get("seconds", {}), **seconds},
        "touches": dict(sorted(merged.items())),
    }
    path.write_text(json.dumps(payload, indent=1) + "\n")
    return path


def run(suites: list[str], limit: int | None = None, root: Path = REPO_ROOT) -> Path:
    index = gcda.FunctionIndex(root, root / COVERAGE_DIR / "gcno_index.json")
    touches: dict[str, list[str]] = {}
    seconds: dict[str, float] = {}
    builders: dict[str, Callable[[Path], dict[str, list[str]]]] = {
        "gtest": lambda scratch: gtest_touches(root, scratch, index, limit),
        "pytest": lambda scratch: pytest_touches(root, scratch),
        "firmware": lambda scratch: firmware_touches(root, scratch, index),
        "playback": lambda scratch: playback_touches(root, scratch, index),
    }
    with tempfile.TemporaryDirectory(prefix="verify_coverage_") as tmp:
        for suite in suites:
            start = time.monotonic()
            found = builders[suite](Path(tmp) / suite)
            seconds[suite] = time.monotonic() - start
            touches.update(found)
            print(f"{suite}: {len(found)} proofs in {seconds[suite]:.0f} s", flush=True)
    return write_coverage(root, touches, seconds, run_store.head_commit(root))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m auto_battlebot.verify.coverage")
    parser.add_argument("--suite", action="append", choices=SUITES, help="Default: all four")
    parser.add_argument("--limit", type=int, help="Only the first N gtest cases (for timing)")
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)
    path = run(args.suite or list(SUITES), args.limit)
    print(f"coverage written to {path.relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
