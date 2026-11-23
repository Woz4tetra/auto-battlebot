"""Per-proof results of every `scripts/verify` run, kept in `data/verify/runs/`."""

from __future__ import annotations

import json
import re
import time
import xml.etree.ElementTree as ElementTree
from collections.abc import Iterable
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from auto_battlebot.verify.state import changed_since, content_hash, git

RUNS_DIR = Path("data/verify/runs")
RUNS_KEPT = 500
STATUSES = ("passed", "failed", "skipped")
GTEST_INSTANCE = re.compile(r"^(?:\w+/)?(\w+)(?:/\d+)?$")
GTEST_PARAM_SUFFIX = re.compile(r"/\d+$")


@dataclass(frozen=True)
class ProofResult:
    """One proof's outcome in one run."""

    id: str
    status: str
    seconds: float


@dataclass
class Run:
    """A stored run: when, on which commit and dirty diff, which tier, and every proof result."""

    stamp: str
    commit: str
    diff_hash: str
    tier: str
    results: list[ProofResult]

    def by_id(self) -> dict[str, ProofResult]:
        return {r.id: r for r in self.results}


def gtest_case_id(suite: str, name: str) -> str:
    """`gtest:Suite.Name` with parameter instantiation prefixes and suffixes removed."""
    match = GTEST_INSTANCE.match(suite)
    base = match.group(1) if match else suite
    return f"gtest:{base}.{GTEST_PARAM_SUFFIX.sub('', name)}"


def seconds_of(value: Any) -> float:
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        return float(value.rstrip("s") or 0.0)
    return 0.0


def gtest_status(test: dict[str, Any]) -> str:
    if test.get("result") == "SKIPPED" or test.get("status") == "NOTRUN":
        return "skipped"
    return "failed" if test.get("failures") else "passed"


def merge(results: Iterable[ProofResult]) -> list[ProofResult]:
    """One result per id: any failure wins, then any pass; durations add."""
    merged: dict[str, ProofResult] = {}
    rank = {"failed": 2, "passed": 1, "skipped": 0}
    for result in results:
        old = merged.get(result.id)
        if old is None:
            merged[result.id] = result
            continue
        status = max(old.status, result.status, key=rank.__getitem__)
        merged[result.id] = ProofResult(result.id, status, old.seconds + result.seconds)
    return list(merged.values())


def gtest_results(path: Path, prefix: str = "gtest:") -> list[ProofResult]:
    """Results from a `--gtest_output=json` file."""
    data = json.loads(path.read_text())
    results = []
    for suite in data.get("testsuites", []):
        for test in suite.get("testsuite", []):
            case = gtest_case_id(test.get("classname", suite.get("name", "")), test["name"])
            proof = prefix + case.removeprefix("gtest:")
            results.append(ProofResult(proof, gtest_status(test), seconds_of(test.get("time"))))
    return merge(results)


def junit_case_id(case: ElementTree.Element, root: Path) -> str | None:
    file = case.get("file")
    classname = case.get("classname", "")
    name = case.get("name", "").split("[", 1)[0]
    parts = classname.split(".")
    if not file:
        for i in range(len(parts), 0, -1):
            candidate = Path(*parts[:i]).with_suffix(".py")
            if (root / candidate).is_file():
                file = str(candidate)
                parts = parts[i:]
                break
        else:
            return None
        return f"pytest:{file}::" + "::".join([*parts, name])
    module_parts = Path(file).with_suffix("").parts
    rest = parts[len(module_parts) :]
    return f"pytest:{file}::" + "::".join([*rest, name])


def junit_results(path: Path, root: Path) -> list[ProofResult]:
    """Results from a pytest `--junitxml` file."""
    results = []
    for case in ElementTree.parse(path).getroot().iter("testcase"):
        proof = junit_case_id(case, root)
        if proof is None:
            continue
        if case.find("skipped") is not None:
            status = "skipped"
        elif case.find("failure") is not None or case.find("error") is not None:
            status = "failed"
        else:
            status = "passed"
        results.append(ProofResult(proof, status, float(case.get("time", 0.0))))
    return merge(results)


def vitest_results(path: Path, root: Path) -> list[ProofResult]:
    """Results from a vitest `--reporter=json` file."""
    data = json.loads(path.read_text())
    results = []
    for suite in data.get("testResults", []):
        file = Path(suite["name"])
        rel = str(file.relative_to(root)) if file.is_absolute() else str(file)
        for test in suite.get("assertionResults", []):
            status = {"passed": "passed", "failed": "failed"}.get(test.get("status"), "skipped")
            seconds = float(test.get("duration") or 0.0) / 1000.0
            results.append(ProofResult(f"vitest:{rel}::{test['title']}", status, seconds))
    return merge(results)


def pio_results(path: Path) -> list[ProofResult]:
    """Results from `pio test --json-output-path`."""
    data = json.loads(path.read_text())
    results = []
    for suite in data.get("test_suites", []):
        suite_name = suite.get("test_name", "")
        for case in suite.get("test_cases", []):
            name = case.get("name", "")
            status = str(case.get("status", "")).lower()
            mapped = {"passed": "passed", "failed": "failed", "errored": "failed"}.get(
                status, "skipped"
            )
            proof = f"native:{suite_name}::{name}"
            results.append(ProofResult(proof, mapped, seconds_of(case.get("duration"))))
    return merge(results)


def dirty_hash(root: Path) -> str:
    """Content hash of every path that differs from HEAD, or empty on a clean tree."""
    changed = changed_since(root, "HEAD")
    return content_hash(root, changed) if changed else ""


def head_commit(root: Path) -> str:
    return git(root, "rev-parse", "--short=12", "HEAD").strip()


def utc_stamp(epoch_us: int) -> str:
    """Sortable UTC time with microseconds: 2026-11-22T14-17-03.123456Z."""
    seconds, micros = divmod(epoch_us, 1_000_000)
    return time.strftime("%Y-%m-%dT%H-%M-%S", time.gmtime(seconds)) + f".{micros:06d}Z"


def run_name(stamp: str, commit: str, diff_hash: str) -> str:
    return f"{stamp}_{commit}" + (f"-{diff_hash}" if diff_hash else "") + ".json"


def write_run(
    root: Path,
    tier: str,
    results: list[ProofResult],
    commit: str | None = None,
    diff_hash: str | None = None,
    keep: int = RUNS_KEPT,
) -> Path:
    """Store one run and prune the store to the newest `keep` runs."""
    for result in results:
        if result.status not in STATUSES:
            raise ValueError(f"{result.id}: status {result.status!r} not in {STATUSES}")
    runs_dir = root / RUNS_DIR
    runs_dir.mkdir(parents=True, exist_ok=True)
    now_us = time.time_ns() // 1000
    commit = commit if commit is not None else head_commit(root)
    diff_hash = diff_hash if diff_hash is not None else dirty_hash(root)
    while True:
        stamp = utc_stamp(now_us)
        path = runs_dir / run_name(stamp, commit, diff_hash)
        if not path.exists():
            break
        now_us += 1
    run = Run(stamp, commit, diff_hash, tier, sorted(results, key=lambda r: r.id))
    payload = {**asdict(run), "results": [asdict(r) for r in run.results]}
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(payload, indent=1) + "\n")
    tmp.rename(path)
    prune(runs_dir, keep)
    return path


def prune(runs_dir: Path, keep: int = RUNS_KEPT) -> None:
    runs = sorted(runs_dir.glob("*.json"))
    for old in runs[:-keep] if keep else runs:
        old.unlink()


def load_runs(root: Path) -> list[Run]:
    """Every stored run, oldest first."""
    runs = []
    for path in sorted((root / RUNS_DIR).glob("*.json")):
        try:
            data = json.loads(path.read_text())
        except (OSError, json.JSONDecodeError):
            continue
        results = [ProofResult(**r) for r in data.pop("results", [])]
        runs.append(Run(results=results, **data))
    return runs


def proof_results_file(path: Path) -> list[ProofResult]:
    """Results a playback, sim, HIL, or verify_mcap runner wrote as a JSON list."""
    return merge(ProofResult(**entry) for entry in json.loads(path.read_text()))


def write_proof_results(path: Path, results: Iterable[ProofResult]) -> None:
    path.write_text(json.dumps([asdict(r) for r in results], indent=1) + "\n")


def read_report(kind: str, path: Path, root: Path) -> list[ProofResult]:
    """Parse one step's report file by its kind; a missing or unreadable file gives nothing."""
    if not path.exists():
        return []
    try:
        if kind == "gtest":
            return gtest_results(path)
        if kind == "junit":
            return junit_results(path, root)
        if kind == "vitest":
            return vitest_results(path, root)
        if kind == "pio":
            return pio_results(path)
        if kind == "proofs":
            return proof_results_file(path)
    except (OSError, ValueError, KeyError, TypeError, ElementTree.ParseError):
        return []
    raise ValueError(f"unknown report kind {kind!r}")
