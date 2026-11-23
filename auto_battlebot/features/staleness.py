"""Proof state from the run store: passing, stale, failed, or never run."""

from __future__ import annotations

import json
import subprocess
from collections.abc import Iterable
from dataclasses import dataclass, field
from pathlib import Path

from auto_battlebot.verify.run_store import Run
from auto_battlebot.verify.state import UNTRACKED_PREFIXES

COVERAGE_DIR = Path("data/verify/coverage")
STATES = ("passing", "stale", "failed", "unrun")


@dataclass
class ProofState:
    """Where one proof stands: its last result, last passing commit, and flake count."""

    id: str
    state: str = "unrun"
    last_status: str | None = None
    last_seconds: float | None = None
    last_stamp: str | None = None
    last_pass_commit: str | None = None
    last_pass_stamp: str | None = None
    flakes: int = 0
    changed: list[str] = field(default_factory=list)


def history(runs: Iterable[Run]) -> dict[str, list[tuple[Run, str, float]]]:
    """Every result each proof id has, oldest first."""
    out: dict[str, list[tuple[Run, str, float]]] = {}
    for run in runs:
        for result in run.results:
            out.setdefault(result.id, []).append((run, result.status, result.seconds))
    return out


def flake_count(entries: list[tuple[Run, str, float]]) -> int:
    """Commits on which the proof both passed and failed."""
    by_commit: dict[tuple[str, str], set[str]] = {}
    for run, status, _ in entries:
        by_commit.setdefault((run.commit, run.diff_hash), set()).add(status)
    return sum(1 for statuses in by_commit.values() if {"passed", "failed"} <= statuses)


class ChangedFiles:
    """Paths that differ between a commit and the working tree, cached per commit."""

    def __init__(self, root: Path) -> None:
        self.root = root
        self._cache: dict[str, list[str] | None] = {}
        self._untracked: list[str] | None = None

    def untracked(self) -> list[str]:
        if self._untracked is None:
            out = subprocess.run(
                ["git", "ls-files", "--others", "--exclude-standard"],
                cwd=self.root,
                capture_output=True,
                text=True,
                check=False,
            ).stdout
            self._untracked = [p for p in out.splitlines() if p]
        return self._untracked

    def since(self, commit: str) -> list[str] | None:
        """Changed paths, or None when git does not know the commit."""
        if commit not in self._cache:
            result = subprocess.run(
                ["git", "diff", "--name-only", "--no-renames", commit],
                cwd=self.root,
                capture_output=True,
                text=True,
                check=False,
            )
            if result.returncode != 0:
                self._cache[commit] = None
            else:
                paths = set(result.stdout.splitlines()) | set(self.untracked())
                self._cache[commit] = sorted(
                    p for p in paths if p and not p.startswith(UNTRACKED_PREFIXES)
                )
        return self._cache[commit]


def matches(path: str, scope: Iterable[str]) -> bool:
    for entry in scope:
        if path == entry or (entry.endswith("/") and path.startswith(entry)):
            return True
    return False


def load_coverage(root: Path, commit: str | None = None) -> dict[str, list[str]]:
    """Proof id to the files it executed, from the newest coverage file (or one commit's)."""
    directory = root / COVERAGE_DIR
    if not directory.is_dir():
        return {}
    files = sorted(directory.glob("*.json"), key=lambda p: p.stat().st_mtime)
    if commit is not None:
        files = [p for p in files if p.stem == commit]
    if not files:
        return {}
    data = json.loads(files[-1].read_text())
    touches: dict[str, list[str]] = data.get("touches", {})
    return touches


def proof_states(
    runs: list[Run],
    scopes: dict[str, list[str]],
    changed: ChangedFiles,
    current: tuple[str, str],
) -> dict[str, ProofState]:
    """State per proof id: a pass goes stale when a file in its scope changed since that pass."""
    states = {}
    for proof, entries in history(runs).items():
        run, status, seconds = entries[-1]
        state = ProofState(proof, last_status=status, last_seconds=seconds, last_stamp=run.stamp)
        state.flakes = flake_count(entries)
        passes = [(r, s) for r, s, _ in entries if s == "passed"]
        if passes:
            last_pass = passes[-1][0]
            state.last_pass_commit = last_pass.commit
            state.last_pass_stamp = last_pass.stamp
        if status == "failed":
            state.state = "failed"
        elif status == "passed":
            state.state = "passing"
            if (run.commit, run.diff_hash) != current:
                since = changed.since(run.commit)
                scope = scopes.get(proof, [])
                if since is None:
                    state.state = "stale"
                else:
                    state.changed = [p for p in since if matches(p, scope)]
                    if state.changed:
                        state.state = "stale"
        states[proof] = state
    return states
