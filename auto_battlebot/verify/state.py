"""Per-session agent state under .claude/state: touched files, change hash, results, verdicts."""

from __future__ import annotations

import hashlib
import json
import re
import subprocess
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

STATE_DIR = Path(".claude/state")
UNTRACKED_PREFIXES = (".claude/state/", ".claude/worktrees/", ".git/")
VERDICT_RESULTS = ("PASS", "FAIL")
CODE_PREFIXES = ("src/", "include/", "firmware/", "auto_battlebot/", "tests/")


def safe_id(value: str) -> str:
    """A session id or hash reduced to characters safe in a file name."""
    cleaned = re.sub(r"[^A-Za-z0-9_-]", "_", value)
    if not cleaned:
        raise ValueError("empty id")
    return cleaned


def state_dir(root: Path) -> Path:
    path = root / STATE_DIR
    path.mkdir(parents=True, exist_ok=True)
    return path


def repo_relative(root: Path, path: str, cwd: Path | None = None) -> str | None:
    """A path as repo-relative text, or None when it is outside the repo or in agent state."""
    candidate = Path(path)
    if not candidate.is_absolute():
        candidate = (cwd or root) / candidate
    try:
        relative = candidate.resolve().relative_to(root.resolve())
    except ValueError:
        return None
    text = relative.as_posix()
    if text in ("", ".") or text.startswith(UNTRACKED_PREFIXES):
        return None
    return text


def git(root: Path, *args: str) -> str:
    return subprocess.run(
        ["git", *args], cwd=root, check=True, capture_output=True, text=True
    ).stdout


def touched_file(root: Path, session: str) -> Path:
    return state_dir(root) / f"touched-{safe_id(session)}.txt"


def base_file(root: Path, session: str) -> Path:
    return state_dir(root) / f"base-{safe_id(session)}.txt"


def record_touch(root: Path, session: str, path: str, cwd: Path | None = None) -> str | None:
    """Append a path to the session's touched list, recording the base commit on first touch."""
    relative = repo_relative(root, path, cwd)
    if relative is None:
        return None
    base = base_file(root, session)
    if not base.exists():
        base.write_text(git(root, "rev-parse", "HEAD").strip() + "\n")
    with touched_file(root, session).open("a") as log:
        log.write(relative + "\n")
    return relative


def read_touched(root: Path, session: str) -> list[str]:
    path = touched_file(root, session)
    if not path.exists():
        return []
    return sorted({line.strip() for line in path.read_text().splitlines() if line.strip()})


def read_base(root: Path, session: str) -> str | None:
    path = base_file(root, session)
    return path.read_text().strip() if path.exists() else None


def content_hash(root: Path, paths: list[str]) -> str:
    """Hash of each touched path and its current bytes; a deleted file hashes as deleted."""
    digest = hashlib.sha256()
    for path in sorted(paths):
        digest.update(path.encode() + b"\0")
        target = root / path
        digest.update(target.read_bytes() if target.is_file() else b"<deleted>")
        digest.update(b"\0")
    return digest.hexdigest()[:16]


def touches_code(paths: list[str]) -> bool:
    return any(path.startswith(CODE_PREFIXES) for path in paths)


def change_diff(root: Path, base: str, paths: list[str]) -> str:
    """The diff of the touched paths from the session's base commit to the working tree."""
    tracked = [p for p in paths if _in_commit(root, base, p)]
    text = git(root, "diff", base, "--", *tracked) if tracked else ""
    for path in paths:
        if path in tracked or not (root / path).is_file():
            continue
        result = subprocess.run(
            ["git", "diff", "--no-index", "--", "/dev/null", path],
            cwd=root,
            capture_output=True,
            text=True,
        )
        text += result.stdout
    return text


def _in_commit(root: Path, commit: str, path: str) -> bool:
    result = subprocess.run(
        ["git", "cat-file", "-e", f"{commit}:{path}"], cwd=root, capture_output=True
    )
    return result.returncode == 0


@dataclass
class StopRecord:
    """The last stop-hook outcome for one change hash, so a repeat block is capped."""

    hash: str = ""
    kind: str = ""
    blocks: int = 0
    reason: str = ""


def read_stop(root: Path, session: str) -> StopRecord:
    path = state_dir(root) / f"stop-{safe_id(session)}.json"
    if not path.exists():
        return StopRecord()
    return StopRecord(**json.loads(path.read_text()))


def write_stop(root: Path, session: str, record: StopRecord) -> None:
    path = state_dir(root) / f"stop-{safe_id(session)}.json"
    path.write_text(json.dumps(asdict(record)) + "\n")


def pass_marker(root: Path, change: str) -> Path:
    return state_dir(root) / f"verify-pass-{safe_id(change)}"


@dataclass
class Verdict:
    """A verifier verdict on one change hash."""

    hash: str
    result: str
    findings: list[Any] = field(default_factory=list)


def verdict_file(root: Path, change: str) -> Path:
    return state_dir(root) / f"verdict-{safe_id(change)}.json"


def write_verdict(root: Path, verdict: Verdict) -> Path:
    if verdict.result not in VERDICT_RESULTS:
        raise ValueError(f"verdict result must be one of {VERDICT_RESULTS}, got {verdict.result}")
    path = verdict_file(root, verdict.hash)
    path.write_text(json.dumps(asdict(verdict), indent=2) + "\n")
    return path


def read_verdict(root: Path, change: str) -> Verdict | None:
    path = verdict_file(root, change)
    if not path.exists():
        return None
    return Verdict(**json.loads(path.read_text()))


def finding_key(finding: Any) -> str:
    if isinstance(finding, dict):
        return f"{finding.get('file', '')}:{finding.get('check', '')}"
    return str(finding)


def finding_counts_file(root: Path, session: str) -> Path:
    return state_dir(root) / f"findings-{safe_id(session)}.json"


def read_finding_counts(root: Path, session: str) -> dict[str, int]:
    path = finding_counts_file(root, session)
    return json.loads(path.read_text()) if path.exists() else {}


def count_findings(root: Path, session: str, findings: list[Any]) -> dict[str, int]:
    """Add one FAIL verdict's findings to the session's per-finding FAIL counts."""
    counts = read_finding_counts(root, session)
    for key in {finding_key(f) for f in findings}:
        counts[key] = counts.get(key, 0) + 1
    finding_counts_file(root, session).write_text(json.dumps(counts, indent=2) + "\n")
    return counts
