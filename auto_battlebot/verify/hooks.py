"""Claude Code hook entry points: touched-files log, stop gate, verifier verdicts."""

from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
from collections.abc import Callable
from pathlib import Path
from typing import Any

from auto_battlebot.verify import state
from auto_battlebot.verify.cli import Outcome, repo_root, verify

VERIFIER_AGENT = "verifier"
MAX_BLOCKS_PER_CHANGE = 2
MAX_FAILS_PER_FINDING = 2
VERDICT_PREFIX = "VERDICT "
HANDBACK_TOOL = "SubagentHandback"
TEST_DIRS = {"tests", "test"}
DETACHING = re.compile(r"\b(nohup|setsid|disown|sleep)\b|(?<![&>|<])&(?![&>])")
CPP_TEST_SUFFIXES = {".cpp", ".cc", ".c", ".hpp", ".h"}
MUTANT_DIR = Path(".claude/worktrees")
ISOLATION_PLUGIN = "mutant_isolation"
ISOLATION_SOURCE = """import sys

sys.meta_path[:] = [f for f in sys.meta_path if "__editable__" not in getattr(f, "__module__", "")]
for _name in [n for n in sys.modules if n.split(".")[0] in ("auto_battlebot",)]:
    del sys.modules[_name]
"""
UNTRACKED_INPUTS = ("data", "third_party", "web/node_modules")

Verifier = Callable[[Path, list[str]], Outcome]


def run_fast(root: Path, files: list[str]) -> Outcome:
    """The fast suites over files, logged under a directory of their own per change."""
    log_dir = state.state_dir(root) / "verify-logs" / state.content_hash(root, files)
    log_dir.mkdir(parents=True, exist_ok=True)
    return verify(root, files, fast=True, log_dir=log_dir)


def read_input(stream: Any = None) -> dict[str, Any]:
    text = (stream or sys.stdin).read()
    return json.loads(text) if text.strip() else {}


def session_start(event: dict[str, Any], root: Path) -> int:
    """SessionStart: record HEAD as the base the session's change is diffed against."""
    session = event.get("session_id")
    if session and event.get("agent_type") != VERIFIER_AGENT:
        state.record_base(root, session)
    return 0


def touch(event: dict[str, Any], root: Path) -> int:
    """PostToolUse on Write/Edit: record a base commit for a session that started without one."""
    if event.get("agent_type") == VERIFIER_AGENT:
        return 0
    tool_input = event.get("tool_input") or {}
    path = tool_input.get("file_path") or tool_input.get("notebook_path")
    session = event.get("session_id")
    if not path or not session:
        return 0
    cwd = Path(event["cwd"]) if event.get("cwd") else None
    state.record_touch(root, session, path, cwd)
    return 0


def bash_guard(event: dict[str, Any]) -> tuple[int, str]:
    """PreToolUse on Bash: the verifier runs every command in the foreground, once."""
    if event.get("agent_type") != VERIFIER_AGENT:
        return 0, ""
    tool_input = event.get("tool_input") or {}
    command = str(tool_input.get("command", ""))
    if tool_input.get("run_in_background") or DETACHING.search(command):
        return 2, (
            "verifier: no run_in_background, nohup, setsid, disown, sleep or trailing &. "
            "Run the command in the foreground with the Bash timeout set to 600000."
        )
    return 0, ""


def verifier_request(session: str, change: str) -> str:
    return (
        f"Code changed (hash {change}) and has no verifier verdict. Spawn the `{VERIFIER_AGENT}` "
        f"agent with session id {session}, change hash {change}, and the user's original request "
        "quoted verbatim. Do not summarize your change for it; it reads the diff itself."
    )


def block(root: Path, session: str, change: str, kind: str, reason: str) -> tuple[int, str]:
    """Block the stop, unless this change already blocked twice for the same reason."""
    record = state.read_stop(root, session)
    blocks = record.blocks + 1 if (record.hash, record.kind) == (change, kind) else 1
    state.write_stop(root, session, state.StopRecord(change, kind, blocks, reason))
    if blocks > MAX_BLOCKS_PER_CHANGE:
        return 0, (
            f"verify gate: {kind} still failing for change {change} after "
            f"{MAX_BLOCKS_PER_CHANGE} blocks; stopping so the user can decide.\n{reason}"
        )
    return 2, reason


def subagents_dir(event: dict[str, Any]) -> Path | None:
    """Where this session's subagent transcripts live, next to the session transcript."""
    transcript = event.get("transcript_path")
    if not transcript:
        return None
    path = Path(transcript).with_suffix("") / "subagents"
    return path if path.is_dir() else None


def agent_meta(directory: Path, agent_id: str) -> dict[str, Any]:
    try:
        meta = json.loads((directory / f"agent-{agent_id}.meta.json").read_text())
    except (OSError, json.JSONDecodeError):
        return {}
    return meta if isinstance(meta, dict) else {}


def running_verifiers(event: dict[str, Any]) -> list[str]:
    """Ids of background verifier subagents the Stop event lists as running."""
    directory = subagents_dir(event)
    if directory is None:
        return []
    return [
        str(task.get("id"))
        for task in event.get("background_tasks") or []
        if task.get("type") == "subagent"
        and task.get("status") == "running"
        and agent_meta(directory, str(task.get("id"))).get("agentType") == VERIFIER_AGENT
    ]


def collect_verdict(
    root: Path, session: str, change: str, event: dict[str, Any]
) -> state.Verdict | None:
    """Record a finished verifier's verdict on this change from its transcript."""
    directory = subagents_dir(event)
    if directory is None:
        return None
    found = None
    for meta_path in sorted(directory.glob("agent-*.meta.json"), key=lambda p: p.stat().st_mtime):
        agent_id = meta_path.name.removeprefix("agent-").removesuffix(".meta.json")
        meta = agent_meta(directory, agent_id)
        if meta.get("agentType") != VERIFIER_AGENT or not meta.get("finished"):
            continue
        report = {"agent_transcript_path": str(directory / f"agent-{agent_id}.jsonl")}
        parsed = parse_verdict(final_report(report))
        if parsed is None or parsed.get("hash") != change:
            continue
        found = state.Verdict(change, str(parsed.get("result")), parsed.get("findings") or [])
    if found is None or found.result not in state.VERDICT_RESULTS:
        return None
    state.write_verdict(root, found)
    if found.result == "FAIL":
        state.count_findings(root, session, found.findings)
    return found


def stop(event: dict[str, Any], root: Path, runner: Verifier = run_fast) -> tuple[int, str]:
    """Stop hook: run affected suites over touched files, then require a verifier verdict."""
    session = event.get("session_id")
    if not session:
        return 0, ""
    touched = state.read_touched(root, session)
    if not touched:
        return 0, ""
    change = state.content_hash(root, touched)
    if not state.pass_marker(root, change).exists():
        outcome = runner(root, touched)
        change = state.content_hash(root, touched)
        if not outcome.ok:
            return block(
                root,
                session,
                change,
                "tests",
                "scripts/verify --fast failed:\n" + outcome.summary(),
            )
        state.pass_marker(root, change).touch()
    if not state.touches_code(touched):
        return 0, ""
    verdict = state.read_verdict(root, change) or collect_verdict(root, session, change, event)
    if verdict is None:
        if running_verifiers(event):
            return 0, f"verify gate: a verifier is still running on change {change}"
        return block(root, session, change, "verdict", verifier_request(session, change))
    if verdict.result != "PASS":
        findings = json.dumps(verdict.findings, indent=2)
        counts = state.read_finding_counts(root, session)
        repeated = sorted(
            key
            for key in {state.finding_key(f) for f in verdict.findings}
            if counts.get(key, 0) >= MAX_FAILS_PER_FINDING
        )
        if repeated:
            return 0, (
                f"verify gate: the verifier failed {', '.join(repeated)} "
                f"{MAX_FAILS_PER_FINDING} times; bring it to the user.\n{findings}"
            )
        return block(
            root,
            session,
            change,
            "verdict",
            f"The verifier failed change {change}. Fix these, then run it again:\n{findings}",
        )
    return 0, ""


def parse_verdict(message: str) -> dict[str, Any] | None:
    """The JSON after the last `VERDICT ` line of the verifier's final message."""
    for line in reversed(message.splitlines()):
        if line.startswith(VERDICT_PREFIX):
            try:
                value = json.loads(line[len(VERDICT_PREFIX) :])
            except json.JSONDecodeError:
                return None
            return value if isinstance(value, dict) else None
    return None


def assistant_texts(transcript: Path) -> list[str]:
    """Assistant text blocks and hand-back messages of a subagent transcript, in order."""
    texts: list[str] = []
    try:
        lines = transcript.read_text(errors="replace").splitlines()
    except OSError:
        return texts
    for line in lines:
        try:
            message = json.loads(line).get("message") or {}
        except (json.JSONDecodeError, AttributeError):
            continue
        if message.get("role") != "assistant" or not isinstance(message.get("content"), list):
            continue
        for block in message["content"]:
            if block.get("type") == "text":
                texts.append(block.get("text") or "")
            elif block.get("type") == "tool_use" and block.get("name") == HANDBACK_TOOL:
                texts.append(str((block.get("input") or {}).get("message") or ""))
    return texts


def final_report(event: dict[str, Any]) -> str:
    """The subagent's last message, or its last hand-back when the message is empty."""
    message = event.get("last_assistant_message") or ""
    if parse_verdict(message) is not None or not event.get("agent_transcript_path"):
        return message
    for text in reversed(assistant_texts(Path(event["agent_transcript_path"]))):
        if parse_verdict(text) is not None:
            return text
    return message


def subagent_stop(event: dict[str, Any], root: Path) -> tuple[int, str]:
    """SubagentStop for the verifier: record its verdict when it names the current change."""
    if event.get("agent_type") != VERIFIER_AGENT:
        return 0, ""
    session = event.get("session_id")
    parsed = parse_verdict(final_report(event))
    if not session or parsed is None:
        return 2, (
            "End your final message with one line: "
            'VERDICT {"hash": "<change hash>", "result": "PASS" or "FAIL", "findings": [...]}'
        )
    current = state.content_hash(root, state.read_touched(root, session))
    if parsed.get("hash") != current:
        return 0, (
            f"verifier verdict ignored: it names change {parsed.get('hash')}, "
            f"but the current change is {current}"
        )
    findings = parsed.get("findings") or []
    result = str(parsed.get("result"))
    try:
        state.write_verdict(root, state.Verdict(current, result, findings))
    except ValueError as error:
        return 2, str(error)
    if result == "FAIL":
        state.count_findings(root, session, findings)
    return 0, ""


def is_test_path(path: str) -> bool:
    parts = Path(path).parts
    name = Path(path).name
    stem = Path(path).stem
    return (
        bool(TEST_DIRS.intersection(parts[:-1]))
        or name.startswith("test_")
        or (stem.endswith("_test") and Path(path).suffix in CPP_TEST_SUFFIXES)
    )


def change_report(root: Path, session: str) -> str:
    """What the verifier reads: hash, base commit, touched files, and the diff."""
    touched = state.read_touched(root, session)
    base = state.read_base(root, session)
    if not touched or base is None:
        return "no touched files recorded for this session"
    lines = [
        f"change hash: {state.content_hash(root, touched)}",
        f"base commit: {base}",
        suites_line(root, state.content_hash(root, touched)),
        "touched files:",
        *[f"  {'test ' if is_test_path(p) else 'code '}{p}" for p in touched],
        "",
        state.change_diff(root, base, touched),
    ]
    return "\n".join(lines)


def run_suites(root: Path, session: str, runner: Verifier = run_fast) -> tuple[bool, str]:
    """Run the fast suites on the session's change once per hash, recording a pass for the hook."""
    touched = state.read_touched(root, session)
    if not touched:
        return True, "no touched files recorded for this session"
    change = state.content_hash(root, touched)
    if state.pass_marker(root, change).exists():
        return True, f"suites: passed for this hash ({change}), not re-run"
    outcome = runner(root, touched)
    if not outcome.ok:
        return False, "scripts/verify --fast failed:\n" + outcome.summary()
    if state.content_hash(root, touched) == change:
        state.pass_marker(root, change).touch()
    return True, f"suites: passed for {change}"


def suites_line(root: Path, change: str) -> str:
    if state.pass_marker(root, change).exists():
        return "suites: passed for this hash (Stop hook, scripts/verify --fast --changed)"
    return "suites: not run for this hash"


def link_missing(source: Path, target: Path) -> None:
    """Symlink what a fresh worktree lacks from the main checkout, keeping tracked files."""
    if not source.exists():
        return
    if not target.exists() and not target.is_symlink():
        target.parent.mkdir(parents=True, exist_ok=True)
        target.symlink_to(source)
    elif source.is_dir() and target.is_dir() and not target.is_symlink():
        for child in source.iterdir():
            link_missing(child, target / child.name)


def make_mutant(root: Path, session: str) -> Path:
    """A worktree holding the change with every non-test hunk reverted to the base commit."""
    touched = state.read_touched(root, session)
    base = state.read_base(root, session)
    if base is None:
        raise SystemExit("no base commit recorded for this session")
    change = state.content_hash(root, touched)
    target = root / MUTANT_DIR / f"mutant-{change}"
    if target.exists():
        subprocess.run(["git", "worktree", "remove", "--force", str(target)], cwd=root, check=True)
    subprocess.run(
        ["git", "worktree", "add", "--detach", str(target), base],
        cwd=root,
        check=True,
        capture_output=True,
    )
    for path in touched:
        if not is_test_path(path):
            continue
        source = root / path
        destination = target / path
        if source.is_file():
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, destination)
        elif destination.exists():
            destination.unlink()
    for name in UNTRACKED_INPUTS:
        link_missing(root / name, target / name)
    (target / f"{ISOLATION_PLUGIN}.py").write_text(ISOLATION_SOURCE)
    return target


def emit(code: int, message: str) -> int:
    if message:
        print(message, file=sys.stderr)
    return code


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m auto_battlebot.verify.hooks")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("session-start", help="SessionStart hook (event JSON on stdin)")
    sub.add_parser("touch", help="PostToolUse hook (event JSON on stdin)")
    sub.add_parser("stop", help="Stop hook (event JSON on stdin)")
    sub.add_parser("subagent-stop", help="SubagentStop hook (event JSON on stdin)")
    sub.add_parser("bash-guard", help="PreToolUse hook on Bash (event JSON on stdin)")
    for name, text in (
        ("change", "Print the change hash, touched files, and diff"),
        ("mutant", "Make a worktree with the change's non-test files reverted"),
        ("suites", "Run scripts/verify --fast on the change once per hash"),
    ):
        command = sub.add_parser(name, help=text)
        command.add_argument("--session", required=True)
    args = parser.parse_args(argv)
    root = repo_root()
    if args.command == "session-start":
        return session_start(read_input(), root)
    if args.command == "touch":
        return touch(read_input(), root)
    if args.command == "stop":
        return emit(*stop(read_input(), root))
    if args.command == "subagent-stop":
        return emit(*subagent_stop(read_input(), root))
    if args.command == "bash-guard":
        return emit(*bash_guard(read_input()))
    if args.command == "change":
        print(change_report(root, args.session))
        return 0
    if args.command == "suites":
        ok, message = run_suites(root, args.session)
        print(message)
        return 0 if ok else 1
    print(make_mutant(root, args.session))
    return 0


if __name__ == "__main__":
    sys.exit(main())
