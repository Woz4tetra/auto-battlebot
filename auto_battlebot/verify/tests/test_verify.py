"""Tests for auto_battlebot.verify: file selection, session state, and the agent hooks."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

from auto_battlebot.compat import tomllib
from auto_battlebot.paths import REPO_ROOT
from auto_battlebot.verify import hooks, selection, state
from auto_battlebot.verify.cli import Outcome, file_list, parse_args, working_changes
from auto_battlebot.verify.selection import Selection, select
from auto_battlebot.verify.suites import StepResult

SESSION = "session-1"


def write(root: Path, path: str, text: str = "") -> Path:
    target = root / path
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(text)
    return target


def git(root: Path, *args: str) -> None:
    subprocess.run(["git", *args], cwd=root, check=True, capture_output=True)


@pytest.fixture
def repo(tmp_path: Path) -> Path:
    git(tmp_path, "init", "-q")
    git(tmp_path, "config", "user.email", "t@example.com")
    git(tmp_path, "config", "user.name", "t")
    write(tmp_path, "README.md", "x\n")
    git(tmp_path, "add", ".")
    git(tmp_path, "commit", "-q", "-m", "base")
    return tmp_path


def depfile(root: Path, source: str, deps: list[str]) -> None:
    lines = [f"{source}.o: \\"] + [f"  {root.resolve() / d} \\" for d in deps] + ["  /usr/x.h"]
    write(root, f"{selection.GTEST_DEPFILE_DIR}/{source}.o.d", "\n".join(lines) + "\n")


def test_python_import_closure_selects_transitive_tests(tmp_path: Path) -> None:
    write(tmp_path, "auto_battlebot/__init__.py")
    write(tmp_path, "auto_battlebot/foo/__init__.py")
    write(tmp_path, "auto_battlebot/foo/bar.py", "from . import baz\n")
    write(tmp_path, "auto_battlebot/foo/baz.py", "X = 1\n")
    write(tmp_path, "scripts/tests/test_a.py", "from auto_battlebot.foo.bar import thing\n")
    write(tmp_path, "scripts/tests/test_b.py", "import os\n")
    chosen = select(["auto_battlebot/foo/baz.py"], tmp_path)
    assert chosen.scopes["pytest"].items == {"scripts/tests/test_a.py"}
    assert not chosen.scopes["gtest"].run


def test_python_tests_naming_a_file_select_it(tmp_path: Path) -> None:
    write(tmp_path, "scripts/deploy.sh", "echo\n")
    write(tmp_path, "scripts/tool.py", "X = 1\n")
    write(tmp_path, "scripts/tests/test_deploy.py", 'S = ROOT / "scripts" / "deploy.sh"\n')
    write(tmp_path, "scripts/tests/test_tool.py", 'P = "scripts/tool.py"\n')
    chosen = select(["scripts/deploy.sh", "scripts/tool.py"], tmp_path)
    assert chosen.scopes["pytest"].items == {
        "scripts/tests/test_deploy.py",
        "scripts/tests/test_tool.py",
    }


def test_a_touched_test_file_selects_itself_and_conftest_selects_all(tmp_path: Path) -> None:
    write(tmp_path, "scripts/tests/test_x.py", "def test_x(): pass\n")
    assert select(["scripts/tests/test_x.py"], tmp_path).scopes["pytest"].items == {
        "scripts/tests/test_x.py"
    }
    assert select(["conftest.py"], tmp_path).scopes["pytest"].everything


def test_gtest_header_maps_to_suites_of_including_tests(tmp_path: Path) -> None:
    write(tmp_path, "include/m/h.hpp")
    write(tmp_path, "src/m/h.cpp")
    write(tmp_path, "tests/test_c.cpp", "TEST(CSuite, a) {}\nTEST_F(CFixture, b) {}\n")
    write(tmp_path, "tests/test_d.cpp", "TEST(DSuite, a) {}\n")
    depfile(tmp_path, "tests/test_c.cpp", ["tests/test_c.cpp", "include/m/h.hpp"])
    depfile(tmp_path, "tests/test_d.cpp", ["tests/test_d.cpp"])
    for touched in ("src/m/h.cpp", "include/m/h.hpp"):
        scope = select([touched], tmp_path).scopes["gtest"]
        assert scope.items == {"CSuite", "CFixture"}
        assert not scope.everything
    assert select(["tests/test_d.cpp"], tmp_path).scopes["gtest"].items == {"DSuite"}


def test_gtest_runs_everything_when_nothing_includes_the_file(tmp_path: Path) -> None:
    write(tmp_path, "src/lonely/z.cpp")
    write(tmp_path, "tests/test_d.cpp", "TEST(DSuite, a) {}\n")
    depfile(tmp_path, "tests/test_d.cpp", ["tests/test_d.cpp"])
    assert select(["src/lonely/z.cpp"], tmp_path).scopes["gtest"].everything
    assert select(["CMakeLists.txt"], tmp_path).scopes["gtest"].everything
    assert select(["config/robots/x.toml"], tmp_path).scopes["gtest"].everything


def test_gtest_without_depfiles_runs_everything(tmp_path: Path) -> None:
    write(tmp_path, "include/m/h.hpp")
    assert select(["include/m/h.hpp"], tmp_path).scopes["gtest"].everything


def test_directory_suites_and_uncovered_files(tmp_path: Path) -> None:
    write(tmp_path, "scripts/orphan.py", "X = 1\n")
    write(tmp_path, "docs/a.md", "x\n")
    chosen = select(
        [
            "firmware/mr_stabs_mk2/src/main.cpp",
            "web/src/App.svelte",
            "training/synthetic/synthgen/a.py",
            "scripts/orphan.py",
            "docs/a.md",
            "web/node_modules/x/index.js",
        ],
        tmp_path,
    )
    runs = {name for name, scope in chosen.scopes.items() if scope.run}
    assert runs == {"firmware", "web", "synthetic"}
    assert chosen.uncovered == ["scripts/orphan.py"]
    assert not select(["docs/a.md"], tmp_path).any()


def test_protocol_header_runs_the_web_check(tmp_path: Path) -> None:
    assert select(["include/remote/protocol.hpp"], tmp_path).scopes["web"].run


def test_gtest_filter_matches_plain_parameterized_and_typed() -> None:
    assert selection.gtest_filter(["A"]) == "A.*:*/A.*:A/*"


def test_parse_depfile_keeps_repo_files_only(tmp_path: Path) -> None:
    text = f"x.o: {tmp_path}/a.cpp \\\n {tmp_path}/inc/b.hpp /usr/include/c.h\n"
    assert selection.parse_depfile(text, tmp_path) == {"a.cpp", "inc/b.hpp"}


def test_real_repo_maps_comment_budget_to_its_test() -> None:
    chosen = select(["scripts/comment_budget.py"], REPO_ROOT)
    assert "scripts/tests/test_comment_budget.py" in chosen.scopes["pytest"].items


def test_record_touch_skips_outside_and_agent_state(repo: Path) -> None:
    assert state.record_touch(repo, SESSION, "/etc/passwd") is None
    assert state.record_touch(repo, SESSION, str(repo / ".claude/state/x")) is None
    assert state.record_touch(repo, SESSION, str(repo / ".claude/worktrees/m/a.py")) is None
    assert (
        state.record_touch(repo, SESSION, ".claude/agents/v.md", cwd=repo) == ".claude/agents/v.md"
    )
    assert state.record_touch(repo, SESSION, "src/a.cpp", cwd=repo) == "src/a.cpp"
    state.record_touch(repo, SESSION, str(repo / "src/a.cpp"))
    assert state.read_touched(repo, SESSION) == [".claude/agents/v.md", "src/a.cpp"]
    assert state.read_base(repo, SESSION) is not None


def test_content_hash_follows_content_and_deletion(repo: Path) -> None:
    write(repo, "a.py", "1\n")
    first = state.content_hash(repo, ["a.py"])
    write(repo, "a.py", "2\n")
    second = state.content_hash(repo, ["a.py"])
    (repo / "a.py").unlink()
    assert len({first, second, state.content_hash(repo, ["a.py"])}) == 3


def test_change_diff_covers_modified_and_new_files(repo: Path) -> None:
    state.record_touch(repo, SESSION, "README.md", cwd=repo)
    write(repo, "README.md", "y\n")
    write(repo, "new.py", "Z = 1\n")
    base = state.read_base(repo, SESSION)
    assert base is not None
    diff = state.change_diff(repo, base, ["README.md", "new.py"])
    assert "+y" in diff and "+Z = 1" in diff


def outcome(ok: bool) -> Outcome:
    result = StepResult("pytest", ok, 1.0, Path("/dev/null"), ["pytest"])
    return Outcome(ok, [result], Selection())


def touch_files(repo: Path, *paths: str) -> None:
    for path in paths:
        write(repo, path, f"{path}\n")
        hooks.touch({"session_id": SESSION, "tool_input": {"file_path": str(repo / path)}}, repo)


def test_stop_passes_with_nothing_touched(repo: Path) -> None:
    assert hooks.stop({"session_id": SESSION}, repo, lambda r, f: outcome(False)) == (0, "")


def test_stop_blocks_on_failure_then_gives_up_after_two_blocks(repo: Path) -> None:
    touch_files(repo, "scripts/a.py")
    event = {"session_id": SESSION}
    failing = lambda r, f: outcome(False)  # noqa: E731
    assert hooks.stop(event, repo, failing)[0] == 2
    assert hooks.stop(event, repo, failing)[0] == 2
    code, message = hooks.stop(event, repo, failing)
    assert code == 0 and "user can decide" in message


def test_stop_reruns_after_the_change_moves(repo: Path) -> None:
    touch_files(repo, "scripts/a.py")
    event = {"session_id": SESSION}
    failing = lambda r, f: outcome(False)  # noqa: E731
    hooks.stop(event, repo, failing)
    hooks.stop(event, repo, failing)
    write(repo, "scripts/a.py", "fixed\n")
    assert hooks.stop(event, repo, failing)[0] == 2


def test_stop_caches_a_pass_by_change_hash(repo: Path) -> None:
    touch_files(repo, "docs/a.md")
    calls = []

    def passing(root: Path, files: list[str]) -> Outcome:
        calls.append(files)
        return outcome(True)

    assert hooks.stop({"session_id": SESSION}, repo, passing) == (0, "")
    assert hooks.stop({"session_id": SESSION}, repo, passing) == (0, "")
    assert calls == [["docs/a.md"]]


def verifier_event(message: str) -> dict[str, str]:
    return {"session_id": SESSION, "agent_type": "verifier", "last_assistant_message": message}


def verdict_line(change: str, result: str, findings: list[dict[str, str]] | None = None) -> str:
    body = {"hash": change, "result": result, "findings": findings or []}
    return "review text\nVERDICT " + json.dumps(body)


def test_code_change_needs_a_passing_verdict(repo: Path) -> None:
    touch_files(repo, "src/a.cpp")
    event = {"session_id": SESSION}
    passing = lambda r, f: outcome(True)  # noqa: E731
    code, message = hooks.stop(event, repo, passing)
    assert code == 2 and "verifier" in message
    change = state.content_hash(repo, state.read_touched(repo, SESSION))
    assert hooks.subagent_stop(verifier_event(verdict_line(change, "PASS")), repo) == (0, "")
    assert hooks.stop(event, repo, passing) == (0, "")


def test_verdict_for_a_stale_hash_is_ignored(repo: Path) -> None:
    touch_files(repo, "src/a.cpp")
    code, message = hooks.subagent_stop(verifier_event(verdict_line("0" * 16, "PASS")), repo)
    assert code == 0 and "ignored" in message
    assert hooks.stop({"session_id": SESSION}, repo, lambda r, f: outcome(True))[0] == 2


def test_verdict_line_is_required_and_other_agents_are_ignored(repo: Path) -> None:
    touch_files(repo, "src/a.cpp")
    assert hooks.subagent_stop(verifier_event("no verdict"), repo)[0] == 2
    other = {"session_id": SESSION, "agent_type": "Explore", "last_assistant_message": ""}
    assert hooks.subagent_stop(other, repo) == (0, "")


def test_verifier_edits_are_not_recorded_as_touched(repo: Path) -> None:
    event = {
        "session_id": SESSION,
        "agent_type": "verifier",
        "tool_input": {"file_path": str(repo / "src/x.cpp")},
    }
    hooks.touch(event, repo)
    assert state.read_touched(repo, SESSION) == []


def test_same_finding_failing_twice_goes_to_the_user(repo: Path) -> None:
    finding = {"file": "src/a.cpp", "check": "mutation", "detail": "tautology"}
    passing = lambda r, f: outcome(True)  # noqa: E731
    for attempt in range(2):
        touch_files(repo, "src/a.cpp")
        write(repo, "src/a.cpp", f"attempt {attempt}\n")
        change = state.content_hash(repo, state.read_touched(repo, SESSION))
        hooks.subagent_stop(verifier_event(verdict_line(change, "FAIL", [finding])), repo)
        code, message = hooks.stop({"session_id": SESSION}, repo, passing)
    assert code == 0 and "bring it to the user" in message


def test_first_fail_verdict_blocks_with_findings(repo: Path) -> None:
    touch_files(repo, "src/a.cpp")
    finding = {"file": "src/a.cpp", "check": "tests", "detail": "no test"}
    change = state.content_hash(repo, state.read_touched(repo, SESSION))
    hooks.subagent_stop(verifier_event(verdict_line(change, "FAIL", [finding])), repo)
    code, message = hooks.stop({"session_id": SESSION}, repo, lambda r, f: outcome(True))
    assert code == 2 and "no test" in message


def test_mutant_keeps_new_tests_and_reverts_code(repo: Path) -> None:
    write(repo, "src/a.py", "old\n")
    write(repo, "data/.keep")
    git(repo, "add", ".")
    git(repo, "commit", "-q", "-m", "code")
    write(repo, "data/clip.svo2", "bytes")
    touch_files(repo, "src/a.py", "scripts/tests/test_a.py")
    write(repo, "src/a.py", "new\n")
    mutant = hooks.make_mutant(repo, SESSION)
    assert (mutant / "src/a.py").read_text() == "old\n"
    assert (mutant / "scripts/tests/test_a.py").read_text() == "scripts/tests/test_a.py\n"
    assert (mutant / "data/clip.svo2").is_symlink()
    assert (repo / "src/a.py").read_text() == "new\n"
    plugin = (mutant / f"{hooks.ISOLATION_PLUGIN}.py").read_text()
    assert "__editable__" in plugin


def test_isolation_plugin_drops_editable_finders() -> None:
    class Finder:
        __module__ = "__editable___auto_battlebot_0_1_0_finder"

    saved = list(sys.meta_path)
    modules = dict(sys.modules)
    sys.meta_path.append(Finder)  # type: ignore[arg-type]
    try:
        exec(hooks.ISOLATION_SOURCE, {})
        assert Finder not in sys.meta_path
    finally:
        sys.meta_path[:] = saved
        sys.modules.update(modules)


def test_is_test_path() -> None:
    assert hooks.is_test_path("tests/test_x.cpp")
    assert hooks.is_test_path("firmware/mr_stabs_mk2/test/test_pid/main.cpp")
    assert hooks.is_test_path("training/synthetic/tests/test_a.py")
    assert hooks.is_test_path("auto_battlebot/eval/tests/test_metrics.py")
    assert hooks.is_test_path("tests/foo_test.cpp")
    assert not hooks.is_test_path("auto_battlebot/latest_frames.py")
    assert not hooks.is_test_path("auto_battlebot/nav/step_test.py")


def test_change_report_lists_hash_and_kinds(repo: Path) -> None:
    touch_files(repo, "src/a.cpp", "tests/test_a.cpp")
    report = hooks.change_report(repo, SESSION)
    assert "change hash: " in report
    assert "code src/a.cpp" in report and "test tests/test_a.cpp" in report


def test_file_list_sources(repo: Path) -> None:
    write(repo, "untracked.py")
    list_file = write(repo, "list.txt", "a.py\n\nb.py\n")
    assert file_list(parse_args([]), repo) is None
    assert file_list(parse_args(["--changed", "x.py"]), repo) == ["x.py"]
    assert file_list(parse_args(["--changed", "--files-from", str(list_file)]), repo) == [
        "a.py",
        "b.py",
    ]
    assert "untracked.py" in file_list(parse_args(["--changed"]), repo)
    with pytest.raises(ValueError):
        file_list(parse_args(["x.py"]), repo)


def test_working_changes_reads_renames(repo: Path) -> None:
    git(repo, "mv", "README.md", "RENAMED.md")
    assert working_changes(repo) == ["RENAMED.md"]


def test_verdict_is_read_from_the_handback_in_the_transcript(repo: Path, tmp_path: Path) -> None:
    touch_files(repo, "src/a.cpp")
    change = state.content_hash(repo, state.read_touched(repo, SESSION))
    transcript = tmp_path / "agent.jsonl"
    lines = [
        {"message": {"role": "assistant", "content": [{"type": "text", "text": "looking"}]}},
        {
            "message": {
                "role": "user",
                "content": [{"type": "text", "text": verdict_line("x", "PASS")}],
            }
        },
        {
            "message": {
                "role": "assistant",
                "content": [
                    {
                        "type": "tool_use",
                        "name": hooks.HANDBACK_TOOL,
                        "input": {"message": verdict_line(change, "PASS")},
                    }
                ],
            }
        },
    ]
    transcript.write_text("\n".join(json.dumps(line) for line in lines) + "\nnot json\n")
    event = {**verifier_event(""), "agent_transcript_path": str(transcript)}
    assert hooks.subagent_stop(event, repo) == (0, "")
    verdict = state.read_verdict(repo, change)
    assert verdict is not None and verdict.result == "PASS"


def session_with_subagents(tmp_path: Path) -> tuple[dict[str, str], Path]:
    transcript = tmp_path / "sessions" / f"{SESSION}.jsonl"
    subagents = transcript.with_suffix("") / "subagents"
    subagents.mkdir(parents=True)
    transcript.write_text("")
    return {"session_id": SESSION, "transcript_path": str(transcript)}, subagents


def handback(text: str) -> str:
    block = {"type": "tool_use", "name": hooks.HANDBACK_TOOL, "input": {"message": text}}
    return json.dumps({"message": {"role": "assistant", "content": [block]}}) + "\n"


def add_agent(subagents: Path, agent_id: str, agent_type: str, finished: bool, text: str) -> None:
    meta = {"agentType": agent_type, "finished": finished}
    (subagents / f"agent-{agent_id}.meta.json").write_text(json.dumps(meta))
    (subagents / f"agent-{agent_id}.jsonl").write_text(handback(text))


def test_stop_collects_a_finished_verifier_verdict_from_its_transcript(
    repo: Path, tmp_path: Path
) -> None:
    touch_files(repo, "src/a.cpp")
    change = state.content_hash(repo, state.read_touched(repo, SESSION))
    event, subagents = session_with_subagents(tmp_path)
    add_agent(subagents, "stale", "verifier", True, verdict_line("0" * 16, "PASS"))
    add_agent(subagents, "other", "Explore", True, verdict_line(change, "PASS"))
    add_agent(subagents, "live", "verifier", False, verdict_line(change, "PASS"))
    passing = lambda r, f: outcome(True)  # noqa: E731
    assert hooks.stop(event, repo, passing)[0] == 2
    add_agent(subagents, "done", "verifier", True, verdict_line(change, "PASS"))
    assert hooks.stop(event, repo, passing) == (0, "")
    assert state.read_verdict(repo, change) is not None


def test_stop_waits_for_a_running_verifier(repo: Path, tmp_path: Path) -> None:
    touch_files(repo, "src/a.cpp")
    event, subagents = session_with_subagents(tmp_path)
    add_agent(subagents, "v1", "verifier", False, "working")
    add_agent(subagents, "e1", "Explore", False, "working")
    passing = lambda r, f: outcome(True)  # noqa: E731
    running = {**event, "background_tasks": [{"id": "e1", "type": "subagent", "status": "running"}]}
    assert hooks.stop(running, repo, passing)[0] == 2
    running["background_tasks"].append({"id": "v1", "type": "subagent", "status": "running"})
    code, message = hooks.stop(running, repo, passing)
    assert code == 0 and "still running" in message


def test_pytest_roots_match_the_pytest_testpaths() -> None:
    config = tomllib.loads((REPO_ROOT / "pyproject.toml").read_text())
    assert tuple(config["tool"]["pytest"]["ini_options"]["testpaths"]) == selection.PYTEST_ROOTS


def test_the_walk_finds_every_collected_test_module() -> None:
    files = selection.PythonIndex(REPO_ROOT).test_files()
    assert "auto_battlebot/verify/tests/test_verify.py" in files
    assert "scripts/tests/test_comment_budget.py" in files
    assert not any(f.startswith("training/synthetic/") for f in files)
