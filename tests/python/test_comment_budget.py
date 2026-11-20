"""Tests for scripts/comment_budget.py, loaded by path since scripts/ is not a package."""

from __future__ import annotations

import importlib.util
import io
import json
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "scripts" / "comment_budget.py"
SPEC = importlib.util.spec_from_file_location("comment_budget", SCRIPT)
assert SPEC and SPEC.loader
cb = importlib.util.module_from_spec(SPEC)
sys.modules["comment_budget"] = cb
SPEC.loader.exec_module(cb)


def counted(path: str, text: str) -> list[int]:
    """Counted line numbers of a source snippet."""
    result = cb.analyze(path, text)
    assert result is not None
    return [line for line, _ in result.counted]


def malformed(path: str, text: str) -> list[int]:
    """Malformed DATASHEET line numbers of a source snippet."""
    result = cb.analyze(path, text)
    assert result is not None
    return result.malformed


def git(root: Path, *args: str) -> None:
    """Run git quietly in a temp repo."""
    subprocess.run(
        ["git", "-C", str(root), "-c", "user.name=t", "-c", "user.email=t@t", *args],
        check=True,
        capture_output=True,
    )


def make_repo(tmp_path: Path, files: dict[str, str]) -> Path:
    """A committed temp repo holding the given files."""
    git(tmp_path, "init", "-q")
    for name, text in files.items():
        (tmp_path / name).parent.mkdir(parents=True, exist_ok=True)
        (tmp_path / name).write_text(text)
    git(tmp_path, "add", "-A")
    git(tmp_path, "commit", "-q", "--no-verify", "-m", "init")
    return tmp_path


def run(root: Path, *args: str) -> int:
    """Run the CLI against a temp repo."""
    return int(cb.main(["--root", str(root), *args]))


def test_cxx_slashes_inside_strings_are_code() -> None:
    src = 'auto url = "http://example.com"; // real\nconst char* s = "/* no */";\n'
    assert counted("a.cpp", src) == [1]


def test_cxx_raw_string_and_char_literals() -> None:
    src = 'auto r = R"x(// not )" still )x";\nchar q = \'"\'; // yes\nint n = 1\'000; // sep\n'
    assert counted("a.cpp", src) == [2, 3]


def test_cxx_block_comment_counts_each_line() -> None:
    assert counted("a.hpp", "int a;\n/* one\n * two\n */\nint b; /* tail */\n") == [2, 3, 4, 5]


def test_cxx_directives_exempt_only_without_prose() -> None:
    src = "x(); // NOLINT(bugprone-foo)\ny(); // NOLINT(bugprone-foo) because reasons\n"
    src += "// clang-format off\n#pragma once\n"
    assert counted("a.cpp", src) == [2]


def test_js_template_literal_hides_slashes() -> None:
    assert counted("a.ts", "const u = `http://${host}//x`;\n// c\n") == [2]


def test_python_hash_in_string_and_directives() -> None:
    src = (
        "#!/usr/bin/env python3\n"
        "s = '# not a comment'\n"
        "x = 1  # noqa: E501\n"
        "y = 2  # noqa: E501 because the URL is long\n"
        "z = f()  # type: ignore[attr-defined]  # noqa: F401\n"
        "w = 3  # type: ignore\n"
    )
    assert counted("a.py", src) == [4, 6]


def test_python_docstrings_count_from_two_lines() -> None:
    src = (
        '"""One line module docstring."""\n'
        "def f():\n"
        '    """Two\n'
        '    lines."""\n'
        '    """A plain string expression,\n'
        '    not a docstring."""\n'
    )
    assert counted("a.py", src) == [3, 4]


def test_python_coding_header_exempt_only_near_top() -> None:
    header = "# -*- coding: utf-8 -*-\n"
    assert counted("a.py", header) == []
    assert counted("a.py", "x = 1\n" * 6 + header) == [7]


def test_shell_quotes_and_parameter_expansion() -> None:
    src = (
        "#!/bin/bash\n"
        'echo \'# a\' "# b" "$(echo "# c")"  # d\n'
        "n=${#arr[@]}; p=${v#pre}; [ $# -gt 0 ]\n"
        'echo "it\'s" # e\n'
    )
    assert counted("a.sh", src) == [2, 4]


def test_shell_heredoc_body_is_not_scanned() -> None:
    src = "cat <<'EOF' >x.toml\n# inside\nEOF\nv=$((1<<2))\n# after\n"
    assert counted("a.sh", src) == [5]


def test_shell_shebang_detects_extensionless_script() -> None:
    assert counted("scripts/tool", "#!/bin/bash\n# c\n") == [2]
    assert cb.analyze("scripts/tool", "plain text\n# c\n") is None


def test_toml_hash_inside_strings() -> None:
    src = 'a = "x # y"\nb = \'z # w\'\nc = """\n# no\n"""\nd = 1 # yes\n# also\n'
    assert counted("a.toml", src) == [6, 7]


def test_yaml_hash_needs_word_boundary() -> None:
    assert counted("a.yml", "url: http://x/#frag\nk: v # c\n'q': '# no'\n") == [2]


def test_datasheet_valid_lines_are_free() -> None:
    src = (
        "// DATASHEET: BNO055 rev 1.4, table 4-2: OPR_MODE NDOF = 0x0C\n"
        "// DATASHEET: ESP32-S3 TRM v1.2, 7.4.3: RMT carrier duty is in APB ticks\n"
    )
    assert counted("a.cpp", src) == []
    assert counted("a.py", "# DATASHEET: AS5600 rev 1.06, p. 18: ANGLE is 12 bits\n") == []


def test_datasheet_missing_revision_or_locator_is_malformed() -> None:
    src = (
        "// DATASHEET: BNO055, table 4-2: no revision\n"
        "// DATASHEET: BNO055 rev 1.4: no locator\n"
        "// DATASHEET: BNO055 rev 1.4, table 4-2: fine\n"
    )
    assert malformed("a.cpp", src) == [1, 2]


def test_check_fails_over_budget_and_passes_within(tmp_path: Path) -> None:
    root = make_repo(tmp_path, {"a.py": "x = 1  # one\n"})
    assert run(root, "check") == 1
    (root / cb.BASELINE_NAME).write_text(json.dumps({"a.py": 1}))
    assert run(root, "check") == 0


def test_malformed_datasheet_fails_regardless_of_budget(tmp_path: Path) -> None:
    root = make_repo(tmp_path, {"a.py": "# DATASHEET: part, p. 1: fact\n"})
    (root / cb.BASELINE_NAME).write_text(json.dumps({"a.py": 5}))
    assert run(root, "check") == 1


def test_git_mv_inherits_budget(tmp_path: Path) -> None:
    body = "x = 1  # one\n" + "".join(f"v{i} = {i}\n" for i in range(20))
    root = make_repo(tmp_path, {"a.py": body, cb.BASELINE_NAME: '{"a.py": 1}'})
    git(root, "mv", "a.py", "b.py")
    assert run(root, "check") == 0
    assert run(root, "ratchet") == 0
    assert json.loads((root / cb.BASELINE_NAME).read_text()) == {"b.py": 1}


def test_ratchet_lowers_never_raises_and_drops_zeros(tmp_path: Path) -> None:
    files = {"a.py": "x = 1  # a\n", "b.py": "y = 2\n", "c.py": "# c1\n# c2\n# c3\n"}
    root = make_repo(tmp_path, files)
    (root / cb.BASELINE_NAME).write_text(json.dumps({"a.py": 4, "b.py": 2, "c.py": 1}))
    assert run(root, "ratchet") == 0
    assert json.loads((root / cb.BASELINE_NAME).read_text()) == {"a.py": 1, "c.py": 1}


def test_staged_check_reads_the_index(tmp_path: Path) -> None:
    root = make_repo(tmp_path, {"a.py": "x = 1\n"})
    (root / "a.py").write_text("x = 1  # staged\n")
    git(root, "add", "a.py")
    (root / "a.py").write_text("x = 1\n")
    assert run(root, "check", "--staged") == 1


def test_hook_exits_2_on_a_commented_edit(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    root = make_repo(tmp_path, {"a.py": "x = 1  # new\n"})
    payload = {"tool_input": {"file_path": str(root / "a.py")}}
    monkeypatch.setattr("sys.stdin", io.StringIO(json.dumps(payload)))
    assert run(root, "hook") == 2


def test_checker_files_are_scanned_and_unbudgeted() -> None:
    baseline = json.loads((REPO_ROOT / cb.BASELINE_NAME).read_text())
    for path in cb.SELF_PATHS:
        text = (REPO_ROOT / path).read_text()
        assert cb.is_scanned(path, text.split("\n")[0]), path
        assert path not in baseline, path
    assert run(REPO_ROOT, "check", "--files", *(str(REPO_ROOT / p) for p in cb.SELF_PATHS)) == 0


def test_baseline_entry_for_checker_is_rejected(tmp_path: Path) -> None:
    root = make_repo(tmp_path, {"scripts/comment_budget.py": "x = 1  # c\n"})
    (root / cb.BASELINE_NAME).write_text(json.dumps({"scripts/comment_budget.py": 9}))
    assert run(root, "check") == 1


def test_self_check_is_live(tmp_path: Path) -> None:
    target = tmp_path / "scripts" / "comment_budget.py"
    target.parent.mkdir()
    shutil.copy(SCRIPT, target)
    make_repo(tmp_path, {})
    assert run(tmp_path, "check") == 0
    target.write_text(target.read_text() + "# one comment\n")
    assert run(tmp_path, "check") == 1


def test_namespace_closer_and_ts_reference_are_exempt() -> None:
    assert counted("a.cpp", "}  // namespace auto_battlebot\n}  // namespace\n") == []
    assert counted("a.ts", '/// <reference types="vite/client" />\n') == []


def test_generated_header_marker_in_first_three_lines() -> None:
    src = "#pragma once\n\n// Generated by tool.py. Do not edit.\nint x; // c\n"
    assert cb.analyze("a.hpp", src) is None
    assert cb.analyze("a.hpp", "int x;\n" * 3 + "// generated by hand\n") is not None


def long_message_lines(path: str, text: str) -> list[int]:
    """Lines of the long message literals the report would list."""
    return [m.line for m in cb.long_messages(path, text)]


def test_cxx_throw_with_adjacent_literals_past_limit_is_listed() -> None:
    part = "x" * 90
    text = f'int a;\nthrow std::runtime_error(\n    "{part}"\n    "{part}");\n'
    assert long_message_lines("src/a.cpp", text) == [2]


def test_cxx_short_throw_and_long_plain_string_are_not_listed() -> None:
    long = "y" * 200
    text = f'throw std::runtime_error("short");\nconst char *k = "{long}";\n'
    assert long_message_lines("src/a.cpp", text) == []


def test_cxx_spdlog_and_logger_calls_are_listed() -> None:
    long = "z" * 170
    text = f'spdlog::warn("{long}");\nlogger_->error("{long}");\n'
    assert long_message_lines("src/a.cpp", text) == [1, 2]


def test_cxx_structured_log_field_keys_are_not_message_text() -> None:
    keys = ", ".join(f'{{"field_{i:02d}_name", x}}' for i in range(12))
    text = f'logger_->debug("profile",\n    {{{keys}}});\n'
    assert long_message_lines("src/a.cpp", text) == []


def test_python_raise_log_and_help_past_limit_are_listed() -> None:
    long = "w" * 170
    text = (
        f'raise ValueError("{long}")\n'
        f'logging.warning("{long}")\n'
        f'parser.add_argument("--x", help="{long}")\n'
        f'parser.add_argument("--y", help="short")\n'
        f'note = "{long}"\n'
    )
    assert long_message_lines("tools/a.py", text) == [1, 2, 3]


def test_python_fstring_pieces_count_toward_the_limit() -> None:
    piece = "v" * 90
    text = f'raise ValueError(f"{piece} {{name}} {piece}")\n'
    assert long_message_lines("tools/a.py", text) == [1]


def test_cxx_cli11_descriptions_are_listed() -> None:
    long = "u" * 170
    text = f'app.add_option("--config", path, "{long}");\n'
    assert long_message_lines("src/main.cpp", text) == [1]
