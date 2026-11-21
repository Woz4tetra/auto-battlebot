"""scripts/lint wires clang-tidy, ctcache, cmake, and taplo correctly, checked with fake tools."""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

from auto_battlebot.paths import REPO_ROOT

REPO = REPO_ROOT

LOGGER = (
    '#!/bin/sh\necho "{name} $* RUST_LOG=$RUST_LOG CTCACHE=$CTCACHE_CLANG_TIDY" >> "$LINT_LOG"\n'
)
FAKES = [
    "clang-format-18",
    "clang-tidy-18",
    "run-clang-tidy-18",
    "run-clang-tidy",
    "cmake",
    "taplo",
]
PASS_ALL = ["ruff", "mypy"]


def run_lint(tmp_path: Path, *, rust_log: str | None = None, with_cmake_failure: bool = False):
    project = tmp_path / "project"
    (project / "scripts").mkdir(parents=True)
    (project / "build").mkdir()
    (project / "config").mkdir()
    (project / "config" / "a.toml").write_text("")
    (project / "build" / "compile_commands.json").write_text("[]")
    (project / ".llvm-version").write_text("18\n")
    shutil.copy(REPO / "scripts" / "lint", project / "scripts" / "lint")
    (project / "scripts" / "check_install_option_leaks.sh").write_text("#!/bin/sh\nexit 0\n")
    (project / "scripts" / "check_install_option_leaks.sh").chmod(0o755)
    (project / "scripts" / "comment_budget.py").write_text("")
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    for name in [*FAKES, *PASS_ALL]:
        body = LOGGER.format(name=name)
        if name == "cmake" and with_cmake_failure:
            body += "exit 1\n"
        (bin_dir / name).write_text(body)
        (bin_dir / name).chmod(0o755)
    home = tmp_path / "home"
    ctcache = home / ".local" / "share" / "ctcache"
    ctcache.mkdir(parents=True)
    (ctcache / "clang-tidy").write_text("#!/bin/sh\n")
    (ctcache / "clang-tidy").chmod(0o755)
    log = tmp_path / "lint.log"
    env = {k: v for k, v in os.environ.items() if k not in {"RUST_LOG", "CTCACHE_CLANG_TIDY"}}
    env.update(PATH=f"{bin_dir}:{env['PATH']}", HOME=str(home), LINT_LOG=str(log))
    if rust_log is not None:
        env["RUST_LOG"] = rust_log
    result = subprocess.run(
        ["bash", str(project / "scripts" / "lint"), "--dry-run"],
        env=env,
        capture_output=True,
        text=True,
        stdin=subprocess.DEVNULL,
    )
    return result, (log.read_text().splitlines() if log.exists() else []), bin_dir


def calls(lines, name):
    return [line for line in lines if line.startswith(f"{name} ")]


def test_clang_tidy_runs_through_the_versioned_driver_not_the_bare_one(tmp_path):
    _, lines, _ = run_lint(tmp_path)
    assert len(calls(lines, "run-clang-tidy-18")) == 1
    assert calls(lines, "run-clang-tidy") == []


def test_ctcache_gets_the_absolute_clang_tidy_path(tmp_path):
    _, lines, bin_dir = run_lint(tmp_path)
    call = calls(lines, "run-clang-tidy-18")[0]
    assert f"CTCACHE={bin_dir}/clang-tidy-18" in call
    assert "-clang-tidy-binary" in call
    assert str(tmp_path / "home" / ".local/share/ctcache/clang-tidy") in call


def test_cmake_reconfigures_before_clang_tidy_runs(tmp_path):
    _, lines, _ = run_lint(tmp_path)
    order = [line.split()[0] for line in lines]
    assert order.index("cmake") < order.index("run-clang-tidy-18")
    assert "-S" in calls(lines, "cmake")[0] and "-B" in calls(lines, "cmake")[0]


def test_taplo_runs_with_rust_log_warn_by_default(tmp_path):
    _, lines, _ = run_lint(tmp_path)
    assert all("RUST_LOG=warn" in line for line in calls(lines, "taplo"))
    assert calls(lines, "taplo")


def test_caller_rust_log_wins(tmp_path):
    _, lines, _ = run_lint(tmp_path, rust_log="debug")
    assert all("RUST_LOG=debug" in line for line in calls(lines, "taplo"))
