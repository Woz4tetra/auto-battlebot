"""setup_claude_code.sh run against a fake claude and a fake git that records its environment."""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
FAKE_GIT = '#!/bin/sh\necho "git $* PROMPT=$GIT_TERMINAL_PROMPT" >> "$FAKE_LOG"\nexit 1\n'
FAKE_OK = '#!/bin/sh\necho "$0 $*" >> "$FAKE_LOG"\n'


def run_setup(tmp_path: Path):
    project = tmp_path / "project"
    (project / "scripts").mkdir(parents=True)
    shutil.copy(REPO / "scripts" / "setup_claude_code.sh", project / "scripts")
    hook = project / "scripts" / "no_ai_slop_check.sh"
    hook.write_text("#!/bin/sh\n")
    hook.chmod(0o644)
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    for name, body in (
        ("git", FAKE_GIT),
        ("claude", FAKE_OK),
        ("clangd", FAKE_OK),
        ("jq", FAKE_OK),
    ):
        (bin_dir / name).write_text(body)
        (bin_dir / name).chmod(0o755)
    log = tmp_path / "fake.log"
    env = {k: v for k, v in os.environ.items() if k != "GIT_TERMINAL_PROMPT"}
    env.update(
        PATH=f"{bin_dir}:{env['PATH']}",
        HOME=str(tmp_path / "home"),
        CLAUDE_CONFIG_DIR=str(tmp_path / "claude_config"),
        FAKE_LOG=str(log),
    )
    result = subprocess.run(
        ["bash", str(project / "scripts" / "setup_claude_code.sh")],
        env=env,
        capture_output=True,
        text=True,
        stdin=subprocess.DEVNULL,
    )
    return result, hook, log.read_text().splitlines()


def test_skill_clone_runs_with_git_prompts_disabled(tmp_path):
    result, _, lines = run_setup(tmp_path)
    assert result.returncode == 0, result.stderr
    clones = [line for line in lines if line.startswith("git clone")]
    assert clones and all(line.endswith("PROMPT=0") for line in clones)


def test_failed_skill_clone_warns_and_does_not_abort(tmp_path):
    result, _, _ = run_setup(tmp_path)
    assert result.returncode == 0
    assert "could not fetch the no-ai-slop skill" in result.stderr


def test_slop_hook_gets_its_exec_bit_back(tmp_path):
    result, hook, _ = run_setup(tmp_path)
    assert result.returncode == 0, result.stderr
    assert os.access(hook, os.X_OK)
