"""deploy_to_jetson.sh run against a fake ssh and a real rsync that writes to a temp dir."""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

from auto_battlebot.paths import REPO_ROOT

REPO = REPO_ROOT
FAKE_RSYNC = """#!/bin/bash
args=()
while [ "$#" -gt 0 ]; do
    if [ "$1" = "-e" ]; then shift 2; continue; fi
    args+=("$1"); shift
done
last=$((${#args[@]} - 1))
dest="${args[$last]}"
args[$last]="${dest#*:}"
exec /usr/bin/rsync "${args[@]}"
"""
FAKE_SSH = '#!/bin/sh\necho "$@" >> "$FAKE_SSH_LOG"\nexit 1\n'


def run_deploy(tmp_path: Path, gitignore: str) -> tuple[subprocess.CompletedProcess, Path]:
    project = tmp_path / "project"
    (project / "scripts").mkdir(parents=True)
    (project / "config").mkdir()
    (project / "config" / "profiles.toml").write_text("")
    shutil.copy(REPO / "scripts" / "deploy_to_jetson.sh", project / "scripts")
    (project / "src.txt").write_text("code\n")
    (project / ".gitignore").write_text(gitignore)
    (project / "ignored.txt").write_text("local\n")
    subprocess.run(["git", "init", "-q"], cwd=project, check=True)
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    for name, body in (("rsync", FAKE_RSYNC), ("ssh", FAKE_SSH)):
        (bin_dir / name).write_text(body)
        (bin_dir / name).chmod(0o755)
    dest = tmp_path / "remote"
    dest.mkdir()
    env = {
        **os.environ,
        "PATH": f"{bin_dir}:{os.environ['PATH']}",
        "HOME": str(tmp_path),
        "JETSON_USER": "tester",
        "JETSON_HOST": "fakehost",
        "JETSON_PATH": str(dest),
        "FAKE_SSH_LOG": str(tmp_path / "ssh.log"),
    }
    result = subprocess.run(
        ["bash", str(project / "scripts" / "deploy_to_jetson.sh"), "--sync-only"],
        env=env,
        capture_output=True,
        text=True,
        stdin=subprocess.DEVNULL,
    )
    return result, dest


def test_gitignored_build_version_still_reaches_the_remote(tmp_path):
    result, dest = run_deploy(tmp_path, ".build_version\nignored.txt\n")
    assert result.returncode == 0, result.stderr
    assert (dest / "src.txt").exists()
    assert (dest / ".build_version").read_text().strip() != ""


def test_other_gitignored_files_stay_off_the_remote(tmp_path):
    result, dest = run_deploy(tmp_path, ".build_version\nignored.txt\n")
    assert result.returncode == 0, result.stderr
    assert not (dest / "ignored.txt").exists()
    assert not (dest / ".git").exists()
