"""download_recordings.sh --list against a fake ssh that runs the remote script locally."""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
FAKE_SSH = """#!/bin/sh
for last; do :; done
cd "$FAKE_REMOTE_ROOT" && exec sh -c "$last"
"""
FAKE_LSOF = "#!/bin/sh\nexit 0\n"


def run_list(tmp_path: Path, remote_files: list[str], local_files: list[str] = ()):
    project = tmp_path / "project"
    (project / "scripts").mkdir(parents=True)
    shutil.copy(REPO / "scripts" / "download_recordings.sh", project / "scripts")
    for rel in local_files:
        (project / rel).parent.mkdir(parents=True, exist_ok=True)
        (project / rel).write_text("")
    remote = tmp_path / "remote"
    remote.mkdir()
    for rel in remote_files:
        (remote / rel).parent.mkdir(parents=True, exist_ok=True)
        (remote / rel).write_text("")
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    for name, body in (("ssh", FAKE_SSH), ("lsof", FAKE_LSOF)):
        (bin_dir / name).write_text(body)
        (bin_dir / name).chmod(0o755)
    env = {
        **os.environ,
        "PATH": f"{bin_dir}:{os.environ['PATH']}",
        "HOME": str(tmp_path),
        "JETSON_USER": "tester",
        "JETSON_HOST": "fakehost",
        "JETSON_PATH": str(remote),
        "FAKE_REMOTE_ROOT": str(remote),
    }
    return subprocess.run(
        ["bash", str(project / "scripts" / "download_recordings.sh"), "--list", "--hours", "0"],
        env=env,
        capture_output=True,
        text=True,
        stdin=subprocess.DEVNULL,
    )


def test_lists_recordings_when_most_search_dirs_are_missing_remotely(tmp_path):
    result = run_list(tmp_path, ["data/svo/a.svo2"])
    assert result.returncode == 0, result.stderr
    assert "1 remote recording(s) found" in result.stdout
    assert "data/svo/a.svo2" in result.stdout


def test_reports_nothing_when_no_search_dir_exists_remotely(tmp_path):
    result = run_list(tmp_path, ["stray.svo2"])
    assert result.returncode == 0, result.stderr
    assert "0 remote recording(s) found" in result.stdout


def test_recordings_already_held_locally_are_not_listed(tmp_path):
    result = run_list(
        tmp_path, ["data/svo/a.svo2", "data/recordings/b.mcap"], local_files=["data/svo/a.svo2"]
    )
    assert result.returncode == 0, result.stderr
    assert "New: 0 SVO(s) -> data/svo, 1 MCAP(s)" in result.stdout
