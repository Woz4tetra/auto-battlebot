from __future__ import annotations

import subprocess
from pathlib import Path

from auto_battlebot.paths import REPO_ROOT
from auto_battlebot.script_loader import load_script

build_script = load_script("firmware/mr_stabs_mk2/scripts/firmware_build.py")


def git(repo: Path, *args: str) -> str:
    return subprocess.run(
        ["git", "-C", str(repo), *args], capture_output=True, text=True, check=True
    ).stdout.strip()


def make_repo(tmp_path: Path) -> Path:
    git(tmp_path, "init", "-q")
    git(tmp_path, "config", "user.email", "test@example.com")
    git(tmp_path, "config", "user.name", "test")
    (tmp_path / "a.txt").write_text("one\n")
    git(tmp_path, "add", "a.txt")
    git(tmp_path, "commit", "-q", "-m", "one")
    return tmp_path


def test_clean_tree_builds_as_the_short_hash(tmp_path: Path) -> None:
    repo = make_repo(tmp_path)
    assert build_script.firmware_build(repo) == git(repo, "rev-parse", "--short=8", "HEAD")


def test_edited_tree_is_marked_dirty_and_fits_the_payload(tmp_path: Path) -> None:
    repo = make_repo(tmp_path)
    (repo / "a.txt").write_text("two\n")
    build = build_script.firmware_build(repo)
    assert build.endswith("-dirty")
    assert len(build) <= build_script.FIRMWARE_BUILD_LENGTH


def test_no_repository_reads_unknown(tmp_path: Path) -> None:
    assert build_script.firmware_build(tmp_path / "missing") == "unknown"


def test_length_matches_the_wire_payload() -> None:
    header = (REPO_ROOT / "firmware/shared/espnow_protocol/src/espnow_protocol.h").read_text()
    assert f"FIRMWARE_BUILD_LENGTH = {build_script.FIRMWARE_BUILD_LENGTH};" in header
