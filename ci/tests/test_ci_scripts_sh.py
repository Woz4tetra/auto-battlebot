from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest

from auto_battlebot.paths import REPO_ROOT

pytestmark = pytest.mark.jetson

FAKE_PIP = '#!/bin/sh\necho "pip $*" >> "$CALLS"\n'
FAKE_PYTHON = '#!/bin/sh\necho "python $*" >> "$CALLS"\nexit "${PYTHON_EXIT:-0}"\n'
FAKE_NPM = '#!/bin/sh\necho "npm $*" >> "$CALLS"\n'


def make_tool(path: Path, body: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(body)
    path.chmod(0o755)


def checkout(tmp_path: Path, python_exit: int = 0) -> tuple[Path, dict[str, str]]:
    """A job checkout with .keep placeholders, and a ci home holding the venv and data."""
    work = tmp_path / "work"
    for placeholder in ("data/svo/.keep", "data/models/.keep"):
        (work / placeholder).parent.mkdir(parents=True)
        (work / placeholder).write_text("")
    shutil.copytree(REPO_ROOT / "ci", work / "ci", ignore=shutil.ignore_patterns("tests"))
    home = tmp_path / "home"
    for name in ("svo", "models", "third_party/foxglove", "npm"):
        (home / "ci-data" / name).mkdir(parents=True)
    make_tool(home / "venv" / "bin" / "pip", FAKE_PIP)
    make_tool(home / "venv" / "bin" / "python", FAKE_PYTHON)
    make_tool(tmp_path / "bin" / "npm", FAKE_NPM)
    env = {
        **os.environ,
        "HOME": str(home),
        "PATH": f"{tmp_path / 'bin'}:{os.environ['PATH']}",
        "CALLS": str(tmp_path / "calls"),
        "PYTHON_EXIT": str(python_exit),
    }
    return work, env


def calls(tmp_path: Path) -> list[str]:
    return (tmp_path / "calls").read_text().splitlines()


def test_link_data_points_the_checkout_at_ci_data_and_repoints_the_venv(tmp_path: Path) -> None:
    work, env = checkout(tmp_path)
    result = subprocess.run(["ci/link_data.sh"], cwd=work, env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    home = Path(env["HOME"])
    assert (work / "venv").resolve() == (home / "venv").resolve()
    assert (work / "data" / "svo").resolve() == (home / "ci-data" / "svo").resolve()
    assert (work / "data" / "models").resolve() == (home / "ci-data" / "models").resolve()
    assert (work / "third_party" / "foxglove").resolve() == (
        home / "ci-data" / "third_party" / "foxglove"
    ).resolve()
    log = calls(tmp_path)
    assert log[0] == "pip install -q -e . --no-deps"
    assert log[1].startswith("python -c import auto_battlebot")
    assert log[2] == f"npm ci --prefix web --cache {home}/ci-data/npm --no-audit --no-fund"


def test_link_data_fails_when_the_venv_imports_another_checkout(tmp_path: Path) -> None:
    work, env = checkout(tmp_path, python_exit=1)
    result = subprocess.run(["ci/link_data.sh"], cwd=work, env=env, capture_output=True, text=True)
    assert result.returncode != 0
    assert not any(line.startswith("npm") for line in calls(tmp_path))


def test_build_engines_runs_the_engine_cache_from_the_linked_venv(tmp_path: Path) -> None:
    work, env = checkout(tmp_path)
    make_tool(work / "venv" / "bin" / "python", FAKE_PYTHON)
    result = subprocess.run(
        ["ci/build_engines.sh", "--cache", "/x"], cwd=work, env=env, capture_output=True, text=True
    )
    assert result.returncode == 0, result.stderr
    assert calls(tmp_path) == ["python -m auto_battlebot.verify.ci_engines build --cache /x"]
