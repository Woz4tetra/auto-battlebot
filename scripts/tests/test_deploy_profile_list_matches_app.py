"""The deploy script offers the same profiles the app's list_available_profiles() does."""

from __future__ import annotations

import re
import subprocess
from pathlib import Path

from auto_battlebot.compat import tomllib
from auto_battlebot.paths import REPO_ROOT

ROOT = REPO_ROOT
CONFIG_DIR = ROOT / "config"


def app_profiles(config_dir: Path, pattern: str) -> list[str]:
    ids = {
        path.relative_to(config_dir).with_suffix("").as_posix()
        for path in config_dir.rglob("*.toml")
        if path.is_file()
    }
    return sorted(i for i in ids if re.fullmatch(pattern, i))


def deploy_profiles(config_dir: Path, pattern: str) -> list[str]:
    out = subprocess.run(
        [str(ROOT / "scripts" / "list_deployable_profiles.sh"), str(config_dir), pattern],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    return out.split()


def test_real_config_matches_app() -> None:
    pattern = tomllib.loads((CONFIG_DIR / "profiles.toml").read_text())["profiles"]["pattern"]
    offered = deploy_profiles(CONFIG_DIR, pattern)
    assert offered
    assert offered == app_profiles(CONFIG_DIR, pattern)


def test_nested_and_base_files(tmp_path: Path) -> None:
    for name in ("robot_jetson", "_jetson", "sub/robot_jetson", "robot_desktop"):
        target = tmp_path / f"{name}.toml"
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text("")
    pattern = "[^_].*_jetson"
    expected = ["robot_jetson", "sub/robot_jetson"]
    assert deploy_profiles(tmp_path, pattern) == expected
    assert app_profiles(tmp_path, pattern) == expected
