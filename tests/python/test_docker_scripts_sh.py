"""scripts/docker/*.sh run against a fake docker that logs its argv and the env compose reads."""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
DOCKER_DIR = REPO / "scripts" / "docker"
COMPOSE_FILE = REPO / "docker" / "docker-compose.playback.yml"
FAKE_DOCKER = """#!/bin/sh
if [ "$1 $2 $3" = "compose version --short" ]; then echo 2.30.0; exit 0; fi
echo "ARGS $*" >> "$FAKE_DOCKER_LOG"
echo "ENV HOST_UID=$HOST_UID HOST_GID=$HOST_GID XAUTHORITY=$XAUTHORITY DISPLAY=$DISPLAY" \
    >> "$FAKE_DOCKER_LOG"
exit 0
"""


FAKE_ID = '#!/bin/sh\ncase "$1" in -u) echo 4242;; -g) echo 4343;; esac\n'


def run_script(tmp_path: Path, script: str, *args: str, env_extra=None, home_files=()):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir(exist_ok=True)
    (bin_dir / "docker").write_text(FAKE_DOCKER)
    (bin_dir / "docker").chmod(0o755)
    (bin_dir / "id").write_text(FAKE_ID)
    (bin_dir / "id").chmod(0o755)
    home = tmp_path / "home"
    home.mkdir(exist_ok=True)
    for name in home_files:
        (home / name).write_text("")
    log = tmp_path / "docker.log"
    env = {k: v for k, v in os.environ.items() if k not in {"XAUTHORITY", "DISPLAY"}}
    env.update(
        PATH=f"{bin_dir}:{env['PATH']}",
        HOME=str(home),
        FAKE_DOCKER_LOG=str(log),
        DISPLAY=":9",
        **(env_extra or {}),
    )
    result = subprocess.run(
        ["bash", str(DOCKER_DIR / script), *args],
        env=env,
        capture_output=True,
        text=True,
        stdin=subprocess.DEVNULL,
    )
    lines = log.read_text().splitlines() if log.exists() else []
    return result, lines


def args_line(lines):
    return next(line for line in lines if line.startswith("ARGS compose"))


def env_line(lines):
    return next(line for line in lines if line.startswith("ENV"))


def test_compose_sees_the_numeric_host_uid_and_gid(tmp_path):
    result, lines = run_script(tmp_path, "shell.sh", "true")
    assert result.returncode == 0, result.stderr
    assert "HOST_UID=4242 " in env_line(lines)
    assert "HOST_GID=4343 " in env_line(lines)


def test_xauthority_falls_back_to_dev_null_without_a_cookie(tmp_path):
    result, lines = run_script(tmp_path, "shell.sh", "true", env_extra={"XAUTHORITY": ""})
    assert result.returncode == 0, result.stderr
    assert "XAUTHORITY=/dev/null " in env_line(lines)


def test_xauthority_uses_the_home_cookie_when_present(tmp_path):
    result, lines = run_script(
        tmp_path, "shell.sh", "true", env_extra={"XAUTHORITY": ""}, home_files=[".Xauthority"]
    )
    assert result.returncode == 0, result.stderr
    assert f"XAUTHORITY={tmp_path / 'home' / '.Xauthority'} " in env_line(lines)


def test_compose_file_mounts_xauthority_unconditionally():
    text = COMPOSE_FILE.read_text()
    assert '"${XAUTHORITY}:${XAUTHORITY}:ro"' in text
    assert "profiles:" not in text


def test_no_display_blanks_display_and_leaves_the_mount_to_compose(tmp_path):
    result, lines = run_script(tmp_path, "shell.sh", "--no-display", "true")
    assert result.returncode == 0, result.stderr
    assert "DISPLAY= " in env_line(lines) + " "
    assert "XAUTHORITY=" in env_line(lines)
    assert "--no-display" not in args_line(lines)


def test_playback_adds_the_default_config_when_the_caller_gave_none(tmp_path):
    result, lines = run_script(tmp_path, "run_playback.sh")
    assert result.returncode == 0, result.stderr
    assert "-c config/playback/mrs_buff_mk3_playback.toml" in args_line(lines)


def test_playback_keeps_the_callers_config(tmp_path):
    result, lines = run_script(tmp_path, "run_playback.sh", "-c", "config/other.toml")
    assert result.returncode == 0, result.stderr
    assert "-c config/other.toml" in args_line(lines)
    assert "mrs_buff_mk3_playback" not in args_line(lines)


def test_compose_is_a_shell_function_so_scripts_cannot_exec_it():
    probe = f'source "{DOCKER_DIR / "docker_common.sh"}"; type -t compose'
    result = subprocess.run(["bash", "-c", probe], capture_output=True, text=True)
    assert result.stdout.strip().splitlines()[-1] == "function"
