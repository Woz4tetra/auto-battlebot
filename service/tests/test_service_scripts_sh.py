"""service/with_display.sh and build_and_install_service.sh run against fake systemctl and sudo."""

from __future__ import annotations

import os
import shutil
import subprocess
import threading
import time
from pathlib import Path

import pytest

from auto_battlebot.paths import REPO_ROOT

pytestmark = pytest.mark.jetson

REPO = REPO_ROOT
SERVICE_DIR = REPO / "service"

FAKE_SYSTEMCTL = """#!/bin/sh
echo "systemctl $*" >> "$FAKE_LOG"
case "$1" in
    is-enabled) [ "$FAKE_DESKTOP" = 1 ] && exit 0 || exit 1 ;;
    get-default) [ "$FAKE_DESKTOP" = 1 ] && echo graphical.target || echo multi-user.target ;;
    is-active) exit 1 ;;
esac
exit 0
"""
FAKE_SUDO = """#!/bin/sh
echo "sudo $*" >> "$FAKE_LOG"
if [ "$1" = cp ]; then cp "$2" "$FAKE_CAPTURE/$(basename "$3")"; fi
exit 0
"""
SHOW_DISPLAY = 'echo "DISPLAY=${DISPLAY-unset}"'


def make_bin(tmp_path: Path) -> Path:
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir(exist_ok=True)
    for name, body in (("systemctl", FAKE_SYSTEMCTL), ("sudo", FAKE_SUDO)):
        (bin_dir / name).write_text(body)
        (bin_dir / name).chmod(0o755)
    return bin_dir


def run_with_display(tmp_path: Path, *, desktop: bool, display: str | None, wait: str = "60"):
    sockets = tmp_path / "x11"
    sockets.mkdir(exist_ok=True)
    env = {k: v for k, v in os.environ.items() if k != "DISPLAY"}
    env.update(
        PATH=f"{make_bin(tmp_path)}:{env['PATH']}",
        FAKE_LOG=str(tmp_path / "fake.log"),
        FAKE_DESKTOP="1" if desktop else "0",
        X11_SOCKET_DIR=str(sockets),
        X_WAIT_SECONDS=wait,
    )
    if display is not None:
        env["DISPLAY"] = display
    cmd = ["bash", str(SERVICE_DIR / "with_display.sh"), "sh", "-c", SHOW_DISPLAY]
    started = time.monotonic()
    result = subprocess.run(cmd, env=env, capture_output=True, text=True, stdin=subprocess.DEVNULL)
    return result, time.monotonic() - started, sockets


def test_headless_box_runs_at_once_without_display(tmp_path):
    result, elapsed, _ = run_with_display(tmp_path, desktop=False, display=":0")
    assert result.stdout.strip() == "DISPLAY=unset"
    assert elapsed < 5


def test_existing_socket_for_the_set_display_is_kept(tmp_path):
    sockets = tmp_path / "x11"
    sockets.mkdir()
    (sockets / "X7").write_text("")
    result, _, _ = run_with_display(tmp_path, desktop=True, display=":7")
    assert result.stdout.strip() == "DISPLAY=:7"


def test_display_number_follows_whichever_socket_exists(tmp_path):
    sockets = tmp_path / "x11"
    sockets.mkdir()
    (sockets / "X1").write_text("")
    result, _, _ = run_with_display(tmp_path, desktop=True, display=":0")
    assert result.stdout.strip() == "DISPLAY=:1"


def test_desktop_box_waits_for_xorg_to_start(tmp_path):
    sockets = tmp_path / "x11"
    sockets.mkdir()
    threading.Timer(1.5, lambda: (sockets / "X1").write_text("")).start()
    result, elapsed, _ = run_with_display(tmp_path, desktop=True, display=None, wait="20")
    assert result.stdout.strip() == "DISPLAY=:1"
    assert 1.0 < elapsed < 10


def test_desktop_box_gives_up_after_the_wait_and_runs_without_display(tmp_path):
    result, elapsed, _ = run_with_display(tmp_path, desktop=True, display=None, wait="1")
    assert result.stdout.strip() == "DISPLAY=unset"
    assert "no X server after 1s" in result.stderr
    assert 1.0 <= elapsed < 10


def run_installer(tmp_path: Path, *, desktop: bool):
    project = tmp_path / "project"
    (project / "service").mkdir(parents=True)
    (project / "install").mkdir()
    (project / "scripts").mkdir()
    for path in SERVICE_DIR.glob("*.service"):
        shutil.copy(path, project / "service")
    shutil.copy(SERVICE_DIR / "build_and_install_service.sh", project / "service")
    for stub in ("install/rename_legacy_services.sh", "scripts/build_and_install.sh"):
        (project / stub).write_text("#!/bin/sh\nexit 0\n")
        (project / stub).chmod(0o755)
    capture = tmp_path / "capture"
    capture.mkdir()
    env = {k: v for k, v in os.environ.items() if k != "SUDO_USER"}
    env.update(
        PATH=f"{make_bin(tmp_path)}:{env['PATH']}",
        FAKE_LOG=str(tmp_path / "fake.log"),
        FAKE_CAPTURE=str(capture),
        FAKE_DESKTOP="1" if desktop else "0",
    )
    result = subprocess.run(
        ["bash", str(project / "service" / "build_and_install_service.sh")],
        env=env,
        capture_output=True,
        text=True,
        stdin=subprocess.DEVNULL,
    )
    log = tmp_path / "fake.log"
    return result, capture, (log.read_text().splitlines() if log.exists() else [])


def test_kiosk_unit_gets_the_numeric_uid_for_xdg_runtime_dir(tmp_path):
    result, capture, _ = run_installer(tmp_path, desktop=True)
    assert result.returncode == 0, result.stderr
    unit = (capture / "dashboard-kiosk.service").read_text()
    assert f'Environment="XDG_RUNTIME_DIR=/run/user/{os.getuid()}"' in unit
    assert "__UID__" not in unit
    assert "__USER__" not in unit


def test_installer_copies_reloads_and_enables_each_unit(tmp_path):
    result, capture, lines = run_installer(tmp_path, desktop=True)
    assert result.returncode == 0, result.stderr
    assert sorted(p.name for p in capture.iterdir()) == [
        "auto-battlebot.service",
        "dashboard-kiosk.service",
        "viz-relay.service",
    ]
    for name in ("viz-relay", "auto-battlebot", "dashboard-kiosk"):
        assert f"sudo systemctl enable {name}" in lines
    assert lines.count("sudo systemctl daemon-reload") == 3


def test_headless_box_installs_the_kiosk_unit_without_enabling_it(tmp_path):
    result, capture, lines = run_installer(tmp_path, desktop=False)
    assert result.returncode == 0, result.stderr
    assert (capture / "dashboard-kiosk.service").exists()
    assert "sudo systemctl enable dashboard-kiosk" not in lines
    assert "sudo systemctl enable viz-relay" in lines
