"""The install-script lint rejects options and traps that leak into the sourcing shell."""

from __future__ import annotations

import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CHECK = ROOT / "scripts" / "check_install_option_leaks.sh"


def run_check(root: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run([str(CHECK), str(root)], capture_output=True, text=True)


def make_tree(tmp_path: Path, files: dict[str, str]) -> Path:
    (tmp_path / "install").mkdir()
    (tmp_path / "scripts").mkdir()
    for name, body in files.items():
        (tmp_path / name).write_text(body)
    return tmp_path


def test_repo_install_scripts_are_clean() -> None:
    result = run_check(ROOT)
    assert result.returncode == 0, result.stdout


def test_pipefail_in_helper_is_rejected(tmp_path: Path) -> None:
    root = make_tree(tmp_path, {"install/install_x.sh": "f() {\n    set -o pipefail\n}\n"})
    result = run_check(root)
    assert result.returncode == 1
    assert "install_x.sh:2" in result.stdout


def test_trap_in_script_install_is_rejected(tmp_path: Path) -> None:
    root = make_tree(tmp_path, {"scripts/install_y.sh": "trap 'echo x' EXIT\n"})
    assert run_check(root).returncode == 1


def test_set_e_fail_loudly_and_opencv_are_allowed(tmp_path: Path) -> None:
    root = make_tree(
        tmp_path,
        {
            "install/install_x.sh": "set -e\n    set -e  # again\n",
            "install/fail_loudly.sh": "set -eE\ntrap 'echo x' ERR\n",
            "install/install_opencv.sh": "(\n    set -euo pipefail\n)\n",
        },
    )
    assert run_check(root).returncode == 0
