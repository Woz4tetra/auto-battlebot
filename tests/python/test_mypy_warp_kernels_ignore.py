import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
WARP_MODULES = [
    "auto_battlebot/mujoco_sim/warp_kernels.py",
    "auto_battlebot/rl/envs/warp_kernels.py",
]
IGNORE_SECTION = re.compile(
    r"\[mypy-auto_battlebot\.[a-z_.]*warp_kernels\]\nignore_errors\s*=?\s*True\n\n?"
)


def run_mypy(config: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, "-m", "mypy", "--config-file", str(config), *WARP_MODULES],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )


def test_warp_kernel_annotations_need_the_ignore_errors_sections(tmp_path: Path) -> None:
    original = (ROOT / "mypy.ini").read_text()
    stripped = IGNORE_SECTION.sub("", original)
    assert stripped.count("warp_kernels") == 0
    config = tmp_path / "mypy_no_warp_ignore.ini"
    config.write_text(stripped)
    result = run_mypy(config)
    assert result.returncode != 0
    for module in WARP_MODULES:
        assert f"{module}:" in result.stdout


def test_ignore_errors_sections_silence_warp_kernels() -> None:
    result = run_mypy(ROOT / "mypy.ini")
    assert result.returncode == 0, result.stdout
