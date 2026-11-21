from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

from auto_battlebot.paths import REPO_ROOT

ROOT = REPO_ROOT
PROJECT = ROOT / "firmware/mr_stabs_mk2"


def coredump_range(project: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(project / "scripts/coredump"), "--range"], capture_output=True, text=True, check=False
    )


def test_range_is_the_coredump_row_of_partitions_csv() -> None:
    result = coredump_range(PROJECT)
    assert result.returncode == 0
    assert result.stdout.split() == ["0x310000", "0x10000"]


def test_range_follows_a_moved_partition(tmp_path: Path) -> None:
    (tmp_path / "scripts").mkdir()
    shutil.copy(PROJECT / "scripts/coredump", tmp_path / "scripts/coredump")
    (tmp_path / "partitions.csv").write_text(
        "# Name, Type, SubType, Offset, Size\n"
        "nvs,      data, nvs,      0x9000,   20K,\n"
        "coredump, data, coredump, 0x330000, 128K,\n"
    )
    result = coredump_range(tmp_path)
    assert result.returncode == 0
    assert result.stdout.split() == ["0x330000", "0x20000"]


def test_range_fails_without_a_coredump_row(tmp_path: Path) -> None:
    (tmp_path / "scripts").mkdir()
    shutil.copy(PROJECT / "scripts/coredump", tmp_path / "scripts/coredump")
    (tmp_path / "partitions.csv").write_text("nvs, data, nvs, 0x9000, 20K,\n")
    assert coredump_range(tmp_path).returncode == 1
