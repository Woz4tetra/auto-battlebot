"""auto_battlebot/datasets/clear_image_cache.py: in-use detection errs busy, ignores its shell."""

from __future__ import annotations

import os
from pathlib import Path
from types import ModuleType

import pytest

from auto_battlebot.datasets import clear_image_cache


@pytest.fixture(scope="module")
def cache() -> ModuleType:
    return clear_image_cache


def test_own_process_chain_excluded(cache) -> None:
    chain = cache.own_process_chain()
    assert os.getpid() in chain
    assert os.getppid() in chain


def test_cmdline_components_whole_parts(cache) -> None:
    components = cache.cmdline_components(
        "python\0training/yolo/validate_yolo_dataset.py\0data=../data/x.yml,../data/y.yml\0"
    )
    assert {"training", "yolo", "validate_yolo_dataset.py", "x.yml", "y.yml"} <= components
    assert "train" not in components
    assert "val" not in components
    assert not any("=" in component for component in components)


def _fake_proc(root: Path, cmdline: bytes) -> Path:
    proc = root / "proc"
    proc.mkdir()
    (proc / "cmdline").write_bytes(cmdline)
    os.symlink("/usr/bin/python3", proc / "exe")
    return proc


def test_relative_dataset_path_marks_busy(cache, tmp_path: Path) -> None:
    dataset = tmp_path / "scenesplit_2026-07-25"
    dataset.mkdir()
    proc = _fake_proc(tmp_path, b"python\0train.py\0../data/scenesplit_2026-07-25/old+new.yml\0")
    resolved = {dataset: (str(dataset.resolve()), dataset.name)}
    assert [d for d, _ in cache.cmdline_matches(proc, resolved)] == [dataset]


def test_generic_split_name_needs_its_parent_dataset(cache, tmp_path: Path) -> None:
    dataset = tmp_path / "corpus" / "train"
    dataset.mkdir(parents=True)
    proc = _fake_proc(tmp_path, b"python\0training/yolo/validate_yolo_dataset.py\0")
    resolved = {dataset: (str(dataset.resolve()), dataset.name)}
    assert cache.cmdline_matches(proc, resolved) == []


def test_name_match_is_whole_component_only(cache, tmp_path: Path) -> None:
    dataset = tmp_path / "old_dataset"
    dataset.mkdir()
    proc = _fake_proc(tmp_path, b"python\0train.py\0../data/old_dataset_v2/x.yml\0")
    resolved = {dataset: (str(dataset.resolve()), dataset.name)}
    assert cache.cmdline_matches(proc, resolved) == []


def test_own_process_chain_is_skipped_by_busy_scan(cache, tmp_path: Path, monkeypatch) -> None:
    import subprocess
    import sys

    dataset = tmp_path / "unique_dataset_name_7731"
    dataset.mkdir()
    child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)", dataset.name])
    try:
        assert child.pid in {
            p for holders in cache.busy_datasets([dataset]).values() for p in holders
        }
        monkeypatch.setattr(cache, "own_process_chain", lambda: {child.pid})
        assert cache.busy_datasets([dataset]) == {}
    finally:
        child.kill()
        child.wait()


def test_dry_run_cli_reports_stale_caches_and_deletes_nothing(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    import runpy

    from auto_battlebot.paths import REPO_ROOT

    images = tmp_path / "old_set" / "images" / "train"
    images.mkdir(parents=True)
    stale = []
    for i in range(3):
        npy = images / f"frame_{i}.npy"
        npy.write_bytes(b"\0" * 1000)
        os.utime(npy, (1_000_000_000, 1_000_000_000))
        stale.append(npy)
    argv = ["clear_image_cache.py", str(tmp_path), "--time-source", "mtime", "--dry-run"]
    monkeypatch.setattr("sys.argv", argv)
    runpy.run_path(str(REPO_ROOT / "training/yolo/clear_image_cache.py"), run_name="__main__")
    out = capsys.readouterr().out
    assert "dry run: would free" in out
    assert all(npy.exists() for npy in stale)
