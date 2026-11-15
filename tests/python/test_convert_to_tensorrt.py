"""training/yolo/convert_to_tensorrt.py: calibration sampling and cache guards."""

from __future__ import annotations

import argparse
from pathlib import Path

import pytest


@pytest.fixture(scope="module")
def converter(load_training):
    return load_training("training/yolo/convert_to_tensorrt.py")


def _corpus(root: Path) -> None:
    root.mkdir()
    for scene, count in (("a", 40), ("b", 25), ("c", 3)):
        for index in range(count):
            (root / f"{scene}__frame_{index:04d}.jpg").write_bytes(b"x")


def test_calibration_partitions_are_disjoint(converter, tmp_path: Path) -> None:
    _corpus(tmp_path / "val")
    first = converter.select_calibration_frames(tmp_path / "val", 20, (0, 2))
    second = converter.select_calibration_frames(tmp_path / "val", 20, (1, 2))
    assert len(first) == len(second) == 20
    assert not set(first) & set(second)


def test_mismatched_cache_without_frames_is_refused(converter, tmp_path: Path) -> None:
    cache = tmp_path / "x.calib"
    cache.write_text("TRT-8000-EntropyCalibration2\nabc\n")
    algo = next(iter(converter.CALIB_CACHE_ALGO_NAMES))
    with pytest.raises(SystemExit):
        converter.check_cache_header(cache, algo, have_frames=False)
    converter.check_cache_header(cache, algo, have_frames=True)


def test_cache_name_includes_sample(converter, tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr(converter, "DEFAULT_INT8_CACHE_DIR", tmp_path / "cache")
    _corpus(tmp_path / "frames")

    def args(count: int, partition: str | None) -> argparse.Namespace:
        return argparse.Namespace(
            int8=True,
            calib_partition=partition,
            calib_count=count,
            calib_cache=None,
            calib_dir=str(tmp_path / "frames"),
            calib_algo=next(iter(converter.CALIB_CACHE_ALGO_NAMES)),
        )

    model = tmp_path / "m.onnx"
    names = {
        converter.resolve_int8_config(model, args(10, None)).cache_path,
        converter.resolve_int8_config(model, args(5, None)).cache_path,
        converter.resolve_int8_config(model, args(10, "0/2")).cache_path,
        converter.resolve_int8_config(model, args(10, "1/2")).cache_path,
    }
    assert len(names) == 4
