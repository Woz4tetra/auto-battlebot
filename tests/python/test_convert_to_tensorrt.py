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


def _frame(path: Path) -> None:
    import cv2
    import numpy as np

    rng = np.random.default_rng(0)
    cv2.imwrite(str(path), rng.integers(0, 255, (110, 200, 3), dtype=np.uint8))


def test_calibrator_feeds_cpp_letterbox_blob_from_a_held_buffer(converter, tmp_path: Path) -> None:
    import cv2
    import numpy as np
    import torch

    from auto_battlebot.perception.trt_yolo import CPP_LETTERBOX_PADDING, preprocess_frame

    frame = tmp_path / "a__frame_0000.jpg"
    _frame(frame)
    int8 = converter.Int8Config(frames=[frame], cache_path=tmp_path / "x.calib", algo="entropy2")
    calibrator = converter._make_calibrator(int8, 64, 96)
    pointers = calibrator.get_batch(["images"])
    assert pointers == [int(calibrator._buffer.data_ptr())]
    expected, *_ = preprocess_frame(
        cv2.imread(str(frame)), 64, 96, letterbox_padding=CPP_LETTERBOX_PADDING
    )
    got = calibrator._buffer.cpu()
    assert torch.equal(got, torch.from_numpy(np.ascontiguousarray(expected, dtype=np.float32)))
    assert calibrator.get_batch(["images"]) is None


def test_calibrator_buffer_is_cuda_resident_on_the_instance(converter, tmp_path: Path) -> None:
    int8 = converter.Int8Config(frames=[], cache_path=tmp_path / "x.calib", algo="minmax")
    calibrator = converter._make_calibrator(int8, 32, 32)
    assert calibrator._buffer.is_cuda
    assert tuple(calibrator._buffer.shape) == (1, 3, 32, 32)


def test_served_pointer_is_not_reused_by_a_later_cuda_allocation(converter, tmp_path: Path) -> None:
    import torch

    frame = tmp_path / "a__frame_0000.jpg"
    _frame(frame)
    int8 = converter.Int8Config(frames=[frame], cache_path=tmp_path / "x.calib", algo="entropy2")
    calibrator = converter._make_calibrator(int8, 64, 96)
    (pointer,) = calibrator.get_batch(["images"])
    later = torch.empty((1, 3, 64, 96), dtype=torch.float32, device="cuda")
    assert int(later.data_ptr()) != pointer


class _StubCalibrator:
    def __init__(self, served: int, cache_hit: bool = False) -> None:
        self.batches_served = served
        self.cache_hit = cache_hit


def test_short_calibration_run_fails_the_build(converter, tmp_path: Path) -> None:
    frames = [tmp_path / f"{i}.jpg" for i in range(5)]
    int8 = converter.Int8Config(frames=frames, cache_path=tmp_path / "x.calib", algo="entropy2")
    with pytest.raises(RuntimeError, match="consumed 3 of 5"):
        converter._check_calibration_ran(_StubCalibrator(3), int8)
    converter._check_calibration_ran(_StubCalibrator(5), int8)
    converter._check_calibration_ran(_StubCalibrator(0, cache_hit=True), int8)
    converter._check_calibration_ran(None, None)


def test_manifest_writes_non_ascii_names_under_an_ascii_locale(tmp_path: Path) -> None:
    import os
    import subprocess
    import sys

    script = (
        "import importlib.util, locale, pathlib, sys\n"
        "assert locale.getpreferredencoding(False).lower() in ('ascii', 'ansi_x3.4-1968'), "
        "locale.getpreferredencoding(False)\n"
        "spec = importlib.util.spec_from_file_location('conv', sys.argv[1])\n"
        "mod = importlib.util.module_from_spec(spec)\n"
        "spec.loader.exec_module(mod)\n"
        "frames = [pathlib.Path('a__frame_\\uff1f.jpg')]\n"
        "mod.write_frames_manifest(pathlib.Path(sys.argv[2]), frames)\n"
    )
    repo_root = Path(__file__).resolve().parents[2]
    manifest = tmp_path / "m.frames.txt"
    env = {
        **os.environ,
        "LC_ALL": "C",
        "LANG": "C",
        "PYTHONUTF8": "0",
        "PYTHONCOERCECLOCALE": "0",
    }
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            script,
            str(repo_root / "training/yolo/convert_to_tensorrt.py"),
            str(manifest),
        ],
        env=env,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    assert "？" in manifest.read_text(encoding="utf-8")


def _tiny_onnx(path: Path) -> None:
    import numpy as np
    import onnx
    from onnx import TensorProto, helper, numpy_helper

    weights = numpy_helper.from_array(np.ones((4, 3, 3, 3), dtype=np.float32), "w")
    graph = helper.make_graph(
        [helper.make_node("Conv", ["x", "w"], ["y"], pads=[1, 1, 1, 1])],
        "g",
        [helper.make_tensor_value_info("x", TensorProto.FLOAT, [1, 3, 32, 32])],
        [helper.make_tensor_value_info("y", TensorProto.FLOAT, [1, 4, 32, 32])],
        initializer=[weights],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)])
    onnx.save(model, str(path))


def test_unreadable_calibration_frame_fails_the_int8_build(converter, tmp_path: Path) -> None:
    onnx_path = tmp_path / "m.onnx"
    _tiny_onnx(onnx_path)
    frames = []
    for index in range(3):
        frame = tmp_path / f"a__frame_{index}.jpg"
        _frame(frame)
        frames.append(frame)
    frames[1].write_bytes(b"not a jpeg")
    int8 = converter.Int8Config(frames=frames, cache_path=tmp_path / "c.calib", algo="entropy2")
    with pytest.raises(RuntimeError, match="consumed 1 of 3"):
        converter.build_engine_from_onnx(
            onnx_path, tmp_path / "m.engine", fp16=False, workspace_gib=1, int8=int8
        )
