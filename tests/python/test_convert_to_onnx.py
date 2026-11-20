"""training/yolo/convert_to_onnx.py: raw-head export lands exactly at the requested path."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import onnx
import pytest
from ultralytics import YOLO


@pytest.fixture(scope="module")
def exporter(load_training):
    return load_training("training/yolo/convert_to_onnx.py")


@pytest.fixture(scope="module")
def checkpoint(tmp_path_factory: pytest.TempPathFactory) -> Path:
    path = tmp_path_factory.mktemp("ckpt") / "tiny_last.pt"
    YOLO("yolo26n.yaml").save(str(path))
    return path


def _export(checkpoint: Path, out: Path, imgsz: int | list[int]) -> None:
    script = (
        "import importlib.util, sys, json\n"
        "from pathlib import Path\n"
        "spec = importlib.util.spec_from_file_location('conv', sys.argv[1])\n"
        "mod = importlib.util.module_from_spec(spec)\n"
        "spec.loader.exec_module(mod)\n"
        "mod.export_onnx(Path(sys.argv[2]), Path(sys.argv[3]), json.loads(sys.argv[4]))\n"
    )
    converter = Path(__file__).resolve().parents[2] / "training/yolo/convert_to_onnx.py"
    subprocess.run(
        [sys.executable, "-c", script, str(converter), str(checkpoint), str(out), str(imgsz)],
        check=True,
        capture_output=True,
    )


def _output_dims(path: Path) -> list[int]:
    graph = onnx.load(str(path)).graph
    return [dim.dim_value for dim in graph.output[0].type.tensor_type.shape.dim]


def test_export_is_raw_head_layout(exporter, checkpoint: Path, tmp_path: Path) -> None:
    out = tmp_path / "raw.onnx"
    _export(checkpoint, out, 64)
    batch, features, anchors = _output_dims(out)
    assert batch == 1
    assert features == 4 + 80
    assert anchors == (8 * 8) + (4 * 4) + (2 * 2)


def test_disable_end2end_clears_every_head_flag(exporter, checkpoint: Path) -> None:
    model = YOLO(str(checkpoint))
    heads = [m for m in model.model.modules() if hasattr(m, "end2end")]
    assert heads
    assert all(head.end2end for head in heads)
    exporter.disable_end2end(model)
    assert not any(head.end2end for head in heads)


def test_export_order_does_not_lose_either_onnx(exporter, checkpoint: Path, tmp_path: Path) -> None:
    default = checkpoint.with_suffix(".onnx")
    rect = tmp_path / "rect.onnx"
    _export(checkpoint, default, 64)
    _export(checkpoint, rect, [64, 96])
    assert default.is_file()
    assert rect.is_file()
    assert _output_dims(default)[2] != _output_dims(rect)[2]
    assert sorted(p.name for p in checkpoint.parent.iterdir()) == [default.name, checkpoint.name]
