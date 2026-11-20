"""training/deeplab/convert_to_tensorrt.py: external-data ONNX models parse from their path."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import onnx
import pytest
import tensorrt as trt
from onnx import TensorProto, helper, numpy_helper


@pytest.fixture(scope="module")
def converter(load_training):
    return load_training("training/deeplab/convert_to_tensorrt.py")


def _external_data_model(directory: Path) -> Path:
    weights = numpy_helper.from_array(np.ones((4, 3, 1, 1), dtype=np.float32), "w")
    graph = helper.make_graph(
        [helper.make_node("Conv", ["x", "w"], ["y"])],
        "g",
        [helper.make_tensor_value_info("x", TensorProto.FLOAT, [1, 3, 8, 8])],
        [helper.make_tensor_value_info("y", TensorProto.FLOAT, [1, 4, 8, 8])],
        initializer=[weights],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)])
    path = directory / "m.onnx"
    onnx.save_model(
        model,
        str(path),
        save_as_external_data=True,
        all_tensors_to_one_file=True,
        location="m.onnx.data",
        size_threshold=0,
    )
    assert (directory / "m.onnx.data").is_file()
    return path


def test_in_memory_parse_cannot_resolve_external_weights(tmp_path: Path) -> None:
    path = _external_data_model(tmp_path)
    logger = trt.Logger(trt.Logger.ERROR)
    network = trt.Builder(logger).create_network(
        1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    )
    assert not trt.OnnxParser(network, logger).parse(path.read_bytes())
    network = trt.Builder(logger).create_network(
        1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    )
    assert trt.OnnxParser(network, logger).parse_from_file(str(path))


def test_engine_builds_from_external_data_onnx(converter, tmp_path: Path) -> None:
    path = _external_data_model(tmp_path)
    engine = tmp_path / "out" / "m.engine"
    converter.build_tensorrt_engine(path, engine, workspace_gib=1, fp16=False)
    assert engine.stat().st_size > 0
