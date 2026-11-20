import os
import subprocess
import sys
from pathlib import Path

import cv2
import pytest

ROOT = Path(__file__).resolve().parents[2]
ONNX = Path(
    os.environ.get(
        "ABT_PARITY_ONNX",
        ROOT / "data/models_v3/yolo26n_nhrl_robots_bbox_2class_2026-07-29.onnx",
    )
)
CPP_TEST = ROOT / "build-test/auto_battlebot_test"
FRAME_INDEX = 120
BOX_TOLERANCE_PX = 1.0
CONFIDENCE_TOLERANCE = 0.005


def parity_inputs() -> tuple[Path, Path]:
    video = os.environ.get("ABT_PARITY_VIDEO")
    if not video or not Path(video).exists() or not ONNX.exists() or not CPP_TEST.exists():
        pytest.skip("needs ABT_PARITY_VIDEO, ABT_PARITY_ONNX and build-test/")
    return Path(video), ONNX


def test_python_and_cpp_detectors_agree_on_one_frame(tmp_path):
    video, onnx = parity_inputs()
    pytest.importorskip("tensorrt")
    from auto_battlebot.perception.trt_yolo import TrtYoloModel

    engine = tmp_path / "parity.engine"
    subprocess.run(
        [sys.executable, str(ROOT / "training/yolo/convert_to_tensorrt.py"), str(onnx)]
        + ["-o", str(engine)],
        check=True,
    )
    built = next(tmp_path.glob("parity*.engine"))
    capture = cv2.VideoCapture(str(video))
    frame = None
    for _ in range(FRAME_INDEX + 1):
        ok, frame = capture.read()
        assert ok
    image_path = tmp_path / "frame.png"
    cv2.imwrite(str(image_path), frame)

    python_boxes = sorted(
        (d[0].tolist(), float(d[1])) for d in TrtYoloModel(str(built), 0.5).infer(frame)
    )
    out = tmp_path / "cpp.txt"
    env = {
        **os.environ,
        "ABT_PARITY_ENGINE": str(built),
        "ABT_PARITY_IMAGE": str(image_path),
        "ABT_PARITY_OUT": str(out),
    }
    subprocess.run(
        [str(CPP_TEST), "--gtest_filter=TrtYoloParityDumpTest.*"], check=True, env=env, cwd=ROOT
    )
    cpp_rows = sorted(
        ([float(v) for v in line.split()[:4]], float(line.split()[4]))
        for line in out.read_text().splitlines()
    )

    assert len(python_boxes) == len(cpp_rows) >= 2
    for (py_box, py_conf), (cpp_box, cpp_conf) in zip(python_boxes, cpp_rows, strict=True):
        assert max(abs(a - b) for a, b in zip(py_box, cpp_box, strict=True)) <= BOX_TOLERANCE_PX
        assert abs(py_conf - cpp_conf) <= CONFIDENCE_TOLERANCE
