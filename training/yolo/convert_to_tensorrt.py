"""Convert a YOLO pose model (.pt or .onnx) to TensorRT engine format.

Engines are GPU- and TensorRT-version specific: an engine built on x86 cannot
run on Jetson (or vice versa). For Jetson deployment, copy the .pt or .onnx
to the Jetson and run this script there (e.g. --from-onnx if you have .onnx)
to produce the .engine used by the C++ app.

The C++ YoloKeypointModel expects:
- Input: single tensor, shape [1, 3, H, W] (NCHW, float32), e.g. [1, 3, 640, 640].
- Output: single tensor, shape [1, num_features, num_predictions], e.g. [1, 56, 8400]
  (features = 4 bbox + num_classes + num_keypoints*3).

Export from .pt uses Ultralytics model.export(format="engine"). Export from .onnx
uses TensorRT Builder + OnnxParser (no Ultralytics required).

For C++ YoloKeypointModel compatibility, prefer building from ONNX (--from-onnx):
  python training/yolo/convert_to_onnx.py model.pt
  python training/yolo/convert_to_tensorrt.py model.onnx --from-onnx -o models/model.engine
Engines built from .pt via Ultralytics may use a different plan format and fail to load in the C++ runtime.

Output filenames include a platform tag (e.g. _x86_64_sm89, _aarch64_sm72) that
encodes both CPU architecture and GPU compute capability so incompatible engines
are not loaded by accident.
"""

import argparse
import ctypes
import platform
from pathlib import Path

import tensorrt as trt
import torch


def _has_lean_runtime() -> bool:
    """Check if the TensorRT lean runtime is available (needed for VERSION_COMPATIBLE)."""
    try:
        ctypes.CDLL("libnvinfer_lean.so.10")
        return True
    except OSError:
        return False


def _get_compute_capability() -> tuple[int, int]:
    """Query GPU compute capability of device 0."""
    if torch.cuda.is_available():
        return torch.cuda.get_device_capability(0)
    raise RuntimeError("CUDA not available — cannot determine GPU compute capability")


def engine_path_with_platform_tag(path: Path) -> Path:
    """Append platform + GPU compute capability tag so incompatible engines are distinct.

    Produces filenames like ``model_x86_64_sm89.engine`` — the ``sm`` tag
    prevents silently loading an engine built for a different GPU architecture.
    """
    arch = platform.machine()
    major, minor = _get_compute_capability()
    tag = f"{arch}_sm{major}{minor}"
    suffix = path.suffix if path.suffix else ".engine"
    return path.parent / f"{path.stem}_{tag}{suffix}"


def build_engine_from_onnx(
    onnx_path: Path,
    engine_path: Path,
    *,
    fp16: bool = True,
    workspace_gib: int = 4,
    logger: trt.ILogger | None = None,
) -> None:
    """Build a TensorRT engine from an ONNX file (fixed input shape)."""
    if logger is None:
        logger = trt.Logger(trt.Logger.INFO)
    builder = trt.Builder(logger)
    network = builder.create_network(
        1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    )
    parser = trt.OnnxParser(network, logger)

    onnx_path = Path(onnx_path)
    if not parser.parse_from_file(str(onnx_path)):
        for i in range(parser.num_errors):
            print(parser.get_error(i))
        raise RuntimeError("Failed to parse ONNX file")

    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, workspace_gib << 30)
    if fp16 and builder.platform_has_fast_fp16:
        config.set_flag(trt.BuilderFlag.FP16)
    if hasattr(trt.BuilderFlag, "VERSION_COMPATIBLE") and _has_lean_runtime():
        config.set_flag(trt.BuilderFlag.VERSION_COMPATIBLE)

    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise RuntimeError("Failed to build TensorRT engine")
    engine_path = Path(engine_path)
    engine_path.parent.mkdir(parents=True, exist_ok=True)
    with open(engine_path, "wb") as f:
        f.write(serialized)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert YOLO pose model to TensorRT engine for C++ YoloKeypointModel"
    )
    parser.add_argument(
        "model",
        type=str,
        help="Path to ONNX file (.onnx)",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        help="Output path for TensorRT engine; platform+GPU tag (e.g. _x86_64_sm89) is appended to stem (default: same name with .engine)",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="Image size H=W for export (default: 640). Must match C++ config image_size.",
    )
    parser.add_argument(
        "--no-fp16",
        action="store_true",
        help="Disable FP16; build FP32 engine only",
    )
    parser.add_argument(
        "--workspace",
        type=int,
        default=4,
        metavar="GIB",
        help="TensorRT workspace size in GiB (default: 4)",
    )
    args = parser.parse_args()

    model_path = Path(args.model)
    if not model_path.exists():
        raise FileNotFoundError(f"Model file not found: {model_path}")

    if args.output:
        output_path = Path(args.output)
    else:
        output_path = model_path.parent / f"{model_path.stem}.engine"
    output_path = engine_path_with_platform_tag(output_path)

    if model_path.suffix.lower() != ".onnx":
        raise ValueError("This script requires an .onnx file path")
    print(f"TensorRT version: {trt.__version__}")
    print(f"Building TensorRT engine from ONNX: {model_path}")
    print(f"Input size: [1, 3, {args.imgsz}, {args.imgsz}]")
    build_engine_from_onnx(
        model_path,
        output_path,
        fp16=not args.no_fp16,
        workspace_gib=args.workspace,
    )
    print(f"Engine saved to: {output_path}")

    print("Done. Use the .engine path as model_path in config for YoloKeypointModel.")
    print(
        "For Jetson: run this script on the Jetson to build an engine that runs there."
    )


if __name__ == "__main__":
    main()
