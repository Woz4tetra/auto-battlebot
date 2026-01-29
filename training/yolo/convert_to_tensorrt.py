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
  python training/yolo/convert_to_tensorrt.py model.onnx --from-onnx -o models/model_x86_64.engine
Engines built from .pt via Ultralytics may use a different plan format and fail to load in the C++ runtime.

Output filenames include a platform tag (e.g. _x86_64, _aarch64) so the wrong
engine is not used by accident.
"""

import argparse
import platform
from pathlib import Path


def engine_path_with_platform_tag(path: Path) -> Path:
    """Append platform tag to engine path stem so x86 vs Jetson engines are distinct."""
    tag = platform.machine()
    suffix = path.suffix if path.suffix else ".engine"
    return path.parent / f"{path.stem}_{tag}{suffix}"

try:
    import tensorrt as trt
except ImportError as e:
    raise ImportError(
        "TensorRT is not installed. Install it (e.g. pip install tensorrt, or use JetPack on Jetson)"
    ) from e


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
        logger = trt.Logger(trt.Logger.WARNING)
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
    # Build version-compatible engine so it can load on runtimes with a different TensorRT patch version.
    if hasattr(trt.BuilderFlag, "VERSION_COMPATIBLE"):
        config.set_flag(trt.BuilderFlag.VERSION_COMPATIBLE)

    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise RuntimeError("Failed to build TensorRT engine")
    engine_path = Path(engine_path)
    engine_path.parent.mkdir(parents=True, exist_ok=True)
    with open(engine_path, "wb") as f:
        f.write(serialized)


def export_pt_to_engine(
    model_path: Path,
    output_path: Path,
    *,
    imgsz: int = 640,
    fp16: bool = True,
    workspace_gib: int = 4,
) -> Path:
    """Export a YOLO .pt model to TensorRT engine using Ultralytics."""
    from ultralytics import YOLO

    model_path = Path(model_path)
    if not model_path.exists():
        raise FileNotFoundError(f"Model file not found: {model_path}")
    if model_path.suffix.lower() not in (".pt", ".pth"):
        print(f"Warning: expected .pt file, got {model_path.suffix}")

    model = YOLO(str(model_path))
    model.to("cuda")

    # Ultralytics export(format="engine") writes to same dir as model by default
    exported = model.export(
        format="engine",
        imgsz=imgsz,
        half=fp16,
        workspace=workspace_gib,
        simplify=True,
    )
    exported_path = Path(exported)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if exported_path.resolve() != output_path.resolve():
        import shutil

        shutil.copy(exported_path, output_path)
        print(f"Engine copied to: {output_path}")
    else:
        print(f"Engine saved to: {output_path}")
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert YOLO pose model to TensorRT engine for C++ YoloKeypointModel"
    )
    parser.add_argument(
        "model",
        type=str,
        help="Path to YOLO model (.pt) or ONNX file (.onnx)",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        help="Output path for TensorRT engine; platform tag (e.g. _x86_64) is appended to stem (default: same name with .engine)",
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
    parser.add_argument(
        "--from-onnx",
        action="store_true",
        help="Build engine from existing ONNX file (model path must be .onnx)",
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

    if args.from_onnx:
        if model_path.suffix.lower() != ".onnx":
            raise ValueError("--from-onnx requires an .onnx file path")
        print(f"Building TensorRT engine from ONNX: {model_path}")
        print(f"Input size: [1, 3, {args.imgsz}, {args.imgsz}]")
        build_engine_from_onnx(
            model_path,
            output_path,
            fp16=not args.no_fp16,
            workspace_gib=args.workspace,
        )
        print(f"Engine saved to: {output_path}")
    else:
        if model_path.suffix.lower() not in (".pt", ".pth"):
            print(f"Warning: expected .pt for Ultralytics export, got {model_path.suffix}")
        print(f"Exporting YOLO model to TensorRT engine: {model_path}")
        print(f"Image size: {args.imgsz}")
        export_pt_to_engine(
            model_path,
            output_path,
            imgsz=args.imgsz,
            fp16=not args.no_fp16,
            workspace_gib=args.workspace,
        )

    print("Done. Use the .engine path as model_path in config for YoloKeypointModel.")
    print("For Jetson: run this script on the Jetson to build an engine that runs there.")


if __name__ == "__main__":
    main()
