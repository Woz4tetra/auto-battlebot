"""Convert a YOLO pose model (.pt or .onnx) to TensorRT engine format.

Always uses a two-step process for C++ runtime compatibility:
  1. .pt → ONNX (via Ultralytics), then
  2. ONNX → TensorRT engine (via TensorRT Builder + OnnxParser).

Engines are GPU- and TensorRT-version specific: an engine built on x86 cannot
run on Jetson (or vice versa). For Jetson deployment, copy the .pt or .onnx
to the Jetson and run this script there to produce the .engine used by the C++ app.

The C++ YoloKeypointModel expects:
- Input: single tensor, shape [1, 3, H, W] (NCHW, float32), e.g. [1, 3, 640, 640].
- Output: single tensor, shape [1, num_features, num_predictions], e.g. [1, 56, 8400]
  (features = 4 bbox + num_classes + num_keypoints*3).

Usage:
  python training/yolo/convert_to_tensorrt.py model.pt -o models/model.engine
  python training/yolo/convert_to_tensorrt.py model.onnx -o models/model.engine

Output filenames include a platform tag (e.g. _x86_64, _aarch64) so the wrong
engine is not used by accident.
"""

import argparse
import platform
import sys
from pathlib import Path


def engine_path_with_platform_tag(path: Path) -> Path:
    """Append platform tag to engine path stem so x86 vs Jetson engines are distinct."""
    tag = platform.machine()
    suffix = path.suffix if path.suffix else ".engine"
    return path.parent / f"{path.stem}_{tag}{suffix}"


# Prefer trt_onnx_builder (C++ nanobind extension) so engines match C++ runtime plan format.
_trt_onnx_builder = None
_parent_dir = Path(__file__).resolve().parent.parent
# Check training/yolo/trt_onnx_builder/build then repo_root/trt_onnx_builder/build
_bindings_build = _parent_dir / "trt_onnx_builder" / "build"
if _bindings_build.exists() and str(_bindings_build) not in sys.path:
    sys.path.insert(0, str(_bindings_build))
import trt_onnx_builder as _trt_onnx_builder


def _build_instructions() -> str:
    return (
        "To use the C++ TensorRT engine builder (recommended for C++ runtime compatibility),\n"
        "build the trt_onnx_builder extension and add its build dir to PYTHONPATH:\n"
        "  cd training/yolo/trt_onnx_builder && mkdir -p build && cd build && cmake .. && make -j$(nproc)\n"
        '  export PYTHONPATH="$(pwd)/training/yolo/trt_onnx_builder/build:$PYTHONPATH"\n'
        "See training/yolo/trt_onnx_builder/README.md for details."
    )


def build_engine_from_onnx(
    onnx_path: Path,
    engine_path: Path,
    *,
    fp16: bool = True,
    workspace_gib: int = 4,
) -> None:
    """Build a TensorRT engine from an ONNX file (fixed input shape). Uses C++ extension if available."""
    onnx_path = Path(onnx_path)
    engine_path = Path(engine_path)
    engine_path.parent.mkdir(parents=True, exist_ok=True)

    if _trt_onnx_builder is not None:
        _trt_onnx_builder.build_engine_from_onnx(
            str(onnx_path), str(engine_path), fp16=fp16, workspace_gib=workspace_gib
        )
        return

    print(_build_instructions())
    raise RuntimeError("trt_onnx_builder not found; using Python TensorRT")


def export_pt_to_onnx(
    model_path: Path,
    onnx_path: Path,
    *,
    imgsz: int = 640,
) -> Path:
    """Export a YOLO .pt model to ONNX using Ultralytics."""
    from ultralytics import YOLO

    model_path = Path(model_path)
    if not model_path.exists():
        raise FileNotFoundError(f"Model file not found: {model_path}")
    if model_path.suffix.lower() not in (".pt", ".pth"):
        print(f"Warning: expected .pt file, got {model_path.suffix}")

    print(f"Step 1: Exporting .pt to ONNX: {model_path} -> {onnx_path}")
    model = YOLO(str(model_path))
    model.to("cuda")
    exported = model.export(format="onnx", imgsz=imgsz, device="cuda")
    exported_path = Path(exported)
    onnx_path = Path(onnx_path)
    onnx_path.parent.mkdir(parents=True, exist_ok=True)
    if exported_path.resolve() != onnx_path.resolve():
        import shutil

        shutil.copy(exported_path, onnx_path)
        print(f"ONNX saved to: {onnx_path}")
    else:
        print(f"ONNX saved to: {onnx_path}")
    return onnx_path


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert YOLO pose model to TensorRT engine for C++ YoloKeypointModel"
    )
    parser.add_argument(
        "model",
        type=str,
        help="Path to YOLO model (.pt) or existing ONNX file (.onnx). For .pt, ONNX is exported first then engine is built.",
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
        default=16,
        metavar="GIB",
        help="TensorRT workspace size in GiB (default: 16)",
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

    if model_path.suffix.lower() == ".onnx":
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
            print(f"Warning: expected .pt or .onnx, got {model_path.suffix}")
        # Two-step: .pt -> ONNX -> engine (for C++ runtime compatibility)
        onnx_path = model_path.parent / f"{model_path.stem}.onnx"
        export_pt_to_onnx(model_path, onnx_path, imgsz=args.imgsz)
        print(f"Step 2: Building TensorRT engine from ONNX: {onnx_path}")
        print(f"Input size: [1, 3, {args.imgsz}, {args.imgsz}]")
        build_engine_from_onnx(
            onnx_path,
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
