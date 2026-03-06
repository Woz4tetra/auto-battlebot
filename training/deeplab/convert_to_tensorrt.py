"""Convert a DeepLab .pth checkpoint (or .onnx model) to TensorRT engine format.

Accepts either a .pth checkpoint (which is first exported to ONNX internally)
or a pre-exported .onnx model (which skips straight to the TensorRT build).

Engines are GPU- and TensorRT-version specific: an engine built on x86 cannot
run on Jetson (or vice versa). For Jetson deployment, copy the .pth/.onnx model
to the Jetson and run this script there to produce the .engine used by the C++
app. Same TensorRT version (or VERSION_COMPATIBLE build) helps across patch
versions on the same platform.

Output filenames include a platform tag (e.g. _x86_64_sm89, _aarch64_sm72) that
encodes both CPU architecture and GPU compute capability so incompatible engines
are not loaded by accident.
"""

import argparse
import os
import platform
import shutil
import tempfile
from pathlib import Path

import torch

from load_deeplabv3 import build_model, SegModelWrapper
from model_config import load_model_config, config_path_for

import tensorrt as trt


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


def export_onnx(
    wrapper: SegModelWrapper,
    output_path: Path,
    input_size: int,
    opset_version: int = 18,
    dynamic_batch: bool = False,
    device: torch.device = torch.device("cuda"),
) -> None:
    """Export model to ONNX (used as intermediate for TensorRT)."""
    dummy_input = torch.randn(1, 3, input_size, input_size, device=device)

    kwargs: dict = {
        "input_names": ["input"],
        "output_names": ["output"],
        "opset_version": opset_version,
    }
    if dynamic_batch:
        kwargs["dynamic_axes"] = {
            "input": {0: "batch"},
            "output": {0: "batch"},
        }
    torch.onnx.export(
        wrapper,
        dummy_input,
        str(output_path),
        **kwargs,
    )


def build_tensorrt_engine(
    onnx_path: Path,
    engine_path: Path,
    *,
    fp16: bool = True,
    workspace_gib: int = 2,
    logger: trt.ILogger | None = None,
) -> None:
    """Build a TensorRT engine from an ONNX file."""
    if logger is None:
        logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    network = builder.create_network(
        1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    )
    parser = trt.OnnxParser(network, logger)

    onnx_path = Path(onnx_path)
    # parse_from_file so TensorRT can resolve external .onnx.data in the same directory
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


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert DeepLab .pth checkpoint or .onnx model to TensorRT engine"
    )
    parser.add_argument(
        "model",
        type=str,
        help="Path to DeepLab checkpoint (.pth) or pre-exported ONNX model (.onnx)",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        help="Output path for TensorRT engine; platform+GPU tag (e.g. _x86_64_sm89) is appended to stem (default: same name with .engine)",
    )
    parser.add_argument(
        "--opset",
        type=int,
        default=18,
        help="ONNX opset version for intermediate export (default: 18)",
    )
    parser.add_argument(
        "--keep-onnx",
        action="store_true",
        help="Keep the intermediate ONNX file (same path as engine with .onnx)",
    )
    parser.add_argument(
        "--no-fp16",
        action="store_true",
        help="Disable FP16; build FP32 engine only",
    )
    parser.add_argument(
        "--workspace",
        type=int,
        default=2,
        metavar="GIB",
        help="TensorRT workspace size in GiB (default: 2)",
    )
    args = parser.parse_args()

    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is required for TensorRT conversion")

    model_path = Path(args.model)
    if not model_path.exists():
        raise FileNotFoundError(f"Model file not found: {model_path}")

    is_onnx = model_path.suffix.lower() == ".onnx"
    if not is_onnx and model_path.suffix.lower() != ".pth":
        print(f"Warning: expected .pth or .onnx file, got {model_path.suffix}")

    cfg = load_model_config(model_path)
    device = torch.device("cuda")

    if args.output:
        output_path = Path(args.output)
    else:
        output_path = model_path.parent / f"{model_path.stem}.engine"
    output_path = engine_path_with_platform_tag(output_path)

    input_size = cfg.input_size
    print(
        f"Backbone: {cfg.backbone}, decoder: {cfg.decoder}, "
        f"num_classes: {cfg.num_classes}, input_size: {input_size}"
    )

    if is_onnx:
        onnx_path = model_path
        use_temp = False
        print(f"Using pre-exported ONNX model: {onnx_path}")
    else:
        print(f"Loading checkpoint from {model_path}...")
        wrapper = build_model(cfg.backbone, cfg.num_classes, device, decoder=cfg.decoder)
        wrapper.eval()
        state = torch.load(model_path, map_location=device, weights_only=True)
        wrapper.model.load_state_dict(state, strict=False)

        if args.keep_onnx:
            onnx_path = output_path.with_suffix(".onnx")
            use_temp = False
        else:
            onnx_fd, onnx_path_str = tempfile.mkstemp(suffix=".onnx")
            os.close(onnx_fd)
            onnx_path = Path(onnx_path_str)
            use_temp = True

        print(f"Exporting to ONNX (input shape: [1, 3, {input_size}, {input_size}])...")
        export_onnx(
            wrapper,
            onnx_path,
            input_size,
            opset_version=args.opset,
            dynamic_batch=False,
            device=device,
        )

    try:
        print("Building TensorRT engine...")
        build_tensorrt_engine(
            onnx_path,
            output_path,
            fp16=not args.no_fp16,
            workspace_gib=args.workspace,
        )
        shutil.copy2(config_path_for(model_path), config_path_for(output_path))
        print(f"Engine saved to: {output_path}")
        print(f"Config copied to: {config_path_for(output_path)}")
        print(
            "For Jetson: run this script on the Jetson to build an engine that runs there."
        )
    finally:
        if use_temp and onnx_path.exists():
            try:
                onnx_path.unlink()
            except OSError:
                pass


if __name__ == "__main__":
    main()
