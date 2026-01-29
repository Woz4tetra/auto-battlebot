"""Convert a DeepLab v3 .pth checkpoint to TensorRT engine format."""

import argparse
import os
import tempfile
from pathlib import Path

import torch
from torchvision.models.segmentation import (
    DeepLabV3,
    deeplabv3_mobilenet_v3_large,
    deeplabv3_resnet50,
    deeplabv3_resnet101,
)

from constants import IMAGE_SIZE, NUM_CLASSES, PAD_SIZE

try:
    import tensorrt as trt
except ImportError as e:
    raise ImportError(
        "TensorRT is not installed. Install it (e.g. pip install tensorrt, or use JetPack on Jetson)"
    ) from e


def get_backbone_from_stem(stem: str) -> str:
    """Infer backbone name from checkpoint filename stem (e.g. model_r50 -> r50)."""
    return stem.split("_")[1]


def build_model(backbone: str, device: torch.device) -> DeepLabV3:
    builders = {
        "mbv3": deeplabv3_mobilenet_v3_large,
        "r50": deeplabv3_resnet50,
        "r101": deeplabv3_resnet101,
    }
    if backbone not in builders:
        raise ValueError(
            f"Unknown backbone '{backbone}'. Must be one of: {list(builders.keys())}"
        )
    model: DeepLabV3 = builders[backbone](num_classes=NUM_CLASSES, aux_loss=True)
    model.to(device)
    model.eval()
    return model


def load_checkpoint(
    model: DeepLabV3, checkpoint_path: Path, device: torch.device
) -> None:
    state = torch.load(checkpoint_path, map_location=device)
    model.load_state_dict(state, strict=False)


def export_onnx(
    model: DeepLabV3,
    output_path: Path,
    input_size: int,
    opset_version: int = 18,
    dynamic_batch: bool = False,
    device: torch.device = torch.device("cuda"),
) -> None:
    """Export model to ONNX (used as intermediate for TensorRT)."""
    dummy_input = torch.randn(1, 3, input_size, input_size, device=device)

    class Wrapper(torch.nn.Module):
        def __init__(self, inner: DeepLabV3) -> None:
            super().__init__()
            self.inner = inner

        def forward(self, x: torch.Tensor) -> torch.Tensor:
            return self.inner(x)["out"]

    wrapped = Wrapper(model)
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
        wrapped,
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

    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise RuntimeError("Failed to build TensorRT engine")
    engine_path = Path(engine_path)
    engine_path.parent.mkdir(parents=True, exist_ok=True)
    with open(engine_path, "wb") as f:
        f.write(serialized)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert DeepLab v3 .pth checkpoint to TensorRT engine"
    )
    parser.add_argument(
        "model",
        type=str,
        help="Path to DeepLab v3 checkpoint (.pth file)",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        help="Output path for TensorRT engine (default: same name with .engine extension)",
    )
    parser.add_argument(
        "--backbone",
        type=str,
        choices=["mbv3", "r50", "r101"],
        help="Backbone name (default: inferred from filename, e.g. model_r50.pth -> r50)",
    )
    parser.add_argument(
        "--input-size",
        type=int,
        default=IMAGE_SIZE + 2 * PAD_SIZE,
        help=f"Input spatial size H=W (default: IMAGE_SIZE + 2*PAD_SIZE = {IMAGE_SIZE + 2 * PAD_SIZE})",
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
    if model_path.suffix.lower() != ".pth":
        print(f"Warning: expected .pth file, got {model_path.suffix}")

    backbone = args.backbone or get_backbone_from_stem(model_path.stem)
    device = torch.device("cuda")

    if args.output:
        output_path = Path(args.output)
    else:
        output_path = model_path.parent / f"{model_path.stem}.engine"

    input_size = args.input_size

    print(f"Loading checkpoint from {model_path}...")
    print(f"Backbone: {backbone}, num_classes: {NUM_CLASSES}")
    model = build_model(backbone, device)
    load_checkpoint(model, model_path, device)

    if args.keep_onnx:
        onnx_path = output_path.with_suffix(".onnx")
        use_temp = False
    else:
        onnx_fd, onnx_path_str = tempfile.mkstemp(suffix=".onnx")
        os.close(onnx_fd)
        onnx_path = Path(onnx_path_str)
        use_temp = True

    try:
        print(f"Exporting to ONNX (input shape: [1, 3, {input_size}, {input_size}])...")
        export_onnx(
            model,
            onnx_path,
            input_size,
            opset_version=args.opset,
            dynamic_batch=False,
            device=device,
        )
        print("Building TensorRT engine...")
        build_tensorrt_engine(
            onnx_path,
            output_path,
            fp16=not args.no_fp16,
            workspace_gib=args.workspace,
        )
        print(f"Engine saved to: {output_path}")
    finally:
        if use_temp and onnx_path.exists():
            try:
                onnx_path.unlink()
            except OSError:
                pass


if __name__ == "__main__":
    main()
