"""Convert a DeepLab v3 .pth checkpoint to ONNX format."""

import argparse
from pathlib import Path

import torch
from torchvision.models.segmentation import (
    DeepLabV3,
    deeplabv3_mobilenet_v3_large,
    deeplabv3_resnet50,
    deeplabv3_resnet101,
)

from constants import IMAGE_SIZE, NUM_CLASSES, PAD_SIZE


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


def load_checkpoint(model: DeepLabV3, checkpoint_path: Path, device: torch.device) -> None:
    state = torch.load(checkpoint_path, map_location=device)
    model.load_state_dict(state, strict=False)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert DeepLab v3 .pth checkpoint to ONNX format"
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
        help="Output path for ONNX model (default: same name with .onnx extension)",
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
        help="ONNX opset version (default: 18; must match PyTorch native to avoid broken version downgrade)",
    )
    parser.add_argument(
        "--dynamic-batch",
        action="store_true",
        help="Export with dynamic batch dimension (can trigger ONNX version-converter errors)",
    )
    args = parser.parse_args()

    model_path = Path(args.model)
    if not model_path.exists():
        raise FileNotFoundError(f"Model file not found: {model_path}")
    if model_path.suffix.lower() != ".pth":
        print(f"Warning: expected .pth file, got {model_path.suffix}")

    backbone = args.backbone or get_backbone_from_stem(model_path.stem)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    print(f"Loading checkpoint from {model_path}...")
    print(f"Backbone: {backbone}, num_classes: {NUM_CLASSES}")
    model = build_model(backbone, device)
    load_checkpoint(model, model_path, device)

    if args.output:
        output_path = Path(args.output)
    else:
        output_path = model_path.parent / f"{model_path.stem}.onnx"

    input_size = args.input_size
    dummy_input = torch.randn(1, 3, input_size, input_size, device=device)

    # Export only the main segmentation output (model returns dict with "out" and "aux")
    class Wrapper(torch.nn.Module):
        def __init__(self, inner: DeepLabV3) -> None:
            super().__init__()
            self.inner = inner

        def forward(self, x: torch.Tensor) -> torch.Tensor:
            return self.inner(x)["out"]

    wrapped = Wrapper(model)

    export_kwargs: dict = {
        "input_names": ["input"],
        "output_names": ["output"],
        "opset_version": args.opset,
    }
    if args.dynamic_batch:
        export_kwargs["dynamic_axes"] = {
            "input": {0: "batch"},
            "output": {0: "batch"},
        }
    print(f"Exporting to ONNX (input shape: [1, 3, {input_size}, {input_size}], opset={args.opset})...")
    torch.onnx.export(
        wrapped,
        dummy_input,
        str(output_path),
        **export_kwargs,
    )

    print(f"Model exported to: {output_path}")


if __name__ == "__main__":
    main()
