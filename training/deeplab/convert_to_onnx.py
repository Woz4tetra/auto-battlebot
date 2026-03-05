"""Convert a DeepLab v3 .pth checkpoint to ONNX format."""

import argparse
import shutil
from pathlib import Path

import torch
from torchvision.models.segmentation import DeepLabV3

from load_deeplabv3 import build_model
from model_config import load_model_config, config_path_for


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

    cfg = load_model_config(model_path)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    print(f"Loading checkpoint from {model_path}...")
    print(f"Backbone: {cfg.backbone}, num_classes: {cfg.num_classes}, "
          f"input_size: {cfg.input_size}")
    model = build_model(cfg.backbone, cfg.num_classes, device)
    state = torch.load(model_path, map_location=device)
    model.load_state_dict(state, strict=False)
    model.eval()

    if args.output:
        output_path = Path(args.output)
    else:
        output_path = model_path.parent / f"{model_path.stem}.onnx"

    input_size = cfg.input_size
    dummy_input = torch.randn(1, 3, input_size, input_size, device=device)

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

    shutil.copy2(config_path_for(model_path), config_path_for(output_path))
    print(f"Model exported to: {output_path}")


if __name__ == "__main__":
    main()
