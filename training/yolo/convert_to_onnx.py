import argparse
import os
import tempfile
from pathlib import Path

from ultralytics import YOLO


def disable_end2end(model: YOLO) -> None:
    """Disable end2end post-processing in the detection head."""
    for m in model.model.modules():
        if hasattr(m, "end2end"):
            m.end2end = False


def export_onnx(model_path: Path, output_path: Path, imgsz: int | list[int]) -> None:
    """Export raw-head ONNX straight to `output_path`, leaving no file beside the checkpoint."""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=output_path.parent) as scratch:
        link = Path(scratch) / model_path.name
        link.symlink_to(model_path.resolve())
        model = YOLO(str(link))
        disable_end2end(model)
        exported = model.export(format="onnx", imgsz=imgsz, device="cpu")
        os.replace(exported, output_path)


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert YOLO model to ONNX format")
    parser.add_argument("model", type=str, help="Path to YOLO model (.pt file)")
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        help="Output path for ONNX model (default: same name with .onnx extension)",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        nargs="+",
        default=[640],
        metavar="N",
        help="Input size: one value for square, or 'H W'; both multiples of 32 (default: 640)",
    )
    args = parser.parse_args()

    if len(args.imgsz) not in (1, 2):
        raise SystemExit("--imgsz takes one value (square) or two (H W)")
    imgsz = args.imgsz[0] if len(args.imgsz) == 1 else list(args.imgsz)
    for dim in args.imgsz:
        if dim % 32:
            raise SystemExit(f"--imgsz values must be multiples of 32, got {dim}")

    model_path = Path(args.model)
    if not model_path.exists():
        raise FileNotFoundError(f"Model file not found: {model_path}")

    print(f"Image size: {imgsz}")
    output_path = Path(args.output) if args.output else model_path.with_suffix(".onnx")
    export_onnx(model_path, output_path, imgsz)
    print(f"Model exported to: {output_path}")


if __name__ == "__main__":
    main()
