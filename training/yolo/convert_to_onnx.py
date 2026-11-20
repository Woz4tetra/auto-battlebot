import argparse
from pathlib import Path

from ultralytics import YOLO


def disable_end2end(model: YOLO) -> None:
    """Disable end2end post-processing in the detection head."""
    for m in model.model.modules():
        if hasattr(m, "end2end"):
            m.end2end = False


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

    print(f"Loading YOLO model from {model_path}...")
    model = YOLO(str(model_path))

    disable_end2end(model)

    print("Moving model to GPU...")
    model.to("cuda")

    head = model.model.model[-1]
    if hasattr(head, "end2end") and head.end2end:
        head.end2end = False
        print(f"  Disabled end2end NMS on {type(head).__name__} for raw ONNX output.")

    if args.output:
        output_path = Path(args.output)
    else:
        output_path = model_path.parent / f"{model_path.stem}.onnx"

    print("Converting to ONNX format...")
    print(f"Image size: {imgsz}")

    exported_path = model.export(format="onnx", imgsz=imgsz, device="cpu")

    print(f"Model successfully exported to: {exported_path}")

    if args.output and exported_path != str(output_path):
        Path(exported_path).rename(output_path)
        print(f"Model moved to: {output_path}")


if __name__ == "__main__":
    main()
