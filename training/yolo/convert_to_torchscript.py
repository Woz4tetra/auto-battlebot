import argparse
from pathlib import Path

from ultralytics import YOLO


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert YOLO model to TorchScript format"
    )
    parser.add_argument("model", type=str, help="Path to YOLO model (.pt file)")
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        help="Output path for TorchScript model (default: same name with .torchscript extension)",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="Image size for export (default: 640)",
    )
    args = parser.parse_args()

    # Load YOLO model
    model_path = Path(args.model)
    if not model_path.exists():
        raise FileNotFoundError(f"Model file not found: {model_path}")

    print(f"Loading YOLO model from {model_path}...")
    model = YOLO(str(model_path))

    # Move model to CPU to avoid device mismatch issues during export
    print("Moving model to GPU...")
    model.to("cuda")

    # Determine output path
    if args.output:
        output_path = Path(args.output)
    else:
        output_path = model_path.parent / f"{model_path.stem}.torchscript"

    print("Converting to TorchScript format...")
    print(f"Image size: {args.imgsz}")

    # Export to TorchScript
    # The export method returns the path to the exported model
    exported_path = model.export(format="torchscript", imgsz=args.imgsz, device="cuda")

    print(f"Model successfully exported to: {exported_path}")

    # If user specified a different output path, rename the file
    if args.output and exported_path != str(output_path):
        Path(exported_path).rename(output_path)
        print(f"Model moved to: {output_path}")


if __name__ == "__main__":
    main()
