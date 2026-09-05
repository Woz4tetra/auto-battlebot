import argparse
from pathlib import Path

from ultralytics import YOLO


def disable_end2end(model: YOLO) -> None:
    """Disable end2end post-processing in the detection head.

    End2end bakes NMS/score-processing into the ONNX graph, which can break
    multi-class outputs in TensorRT. The C++ pipeline does its own NMS, so
    the raw [1, num_features, num_predictions] output is what we need.
    """
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
        help="Input size: one value for square, or 'H W' for rectangular (default: 640). "
        "Both must be multiples of 32. A 16:9 source letterboxed into a square 640x640 wastes "
        "44%% of the tensor on grey padding; '--imgsz 384 640' cuts that to 6%% at the same "
        "detection scale. The C++ model reads the shape from the engine, so a rectangular "
        "engine is a drop-in swap.",
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

    # Move model to CPU to avoid device mismatch issues during export
    print("Moving model to GPU...")
    model.to("cuda")

    # Disable the built-in end-to-end NMS head before exporting.
    #
    # YOLO26 (and YOLO10+) models use a Detect/Pose head with end2end=True,
    # which bakes top-k NMS into the graph and changes the output to
    # [1, max_det, features] (e.g. [1, 300, 12]) instead of the raw pre-NMS
    # anchor grid [1, features, num_anchors] (e.g. [1, 12, 8400]).
    #
    # The C++ YoloKeypointModel and test_tensorrt_video.py both expect the raw
    # pre-NMS format, so we must disable end2end before export.
    head = model.model.model[-1]
    if hasattr(head, "end2end") and head.end2end:
        head.end2end = False
        print(f"  Disabled end2end NMS on {type(head).__name__} for raw ONNX output.")

    # Determine output path
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
