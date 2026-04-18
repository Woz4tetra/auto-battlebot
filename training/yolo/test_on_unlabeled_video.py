import argparse
import time
from pathlib import Path

import cv2
from tqdm import tqdm
from ultralytics import YOLO

IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"}


def normalize_annotated_frame(annotated_frame):
    if annotated_frame.dtype != "uint8":
        annotated_frame = annotated_frame.clip(0, 255).astype("uint8")
    if len(annotated_frame.shape) == 2:
        annotated_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_GRAY2BGR)
    elif annotated_frame.shape[2] == 4:
        annotated_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_BGRA2BGR)
    return annotated_frame


def build_inference_kwargs(args, model_task: str) -> dict:
    inference_kwargs = {
        "conf": args.conf,
        "imgsz": args.imgsz,
        "device": args.device,
        "verbose": False,
    }
    if model_task == "segment" and args.retina_masks:
        inference_kwargs["retina_masks"] = True
    return inference_kwargs


def annotate_frame(model, frame, inference_kwargs: dict):
    results = model(frame, **inference_kwargs)
    annotated_frame = results[0].plot()
    return normalize_annotated_frame(annotated_frame)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Test a trained YOLO model on a video or directory of images"
    )
    parser.add_argument(
        "model",
        type=str,
        help="Path to trained model weights (e.g., ../projects/auto_battlebots_keypoints/weights/best.pt)",
    )
    parser.add_argument(
        "input_path",
        type=str,
        help="Path to input video file or image directory",
    )
    parser.add_argument(
        "-o",
        "--output",
        default="",
        type=str,
        help=(
            "Output path: video file path for video input, or output directory for image input "
            "(defaults: input_name_annotated.mp4 / input_dir_annotated/)"
        ),
    )
    parser.add_argument(
        "-c",
        "--conf",
        default=0.25,
        type=float,
        help="Confidence threshold for detections (default: 0.25)",
    )
    parser.add_argument(
        "--imgsz",
        default=640,
        type=int,
        help="Image size for inference (default: 640)",
    )
    parser.add_argument(
        "--device",
        default="0",
        type=str,
        help="Device to run inference on (default: 0 for GPU, 'cpu' for CPU)",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display frames/images while processing",
    )
    parser.add_argument(
        "--retina-masks",
        action="store_true",
        help="Use high-resolution masks for YOLO segmentation models",
    )
    parser.add_argument(
        "--no-save",
        action="store_true",
        help="Don't save output files",
    )
    args = parser.parse_args()

    # Load the model
    print(f"Loading model from {args.model}...")
    model = YOLO(args.model)
    model_task = getattr(model, "task", "detect")
    print(f"Model task: {model_task}")
    input_path = Path(args.input_path)
    if not input_path.exists():
        print(f"Error: Input path '{input_path}' not found")
        return

    inference_kwargs = build_inference_kwargs(args, model_task)
    frame_count = 0
    start_time = time.time()

    if input_path.is_dir():
        image_paths = sorted(
            p
            for p in input_path.iterdir()
            if p.is_file() and p.suffix.lower() in IMAGE_EXTENSIONS
        )
        if not image_paths:
            print(f"Error: No images found in directory '{input_path}'")
            return

        output_dir = None
        if not args.no_save:
            output_dir = (
                Path(args.output)
                if args.output
                else input_path.parent / f"{input_path.name}_annotated"
            )
            output_dir.mkdir(parents=True, exist_ok=True)
            print(f"Saving annotated images to {output_dir}")

        try:
            with tqdm(
                total=len(image_paths), desc="Processing images", unit="image"
            ) as pbar:
                for image_path in image_paths:
                    frame = cv2.imread(str(image_path))
                    if frame is None:
                        print(f"Warning: Could not read image '{image_path}', skipping")
                        pbar.update(1)
                        continue

                    frame_count += 1
                    annotated_frame = annotate_frame(model, frame, inference_kwargs)

                    if output_dir is not None:
                        out_path = output_dir / image_path.name
                        cv2.imwrite(str(out_path), annotated_frame)

                    if args.show:
                        cv2.imshow(
                            f"YOLO {model_task.capitalize()} Results", annotated_frame
                        )
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            print("Interrupted by user")
                            break

                    elapsed_time = time.time() - start_time
                    current_fps = frame_count / elapsed_time if elapsed_time > 0 else 0
                    pbar.set_postfix({"FPS": f"{current_fps:.2f}"})
                    pbar.update(1)
        finally:
            if args.show:
                cv2.destroyAllWindows()

        elapsed_time = time.time() - start_time
        avg_fps = frame_count / elapsed_time if elapsed_time > 0 else 0
        print(
            f"\nProcessing complete! Processed {frame_count} images in {elapsed_time:.2f}s"
        )
        print(f"Average FPS: {avg_fps:.2f}")
        if output_dir is not None:
            print(f"Output saved to: {output_dir}")
        return

    if not input_path.is_file():
        print(f"Error: Input path '{input_path}' is not a file or directory")
        return

    # Video path flow
    output_path = (
        Path(args.output)
        if args.output
        else input_path.parent / f"{input_path.stem}_annotated.mp4"
    )

    cap = cv2.VideoCapture(str(input_path))
    if not cap.isOpened():
        print(f"Error: Could not open video '{input_path}'")
        return

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if fps <= 0:
        fps = 30

    print(f"Video properties: {width}x{height} @ {fps} FPS, {total_frames} frames")

    writer = None
    if not args.no_save:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(output_path), fourcc, fps, (width, height))
        print(f"Saving output to {output_path}")

    try:
        with tqdm(total=total_frames, desc="Processing video", unit="frame") as pbar:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break

                frame_count += 1
                annotated_frame = annotate_frame(model, frame, inference_kwargs)

                if writer is not None:
                    writer.write(annotated_frame)

                if args.show:
                    cv2.imshow(
                        f"YOLO {model_task.capitalize()} Results", annotated_frame
                    )
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        print("Interrupted by user")
                        break

                elapsed_time = time.time() - start_time
                current_fps = frame_count / elapsed_time if elapsed_time > 0 else 0
                pbar.set_postfix({"FPS": f"{current_fps:.2f}"})
                pbar.update(1)
    finally:
        cap.release()
        if writer is not None:
            writer.release()
        if args.show:
            cv2.destroyAllWindows()

    elapsed_time = time.time() - start_time
    avg_fps = frame_count / elapsed_time if elapsed_time > 0 else 0
    print(
        f"\nProcessing complete! Processed {frame_count} frames in {elapsed_time:.2f}s"
    )
    print(f"Average FPS: {avg_fps:.2f}")
    if not args.no_save:
        print(f"Output saved to: {output_path}")


if __name__ == "__main__":
    main()
