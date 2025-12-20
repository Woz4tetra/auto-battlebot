import argparse
import os
import time
from pathlib import Path

import cv2
from tqdm import tqdm
from ultralytics import YOLO

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Test a trained YOLO pose model on video"
    )
    parser.add_argument(
        "model",
        type=str,
        help="Path to trained model weights (e.g., ../projects/auto_battlebots_keypoints/weights/best.pt)",
    )
    parser.add_argument(
        "video",
        type=str,
        help="Path to input video file",
    )
    parser.add_argument(
        "-o",
        "--output",
        default="",
        type=str,
        help="Path to output video file (default: input_name_annotated.mp4)",
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
        default=1280,
        type=int,
        help="Image size for inference (default: 1280)",
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
        help="Display video while processing",
    )
    parser.add_argument(
        "--no-save",
        action="store_true",
        help="Don't save output video",
    )
    args = parser.parse_args()

    # Load the model
    print(f"Loading model from {args.model}...")
    model = YOLO(args.model)

    # Check if video file exists
    if not os.path.exists(args.video):
        print(f"Error: Video file '{args.video}' not found")
        return

    # Set up output path
    if args.output:
        output_path = args.output
    else:
        video_path = Path(args.video)
        output_path = str(video_path.parent / f"{video_path.stem}_annotated.mp4")

    # Open video
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"Error: Could not open video '{args.video}'")
        return

    # Get video properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    print(f"Video properties: {width}x{height} @ {fps} FPS, {total_frames} frames")

    # Set up video writer
    writer = None
    if not args.no_save:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
        print(f"Saving output to {output_path}")

    frame_count = 0
    start_time = time.time()
    try:
        with tqdm(total=total_frames, desc="Processing video", unit="frame") as pbar:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break

                frame_count += 1

                # Run inference
                results = model(
                    frame,
                    conf=args.conf,
                    imgsz=args.imgsz,
                    device=args.device,
                    verbose=False,
                )

                # Annotate frame
                annotated_frame = results[0].plot()

                # Write frame
                if writer is not None:
                    writer.write(annotated_frame)

                # Display frame
                if args.show:
                    cv2.imshow("YOLO Pose Detection", annotated_frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        print("Interrupted by user")
                        break

                # Update progress bar with FPS
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
