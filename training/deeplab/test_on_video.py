"""Run a trained DeepLab segmentation model on a video and visualise the output.

The overlay blends the predicted floor mask onto the original frames using a
configurable colour and opacity.

Usage:
    python test_on_video.py model_mbv3.pth input.mp4
    python test_on_video.py model_mbv3.pth input.mp4 --show --no-save
    python test_on_video.py model_mbv3.pth input.mp4 -o output.mp4 --image-size 256 --alpha 0.5
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch
from tqdm import tqdm

sys.path.insert(0, str(Path(__file__).resolve().parent))
from load_deeplabv3 import common_transforms, load_model

FLOOR_CLASS = 1
OVERLAY_COLOR = (0, 255, 0)


def predict_mask(
    model: torch.nn.Module,
    frame_rgb: np.ndarray,
    transform: torch.nn.Module,
    device: torch.device,
    image_size: int,
    pad_size: int,
) -> np.ndarray:
    resized = cv2.resize(
        frame_rgb, (image_size, image_size), interpolation=cv2.INTER_LINEAR
    )
    input_tensor = transform(resized).unsqueeze(0).to(device)

    with torch.no_grad():
        output = model(input_tensor)["out"]
    pred = torch.argmax(output, dim=1).squeeze(0).cpu().numpy()

    if pad_size > 0:
        pred = pred[pad_size:-pad_size, pad_size:-pad_size]

    return pred.astype(np.uint8)


def overlay_mask(
    frame: np.ndarray,
    mask: np.ndarray,
    color: tuple[int, int, int],
    alpha: float,
) -> np.ndarray:
    h, w = frame.shape[:2]
    mask_resized = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)

    overlay = frame.copy()
    overlay[mask_resized == FLOOR_CLASS] = color
    return cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Test a trained DeepLab segmentation model on video"
    )
    parser.add_argument(
        "model",
        type=str,
        help="Path to trained model checkpoint (e.g. output/model_mbv3.pth)",
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
        help="Path to output video file (default: input_name_segmented.mp4)",
    )
    parser.add_argument(
        "--alpha",
        default=0.4,
        type=float,
        help="Overlay opacity (default: 0.4)",
    )
    parser.add_argument(
        "--device",
        default="",
        type=str,
        help="Device for inference (default: cuda if available, else cpu)",
    )
    parser.add_argument(
        "--skip",
        default=0,
        type=int,
        help="Process every Nth frame, skipping the rest (default: 0 = no skip)",
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

    model_path = Path(args.model)
    video_path = Path(args.video)

    if not model_path.exists():
        print(f"Error: model file not found: {model_path}", file=sys.stderr)
        sys.exit(1)
    if not video_path.exists():
        print(f"Error: video file not found: {video_path}", file=sys.stderr)
        sys.exit(1)

    if args.device:
        device = torch.device(args.device)
    else:
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    print(f"Loading model from {model_path} (device={device})...")
    model, model_cfg = load_model(model_path, device)
    print(
        f"Model config: backbone={model_cfg.backbone}, image_size={model_cfg.image_size}, "
        f"pad_size={model_cfg.pad_size}, num_classes={model_cfg.num_classes}"
    )
    transform = common_transforms(pad_size=model_cfg.pad_size)

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        print(f"Error: could not open video: {video_path}", file=sys.stderr)
        sys.exit(1)

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"Video: {width}x{height} @ {fps} FPS, {total_frames} frames")

    if args.output:
        output_path = args.output
    else:
        output_path = str(video_path.parent / f"{video_path.stem}_segmented.mp4")

    writer = None
    if not args.no_save:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
        print(f"Saving output to {output_path}")

    frame_count = 0
    frame_idx = 0
    skip = args.skip
    start_time = time.time()
    try:
        with tqdm(total=total_frames, desc="Processing video", unit="frame") as pbar:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break

                frame_idx += 1
                pbar.update(1)

                if skip > 0 and (frame_idx - 1) % (skip + 1) != 0:
                    continue

                frame_count += 1
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                mask = predict_mask(
                    model,
                    frame_rgb,
                    transform,
                    device,
                    model_cfg.image_size,
                    model_cfg.pad_size,
                )
                annotated = overlay_mask(frame, mask, OVERLAY_COLOR, args.alpha)

                if writer is not None:
                    writer.write(annotated)

                if args.show:
                    cv2.imshow("DeepLab Segmentation", annotated)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        print("Interrupted by user")
                        break

                elapsed = time.time() - start_time
                current_fps = frame_count / elapsed if elapsed > 0 else 0
                pbar.set_postfix({"FPS": f"{current_fps:.2f}"})

    finally:
        cap.release()
        if writer is not None:
            writer.release()
        if args.show:
            cv2.destroyAllWindows()

    elapsed = time.time() - start_time
    avg_fps = frame_count / elapsed if elapsed > 0 else 0
    print(f"\nProcessed {frame_count} frames in {elapsed:.2f}s")
    print(f"Average FPS: {avg_fps:.2f}")
    if not args.no_save:
        print(f"Output saved to: {output_path}")


if __name__ == "__main__":
    main()
