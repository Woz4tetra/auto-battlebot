#!/usr/bin/env python3
# Copyright (c) Meta Platforms, Inc. and affiliates. All Rights Reserved
"""
Script to perform video object segmentation using SAM 3 with point prompts.

Usage:
    python scripts/video_segmentation_with_points.py \
        --video /path/to/input_video.mp4 \
        --points "[[210, 350], [250, 220]]" \
        --labels "[1, 1]" \
        --output /path/to/output_video.mp4

    # Multiple objects with different object IDs:
    python scripts/video_segmentation_with_points.py \
        --video /path/to/input_video.mp4 \
        --points "[[210, 350], [250, 220]]" \
        --labels "[1, 1]" \
        --obj-ids "[1, 1]" \
        --output /path/to/output_video.mp4

    # Multiple objects (each with their own points):
    python scripts/video_segmentation_with_points.py \
        --video /path/to/input_video.mp4 \
        --points "[[[210, 350]], [[400, 150]]]" \
        --labels "[[1], [1]]" \
        --obj-ids "[1, 2]" \
        --output /path/to/output_video.mp4
"""

import argparse
import json
import os
import sys

import cv2
import numpy as np
import torch
from tqdm import tqdm

from sam3.model_builder import build_sam3_video_model


def setup_device():
    """Set up the computation device and enable optimizations."""
    if torch.cuda.is_available():
        device = torch.device("cuda")
        torch.autocast("cuda", dtype=torch.bfloat16).__enter__()
        if torch.cuda.get_device_properties(0).major >= 8:
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True
    elif torch.backends.mps.is_available():
        device = torch.device("mps")
    else:
        device = torch.device("cpu")

    print(f"Using device: {device}")
    return device


def get_mask_color(obj_id: int, alpha: float = 0.6) -> np.ndarray:
    """Get a color for a given object ID using tab10 colormap."""
    import matplotlib.pyplot as plt

    cmap = plt.get_cmap("tab10")
    color = np.array([*cmap(obj_id % 10)[:3], alpha])
    return color


def overlay_mask_on_frame(
    frame: np.ndarray, mask: np.ndarray, obj_id: int, alpha: float = 0.6
) -> np.ndarray:
    """Overlay a colored mask on a frame."""
    color = get_mask_color(obj_id, alpha=1.0)[:3]  # RGB color without alpha
    color_bgr = (np.array(color) * 255).astype(np.uint8)[::-1]  # Convert to BGR

    # Create colored mask
    mask_bool = mask.squeeze().astype(bool)
    colored_mask = np.zeros_like(frame)
    colored_mask[mask_bool] = color_bgr

    # Blend with original frame
    result = frame.copy()
    result[mask_bool] = cv2.addWeighted(frame, 1 - alpha, colored_mask, alpha, 0)[
        mask_bool
    ]

    return result


def draw_points_on_frame(
    frame: np.ndarray, points: np.ndarray, labels: np.ndarray
) -> np.ndarray:
    """Draw point prompts on a frame."""
    result = frame.copy()

    for point, label in zip(points, labels):
        x, y = int(point[0]), int(point[1])
        color = (
            (0, 255, 0) if label == 1 else (0, 0, 255)
        )  # Green for positive, red for negative
        cv2.drawMarker(
            result, (x, y), color, cv2.MARKER_STAR, markerSize=20, thickness=2
        )

    return result


def parse_points_input(points_str: str, labels_str: str, obj_ids_str: str = None):
    """
    Parse points input from command line.

    Supports two formats:
    1. Single object: points="[[x1,y1], [x2,y2]]", labels="[1, 1]"
    2. Multiple objects: points="[[[x1,y1]], [[x2,y2]]]", labels="[[1], [1]]", obj_ids="[1, 2]"

    Returns:
        List of tuples: [(obj_id, points_array, labels_array), ...]
    """
    points = json.loads(points_str)
    labels = json.loads(labels_str)

    # Check if it's multi-object format (3D array for points)
    if (
        len(points) > 0
        and isinstance(points[0], list)
        and len(points[0]) > 0
        and isinstance(points[0][0], list)
    ):
        # Multi-object format
        if obj_ids_str:
            obj_ids = json.loads(obj_ids_str)
        else:
            obj_ids = list(range(1, len(points) + 1))

        result = []
        for i, (pts, lbls, obj_id) in enumerate(zip(points, labels, obj_ids)):
            pts_array = np.array(pts, dtype=np.float32)
            lbls_array = np.array(lbls, dtype=np.int32)
            result.append((obj_id, pts_array, lbls_array))
        return result
    else:
        # Single object format
        obj_id = 1
        if obj_ids_str:
            obj_ids = json.loads(obj_ids_str)
            if isinstance(obj_ids, list) and len(obj_ids) > 0:
                obj_id = obj_ids[0]
            else:
                obj_id = obj_ids

        pts_array = np.array(points, dtype=np.float32)
        lbls_array = np.array(labels, dtype=np.int32)
        return [(obj_id, pts_array, lbls_array)]


def main():
    parser = argparse.ArgumentParser(
        description="Video object segmentation with SAM 3 using point prompts",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--video", "-v", required=True, help="Path to input video file")
    parser.add_argument(
        "--points",
        "-p",
        required=True,
        help='Points as JSON array, e.g., "[[210, 350], [250, 220]]" for single object '
        'or "[[[210, 350]], [[400, 150]]]" for multiple objects',
    )
    parser.add_argument(
        "--labels",
        "-l",
        required=True,
        help='Labels as JSON array (1=positive, 0=negative), e.g., "[1, 1]" '
        'or "[[1], [1]]" for multiple objects',
    )
    parser.add_argument(
        "--obj-ids",
        "-i",
        default=None,
        help='Object IDs as JSON array, e.g., "[1, 2]". If not provided, will auto-assign.',
    )
    parser.add_argument(
        "--output", "-o", required=True, help="Path to output video file"
    )
    parser.add_argument(
        "--frame-idx",
        "-f",
        type=int,
        default=0,
        help="Frame index to add prompts on (default: 0)",
    )
    parser.add_argument(
        "--max-frames",
        "-m",
        type=int,
        default=None,
        help="Maximum number of frames to process (default: all)",
    )
    parser.add_argument(
        "--show-points",
        action="store_true",
        help="Draw point prompts on the first frame",
    )
    parser.add_argument(
        "--alpha",
        type=float,
        default=0.5,
        help="Mask overlay transparency (0-1, default: 0.5)",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=None,
        help="Output video FPS (default: same as input)",
    )

    args = parser.parse_args()

    # Validate inputs
    if not os.path.exists(args.video):
        print(f"Error: Video file not found: {args.video}")
        sys.exit(1)

    # Setup device
    device = setup_device()

    # Parse points input
    try:
        prompts = parse_points_input(args.points, args.labels, args.obj_ids)
    except json.JSONDecodeError as e:
        print(f"Error parsing points/labels: {e}")
        sys.exit(1)

    print(f"Parsed {len(prompts)} object(s) to track")
    for obj_id, pts, lbls in prompts:
        print(f"  Object {obj_id}: {len(pts)} point(s)")

    # Load video to get dimensions
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"Error: Could not open video: {args.video}")
        sys.exit(1)

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    input_fps = cap.get(cv2.CAP_PROP_FPS)
    cap.release()

    output_fps = args.fps if args.fps else input_fps
    max_frames = args.max_frames if args.max_frames else total_frames

    print(f"Video: {width}x{height}, {total_frames} frames @ {input_fps:.2f} FPS")
    print(f"Processing up to {max_frames} frames")

    # Build model
    print("Loading SAM 3 model...")
    sam3_model = build_sam3_video_model()
    predictor = sam3_model.tracker
    predictor.backbone = sam3_model.detector.backbone
    print("Model loaded successfully")

    # Initialize inference state
    print("Initializing inference state...")
    inference_state = predictor.init_state(video_path=args.video)

    # Add point prompts for each object
    ann_frame_idx = args.frame_idx

    for obj_id, points, labels in prompts:
        # Convert points to relative coordinates
        rel_points = [[x / width, y / height] for x, y in points]
        points_tensor = torch.tensor(rel_points, dtype=torch.float32)
        labels_tensor = torch.tensor(labels, dtype=torch.int32)

        print(f"Adding points for object {obj_id} on frame {ann_frame_idx}...")
        _, out_obj_ids, low_res_masks, video_res_masks = predictor.add_new_points(
            inference_state=inference_state,
            frame_idx=ann_frame_idx,
            obj_id=obj_id,
            points=points_tensor,
            labels=labels_tensor,
            clear_old_points=False,
        )

    # Propagate through video
    print("Propagating masks through video...")
    video_segments = {}

    for frame_idx, obj_ids, low_res_masks, video_res_masks, obj_scores in tqdm(
        predictor.propagate_in_video(
            inference_state,
            start_frame_idx=0,
            max_frame_num_to_track=max_frames,
            reverse=False,
            propagate_preflight=True,
        ),
        total=max_frames,
        desc="Propagating",
    ):
        video_segments[frame_idx] = {
            out_obj_id: (video_res_masks[i] > 0.0).cpu().numpy()
            for i, out_obj_id in enumerate(obj_ids)
        }

    print(f"Generated masks for {len(video_segments)} frames")

    # Create output video
    print("Creating output video...")
    cap = cv2.VideoCapture(args.video)

    # Set up video writer
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(args.output, fourcc, output_fps, (width, height))

    frame_idx = 0
    pbar = tqdm(total=min(len(video_segments), max_frames), desc="Writing video")

    while cap.isOpened() and frame_idx < max_frames:
        ret, frame = cap.read()
        if not ret:
            break

        if frame_idx in video_segments:
            # Overlay masks
            for obj_id, mask in video_segments[frame_idx].items():
                frame = overlay_mask_on_frame(frame, mask, obj_id, alpha=args.alpha)

            # Optionally draw points on the annotation frame
            if args.show_points and frame_idx == ann_frame_idx:
                for obj_id, points, labels in prompts:
                    frame = draw_points_on_frame(frame, points, labels)

        out.write(frame)
        frame_idx += 1
        pbar.update(1)

    pbar.close()
    cap.release()
    out.release()

    print(f"Output video saved to: {args.output}")

    # Try to use ffmpeg to re-encode for better compatibility
    try:
        import subprocess

        temp_output = args.output + ".temp.mp4"
        os.rename(args.output, temp_output)
        subprocess.run(
            [
                "ffmpeg",
                "-y",
                "-i",
                temp_output,
                "-c:v",
                "libx264",
                "-preset",
                "fast",
                "-crf",
                "23",
                args.output,
            ],
            check=True,
            capture_output=True,
        )
        os.remove(temp_output)
        print("Re-encoded with ffmpeg for better compatibility")
    except Exception:
        if os.path.exists(temp_output):
            os.rename(temp_output, args.output)
        # ffmpeg not available or failed, keep original output


if __name__ == "__main__":
    main()
