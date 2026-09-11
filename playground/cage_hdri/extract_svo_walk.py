"""Replay a hand-held ZED walk-around and dump posed frames for HDRI stitching.

Stage 1 of the cage HDRI pipeline. Runs the SVO through ZED positional tracking
with IMU fusion, so every kept frame gets a gravity-aligned world pose, and writes
the left image and the pose to OUTPUT_DIR:

    frames/000123.png       left image, 8-bit BGR, rectified
    poses.npz               frame_index, timestamp_ns, world_T_cam (N,4,4), confidence
    poses.csv               same, one row per frame, for eyeballing
    trajectory.png          top-down plot of the camera path

The SVO is walked sequentially. Seeking smears the decode, so the stride is applied
by skipping frames after grab, never by seek. Depth is off: stage 2 uses orientation
only.

Coordinate system is sl.COORDINATE_SYSTEM.IMAGE: x right, y down, z forward, in both
the camera and the world frame. Gravity therefore points along +y in the world frame.
Stage 2 relies on that.

Usage:
    python extract_svo_walk.py data/svo/tests/1729981242.4295187.svo2 runs/cage_hdri/walk
    python extract_svo_walk.py IN.svo2 OUT --stride 3
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import cv2
import numpy as np
import pyzed.sl as sl

TRAJECTORY_PX = 1200
TRAJECTORY_MARGIN_PX = 60


def open_camera(svo_path: Path) -> sl.Camera:
    init = sl.InitParameters()
    init.set_from_svo_file(str(svo_path))
    init.depth_mode = sl.DEPTH_MODE.NONE
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
    init.sdk_verbose = 0

    camera = sl.Camera()
    status = camera.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"open failed: {status}")

    tracking = sl.PositionalTrackingParameters()
    tracking.enable_imu_fusion = True
    tracking.enable_area_memory = True
    tracking.set_floor_as_origin = False
    status = camera.enable_positional_tracking(tracking)
    if status != sl.ERROR_CODE.SUCCESS:
        camera.close()
        raise RuntimeError(f"tracking failed: {status}")
    return camera


def intrinsics_of(camera: sl.Camera) -> dict[str, float]:
    config = camera.get_camera_information().camera_configuration
    left = config.calibration_parameters.left_cam
    return {
        "fx": float(left.fx),
        "fy": float(left.fy),
        "cx": float(left.cx),
        "cy": float(left.cy),
        "width": float(config.resolution.width),
        "height": float(config.resolution.height),
    }


def draw_trajectory(positions: np.ndarray, confidences: np.ndarray, destination: Path) -> None:
    """Top-down plot. Image y-down world means the ground plane is x-z."""
    canvas = np.full((TRAJECTORY_PX, TRAJECTORY_PX, 3), 255, dtype=np.uint8)
    if len(positions) == 0:
        cv2.imwrite(str(destination), canvas)
        return
    xz = positions[:, [0, 2]]
    low = xz.min(axis=0)
    span = max(float((xz.max(axis=0) - low).max()), 1e-3)
    scale = (TRAJECTORY_PX - 2 * TRAJECTORY_MARGIN_PX) / span
    points = ((xz - low) * scale + TRAJECTORY_MARGIN_PX).astype(np.int32)
    for (x, z), confidence in zip(points, confidences):
        shade = int(255 * (1.0 - confidence / 100.0))
        cv2.circle(canvas, (int(x), int(z)), 3, (shade, 0, 255 - shade), -1)
    cv2.circle(canvas, tuple(int(v) for v in points[0]), 9, (0, 160, 0), 2)
    cv2.putText(
        canvas,
        f"{len(positions)} frames, span {span:.2f} m, red=confident blue=not, green=start",
        (12, TRAJECTORY_PX - 16),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 0, 0),
        1,
    )
    cv2.imwrite(str(destination), canvas)


def extract(svo_path: Path, output_dir: Path, stride: int) -> int:
    frames_dir = output_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    camera = open_camera(svo_path)
    intrinsics = intrinsics_of(camera)
    total = camera.get_svo_number_of_frames()
    print(f"{svo_path.name}: {total} frames, stride {stride}")

    image = sl.Mat()
    pose = sl.Pose()
    runtime = sl.RuntimeParameters()

    frame_indices: list[int] = []
    timestamps: list[int] = []
    transforms: list[np.ndarray] = []
    confidences: list[float] = []
    skipped_untracked = 0
    index = 0

    while camera.grab(runtime) == sl.ERROR_CODE.SUCCESS:
        state = camera.get_position(pose, sl.REFERENCE_FRAME.WORLD)
        if index % stride == 0:
            if state != sl.POSITIONAL_TRACKING_STATE.OK:
                skipped_untracked += 1
            else:
                camera.retrieve_image(image, sl.VIEW.LEFT)
                bgr = np.ascontiguousarray(image.get_data()[:, :, :3])
                cv2.imwrite(str(frames_dir / f"{index:06d}.png"), bgr)
                frame_indices.append(index)
                timestamps.append(pose.timestamp.get_nanoseconds())
                transforms.append(np.array(pose.pose_data().m, dtype=np.float64))
                confidences.append(float(pose.pose_confidence))
        if index % 500 == 0:
            print(f"  frame {index}/{total}, kept {len(frame_indices)}")
        index += 1
    camera.close()

    world_t_cam = np.stack(transforms) if transforms else np.zeros((0, 4, 4))
    np.savez(
        output_dir / "poses.npz",
        frame_index=np.array(frame_indices, dtype=np.int64),
        timestamp_ns=np.array(timestamps, dtype=np.int64),
        world_T_cam=world_t_cam,
        confidence=np.array(confidences, dtype=np.float64),
        **{f"intrinsics_{key}": value for key, value in intrinsics.items()},
    )
    with (output_dir / "poses.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["frame", "timestamp_ns", "confidence", "tx", "ty", "tz"])
        for frame, stamp, confidence, transform in zip(
            frame_indices, timestamps, confidences, transforms
        ):
            writer.writerow(
                [frame, stamp, f"{confidence:.0f}", *(f"{v:.4f}" for v in transform[:3, 3])]
            )
    draw_trajectory(world_t_cam[:, :3, 3], np.array(confidences), output_dir / "trajectory.png")

    print(
        f"decoded {index} frames, kept {len(frame_indices)}, "
        f"skipped {skipped_untracked} untracked, wrote {output_dir}"
    )
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("svo", type=Path, help="Walk-around SVO recording")
    parser.add_argument("output_dir", type=Path, help="Directory to write frames, depth and poses")
    parser.add_argument("--stride", type=int, default=3, help="Keep every Nth frame (default 3)")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.svo.exists():
        print(f"{args.svo} does not exist")
        return 1
    return extract(args.svo, args.output_dir, args.stride)


if __name__ == "__main__":
    sys.exit(main())
