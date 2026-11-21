#!/usr/bin/env python3
"""Left image, metric XYZ and intrinsics for one frame of a ZED SVO, for the basement scene fit."""

from __future__ import annotations

import argparse
import json
from collections.abc import Callable
from pathlib import Path

import cv2
import numpy as np
import pyzed.sl as sl


def open_svo(path: Path) -> tuple[sl.Camera, str]:
    for mode in ("NEURAL", "ULTRA"):
        init = sl.InitParameters()
        init.set_from_svo_file(str(path))
        init.svo_real_time_mode = False
        init.depth_mode = getattr(sl.DEPTH_MODE, mode)
        init.coordinate_units = sl.UNIT.METER
        init.depth_maximum_distance = 10.0
        camera = sl.Camera()
        if camera.open(init) == sl.ERROR_CODE.SUCCESS:
            return camera, mode
    raise SystemExit(f"{path}: could not open with NEURAL or ULTRA depth")


def decode_sequentially(camera, runtime, last_frame: int, on_frame: Callable[[int], None]) -> None:
    """Grab every frame from 0 to last_frame in order; seeking returns smeared frames."""
    for index in range(last_frame + 1):
        if camera.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            raise SystemExit(f"ran out of frames at {index}")
        on_frame(index)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("svo", type=Path)
    parser.add_argument("--frame", type=int, required=True)
    parser.add_argument("--depth-frames", type=int, default=15)
    parser.add_argument("--confidence", type=int, default=95)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()

    camera, mode = open_svo(args.svo)
    runtime = sl.RuntimeParameters()
    runtime.confidence_threshold = args.confidence
    image, xyz = sl.Mat(), sl.Mat()
    stack: list[np.ndarray] = []
    holder: dict[str, np.ndarray] = {}

    def on_frame(index: int) -> None:
        if index > args.frame - args.depth_frames:
            camera.retrieve_measure(xyz, sl.MEASURE.XYZ)
            stack.append(xyz.get_data()[:, :, :3].copy())
        if index == args.frame:
            camera.retrieve_image(image, sl.VIEW.LEFT)
            holder["left"] = image.get_data()[:, :, :3].copy()

    decode_sequentially(camera, runtime, args.frame, on_frame)
    left = holder["left"]
    left_cam = camera.get_camera_information().camera_configuration.calibration_parameters.left_cam
    camera.close()

    depth = np.stack(stack)
    median = np.nanmedian(depth, axis=0).astype(np.float32)
    spread = np.nanpercentile(np.linalg.norm(depth[-1] - median, axis=2), [50, 90])
    args.out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.out / "left.png"), left)
    np.save(args.out / "xyz.npy", median)
    height, width = median.shape[:2]
    intrinsics = {
        "fx": float(left_cam.fx),
        "fy": float(left_cam.fy),
        "cx": float(left_cam.cx),
        "cy": float(left_cam.cy),
        "width": int(width),
        "height": int(height),
        "note": f"ZED left camera of {args.svo.name}, read from the SVO by export_svo_depth.py",
    }
    (args.out / "camera_rect.json").write_text(json.dumps(intrinsics, indent=2) + "\n")
    print(
        f"{args.svo.name} frame {args.frame}: {mode} depth over {len(stack)} frames, "
        f"{np.isfinite(median[:, :, 2]).mean():.1%} valid, last-vs-median "
        f"p50 {spread[0] * 1000:.1f} mm p90 {spread[1] * 1000:.1f} mm, fx {left_cam.fx:.2f}"
    )


if __name__ == "__main__":
    main()
