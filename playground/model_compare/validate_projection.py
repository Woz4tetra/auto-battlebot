"""Sanity-check the field->image projection on one frame before rendering a whole video.

Draws the nominal 8 ft arena square and the opponent footprint from the original run. The
square landing on the arena walls is the documented check (see training/model_eval/README).
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from auto_battlebot.track_overlay import (
    footprint_circle,
    project_field_points,
    read_camera_matrix,
    read_frame_meta,
    read_opponent_track,
)

FIELD_HALF_M = 2.44 / 2.0

# 'camera' may be REP-103 body axes (x forward, z up) rather than optical (z forward).
CONVENTIONS = {
    "optical": np.eye(3),
    "rep103": np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]),
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--old-mcap", required=True)
    ap.add_argument("--new-mcap", required=True)
    ap.add_argument("--svo", required=True)
    ap.add_argument("--stamp-ns", type=int, required=True)
    ap.add_argument("--out-dir", required=True)
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    _, stamp_to_index = read_frame_meta(args.new_mcap)
    frame_index = stamp_to_index.nearest(args.stamp_ns)
    print(f"stamp {args.stamp_ns} -> svo frame {frame_index}")

    camera_matrix = read_camera_matrix(args.old_mcap)
    print("camera_matrix =", np.round(camera_matrix, 2).tolist())

    old_track = read_opponent_track(args.old_mcap)
    sample = old_track.nearest(args.stamp_ns, tolerance_ns=100_000_000)
    if sample is None:
        print("no opponent sample near that stamp")
        return 1
    print(f"opponent field xy = {np.round(sample.position, 3).tolist()} r={sample.radius:.3f}")

    import pyzed.sl as sl

    cam = sl.Camera()
    init = sl.InitParameters()
    init.set_from_svo_file(args.svo)
    init.svo_real_time_mode = False
    if cam.open(init) != sl.ERROR_CODE.SUCCESS:
        print("failed to open SVO")
        return 1
    cam.set_svo_position(int(frame_index))
    mat = sl.Mat()
    if cam.grab() != sl.ERROR_CODE.SUCCESS:
        print("grab failed")
        return 1
    cam.retrieve_image(mat, sl.VIEW.LEFT)
    frame = cv2.cvtColor(mat.get_data(), cv2.COLOR_BGRA2BGR)
    cam.close()
    print("frame", frame.shape)

    square = np.array(
        [
            [-FIELD_HALF_M, -FIELD_HALF_M, 0.0],
            [FIELD_HALF_M, -FIELD_HALF_M, 0.0],
            [FIELD_HALF_M, FIELD_HALF_M, 0.0],
            [-FIELD_HALF_M, FIELD_HALF_M, 0.0],
        ]
    )
    ring = footprint_circle(sample.position, sample.radius)

    for name, fix in CONVENTIONS.items():
        canvas = frame.copy()
        tf = sample.tf_field_from_camera.copy()
        tf[:3, :3] = tf[:3, :3] @ fix.T
        sq, sq_ok = project_field_points(square, tf, camera_matrix)
        rg, rg_ok = project_field_points(ring, tf, camera_matrix)
        h, w = frame.shape[:2]
        inside = sum(1 for p, ok in zip(sq, sq_ok) if ok and 0 <= p[0] < w and 0 <= p[1] < h)
        print(f"{name:8s} square corners in frame: {inside}/4  px={np.round(sq, 1).tolist()}")
        if sq_ok.all():
            cv2.polylines(canvas, [sq.astype(np.int32)], True, (0, 255, 255), 3)
        if rg_ok.all():
            cv2.polylines(canvas, [rg.astype(np.int32)], True, (0, 0, 255), 3)
        cv2.putText(canvas, name, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
        cv2.imwrite(str(out_dir / f"validate_{name}.png"), canvas)
    print("wrote", out_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
