"""Solve field-plane robot poses from an apriltag_track.py image recording.

Replays the recorded camera images, re-runs the AprilTag detection (apriltag_detect, the same code the
live preview used), and solves the (t, x, y, yaw, visible) truth CSV that fit_plant_calib.py consumes.
Two solves, mirroring capture:
  1. The floor extrinsic from the /floor/image lock burst (board visible).
  2. The robot tag pose in that floor frame, per /camera/image frame.

Because the images are recorded raw, the floor lock, detector tuning, intrinsics, and yaw offset can all
be corrected here and re-run without re-driving the robot.

Usage:
    source scripts/activate_python.sh
    python playground/calibration/analyze_apriltag_mcap.py \
        playground/calibration/out/apriltag_track.mcap --out playground/calibration/out/truth_log.csv
    # Re-solve with a corrected yaw offset (overrides the value recorded at capture):
    python playground/calibration/analyze_apriltag_mcap.py rec.mcap --yaw-offset-deg 90 --plot fit.png
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np
from calib_lib import apriltag_detect as ad
from calib_lib import apriltag_mcap as amcap


def solve_floor(metadata: dict, path: Path, detector, k: np.ndarray, d: np.ndarray):
    """Re-solve the floor extrinsic (R_fc, t_fc) from the recorded /floor/image lock burst."""
    floor = metadata["floor"]
    board = ad.make_floor_board(
        int(floor["cols"]), int(floor["rows"]), float(floor["marker_size"]),
        float(floor["marker_sep"]), int(floor["first_id"]),
    )
    frames = amcap.read_floor_frames(path)
    if not frames:
        raise SystemExit(f"{path}: no {amcap.TOPIC_FLOOR_IMAGE} frames; cannot re-solve the floor frame.")
    return ad.solve_floor_extrinsic(frames, detector, board, k, d)


def solve_poses(metadata: dict, path: Path, tag_id: int, yaw_offset: float) -> list[dict]:
    """Detect the robot tag in each /camera/image frame and solve its field-plane (t, x, y, yaw, visible)."""
    k = np.asarray(metadata["camera_matrix"], dtype=np.float64).reshape(3, 3)
    d = np.asarray(metadata["dist_coeffs"], dtype=np.float64).reshape(-1, 1)
    detector = ad.make_detector()
    r_fc, t_fc = solve_floor(metadata, path, detector, k, d)
    obj = ad.tag_object_points(float(metadata["tag_size"]))

    rows: list[dict] = []
    for t, frame in amcap.iter_images(path, amcap.TOPIC_CAMERA_IMAGE):
        _, corners, ids = ad.detect_markers(detector, frame)
        pose = None
        if ids is not None and tag_id in ids.flatten():
            idx = int(np.where(ids.flatten() == tag_id)[0][0])
            pose = ad.tag_pose_field(corners[idx], obj, k, d, r_fc, t_fc, yaw_offset)
        if pose is not None:
            rows.append({"t": t, "x": pose[0], "y": pose[1], "yaw": pose[2], "visible": 1})
        else:
            rows.append({"t": t, "x": None, "y": None, "yaw": None, "visible": 0})
    return rows


def write_csv(path: Path, rows: list[dict]) -> None:
    """Write the truth CSV in fit_plant_calib.py's expected (t, x, y, yaw, visible) format."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "x", "y", "yaw", "visible"])
        for r in rows:
            if r["visible"]:
                writer.writerow([f"{r['t']:.4f}", f"{r['x']:.5f}", f"{r['y']:.5f}", f"{r['yaw']:.5f}", 1])
            else:
                writer.writerow([f"{r['t']:.4f}", "", "", "", 0])


def print_summary(rows: list[dict], tag_id: int) -> None:
    n = len(rows)
    seen = [r for r in rows if r["visible"]]
    if n == 0:
        raise SystemExit("recording contained no /camera/image frames")
    span = rows[-1]["t"] - rows[0]["t"]
    print(f"robot tag id {tag_id}: {len(seen)}/{n} frames ({100.0 * len(seen) / n:.1f}%) over {span:.1f} s")
    if not seen:
        print("WARNING: tag never solved; check --tag-id and that the robot was in view.")
        return
    ts = np.array([r["t"] for r in seen])
    dt = np.median(np.diff(ts)) if len(ts) > 1 else float("nan")
    x = np.array([r["x"] for r in seen])
    y = np.array([r["y"] for r in seen])
    yaw_deg = np.degrees([r["yaw"] for r in seen])
    print(f"  median dt {dt * 1e3:.1f} ms (~{1.0 / dt:.0f} fps)" if dt == dt else "  single frame")
    print(f"  x [{x.min():+.3f}, {x.max():+.3f}] m   y [{y.min():+.3f}, {y.max():+.3f}] m")
    print(f"  yaw [{yaw_deg.min():+.1f}, {yaw_deg.max():+.1f}] deg")


def save_plot(path: Path, rows: list[dict], tag_id: int) -> None:
    import matplotlib.pyplot as plt

    seen = [r for r in rows if r["visible"]]
    if not seen:
        print("no solved poses to plot")
        return
    t0 = rows[0]["t"]
    t = [r["t"] - t0 for r in seen]
    x = [r["x"] for r in seen]
    y = [r["y"] for r in seen]
    yaw_deg = [math.degrees(r["yaw"]) for r in seen]

    fig, (ax_xy, ax_yaw) = plt.subplots(1, 2, figsize=(12, 5))
    sc = ax_xy.scatter(x, y, c=t, cmap="viridis", s=8)
    ax_xy.plot(x[0], y[0], "go", label="start")
    ax_xy.plot(x[-1], y[-1], "rs", label="end")
    ax_xy.set_aspect("equal", "datalim")
    ax_xy.set_xlabel("x (m)")
    ax_xy.set_ylabel("y (m)")
    ax_xy.set_title(f"robot tag {tag_id} field-plane track")
    ax_xy.legend()
    fig.colorbar(sc, ax=ax_xy, label="t (s)")

    ax_yaw.plot(t, yaw_deg, ".", ms=3)
    ax_yaw.set_xlabel("t (s)")
    ax_yaw.set_ylabel("yaw (deg)")
    ax_yaw.set_title("yaw over time")

    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=120)
    print(f"plot: {path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("mcap", type=Path, help="image recording from apriltag_track.py")
    parser.add_argument("--out", type=Path, default=None,
                        help="truth CSV path (default: <mcap stem>.csv next to the recording)")
    parser.add_argument("--tag-id", type=int, default=None,
                        help="robot tag id to solve (default: the id recorded in the metadata)")
    parser.add_argument("--yaw-offset-deg", type=float, default=None,
                        help="override the yaw offset recorded at capture")
    parser.add_argument("--plot", type=Path, default=None, help="also save a trajectory plot to this path")
    args = parser.parse_args()

    metadata = amcap.read_metadata(args.mcap)
    tag_id = args.tag_id if args.tag_id is not None else int(metadata["tag_id"])
    yaw_offset_deg = (args.yaw_offset_deg if args.yaw_offset_deg is not None
                      else float(metadata["yaw_offset_deg"]))

    rows = solve_poses(metadata, args.mcap, tag_id, math.radians(yaw_offset_deg))

    out = args.out if args.out is not None else args.mcap.with_suffix(".csv")
    write_csv(out, rows)
    print_summary(rows, tag_id)
    print(f"truth log: {out}")
    if args.plot is not None:
        save_plot(args.plot, rows, tag_id)


if __name__ == "__main__":
    main()
