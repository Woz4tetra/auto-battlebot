"""Solve field-plane robot poses from an apriltag_track.py image recording."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import cv2
import numpy as np

from auto_battlebot.calibration.apriltag import apriltag_detect as ad
from auto_battlebot.calibration.apriltag import apriltag_mcap as amcap

OVERLAY_DEFAULT = object()

ROBOT_DETECT_SCALE = 2.0


def solve_floor(
    metadata: dict,
    path: Path,
    detector: cv2.aruco.ArucoDetector,
    k: np.ndarray,
    d: np.ndarray,
) -> ad.FieldToCamera:
    """Re-solve the floor extrinsic (R_fc, t_fc) from the recorded /floor/image lock burst."""
    floor = metadata["floor"]
    board = ad.make_floor_board(
        int(floor["cols"]),
        int(floor["rows"]),
        float(floor["marker_size"]),
        float(floor["marker_sep"]),
        int(floor["first_id"]),
    )
    frames = amcap.read_floor_frames(path)
    if not frames:
        raise SystemExit(
            f"{path}: no {amcap.TOPIC_FLOOR_IMAGE} frames; cannot re-solve the floor frame."
        )
    return ad.solve_floor_extrinsic(frames, detector, board, k, d)


def tag_field_height(
    pose: tuple[float, float, float, np.ndarray, np.ndarray],
    r_fc: np.ndarray,
    t_fc: np.ndarray,
) -> float:
    tvec = np.asarray(pose[4], dtype=np.float64).reshape(3)
    return float((r_fc.T @ (tvec - t_fc))[2])


def attach_sticks(row: dict, channels_by_t: dict, t: float) -> None:
    tx = channels_by_t.get(t)
    if tx is not None:
        row["sample_t"] = tx["sample_t"]
        row["channels"] = tx["channels"]


def attach_zoh_command(row: dict, commands: list, cmd_t: np.ndarray, t: float) -> None:
    if cmd_t.size:
        ci = int(np.searchsorted(cmd_t, t, side="right")) - 1
        if ci >= 0:
            c = commands[ci]
            row["cmd_lin"] = c["cmd_lin"]
            row["cmd_ang"] = c["cmd_ang"]
            row["trainer_lin"] = c["trainer_lin"]
            row["trainer_ang"] = c["trainer_ang"]
            row["label"] = c["label"]


def solve_poses(
    metadata: dict, path: Path, tag_id: int, yaw_offset: float
) -> tuple[list[dict], np.ndarray, np.ndarray]:
    """Detect the robot tag per frame and solve its field-plane pose."""
    k = np.asarray(metadata["camera_matrix"], dtype=np.float64).reshape(3, 3)
    d = np.asarray(metadata["dist_coeffs"], dtype=np.float64).reshape(-1, 1)
    detector = ad.make_detector()
    r_fc, t_fc = solve_floor(metadata, path, detector, k, d)
    obj = ad.tag_object_points(float(metadata["tag_size"]))

    channels_by_t = amcap.read_channels(path)
    commands = amcap.read_commands(path)
    cmd_t = np.array([c["t"] for c in commands], dtype=np.float64)

    rows: list[dict] = []
    for t, frame in amcap.iter_images(path, amcap.TOPIC_CAMERA_IMAGE):
        _, corners, ids = ad.detect_markers(detector, frame, scale=ROBOT_DETECT_SCALE)
        pose = None
        if ids is not None and tag_id in ids.flatten():
            idx = int(np.where(ids.flatten() == tag_id)[0][0])
            pose = ad.tag_pose_field(corners[idx], obj, k, d, r_fc, t_fc, yaw_offset)
        if pose is not None:
            z = tag_field_height(pose, r_fc, t_fc)
            row: dict = {"t": t, "x": pose[0], "y": pose[1], "yaw": pose[2], "z": z, "visible": 1}
        else:
            row = {"t": t, "x": None, "y": None, "yaw": None, "z": None, "visible": 0}
        attach_sticks(row, channels_by_t, t)
        attach_zoh_command(row, commands, cmd_t, t)
        rows.append(row)
    return rows, r_fc, t_fc


CMD_COLUMNS = ["cmd_lin", "cmd_ang", "trainer_lin", "trainer_ang", "label"]


def write_csv(path: Path, rows: list[dict]) -> None:
    """Write the truth CSV in the column format fit_plant_calib.py reads by name."""
    n_ch = max((len(r["channels"]) for r in rows if "channels" in r), default=0)
    has_cmd = any("cmd_lin" in r for r in rows)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        header = ["t", "x", "y", "yaw", "visible"]
        if n_ch:
            header += ["tx_sample_t"] + [f"ch{i}" for i in range(n_ch)]
        if has_cmd:
            header += CMD_COLUMNS
        writer.writerow(header)
        for r in rows:
            if r["visible"]:
                row = [f"{r['t']:.4f}", f"{r['x']:.5f}", f"{r['y']:.5f}", f"{r['yaw']:.5f}", 1]
            else:
                row = [f"{r['t']:.4f}", "", "", "", 0]
            if n_ch:
                if "channels" in r:
                    row += [f"{r['sample_t']:.4f}"] + [str(c) for c in r["channels"]]
                else:
                    row += [""] * (1 + n_ch)
            if has_cmd:
                if "cmd_lin" in r:
                    row += [
                        f"{r['cmd_lin']:.4f}",
                        f"{r['cmd_ang']:.4f}",
                        str(r["trainer_lin"]),
                        str(r["trainer_ang"]),
                        r["label"],
                    ]
                else:
                    row += [""] * len(CMD_COLUMNS)
            writer.writerow(row)


def print_summary(rows: list[dict], tag_id: int) -> None:
    n = len(rows)
    seen = [r for r in rows if r["visible"]]
    if n == 0:
        raise SystemExit("recording contained no /camera/image frames")
    span = rows[-1]["t"] - rows[0]["t"]
    print(
        f"robot tag id {tag_id}: {len(seen)}/{n} frames ({100.0 * len(seen) / n:.1f}%) over "
        f"{span:.1f} s"
    )
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


HITCH_GAP_FACTOR = 1.5


def print_timing(rows: list[dict]) -> None:
    """Report frame-rate consistency over every recorded frame, solved or not."""
    ts = np.array([r["t"] for r in rows], dtype=np.float64)
    if len(ts) < 2:
        return
    dt = np.diff(ts)
    dt = dt[dt > 0.0]
    if dt.size == 0:
        return
    med = float(np.median(dt))
    dropped = int(np.maximum(0, np.round(dt / med).astype(int) - 1).sum())
    hitches = int(np.count_nonzero(dt > HITCH_GAP_FACTOR * med))
    worst_i = int(np.argmax(dt))
    worst_dt = float(dt[worst_i])
    print(f"frame timing: {len(ts)} frames over {ts[-1] - ts[0]:.1f} s")
    print(
        f"  median {med * 1e3:.1f} ms ({1.0 / med:.1f} fps)   mean {dt.mean() * 1e3:.1f} ms   "
        f"jitter(std) {dt.std() * 1e3:.1f} ms"
    )
    print(
        f"  dt range [{dt.min() * 1e3:.1f}, {dt.max() * 1e3:.1f}] ms   "
        f"p95 {np.percentile(dt, 95) * 1e3:.1f} ms   p99 {np.percentile(dt, 99) * 1e3:.1f} ms"
    )
    if hitches:
        worst_t = float(ts[worst_i + 1] - ts[0])
        print(
            f"  hitches (dt > 1.5x median): {hitches} "
            f"({100.0 * hitches / dt.size:.1f}% of frames)   "
            f"est. dropped frames: {dropped}"
        )
        print(
            f"  worst: {worst_dt * 1e3:.1f} ms gap at t={worst_t:.2f} s "
            f"(~{int(round(worst_dt / med)) - 1} frames skipped)"
        )
    else:
        print(f"  no hitches (every gap within {HITCH_GAP_FACTOR}x median)")


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
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="truth CSV path (default: <mcap stem>.csv next to the recording)",
    )
    parser.add_argument(
        "--tag-id",
        type=int,
        default=None,
        help="robot tag id to solve (default: the id recorded in the metadata)",
    )
    parser.add_argument(
        "--yaw-offset-deg",
        type=float,
        default=None,
        help="override the yaw offset recorded at capture",
    )
    parser.add_argument(
        "--plot", type=Path, default=None, help="also save a trajectory plot to this path"
    )
    parser.add_argument(
        "--overlay-mcap",
        type=Path,
        nargs="?",
        default=None,
        const=OVERLAY_DEFAULT,
        help="write a Foxglove overlay MCAP of the solved poses; no argument means "
        "<mcap stem>_overlay.mcap",
    )
    args = parser.parse_args()

    metadata = amcap.read_metadata(args.mcap)
    tag_id = args.tag_id if args.tag_id is not None else int(metadata["tag_id"])
    yaw_offset_deg = (
        args.yaw_offset_deg
        if args.yaw_offset_deg is not None
        else float(metadata["yaw_offset_deg"])
    )

    rows, r_fc, t_fc = solve_poses(metadata, args.mcap, tag_id, math.radians(yaw_offset_deg))

    out = args.out if args.out is not None else args.mcap.with_suffix(".csv")
    write_csv(out, rows)
    print_summary(rows, tag_id)
    print_timing(rows)
    n_tx = sum("channels" in r for r in rows)
    n_cmd = sum("cmd_lin" in r for r in rows)
    if n_cmd:
        print(
            f"transmitter: scripted commands on {n_cmd}/{len(rows)} frames "
            "(--drive; fit with fit_plant_calib.py)"
        )
    elif n_tx:
        n_ch = max(len(r["channels"]) for r in rows if "channels" in r)
        print(f"transmitter: stick axes on {n_tx}/{len(rows)} frames ({n_ch} channels each)")
    else:
        print("transmitter: no stick axes or commands in this recording")
    print(f"truth log: {out}")
    if args.plot is not None:
        save_plot(args.plot, rows, tag_id)
    if args.overlay_mcap is not None:
        overlay_path = args.overlay_mcap
        if overlay_path is OVERLAY_DEFAULT:
            overlay_path = args.mcap.with_name(f"{args.mcap.stem}_overlay.mcap")
        amcap.write_overlay(overlay_path, args.mcap, metadata, rows, r_fc, t_fc)
        print(f"overlay mcap: {overlay_path}")


if __name__ == "__main__":
    main()
