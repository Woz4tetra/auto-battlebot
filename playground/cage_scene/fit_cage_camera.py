#!/usr/bin/env python3
"""Fit the cage-high camera pose per clip from the rectified mat hull.

For each `targets/<clip>/` written by extract_targets.py: fit straight lines to the mat
outline where it is actually visible (the near edge runs off the bottom of the cage-high
frame, so usually three sides), solve the pose against the known mat size at the rectified K
by least squares from a seed pose, and check the answer is a camera above the mat at a
plausible height and tilt. The seed is the true_battlebot Cage 2 prior when it exists, else
the four-corner homography. A clip whose outline is fully in frame is solved from four
corners first and refined on its four lines. Poses are written in the `config/cages/*.toml`
format the C++ `CalibratedFieldFilter` reads, one per clip and, with --event-pose, one per
event (median translation, chordal-mean rotation) for rendering sweeps.

`--landmarks <json>` adds hand-picked correspondences `[{"image": [u, v], "field": [x, y, z]}]`
in hfield metres (see `auto_battlebot.perception.cage_calibration`) and refines the
homography pose with solvePnP. Use it only when the grading says the far edge is off.

Usage:
    venv/bin/python playground/cage_scene/fit_cage_camera.py runs/cage_scene --event-pose
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from auto_battlebot.perception.cage_calibration import (
    CageCalibration,
    camera_from_world,
    load_prior_pose,
    pose_summary,
    save_cage_calibration,
    world_from_camera,
)
from auto_battlebot.perception.field_pose import (
    contour_points_off_border,
    largest_contour_mask,
    order_corners,
    pose_from_corners,
    pose_from_lines,
    project_field_corners,
    quad_from_mask,
    side_lines_from_points,
    touches_border,
)

PRIOR_DIR = Path("~/Documents/true_battlebot/perception/configs/metrics_tool").expanduser()
DEFAULT_SEED_PRIOR = PRIOR_DIR / "nhrl_cage_2.toml"
LINE_FIT_ROUNDS = 3
# The undistorted hull polygon lands a few pixels inside the frame where the mat ran off it.
BORDER_MARGIN_PX = 6
# A fitted side this close to, and parallel with, an image edge is the sensor border, not the mat.
BORDER_LINE_PX = 14.0
# NHRL squares the camera to the mat; a fit yawed past this followed something other than the mat.
YAW_LIMIT_DEG = 10.0
HEIGHT_RANGE_M = (1.0, 2.0)
TILT_RANGE_DEG = (20.0, 40.0)


def load_k_rect(path: Path) -> np.ndarray:
    data = json.loads(path.read_text())
    return np.array([[data["fx"], 0.0, data["cx"]], [0.0, data["fy"], data["cy"]], [0.0, 0.0, 1.0]])


def load_landmarks(path: Path | None) -> tuple[np.ndarray, np.ndarray] | None:
    if path is None or not path.exists():
        return None
    records = json.loads(path.read_text())
    image = np.array([r["image"] for r in records], dtype=np.float64)
    field = np.array([r["field"] for r in records], dtype=np.float64)
    return image, field


def refine_with_landmarks(
    tf_cam_from_hfield: np.ndarray, landmarks: tuple[np.ndarray, np.ndarray], k_rect: np.ndarray
) -> tuple[np.ndarray, float]:
    image, field = landmarks
    rvec0, _ = cv2.Rodrigues(tf_cam_from_hfield[:3, :3])
    tvec0 = tf_cam_from_hfield[:3, 3].reshape(3, 1)
    ok, rvec, tvec = cv2.solvePnP(
        field,
        image,
        k_rect,
        None,
        rvec0,
        tvec0,
        useExtrinsicGuess=True,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        return tf_cam_from_hfield, float("nan")
    tf = np.eye(4)
    tf[:3, :3], _ = cv2.Rodrigues(rvec)
    tf[:3, 3] = tvec.ravel()
    projected, _ = cv2.projectPoints(field, rvec, tvec, k_rect, None)
    residual = float(np.linalg.norm(projected.reshape(-1, 2) - image, axis=1).mean())
    return tf, residual


def drop_border_lines(
    lines: list[tuple[np.ndarray, float] | None], width: int, height: int
) -> list[tuple[np.ndarray, float] | None]:
    """None out any side line that coincides with an image border."""
    out: list[tuple[np.ndarray, float] | None] = []
    for line in lines:
        if line is None:
            out.append(None)
            continue
        normal, offset = line
        nx, ny = abs(float(normal[0])), abs(float(normal[1]))
        on_border = False
        if ny > 0.97:  # horizontal line: y = offset / normal_y
            y = offset / float(normal[1])
            on_border = min(abs(y), abs(height - 1 - y)) < BORDER_LINE_PX
        elif nx > 0.97:
            x = offset / float(normal[0])
            on_border = min(abs(x), abs(width - 1 - x)) < BORDER_LINE_PX
        out.append(None if on_border else line)
    return out


def chordal_mean_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    u, _, vh = np.linalg.svd(np.sum(rotations, axis=0))
    rotation = u @ vh
    if np.linalg.det(rotation) < 0:
        rotation = u @ np.diag([1.0, 1.0, -1.0]) @ vh
    return np.asarray(rotation)


def draw_overlay(
    target: np.ndarray,
    quad: np.ndarray,
    tf_cam_from_hfield: np.ndarray,
    mat: tuple[float, float],
    k_rect: np.ndarray,
    text: list[str],
) -> np.ndarray:
    out = target.copy()
    projected = project_field_corners(tf_cam_from_hfield, mat, k_rect)
    cv2.polylines(out, [np.round(quad).astype(np.int32)], True, (0, 255, 255), 2)
    cv2.polylines(out, [np.round(projected).astype(np.int32)], True, (255, 0, 255), 2)
    # W axes from the mat centre: x red, y green, z blue, 0.5 m each.
    tf_w_from_cam = world_from_camera(tf_cam_from_hfield)
    tf_cam_from_w = np.linalg.inv(tf_w_from_cam)
    axes_w = np.array(
        [[0, 0, 0, 1], [0.5, 0, 0, 1], [0, 0.5, 0, 1], [0, 0, 0.5, 1]], dtype=np.float64
    )
    cam = (tf_cam_from_w @ axes_w.T)[:3]
    px = k_rect @ cam
    px = (px[:2] / px[2]).T
    origin = tuple(np.round(px[0]).astype(int))
    for end, color in zip(px[1:], [(0, 0, 255), (0, 255, 0), (255, 0, 0)]):
        cv2.arrowedLine(out, origin, tuple(np.round(end).astype(int)), color, 3, tipLength=0.1)
    for i, line in enumerate(text):
        cv2.putText(out, line, (20, 40 + 32 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 4)
        cv2.putText(out, line, (20, 40 + 32 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    return out


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "scene_dir", type=Path, help="runs/cage_scene, holding targets/ and camera_rect.json"
    )
    parser.add_argument("--mat-size", type=float, nargs=2, default=[2.35, 2.35], metavar=("X", "Y"))
    parser.add_argument(
        "--landmarks-dir", type=Path, help="<event>.json or <clip>.json correspondences"
    )
    parser.add_argument("--event-pose", action="store_true", help="also write one pose per event")
    parser.add_argument("--prior-dir", type=Path, default=PRIOR_DIR)
    parser.add_argument(
        "--seed-prior",
        type=Path,
        default=DEFAULT_SEED_PRIOR,
        help="true_battlebot metrics_tool camera TOML seeding clipped outlines",
    )
    args = parser.parse_args()

    k_rect = load_k_rect(args.scene_dir / "camera_rect.json")
    mat = (float(args.mat_size[0]), float(args.mat_size[1]))
    poses_dir = args.scene_dir / "poses"
    poses_dir.mkdir(parents=True, exist_ok=True)

    priors: dict[str, dict[str, Any]] = {}
    for prior_file in (
        sorted(args.prior_dir.glob("nhrl_cage_*.toml")) if args.prior_dir.exists() else []
    ):
        priors[prior_file.stem] = pose_summary(load_prior_pose(prior_file))

    rows: list[dict[str, Any]] = []
    per_event: dict[str, list[np.ndarray]] = {}
    event_cage: dict[str, str] = {}
    seed_prior: np.ndarray | None = None
    if args.seed_prior is not None and args.seed_prior.exists():
        seed_prior = camera_from_world(load_prior_pose(args.seed_prior))
        print(f"seed prior: {args.seed_prior} -> {pose_summary(load_prior_pose(args.seed_prior))}")
    for target_dir in sorted((args.scene_dir / "targets").iterdir()):
        mask_path = target_dir / "hull_mask.png"
        if not mask_path.exists():
            continue
        meta = json.loads((target_dir / "meta.json").read_text())
        clip, event = meta["clip"], str(meta.get("event_group") or meta.get("event", "unknown"))
        mask = largest_contour_mask(cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE))
        row: dict[str, Any] = {"clip": clip, "event": event, "cage": meta.get("cage")}
        row["clipped"] = touches_border(mask)
        points = contour_points_off_border(mask, border_margin_px=BORDER_MARGIN_PX)

        # The hull's own four-vertex outline assigns contour points to sides even when the
        # near edge is the image border; the border side is rejected below.
        quad = quad_from_mask(mask)
        seed_quad = order_corners(quad) if quad is not None else None
        tf_cam_from_hfield: np.ndarray | None = None
        reprojection_px = float("nan")
        if quad is not None and not row["clipped"]:
            corners = order_corners(quad)
            quad_area = abs(cv2.contourArea(corners.astype(np.float32)))
            row["mask_over_quad_area"] = float((mask > 0).sum()) / max(1.0, quad_area)
            solved = pose_from_corners(corners, mat, k_rect)
            if solved is not None:
                tf_cam_from_hfield, reprojection_px = solved
        if tf_cam_from_hfield is None:
            if seed_prior is None:
                row["status"] = "clipped outline and no seed prior"
                rows.append(row)
                print(f"{clip}: {row['status']}")
                continue
            tf_cam_from_hfield = seed_prior

        # Refine on the visible edge lines, re-seeding the side assignment from each solution.
        supported = 0
        line_rms = float("nan")
        height, width = mask.shape[:2]
        for round_index in range(LINE_FIT_ROUNDS):
            if seed_quad is None or round_index > 0:
                seed_quad = project_field_corners(tf_cam_from_hfield, mat, k_rect)
            lines = drop_border_lines(side_lines_from_points(points, seed_quad), width, height)
            supported = sum(line is not None for line in lines)
            refined = pose_from_lines(lines, mat, k_rect, tf_cam_from_hfield)
            if refined is None:
                break
            tf_cam_from_hfield, line_rms = refined
        row["sides_supported"] = supported
        row["line_rms_px"] = line_rms
        row["reprojection_px"] = reprojection_px
        corners = project_field_corners(tf_cam_from_hfield, mat, k_rect)
        row["landmark_residual_px"] = ""
        landmarks = None
        if args.landmarks_dir is not None:
            landmarks = load_landmarks(args.landmarks_dir / f"{clip}.json") or load_landmarks(
                args.landmarks_dir / f"{event}.json"
            )
        if landmarks is not None:
            tf_cam_from_hfield, residual = refine_with_landmarks(
                tf_cam_from_hfield, landmarks, k_rect
            )
            row["landmark_residual_px"] = residual

        r3, t = tf_cam_from_hfield[:3, 2], tf_cam_from_hfield[:3, 3]
        row["r3_dot_t"] = float(r3 @ t)
        tf_w_from_cam = world_from_camera(tf_cam_from_hfield)
        summary = pose_summary(tf_w_from_cam)
        row.update({k: round(v, 4) for k, v in summary.items()})
        far_point = np.linalg.inv(tf_w_from_cam) @ np.array([0.0, 1.0, 0.0, 1.0])
        far_px = k_rect @ far_point[:3]
        far_v = far_px[1] / far_px[2]
        checks = {
            "normal_faces_camera": row["r3_dot_t"] < 0,
            "height_ok": HEIGHT_RANGE_M[0] <= summary["height_m"] <= HEIGHT_RANGE_M[1],
            "tilt_ok": TILT_RANGE_DEG[0] <= summary["tilt_from_down_deg"] <= TILT_RANGE_DEG[1],
            "far_edge_above_centre": far_v < k_rect[1, 2],
            "yaw_ok": abs(summary["yaw_deg"]) <= YAW_LIMIT_DEG,
            "three_sides_seen": supported >= 3,
            "lines_fit": line_rms == line_rms and line_rms < 6.0,
        }
        row.update(checks)
        row["status"] = (
            "ok"
            if all(checks.values())
            else "CHECK " + ",".join(k for k, v in checks.items() if not v)
        )
        for name, prior in priors.items():
            row[f"d_{name}_m"] = round(
                float(np.hypot(summary["x_m"] - prior["x_m"], summary["y_m"] - prior["y_m"])), 3
            )
            row[f"d_{name}_tilt_deg"] = round(
                summary["tilt_from_down_deg"] - prior["tilt_from_down_deg"], 2
            )
        rows.append(row)
        if row["status"] == "ok":
            per_event.setdefault(event, []).append(tf_w_from_cam)
        event_cage[event] = str(meta.get("cage"))

        header = (
            f"Fixed cage-high camera fit from {clip}\n"
            f"mat {mat[0]} x {mat[1]} m assumed; {supported} sides seen, "
            f"line RMS {line_rms:.2f} px; height {summary['height_m']:.3f} m, "
            f"tilt {summary['tilt_from_down_deg']:.1f} deg\n"
            "Written by playground/cage_scene/fit_cage_camera.py"
        )
        cage_id = f"brettzone_cage{meta.get('cage')}_{event}_{meta.get('game_id')}".replace(
            "-", "_"
        )
        save_cage_calibration(
            poses_dir / f"{clip}.toml",
            CageCalibration(cage_id, mat[0], mat[1], tf_cam_from_hfield),
            header,
        )
        target = cv2.imread(str(target_dir / "target.png"))
        text = [
            clip,
            f"height {summary['height_m']:.2f} m  tilt {summary['tilt_from_down_deg']:.1f} deg  "
            f"yaw {summary['yaw_deg']:.1f} deg",
            f"{supported} sides  line rms {line_rms:.2f} px",
            row["status"],
        ]
        cv2.imwrite(
            str(poses_dir / f"{clip}_overlay.png"),
            draw_overlay(target, corners, tf_cam_from_hfield, mat, k_rect, text),
        )
        print(
            f"{clip}: {row['status']}  h {summary['height_m']:.3f} m  "
            f"tilt {summary['tilt_from_down_deg']:.1f}  yaw {summary['yaw_deg']:.1f}  "
            f"{supported} sides  line rms {line_rms:.2f} px"
        )

    if args.event_pose:
        for event, transforms in per_event.items():
            cage = event_cage.get(event, "x")
            translation = np.median([tf[:3, 3] for tf in transforms], axis=0)
            rotation = chordal_mean_rotation([tf[:3, :3] for tf in transforms])
            tf_w_from_cam = np.eye(4)
            tf_w_from_cam[:3, :3] = rotation
            tf_w_from_cam[:3, 3] = translation
            summary = pose_summary(tf_w_from_cam)
            header = (
                f"Event pose for {event}: median translation and chordal-mean rotation over "
                f"{len(transforms)} clips\nheight {summary['height_m']:.3f} m, "
                f"tilt {summary['tilt_from_down_deg']:.1f} deg\n"
                "Written by playground/cage_scene/fit_cage_camera.py --event-pose"
            )
            cage_id = f"brettzone_cage{cage}_{event}".replace("-", "_")
            save_cage_calibration(
                poses_dir / f"cage{cage}_{event}.toml",
                CageCalibration(cage_id, mat[0], mat[1], camera_from_world(tf_w_from_cam)),
                header,
            )
            print(
                f"event {event}: {len(transforms)} clips, h {summary['height_m']:.3f} m, "
                f"tilt {summary['tilt_from_down_deg']:.1f} deg"
            )

    if rows:
        keys: list[str] = []
        for row in rows:
            keys += [k for k in row if k not in keys]
        with (poses_dir / "summary.csv").open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=keys)
            writer.writeheader()
            writer.writerows(rows)
        print(f"summary: {poses_dir / 'summary.csv'}")


if __name__ == "__main__":
    main()
