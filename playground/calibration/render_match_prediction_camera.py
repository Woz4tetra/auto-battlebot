"""Render open-loop prediction videos reprojected into the camera view.

Same content as `render_match_prediction.py` (plant integrates the transmitted
commands open loop, resetting to the measured pose every few seconds) but drawn
over the actual SVO RGB frames: measured footprint in blue, predicted ghost in
orange, both projected onto the field plane through the replay's per-tick
field->camera_world->camera transforms and camera intrinsics.

The SVO file is taken from `/camera/frame_meta` (it records the path the replay
played), and the Nth decoded left-eye JPEG is SVO frame N, so frames pair with
ticks exactly. Projection uses the tick's own field->camera_world transform, so
field re-initializations mid-replay cannot smear the overlay.

Usage:
    source scripts/activate_python.sh
    python playground/calibration/render_match_prediction_camera.py \\
        --pair <replay>.mcap:<original>.mcap:2026-05-01_17-42-20 \\
        --params playground/calibration/out/plant_match.toml \\
        --cmd-lead-ms 40 \\
        --out-dir playground/calibration/out/match_fit/videos
"""

from __future__ import annotations

import argparse
import json
import subprocess
from contextlib import closing
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from auto_battlebot import svo2
from auto_battlebot.calibration.match_windows import (
    build_match_run,
    load_commands,
    load_replay_track,
)
from auto_battlebot.mcap_io import (
    decode_camera_info,
    decode_string,
    decode_tf_message,
    iter_messages,
)
from auto_battlebot.plant import FULL_MODEL, PlantParams, wrap_angle
from playground.calibration.render_match_prediction import ROBOT_HALF_M, predict_track

BLUE = (200, 100, 30)  # BGR
ORANGE = (30, 140, 240)
RED = (60, 60, 230)
WHITE = (240, 240, 240)


@dataclass
class CameraTrack:
    """Per-tick projection data pulled from a replay mcap."""

    svo_index: np.ndarray  # (M,) per tick with a valid frame
    stamp_ns: np.ndarray
    cam_from_field: np.ndarray  # (M, 4, 4), nan rows where no transform yet
    intrinsics: np.ndarray  # 3x3 K
    width: int  # frame size; the recordings vary (1080p practice, 720p fights)
    height: int
    svo_path: Path


def load_camera_track(replay_path: Path | str) -> CameraTrack:
    metas: list[tuple[int, int, int, str]] = []
    for _topic, ts, data in iter_messages(replay_path, ["/camera/frame_meta"]):
        payload = json.loads(decode_string(data))
        metas.append(
            (
                ts,
                int(payload["image_stamp_ns"]),
                int(payload["svo_frame_index"]),
                payload["svo_path"],
            )
        )
    meta_lt = np.array([m[0] for m in metas], dtype=np.int64)

    info = None
    for _topic, _ts, data in iter_messages(replay_path, ["/camera/camera_info"]):
        info = decode_camera_info(data)
        break
    if info is None:
        raise ValueError(f"{replay_path} has no /camera/camera_info")

    field_from_world: dict[int, np.ndarray] = {}
    world_from_cam: dict[int, np.ndarray] = {}
    for _topic, ts, data in iter_messages(replay_path, ["/tf"]):
        tick = int(np.searchsorted(meta_lt, ts)) - 1
        for tr in decode_tf_message(data):
            if tr.key == ("field", "camera_world"):
                field_from_world[tick] = tr.matrix
            elif tr.key == ("camera_world", "camera"):
                world_from_cam[tick] = tr.matrix

    rows = []
    mats = []
    last_fw = None
    last_wc = None
    for tick, (_lt, stamp, svo_index, _path) in enumerate(metas):
        if svo_index < 0:
            continue
        last_fw = field_from_world.get(tick, last_fw)
        last_wc = world_from_cam.get(tick, last_wc)
        rows.append((svo_index, stamp))
        if last_fw is None or last_wc is None:
            mats.append(np.full((4, 4), np.nan))
        else:
            mats.append(np.linalg.inv(last_fw @ last_wc))
    return CameraTrack(
        svo_index=np.array([r[0] for r in rows], dtype=np.int64),
        stamp_ns=np.array([r[1] for r in rows], dtype=np.int64),
        cam_from_field=np.stack(mats),
        intrinsics=info.intrinsics,
        width=info.width,
        height=info.height,
        svo_path=Path(metas[0][3]),
    )


def project_points(
    cam_from_field: np.ndarray, intrinsics: np.ndarray, pts_field: np.ndarray
) -> np.ndarray:
    """Field-plane points (N, 2 or 3) to pixel coords (N, 2); nan when behind."""
    pts = np.asarray(pts_field, dtype=float)
    if pts.shape[1] == 2:
        pts = np.hstack([pts, np.zeros((len(pts), 1))])
    cam = pts @ cam_from_field[:3, :3].T + cam_from_field[:3, 3]
    out = np.full((len(pts), 2), np.nan)
    ok = cam[:, 2] > 0.05
    out[ok, 0] = intrinsics[0, 0] * cam[ok, 0] / cam[ok, 2] + intrinsics[0, 2]
    out[ok, 1] = intrinsics[1, 1] * cam[ok, 1] / cam[ok, 2] + intrinsics[1, 2]
    return out


def _robot_footprint(x: float, y: float, theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    rot = np.array([[c, -s], [s, c]])
    square = ROBOT_HALF_M * np.array([[1, 1], [-1, 1], [-1, -1], [1, -1]], dtype=float)
    return square @ rot.T + (x, y)


def _poly(img: np.ndarray, uv: np.ndarray, color, thickness=2, closed=True) -> None:
    keep = np.isfinite(uv).all(axis=1)
    if keep.sum() < 2:
        return
    pts = np.clip(uv[keep], -1e5, 1e5).astype(np.int32)
    cv2.polylines(img, [pts], closed, color, thickness, lineType=cv2.LINE_AA)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pair",
        type=str,
        action="append",
        required=True,
        metavar="REPLAY:ORIGINAL:NAME",
    )
    parser.add_argument(
        "--params", type=Path, default=Path("playground/calibration/out/plant_match.toml")
    )
    parser.add_argument("--reset-s", type=float, default=3.0)
    parser.add_argument("--cmd-lead-ms", type=float, default=40.0)
    parser.add_argument("--holdout-fraction", type=float, default=0.30)
    parser.add_argument(
        "--out-dir", type=Path, default=Path("playground/calibration/out/match_fit/videos")
    )
    parser.add_argument("--crf", type=int, default=23)
    args = parser.parse_args()

    params = PlantParams.from_toml(args.params)

    for pair in args.pair:
        replay_s, original_s, name = pair.split(":")
        replay, original = Path(replay_s), Path(original_s)
        print(f"\n{name}: loading")
        track = load_replay_track(replay)
        commands = load_commands(original)
        if args.cmd_lead_ms:
            commands.t_ns = commands.t_ns - int(args.cmd_lead_ms * 1e6)
        run = build_match_run(track, commands, name=name, role="video")
        cam = load_camera_track(replay)
        if not cam.svo_path.exists():
            raise SystemExit(f"SVO not found: {cam.svo_path} (recorded in frame_meta)")

        predicted = predict_track(run, params, FULL_MODEL, args.reset_s)
        holdout_from_s = None
        if "17-42-20" in name:
            holdout_from_s = float(run.t[int(len(run.t) * (1 - args.holdout_fraction))])

        # Tick -> grid slot, by the same rounding the grid was built with.
        slots = np.round((cam.stamp_ns - run.t0_ns) / (run.dt * 1e9)).astype(int)
        slots = np.clip(slots, 0, len(run.t) - 1)

        first_idx = int(cam.svo_index[0])
        last_idx = int(cam.svo_index[-1])
        index_to_tick = {int(v): i for i, v in enumerate(cam.svo_index)}

        out_path = args.out_dir / f"prediction_camera_{name}.mp4"
        out_path.parent.mkdir(parents=True, exist_ok=True)
        topic, _count = svo2.find_side_by_side_topic(cam.svo_path)
        fps = svo2.sample_fps(cam.svo_path, topic)

        encoder = subprocess.Popen(
            [
                "ffmpeg",
                "-hide_banner",
                "-loglevel",
                "error",
                "-y",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "bgr24",
                "-s",
                f"{cam.width}x{cam.height}",
                "-r",
                f"{fps:.6f}",
                "-i",
                "pipe:0",
                "-c:v",
                "libx264",
                "-preset",
                "veryfast",
                "-crf",
                str(args.crf),
                "-pix_fmt",
                "yuv420p",
                str(out_path),
            ],
            stdin=subprocess.PIPE,
        )
        assert encoder.stdin is not None

        trail_steps = max(1, int(round(args.reset_s / run.dt)))
        drawn = 0
        with closing(svo2.iter_left_jpegs(cam.svo_path, topic, fps)) as jpegs:
            for svo_index, jpeg in enumerate(jpegs):
                if svo_index < first_idx:
                    continue
                if svo_index > last_idx:
                    break
                tick = index_to_tick.get(svo_index)
                if tick is None:
                    continue  # frame the replay never delivered; keep timing by skipping
                img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                if img is None or img.shape != (cam.height, cam.width, 3):
                    # The stream tail can yield one truncated JPEG; skip it
                    # rather than hand ffmpeg a short packet.
                    continue
                k = int(slots[tick])
                cam_from_field = cam.cam_from_field[tick]
                have_tf = bool(np.isfinite(cam_from_field).all())

                def prj(pts):
                    return project_points(
                        cam_from_field, cam.intrinsics, np.asarray(pts, dtype=float)
                    )

                err_text = "pred  off (manual)"
                if have_tf:
                    meas_ok = bool(np.isfinite(run.x[k]))
                    lo = max(0, k - trail_steps)
                    trail = np.stack([run.x[lo : k + 1], run.y[lo : k + 1]], axis=1)
                    trail = trail[np.isfinite(trail).all(axis=1)]
                    if len(trail) >= 2:
                        _poly(img, prj(trail), BLUE, 2, closed=False)
                    if meas_ok:
                        corners = _robot_footprint(run.x[k], run.y[k], run.theta[k])
                        _poly(img, prj(corners), BLUE, 3)
                        head = np.array(
                            [
                                [run.x[k], run.y[k]],
                                [
                                    run.x[k] + ROBOT_HALF_M * np.cos(run.theta[k]),
                                    run.y[k] + ROBOT_HALF_M * np.sin(run.theta[k]),
                                ],
                            ]
                        )
                        _poly(img, prj(head), BLUE, 3, closed=False)

                    seg = predicted.segment[k]
                    if seg >= 0:
                        px, py, pth = predicted.pose[k]
                        _poly(img, prj(_robot_footprint(px, py, pth)), ORANGE, 3)
                        head = np.array(
                            [
                                [px, py],
                                [
                                    px + ROBOT_HALF_M * np.cos(pth),
                                    py + ROBOT_HALF_M * np.sin(pth),
                                ],
                            ]
                        )
                        _poly(img, prj(head), ORANGE, 3, closed=False)
                        sel = np.flatnonzero(predicted.segment[: k + 1] == seg)
                        _poly(img, prj(predicted.pose[sel, :2]), ORANGE, 2, closed=False)
                        since_reset = (k - sel[0]) * run.dt
                        if meas_ok:
                            _poly(
                                img,
                                prj(np.array([[px, py], [run.x[k], run.y[k]]])),
                                RED,
                                1,
                                closed=False,
                            )
                            err_mm = float(np.hypot(px - run.x[k], py - run.y[k])) * 1e3
                            err_deg = float(np.degrees(abs(wrap_angle(pth - run.theta[k]))))
                            err_text = (
                                f"pred +{since_reset:4.1f}s"
                                f"  err {err_mm:5.0f} mm {err_deg:5.1f} deg"
                            )
                        else:
                            err_text = f"pred +{since_reset:4.1f}s  (no measurement)"

                flags = []
                if not run.auto[k]:
                    flags.append("MANUAL")
                if not np.isfinite(run.x[k]):
                    flags.append("dropout")
                if run.contact[k]:
                    flags.append("contact-gated")
                if holdout_from_s is not None:
                    flags.append("holdout" if run.t[k] >= holdout_from_s else "train")

                hud = [
                    f"t = {run.t[k]:6.1f} s  {' '.join(flags)}",
                    err_text,
                    "blue = measured   orange = plant prediction"
                    f" (reset every {args.reset_s:.0f} s)",
                ]
                hud_scale = cam.width / 1920.0
                for i, line in enumerate(hud):
                    origin = (int(18 * hud_scale), int((34 + 30 * i) * hud_scale))
                    cv2.putText(
                        img,
                        line,
                        origin,
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.75 * hud_scale,
                        (0, 0, 0),
                        max(2, int(4 * hud_scale)),
                        cv2.LINE_AA,
                    )
                    cv2.putText(
                        img,
                        line,
                        origin,
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.75 * hud_scale,
                        WHITE,
                        1,
                        cv2.LINE_AA,
                    )

                # Stick bars, bottom right.
                bx = int(1560 * hud_scale)
                by = int(1000 * hud_scale)
                bw = int(300 * hud_scale)
                bh = int(24 * hud_scale)
                for row, (value, color, label) in enumerate(
                    (
                        (float(run.cmd_lin[k]), BLUE, "lin"),
                        (float(run.cmd_ang[k]), (60, 180, 60), "ang"),
                    )
                ):
                    y0 = by + row * (bh + int(8 * hud_scale))
                    cv2.rectangle(img, (bx, y0), (bx + bw, y0 + bh), (30, 30, 30), -1)
                    mid = bx + bw // 2
                    cv2.line(img, (mid, y0), (mid, y0 + bh), (120, 120, 120), 1)
                    extent = int(np.clip(value, -1, 1) * (bw // 2))
                    cv2.rectangle(img, (mid, y0 + 3), (mid + extent, y0 + bh - 3), color, -1)
                    cv2.putText(
                        img,
                        label,
                        (bx - int(52 * hud_scale), y0 + bh - int(6 * hud_scale)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6 * hud_scale,
                        WHITE,
                        1,
                        cv2.LINE_AA,
                    )

                encoder.stdin.write(img.tobytes())
                drawn += 1

        encoder.stdin.close()
        if encoder.wait() != 0:
            raise SystemExit(f"ffmpeg failed for {out_path}")
        print(f"wrote {out_path} ({drawn} frames at {fps:.1f} fps)")


if __name__ == "__main__":
    main()
