"""Render the opponent filter coasting through dropouts, drawn over the camera video.

One video per scene, each with a 0.2 s, a 1 s and a 3 s dropout:

- `apartment`: Mr Stabs Mk2 sysid session `2026-10-15_00-29-00`, with AprilTag truth. Frames come
  from the recording's `/camera/video`; the camera pose from its `/tf`.
- `nhrl`: NHRL May 17-26-14 KALMAN replay. Frames from the SVO, matched through
  `/camera/frame_meta`.
- `massd`: MassD 13-20-08 KALMAN replay, same way.

For each dropout, every measurement inside the gap is withheld and the measurement stream runs
through the filter mirror and through HOLD, rendered at each frame's own time with the 63 ms
perception latency (no render lead, so the overlay lines up with the picture). Drawn in field
coordinates and projected onto the floor:

- truth (apartment only): green footprint and heading
- measurements: white dot, hollow grey while withheld
- HOLD: orange footprint
- the 6-state filter: blue footprint, heading arrow and 2-sigma position ring

Videos play at half speed. Output: `playground/opponent_prediction/out/videos/<scene>.mp4`.

Usage:
    source scripts/activate_python.sh
    python playground/opponent_prediction/render_dropout_videos.py            # all scenes
    python playground/opponent_prediction/render_dropout_videos.py --scene nhrl
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
from collections.abc import Iterator
from contextlib import closing
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

from auto_battlebot.perception.sysid_sessions import BUNDLES, OUT, REPO_ROOT, load_session
from auto_battlebot.perception.tag_pose_smoother import up_from_field
from auto_battlebot.recording import svo2
from auto_battlebot.recording.mcap_io import (
    decode_json,
    decode_string,
    iter_messages,
    iter_video_frames,
)
from auto_battlebot.recording.sysid_io import load_field_from_camera, load_robot_tags
from playground.calibration.render_match_prediction_camera import load_camera_track
from playground.opponent_prediction import evaluate_opponent_prediction as ev
from playground.opponent_prediction.plot_dropout_samples import pick_samples, replay_session

GAPS_S = (0.2, 1.0, 3.0)
PRE_S = 1.5
POST_S = 1.5
SPEED = 0.5
TITLE_S = 1.2
OUT_WIDTH = 1280
INSET_CROP = 280
INSET_SCALE = 2.0
RING_CAP_M = 1.5

# BGR
BLUE = (214, 120, 42)
ORANGE = (52, 104, 235)
GREEN = (80, 190, 60)
WHITE = (245, 245, 245)
GREY = (160, 160, 160)
BLACK = (20, 20, 20)
REPLAYS = OUT / "replays"


@dataclass
class Scene:
    name: str
    title: str
    kind: str  # "sysid" or "replay"
    session: str = ""
    mcap: Path = field(default_factory=Path)
    svo: Path = field(default_factory=Path)
    half_size_m: float = 0.08
    # Height of the plane the measurements live on: the estimator projects opponent keypoints
    # onto keypoint_height_meters (0.03); the sysid fixes are axle centres on the floor.
    plane_z_m: float = 0.0


SCENES = {
    "apartment": Scene(
        "apartment",
        "Apartment floor, Mr Stabs Mk2 sysid (AprilTag truth)",
        "sysid",
        session="2026-10-15_00-29-00",
        half_size_m=0.075,
    ),
    "nhrl": Scene(
        "nhrl",
        "NHRL May 2026, cage, 17-26-14 (no truth; withheld measurements shown)",
        "replay",
        mcap=REPLAYS / "2026-05-02T17-26-14_KALMAN.mcap",
        svo=REPO_ROOT / "data/svo/NHRL_2026-05-02/2026-05-02T17-26-14.svo2",
        half_size_m=0.11,
        plane_z_m=0.03,
    ),
    "massd": Scene(
        "massd",
        "MassD Aug 2026, arena, 13-20-08 (no truth; withheld measurements shown)",
        "replay",
        mcap=REPLAYS / "2026-08-29T13-20-08_KALMAN.mcap",
        svo=REPO_ROOT / "data/svo/MassD_2026-08-29/2026-08-29T13-20-08.svo2",
        half_size_m=0.11,
        plane_z_m=0.03,
    ),
}


# ------------------------------------------------------------------------------- frames


@dataclass
class FrameSource:
    """Frames on the session clock with the camera_from_field transform for each."""

    frames: Iterator[tuple[int, np.ndarray, np.ndarray]]  # (session-clock ns, BGR, cam_from_field)
    intrinsics: np.ndarray
    fps: float


def sysid_frames(session: str) -> FrameSource:
    mcap = sorted((REPO_ROOT / "data/recordings").glob(f"*sysid*{session}.mcap"))[0]
    cal = json.loads((BUNDLES / f"{session}_yolo/session.json").read_text())["yolo"]["calibration"]
    offset_ns = int(round(cal["offset_ms"] * 1e6))
    k = load_robot_tags(mcap)[0].camera.matrix
    field_from_camera = load_field_from_camera(mcap)
    t_up_field, _ = up_from_field(field_from_camera)

    def gen() -> Iterator[tuple[int, np.ndarray, np.ndarray]]:
        for stamp_ns, image in iter_video_frames(mcap):
            if stamp_ns == 0:
                continue
            up_from_camera = t_up_field @ field_from_camera.at(np.array([stamp_ns]))[0]
            yield stamp_ns + offset_ns, image, np.linalg.inv(up_from_camera)

    return FrameSource(gen(), k, 60.0)


def replay_clock_offset_ns(mcap: Path) -> int:
    """Rebased pipeline stamp minus the SVO image stamp.

    `/keypoint_detections` is logged at its own (rebased) stamp and each tick's
    `/camera/frame_meta` follows it by the camera grab time, so a detection pairs with the next
    frame_meta.
    """
    fm_log, fm_stamp = [], []
    for _t, ns, data in iter_messages(mcap, ["/camera/frame_meta"]):
        fm_log.append(ns)
        fm_stamp.append(int(json.loads(decode_string(data))["image_stamp_ns"]))
    kd_log, kd_stamp = [], []
    for _t, ns, data in iter_messages(mcap, ["/keypoint_detections"]):
        kd_log.append(ns)
        kd_stamp.append(int(round(decode_json(data)["stamp"] * 1e9)))
    fl = np.array(fm_log)
    j = np.clip(np.searchsorted(fl, np.array(kd_log)), 0, len(fl) - 1)
    return int(np.median(np.array(kd_stamp) - np.array(fm_stamp)[j]))


def replay_frames(scene: Scene, last_index: int) -> FrameSource:
    cam = load_camera_track(scene.mcap)
    offset_ns = replay_clock_offset_ns(scene.mcap)
    index_to_row = {int(i): r for r, i in enumerate(cam.svo_index)}
    topic, _count = svo2.find_side_by_side_topic(scene.svo)
    fps = svo2.sample_fps(scene.svo, topic)

    def gen() -> Iterator[tuple[int, np.ndarray, np.ndarray]]:
        with closing(svo2.iter_left_jpegs(scene.svo, topic, fps)) as jpegs:
            for svo_index, jpeg in enumerate(jpegs):
                if svo_index > last_index:
                    break
                row = index_to_row.get(svo_index)
                if row is None or not np.isfinite(cam.cam_from_field[row]).all():
                    continue
                img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    continue
                yield int(cam.stamp_ns[row]) + offset_ns, img, cam.cam_from_field[row]

    return FrameSource(gen(), cam.intrinsics, fps)


def replay_last_index(scene: Scene, end_ns: int) -> int:
    cam = load_camera_track(scene.mcap)
    offset_ns = replay_clock_offset_ns(scene.mcap)
    after = np.nonzero(cam.stamp_ns + offset_ns > end_ns)[0]
    return int(cam.svo_index[after[0]]) if after.size else int(cam.svo_index[-1])


# ------------------------------------------------------------------------------- drawing


def project(cam_from_field: np.ndarray, k: np.ndarray, pts: np.ndarray) -> np.ndarray:
    pts = np.asarray(pts, dtype=float)
    if pts.shape[1] == 2:
        pts = np.hstack([pts, np.zeros((len(pts), 1))])
    cam = pts @ cam_from_field[:3, :3].T + cam_from_field[:3, 3]
    out = np.full((len(pts), 2), np.nan)
    ok = cam[:, 2] > 0.05
    out[ok, 0] = k[0, 0] * cam[ok, 0] / cam[ok, 2] + k[0, 2]
    out[ok, 1] = k[1, 1] * cam[ok, 1] / cam[ok, 2] + k[1, 2]
    return out


def poly(img, uv, color, thickness=2, closed=True) -> None:
    keep = np.isfinite(uv).all(axis=1)
    if keep.sum() < 2:
        return
    pts = np.clip(uv[keep], -1e5, 1e5).astype(np.int32)
    cv2.polylines(img, [pts], closed, color, thickness, lineType=cv2.LINE_AA)


def footprint(x, y, yaw, half) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    sq = half * np.array([[1, 1], [-1, 1], [-1, -1], [1, -1]], dtype=float)
    return sq @ np.array([[c, -s], [s, c]]).T + (x, y)


def draw_robot(img, cff, k, x, y, yaw, half, color, thickness=2, arrow=True) -> None:
    if not (np.isfinite(x) and np.isfinite(y)):
        return
    poly(img, project(cff, k, footprint(x, y, yaw, half)), color, thickness)
    if arrow and np.isfinite(yaw):
        tip = (x + 2.2 * half * math.cos(yaw), y + 2.2 * half * math.sin(yaw))
        uv = project(cff, k, np.array([[x, y], tip]))
        if np.isfinite(uv).all():
            a, b = uv.astype(int)
            cv2.arrowedLine(img, tuple(a), tuple(b), color, thickness, cv2.LINE_AA, tipLength=0.3)


def draw_ring(img, cff, k, x, y, radius, color) -> None:
    if not (np.isfinite(radius) and radius > 0):
        return
    a = np.linspace(0, 2 * math.pi, 64)
    r = min(radius, RING_CAP_M)
    poly(img, project(cff, k, np.stack([x + r * np.cos(a), y + r * np.sin(a)], 1)), color, 1)


def label(img, text, org, scale=0.7, color=WHITE, bg=BLACK, thickness=2) -> None:
    (w, h), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
    x, y = org
    cv2.rectangle(img, (x - 6, y - h - 8), (x + w + 6, y + base + 4), bg, -1)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)


def legend(img, scale, has_truth) -> None:
    rows = [("6-state filter (2-sigma ring)", BLUE), ("HOLD (production before)", ORANGE)]
    if has_truth:
        rows.insert(0, ("AprilTag truth", GREEN))
    rows.append(("measurement / withheld", WHITE))
    y = img.shape[0] - 20 - int(34 * scale) * (len(rows) - 1)
    for text, color in rows:
        cv2.rectangle(img, (14, y - int(18 * scale)), (14 + int(22 * scale), y), color, -1)
        label(img, text, (14 + int(32 * scale), y), 0.6 * scale)
        y += int(34 * scale)


# ------------------------------------------------------------------------------- segments


@dataclass
class Segment:
    gap_s: float
    start_ns: int
    window: tuple[int, int]
    phase1: ev.Trace
    hold: ev.Trace


def pick_segments(data) -> list[Segment]:
    span = (data.meas_stamp_ns[0], data.meas_stamp_ns[-1])
    thirds = np.linspace(span[0], span[1], 4)
    cfg, sigma = ev.tuned_config()
    cfg.render_lead_s = 0.0
    segments: list[Segment] = []
    used: list[tuple[int, int]] = []
    for i, gap in enumerate(GAPS_S):
        candidates = pick_samples(data, gap, 30)
        in_third = [c for c in candidates if thirds[i] <= c < thirds[i + 1]] or candidates
        start = None
        for c in in_third[len(in_third) // 2 :] + in_third[: len(in_third) // 2]:
            w = (c - int(PRE_S * 1e9), c + int((gap + POST_S) * 1e9))
            if all(w[1] < u[0] or w[0] > u[1] for u in used):
                start = c
                break
        if start is None:
            raise ValueError(f"no free window for a {gap} s dropout in {data.name}")
        window = (start - int((PRE_S + 3.0) * 1e9), start + int((gap + POST_S) * 1e9))
        used.append((start - int(PRE_S * 1e9), window[1]))
        keep = ~((data.meas_stamp_ns >= start) & (data.meas_stamp_ns < start + int(gap * 1e9)))
        keep = keep[None, :]
        phase1 = ev.simulate_filter(data, ev.OpponentFilter(1, cfg), keep, sigma, window_ns=window)
        hold = ev.simulate_hold(data, keep, 0.0, window_ns=window)
        segments.append(Segment(gap, start, window, phase1, hold))
    return sorted(segments, key=lambda s: s.start_ns)


def state_at(trace: ev.Trace, t_ns: int) -> dict | None:
    k = int(np.searchsorted(trace.target_ns, t_ns, side="right")) - 1
    if k < 0 or trace.values["active"][k, 0] <= 0:
        return None
    return {name: float(v[k, 0]) for name, v in trace.values.items()}


def render_frame(img, t_ns, cff, k, data, seg: Segment, scene: Scene, scale: float) -> np.ndarray:
    # Lift the field points onto the measurement plane by shifting the camera down by its height.
    lift = np.eye(4)
    lift[2, 3] = scene.plane_z_m
    cff = cff @ lift
    out = img.copy()
    has_truth = scene.kind == "sysid"
    half = scene.half_size_m
    gap_end = seg.start_ns + int(seg.gap_s * 1e9)
    in_gap = seg.start_ns <= t_ns < gap_end

    if has_truth:
        tr = data.truth_at(np.array([t_ns]))
        if tr["valid"][0]:
            draw_robot(out, cff, k, tr["x"][0], tr["y"][0], tr["yaw"][0], half, GREEN, 2)

    # The newest measurement taken by this frame time, from the last 0.1 s.
    ms = data.meas_stamp_ns
    j = int(np.searchsorted(ms, t_ns, side="right")) - 1
    if j >= 0 and t_ns - ms[j] < 1e8:
        uv = project(cff, k, np.array([[data.meas_x[j], data.meas_y[j]]]))[0]
        if np.isfinite(uv).all():
            withheld = seg.start_ns <= ms[j] < gap_end
            if withheld:
                cv2.circle(out, tuple(uv.astype(int)), int(7 * scale), GREY, 2, cv2.LINE_AA)
            else:
                cv2.circle(out, tuple(uv.astype(int)), int(5 * scale), WHITE, -1, cv2.LINE_AA)

    hold = state_at(seg.hold, t_ns)
    if hold is not None:
        draw_robot(out, cff, k, hold["x"], hold["y"], hold["yaw"], half, ORANGE, 2)
    est = state_at(seg.phase1, t_ns)
    if est is not None:
        draw_ring(out, cff, k, est["x"], est["y"], 2.0 * est["position_sigma_m"], BLUE)
        draw_robot(out, cff, k, est["x"], est["y"], est["yaw"], half, BLUE, 3)

    # Zoomed inset around the filter estimate.
    if est is not None:
        c = project(cff, k, np.array([[est["x"], est["y"]]]))[0]
        if np.isfinite(c).all():
            h, w = out.shape[:2]
            cx = int(np.clip(c[0], INSET_CROP // 2, w - INSET_CROP // 2))
            cy = int(np.clip(c[1], INSET_CROP // 2, h - INSET_CROP // 2))
            crop = out[
                cy - INSET_CROP // 2 : cy + INSET_CROP // 2,
                cx - INSET_CROP // 2 : cx + INSET_CROP // 2,
            ]
            size = int(INSET_CROP * INSET_SCALE)
            inset = cv2.resize(crop, (size, size), interpolation=cv2.INTER_LINEAR)
            # Opposite side from the robot, so the inset never covers the real one.
            x0 = 12 if c[0] > w / 2 else w - size - 12
            y0 = int(130 * scale)
            if y0 + size < h:
                out[y0 : y0 + size, x0 : x0 + size] = inset
                cv2.rectangle(out, (x0, y0), (x0 + size, y0 + size), WHITE, 2)

    # Banner and status.
    label(out, f"{scene.title}   [{SPEED:g}x speed]", (14, int(32 * scale)), 0.7 * scale)
    if in_gap:
        status = (
            f"DROPOUT {seg.gap_s:g} s: measurements withheld   +{(t_ns - seg.start_ns) / 1e9:.2f} s"
        )
        color = (60, 60, 230)
    elif t_ns >= gap_end:
        status = f"measurements back after the {seg.gap_s:g} s dropout"
        color = GREEN
    else:
        status = f"tracking; {seg.gap_s:g} s dropout in {(seg.start_ns - t_ns) / 1e9:.2f} s"
        color = WHITE
    label(out, status, (14, int(70 * scale)), 0.75 * scale, color)
    if est is not None:
        ref = None
        if has_truth:
            tr = data.truth_at(np.array([t_ns]))
            if tr["valid"][0]:
                ref = (tr["x"][0], tr["y"][0], "truth")
        elif j >= 0 and t_ns - ms[j] < 5e7:
            ref = (data.meas_x[j], data.meas_y[j], "measurement")
        sig = est["position_sigma_m"]
        line = f"filter 2-sigma {200 * sig:.0f} cm" if np.isfinite(sig) else "filter 2-sigma -"
        if ref is not None:
            e_f = 1e3 * math.hypot(est["x"] - ref[0], est["y"] - ref[1])
            line = f"error vs {ref[2]}: filter {e_f:.0f} mm"
            if hold is not None:
                e_h = 1e3 * math.hypot(hold["x"] - ref[0], hold["y"] - ref[1])
                line += f", HOLD {e_h:.0f} mm"
            if np.isfinite(sig):
                line += f"   filter 2-sigma {200 * sig:.0f} cm"
        label(out, line, (14, int(108 * scale)), 0.65 * scale)
    legend(out, scale, has_truth)
    return out


def title_card(shape, scene: Scene, gap: float) -> np.ndarray:
    img = np.full(shape, 24, np.uint8)
    h, w = shape[:2]
    for i, (text, size) in enumerate(
        [(scene.title, 0.9), (f"{gap:g} s dropout", 1.8), ("measurements withheld", 0.9)]
    ):
        (tw, _th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, size, 2)
        y = int(h * (0.38 + 0.12 * i))
        cv2.putText(
            img, text, ((w - tw) // 2, y), cv2.FONT_HERSHEY_SIMPLEX, size, WHITE, 2, cv2.LINE_AA
        )
    return img


# ------------------------------------------------------------------------------- main


def render_scene(scene: Scene) -> Path:
    if scene.kind == "sysid":
        data = load_session(scene.session)
    else:
        data = replay_session(scene.mcap)
    segments = pick_segments(data)
    last_ns = max(s.window[1] for s in segments)
    if scene.kind == "sysid":
        source = sysid_frames(scene.session)
    else:
        source = replay_frames(scene, replay_last_index(scene, last_ns))

    out_path = OUT / "videos" / f"{scene.name}.mp4"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    encoder = None
    shape = None
    current = -1
    written = 0
    show_from = [s.start_ns - int(PRE_S * 1e9) for s in segments]
    for t_ns, img, cff in source.frames:
        if t_ns > last_ns:
            break
        idx = next((i for i, s in enumerate(segments) if show_from[i] <= t_ns <= s.window[1]), None)
        if idx is None:
            continue
        scale_px = OUT_WIDTH / img.shape[1]
        out_h = int(round(img.shape[0] * scale_px / 2)) * 2
        if encoder is None:
            shape = (out_h, OUT_WIDTH, 3)
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
                    f"{OUT_WIDTH}x{out_h}",
                    "-r",
                    f"{source.fps * SPEED:.3f}",
                    "-i",
                    "pipe:0",
                    "-c:v",
                    "libx264",
                    "-preset",
                    "veryfast",
                    "-crf",
                    "20",
                    "-pix_fmt",
                    "yuv420p",
                    str(out_path),
                ],
                stdin=subprocess.PIPE,
            )
        assert encoder.stdin is not None and shape is not None
        if idx != current:
            card = title_card(shape, scene, segments[idx].gap_s)
            for _ in range(int(TITLE_S * source.fps * SPEED)):
                encoder.stdin.write(card.tobytes())
            current = idx
        frame = render_frame(
            img, t_ns, cff, source.intrinsics, data, segments[idx], scene, 1.0 / scale_px
        )
        frame = cv2.resize(frame, (OUT_WIDTH, out_h), interpolation=cv2.INTER_AREA)
        encoder.stdin.write(frame.tobytes())
        written += 1
    if encoder is None:
        raise RuntimeError(f"{scene.name}: no frames fell inside the chosen segments")
    assert encoder.stdin is not None
    encoder.stdin.close()
    encoder.wait()
    starts = ", ".join(
        f"{s.gap_s:g} s at {(s.start_ns - data.meas_stamp_ns[0]) / 1e9:.1f} s" for s in segments
    )
    print(f"{out_path}: {written} frames; dropouts {starts}")
    return out_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--scene", choices=sorted(SCENES), action="append")
    args = parser.parse_args()
    for name in args.scene or list(SCENES):
        render_scene(SCENES[name])


if __name__ == "__main__":
    main()
