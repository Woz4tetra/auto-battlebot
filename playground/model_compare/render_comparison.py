"""Render the old-vs-new opponent overlay onto the original SVO.

Old side: the opponent track recorded live on 2026-05-02, which ran
yolo26n-seg_nhrl_robots_2026-04-27. New side: the same SVO replayed through
yolo26n_nhrl_robots_bbox_2class_mixed_2026-07-31. Only the opponent model differs in what
this draws; our-robot keypoints run the same pose model in both and are drawn once from the
replay, whose per-tick SVO frame index keeps them in step with the image.

Each side is projected through the field<-camera transform and intrinsics of the run that
produced it, so the on-screen gap reflects the 2D detections rather than the field fit,
which also moved between the two runs.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from track_overlay import (  # noqa: E402
    footprint_circle,
    project_field_points,
    read_camera_matrix,
    read_frame_meta,
    read_opponent_track,
    read_transforms,
    _iter,
)

OLD_COLOR = (60, 200, 255)  # amber, BGR
NEW_COLOR = (90, 230, 90)  # green
KEYPOINT_COLOR = (220, 220, 220)
FIELD_COLOR = (70, 70, 70)
FIELD_HALF_M = 2.44 / 2.0
FONT = cv2.FONT_HERSHEY_SIMPLEX


def draw_track(canvas, sample, K, color, label, dashed=False, label_above=True):
    """Draw one run's opponent footprint. Returns the centre pixel, or None."""
    ring = footprint_circle(sample.position, sample.radius)
    pixels, ok = project_field_points(ring, sample.tf_field_from_camera, K)
    if not ok.all():
        return None
    poly = pixels.astype(np.int32)
    if dashed:
        for i in range(0, len(poly), 2):
            cv2.line(canvas, tuple(poly[i]), tuple(poly[(i + 1) % len(poly)]), color, 3, cv2.LINE_AA)
    else:
        cv2.polylines(canvas, [poly], True, color, 3, cv2.LINE_AA)

    centre, centre_ok = project_field_points(sample.position[None, :], sample.tf_field_from_camera, K)
    if not centre_ok[0]:
        return None
    cx, cy = int(centre[0][0]), int(centre[0][1])
    cv2.drawMarker(canvas, (cx, cy), color, cv2.MARKER_CROSS, 18, 2, cv2.LINE_AA)
    # The two footprints sit almost on top of each other, so park one label above the
    # ring and the other below, otherwise they overprint and neither is readable.
    y = int(poly[:, 1].min()) - 10 if label_above else int(poly[:, 1].max()) + 22
    cv2.putText(canvas, label, (cx - 22, y), FONT, 0.6, color, 2, cv2.LINE_AA)
    return np.array([centre[0][0], centre[0][1]])


def read_keypoints(path: str, to_raw_stamp=None):
    """Our-robot keypoint spheres, keyed on the raw SVO clock.

    Taken from the replay rather than the live run: the replay stamps every tick with the
    SVO frame it came from, so the dots land on the frame that produced them. The live
    run's stamps are wall-clock, and matching those to SVO frames within a tolerance drags
    the keypoints a fraction of a second behind the image.
    """
    statics, dynamics = read_transforms(path)
    stamps, values = [], []
    for _, _, msg, dec in _iter(path, ["/robot_markers"]):
        pts = [
            [mk.pose.position.x, mk.pose.position.y, mk.pose.position.z]
            for mk in dec.markers
            if mk.ns == "robot_keypoints"
        ]
        if not pts:
            continue
        static = statics.latest_at(msg.log_time)
        dynamic = dynamics.nearest(msg.log_time, tolerance_ns=100_000_000)
        if static is None or dynamic is None:
            continue
        raw = msg.log_time if to_raw_stamp is None else to_raw_stamp.nearest(msg.log_time)
        if raw is None:
            continue
        stamps.append(int(raw))
        values.append((np.array(pts), static @ dynamic))
    from track_overlay import TimeSeries

    return TimeSeries(stamps, values)


def legend(canvas, old_seen, new_seen, gap_px, frame_index, seconds):
    h, w = canvas.shape[:2]
    panel = canvas[0:132, 0:560].copy()
    canvas[0:132, 0:560] = cv2.addWeighted(panel, 0.25, np.zeros_like(panel), 0.75, 0)
    cv2.putText(canvas, "opponent model", (16, 30), FONT, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

    old_text = "old  seg 2026-04-27 (as recorded)" + ("" if old_seen else "   [no track]")
    new_text = "new  bbox-2class-mixed 2026-07-31" + ("" if new_seen else "   [no track]")
    cv2.line(canvas, (18, 55), (54, 55), OLD_COLOR, 3, cv2.LINE_AA)
    cv2.putText(canvas, old_text, (64, 61), FONT, 0.55, OLD_COLOR, 2, cv2.LINE_AA)
    cv2.line(canvas, (18, 83), (54, 83), NEW_COLOR, 3, cv2.LINE_AA)
    cv2.putText(canvas, new_text, (64, 89), FONT, 0.55, NEW_COLOR, 2, cv2.LINE_AA)

    gap = f"gap {gap_px:5.1f} px" if gap_px is not None else "gap      -"
    cv2.putText(canvas, gap, (18, 118), FONT, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    stamp = f"svo frame {frame_index}   t+{seconds:6.2f}s"
    cv2.putText(canvas, stamp, (200, 118), FONT, 0.6, (200, 200, 200), 2, cv2.LINE_AA)
    cv2.putText(
        canvas,
        "keypoints unchanged (pose 2026-05-01, from replay)",
        (16, h - 18),
        FONT,
        0.5,
        KEYPOINT_COLOR,
        1,
        cv2.LINE_AA,
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--old-mcap", required=True)
    ap.add_argument("--new-mcap", required=True)
    ap.add_argument("--svo", required=True)
    ap.add_argument("--start-ns", type=int, required=True)
    ap.add_argument("--end-ns", type=int, required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--tolerance-ms", type=float, default=50.0)
    ap.add_argument("--draw-field", action="store_true", help="outline the nominal 8 ft arena")
    args = ap.parse_args()

    tolerance_ns = int(args.tolerance_ms * 1e6)

    print("reading tracks...")
    log_to_raw, stamp_to_index = read_frame_meta(args.new_mcap)
    old_track = read_opponent_track(args.old_mcap)
    new_track = read_opponent_track(args.new_mcap, to_raw_stamp=log_to_raw)
    keypoints = read_keypoints(args.new_mcap, to_raw_stamp=log_to_raw)
    K_old = read_camera_matrix(args.old_mcap)
    K_new = read_camera_matrix(args.new_mcap)
    print(f"  old samples {len(old_track.stamps)}, new samples {len(new_track.stamps)}")

    first = stamp_to_index.nearest(args.start_ns)
    last = stamp_to_index.nearest(args.end_ns)
    print(f"  svo frames {first}..{last}")

    import pyzed.sl as sl

    cam = sl.Camera()
    init = sl.InitParameters()
    init.set_from_svo_file(args.svo)
    init.svo_real_time_mode = False
    if cam.open(init) != sl.ERROR_CODE.SUCCESS:
        print("failed to open SVO")
        return 1
    cam.set_svo_position(int(first))
    mat = sl.Mat()

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    encoder = subprocess.Popen(
        [
            "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
            "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", "1280x720",
            "-r", str(args.fps), "-i", "-",
            "-c:v", "libx264", "-preset", "medium", "-crf", "20", "-pix_fmt", "yuv420p",
            str(out_path),
        ],
        stdin=subprocess.PIPE,
    )

    square = np.array(
        [
            [-FIELD_HALF_M, -FIELD_HALF_M, 0.0],
            [FIELD_HALF_M, -FIELD_HALF_M, 0.0],
            [FIELD_HALF_M, FIELD_HALF_M, 0.0],
            [-FIELD_HALF_M, FIELD_HALF_M, 0.0],
        ]
    )

    written = 0
    both = 0
    gaps: list[float] = []
    old_only = new_only = neither = 0
    while True:
        if cam.grab() != sl.ERROR_CODE.SUCCESS:
            break
        index = cam.get_svo_position() - 1
        if index > int(last):
            break
        stamp_ns = int(cam.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds())
        cam.retrieve_image(mat, sl.VIEW.LEFT)
        canvas = cv2.cvtColor(mat.get_data(), cv2.COLOR_BGRA2BGR)

        old_sample = old_track.nearest(stamp_ns, tolerance_ns=tolerance_ns)
        new_sample = new_track.nearest(stamp_ns, tolerance_ns=tolerance_ns)

        if args.draw_field and old_sample is not None:
            sq, ok = project_field_points(square, old_sample.tf_field_from_camera, K_old)
            if ok.all():
                cv2.polylines(canvas, [sq.astype(np.int32)], True, FIELD_COLOR, 1, cv2.LINE_AA)

        kp = keypoints.nearest(stamp_ns, tolerance_ns=tolerance_ns)
        if kp is not None:
            pts, tf = kp
            px, ok = project_field_points(pts, tf, K_new)
            for p, good in zip(px, ok):
                if good:
                    cv2.circle(canvas, (int(p[0]), int(p[1])), 4, KEYPOINT_COLOR, -1, cv2.LINE_AA)

        old_centre = new_centre = None
        if old_sample is not None:
            old_centre = draw_track(canvas, old_sample, K_old, OLD_COLOR, "old", dashed=True)
        if new_sample is not None:
            new_centre = draw_track(
                canvas, new_sample, K_new, NEW_COLOR, "new", label_above=False
            )

        gap = None
        if old_centre is not None and new_centre is not None:
            gap = float(np.linalg.norm(old_centre - new_centre))
            gaps.append(gap)
            both += 1
            cv2.line(
                canvas,
                tuple(old_centre.astype(int)),
                tuple(new_centre.astype(int)),
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        elif old_centre is not None:
            old_only += 1
        elif new_centre is not None:
            new_only += 1
        else:
            neither += 1

        legend(
            canvas,
            old_sample is not None,
            new_sample is not None,
            gap,
            index,
            (stamp_ns - args.start_ns) / 1e9,
        )
        encoder.stdin.write(canvas.tobytes())
        written += 1

    encoder.stdin.close()
    encoder.wait()
    cam.close()

    print(f"\nwrote {written} frames -> {out_path}")
    print(f"  both tracked   {both:4d}  ({100.0 * both / max(1, written):.1f}%)")
    print(f"  old only       {old_only:4d}")
    print(f"  new only       {new_only:4d}")
    print(f"  neither        {neither:4d}")
    if gaps:
        arr = np.array(gaps)
        print(
            f"  centre gap px: mean {arr.mean():.1f}  median {np.median(arr):.1f}  "
            f"p90 {np.percentile(arr, 90):.1f}  max {arr.max():.1f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
