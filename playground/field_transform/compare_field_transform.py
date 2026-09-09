"""Compare the depth+plane camera<->field transform against an RGB-only homography.

Reads a ROS1-profile MCAP (convert a .bag with `mcap convert in.bag out.mcap`), picks
frames where the camera is holding still on its tripod, and runs both methods on the same
DeepLab field mask so the only difference is how the pose is derived.

`depth_plane` is the shipped C++ path and measures the field size. `homography` uses the
RGB outline plus the field size you supply, and needs no depth at all.

Usage:
    python compare_field_transform.py RUN.mcap --field-size 2.4384 2.4384 -o out/
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from playground.bgsub_cage.field_hull import FieldSegmenter
from playground.field_transform.field_methods import (
    FieldResult,
    compare,
    depth_plane_field,
    homography_field,
)

RGB_HINTS = ("rgb/image_rect_color", "left/image_rect_color", "rgb/image_raw", "left/image_raw")
DEPTH_HINTS = ("depth/depth_registered", "depth/image_raw", "depth_registered")
INFO_HINTS = ("rgb/camera_info", "left/camera_info")
CLOUD_HINTS = ("point_cloud/cloud_registered", "cloud_registered")
SQUARE_TOLERANCE_M = 0.05


def decode_image(msg: Any) -> np.ndarray:
    """ROS1 sensor_msgs/Image to an ndarray, colour or depth."""
    height, width = msg.height, msg.width
    encoding = msg.encoding
    buffer = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if encoding in ("32FC1",):
        return buffer.view(np.float32).reshape(height, width)
    if encoding in ("16UC1", "mono16"):
        return buffer.view(np.uint16).reshape(height, width)
    if encoding in ("bgra8", "rgba8"):
        image = buffer.reshape(height, width, 4)
        return cv2.cvtColor(
            image, cv2.COLOR_BGRA2BGR if encoding == "bgra8" else cv2.COLOR_RGBA2BGR
        )
    if encoding in ("bgr8", "rgb8"):
        image = buffer.reshape(height, width, 3)
        return image if encoding == "bgr8" else cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == "mono8":
        return buffer.reshape(height, width)
    raise ValueError(f"unhandled image encoding {encoding!r}")


def pick_topic(available: list[str], hints: tuple[str, ...], override: str | None) -> str | None:
    if override:
        return override if override in available else None
    for hint in hints:
        for topic in available:
            if topic.endswith(hint):
                return topic
    return None


def decode_organized_cloud(msg: Any) -> tuple[np.ndarray, np.ndarray]:
    """An organized XYZ+BGRA PointCloud2 back into a BGR image and a metric depth image.

    The 2024 competition bags never recorded the ZED image or depth streams; the only
    per-pixel data is this cloud, published once when the field was initialised. It is in
    the optical frame (verified: unprojecting z through K reproduces the cloud's own x and
    y to 0.0000 m), so z is exactly the depth image the C++ path expects.
    """
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
        msg.height, msg.width, msg.point_step
    )
    xyz = raw[:, :, :12].copy().view(np.float32).reshape(msg.height, msg.width, 3)
    bgr = raw[:, :, 12:15].copy()
    depth = xyz[:, :, 2].astype(np.float32)
    return bgr, depth


def load_frames(path: Path, args: argparse.Namespace) -> tuple[list[dict], dict]:
    """Every RGB frame with its nearest depth frame, plus the intrinsics."""
    from mcap.reader import make_reader
    from mcap_ros1.decoder import DecoderFactory

    with path.open("rb") as handle:
        reader = make_reader(handle, decoder_factories=[DecoderFactory()])
        channels = {c.topic for c in reader.get_summary().channels.values()}
        topics = sorted(channels)
        rgb_topic = pick_topic(topics, RGB_HINTS, args.rgb_topic)
        depth_topic = pick_topic(topics, DEPTH_HINTS, args.depth_topic)
        info_topic = pick_topic(topics, INFO_HINTS, args.info_topic)
        cloud_topic = pick_topic(topics, CLOUD_HINTS, args.cloud_topic)
        if info_topic is None:
            print(f"  topics present: {topics}")
            raise SystemExit(f"no camera_info topic in {path.name}")

        # Separate image and depth streams when they were recorded; otherwise fall back to
        # an organized registered cloud, which carries both.
        use_cloud = (rgb_topic is None or depth_topic is None) and cloud_topic is not None
        if not use_cloud and (rgb_topic is None or depth_topic is None):
            print(f"  topics present: {topics}")
            raise SystemExit(f"no rgb+depth and no organized cloud in {path.name}")
        if use_cloud:
            print(f"  cloud={cloud_topic} (no image/depth streams recorded)\n  info={info_topic}")
            wanted = [cloud_topic, info_topic]
        else:
            print(f"  rgb={rgb_topic}\n  depth={depth_topic}\n  info={info_topic}")
            wanted = [rgb_topic, depth_topic, info_topic]

        intrinsics: np.ndarray | None = None
        rgb: list[tuple[int, np.ndarray]] = []
        depth: list[tuple[int, np.ndarray]] = []
        for _, channel, message, decoded in reader.iter_decoded_messages(topics=wanted):
            stamp = message.log_time
            if channel.topic == info_topic:
                if intrinsics is None:
                    intrinsics = np.array(decoded.K, dtype=np.float64).reshape(3, 3)
            elif use_cloud and channel.topic == cloud_topic:
                image, depth_image = decode_organized_cloud(decoded)
                rgb.append((stamp, image))
                depth.append((stamp, depth_image))
            elif channel.topic == rgb_topic:
                rgb.append((stamp, decode_image(decoded)))
            elif channel.topic == depth_topic:
                depth.append((stamp, decode_image(decoded)))

    if intrinsics is None or not rgb or not depth:
        raise SystemExit(
            f"{path.name}: rgb={len(rgb)} depth={len(depth)} info={intrinsics is not None}"
        )

    depth_stamps = np.array([s for s, _ in depth])
    frames = []
    for stamp, image in rgb:
        j = int(np.argmin(np.abs(depth_stamps - stamp)))
        # A pairing worse than half a frame at 30 Hz is a dropped message, not a match.
        if abs(depth_stamps[j] - stamp) > 16_000_000:
            continue
        frames.append({"stamp": stamp, "rgb": image, "depth": depth[j][1]})
    return frames, {"intrinsics": intrinsics, "rgb_topic": rgb_topic, "depth_topic": depth_topic}


def stillest_frames(frames: list[dict], count: int, stride: int) -> list[int]:
    """Indices of the frames where the tripod is most settled.

    Motion is measured by phase correlation against the previous sampled frame, the same
    way the cage-camera stability check works. The camera is meant to be static, so this
    is really a filter for someone bumping the tripod or walking through the shot.
    """
    sampled = list(range(0, len(frames), max(1, stride)))
    scores = []
    previous = None
    for index in sampled:
        grey = cv2.cvtColor(frames[index]["rgb"], cv2.COLOR_BGR2GRAY).astype(np.float32)
        grey = cv2.resize(grey, (grey.shape[1] // 2, grey.shape[0] // 2))
        if previous is not None:
            (dx, dy), _ = cv2.phaseCorrelate(previous, grey)
            scores.append((float(np.hypot(dx, dy)) * 2.0, index))
        previous = grey
    scores.sort()
    chosen = sorted(index for _, index in scores[:count])
    return chosen or sampled[:count]


def overlay(
    frame: dict,
    mask: np.ndarray,
    results: dict[str, FieldResult],
    intrinsics: np.ndarray,
    field_size: tuple[float, float],
) -> np.ndarray:
    """Both methods' field rectangle reprojected onto the RGB frame."""
    canvas = frame["rgb"].copy()
    canvas[mask > 0] = (0.75 * canvas[mask > 0] + 0.25 * np.array([0, 180, 0])).astype(np.uint8)
    colors = {"depth_plane": (90, 230, 90), "homography": (60, 200, 255)}
    w, h = field_size
    for offset, (name, result) in enumerate(results.items()):
        if not result.ok:
            continue
        size = result.size_xy_m or field_size
        half = np.array([size[0] / 2.0, size[1] / 2.0])
        corners = np.array(
            [
                [-half[0], -half[1], 0.0],
                [-half[0], half[1], 0.0],
                [half[0], half[1], 0.0],
                [half[0], -half[1], 0.0],
            ]
        )
        cam = (result.tf_camera_from_field[:3, :3] @ corners.T).T + result.tf_camera_from_field[
            :3, 3
        ]
        in_front = cam[:, 2] > 1e-6
        if not in_front.all():
            continue
        uv = (intrinsics @ cam.T).T
        uv = (uv[:, :2] / uv[:, 2:3]).astype(np.int32)
        cv2.polylines(canvas, [uv], True, colors[name], 2, cv2.LINE_AA)
        t = result.tf_camera_from_field[:3, 3]
        cv2.putText(
            canvas,
            f"{name}: range {np.linalg.norm(t):.3f} m  size {size[0]:.3f}x{size[1]:.3f}",
            (10, 28 + 26 * offset),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            colors[name],
            2,
            cv2.LINE_AA,
        )
    return canvas


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("mcap", type=Path, nargs="+", help="ROS1-profile MCAP file(s)")
    parser.add_argument("-o", "--output", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=10, help="Frames per file (default 10)")
    parser.add_argument("--stride", type=int, default=15, help="Sample every Nth frame")
    parser.add_argument(
        "--field-size",
        type=float,
        nargs=2,
        default=[2.4384, 2.4384],
        metavar=("X", "Y"),
        help="Known field size in metres (default 8 ft square)",
    )
    parser.add_argument(
        "--distance-threshold",
        type=float,
        default=0.05,
        help="RANSAC plane inlier distance, matching config/_common.toml",
    )
    parser.add_argument(
        "--depth-scale",
        type=float,
        default=1.0,
        help="Multiplier to convert the depth image to metres",
    )
    parser.add_argument("--rgb-topic", default=None)
    parser.add_argument("--depth-topic", default=None)
    parser.add_argument(
        "--cloud-topic",
        default=None,
        help="Organized XYZ+BGRA cloud, used when no image/depth streams were recorded",
    )
    parser.add_argument("--info-topic", default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    field_size = (args.field_size[0], args.field_size[1])
    square = abs(field_size[0] - field_size[1]) < SQUARE_TOLERANCE_M
    segmenter = FieldSegmenter()
    rows: list[dict] = []

    for path in args.mcap:
        print(f"\n=== {path.name} ===")
        frames, meta = load_frames(path, args)
        intrinsics = meta["intrinsics"]
        print(f"  {len(frames)} paired rgb+depth frames")
        chosen = stillest_frames(frames, args.frames, args.stride)
        print(f"  scoring frames: {chosen}")

        for order, index in enumerate(chosen):
            frame = frames[index]
            mask = segmenter.raw_mask(frame["rgb"])
            depth_m = frame["depth"].astype(np.float32) * args.depth_scale
            a = depth_plane_field(mask, depth_m, intrinsics, args.distance_threshold)
            b = homography_field(mask, intrinsics, field_size)
            row = {
                "file": path.name,
                "frame_index": index,
                "stamp_ns": frame["stamp"],
                "depth_plane": {
                    k: v
                    for k, v in asdict(a).items()
                    if k not in ("tf_camera_from_field", "corners_image", "plane_normal")
                },
                "homography": {
                    k: v
                    for k, v in asdict(b).items()
                    if k not in ("tf_camera_from_field", "corners_image", "plane_normal")
                },
                "comparison": compare(a, b, square),
            }
            if a.ok:
                row["depth_plane"]["tf"] = a.tf_camera_from_field.tolist()
            if b.ok:
                row["homography"]["tf"] = b.tf_camera_from_field.tolist()
            rows.append(row)

            cv2.imwrite(
                str(args.output / f"{path.stem}_f{index:06d}.jpg"),
                overlay(frame, mask, {"depth_plane": a, "homography": b}, intrinsics, field_size),
            )
            c = row["comparison"]
            if c["ok"]:
                print(
                    f"   [{order + 1:2d}] frame {index:6d}  range "
                    f"{c['range_a_m']:.3f} / {c['range_b_m']:.3f} m  "
                    f"dt {c['translation_diff_m'] * 100:5.1f} cm  "
                    f"normal {c['normal_angle_deg']:5.2f} deg  yaw {c['yaw_diff_deg']:5.2f} deg"
                    + ("  [CLIPPED]" if b.extra.get("clipped") else "")
                )
            else:
                print(f"   [{order + 1:2d}] frame {index:6d}  {c['notes']}")

    (args.output / "results.json").write_text(json.dumps(rows, indent=2) + "\n")
    good = [r["comparison"] for r in rows if r["comparison"]["ok"]]
    if good:
        dt = np.array([g["translation_diff_m"] for g in good])
        na = np.array([g["normal_angle_deg"] for g in good])
        yaw = np.array([g["yaw_diff_deg"] for g in good])
        print(f"\n{len(good)}/{len(rows)} frames solved by both methods")
        dt_med, dt_max = np.median(dt) * 100, dt.max() * 100
        print(f"  translation diff  median {dt_med:6.1f} cm  max {dt_max:6.1f} cm")
        print(f"  plane normal diff median {np.median(na):6.2f} deg  max {na.max():6.2f} deg")
        print(f"  yaw diff          median {np.median(yaw):6.2f} deg  max {yaw.max():6.2f} deg")
    print(f"\nOverlays and results.json in {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
