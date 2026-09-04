#!/usr/bin/env python3
"""Render a recording's depth as a bird's-eye video beside its RGB, over the tracked field.

Depth is not in the MCAP: the ZED SDK computes it on device and the pipeline never publishes
it. The stereo pair is in the SVO though, so replaying it through pyzed recomputes depth, and
offline the NEURAL_PLUS mode (the most accurate the SDK offers) is affordable on every frame.

The C++ point cloud debug draws its scatter panels only at field init, from a fresh plane fit.
This renders every processed frame and reuses the field the recording already tracked:

1. `/camera/frame_meta` carries `svo_frame_index` per tick, the exact join key back into the
   SVO. Never match by timestamp: SVO stamps sit about half a frame off the pipeline stamps.
   Recordings from before frame_meta existed (the May 2026 NHRL ones) fall back to the interval
   join `export_camera_transforms.py` validated: the pipeline stamp lands 0.4 to 0.9 of the way
   into the SVO frame interval it belongs to and never crosses the boundary, so a fast
   depth-off pass collects the SVO's own frame stamps and each `/tf` tick resolves to the frame
   whose interval contains it.
2. `/tf` carries `field -> camera_world` (changes only on re-init, restamped every cycle; May-era
   recordings wrote it once per init on `/tf_static` instead) and `camera_world -> camera` (tracked
   every cycle). Edges are associated to their tick by log order, the same association
   `auto_battlebot.calibration.match_windows` relies on. Composing the two gives
   `tf_field_from_camera` per tick.
3. `/field_markers` (ns `field`, first four points) gives the fitted field border. The corners
   are recorded in the camera_world frame of their init, so each tick maps them into the field
   frame with its own `field -> camera_world` edge. Field state is applied on the recording's
   own timeline: each tick uses the border marker in effect at that tick's log time, never a
   fit from later in the recording, so the overlay re-initializes exactly when the original
   run did.

The SVO is decoded strictly sequentially (frames outside the stride are grabbed with depth
off) because seeking lands on non-keyframes and smears both the RGB and the depth. Every SVO
frame between the first and last tracked tick is written, so the video plays back at the SVO's
own frame rate: frames the pipeline dropped reuse the most recent tick's field pose (the
camera is on a fixed mount) and carry a "held" tag in the overlay.

Each frame's XYZ cloud is retrieved in the camera IMAGE frame (the convention the C++ stack
opens the ZED with), transformed into the field frame, cropped to the raster extent, and
rasterized top-down: points further than the height clip above or below the field plane are
dropped (walls, glass, crowd), then per output pixel the highest point wins, colored by height
above the floor plane (field frame +z points from the floor toward the camera). Pixels whose
top point falls in the highlight band, robot-height by default, are painted solid magenta.
The left RGB image of the same SVO frame is placed beside the projection. Frames before the
first tracked field are skipped.

Usage:
    python playground/render_depth_birdseye.py <recording.mcap> <recording.svo2> \
        -o depth_birdseye.mp4
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from auto_battlebot.mcap_io import (
    decode_string,
    decode_tf_message,
    iter_messages,
)

REPO = Path(__file__).resolve().parents[1]

FRAME_META_TOPIC = "/camera/frame_meta"
TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"
FIELD_MARKERS_TOPIC = "/field_markers"
FIELD_FRAME = "field"
CAMERA_WORLD_FRAME = "camera_world"

# 8 ft, same constant as camera_geometry.NOMINAL_FIELD_SIZE_M.
NOMINAL_FIELD_SIZE_M = 2.4384

DEFAULT_DEPTH_MODE = "NEURAL_PLUS"

# config/_common.toml does not set depth_minimum_distance; 0.3 m is the ZED default for METER
# units and the arena floor is never closer than that. Same as cache_gt_depth.py.
DEPTH_MIN_M = 0.3

BACKGROUND_BGR = (35, 35, 35)
BORDER_BGR = (255, 200, 50)
HIGHLIGHT_BGR = (255, 0, 255)
GRID_BGR = (70, 70, 70)
TEXT_BGR = (220, 220, 220)


@dataclass
class Tick:
    """One processed pipeline frame with its SVO join key and tracked field state."""

    svo_frame_index: int
    image_stamp_ns: int
    log_time_ns: int
    tf_field_from_camera: np.ndarray  # 4x4, camera-frame points into field frame
    tf_field_from_cameraworld: np.ndarray  # 4x4, this tick's field edge, for the border marker
    field_corners: np.ndarray | None = None  # (4, 2) field-frame border in effect at this tick


def load_ticks(mcap_path: Path) -> list[Tick] | None:
    """Every tick with a valid SVO index and a tracked camera->field transform.

    `/tf` edges land after their tick's frame_meta and before the next one, so the tick a tf
    message belongs to is the last frame_meta at or before its log time. `field ->
    camera_world` is constant between field re-inits, so the latest one seen is carried
    forward; `camera_world -> camera` is per-tick tracking output and is never carried, a tick
    without it was not tracked and is skipped.
    """
    metas: list[tuple[int, int, int]] = []  # (log_time, image_stamp_ns, svo_frame_index)
    for _topic, ts, data in iter_messages(mcap_path, [FRAME_META_TOPIC]):
        payload = json.loads(decode_string(data))
        metas.append((ts, int(payload["image_stamp_ns"]), int(payload["svo_frame_index"])))
    if not metas:
        return None  # recording predates frame_meta; caller falls back to the interval join

    meta_log = np.array([m[0] for m in metas], dtype=np.int64)
    field_from_cameraworld: dict[int, np.ndarray] = {}
    cameraworld_from_camera: dict[int, np.ndarray] = {}
    for _topic, ts, data in iter_messages(mcap_path, [TF_TOPIC]):
        tick = int(np.searchsorted(meta_log, ts, side="right")) - 1
        if tick < 0:
            continue
        for transform in decode_tf_message(data):
            if transform.parent_frame_id == FIELD_FRAME:
                field_from_cameraworld[tick] = transform.matrix
            elif transform.parent_frame_id == CAMERA_WORLD_FRAME:
                cameraworld_from_camera[tick] = transform.matrix

    ticks: list[Tick] = []
    latest_field: np.ndarray | None = None
    for index, (log_time, image_stamp_ns, svo_index) in enumerate(metas):
        latest_field = field_from_cameraworld.get(index, latest_field)
        dynamic = cameraworld_from_camera.get(index)
        if svo_index < 0 or latest_field is None or dynamic is None:
            continue
        ticks.append(
            Tick(svo_index, image_stamp_ns, log_time, latest_field @ dynamic, latest_field)
        )
    return ticks


def load_stamp_ticks(mcap_path: Path) -> list[tuple[int, int, np.ndarray, np.ndarray]]:
    """(log_time_ns, stamp_ns, tf_field_from_camera, field edge) per tick, pre-frame_meta mcaps.

    Iterates `/tf` and `/tf_static` together in log order so the latest field edge is carried
    forward across re-inits, exactly as a live tf2 buffer would have resolved it.
    """
    ticks: list[tuple[int, int, np.ndarray, np.ndarray]] = []
    latest_field: np.ndarray | None = None
    for _topic, ts, data in iter_messages(mcap_path, [TF_TOPIC, TF_STATIC_TOPIC]):
        for transform in decode_tf_message(data):
            if transform.parent_frame_id == FIELD_FRAME:
                latest_field = transform.matrix
            elif transform.parent_frame_id == CAMERA_WORLD_FRAME and latest_field is not None:
                ticks.append(
                    (ts, transform.stamp_ns, latest_field @ transform.matrix, latest_field)
                )
    return ticks


def scan_svo_stamps(svo_path: Path) -> np.ndarray:
    """Every frame's recorded stamp_ns, in order, from a fast depth-off pass over the SVO."""
    import pyzed.sl as sl  # type: ignore[import-untyped]

    init = sl.InitParameters()
    init.set_from_svo_file(str(svo_path))
    init.depth_mode = sl.DEPTH_MODE.NONE
    init.sdk_verbose = 0
    camera = sl.Camera()
    status = camera.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"failed to open {svo_path}: {status}")
    stamps: list[int] = []
    try:
        while camera.grab(sl.RuntimeParameters()) == sl.ERROR_CODE.SUCCESS:
            stamps.append(camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds())
    finally:
        camera.close()
    return np.array(stamps, dtype=np.int64)


def assign_svo_indices(
    stamp_ticks: list[tuple[int, int, np.ndarray, np.ndarray]], svo_stamps: np.ndarray
) -> list[Tick]:
    """Resolve each pipeline stamp to the SVO frame whose interval contains it."""
    ticks: list[Tick] = []
    for log_time_ns, stamp_ns, tf_field_from_camera, field_edge in stamp_ticks:
        index = int(np.searchsorted(svo_stamps, stamp_ns, side="right")) - 1
        if index < 0:
            continue
        # One tf per frame: a repeated index means the pipeline republished within one frame
        # interval, keep the first.
        if ticks and ticks[-1].svo_frame_index == index:
            continue
        ticks.append(Tick(index, stamp_ns, log_time_ns, tf_field_from_camera, field_edge))
    return ticks


def load_field_markers(mcap_path: Path) -> list[tuple[int, np.ndarray]]:
    """(log_time_ns, (4, 3) border corners in camera_world) for every field re-init, in order."""
    # mcap_ros1 handles the visualization_msgs decoding; imported here so the rest of the
    # script works without it (the border overlay is then skipped).
    try:
        from mcap_ros1.reader import read_ros1_messages
    except ImportError:
        return []
    markers: list[tuple[int, np.ndarray]] = []
    for msg in read_ros1_messages(source=str(mcap_path), topics=[FIELD_MARKERS_TOPIC]):
        for marker in msg.ros_msg.markers:
            if marker.ns != "field" or len(marker.points) < 4:
                continue
            # The border is a closed LINE_STRIP whose first four points are the corners.
            corners = np.array([[p.x, p.y, p.z] for p in marker.points[:4]], dtype=np.float64)
            markers.append((msg.log_time_ns, corners))
    return markers


def attach_field_corners(ticks: list[Tick], markers: list[tuple[int, np.ndarray]]) -> None:
    """Give each tick the border marker in effect at its log time, never a later fit.

    Marker corners are recorded in the camera_world frame of their init; mapping them through
    the tick's own field edge puts them in the field frame the points are rendered in. When
    marker and edge come from the same init this lands the fitted rectangle centered on the
    origin; a visibly offset border means the two edges straddle a re-init boundary.
    """
    if not markers:
        return
    marker_log = np.array([log_time for log_time, _ in markers], dtype=np.int64)
    for tick in ticks:
        index = int(np.searchsorted(marker_log, tick.log_time_ns, side="right")) - 1
        if index < 0:
            continue
        corners_cameraworld = markers[index][1]
        rotation = tick.tf_field_from_cameraworld[:3, :3]
        translation = tick.tf_field_from_cameraworld[:3, 3]
        tick.field_corners = (corners_cameraworld @ rotation.T + translation)[:, :2]


def raster_extent(markers: list[tuple[int, np.ndarray]]) -> tuple[float, float]:
    """Fixed (x, y) extent covering every fitted border, never below the nominal square.

    The May-era fits underestimate the field badly (y down to 0.27 m for an 8 ft arena), so
    the raster never crops below the nominal square: each tick's fitted rectangle is drawn as
    the border, but the floor around it stays visible.
    """
    extent_x, extent_y = NOMINAL_FIELD_SIZE_M, NOMINAL_FIELD_SIZE_M
    for _log_time, corners in markers:
        edges = [float(np.linalg.norm(corners[(i + 1) % 4] - corners[i])) for i in range(4)]
        extent_x = max(extent_x, (edges[0] + edges[2]) / 2.0)
        extent_y = max(extent_y, (edges[1] + edges[3]) / 2.0)
    return extent_x, extent_y


# Sentinel for raster cells no point landed in; anything measured is far above it.
UNMEASURED_Z = -1.0e6


class BirdsEyeRaster:
    """Top-down raster of field-frame points, highest point per pixel, colored by height."""

    def __init__(
        self,
        extent_m: tuple[float, float],
        margin_m: float,
        pixels_per_meter: float,
        height_clip_m: float,
        highlight_range_m: tuple[float, float],
        splat_px: int = 3,
    ) -> None:
        self.half_extent_x = extent_m[0] / 2.0 + margin_m
        self.half_extent_y = extent_m[1] / 2.0 + margin_m
        self.ppm = pixels_per_meter
        # Points further than this above or below the field plane are dropped, and the
        # colormap spans exactly the band that survives.
        self.z_min, self.z_max = -height_clip_m, height_clip_m
        self.highlight_lo, self.highlight_hi = highlight_range_m
        self.splat_px = splat_px
        self.width = int(round(2.0 * self.half_extent_x * self.ppm))
        self.height = int(round(2.0 * self.half_extent_y * self.ppm))

    def to_pixels(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Field-frame metres to raster pixels, +x right and +y up on screen."""
        u = ((x + self.half_extent_x) * self.ppm).astype(np.int32)
        v = ((self.half_extent_y - y) * self.ppm).astype(np.int32)
        return u, v

    def render(self, points_field: np.ndarray, field_corners: np.ndarray | None) -> np.ndarray:
        """BGR frame from an (N, 3) field-frame cloud. NaN rows are ignored."""
        finite = np.isfinite(points_field).all(axis=1)
        x, y, z = points_field[finite].T
        inside = (
            (np.abs(x) <= self.half_extent_x)
            & (np.abs(y) <= self.half_extent_y)
            & (z >= self.z_min)
            & (z <= self.z_max)
        )
        x, y, z = x[inside], y[inside], z[inside]

        u, v = self.to_pixels(x, y)
        np.clip(u, 0, self.width - 1, out=u)
        np.clip(v, 0, self.height - 1, out=v)

        top = np.full(self.height * self.width, UNMEASURED_Z, dtype=np.float32)
        np.maximum.at(top, v * self.width + u, z.astype(np.float32))
        top_image = top.reshape(self.height, self.width)
        if self.splat_px > 1:
            # At range the projected point spacing is coarser than the raster, leaving moire
            # gaps between scanlines. Grayscale dilation is a max filter, so filling with it
            # stays consistent with highest-point-wins.
            kernel = np.ones((self.splat_px, self.splat_px), np.uint8)
            top_image = cv2.dilate(top_image, kernel)
        measured = top_image > UNMEASURED_Z / 2.0

        normalized = np.zeros_like(top_image)
        span = max(self.z_max - self.z_min, 1e-6)
        normalized[measured] = np.clip((top_image[measured] - self.z_min) / span, 0.0, 1.0)
        frame = cv2.applyColorMap((normalized * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
        frame[~measured] = BACKGROUND_BGR
        highlight = measured & (top_image >= self.highlight_lo) & (top_image <= self.highlight_hi)
        frame[highlight] = HIGHLIGHT_BGR

        self._draw_overlay(frame, field_corners)
        return frame

    def _draw_overlay(self, frame: np.ndarray, field_corners: np.ndarray | None) -> None:
        for grid_x in np.arange(-self.half_extent_x, self.half_extent_x, 0.5):
            u, _ = self.to_pixels(np.array([grid_x]), np.array([0.0]))
            cv2.line(frame, (int(u[0]), 0), (int(u[0]), self.height - 1), GRID_BGR, 1)
        for grid_y in np.arange(-self.half_extent_y, self.half_extent_y, 0.5):
            _, v = self.to_pixels(np.array([0.0]), np.array([grid_y]))
            cv2.line(frame, (0, int(v[0])), (self.width - 1, int(v[0])), GRID_BGR, 1)

        if field_corners is not None:
            u, v = self.to_pixels(field_corners[:, 0], field_corners[:, 1])
            polygon = np.stack([u, v], axis=1).reshape(-1, 1, 2)
            cv2.polylines(frame, [polygon], isClosed=True, color=BORDER_BGR, thickness=2)


def open_svo_camera(svo_path: Path, depth_mode: str):
    """SVO opened with the same conventions the C++ stack uses, so /tf applies directly."""
    import pyzed.sl as sl  # type: ignore[import-untyped]

    init = sl.InitParameters()
    init.set_from_svo_file(str(svo_path))
    init.depth_mode = getattr(sl.DEPTH_MODE, depth_mode)
    init.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
    init.coordinate_units = sl.UNIT.METER
    init.depth_minimum_distance = DEPTH_MIN_M
    init.sdk_verbose = 0
    camera = sl.Camera()
    status = camera.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"failed to open {svo_path}: {status}")
    return camera


def compose_frame(
    camera, tick: Tick, raster: BirdsEyeRaster, rgb_width: int, label: str
) -> np.ndarray:
    """RGB panel beside the bird's-eye projection for the frame just grabbed."""
    import pyzed.sl as sl  # type: ignore[import-untyped]

    xyz_mat = sl.Mat()
    rgb_mat = sl.Mat()
    camera.retrieve_measure(xyz_mat, sl.MEASURE.XYZ)
    camera.retrieve_image(rgb_mat, sl.VIEW.LEFT)
    points_camera = xyz_mat.get_data()[:, :, :3].reshape(-1, 3).astype(np.float64)
    # Pixels with no stereo match come back NaN; drop them before the transform.
    points_camera = points_camera[np.isfinite(points_camera[:, 2])]

    rotation = tick.tf_field_from_camera[:3, :3]
    translation = tick.tf_field_from_camera[:3, 3]
    points_field = points_camera @ rotation.T + translation

    birdseye = raster.render(points_field, tick.field_corners)
    cv2.putText(birdseye, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, TEXT_BGR, 1, cv2.LINE_AA)
    rgb = cv2.cvtColor(rgb_mat.get_data(), cv2.COLOR_BGRA2BGR)
    rgb = cv2.resize(rgb, (rgb_width, raster.height), interpolation=cv2.INTER_AREA)
    return np.hstack([rgb, birdseye])


def iter_writable_frames(camera, ticks: list[Tick], first_index: int, last_index: int, stride: int):
    """Yield (position, tick in effect, fresh, stamp_ns) per frame to write, grabbing in order.

    Seeking lands on non-keyframes and smears the decode, so every frame is grabbed
    sequentially; frames that are not written skip the depth compute (a decode-only grab is a
    few milliseconds). Stops early on a failed grab.
    """
    import pyzed.sl as sl  # type: ignore[import-untyped]

    runtime = sl.RuntimeParameters()
    skip_runtime = sl.RuntimeParameters()
    skip_runtime.enable_depth = False
    tick_cursor = 0
    current: Tick | None = None
    while camera.get_svo_position() < first_index:
        if camera.grab(skip_runtime) != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"failed to reach frame {first_index}")
    for position in range(first_index, last_index + 1):
        write_this = (position - first_index) % stride == 0
        if camera.grab(runtime if write_this else skip_runtime) != sl.ERROR_CODE.SUCCESS:
            print(f"  grab failed at frame {position}; stopping early")
            return
        while tick_cursor < len(ticks) and ticks[tick_cursor].svo_frame_index <= position:
            current = ticks[tick_cursor]
            tick_cursor += 1
        if not write_this or current is None:
            continue
        stamp_ns = camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
        yield position, current, current.svo_frame_index == position, stamp_ns


def render_video(
    ticks: list[Tick],
    raster: BirdsEyeRaster,
    svo_path: Path,
    depth_mode: str,
    output: Path,
    fps_override: float | None,
    stride: int,
    max_frames: int | None,
) -> tuple[int, int]:
    """Replay the SVO through pyzed and write one RGB + bird's-eye frame per SVO frame.

    Every SVO frame from the first tracked tick to the last is written (every `stride`-th when
    striding, with the fps divided to match), so playback runs at the SVO's own rate. Frames
    the pipeline dropped reuse the most recent tick's field pose and are tagged "held".
    Returns (rendered, held).
    """
    camera = open_svo_camera(svo_path, depth_mode)
    total_svo_frames = camera.get_svo_number_of_frames()
    config = camera.get_camera_information().camera_configuration
    fps = fps_override if fps_override is not None else config.fps / stride
    rgb_width = int(round(config.resolution.width * raster.height / config.resolution.height))
    frame_size = (rgb_width + raster.width, raster.height)
    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"mp4v"), fps, frame_size)
    if not writer.isOpened():
        camera.close()
        raise RuntimeError(f"failed to open video writer for {output}")

    first_index = ticks[0].svo_frame_index
    last_index = min(ticks[-1].svo_frame_index, total_svo_frames - 1)
    planned = (last_index - first_index) // stride + 1
    if max_frames is not None:
        planned = min(planned, max_frames)
    print(f"  video {frame_size[0]}x{frame_size[1]} at {fps:.1f} fps, {planned} frames")

    start_stamp_ns: int | None = None
    rendered = 0
    held = 0
    started = time.monotonic()
    try:
        frames = iter_writable_frames(camera, ticks, first_index, last_index, stride)
        for position, tick, fresh, stamp_ns in frames:
            if max_frames is not None and rendered >= max_frames:
                break
            held += 0 if fresh else 1
            if start_stamp_ns is None:
                start_stamp_ns = stamp_ns
            label = f"svo {position}  t={(stamp_ns - start_stamp_ns) / 1e9:7.2f}s"
            if not fresh:
                label += "  held"
            writer.write(compose_frame(camera, tick, raster, rgb_width, label))
            rendered += 1
            if rendered % 300 == 0:
                rate = rendered / (time.monotonic() - started)
                remaining_s = (planned - rendered) / max(rate, 1e-6)
                print(
                    f"  {rendered}/{planned} frames, {rate:.1f} fps, "
                    f"~{remaining_s / 60:.1f} min left"
                )
    finally:
        writer.release()
        camera.close()
    return rendered, held


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("mcap", type=Path, help="recording MCAP with /camera/frame_meta and /tf")
    parser.add_argument("svo", type=Path, help="matching .svo2 file")
    parser.add_argument("-o", "--output", type=Path, default=None, help="output .mp4 path")
    parser.add_argument("--depth-mode", default=DEFAULT_DEPTH_MODE, help="ZED DEPTH_MODE name")
    parser.add_argument(
        "--pixels-per-meter", type=float, default=240.0, help="bird's-eye raster scale"
    )
    parser.add_argument(
        "--margin", type=float, default=0.3, help="metres shown beyond the field border"
    )
    parser.add_argument(
        "--height-clip",
        type=float,
        default=0.1,
        help="drop points further than this above or below the field plane, in metres",
    )
    parser.add_argument(
        "--highlight-range",
        type=float,
        nargs=2,
        default=(0.01, 0.05),
        metavar=("MIN", "MAX"),
        help="paint pixels whose top point sits in this height band above the plane",
    )
    parser.add_argument(
        "--splat",
        type=int,
        default=3,
        help="max-filter kernel in pixels that fills gaps between projected points",
    )
    parser.add_argument(
        "--stride",
        type=int,
        default=1,
        help="write every Nth SVO frame; fps is divided so playback stays real-time",
    )
    parser.add_argument("--max-frames", type=int, default=None, help="stop after N rendered frames")
    parser.add_argument("--fps", type=float, default=None, help="override output video fps")
    args = parser.parse_args()

    output = args.output or Path.cwd() / f"{args.mcap.stem}_depth_birdseye.mp4"

    ticks = load_ticks(args.mcap)
    if ticks is None:
        print(f"{args.mcap.name} has no {FRAME_META_TOPIC}; using the stamp-interval join")
        stamp_ticks = load_stamp_ticks(args.mcap)
        svo_stamps = scan_svo_stamps(args.svo)
        print(f"  {len(stamp_ticks)} tf ticks, {len(svo_stamps)} SVO frames")
        ticks = assign_svo_indices(stamp_ticks, svo_stamps)
    if not ticks:
        print("no ticks with both an SVO index and a tracked field; nothing to render")
        return 1
    markers = load_field_markers(args.mcap)
    attach_field_corners(ticks, markers)
    extent = raster_extent(markers)

    raster = BirdsEyeRaster(
        extent,
        args.margin,
        args.pixels_per_meter,
        args.height_clip,
        tuple(args.highlight_range),
        args.splat,
    )
    print(
        f"{len(ticks)} ticks, {len(markers)} field inits, extent {extent[0]:.3f} x "
        f"{extent[1]:.3f} m, raster {raster.width}x{raster.height} -> {output}"
    )

    rendered, held = render_video(
        ticks, raster, args.svo, args.depth_mode, output, args.fps, args.stride, args.max_frames
    )

    print(f"wrote {rendered} frames to {output} ({held} held from dropped ticks)")
    return 0 if rendered else 1


if __name__ == "__main__":
    sys.exit(main())
