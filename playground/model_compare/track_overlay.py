"""Shared plumbing for the old-vs-new opponent overlay.

Both runs store the opponent as a field-frame track, not a 2D box. Each run's field
position was derived from its own 2D detection through its own field<-camera transform,
so projecting a track back through the transform its own run used recovers roughly the
2D detection that produced it. Projecting both through one shared transform would fold
the field-fit difference (0.41 m / 21 deg between these two runs) into the picture and
read as opponent-model error, which is the opposite of what this video is for.
"""

from __future__ import annotations

import bisect
import json
from dataclasses import dataclass
from typing import Iterator

import numpy as np
from mcap.reader import make_reader
from mcap_ros1.decoder import DecoderFactory

# The live run labels the track "their_robot_1"; the current build appends the detected
# class, as in "their_robot_1 (opponent)". Match on the prefix so both runs parse.
OPPONENT_LABEL_PREFIX = "their_robot"


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def transform_matrix(translation, rotation) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = quat_to_matrix(rotation[0], rotation[1], rotation[2], rotation[3])
    out[:3, 3] = translation
    return out


@dataclass
class Sample:
    """One run's opponent estimate at one pipeline tick."""

    stamp_ns: int  # raw SVO-clock ns, comparable across runs
    position: np.ndarray  # field frame, metres
    radius: float  # footprint radius, metres (marker scale is a diameter)
    tf_field_from_camera: np.ndarray


class TimeSeries:
    """Nearest-value lookup over monotonically stamped items."""

    def __init__(self, stamps: list[int], values: list):
        order = np.argsort(stamps)
        self.stamps = [stamps[i] for i in order]
        self.values = [values[i] for i in order]

    def nearest(self, stamp_ns: int, tolerance_ns: int | None = None):
        if not self.stamps:
            return None
        i = bisect.bisect_left(self.stamps, stamp_ns)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(self.stamps):
                d = abs(self.stamps[j] - stamp_ns)
                if best is None or d < best[0]:
                    best = (d, self.values[j])
        if best is None or (tolerance_ns is not None and best[0] > tolerance_ns):
            return None
        return best[1]

    def latest_at(self, stamp_ns: int):
        i = bisect.bisect_right(self.stamps, stamp_ns) - 1
        return self.values[i] if i >= 0 else None


def _iter(path: str, topics: list[str]) -> Iterator:
    with open(path, "rb") as handle:
        reader = make_reader(handle, decoder_factories=[DecoderFactory()])
        yield from reader.iter_decoded_messages(topics=topics)


def read_frame_meta(path: str) -> tuple[TimeSeries, TimeSeries]:
    """Return (log_time -> raw stamp) and (raw stamp -> svo frame index) for a replay."""
    log_times, raw_stamps, idx_stamps, indices = [], [], [], []
    for _, _, msg, dec in _iter(path, ["/camera/frame_meta"]):
        meta = json.loads(dec.data)
        log_times.append(msg.log_time)
        raw_stamps.append(int(meta["image_stamp_ns"]))
        idx_stamps.append(int(meta["image_stamp_ns"]))
        indices.append(int(meta["svo_frame_index"]))
    return TimeSeries(log_times, raw_stamps), TimeSeries(idx_stamps, indices)


def read_camera_matrix(path: str) -> np.ndarray:
    for _, _, _, dec in _iter(path, ["/camera/camera_info"]):
        k = list(dec.K)
        return np.array([[k[0], k[1], k[2]], [k[3], k[4], k[5]], [k[6], k[7], k[8]]])
    raise RuntimeError(f"no /camera/camera_info in {path}")


def read_transforms(path: str) -> tuple[TimeSeries, TimeSeries]:
    """field<-camera_world is latched on /tf_static; camera_world<-camera ticks per frame."""
    st_t, st_v, dy_t, dy_v = [], [], [], []
    for _, channel, msg, dec in _iter(path, ["/tf", "/tf_static"]):
        for tf in dec.transforms:
            tr = tf.transform.translation
            ro = tf.transform.rotation
            mat = transform_matrix([tr.x, tr.y, tr.z], [ro.x, ro.y, ro.z, ro.w])
            pair = (tf.header.frame_id, tf.child_frame_id)
            if pair == ("field", "camera_world"):
                st_t.append(msg.log_time)
                st_v.append(mat)
            elif pair == ("camera_world", "camera"):
                dy_t.append(msg.log_time)
                dy_v.append(mat)
    return TimeSeries(st_t, st_v), TimeSeries(dy_t, dy_v)


def read_opponent_track(path: str, to_raw_stamp=None) -> TimeSeries:
    """Opponent samples keyed on the raw SVO clock.

    `to_raw_stamp` converts a replay's wall-clock-rebased log_time back to the raw SVO
    clock. The original live run is already on that clock, so it passes None.
    """
    statics, dynamics = read_transforms(path)
    stamps, samples = [], []
    labelled: dict[int, str] = {}
    for _, _, msg, dec in _iter(path, ["/robot_markers"]):
        for mk in dec.markers:
            if mk.ns == "robot_labels":
                labelled[mk.id] = mk.text
        bounds = [
            mk
            for mk in dec.markers
            if mk.ns == "robot_bounds"
            and labelled.get(mk.id, "").startswith(OPPONENT_LABEL_PREFIX)
        ]
        if not bounds:
            continue
        mk = bounds[0]
        static = statics.latest_at(msg.log_time)
        dynamic = dynamics.nearest(msg.log_time, tolerance_ns=100_000_000)
        if static is None or dynamic is None:
            continue
        raw = msg.log_time if to_raw_stamp is None else to_raw_stamp.nearest(msg.log_time)
        if raw is None:
            continue
        stamps.append(int(raw))
        samples.append(
            Sample(
                stamp_ns=int(raw),
                position=np.array([mk.pose.position.x, mk.pose.position.y, mk.pose.position.z]),
                radius=float(max(mk.scale.x, mk.scale.y)) / 2.0,
                tf_field_from_camera=static @ dynamic,
            )
        )
    return TimeSeries(stamps, samples)


def project_field_points(points: np.ndarray, tf_field_from_camera: np.ndarray, K: np.ndarray):
    """Field-frame points -> pixels. Returns (pixels, valid mask)."""
    cam_from_field = np.linalg.inv(tf_field_from_camera)
    homogeneous = np.hstack([points, np.ones((len(points), 1))])
    in_camera = (cam_from_field @ homogeneous.T).T[:, :3]
    depth = in_camera[:, 2]
    valid = depth > 1e-6
    safe = np.where(valid[:, None], in_camera, np.array([0.0, 0.0, 1.0]))
    pixels = (K @ (safe / safe[:, 2:3]).T).T[:, :2]
    return pixels, valid


def footprint_circle(center: np.ndarray, radius: float, segments: int = 48) -> np.ndarray:
    angle = np.linspace(0.0, 2.0 * np.pi, segments, endpoint=False)
    ring = np.zeros((segments, 3))
    ring[:, 0] = center[0] + radius * np.cos(angle)
    ring[:, 1] = center[1] + radius * np.sin(angle)
    ring[:, 2] = center[2]
    return ring
