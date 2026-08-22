#!/usr/bin/env python3
"""Loaders for Stage 0 control metrics.

Reuses the diagnostic_msgs/DiagnosticArray byte decoder from the shared
``auto_battlebot.mcap_io`` package and adds extraction of the extra
subsections and topics Stage 0 needs:

- runner/navigation/using_previous_robots  (reliability / dropout proxy)
- runner/perception/*                      (future-run detection counts, when present)
- runner stage FunctionTimers              (per-stage elapsed_ms)
- /robot_markers                           (opponent presence + frame-id switches)
- /field_markers                           (arena size, from corner edge lengths)
- /tf + /tf_static                         (camera pose in the field frame)

All data here comes straight from existing Jetson recordings; nothing requires
re-running the stack (laptop results differ from the Jetson).

Dependencies: mcap, mcap_ros1, numpy, pandas
"""

from __future__ import annotations

from collections import defaultdict
from pathlib import Path

import numpy as np
import pandas as pd
from mcap.reader import make_reader
from mcap_ros1.reader import read_ros1_messages

# Canonical decoder lives in the shared package (install with `pip install -e .`).
from auto_battlebot.mcap_io import decode_diagnostic_array as _decode_diagnostic_array

DIAGNOSTICS_TOPIC = "/diagnostics"
ROBOT_MARKERS_TOPIC = "/robot_markers"
FIELD_MARKERS_TOPIC = "/field_markers"
TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"

PURSUIT_NAV_HW_ID = "pursuit_nav"
MOTION_PROFILE_NAV_HW_ID = "motion_profile_nav"
# Navigation implementations log the same tick schema (our_x, target_x, distance, angle_error_deg,
# facing_target, linear_x, angular_z, ...) under their own hardware_id. Only one nav runs per fight,
# so any of these ids is "the navigation stage" for a given recording.
NAV_HW_IDS = frozenset({PURSUIT_NAV_HW_ID, MOTION_PROFILE_NAV_HW_ID})
RUNNER_HW_ID = "runner"
TRANSMITTER_HW_ID = "opentx_transmitter"

# FrameId enum order from include/enums/frame_id.hpp. Robot markers set
# marker.id = enum_index(frame_id), so the index maps back to the name here.
FRAME_IDS = [
    "EMPTY",
    "VISUAL_ODOMETRY",
    "CAMERA_WORLD",
    "CAMERA",
    "OUR_ROBOT_1",
    "OUR_ROBOT_2",
    "THEIR_ROBOT_1",
    "THEIR_ROBOT_2",
    "THEIR_ROBOT_3",
    "NEUTRAL_ROBOT_1",
    "NEUTRAL_ROBOT_2",
    "FIELD",
]

# Numeric columns to coerce after assembling the diagnostics DataFrame.
_NUMERIC_DIAG_COLS = [
    "our_x",
    "our_y",
    "our_yaw_deg",
    "target_x",
    "target_y",
    "distance",
    "angle_to_target_deg",
    "angle_error_deg",
    "threshold_deg",
    "facing_target",
    "turn_commit",
    "linear_x",
    "angular_z",
    "pipeline/latency_ms",
    "nav/using_previous_robots",
    "perc/their_count_live",
    "perc/their_count_total",
    "perc/our_present_live",
]


def _log_time_ns(m) -> int:
    """Normalize an mcap_ros1 message log_time (datetime or int) to int ns,
    matching the raw mcap reader used for /diagnostics."""
    lt = m.log_time
    if isinstance(lt, (int, np.integer)):
        return int(lt)
    return int(round(lt.timestamp() * 1e9))


def frame_name(marker_id: int) -> str:
    if 0 <= marker_id < len(FRAME_IDS):
        return FRAME_IDS[marker_id]
    return f"ID_{marker_id}"


def group_of(name: str) -> str:
    if name.startswith("OUR_"):
        return "OURS"
    if name.startswith("THEIR_"):
        return "THEIRS"
    if name.startswith("NEUTRAL_"):
        return "NEUTRAL"
    return "OTHER"


# ---------------------------------------------------------------------------
# /diagnostics
# ---------------------------------------------------------------------------


def load_diagnostics(path: Path) -> pd.DataFrame:
    """One row per tick (keyed on message log_time), columns drawn from
    pursuit_nav, runner/pipeline, runner/navigation, runner/perception, the
    runner stage timers, and the transmitter auto channel."""
    rows: dict[int, dict[str, str]] = defaultdict(dict)

    with open(path, "rb") as f:
        reader = make_reader(f)
        for _schema, channel, message in reader.iter_messages():
            if channel.topic != DIAGNOSTICS_TOPIC:
                continue
            ts = message.log_time
            for status in _decode_diagnostic_array(message.data):
                hw = status["hardware_id"]
                name = status["name"]
                kv = status["values"]
                if hw in NAV_HW_IDS:
                    rows[ts].update(kv)
                elif hw == RUNNER_HW_ID:
                    if name == "pipeline":
                        for k, v in kv.items():
                            rows[ts][f"pipeline/{k}"] = v
                    elif name == "navigation":
                        for k, v in kv.items():
                            rows[ts][f"nav/{k}"] = v
                    elif name == "perception":
                        for k, v in kv.items():
                            rows[ts][f"perc/{k}"] = v
                    elif "elapsed_ms" in kv:
                        # FunctionTimer stage (tick, camera.get, robot_filter.update, ...)
                        rows[ts][f"stage/{name}/elapsed_ms"] = kv["elapsed_ms"]
                elif hw == TRANSMITTER_HW_ID and name == "channels":
                    # values/15 = auto switch. values/0,1 = the actual transmitted drive command
                    # (post trainer-mode mix of navigation + driver sticks), the true plant input.
                    for src, dst in (
                        ("values/0", "ch_linear"),
                        ("values/1", "ch_angular"),
                        ("values/15", "ch15"),
                    ):
                        if src in kv:
                            rows[ts][dst] = kv[src]

    if not rows:
        raise SystemExit(f"No /diagnostics found in {path}")

    timestamps = sorted(rows)
    records = []
    for ts in timestamps:
        row = rows[ts]
        row["timestamp_ns"] = ts
        records.append(row)
    df = pd.DataFrame(records)

    for col in _NUMERIC_DIAG_COLS:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    # Any stage timer columns are numeric too.
    for col in df.columns:
        if col.startswith("stage/") and col.endswith("/elapsed_ms"):
            df[col] = pd.to_numeric(df[col], errors="coerce")

    # The auto switch channel is logged only on a fraction of ticks. It is a
    # latched physical state, so forward-fill it across the gaps; defaulting the
    # gaps to "not auto" would invent spurious auto<->manual transitions.
    if "ch15" in df.columns:
        ch15 = df["ch15"].ffill()
    else:
        ch15 = pd.Series(index=df.index, dtype=object)
    df["is_auto"] = ch15.astype(str).str.strip() == "1024"

    # Drive command channels are also logged only on change; forward-fill them.
    for col in ("ch_linear", "ch_angular"):
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce").ffill()

    t0 = df["timestamp_ns"].iloc[0]
    df["t"] = (df["timestamp_ns"] - t0) / 1e9
    return df


# ---------------------------------------------------------------------------
# /field_markers -> arena size (field-center frame)
# ---------------------------------------------------------------------------


def load_field_size(path: Path) -> tuple[float, float] | None:
    """Arena (width, height) in metres, recovered from the field border marker.

    The C++ encoder builds the four corners as (+-size.x/2, +-size.y/2) in the
    field-center frame and then rigid-transforms them, so the rectangle edge
    lengths recover the true size regardless of the frame they are expressed in.

    The field is published a few times (initial + re-inits) and early detections
    can be bad. The arena is fixed, so the largest-area detection is the best
    estimate; return that.
    """
    best: tuple[float, float] | None = None
    best_area = -1.0
    for m in read_ros1_messages(str(path), topics=[FIELD_MARKERS_TOPIC]):
        for mk in m.ros_msg.markers:
            if mk.ns != "field" or len(mk.points) < 4:
                continue
            pts = [np.array([p.x, p.y, p.z]) for p in mk.points[:4]]
            width = float(np.linalg.norm(pts[1] - pts[0]))
            height = float(np.linalg.norm(pts[2] - pts[1]))
            if width * height > best_area:
                best_area = width * height
                best = (width, height)
    return best


# ---------------------------------------------------------------------------
# /robot_markers -> opponent track presence and frame-id switches
# ---------------------------------------------------------------------------


def load_robot_tracks(path: Path) -> pd.DataFrame:
    """One row per /robot_markers message: which robots were present that tick.

    Uses the CUBE body markers (ns == "robot_bounds"); marker.id maps to the
    FrameId enum index. Does not carry is_stale (markers do not encode it)."""
    records = []
    for m in read_ros1_messages(str(path), topics=[ROBOT_MARKERS_TOPIC]):
        ts = _log_time_ns(m)
        their, ours, neutral = [], [], []
        for mk in m.ros_msg.markers:
            if mk.ns != "robot_bounds":
                continue
            name = frame_name(mk.id)
            grp = group_of(name)
            if grp == "THEIRS":
                their.append(name)
            elif grp == "OURS":
                ours.append(name)
            elif grp == "NEUTRAL":
                neutral.append(name)
        their_sorted = sorted(set(their))
        records.append(
            {
                "timestamp_ns": ts,
                "n_their": len(their_sorted),
                "their_ids": ",".join(their_sorted),
                "primary_their": their_sorted[0] if their_sorted else "",
                "our_present": int(bool(ours)),
            }
        )
    return pd.DataFrame(records)


def load_robot_positions(path: Path) -> pd.DataFrame:
    """One row per robot per /robot_markers message: where each robot was that tick.

    Sibling of load_robot_tracks: same CUBE body markers (ns == "robot_bounds",
    marker.id -> FrameId enum index), but keeps the marker pose instead of only
    presence. Positions are in the marker's own frame (field-center for these
    recordings). Columns: timestamp_ns, frame, group, x, y."""
    records = []
    for m in read_ros1_messages(str(path), topics=[ROBOT_MARKERS_TOPIC]):
        ts = _log_time_ns(m)
        for mk in m.ros_msg.markers:
            if mk.ns != "robot_bounds":
                continue
            name = frame_name(mk.id)
            records.append(
                {
                    "timestamp_ns": ts,
                    "frame": name,
                    "group": group_of(name),
                    "x": float(mk.pose.position.x),
                    "y": float(mk.pose.position.y),
                }
            )
    return pd.DataFrame(records)


# ---------------------------------------------------------------------------
# /tf + /tf_static -> camera position in the field frame
# ---------------------------------------------------------------------------


def _tf_matrix(translation, rotation) -> np.ndarray:
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        r = np.eye(3)
    else:
        s = 2.0 / n
        r = np.array(
            [
                [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
                [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
                [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
            ]
        )
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = [translation.x, translation.y, translation.z]
    return m


def load_camera_in_field(path: Path) -> pd.DataFrame:
    """Per-tick camera optical-centre position in the field frame.

    Frame chain (header -> child): static field -> camera_world, dynamic
    camera_world -> camera. Camera-in-field = T(field<-camera_world) @
    T(camera_world<-camera); the translation is the camera origin in field
    coordinates. Returns columns timestamp_ns, cam_x, cam_y, cam_z.
    """
    field_from_world: np.ndarray | None = None
    for m in read_ros1_messages(str(path), topics=[TF_STATIC_TOPIC]):
        for tr in m.ros_msg.transforms:
            if tr.header.frame_id == "field" and tr.child_frame_id == "camera_world":
                field_from_world = _tf_matrix(
                    tr.transform.translation, tr.transform.rotation
                )
    if field_from_world is None:
        return pd.DataFrame(columns=["timestamp_ns", "cam_x", "cam_y", "cam_z"])

    records = []
    for m in read_ros1_messages(str(path), topics=[TF_TOPIC]):
        ts = _log_time_ns(m)
        for tr in m.ros_msg.transforms:
            if tr.header.frame_id == "camera_world" and tr.child_frame_id == "camera":
                world_from_cam = _tf_matrix(
                    tr.transform.translation, tr.transform.rotation
                )
                cam_in_field = field_from_world @ world_from_cam
                records.append(
                    {
                        "timestamp_ns": ts,
                        "cam_x": float(cam_in_field[0, 3]),
                        "cam_y": float(cam_in_field[1, 3]),
                        "cam_z": float(cam_in_field[2, 3]),
                    }
                )
    return pd.DataFrame(records)
