#!/usr/bin/env python3
"""Loaders for Stage 0 control metrics.

Reuses the diagnostics decoder from the shared ``auto_battlebot.mcap_io`` package and
adds extraction of the extra subsections and topics Stage 0 needs:

- runner/navigation/using_previous_robots  (reliability / dropout proxy)
- runner/perception/*                      (future-run detection counts, when present)
- runner stage FunctionTimers              (per-stage elapsed_ms)
- /robot_markers                           (opponent presence + frame-id switches)
- /field_markers                           (arena size, from corner edge lengths)
- /tf + /tf_static                         (camera pose in the field frame)

All data here comes straight from existing Jetson recordings; nothing requires
re-running the stack (laptop results differ from the Jetson).

Reads the per-module ``/diagnostics/<module>`` JSON channels with typed values
(``docs/foxglove_recording_format.md``).

Dependencies: mcap, numpy, pandas
"""

from __future__ import annotations

from collections import defaultdict
from pathlib import Path
from typing import Any, Iterator

import numpy as np
import pandas as pd

from auto_battlebot.mcap_io import (
    decode_diagnostic_array,
    decode_scene_update,
    decode_tf_message,
    iter_messages,
)

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


# Runner sub-status name -> column prefix. Anything else the runner publishes with an
# elapsed_ms is a FunctionTimer stage and is namespaced by its own name instead.
_RUNNER_PREFIXES = {"pipeline": "pipeline", "navigation": "nav", "perception": "perc"}

# Transmitter channel -> column. values/15 is the auto switch; values/0 and values/1 are the
# actual transmitted drive command (post trainer-mode mix of navigation and driver sticks),
# which is the true plant input.
_TRANSMITTER_COLUMNS = (
    ("values/0", "ch_linear"),
    ("values/1", "ch_angular"),
    ("values/15", "ch15"),
)


def _merge_status(rows: dict[int, dict[str, Any]], ts: int, status: dict[str, Any]) -> None:
    """Fold one DiagnosticStatus into the row for its tick.

    `rows` is a defaultdict, so it is indexed only where this status actually contributes a
    column. Touching it unconditionally would mint an all-NaN row for every message that
    carries nothing this loader wants.
    """
    hw = status["hardware_id"]
    name = status["name"]
    kv = status["values"]

    if hw in NAV_HW_IDS:
        rows[ts].update(kv)
    elif hw == RUNNER_HW_ID:
        prefix = _RUNNER_PREFIXES.get(name)
        if prefix is not None:
            for key, value in kv.items():
                rows[ts][f"{prefix}/{key}"] = value
        elif "elapsed_ms" in kv:
            # FunctionTimer stage (tick, camera.get, robot_filter.update, ...)
            rows[ts][f"stage/{name}/elapsed_ms"] = kv["elapsed_ms"]
    elif hw == TRANSMITTER_HW_ID and name == "channels":
        for src, dst in _TRANSMITTER_COLUMNS:
            if src in kv:
                rows[ts][dst] = kv[src]


def iter_diagnostic_statuses(path: Path | str) -> Iterator[tuple[int, dict[str, Any]]]:
    """Every diagnostic status in a recording as (log_time_ns, status dict), in log order.

    Status dicts are ``{level, name, message, hardware_id, values}`` with typed values.
    """
    for _topic, log_time_ns, data in iter_messages(path, [DIAGNOSTICS_TOPIC]):
        for status in decode_diagnostic_array(data):
            yield log_time_ns, status


def load_diagnostics(path: Path) -> pd.DataFrame:
    """One row per tick (keyed on message log_time), columns drawn from
    pursuit_nav, runner/pipeline, runner/navigation, runner/perception, the
    runner stage timers, and the transmitter auto channel."""
    rows: dict[int, dict[str, Any]] = defaultdict(dict)

    for log_time_ns, status in iter_diagnostic_statuses(path):
        _merge_status(rows, log_time_ns, status)

    if not rows:
        raise SystemExit(f"No /diagnostics found in {path}")

    df = pd.DataFrame([{**rows[ts], "timestamp_ns": ts} for ts in sorted(rows)])
    _fill_latched_channels(df)

    t0 = df["timestamp_ns"].iloc[0]
    df["t"] = (df["timestamp_ns"] - t0) / 1e9
    return df


def _fill_latched_channels(df: pd.DataFrame) -> None:
    """Forward-fill the transmitter channels, which are logged only on change.

    They are latched physical state. Defaulting the gaps instead would invent spurious
    auto<->manual transitions and zero-command ticks.
    """
    ch15 = df["ch15"].ffill() if "ch15" in df.columns else pd.Series(index=df.index, dtype=object)
    df["is_auto"] = pd.to_numeric(ch15, errors="coerce") == 1024

    for col in ("ch_linear", "ch_angular"):
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce").ffill()


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
    for _topic, _ts, data in iter_messages(path, [FIELD_MARKERS_TOPIC]):
        for entity in decode_scene_update(data).entities:
            if entity.namespace != "field" or not entity.lines:
                continue
            points = entity.lines[0].points
            if len(points) < 4:
                continue
            pts = [np.array(p) for p in points[:4]]
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
    for _topic, ts, data in iter_messages(path, [ROBOT_MARKERS_TOPIC]):
        their, ours, neutral = [], [], []
        for entity in decode_scene_update(data).entities:
            if entity.namespace != "robot_bounds":
                continue
            name = frame_name(entity.index)
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
    for _topic, ts, data in iter_messages(path, [ROBOT_MARKERS_TOPIC]):
        for entity in decode_scene_update(data).entities:
            if entity.namespace != "robot_bounds" or not entity.cubes:
                continue
            name = frame_name(entity.index)
            position = entity.cubes[0].pose.position
            records.append(
                {
                    "timestamp_ns": ts,
                    "frame": name,
                    "group": group_of(name),
                    "x": float(position[0]),
                    "y": float(position[1]),
                }
            )
    return pd.DataFrame(records)


# ---------------------------------------------------------------------------
# /tf + /tf_static -> camera position in the field frame
# ---------------------------------------------------------------------------


def load_camera_in_field(path: Path) -> pd.DataFrame:
    """Per-tick camera optical-centre position in the field frame.

    Frame chain (header -> child): static field -> camera_world, dynamic
    camera_world -> camera. Camera-in-field = T(field<-camera_world) @
    T(camera_world<-camera); the translation is the camera origin in field
    coordinates. Returns columns timestamp_ns, cam_x, cam_y, cam_z.
    """
    field_from_world: np.ndarray | None = None
    for _topic, _ts, data in iter_messages(path, [TF_STATIC_TOPIC]):
        for tr in decode_tf_message(data):
            if tr.key == ("field", "camera_world"):
                field_from_world = tr.matrix
    if field_from_world is None:
        return pd.DataFrame(columns=["timestamp_ns", "cam_x", "cam_y", "cam_z"])

    records = []
    for _topic, ts, data in iter_messages(path, [TF_TOPIC]):
        for tr in decode_tf_message(data):
            if tr.key == ("camera_world", "camera"):
                world_from_cam = tr.matrix
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
