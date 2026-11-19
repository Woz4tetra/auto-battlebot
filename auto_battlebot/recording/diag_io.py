#!/usr/bin/env python3
"""Loaders for control metrics from the /diagnostics and marker topics."""

from __future__ import annotations

from collections import defaultdict
from pathlib import Path
from typing import Any, Iterator

import numpy as np
import pandas as pd

from auto_battlebot.recording.mcap_io import (
    DiagnosticStatus,
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
NAV_HW_IDS = frozenset({PURSUIT_NAV_HW_ID, MOTION_PROFILE_NAV_HW_ID})
RUNNER_HW_ID = "runner"
TRANSMITTER_HW_ID = "opentx_transmitter"

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


_RUNNER_PREFIXES = {"pipeline": "pipeline", "navigation": "nav", "perception": "perc"}

_TRANSMITTER_COLUMNS = (
    ("values/0", "ch_linear"),
    ("values/1", "ch_angular"),
    ("values/15", "ch15"),
)


def _merge_status(rows: dict[int, dict[str, Any]], ts: int, status: DiagnosticStatus) -> None:
    """Fold one DiagnosticStatus into the row for its tick."""
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
            rows[ts][f"stage/{name}/elapsed_ms"] = kv["elapsed_ms"]
    elif hw == TRANSMITTER_HW_ID and name == "channels":
        for src, dst in _TRANSMITTER_COLUMNS:
            if src in kv:
                rows[ts][dst] = kv[src]


def iter_diagnostic_statuses(path: Path | str) -> Iterator[tuple[int, DiagnosticStatus]]:
    """Every diagnostic status in a recording as (log_time_ns, status), in log order."""
    for _topic, log_time_ns, data in iter_messages(path, [DIAGNOSTICS_TOPIC]):
        for status in decode_diagnostic_array(data):
            yield log_time_ns, status


def load_diagnostics(path: Path) -> pd.DataFrame:
    """One row per tick keyed on message log_time."""
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
    """Forward-fill the transmitter channels, which are logged only on change."""
    ch15 = df["ch15"].ffill() if "ch15" in df.columns else pd.Series(index=df.index, dtype=object)
    df["is_auto"] = pd.to_numeric(ch15, errors="coerce") == 1024

    for col in ("ch_linear", "ch_angular"):
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce").ffill()


def load_field_size(path: Path) -> tuple[float, float] | None:
    """Arena (width, height) in metres, recovered from the field border marker."""
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


def load_robot_tracks(path: Path) -> pd.DataFrame:
    """One row per /robot_markers message: which robots were present that tick."""
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
    """One row per robot per /robot_markers message with its marker pose."""
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


def load_camera_in_field(path: Path) -> pd.DataFrame:
    """Per-tick camera optical-centre position in the field frame."""
    field_from_world: np.ndarray | None = None
    for _topic, _ts, data in iter_messages(path, [TF_STATIC_TOPIC]):
        for tr in decode_tf_message(data):
            if tr.key == ("field", "camera_world"):
                field_from_world = tr.parent_from_child
    if field_from_world is None:
        return pd.DataFrame(columns=["timestamp_ns", "cam_x", "cam_y", "cam_z"])

    records = []
    for _topic, ts, data in iter_messages(path, [TF_TOPIC]):
        for tr in decode_tf_message(data):
            if tr.key == ("camera_world", "camera"):
                world_from_cam = tr.parent_from_child
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
