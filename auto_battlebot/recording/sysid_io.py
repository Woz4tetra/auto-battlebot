"""Readers for the drivetrain sysid topics: robot AprilTags, ESP32 diagnostics, sticks, field pose.

Contracts (written by the C++ app, see ``docs/plans/mujoco_warp_mr_stabs_plan.md``):

- ``/apriltag/robot_tags`` (JSON): one message per camera frame, frames without detections
  included. Each detection carries its four corners in OpenCV aruco order and every IPPE
  solution as (rvec, tvec, reprojection error). rvec/tvec map marker-frame points (origin at the
  tag center, x right, y up in the printed image, z out of the face) into the OpenCV camera
  frame (x right, y down, z forward).
- ``/robot/esp32_diagnostics`` (JSON): one flat object per firmware loop. ``host_receive_ns`` is
  the app clock at receipt (also the MCAP log_time); ``timestamp_ms`` is the robot clock.
- ``/diagnostics/opentx_transmitter``, subsection ``channels``: the 16 stick channels the radio
  echoes over USB, logged only when they change, as ``values/0`` .. ``values/15``.
- ``/tf`` and ``/tf_static``: ``field -> camera_world`` and ``camera_world -> camera``. The app
  republishes both on ``/tf`` every cycle; older recordings put the first on ``/tf_static``.
  ``diag_io.load_camera_in_field`` reads only ``/tf_static`` for it and returns an empty frame on
  current recordings, and it keeps only the translation, so this module has its own loader that
  returns the full transform.

Dependencies: mcap, numpy, pandas
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from auto_battlebot.recording.diag_io import TRANSMITTER_HW_ID, iter_diagnostic_statuses
from auto_battlebot.recording.mcap_io import decode_json, decode_tf_message, iter_messages

ROBOT_TAGS_TOPIC = "/apriltag/robot_tags"
ESP32_DIAGNOSTICS_TOPIC = "/robot/esp32_diagnostics"
TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"
FIELD_FRAME = "field"
CAMERA_WORLD_FRAME = "camera_world"
CAMERA_FRAME = "camera"

NUM_TRANSMITTER_CHANNELS = 16

# Field order of one /robot/esp32_diagnostics event, with the pandas dtype each column gets.
# Booleans stay bool; vbat and ibat are float with NaN where the firmware had no INA228 reading
# (null, or a recording older than the field).
ESP32_FIELDS: tuple[tuple[str, str], ...] = (
    ("host_receive_ns", "int64"),
    ("timestamp_ms", "int64"),
    ("radio_connected", "bool"),
    ("armed", "bool"),
    ("a_percent", "float64"),
    ("b_percent", "float64"),
    ("button_state", "bool"),
    ("flip_switch", "int64"),
    ("left_cmd", "float64"),
    ("right_cmd", "float64"),
    ("accel_x", "float64"),
    ("accel_y", "float64"),
    ("accel_z", "float64"),
    ("is_upside_down", "bool"),
    ("loop_us", "int64"),
    ("wifi_clients", "int64"),
    ("orientation_x", "float64"),
    ("orientation_y", "float64"),
    ("orientation_z", "float64"),
    ("pid_setpoint", "float64"),
    ("pid_output", "float64"),
    ("vbat", "float64"),
    ("ibat", "float64"),
)


# ---------------------------------------------------------------------------
# /apriltag/robot_tags
# ---------------------------------------------------------------------------


@dataclass
class TagCamera:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @property
    def matrix(self) -> np.ndarray:
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])


@dataclass
class TagSolution:
    rvec: np.ndarray  # (3,) Rodrigues, marker -> camera
    tvec: np.ndarray  # (3,) metres, tag center in the camera frame
    reprojection_error_px: float

    @property
    def transform(self) -> np.ndarray:
        """4x4 T_camera_tag (maps marker-frame points into the camera frame)."""
        return pose_matrix(rodrigues(self.rvec), self.tvec)


@dataclass
class TagDetection:
    tag_id: int
    corners: np.ndarray  # (4, 2) pixels, OpenCV aruco order
    decision_margin: float | None
    solutions: list[TagSolution]

    @property
    def size_px(self) -> float:
        """Mean edge length of the corner quad in pixels."""
        edges = np.roll(self.corners, -1, axis=0) - self.corners
        return float(np.mean(np.linalg.norm(edges, axis=1)))


@dataclass
class RobotTagsFrame:
    log_time_ns: int
    image_stamp_ns: int
    frame_id: str
    camera: TagCamera
    tag_size_m: float
    roi: tuple[int, int, int, int] | None
    detections: list[TagDetection]


def rodrigues(rvec: np.ndarray) -> np.ndarray:
    """Rotation vector to 3x3 matrix, same result as ``cv2.Rodrigues``."""
    r = np.asarray(rvec, dtype=np.float64).reshape(3)
    theta = float(np.linalg.norm(r))
    if theta < 1e-12:
        return np.eye(3)
    k = r / theta
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    out: np.ndarray = np.eye(3) + math.sin(theta) * kx + (1.0 - math.cos(theta)) * (kx @ kx)
    return out


def pose_matrix(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = rotation
    m[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return m


def _float_or_none(value: Any) -> float | None:
    return None if value is None else float(value)


def parse_robot_tags(payload: dict[str, Any], log_time_ns: int) -> RobotTagsFrame:
    """One decoded ``/apriltag/robot_tags`` JSON object."""
    cam = payload["camera"]
    roi = payload.get("roi")
    detections = []
    for det in payload.get("detections", []):
        solutions = [
            TagSolution(
                rvec=np.asarray(sol["rvec"], dtype=np.float64).reshape(3),
                tvec=np.asarray(sol["tvec"], dtype=np.float64).reshape(3),
                reprojection_error_px=float(sol["reprojection_error_px"]),
            )
            for sol in det.get("solutions", [])
        ]
        detections.append(
            TagDetection(
                tag_id=int(det["id"]),
                corners=np.asarray(det["corners"], dtype=np.float64).reshape(4, 2),
                decision_margin=_float_or_none(det.get("decision_margin")),
                solutions=solutions,
            )
        )
    return RobotTagsFrame(
        log_time_ns=int(log_time_ns),
        image_stamp_ns=int(payload["image_stamp_ns"]),
        frame_id=str(payload.get("frame_id", "")),
        camera=TagCamera(
            fx=float(cam["fx"]),
            fy=float(cam["fy"]),
            cx=float(cam["cx"]),
            cy=float(cam["cy"]),
            width=int(cam["width"]),
            height=int(cam["height"]),
        ),
        tag_size_m=float(payload["tag_size_m"]),
        roi=None if roi is None else (int(roi[0]), int(roi[1]), int(roi[2]), int(roi[3])),
        detections=detections,
    )


def load_robot_tags(path: Path | str) -> list[RobotTagsFrame]:
    """Every ``/apriltag/robot_tags`` message, in log order."""
    return [
        parse_robot_tags(decode_json(data), log_time_ns)
        for _topic, log_time_ns, data in iter_messages(path, [ROBOT_TAGS_TOPIC])
    ]


# ---------------------------------------------------------------------------
# /robot/esp32_diagnostics
# ---------------------------------------------------------------------------


def _esp32_value(value: Any, dtype: str) -> Any:
    if dtype == "float64":
        return math.nan if value is None else float(value)
    if dtype == "bool":
        return bool(value) if value is not None else False
    return int(value) if value is not None else 0


def esp32_events_frame(events: list[dict[str, Any]]) -> pd.DataFrame:
    """Typed DataFrame from decoded ESP32 event dicts, one row each, in ``ESP32_FIELDS`` order.

    A missing or null float (``vbat`` or ``ibat`` without an INA228 reading) becomes NaN.
    """
    columns: dict[str, list[Any]] = {name: [] for name, _ in ESP32_FIELDS}
    for event in events:
        for name, dtype in ESP32_FIELDS:
            columns[name].append(_esp32_value(event.get(name), dtype))
    return pd.DataFrame(
        {name: pd.Series(columns[name], dtype=dtype) for name, dtype in ESP32_FIELDS}
    )


def load_esp32_diagnostics(path: Path | str) -> pd.DataFrame:
    """Every ``/robot/esp32_diagnostics`` event, in log order, plus the MCAP ``log_time_ns``."""
    events = []
    log_times = []
    for _topic, log_time_ns, data in iter_messages(path, [ESP32_DIAGNOSTICS_TOPIC]):
        events.append(decode_json(data))
        log_times.append(log_time_ns)
    df = esp32_events_frame(events)
    df.insert(0, "log_time_ns", pd.Series(log_times, dtype="int64"))
    return df


# ---------------------------------------------------------------------------
# /diagnostics/opentx_transmitter -> stick channels
# ---------------------------------------------------------------------------


def load_transmitter_channels(path: Path | str) -> pd.DataFrame:
    """Stick channels as the radio reported them: one row per change, columns
    ``stamp_ns, ch0 .. ch15`` (raw CRSF units, 172..1811, center 992).

    The app logs channels only when they change, so hold each row until the next one. The stamp
    is the diagnostics publish tick, which trails the serial read by up to one tick.
    """
    rows = []
    for log_time_ns, status in iter_diagnostic_statuses(path):
        if status["hardware_id"] != TRANSMITTER_HW_ID or status["name"] != "channels":
            continue
        values = status["values"]
        row: dict[str, Any] = {"stamp_ns": int(log_time_ns)}
        for ch in range(NUM_TRANSMITTER_CHANNELS):
            value = values.get(f"values/{ch}")
            row[f"ch{ch}"] = math.nan if value is None else float(value)
        rows.append(row)
    columns = ["stamp_ns"] + [f"ch{ch}" for ch in range(NUM_TRANSMITTER_CHANNELS)]
    return pd.DataFrame(rows, columns=columns)


# ---------------------------------------------------------------------------
# /tf -> camera pose in the field frame
# ---------------------------------------------------------------------------


@dataclass
class FieldFromCamera:
    """Time series of T_field_camera (4x4, maps OpenCV camera points into the field frame)."""

    stamps_ns: np.ndarray  # (N,) int64, ascending
    matrices: np.ndarray  # (N, 4, 4)

    def __len__(self) -> int:
        return int(self.stamps_ns.shape[0])

    def at(self, stamps_ns: np.ndarray) -> np.ndarray:
        """The latest transform at or before each stamp (the first one before the series)."""
        idx = np.searchsorted(self.stamps_ns, np.asarray(stamps_ns, dtype=np.int64), side="right")
        idx = np.clip(idx - 1, 0, len(self) - 1)
        return np.asarray(self.matrices[idx])


def load_field_from_camera(path: Path | str) -> FieldFromCamera:
    """T_field_camera = T(field <- camera_world) @ T(camera_world <- camera), per /tf tick.

    ``field -> camera_world`` is taken from either ``/tf`` or ``/tf_static``, whichever carried
    it most recently; each ``camera_world -> camera`` then yields one sample.
    """
    field_from_world: np.ndarray | None = None
    stamps: list[int] = []
    mats: list[np.ndarray] = []
    for topic, log_time_ns, data in iter_messages(path, [TF_TOPIC, TF_STATIC_TOPIC]):
        transforms = decode_tf_message(data)
        for tr in transforms:
            if tr.key == (FIELD_FRAME, CAMERA_WORLD_FRAME):
                field_from_world = tr.matrix
        if topic != TF_TOPIC or field_from_world is None:
            continue
        for tr in transforms:
            if tr.key == (CAMERA_WORLD_FRAME, CAMERA_FRAME):
                stamps.append(int(log_time_ns))
                mats.append(field_from_world @ tr.matrix)
    if not mats:
        return FieldFromCamera(np.zeros(0, dtype=np.int64), np.zeros((0, 4, 4)))
    order = np.argsort(np.asarray(stamps, dtype=np.int64), kind="stable")
    return FieldFromCamera(
        stamps_ns=np.asarray(stamps, dtype=np.int64)[order], matrices=np.stack(mats)[order]
    )


def rotation_angle(r: np.ndarray) -> float:
    """Angle of a 3x3 rotation in radians."""
    c = (float(np.trace(r)) - 1.0) / 2.0
    return math.acos(min(1.0, max(-1.0, c)))


def field_transform_constancy(series: FieldFromCamera) -> dict[str, float]:
    """How far the camera pose wanders from its median over the session.

    The tripod does not move, so anything above a millimetre or two means a field re-init or a
    bumped tripod, and the session needs a look before its poses are used.
    """
    if len(series) == 0:
        return {"samples": 0, "max_translation_m": math.nan, "max_rotation_deg": math.nan}
    t = series.matrices[:, :3, 3]
    median_t = np.median(t, axis=0)
    ref = series.matrices[len(series) // 2, :3, :3]
    angles = [rotation_angle(ref.T @ m[:3, :3]) for m in series.matrices]
    return {
        "samples": len(series),
        "max_translation_m": float(np.max(np.linalg.norm(t - median_t, axis=1))),
        "max_rotation_deg": float(np.degrees(max(angles))),
    }
