"""Robot telemetry for the sysid monitor: the frame schema, the plant inputs, the robot clock.

``/robot/espnow_telemetry`` is the source. The column list is read from the JSON schema the
recorder writes (``mcap_write.ESPNOW_TELEMETRY_JSON_SCHEMA``), so a field added there shows up
here with no edit and a renamed one fails loudly where it is used instead of reading as NaN.
Integers load as nullable ``Int64`` (``telemetry_seq`` is <NA> in recordings older than it),
numbers as float with NaN for null, booleans as bool.

Recordings without ESP-NOW telemetry fall back to ``/robot/esp32_diagnostics``, the WiFi stream
with the same drive fields. The columns it lacks (``espnow_active``, ``autonomy_enabled``,
``telemetry_seq``) read as False or <NA>.

Plant inputs:

- fit input: ``left_cmd``/``right_cmd``, what the ESCs got after the deadband and the yaw loop,
  through ``firmware_mixer.motor_to_plant``.
- prediction input and stick display: ``a_percent``/``b_percent``, the arbitrated sticks before
  the mixer. ``throttle = -a / 100`` (forward positive) and ``steer = -b / 100`` (left positive),
  the plant's sign convention.
"""

from __future__ import annotations

import json
import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import pandas as pd

from auto_battlebot.recording.esp32_clock import ClockSegment, fit_robot_clock
from auto_battlebot.recording.mcap_io import decode_json, iter_messages, recording_topics
from auto_battlebot.recording.mcap_write import ESPNOW_TELEMETRY_JSON_SCHEMA

ESPNOW_TELEMETRY_TOPIC = "/robot/espnow_telemetry"
ESP32_DIAGNOSTICS_TOPIC = "/robot/esp32_diagnostics"

# crsf_bridge::three_state_switch: the flip switch position that turns the yaw loop on.
FLIP_SWITCH_DOWN = 0


def _schema_columns() -> tuple[tuple[str, str], ...]:
    props = json.loads(ESPNOW_TELEMETRY_JSON_SCHEMA)["properties"]
    columns = []
    for name, spec in props.items():
        kinds = spec["type"] if isinstance(spec["type"], list) else [spec["type"]]
        if "boolean" in kinds:
            dtype = "bool"
        elif "number" in kinds:
            dtype = "float64"
        elif "integer" in kinds:
            dtype = "Int64"
        else:
            raise ValueError(f"telemetry field {name} has unsupported type {spec['type']}")
        columns.append((name, dtype))
    return tuple(columns)


TELEMETRY_COLUMNS = _schema_columns()


def _value(value: Any, dtype: str) -> Any:
    if dtype == "float64":
        return math.nan if value is None else float(value)
    if dtype == "bool":
        return bool(value) if value is not None else False
    return None if value is None else int(value)


def telemetry_frame(events: Iterable[dict[str, Any]]) -> pd.DataFrame:
    """Typed DataFrame from decoded telemetry dicts, one row each, in schema order."""
    columns: dict[str, list[Any]] = {name: [] for name, _ in TELEMETRY_COLUMNS}
    for event in events:
        for name, dtype in TELEMETRY_COLUMNS:
            columns[name].append(_value(event.get(name), dtype))
    return pd.DataFrame(
        {name: pd.Series(columns[name], dtype=dtype) for name, dtype in TELEMETRY_COLUMNS}
    )


def telemetry_topic(path: Path | str) -> str | None:
    """The telemetry topic a recording carries, ESP-NOW first."""
    topics = recording_topics(path)
    for topic in (ESPNOW_TELEMETRY_TOPIC, ESP32_DIAGNOSTICS_TOPIC):
        if topic in topics:
            return topic
    return None


def load_telemetry(path: Path | str) -> pd.DataFrame:
    """Every telemetry frame in log order, plus the MCAP ``log_time_ns``."""
    topic = telemetry_topic(path)
    events: list[dict[str, Any]] = []
    log_times: list[int] = []
    if topic is not None:
        for _topic, log_time_ns, data in iter_messages(path, [topic]):
            events.append(decode_json(data))
            log_times.append(log_time_ns)
    df = telemetry_frame(events)
    df.insert(0, "log_time_ns", pd.Series(log_times, dtype="int64"))
    return df


def sticks(a_percent: Any, b_percent: Any) -> tuple[np.ndarray, np.ndarray]:
    """(throttle, steer) in [-1, 1], forward and left positive."""
    return (
        -np.asarray(a_percent, dtype=float) / 100.0,
        -np.asarray(b_percent, dtype=float) / 100.0,
    )


# ---------------------------------------------------------------------------
# Live robot clock
# ---------------------------------------------------------------------------


@dataclass
class ClockStatus:
    offset_ms: float
    drift_ppm: float
    residual_p50_ms: float
    residual_p95_ms: float
    samples: int


class LiveClock:
    """Robot ``timestamp_ms`` to host ns, refit over a sliding window.

    Same lower-envelope line as ``esp32_clock.fit_robot_clock``, over the last ``window_s`` of
    frames and refit every ``refit_s``. A reboot (robot time stepping back) drops the buffer.
    Until the first fit, frames map through their receive time minus the smallest delay seen.
    """

    def __init__(self, window_s: float = 60.0, refit_s: float = 5.0) -> None:
        self.window_ms = int(window_s * 1000)
        self.refit_ms = int(refit_s * 1000)
        self._robot: deque[int] = deque()
        self._host: deque[int] = deque()
        self._segment: ClockSegment | None = None
        self._fit_at_ms: int | None = None
        self._min_gap_ns: int | None = None
        self.status: ClockStatus | None = None

    def push(self, robot_ms: int, host_ns: int) -> None:
        if self._robot and robot_ms < self._robot[-1] - 1000:
            self._robot.clear()
            self._host.clear()
            self._segment = None
            self._fit_at_ms = None
            self._min_gap_ns = None
        self._robot.append(int(robot_ms))
        self._host.append(int(host_ns))
        gap = int(host_ns) - int(robot_ms) * 1_000_000
        self._min_gap_ns = gap if self._min_gap_ns is None else min(self._min_gap_ns, gap)
        while self._robot and self._robot[0] < robot_ms - self.window_ms:
            self._robot.popleft()
            self._host.popleft()
        if len(self._robot) >= 50 and (
            self._fit_at_ms is None or robot_ms - self._fit_at_ms >= self.refit_ms
        ):
            self._refit()
            self._fit_at_ms = robot_ms

    def _refit(self) -> None:
        robot = np.fromiter(self._robot, dtype=np.int64)
        host = np.fromiter(self._host, dtype=np.int64)
        fit = fit_robot_clock(robot, host)
        self._segment = fit.segments[-1]
        res = fit.residual_ms
        self.status = ClockStatus(
            offset_ms=self._segment.offset_s * 1e3,
            drift_ppm=self._segment.drift_ppm,
            residual_p50_ms=float(np.percentile(res, 50)),
            residual_p95_ms=float(np.percentile(res, 95)),
            samples=int(robot.size),
        )

    def to_host_ns(self, robot_ms: Any) -> np.ndarray:
        r = np.asarray(robot_ms, dtype=np.int64)
        if self._segment is not None:
            return self._segment.to_host_ns(r)
        if self._min_gap_ns is None:
            raise ValueError("no telemetry pushed yet")
        out: np.ndarray = r * 1_000_000 + self._min_gap_ns
        return out
