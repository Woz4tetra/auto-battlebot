"""Where the sysid monitor's messages come from: a recording, or the live relay.

Both yield ``(topic, log_time_ns, MessageBytes)`` in log order. ``Dispatcher`` decodes them
into the ``CoverageSession`` inputs.
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Iterator

import numpy as np

from auto_battlebot.calibration.sysid.coverage import CoverageSession
from auto_battlebot.calibration.sysid.telemetry import (
    ESP32_DIAGNOSTICS_TOPIC,
    ESPNOW_TELEMETRY_TOPIC,
    telemetry_topic,
)
from auto_battlebot.recording.foxglove_ws import FoxgloveSubscriber
from auto_battlebot.recording.mcap_io import (
    MessageBytes,
    decode_json,
    decode_tf_message,
    iter_messages,
)
from auto_battlebot.recording.sysid_io import (
    CAMERA_FRAME,
    CAMERA_WORLD_FRAME,
    FIELD_FRAME,
    ROBOT_TAGS_TOPIC,
    TF_STATIC_TOPIC,
    TF_TOPIC,
    parse_robot_tags,
)

LIVE_TOPICS = [ESPNOW_TELEMETRY_TOPIC, ROBOT_TAGS_TOPIC, TF_TOPIC, TF_STATIC_TOPIC]

Message = tuple[str, int, MessageBytes]


def replay(path: Path | str, speed: float = 0.0) -> Iterator[Message]:
    """A recording's sysid topics in log order, paced at ``speed`` times real time (0: as fast
    as the session takes them)."""
    topic = telemetry_topic(path) or ESPNOW_TELEMETRY_TOPIC
    start_wall = time.monotonic()
    first: int | None = None
    for msg in iter_messages(path, [topic, ROBOT_TAGS_TOPIC, TF_TOPIC, TF_STATIC_TOPIC]):
        if speed > 0:
            first = msg[1] if first is None else first
            due = start_wall + (msg[1] - first) / 1e9 / speed
            wait = due - time.monotonic()
            if wait > 0:
                time.sleep(wait)
        yield msg


def live(url: str) -> tuple[FoxgloveSubscriber, Iterator[Message]]:
    sub = FoxgloveSubscriber(url, LIVE_TOPICS)
    return sub, sub.messages()


class Dispatcher:
    """Decodes messages into the session. Keeps the two /tf links to compose the camera pose."""

    def __init__(self, session: CoverageSession) -> None:
        self.session = session
        self._field_from_world: np.ndarray | None = None
        self._world_from_camera: np.ndarray | None = None

    def __call__(self, topic: str, log_time_ns: int, data: MessageBytes) -> None:
        if topic in (ESPNOW_TELEMETRY_TOPIC, ESP32_DIAGNOSTICS_TOPIC):
            self.session.feed_telemetry(decode_json(data), log_time_ns)
        elif topic == ROBOT_TAGS_TOPIC:
            self.session.feed_tags(parse_robot_tags(decode_json(data), log_time_ns))
        elif topic in (TF_TOPIC, TF_STATIC_TOPIC):
            for tr in decode_tf_message(data):
                if tr.key == (FIELD_FRAME, CAMERA_WORLD_FRAME):
                    self._field_from_world = tr.matrix
                elif tr.key == (CAMERA_WORLD_FRAME, CAMERA_FRAME):
                    self._world_from_camera = tr.matrix
            if self._field_from_world is not None and self._world_from_camera is not None:
                self.session.feed_field(self._field_from_world @ self._world_from_camera)
