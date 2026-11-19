"""MCAP layout for the AprilTag ground-truth pipeline: raw camera images in, poses out (offline)."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Iterator

import cv2
import numpy as np

from auto_battlebot.recording import mcap_io, mcap_write

TOPIC_METADATA = "/calibration/metadata"
TOPIC_FLOOR_IMAGE = "/floor/image"
TOPIC_CAMERA_IMAGE = "/camera/image"
TOPIC_TRANSMITTER = "/transmitter/channels"
TOPIC_COMMAND = "/drive/command"

JPEG_QUALITY = 95

METADATA_SCHEMA = {
    "type": "object",
    "properties": {
        "t": {"type": "number", "description": "CLOCK_MONOTONIC seconds at recording start"},
        "camera_matrix": {
            "type": "array",
            "items": {"type": "number"},
            "description": "3x3 K, row-major (9 values)",
        },
        "dist_coeffs": {"type": "array", "items": {"type": "number"}},
        "image_width": {"type": "integer"},
        "image_height": {"type": "integer"},
        "tag_size": {"type": "number", "description": "robot tag edge length (m)"},
        "tag_id": {"type": "integer"},
        "yaw_offset_deg": {"type": "number"},
        "floor": {"type": "object", "description": "floor grid board parameters for the lock"},
        "image_format": {"type": "string", "description": "jpeg (lossy) or raw (lossless bgr8)"},
        "clock": {"type": "string"},
    },
}

CHANNELS_SCHEMA = {
    "type": "object",
    "properties": {
        "sample_t": {
            "type": "number",
            "description": "CLOCK_MONOTONIC seconds the channel snapshot was decoded",
        },
        "channels": {
            "type": "array",
            "items": {"type": "integer"},
            "description": "raw OpenTX channel values (~[-1024, 1024]); stick axes on low chans",
        },
    },
}

COMMAND_SCHEMA = {
    "type": "object",
    "properties": {
        "cmd_lin": {"type": "number", "description": "normalized linear command [-1, 1]"},
        "cmd_ang": {"type": "number", "description": "normalized angular command [-1, 1]"},
        "trainer_lin": {"type": "integer", "description": "trainer linear value sent [-500, 500]"},
        "trainer_ang": {"type": "integer", "description": "trainer angular value sent [-500, 500]"},
        "label": {"type": "string", "description": "excitation protocol phase label"},
    },
}

METADATA_JSON_SCHEMA = ("apriltag_calibration_metadata", json.dumps(METADATA_SCHEMA))
CHANNELS_JSON_SCHEMA = ("transmitter_channels", json.dumps(CHANNELS_SCHEMA))
COMMAND_JSON_SCHEMA = ("drive_command", json.dumps(COMMAND_SCHEMA))

IMAGE_FORMATS = ("jpeg", "raw")


def _ns(t: float) -> int:
    """Monotonic seconds to integer nanoseconds for the MCAP log_time."""
    return int(round(t * 1e9))


def _decode_image(data: bytes) -> np.ndarray:
    """Decode an image message by its schema: RawImage is raw bgr8, else JPEG."""
    schema = getattr(data, "schema_name", "")
    if schema == "foxglove.RawImage":
        _stamp, _frame_id, frame, encoding = mcap_io.decode_raw_image_bytes(data)
        if encoding not in ("bgr8", ""):
            raise ValueError(f"Unsupported raw image encoding {encoding!r}")
        return frame
    return mcap_io.decode_compressed_image(data).image


class CaptureWriter:
    """Writes the calibration metadata and the floor / camera image streams to an MCAP file."""

    def __init__(self, path: Path, image_format: str = "jpeg") -> None:
        if image_format not in IMAGE_FORMATS:
            raise ValueError(f"image_format must be one of {IMAGE_FORMATS}, got {image_format!r}")
        self._image_format = image_format
        compression = "zstd" if image_format == "jpeg" else None
        self._writer = mcap_write.McapWriter(path, allow_overwrite=True, compression=compression)
        self._meta: dict | None = None
        self._meta_written = False

    def set_metadata(self, meta: dict) -> None:
        """Stash the metadata; image size is filled from the first frame."""
        self._meta = dict(meta)

    def _ensure_metadata(self, t: float, frame: np.ndarray) -> None:
        if self._meta_written:
            return
        if self._meta is None:
            raise RuntimeError("set_metadata() must be called before recording frames")
        h, w = frame.shape[:2]
        payload = {
            **self._meta,
            "t": t,
            "image_width": int(w),
            "image_height": int(h),
            "image_format": self._image_format,
        }
        self._writer.log_json(TOPIC_METADATA, payload, _ns(t), schema=METADATA_JSON_SCHEMA)
        self._meta_written = True

    def _image_message(self, t: float, frame: np.ndarray) -> Any:
        if self._image_format == "raw":
            return mcap_write.raw_image(_ns(t), "camera", frame)
        ok, enc = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            raise RuntimeError("cv2.imencode failed to JPEG-encode a frame")
        return mcap_write.compressed_image(_ns(t), "camera", enc.tobytes())

    def _write_image(self, topic: str, t: float, frame: np.ndarray) -> None:
        self._ensure_metadata(t, frame)
        self._writer.log(topic, self._image_message(t, frame), _ns(t))

    def write_floor_image(self, t: float, frame: np.ndarray) -> None:
        self._write_image(TOPIC_FLOOR_IMAGE, t, frame)

    def write_image(self, t: float, frame: np.ndarray) -> None:
        self._write_image(TOPIC_CAMERA_IMAGE, t, frame)

    def write_channels(self, image_t: float, sample_t: float, channels: list[int]) -> None:
        """Record the driver's stick axes for one frame at image time `image_t`."""
        payload = {"sample_t": sample_t, "channels": [int(c) for c in channels]}
        self._writer.log_json(TOPIC_TRANSMITTER, payload, _ns(image_t), schema=CHANNELS_JSON_SCHEMA)

    def write_command(
        self,
        t: float,
        cmd_lin: float,
        cmd_ang: float,
        trainer_lin: int,
        trainer_ang: int,
        label: str,
    ) -> None:
        """Record one scripted excitation command (--drive) at send time `t`."""
        payload = {
            "cmd_lin": float(cmd_lin),
            "cmd_ang": float(cmd_ang),
            "trainer_lin": int(trainer_lin),
            "trainer_ang": int(trainer_ang),
            "label": str(label),
        }
        self._writer.log_json(TOPIC_COMMAND, payload, _ns(t), schema=COMMAND_JSON_SCHEMA)

    def close(self) -> None:
        self._writer.close()


def read_metadata(path: Path) -> dict:
    """Return the calibration metadata dict from a recording."""
    for _topic, _log_time, data in mcap_io.iter_messages(path, [TOPIC_METADATA]):
        metadata: dict = mcap_io.decode_json(data)
        return metadata
    raise SystemExit(f"{path}: no {TOPIC_METADATA} message; not an apriltag_track recording.")


def iter_images(path: Path, topic: str) -> Iterator[tuple[float, np.ndarray]]:
    """Yield (t_seconds, bgr_frame) for each image on topic, in recorded order."""
    for _topic, log_time, data in mcap_io.iter_messages(path, [topic]):
        yield log_time / 1e9, _decode_image(data)


def read_floor_frames(path: Path) -> list[np.ndarray]:
    """Load every floor-lock frame into memory."""
    return [frame for _t, frame in iter_images(path, TOPIC_FLOOR_IMAGE)]


def read_channels(path: Path) -> dict[float, dict]:
    """Read the per-frame transmitter stick axes, keyed by their frame time."""
    out: dict[float, dict] = {}
    for _topic, log_time, data in mcap_io.iter_messages(path, [TOPIC_TRANSMITTER]):
        out[log_time / 1e9] = mcap_io.decode_json(data)
    return out


def read_commands(path: Path) -> list[dict]:
    """Read the scripted excitation commands, sorted by send time."""
    out: list[dict] = []
    for _topic, log_time, data in mcap_io.iter_messages(path, [TOPIC_COMMAND]):
        payload = mcap_io.decode_json(data)
        payload["t"] = log_time / 1e9
        out.append(payload)
    out.sort(key=lambda r: r["t"])
    return out


TOPIC_CAMERA_INFO = "/camera/camera_info"
TOPIC_TF = "/tf"
TOPIC_TF_STATIC = "/tf_static"
TOPIC_MARKERS = "/overlay/markers"

WORLD_FRAME = "world"
FIELD_FRAME = "field"
CAMERA_FRAME = "camera"
ROBOT_FRAME = "robot"


def _quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """Quaternion (x, y, z, w) for a rotation of `yaw` radians about z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _quat_from_matrix(r: np.ndarray) -> tuple[float, float, float, float]:
    """Quaternion (x, y, z, w) from a 3x3 rotation matrix."""
    return mcap_write.rotation_to_quaternion(r)


def _line_entity(
    stamp_ns: int, entity_id: str, points: list[tuple[float, float, float]], color: tuple
) -> mcap_io.SceneEntity:
    return mcap_io.SceneEntity(
        id=entity_id,
        frame_id=FIELD_FRAME,
        stamp_ns=stamp_ns,
        frame_locked=True,
        lines=[
            mcap_io.LinePrimitive(
                type="LINE_STRIP",
                pose=mcap_io.Pose(),
                thickness=0.01,
                scale_invariant=False,
                points=list(points),
                color=mcap_io.Color(*color),
            )
        ],
    )


class OverlayWriter:
    """Writes a Foxglove overlay MCAP: frames, calibration, TF, pose markers."""

    _BODY = (0.16, 0.16, 0.05)
    _ARROW_LENGTH_M = 0.22
    _ARROW_SHAFT_DIAMETER_M = 0.02
    _ARROW_HEAD_DIAMETER_M = 0.04
    _BODY_COLOR = (0.10, 0.80, 0.30, 0.6)
    _ARROW_COLOR = (1.00, 0.55, 0.05, 0.95)
    _TRAJ_COLOR = (0.20, 0.55, 1.00, 0.9)
    _BODY_FLOOR_COLOR = (0.45, 0.45, 0.50, 0.35)
    _ARROW_FLOOR_COLOR = (0.70, 0.55, 0.35, 0.5)
    _TRAJ_FLOOR_COLOR = (0.40, 0.45, 0.55, 0.55)

    def __init__(self, path: Path, k: list[float], d: list[float], width: int, height: int) -> None:
        self._k = [float(v) for v in k]
        self._w = int(width)
        self._h = int(height)
        k3 = np.asarray(k, dtype=np.float64).reshape(3, 3)
        dv = np.asarray(d, dtype=np.float64).reshape(-1)
        self._map1, self._map2 = cv2.initUndistortRectifyMap(
            k3, dv, None, k3, (self._w, self._h), cv2.CV_16SC2
        )
        self._writer = mcap_write.McapWriter(path, allow_overwrite=True, compression="zstd")

    def write_static_tf(self, t: float, r_fc: np.ndarray, t_fc: np.ndarray) -> None:
        """Publish the fixed world->field and field->camera transforms on /tf_static."""
        r_fc = np.asarray(r_fc, dtype=np.float64).reshape(3, 3)
        t_fc = np.asarray(t_fc, dtype=np.float64).reshape(3)
        r_cf = r_fc.T
        cam_in_field = -r_cf @ t_fc
        quat = _quat_from_matrix(r_cf)
        flip = (0.0, 0.0, 0.0, 1.0) if cam_in_field[2] >= 0.0 else (1.0, 0.0, 0.0, 0.0)
        stamp_ns = _ns(t)
        self._writer.log(
            TOPIC_TF_STATIC,
            mcap_write.fg.FrameTransforms(
                transforms=[
                    mcap_write.frame_transform_from_parts(
                        stamp_ns, WORLD_FRAME, FIELD_FRAME, (0.0, 0.0, 0.0), flip
                    ),
                    mcap_write.frame_transform_from_parts(
                        stamp_ns, FIELD_FRAME, CAMERA_FRAME, cam_in_field.tolist(), quat
                    ),
                ]
            ),
            stamp_ns,
        )

    def write_trajectory(self, t: float, rows: list[dict]) -> None:
        """Draw the full solved path as 3d and floor line strips."""
        pts_3d = [
            (float(r["x"]), float(r["y"]), float(r.get("z") or 0.0)) for r in rows if r["visible"]
        ]
        if len(pts_3d) < 2:
            return
        pts_floor = [(p[0], p[1], 0.0) for p in pts_3d]
        stamp_ns = _ns(t)
        update = mcap_io.SceneUpdate(
            entities=[
                _line_entity(stamp_ns, "trajectory_3d/0", pts_3d, self._TRAJ_COLOR),
                _line_entity(stamp_ns, "trajectory_floor/0", pts_floor, self._TRAJ_FLOOR_COLOR),
            ]
        )
        self._writer.log(TOPIC_MARKERS, mcap_write.scene_update(update), stamp_ns)

    def write_image(self, t: float, frame: np.ndarray) -> None:
        """Undistort a source camera frame and write it as JPEG so the pinhole overlay aligns."""
        rectified = cv2.remap(frame, self._map1, self._map2, cv2.INTER_LINEAR)
        ok, encoded = cv2.imencode(".jpg", rectified, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            raise RuntimeError("cv2.imencode failed for overlay frame")
        stamp_ns = _ns(t)
        self._writer.log(
            TOPIC_CAMERA_IMAGE,
            mcap_write.compressed_image(stamp_ns, CAMERA_FRAME, encoded.tobytes()),
            stamp_ns,
        )

    def write_camera_info(self, t: float) -> None:
        stamp_ns = _ns(t)
        self._writer.log(
            TOPIC_CAMERA_INFO,
            mcap_write.camera_calibration(
                stamp_ns, CAMERA_FRAME, self._w, self._h, self._k, [0.0] * 5
            ),
            stamp_ns,
        )

    def _robot_entities(
        self,
        stamp_ns: int,
        ns: str,
        x: float,
        y: float,
        z: float,
        quat: tuple,
        body_color: tuple,
        arrow_color: tuple,
    ) -> list[mcap_io.SceneEntity]:
        """Body cube and heading arrow at (x, y, z) with the given yaw."""
        pose = mcap_io.Pose(position=(x, y, z), orientation=tuple(quat))
        length = self._ARROW_LENGTH_M
        shaft_diameter = self._ARROW_SHAFT_DIAMETER_M
        head_diameter = self._ARROW_HEAD_DIAMETER_M
        return [
            mcap_io.SceneEntity(
                id=f"{ns}/0",
                frame_id=FIELD_FRAME,
                stamp_ns=stamp_ns,
                frame_locked=True,
                cubes=[
                    mcap_io.CubePrimitive(
                        pose=pose, size=self._BODY, color=mcap_io.Color(*body_color)
                    )
                ],
            ),
            mcap_io.SceneEntity(
                id=f"{ns}/1",
                frame_id=FIELD_FRAME,
                stamp_ns=stamp_ns,
                frame_locked=True,
                arrows=[
                    mcap_io.ArrowPrimitive(
                        pose=pose,
                        shaft_length=0.77 * length,
                        shaft_diameter=shaft_diameter,
                        head_length=0.23 * length,
                        head_diameter=head_diameter,
                        color=mcap_io.Color(*arrow_color),
                    )
                ],
            ),
        ]

    def write_pose(self, t: float, row: dict) -> None:
        """field->robot TF and robot markers for one frame."""
        stamp_ns = _ns(t)
        if row["visible"]:
            x, y, yaw = float(row["x"]), float(row["y"]), float(row["yaw"])
            z = float(row.get("z") or 0.0)
            quat = _quat_from_yaw(yaw)
            self._writer.log(
                TOPIC_TF,
                mcap_write.fg.FrameTransforms(
                    transforms=[
                        mcap_write.frame_transform_from_parts(
                            stamp_ns, FIELD_FRAME, ROBOT_FRAME, (x, y, z), quat
                        )
                    ]
                ),
                stamp_ns,
            )
            entities = self._robot_entities(
                stamp_ns, "robot_3d", x, y, z, quat, self._BODY_COLOR, self._ARROW_COLOR
            )
            entities += self._robot_entities(
                stamp_ns,
                "robot_floor",
                x,
                y,
                0.0,
                quat,
                self._BODY_FLOOR_COLOR,
                self._ARROW_FLOOR_COLOR,
            )
            update = mcap_io.SceneUpdate(entities=entities)
        else:
            update = mcap_io.SceneUpdate(
                deletions=[
                    mcap_io.SceneEntityDeletion(
                        type="MATCHING_ID", id=f"{ns}/{i}", stamp_ns=stamp_ns
                    )
                    for ns in ("robot_3d", "robot_floor")
                    for i in (0, 1)
                ]
            )
        self._writer.log(TOPIC_MARKERS, mcap_write.scene_update(update), stamp_ns)

    def write_channels(self, t: float, sample_t: float, channels: list[int]) -> None:
        """Copy one frame's transmitter stick axes into the overlay."""
        payload = {"sample_t": sample_t, "channels": [int(c) for c in channels]}
        self._writer.log_json(TOPIC_TRANSMITTER, payload, _ns(t), schema=CHANNELS_JSON_SCHEMA)

    def close(self) -> None:
        self._writer.close()


def write_overlay(
    out_path: Path,
    src_path: Path,
    metadata: dict,
    rows: list[dict],
    r_fc: np.ndarray,
    t_fc: np.ndarray,
) -> None:
    """Write a Foxglove overlay MCAP of the source frames, solved poses, and TFs."""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    writer = OverlayWriter(
        out_path,
        metadata["camera_matrix"],
        metadata["dist_coeffs"],
        int(metadata["image_width"]),
        int(metadata["image_height"]),
    )
    if rows:
        writer.write_static_tf(rows[0]["t"], r_fc, t_fc)
        writer.write_trajectory(rows[0]["t"], rows)
    for row, (_t, frame) in zip(rows, iter_images(src_path, TOPIC_CAMERA_IMAGE), strict=True):
        ts = row["t"]
        writer.write_image(ts, frame)
        writer.write_camera_info(ts)
        writer.write_pose(ts, row)
        if "channels" in row:
            writer.write_channels(ts, row["sample_t"], row["channels"])
    writer.close()
