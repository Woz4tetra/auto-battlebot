"""Write and read back the AprilTag calibration recordings in both image formats, then the
overlay the analysis emits from them."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from auto_battlebot.calibration.apriltag import apriltag_mcap as amcap
from auto_battlebot.recording import mcap_io

K = [800.0, 0.0, 320.0, 0.0, 800.0, 240.0, 0.0, 0.0, 1.0]
D = [0.01, -0.02, 0.0, 0.0, 0.0]


def _frame(seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    return rng.integers(0, 255, size=(48, 64, 3), dtype=np.uint8)


@pytest.mark.parametrize("image_format", amcap.IMAGE_FORMATS)
def test_capture_round_trip(tmp_path: Path, image_format: str) -> None:
    path = tmp_path / f"capture_{image_format}.mcap"
    writer = amcap.CaptureWriter(path, image_format=image_format)
    writer.set_metadata({"camera_matrix": K, "dist_coeffs": D, "tag_size": 0.1, "tag_id": 0})
    floor = _frame(1)
    frames = [_frame(2), _frame(3)]
    writer.write_floor_image(10.0, floor)
    for i, frame in enumerate(frames):
        t = 11.0 + i * 0.5
        writer.write_image(t, frame)
        writer.write_channels(t, t - 0.01, [100, -200, 0])
    writer.write_command(11.25, 0.5, -0.25, 250, -125, "step")
    writer.close()

    meta = amcap.read_metadata(path)
    assert meta["image_width"] == 64 and meta["image_height"] == 48
    assert meta["image_format"] == image_format
    assert meta["t"] == 10.0

    floor_frames = amcap.read_floor_frames(path)
    assert len(floor_frames) == 1
    images = list(amcap.iter_images(path, amcap.TOPIC_CAMERA_IMAGE))
    assert [t for t, _ in images] == [11.0, 11.5]
    for (_t, decoded), original in zip(images, frames):
        assert decoded.shape == original.shape
        if image_format == "raw":
            np.testing.assert_array_equal(decoded, original)
            np.testing.assert_array_equal(floor_frames[0], floor)

    channels = amcap.read_channels(path)
    assert set(channels) == {11.0, 11.5}
    assert channels[11.0] == {"sample_t": 10.99, "channels": [100, -200, 0]}

    commands = amcap.read_commands(path)
    assert len(commands) == 1
    assert commands[0]["t"] == 11.25
    assert commands[0]["label"] == "step"
    assert commands[0]["trainer_lin"] == 250


def test_overlay(tmp_path: Path) -> None:
    src = tmp_path / "capture.mcap"
    writer = amcap.CaptureWriter(src, image_format="jpeg")
    writer.set_metadata({"camera_matrix": K, "dist_coeffs": D, "tag_size": 0.1, "tag_id": 0})
    for i in range(3):
        writer.write_image(5.0 + i * 0.1, _frame(i))
    writer.close()
    metadata = amcap.read_metadata(src)

    rows = [
        {"t": 5.0, "visible": True, "x": 0.1, "y": 0.2, "z": 0.03, "yaw": 0.5},
        {"t": 5.1, "visible": False},
        {
            "t": 5.2,
            "visible": True,
            "x": 0.15,
            "y": 0.25,
            "z": 0.03,
            "yaw": 0.6,
            "channels": [1, 2],
            "sample_t": 5.19,
        },
    ]
    r_fc = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=np.float64)
    t_fc = np.array([0.0, 0.0, 1.5])
    out = tmp_path / "overlay.mcap"
    amcap.write_overlay(out, src, metadata, rows, r_fc, t_fc)

    topics = {}
    for topic, _log_time, data in mcap_io.iter_messages(out):
        topics.setdefault(topic, []).append(data)
    assert len(topics[amcap.TOPIC_CAMERA_IMAGE]) == 3
    assert len(topics[amcap.TOPIC_CAMERA_INFO]) == 3
    assert len(topics[amcap.TOPIC_TF_STATIC]) == 1
    assert len(topics[amcap.TOPIC_TF]) == 2
    assert len(topics[amcap.TOPIC_TRANSMITTER]) == 1

    statics = mcap_io.decode_tf_message(topics[amcap.TOPIC_TF_STATIC][0])
    assert [tf.key for tf in statics] == [("world", "field"), ("field", "camera")]
    cam = statics[1].matrix
    # Camera pose in the field frame is the inverse of (r_fc, t_fc).
    np.testing.assert_allclose(cam[:3, :3], r_fc.T, atol=1e-12)
    np.testing.assert_allclose(cam[:3, 3], -r_fc.T @ t_fc, atol=1e-12)

    info = mcap_io.decode_camera_info(topics[amcap.TOPIC_CAMERA_INFO][0])
    np.testing.assert_allclose(info.intrinsics.reshape(-1), K)
    assert np.all(info.distortion == 0.0)

    updates = [mcap_io.decode_scene_update(d) for d in topics[amcap.TOPIC_MARKERS]]
    # trajectory, then per frame: visible, deleted, visible
    assert [e.id for e in updates[0].entities] == ["trajectory_3d/0", "trajectory_floor/0"]
    assert len(updates[0].entities[0].lines[0].points) == 2
    assert {e.id for e in updates[1].entities} == {
        "robot_3d/0",
        "robot_3d/1",
        "robot_floor/0",
        "robot_floor/1",
    }
    assert updates[1].entities[0].cubes[0].pose.position == (0.1, 0.2, 0.03)
    assert {d.id for d in updates[2].deletions} == {
        "robot_3d/0",
        "robot_3d/1",
        "robot_floor/0",
        "robot_floor/1",
    }
    assert updates[3].entities[1].arrows[0].pose.position == (0.15, 0.25, 0.03)
