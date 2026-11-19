from __future__ import annotations

import json
import subprocess
from pathlib import Path

import cv2
import numpy as np
import pytest

from auto_battlebot.recording import diag_io, mcap_io, mcap_write

STAMP = 1788011445.339499712
STAMP_NS = mcap_io.stamp_to_ns(STAMP)
LOG_TIME = 1_788_011_445_400_000_000
IMAGE_STAMP_NS_ABOVE_2_POW_53 = 1788011445339499712


def _jpeg() -> bytes:
    image = np.zeros((8, 12, 3), dtype=np.uint8)
    image[:, :, 2] = 200
    ok, encoded = cv2.imencode(".jpg", image)
    assert ok
    return encoded.tobytes()


@pytest.fixture(scope="module")
def recording(tmp_path_factory: pytest.TempPathFactory) -> Path:
    path = tmp_path_factory.mktemp("mcap") / "contract.mcap"
    jpeg = _jpeg()
    with mcap_write.McapWriter(path, active_profile="unit_test") as writer:
        writer.log("/camera/image", mcap_write.compressed_image(STAMP_NS, "camera", jpeg), LOG_TIME)
        writer.log(
            "/camera/camera_info",
            mcap_write.camera_calibration(
                STAMP_NS, "camera", 12, 8, [500, 0, 6, 0, 500, 4, 0, 0, 1], [0.1, -0.2, 0, 0, 0]
            ),
            LOG_TIME,
        )
        writer.log_json(
            "/camera/frame_meta",
            {"image_stamp_ns": str(IMAGE_STAMP_NS_ABOVE_2_POW_53), "video_frame_index": 42},
            LOG_TIME,
        )
        matrix = np.eye(4)
        matrix[:3, :3] = mcap_io.quaternion_to_rotation(0.0, 0.0, np.sin(0.3), np.cos(0.3))
        matrix[:3, 3] = (1.0, 2.0, 3.0)
        writer.log(
            "/tf",
            mcap_write.fg.FrameTransforms(
                transforms=[mcap_write.frame_transform(STAMP_NS, "camera_world", "camera", matrix)]
            ),
            LOG_TIME,
        )
        writer.log(
            "/field_mask", mcap_write.compressed_image(STAMP_NS, "camera_world", jpeg), LOG_TIME
        )
        writer.log(
            "/field_mask/camera_info",
            mcap_write.camera_calibration(STAMP_NS, "camera_world", 12, 8, np.eye(3), []),
            LOG_TIME,
        )
        border = mcap_io.SceneEntity(
            id="field/0",
            frame_id="camera",
            stamp_ns=STAMP_NS,
            lines=[
                mcap_io.LinePrimitive(
                    type=mcap_io.LineType.LINE_STRIP,
                    pose=mcap_io.Pose(),
                    thickness=0.01,
                    scale_invariant=False,
                    points=[(0, 0, 0), (2, 0, 0), (2, 3, 0), (0, 3, 0), (0, 0, 0)],
                    color=mcap_io.Color(0, 1, 0, 1),
                )
            ],
        )
        writer.log(
            "/field_markers",
            mcap_write.scene_update(mcap_io.SceneUpdate(entities=[border])),
            LOG_TIME,
        )
        points = np.array([[0.5, 0.25, 0.125], [1.0, 2.0, -3.0]], dtype=np.float32)
        writer.log(
            "/field_points", mcap_write.point_cloud_xyz(STAMP_NS, "camera", points), LOG_TIME
        )
        robot = mcap_io.SceneEntity(
            id="robot_bounds/4",
            frame_id="field",
            stamp_ns=STAMP_NS,
            lifetime_ns=100_000_000,
            cubes=[
                mcap_io.CubePrimitive(
                    pose=mcap_io.Pose(position=(0.4, -0.2, 0.05)),
                    size=(0.3, 0.3, 0.1),
                    color=mcap_io.Color(1, 0, 0, 0.7),
                )
            ],
        )
        label = mcap_io.SceneEntity(
            id="robot_labels/4",
            frame_id="field",
            stamp_ns=STAMP_NS,
            texts=[
                mcap_io.TextPrimitive(
                    pose=mcap_io.Pose(position=(0.4, -0.2, 0.2)),
                    billboard=True,
                    font_size=0.1,
                    scale_invariant=False,
                    color=mcap_io.Color(1, 1, 1, 1),
                    text="our_robot_1 (mr_stabs_mk2)",
                )
            ],
        )
        deletion = mcap_io.SceneEntityDeletion(
            type=mcap_io.DeletionType.MATCHING_ID, id="nav_target/0", stamp_ns=STAMP_NS
        )
        writer.log(
            "/robot_markers",
            mcap_write.scene_update(
                mcap_io.SceneUpdate(entities=[robot, label], deletions=[deletion])
            ),
            LOG_TIME,
        )
        writer.log_json(
            "/blob_detections",
            {
                "stamp": STAMP,
                "w": 1280,
                "h": 720,
                "dets": [
                    {
                        "x1": 10.0,
                        "y1": 20.0,
                        "x2": 110.0,
                        "y2": 120.0,
                        "conf": 0.9123,
                        "class_id": 0,
                        "label": "mr_stabs_mk1",
                        "kps": [[55.0, 70.0, 0.98]],
                    }
                ],
            },
            LOG_TIME,
        )
        writer.log_diagnostics(
            "pursuit_nav",
            {
                "pursuit_nav": {
                    "level": 0,
                    "message": "",
                    "values": {"distance": 0.75, "facing_target": 1, "mode": "chase"},
                }
            },
            LOG_TIME,
        )
        writer.log_diagnostics(
            "runner",
            {
                "navigation": {"level": 0, "message": "", "values": {"using_previous_robots": 0}},
                "tick": {"level": 1, "message": "slow", "values": {"elapsed_ms": 12.5}},
            },
            LOG_TIME + 1,
        )
        writer.log(
            "/log",
            mcap_write.log_message(
                mcap_io.LogMessage(
                    STAMP_NS, mcap_io.LogLevel.WARN, "auto_battlebot", "hello", "main.cpp", 7
                )
            ),
            LOG_TIME,
        )
    return path


def _one(path: Path, topic: str) -> mcap_io.MessageBytes:
    messages = [data for _t, _ts, data in mcap_io.iter_messages(path, [topic])]
    assert len(messages) == 1, topic
    return messages[0]


def test_metadata(recording: Path) -> None:
    assert mcap_io.read_active_profile(recording) == "unit_test"


def test_compressed_image(recording: Path) -> None:
    data = _one(recording, "/camera/image")
    assert data.encoding == "protobuf"
    assert mcap_io.decode_image_stamp_ns(data) == STAMP_NS
    image = mcap_io.decode_compressed_image(data)
    assert image.frame_id == "camera"
    assert image.format == "jpeg"
    assert image.image.shape == (8, 12, 3)
    assert image.image[..., 2].mean() > 150


def test_camera_info(recording: Path) -> None:
    info = mcap_io.decode_camera_info(_one(recording, "/camera/camera_info"))
    assert (info.width, info.height) == (12, 8)
    assert info.distortion_model == "plumb_bob"
    np.testing.assert_allclose(info.intrinsics, [[500, 0, 6], [0, 500, 4], [0, 0, 1]])
    np.testing.assert_allclose(info.distortion, [0.1, -0.2, 0, 0, 0])
    np.testing.assert_allclose(info.projection[:, :3], info.intrinsics)
    np.testing.assert_allclose(info.rectification, np.eye(3))
    assert (
        mcap_io.decode_camera_info(_one(recording, "/field_mask/camera_info")).distortion.size == 0
    )


def test_frame_meta(recording: Path) -> None:
    data = _one(recording, "/camera/frame_meta")
    meta = mcap_io.decode_frame_meta(data)
    assert meta.image_stamp_ns == IMAGE_STAMP_NS_ABOVE_2_POW_53
    assert meta.video_frame_index == 42
    assert json.loads(mcap_io.decode_string(data))["image_stamp_ns"] == str(
        IMAGE_STAMP_NS_ABOVE_2_POW_53
    )
    bare = json.dumps({"image_stamp_ns": IMAGE_STAMP_NS_ABOVE_2_POW_53, "video_frame_index": -1})
    tagged = mcap_io.MessageBytes(bare.encode(), "json", "", "/camera/frame_meta")
    assert mcap_io.decode_frame_meta(tagged).image_stamp_ns == IMAGE_STAMP_NS_ABOVE_2_POW_53


def test_tf(recording: Path) -> None:
    transforms = mcap_io.decode_tf_message(_one(recording, "/tf"))
    assert len(transforms) == 1
    tf = transforms[0]
    assert tf.key == ("camera_world", "camera")
    assert tf.stamp_ns == STAMP_NS
    expected = np.eye(4)
    expected[:3, :3] = mcap_io.quaternion_to_rotation(0.0, 0.0, np.sin(0.3), np.cos(0.3))
    expected[:3, 3] = (1.0, 2.0, 3.0)
    np.testing.assert_allclose(tf.parent_from_child, expected, atol=1e-12)


def test_field_markers_and_points(recording: Path) -> None:
    update = mcap_io.decode_scene_update(_one(recording, "/field_markers"))
    assert [e.id for e in update.entities] == ["field/0"]
    border = update.entities[0]
    assert border.namespace == "field" and border.index == 0
    assert border.lines[0].type == "LINE_STRIP"
    assert border.lines[0].points[:4] == [(0, 0, 0), (2, 0, 0), (2, 3, 0), (0, 3, 0)]
    assert border.lines[0].color.g == 1.0
    assert diag_io.load_field_size(recording) == (2.0, 3.0)

    cloud = mcap_io.decode_point_cloud(_one(recording, "/field_points"))
    assert cloud.dtype == np.float32
    np.testing.assert_array_equal(cloud, [[0.5, 0.25, 0.125], [1.0, 2.0, -3.0]])


def test_robot_markers(recording: Path) -> None:
    update = mcap_io.decode_scene_update(_one(recording, "/robot_markers"))
    assert [e.id for e in update.entities] == ["robot_bounds/4", "robot_labels/4"]
    robot = update.entities[0]
    assert robot.lifetime_ns == 100_000_000
    assert robot.cubes[0].pose.position == (0.4, -0.2, 0.05)
    assert robot.cubes[0].size == (0.3, 0.3, 0.1)
    assert robot.cubes[0].color.a == pytest.approx(0.7)
    assert update.entities[1].texts[0].text == "our_robot_1 (mr_stabs_mk2)"
    assert update.entities[1].texts[0].billboard is True
    assert [(d.type, d.id) for d in update.deletions] == [("MATCHING_ID", "nav_target/0")]

    tracks = diag_io.load_robot_tracks(recording)
    assert tracks.iloc[0]["our_present"] == 1
    positions = diag_io.load_robot_positions(recording)
    assert positions.iloc[0]["frame"] == "OUR_ROBOT_1"
    assert positions.iloc[0]["x"] == pytest.approx(0.4)


def test_detections(recording: Path) -> None:
    dets = mcap_io.read_detections(recording, "/blob_detections")
    assert len(dets) == 1
    assert dets[0].stamp_ns == round(STAMP * 1e9)
    assert (dets[0].image_width, dets[0].image_height) == (1280, 720)
    det = dets[0].detections[0]
    assert (det.x1, det.y1, det.x2, det.y2) == (10.0, 20.0, 110.0, 120.0)
    assert det.label == "mr_stabs_mk1"
    assert det.keypoints[0].confidence == 0.98


def test_diagnostics(recording: Path) -> None:
    statuses = list(diag_io.iter_diagnostic_statuses(recording))
    assert [(ts - LOG_TIME, s["hardware_id"], s["name"]) for ts, s in statuses] == [
        (0, "pursuit_nav", "pursuit_nav"),
        (1, "runner", "navigation"),
        (1, "runner", "tick"),
    ]
    values = statuses[0][1]["values"]
    assert values == {"distance": 0.75, "facing_target": 1, "mode": "chase"}
    assert isinstance(values["facing_target"], int)
    assert statuses[2][1]["level"] == 1 and statuses[2][1]["message"] == "slow"

    df = diag_io.load_diagnostics(recording)
    assert list(df["distance"]) == [0.75, None] or df["distance"].iloc[0] == 0.75
    assert df["nav/using_previous_robots"].iloc[1] == 0
    assert df["stage/tick/elapsed_ms"].iloc[1] == 12.5


def test_legacy_diagnostics_topic_selects_per_module_channels(recording: Path) -> None:
    topics = {t for t, _ts, _d in mcap_io.iter_messages(recording, ["/diagnostics"])}
    assert topics == {"/diagnostics/pursuit_nav", "/diagnostics/runner"}


def test_log(recording: Path) -> None:
    entries = [mcap_io.decode_log(d) for _t, _ts, d in mcap_io.iter_messages(recording, ["/log"])]
    assert len(entries) == 1
    assert entries[0].level == "WARN"
    assert entries[0].message == "hello"
    assert entries[0].file == "main.cpp" and entries[0].line == 7
    assert entries[0].stamp_ns == STAMP_NS


def test_untagged_or_legacy_payload_is_refused() -> None:
    """Readers only understand the Foxglove layout; anything else must fail loudly."""
    with pytest.raises(ValueError, match="convert_ros1_mcap"):
        mcap_io.decode_camera_info(b"\x00\x01\x02")
    legacy = mcap_io.MessageBytes(b"{}", "ros1", "std_msgs/String", "/x")
    with pytest.raises(ValueError, match="convert_ros1_mcap"):
        mcap_io.decode_string(legacy)


def test_video_decoder_rejects_out_of_order_frames(tmp_path: Path) -> None:
    from auto_battlebot.recording import video_import

    clip = tmp_path / "clip.mp4"
    subprocess.run(
        ["ffmpeg", "-loglevel", "error", "-f", "lavfi", "-i", "testsrc=size=64x48:rate=30"]
        + ["-frames:v", "10", "-c:v", "libx264", str(clip)],
        check=True,
    )
    units = list(video_import.iter_access_units(clip, 30.0, preset="ultrafast"))
    with pytest.raises(ValueError, match="IDR"):
        mcap_io.VideoStreamDecoder().decode(units[3])
    decoder = mcap_io.VideoStreamDecoder()
    frames = [f for unit in units for f in decoder.decode(unit)] + decoder.flush()
    assert len(frames) == len(units)


def test_sysid_topics_round_trip_through_writer_and_sysid_io(tmp_path: Path) -> None:
    from auto_battlebot.recording import sysid_io

    path = tmp_path / "sysid.mcap"
    tags = {
        "image_stamp_ns": 5,
        "frame_id": "camera",
        "camera": {"fx": 1.0, "fy": 1.0, "cx": 0.0, "cy": 0.0, "width": 4, "height": 4},
        "tag_size_m": 0.1,
        "roi": None,
        "detections": [],
    }
    with mcap_write.McapWriter(path) as writer:
        writer.log_json(sysid_io.ROBOT_TAGS_TOPIC, tags, 10)
        writer.log_json(sysid_io.ESPNOW_TELEMETRY_TOPIC, {"timestamp_ms": 7}, 20)
    assert [f.image_stamp_ns for f in sysid_io.load_robot_tags(path)] == [5]
    telemetry = sysid_io.load_robot_telemetry(path)
    assert telemetry["timestamp_ms"].tolist() == [7]

    diagnostics_path = tmp_path / "diag.mcap"
    with mcap_write.McapWriter(diagnostics_path) as writer:
        writer.log_json(sysid_io.ESP32_DIAGNOSTICS_TOPIC, {"timestamp_ms": 9}, 30)
    assert sysid_io.load_robot_telemetry(diagnostics_path)["timestamp_ms"].tolist() == [9]


def test_two_writers_same_topic_separate_files(tmp_path: Path) -> None:
    first, second = tmp_path / "a.mcap", tmp_path / "b.mcap"
    jpeg = _jpeg()
    with (
        mcap_write.McapWriter(first) as a,
        mcap_write.McapWriter(second) as b,
    ):
        a.log("/camera/image", mcap_write.compressed_image(1, "camera", jpeg), 1)
        b.log("/camera/image", mcap_write.compressed_image(2, "camera", jpeg), 2)
        b.log("/camera/image", mcap_write.compressed_image(3, "camera", jpeg), 3)
    counts = [len(list(mcap_io.iter_messages(p, ["/camera/image"]))) for p in (first, second)]
    assert counts == [1, 2]


def test_camera_info_rejects_wrong_matrix_shapes() -> None:
    with pytest.raises(ValueError, match="projection"):
        mcap_io.CameraInfo(1, "camera", 4, 4, "plumb_bob", np.zeros(5), np.eye(3), np.eye(3))


def test_sysid_io_rejects_malformed_arrays() -> None:
    from auto_battlebot.recording import sysid_io

    with pytest.raises(ValueError, match="corners"):
        sysid_io.TagDetection(1, np.zeros((3, 2)), None, [])
    with pytest.raises(ValueError, match="ascending"):
        sysid_io.FieldFromCamera(np.array([2, 1]), np.zeros((2, 4, 4)))
    with pytest.raises(ValueError, match="matrices"):
        sysid_io.FieldFromCamera(np.array([1, 2]), np.zeros((3, 4, 4)))
