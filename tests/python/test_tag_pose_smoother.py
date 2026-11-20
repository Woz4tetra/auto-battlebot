"""Sysid readers, clock fit, IPPE choice, RTS smoother and window gating on a synthetic session."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, replace
from pathlib import Path
from types import SimpleNamespace
from typing import Any, NamedTuple

import cv2
import numpy as np
import pandas as pd
import pytest

from auto_battlebot.perception import tag_pose_smoother as tps
from auto_battlebot.perception.sysid_windows import WindowOptions, make_windows, wheel_speeds
from auto_battlebot.recording import esp32_clock, mcap_write, sysid_io

REPO_ROOT = Path(__file__).resolve().parents[2]
MASS_PROPERTIES = (
    REPO_ROOT / "simulation" / "assets" / "robots" / "mr_stabs_mk2" / "mass_properties.toml"
)

T0_NS = 1_790_000_000_000_000_000
TAG_SIZE = 0.064
MAX_CLEAN_FIXES_AT_GATE = 3
REST_PITCH = 0.198
K = np.array([[665.0, 0.0, 960.0], [0.0, 716.0, 600.0], [0.0, 0.0, 1.0]])
WIDTH, HEIGHT = 1920, 1200
PIXEL_NOISE = 0.3
FIELD_Z_INTO_FLOOR_FLIP = np.diag([1.0, -1.0, -1.0, 1.0])

R_TOP = np.array([[0.0, -0.986029, 0.166572], [1.0, 0.0, 0.0], [0.0, 0.166572, 0.986029]])
R_BOTTOM = np.array([[0.0, -0.983904, 0.178696], [-1.0, 0.0, 0.0], [0.0, -0.178696, -0.983904]])
TOP_ID, BOTTOM_ID = 76, 41


def _orthonormal(r: np.ndarray) -> np.ndarray:
    u, _, vt = np.linalg.svd(r)
    return np.asarray(u @ vt)


def _pose(r: np.ndarray, t: np.ndarray) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = t
    return m


def mounts() -> dict[int, tps.TagMount]:
    return {
        TOP_ID: tps.TagMount(TOP_ID, _pose(_orthonormal(R_TOP), [0.0356, 0.0, 0.0142]), False),
        BOTTOM_ID: tps.TagMount(
            BOTTOM_ID, _pose(_orthonormal(R_BOTTOM), [0.030, 0.0, -0.011]), True
        ),
    }


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def camera_in_up(position: np.ndarray, target: np.ndarray) -> np.ndarray:
    """T_up_camera for an OpenCV camera at ``position`` looking at ``target``."""
    z = target - position
    z /= np.linalg.norm(z)
    x = np.cross(z, [0.0, 0.0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return _pose(np.stack([x, y, z], axis=1), position)


T_UP_CAMERA = camera_in_up(np.array([0.05, -0.5, 0.8]), np.array([0.0, 0.05, 0.0]))
T_FIELD_CAMERA = FIELD_Z_INTO_FLOOR_FLIP @ T_UP_CAMERA

MARKER_CORNERS = np.array(
    [
        [-TAG_SIZE / 2, TAG_SIZE / 2, 0.0],
        [TAG_SIZE / 2, TAG_SIZE / 2, 0.0],
        [TAG_SIZE / 2, -TAG_SIZE / 2, 0.0],
        [-TAG_SIZE / 2, -TAG_SIZE / 2, 0.0],
    ]
)


def body_pose(
    x: float, y: float, yaw: float, pitch: float, upside_down: bool = False
) -> np.ndarray:
    r = rot_z(yaw) @ rot_y(pitch)
    if upside_down:
        r = r @ rot_x(math.pi)
    return _pose(r, [x, y, 0.025])


def detect(
    t_up_body: np.ndarray, mount: tps.TagMount, rng: np.random.Generator
) -> dict[str, Any] | None:
    """Render one tag and solve it the way the app does; None when it is not visible."""
    t_cam_tag = np.linalg.inv(T_UP_CAMERA) @ t_up_body @ mount.t_body_tag
    normal = t_cam_tag[:3, 2]
    to_camera = -t_cam_tag[:3, 3]
    cos_view = float(normal @ to_camera / np.linalg.norm(to_camera))
    if cos_view < math.cos(math.radians(70.0)):
        return None
    rvec, _ = cv2.Rodrigues(t_cam_tag[:3, :3])
    pts, _ = cv2.projectPoints(MARKER_CORNERS, rvec, t_cam_tag[:3, 3], K, None)
    pts = pts.reshape(4, 2) + rng.normal(0.0, PIXEL_NOISE, (4, 2))
    if np.any(pts < 0) or np.any(pts[:, 0] >= WIDTH) or np.any(pts[:, 1] >= HEIGHT):
        return None
    n, rvecs, tvecs, errs = cv2.solvePnPGeneric(
        MARKER_CORNERS, pts, K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE
    )
    solutions = [
        {
            "rvec": np.asarray(rvecs[i]).ravel().tolist(),
            "tvec": np.asarray(tvecs[i]).ravel().tolist(),
            "reprojection_error_px": float(np.asarray(errs).ravel()[i]),
        }
        for i in range(int(n))
    ]
    return {
        "id": mount.tag_id,
        "corners": pts.tolist(),
        "decision_margin": 60.0,
        "solutions": solutions,
    }


@dataclass
class Truth:
    t_s: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    pitch: np.ndarray
    v: np.ndarray
    w: np.ndarray

    def at(self, t: np.ndarray) -> dict[str, np.ndarray]:
        return {
            name: np.interp(t, self.t_s, getattr(self, name))
            for name in ("x", "y", "yaw", "pitch", "v", "w")
        }


def _smooth(x: np.ndarray, sigma_samples: float) -> np.ndarray:
    half = int(3 * sigma_samples)
    k = np.exp(-0.5 * (np.arange(-half, half + 1) / sigma_samples) ** 2)
    k /= k.sum()
    padded = np.concatenate([np.full(half, x[0]), x, np.full(half, x[-1])])
    return np.convolve(padded, k, mode="valid")


class Segment(NamedTuple):
    label: str
    t0: float
    t1: float
    v: float
    w: float


SEGMENTS = (
    Segment("straight hold", 1.5, 2.7, 0.4, 0.0),
    Segment("reversal", 2.7, 3.9, -0.4, 0.0),
    Segment("spin left", 5.0, 6.5, 0.0, 8.0),
    Segment("spin right, a reversal of the spin", 6.5, 8.0, 0.0, -8.0),
    Segment("arc", 9.0, 10.4, 0.3, 2.0),
)


def make_truth(duration: float = 13.0) -> Truth:
    dt = 0.001
    t = np.arange(0.0, duration, dt)
    v = np.zeros_like(t)
    w = np.zeros_like(t)
    pitch = np.full_like(t, REST_PITCH)
    for segment in SEGMENTS:
        t0, t1, vv, ww = segment.t0, segment.t1, segment.v, segment.w
        sel = (t >= t0) & (t < t1)
        v[sel] = vv
        w[sel] = ww
    lift = (t >= 10.6) & (t < 11.4)
    pitch[lift] = REST_PITCH - NOSE_LIFT_RAD
    v[(t >= 10.6) & (t < 10.9)] = 0.3
    v = _smooth(v, 30.0)
    w = _smooth(w, 30.0)
    pitch = _smooth(pitch, 60.0)
    yaw = 0.3 + np.cumsum(w) * dt
    x = -0.1 + np.cumsum(v * np.cos(yaw)) * dt
    y = -0.15 + np.cumsum(v * np.sin(yaw)) * dt
    return Truth(t, x, y, yaw, pitch, v, w)


class StickSample(NamedTuple):
    log_ns: int
    channels: list[int]


@dataclass
class Session:
    truth: Truth
    frames: list[dict[str, Any]]
    frame_log_ns: list[int]
    esp32: list[dict[str, Any]]
    host_true_app_clock_ns: np.ndarray
    sticks: list[StickSample]
    outlier_stamps: set[int]


RADIO_DELAY_S = 0.015
IMU_LAG_S = 0.010
ROBOT_CLOCK_DRIFT_PPM = -40.0
ROBOT_DRIFT = ROBOT_CLOCK_DRIFT_PPM * 1e-6
MIN_LATENCY_MS = 2.0
ESP32_EVENT_PERIOD_S = 0.004
DIAG_TICK_S = 0.01
BLACKOUT_S = (5.6, 5.9)
NOSE_LIFT_RAD = 0.35
NOSE_LIFT_WINDOW_RAD = 0.3
NO_TEMPORAL_HELP_GAP_NS = 10**9
EVERY_CALL_CLOSE_MARGIN_DEG = 360.0
MAX_YAW_STEP_RAD = 0.2
BLACKOUT_START_S = 5.5
LIFT_WINDOW_S = (10.9, 11.1)
REST_WINDOW_S = (12.0, 12.8)
GRID_STEP_NS = 5_000_000
GRID_HZ = 200.0
FIX_RATE_HZ = 51.0
HEADING_CLOCKWISE = 1
STICK_HOLD_HALF_TICK_S = 0.005
RAIL_X_M = 0.70
RAIL_X_LIMIT_M = 0.76
RAIL_MARGIN_M = 0.15
RAIL_WINDOW_S = (4.0, 6.0)
ESP32_GAP_S = (15.0, 15.3)
HALF_COVERED_S = (8.0, 10.0)
GAP_WINDOW_S = (1.3, 1.4)
MEASURED_WINDOW_S = (0.5, 0.9)


def stick_channels(v: float, w: float) -> list[int]:
    ch = [992] * 16
    ch[0] = int(round(992 + 819.5 * v / 1.0))
    ch[1] = int(round(992 + 819.5 * w / 10.0))
    return ch


def in_order_delivery(host_true_ns: np.ndarray, latency_ms: np.ndarray) -> np.ndarray:
    return np.maximum.accumulate(host_true_ns + np.round(latency_ms * 1e6).astype(np.int64))


def make_session(
    seed: int = 1, outliers: int = 0, duration: float = 13.0, vbat_nulls: bool = True
) -> Session:
    rng = np.random.default_rng(seed)
    truth = make_truth(duration)
    m = mounts()
    frames = []
    frame_log = []
    outlier_stamps: set[int] = set()
    frame_t = np.arange(0.02, duration - 0.02, 1.0 / 60.0)
    drop = rng.random(frame_t.size) < 0.15
    drop |= (frame_t > BLACKOUT_S[0]) & (frame_t < BLACKOUT_S[1])
    outlier_idx = set(
        rng.choice(np.flatnonzero(~drop & (frame_t > 1.0)), size=outliers, replace=False).tolist()
    )
    at = truth.at(frame_t)
    for i, ft in enumerate(frame_t):
        stamp = T0_NS + int(round(ft * 1e9))
        detections = []
        if not drop[i]:
            pose = body_pose(at["x"][i], at["y"][i], at["yaw"][i], at["pitch"][i])
            det = detect(pose, m[TOP_ID], rng)
            if det is not None:
                if i in outlier_idx:
                    for sol in det["solutions"]:
                        sol["tvec"][0] += 0.12
                    outlier_stamps.add(stamp)
                detections.append(det)
        frames.append(
            {
                "image_stamp_ns": stamp,
                "frame_id": "camera",
                "camera": {
                    "fx": K[0, 0],
                    "fy": K[1, 1],
                    "cx": K[0, 2],
                    "cy": K[1, 2],
                    "width": WIDTH,
                    "height": HEIGHT,
                },
                "tag_size_m": TAG_SIZE,
                "roi": None,
                "detections": detections,
            }
        )
        frame_log.append(stamp + 25_000_000)

    ev_t = np.arange(0.0, duration, ESP32_EVENT_PERIOD_S)
    host_true = T0_NS + np.round(ev_t * 1e9).astype(np.int64)
    latency_ms = MIN_LATENCY_MS + rng.exponential(3.0, ev_t.size)
    burst = rng.random(ev_t.size) < 0.02
    latency_ms[burst] += rng.uniform(20.0, 80.0, burst.sum())
    receive = in_order_delivery(host_true, latency_ms)
    robot_ms = np.floor(5_000_000.0 + ev_t * 1e3 * (1.0 + ROBOT_DRIFT)).astype(np.int64)
    radio = truth.at(ev_t - RADIO_DELAY_S)
    imu = truth.at(ev_t - IMU_LAG_S)
    now = truth.at(ev_t)
    events = []
    for i in range(ev_t.size):
        a = radio["v"][i] * 100.0
        b = radio["w"][i] * 10.0
        heading = (-math.degrees(imu["yaw"][i])) % 360.0
        events.append(
            {
                "host_receive_ns": int(receive[i]),
                "timestamp_ms": int(robot_ms[i]),
                "radio_connected": True,
                "armed": True,
                "a_percent": a,
                "b_percent": b,
                "button_state": False,
                "flip_switch": 0,
                "left_cmd": -a + b,
                "right_cmd": -a - b,
                "accel_x": 0.0,
                "accel_y": 0.0,
                "accel_z": 9.8,
                "is_upside_down": False,
                "loop_us": 900,
                "wifi_clients": 1,
                "orientation_x": round(heading * 16.0) / 16.0,
                "orientation_y": math.degrees(now["pitch"][i]),
                "orientation_z": 0.0,
                "pid_setpoint": 0.0,
                "pid_output": 0.0,
                "vbat": None if (vbat_nulls and i % 50 == 0) else 11.4,
            }
        )
    sticks = []
    last: list[int] | None = None
    tick_t = np.arange(0.0, duration, DIAG_TICK_S)
    st = truth.at(tick_t)
    for i, tt in enumerate(tick_t):
        ch = stick_channels(st["v"][i], st["w"][i])
        if ch != last:
            sticks.append(StickSample(T0_NS + int(round(tt * 1e9)), ch))
            last = ch
    return Session(truth, frames, frame_log, events, host_true, sticks, outlier_stamps)


def write_mcap(session: Session, path: Path) -> Path:
    with mcap_write.McapWriter(path, active_profile="unit_test") as writer:
        cw = _pose(np.eye(3), [0.0, 0.0, 0.0])
        for k in range(0, len(session.frames), 6):
            stamp = session.frames[k]["image_stamp_ns"]
            writer.log(
                "/tf",
                mcap_write.fg.FrameTransforms(
                    transforms=[
                        mcap_write.frame_transform(stamp, "field", "camera_world", T_FIELD_CAMERA),
                        mcap_write.frame_transform(stamp, "camera_world", "camera", cw),
                    ]
                ),
                stamp,
            )
        for payload, log_ns in zip(session.frames, session.frame_log_ns):
            writer.log_json(sysid_io.ROBOT_TAGS_TOPIC, payload, log_ns)
        for event in session.esp32:
            writer.log_json(
                sysid_io.ESP32_DIAGNOSTICS_TOPIC,
                event,
                event["host_receive_ns"],
            )
        for log_ns, ch in session.sticks:
            writer.log_diagnostics(
                "opentx_transmitter",
                {
                    "channels": {
                        "level": 0,
                        "message": "",
                        "values": {f"values/{i}": v for i, v in enumerate(ch)},
                    }
                },
                log_ns,
            )
    return path


def parsed_frames(session: Session) -> list[sysid_io.RobotTagsFrame]:
    return [
        sysid_io.parse_robot_tags(json.loads(json.dumps(p)), log)
        for p, log in zip(session.frames, session.frame_log_ns)
    ]


def field_series(session: Session) -> sysid_io.FieldFromCamera:
    stamps = np.array([session.frames[0]["image_stamp_ns"]], dtype=np.int64)
    return sysid_io.FieldFromCamera(stamps, T_FIELD_CAMERA[None].copy())


def esp32_frame(session: Session) -> pd.DataFrame:
    df = sysid_io.esp32_events_frame(session.esp32)
    clock = esp32_clock.fit_robot_clock(df["timestamp_ms"].to_numpy(), df["host_receive_ns"])
    df["stamp_ns"] = clock.app_clock_stamp_ns
    return df


@pytest.fixture(scope="module")
def session() -> Session:
    return make_session()


@pytest.fixture(scope="module")
def smoothed(session: Session) -> tps.SessionSmoothing:
    return tps.smooth_session(
        parsed_frames(session), field_series(session), mounts(), esp32=esp32_frame(session)
    )


def test_reader_round_trip(session: Session, tmp_path: Path) -> None:
    path = write_mcap(session, tmp_path / "session.mcap")

    frames = sysid_io.load_robot_tags(path)
    assert len(frames) == len(session.frames)
    src = session.frames[100]
    got = frames[100]
    assert got.image_stamp_ns == src["image_stamp_ns"]
    assert got.log_time_ns == session.frame_log_ns[100]
    assert got.tag_size_m == pytest.approx(TAG_SIZE)
    assert got.camera.fx == pytest.approx(K[0, 0])
    assert got.roi is None
    assert len(got.detections) == len(src["detections"])
    if got.detections:
        det = got.detections[0]
        assert det.tag_id == TOP_ID
        np.testing.assert_allclose(det.corners_aruco_order_px, src["detections"][0]["corners"])
        np.testing.assert_allclose(
            det.solutions[0].rvec_camera_from_marker, src["detections"][0]["solutions"][0]["rvec"]
        )
        rot, _ = cv2.Rodrigues(det.solutions[0].rvec_camera_from_marker)
        np.testing.assert_allclose(det.solutions[0].transform[:3, :3], rot, atol=1e-12)
    assert sum(1 for f in frames if not f.detections) > 0

    esp = sysid_io.load_robot_telemetry(path)
    assert len(esp) == len(session.esp32)
    assert list(esp.columns[1:]) == [name for name, _ in sysid_io.ESP32_FIELDS]
    assert (esp["log_time_ns"] == esp["host_receive_ns"]).all()
    assert esp["vbat"].isna().sum() == sum(1 for e in session.esp32 if e["vbat"] is None)
    assert esp["armed"].dtype == bool
    assert esp["timestamp_ms"].iloc[5] == session.esp32[5]["timestamp_ms"]

    sticks = sysid_io.load_transmitter_channels(path)
    assert len(sticks) == len(session.sticks)
    assert sticks["ch0"].iloc[3] == session.sticks[3][1][0]

    field = sysid_io.load_field_from_camera(path)
    assert len(field) > 10
    np.testing.assert_allclose(field.matrices[0], T_FIELD_CAMERA, atol=1e-9)
    constancy = sysid_io.field_transform_constancy(field)
    assert constancy["max_translation_m"] < 1e-9
    assert constancy["max_rotation_deg"] < 1e-4


def test_rodrigues_matches_opencv() -> None:
    rng = np.random.default_rng(3)
    for _ in range(20):
        rvec = rng.normal(size=3)
        expected, _ = cv2.Rodrigues(rvec)
        np.testing.assert_allclose(sysid_io.rodrigues(rvec), expected, atol=1e-12)


def _single_frame(det: dict[str, Any], stamp: int) -> sysid_io.RobotTagsFrame:
    payload = {
        "image_stamp_ns": stamp,
        "frame_id": "camera",
        "camera": {
            "fx": 665.0,
            "fy": 716.0,
            "cx": 960.0,
            "cy": 600.0,
            "width": 1920,
            "height": 1200,
        },
        "tag_size_m": TAG_SIZE,
        "roi": None,
        "detections": [det],
    }
    return sysid_io.parse_robot_tags(payload, stamp)


@pytest.mark.parametrize("upside_down", [False, True])
def test_ippe_selection_on_tilted_tag(upside_down: bool) -> None:
    rng = np.random.default_rng(5)
    m = mounts()
    mount = m[BOTTOM_ID if upside_down else TOP_ID]
    rest = 0.0 if upside_down else REST_PITCH
    frames = []
    truths = []
    stamp = T0_NS
    for yaw in np.linspace(-math.pi, math.pi, 36, endpoint=False):
        for x, y in ((-0.3, -0.3), (0.0, 0.0), (0.4, 0.3)):
            pose = body_pose(x, y, yaw, rest, upside_down=upside_down)
            det = detect(pose, mount, rng)
            if det is None or len(det["solutions"]) < 2:
                continue
            frames.append(_single_frame(det, stamp))
            truths.append(pose)
            stamp += NO_TEMPORAL_HELP_GAP_NS
    assert len(frames) > 50
    field = sysid_io.FieldFromCamera(np.array([T0_NS]), T_FIELD_CAMERA[None].copy())
    t_up_field, flipped = tps.up_from_field(field)
    assert flipped
    cands = tps.build_candidates(frames, field, m, t_up_field, REST_PITCH, 0.0)
    ippe = tps.select_ippe(cands)
    wrong_far = 0
    for i, truth in enumerate(truths):
        errs = [
            math.degrees(sysid_io.rotation_angle(truth[:3, :3].T @ cands.t_up_body[i, s, :3, :3]))
            for s in (0, 1)
        ]
        if not ippe.close_call_decided_by_neighbours[i]:
            assert ippe.solution_index[i] == int(np.argmin(errs)), (
                i,
                errs,
                cands.tilt_from_rest_rad[i],
            )
            assert errs[ippe.solution_index[i]] < 6.0
        wrong_far += max(errs) > 5.0
    assert wrong_far > len(truths) // 2
    assert ippe.close_calls < len(truths) // 4


def test_ippe_close_call_uses_neighbours() -> None:
    """Force every call to be close: the temporal tiebreak still follows the clear neighbours."""
    rng = np.random.default_rng(7)
    m = mounts()
    frames = []
    truths = []
    for k in range(40):
        pose = body_pose(0.1, 0.0, 0.5 + 0.01 * k, REST_PITCH)
        det = detect(pose, m[TOP_ID], rng)
        assert det is not None
        frames.append(_single_frame(det, T0_NS + k * 16_666_667))
        truths.append(pose)
    field = sysid_io.FieldFromCamera(np.array([T0_NS]), T_FIELD_CAMERA[None].copy())
    cands = tps.build_candidates(frames, field, m, tps.up_from_field(field)[0], REST_PITCH)
    cands.n_solutions[:] = 2
    ippe = tps.select_ippe(cands, close_margin_deg=EVERY_CALL_CLOSE_MARGIN_DEG)
    assert ippe.close_calls == len(cands)
    for i, truth in enumerate(truths):
        errs = [
            sysid_io.rotation_angle(truth[:3, :3].T @ cands.t_up_body[i, s, :3, :3]) for s in (0, 1)
        ]
        assert ippe.solution_index[i] == int(np.argmin(errs))


def _truth_at_measurements(truth: Truth, stamps: np.ndarray) -> dict[str, np.ndarray]:
    return truth.at((stamps - T0_NS) / 1e9)


def test_rts_recovers_trajectory_better_than_raw(
    session: Session, smoothed: tps.SessionSmoothing
) -> None:
    meas = smoothed.detections_df
    assert smoothed.field_flipped
    good = ~meas["rejected"].to_numpy()
    truth = _truth_at_measurements(session.truth, meas["stamp_ns"].to_numpy())
    raw_pos = np.hypot(meas["x"] - truth["x"], meas["y"] - truth["y"])[good]
    smooth_pos = np.hypot(meas["smoothed_x"] - truth["x"], meas["smoothed_y"] - truth["y"])[good]
    raw_yaw = tps.wrap_angle(meas["yaw"] - truth["yaw"])[good]
    smooth_yaw = tps.wrap_angle(meas["smoothed_yaw"] - truth["yaw"])[good]
    rms = lambda v: float(np.sqrt(np.mean(np.square(v))))  # noqa: E731
    assert rms(raw_pos) < 0.01, "raw body pose is right to begin with"
    assert rms(smooth_pos) < 0.75 * rms(raw_pos)
    assert rms(smooth_yaw) < 0.75 * rms(raw_yaw)
    assert int(meas["rejected"].sum()) <= MAX_CLEAN_FIXES_AT_GATE

    grid = smoothed.smoothed_grid_df
    gt = session.truth.at((grid["stamp_ns"].to_numpy() - T0_NS) / 1e9)
    assert np.max(np.abs(np.diff(grid["yaw"]))) < MAX_YAW_STEP_RAD, (
        "yaw unwrapped through both spins"
    )
    assert rms(tps.wrap_angle(grid["yaw"] - gt["yaw"])) < math.radians(2.0)
    sel = ((grid["stamp_ns"] - T0_NS) / 1e9 < BLACKOUT_START_S).to_numpy()
    v = grid["vx"] * np.cos(grid["yaw"]) + grid["vy"] * np.sin(grid["yaw"])
    assert rms((v - gt["v"])[sel]) < 0.05
    assert rms((grid["yaw_rate"] - gt["w"])[sel]) < 0.4
    t = (grid["stamp_ns"] - T0_NS) / 1e9
    lift = ((t > LIFT_WINDOW_S[0]) & (t < LIFT_WINDOW_S[1])).to_numpy()
    assert np.mean(grid["pitch"][lift]) == pytest.approx(REST_PITCH - NOSE_LIFT_RAD, abs=0.03)
    rest = ((t > REST_WINDOW_S[0]) & (t < REST_WINDOW_S[1])).to_numpy()
    assert np.mean(grid["pitch"][rest]) == pytest.approx(REST_PITCH, abs=0.01)
    assert np.all(np.diff(grid["stamp_ns"]) == GRID_STEP_NS)
    assert grid["has_measurement"].mean() == pytest.approx(FIX_RATE_HZ / GRID_HZ, abs=0.1)


def test_noise_calibration_and_checks(smoothed: tps.SessionSmoothing) -> None:
    assert smoothed.noise.calibrated
    assert smoothed.noise.noise_calibration_segments >= 3
    floor = smoothed.checks["noise_floor"]
    assert floor["detections"] > 100
    assert 0.0 < floor["sigma_x"] < 0.005
    assert 0.0 < floor["sigma_yaw"] < math.radians(1.0)
    white = smoothed.checks["residual_whiteness"]
    assert -0.5 < white["across_innovation"]["lag1"] < 0.1
    for cv, group in ((smoothed.cv_planar, "xy"), (smoothed.cv_planar, "yaw")):
        k = int(np.nanargmin(cv.heldout_score_by_group[group]))
        assert 0 < k < cv.heldout_score_by_group[group].size - 1, (
            group,
            cv.heldout_score_by_group[group],
        )


def test_imu_yaw_rate_agreement(smoothed: tps.SessionSmoothing) -> None:
    check = smoothed.checks["imu_yaw_rate"]
    assert check["measured_sign"] == HEADING_CLOCKWISE
    assert check["lag_s"] == pytest.approx(IMU_LAG_S, abs=0.006)
    assert check["correlation"] > 0.95
    assert check["spin_rms_rad_s"] < 0.6
    assert check["spin_gain"] == pytest.approx(1.0, abs=0.05)


def test_imu_fusion_runs(session: Session) -> None:
    result = tps.smooth_session(
        parsed_frames(session),
        field_series(session),
        mounts(),
        tps.SmootherOptions(fuse_imu=True),
        esp32=esp32_frame(session),
    )
    fused = result.checks["imu_fused"]
    assert fused and fused["sign"] == 1.0
    grid = result.smoothed_grid_df
    t = (grid["stamp_ns"] - T0_NS) / 1e9
    gt = session.truth.at(t.to_numpy())
    blackout = ((t > 5.62) & (t < 5.88)).to_numpy()
    err = np.abs(grid["yaw_rate"] - gt["w"])[blackout]
    assert float(np.max(err)) < 0.6


def test_imu_rate_pulls_an_aliased_yaw_rate_back() -> None:
    """A rate knot from the IMU pulls yaw rate off a whole turn per frame despite the gate."""
    alias = 2.0 * math.pi * 60.0
    tf = np.arange(180) / 60.0
    ti = np.arange(0.005, 3.0, 0.01)
    t = np.r_[tf, ti]
    order = np.argsort(t, kind="stable")
    kind = np.r_[np.full(tf.size, tps.KnotKind.POSITION), np.full(ti.size, tps.KnotKind.RATE)]
    z = np.r_[np.full(tf.size, 1.0), np.where(ti < 0.5, alias, 0.0)]
    r = np.r_[np.full(tf.size, 0.01**2), np.full(ti.size, 0.3**2)]
    knots = tps.Knots(
        t_since_t0_s=t[order],
        t0_ns=0,
        kind=kind.astype(np.int8)[order, None],
        z=z[order, None],
        variance=r[order, None],
    )
    opts = tps.FilterOptions(angular=(True,), prior_std=((math.pi, 1000.0, 1000.0),))
    run = tps.run_smoother(knots, np.array([[1e3]]), opts)
    assert run.m_filt[0, np.searchsorted(knots.t_since_t0_s, 0.45), 0, 1] == pytest.approx(
        alias, rel=0.01
    )
    assert np.max(np.abs(run.m_smooth[0, knots.t_since_t0_s > 2.0, 0, 1])) < 0.1


def test_gating_rejects_injected_outliers() -> None:
    session = make_session(seed=11, outliers=12)
    result = tps.smooth_session(parsed_frames(session), field_series(session), mounts())
    meas = result.detections_df
    injected = meas["stamp_ns"].isin(session.outlier_stamps).to_numpy()
    assert injected.sum() == 12
    rejected = meas["rejected"].to_numpy()
    assert rejected[injected].all()
    assert rejected[~injected].sum() <= 2
    assert len(result.rejections) == int(rejected.sum())
    assert result.checks["gating"]["rejected"] == int(rejected.sum())


def test_smoother_bridges_a_dropout_without_lag() -> None:
    """A constant-acceleration track through a 0.3 s gap: RTS output centred, no lag."""
    rng = np.random.default_rng(2)
    t = np.arange(0.0, 3.0, 1.0 / 60.0)
    t = t[(t < 1.2) | (t > 1.5)]
    x_true = 0.2 * t**2
    knots = tps.Knots(
        t_since_t0_s=t,
        t0_ns=0,
        kind=np.full((t.size, 1), tps.KnotKind.POSITION, dtype=np.int8),
        z=(x_true + rng.normal(0.0, 0.002, t.size))[:, None],
        variance=np.full((t.size, 1), 0.002**2),
    )
    opts = tps.FilterOptions(angular=(False,), prior_std=((1.0, 5.0, 50.0),))
    run = tps.run_smoother(knots, np.array([[10.0]]), opts)
    grid = np.arange(0, int(3.0e9), 5_000_000, dtype=np.int64)
    m, cov = tps.interpolate_smoothed(run, knots, np.array([[10.0]]), grid)
    tg = grid / 1e9
    inside = (tg > 0.1) & (tg < 2.9)
    assert np.max(np.abs(m[inside, 0, 0] - 0.2 * tg[inside] ** 2)) < 0.004
    assert np.max(np.abs(m[inside, 0, 1] - 0.4 * tg[inside])) < 0.05
    gap = (tg > GAP_WINDOW_S[0]) & (tg < GAP_WINDOW_S[1])
    measured = (tg > MEASURED_WINDOW_S[0]) & (tg < MEASURED_WINDOW_S[1])
    assert np.sqrt(cov[gap, 0, 0, 0]).min() > np.sqrt(cov[measured, 0, 0, 0]).max(), (
        "uncertainty grows inside the gap"
    )


def test_clock_fit_recovers_offset_and_drift() -> None:
    rng = np.random.default_rng(4)
    t = np.arange(0.0, 600.0, 0.01)
    host_true = T0_NS + np.round(t * 1e9).astype(np.int64)
    latency_ms = MIN_LATENCY_MS + rng.exponential(3.0, t.size)
    burst = rng.random(t.size) < 0.03
    latency_ms[burst] += rng.uniform(20.0, 120.0, burst.sum())
    receive = host_true + np.round(latency_ms * 1e6).astype(np.int64)
    robot_ms = np.floor(123_456.0 + t * 1e3 * (1.0 + ROBOT_DRIFT)).astype(np.int64)
    fit = esp32_clock.fit_robot_clock(robot_ms, receive)
    assert len(fit.segments) == 1
    host_ppm_per_robot_second = (1.0 / (1.0 + ROBOT_DRIFT) - 1.0) * 1e6
    assert fit.segments[0].drift_ppm == pytest.approx(host_ppm_per_robot_second, abs=3.0)
    err_ms = (fit.app_clock_stamp_ns - host_true) / 1e6 - MIN_LATENCY_MS
    assert abs(float(np.median(err_ms))) < 1.0
    assert float(np.max(np.abs(err_ms))) < 1.5
    assert np.all(fit.receive_minus_mapped_ms > -1e-6)
    assert fit.segments[0].residual_ms_percentiles["p1"] < 0.5


def test_clock_fit_splits_reboots() -> None:
    t = np.arange(0.0, 20.0, 0.01)
    receive = T0_NS + np.round((t + 0.003) * 1e9).astype(np.int64)
    robot_ms = np.round(t * 1e3).astype(np.int64) + 50_000
    robot_ms[t >= 10.0] = np.round((t[t >= 10.0] - 10.0) * 1e3).astype(np.int64) + 5
    fit = esp32_clock.fit_robot_clock(robot_ms, receive)
    assert len(fit.segments) == 2
    assert float(np.max(np.abs(fit.app_clock_stamp_ns - receive))) < 1.1e6


def test_radio_delay_from_sticks(session: Session, tmp_path: Path) -> None:
    esp = esp32_frame(session)
    sticks = pd.DataFrame(
        [[s] + ch for s, ch in session.sticks], columns=["stamp_ns"] + [f"ch{i}" for i in range(16)]
    )
    lag = esp32_clock.radio_link_delay(
        sticks["stamp_ns"].to_numpy(),
        esp32_clock.crsf_to_unit(sticks["ch0"].to_numpy()),
        esp["stamp_ns"].to_numpy(),
        esp["a_percent"].to_numpy(),
    )
    assert lag.sign == 1
    assert lag.correlation > 0.8
    assert lag.b_lag_behind_a_s == pytest.approx(
        RADIO_DELAY_S - STICK_HOLD_HALF_TICK_S + MIN_LATENCY_MS / 1000.0, abs=0.006
    )


def _grid(duration: float, dt: float = 0.005) -> pd.DataFrame:
    n = int(round(duration / dt))
    stamps = T0_NS + np.arange(n, dtype=np.int64) * int(dt * 1e9)
    return pd.DataFrame(
        {
            "stamp_ns": stamps,
            "x": np.zeros(n),
            "y": np.zeros(n),
            "yaw": np.zeros(n),
            "vx": np.zeros(n),
            "vy": np.zeros(n),
            "yaw_rate": np.zeros(n),
            "pitch": np.full(n, REST_PITCH),
            "upside_down": np.zeros(n, dtype=bool),
        }
    )


def test_window_gating() -> None:
    grid = _grid(20.0)
    t = (grid["stamp_ns"] - T0_NS) / 1e9
    assert RAIL_X_M > RAIL_X_LIMIT_M - RAIL_MARGIN_M
    grid.loc[(t >= RAIL_WINDOW_S[0]) & (t < RAIL_WINDOW_S[1]), "x"] = RAIL_X_M
    grid.loc[(t >= 12.0) & (t < 13.0), "pitch"] = REST_PITCH - NOSE_LIFT_WINDOW_RAD
    grid.loc[t < 1.0, "vx"] = 0.5
    grid.loc[t < 1.0, "yaw_rate"] = 2.0
    frames = T0_NS + np.arange(0, int(20e9), 16_666_667, dtype=np.int64)
    ft = (frames - T0_NS) / 1e9
    accepted = frames[
        ~((ft >= HALF_COVERED_S[0]) & (ft < HALF_COVERED_S[1]) & (np.arange(frames.size) % 2 == 0))
    ]
    esp = T0_NS + np.arange(0, int(20e9), 4_000_000, dtype=np.int64)
    et = (esp - T0_NS) / 1e9
    esp = esp[~((et > ESP32_GAP_S[0]) & (et < ESP32_GAP_S[1]))]
    geometry = tps.RobotGeometry()
    windows, report = make_windows(grid, frames, accepted, esp, geometry, WindowOptions())

    assert list(windows["window_id"]) == list(range(len(windows)))
    start = (windows["start_ns"] - T0_NS) / 1e9
    end = (windows["end_ns"] - T0_NS) / 1e9
    length = end - start
    assert (length >= 0.99).all() and (length < 2.0).all()
    for lo, hi in (RAIL_WINDOW_S, ESP32_GAP_S, HALF_COVERED_S):
        assert not ((start < hi) & (end > lo)).any(), ("rail, ESP32 gap or half-covered", lo, hi)
    assert report["windows_dropped"]["coverage"] >= 1
    assert report["gate_drop_seconds"]["near_rail"] == pytest.approx(2.0, abs=0.02)
    assert report["gate_drop_seconds"]["esp32_gap"] == pytest.approx(0.3, abs=0.02)
    lift = windows[windows["kind"] == "nose_lift"]
    assert len(lift) >= 1
    assert ((lift["start_ns"] - T0_NS) / 1e9 < 13.0).all()
    assert report["windows"]["flat"] == int((windows["kind"] == "flat").sum())
    assert (windows["coverage"] >= 0.7).all()
    first = windows.iloc[0]
    left, right = wheel_speeds(
        np.array([0.5]),
        np.array([0.0]),
        np.array([0.0]),
        np.array([2.0]),
        np.array([False]),
        geometry,
    )
    assert first["wheel_left0"] == pytest.approx((0.5 - 2.0 * 0.06526) / 0.025)
    assert first["wheel_right0"] == pytest.approx((0.5 + 2.0 * 0.06526) / 0.025)
    assert first["wheel_left0"] == pytest.approx(left[0])
    assert right[0] > left[0]


def test_wheel_speeds_inverted() -> None:
    geometry = tps.RobotGeometry()
    left, right = wheel_speeds(
        np.array([0.0]),
        np.array([0.0]),
        np.array([0.0]),
        np.array([1.0]),
        np.array([True]),
        geometry,
    )
    assert left[0] > 0.0 > right[0], "inverted: counter-clockwise from above is clockwise about z"


def _write_mass_properties(path: Path) -> Path:
    lines = ["[geometry]", "track_half_width_m = 0.06526", "wheel_radius_m = 0.025", ""]
    for tag_id, mount in mounts().items():
        r = mount.t_body_tag[:3, :3]
        lines += [
            f"[tags.{tag_id}]",
            f"upside_down = {'true' if mount.tag_faces_floor_when_upright else 'false'}",
            f"translation_m = {mount.t_body_tag[:3, 3].tolist()}",
            f"rotation = {r.tolist()}",
            f"size_m = {TAG_SIZE}",
            "",
        ]
    path.write_text("\n".join(lines))
    return path


def test_cli_writes_bundle(session: Session, tmp_path: Path) -> None:
    from playground.calibration import smooth_tag_poses as cli

    mcap = write_mcap(session, tmp_path / "synthetic_session.mcap")
    mass = _write_mass_properties(tmp_path / "mass_properties.toml")
    out = cli.main(
        [
            str(mcap),
            "--out-root",
            str(tmp_path / "out"),
            "--mass-properties",
            str(mass),
            "--rest-pitch-rad",
            str(REST_PITCH),
            "--plots",
        ]
    )
    assert out == tmp_path / "out" / "synthetic_session"
    smoothed = pd.read_csv(out / "smoothed.csv")
    assert list(smoothed.columns) == cli.SMOOTHED_COLUMNS
    assert np.all(np.diff(smoothed["stamp_ns"]) == 5_000_000)
    meas = pd.read_csv(out / "measurements.csv")
    assert list(meas.columns) == cli.MEASUREMENT_COLUMNS
    commands = pd.read_csv(out / "commands.csv")
    assert list(commands.columns) == cli.COMMAND_COLUMNS
    assert len(commands) == len(session.esp32)
    assert commands["vbat"].isna().sum() == sum(1 for e in session.esp32 if e["vbat"] is None)
    err_ms = (commands["stamp_ns"].to_numpy() - session.host_true_app_clock_ns) / 1e6
    assert float(np.percentile(np.abs(err_ms - MIN_LATENCY_MS), 95)) < 2.0, (
        "command stamps follow the clock fit at minimum latency, not the jittered receive time"
    )
    windows = pd.read_csv(out / "windows.csv")
    assert list(windows.columns) == [
        "window_id", "start_ns", "end_ns", "kind", "coverage", "x0", "y0", "yaw0",
        "vx0", "vy0", "yaw_rate0", "pitch0", "wheel_left0", "wheel_right0",
    ]  # fmt: skip
    assert len(windows) > 0
    assert set(windows["kind"]) <= {"flat", "nose_lift"}
    info = json.loads((out / "session.json").read_text())
    for key in (
        "noise_floor",
        "process_noise",
        "clock_fit",
        "radio_delay",
        "imu_yaw_rate",
        "ippe",
        "gating",
        "field_transform_constancy",
        "windows",
    ):
        assert key in info, key
    assert info["output_frame"]["recorded_field_z_down"] is True
    assert info["radio_delay"]["linear"]["lag_ms"] == pytest.approx(12.0, abs=6.0)
    assert info["clock_fit"]["drift_ppm"][0] == pytest.approx(40.0, abs=80.0)
    for png in info["plots"]:
        assert (out / png).stat().st_size > 1000


@pytest.mark.skipif(not MASS_PROPERTIES.exists(), reason="mass_properties.toml not written yet")
def test_loads_mass_properties_file() -> None:
    mounts_, geometry = tps.load_mass_properties(MASS_PROPERTIES)
    assert set(mounts_) >= {41, 76}
    assert geometry.track_half_width_m == pytest.approx(0.06526, abs=1e-4)
    assert geometry.wheel_radius_m == pytest.approx(0.025, abs=1e-4)
    for mount in mounts_.values():
        r = mount.t_body_tag[:3, :3]
        np.testing.assert_allclose(r @ r.T, np.eye(3), atol=1e-4)
        assert mount.tag_faces_floor_when_upright == (r[2, 2] < 0.0)
        tilt = math.degrees(math.acos(abs(r[2, 2])))
        assert 5.0 < tilt < 15.0
    assert mounts_[41].tag_faces_floor_when_upright != mounts_[76].tag_faces_floor_when_upright


def test_tag_mount_upside_down_defaults_from_normal() -> None:
    up = np.eye(3)
    down = np.diag([1.0, -1.0, -1.0])
    table = {"translation_m": [0.0, 0.0, 0.0]}
    assert not tps.tag_mount_from_table(1, {**table, "rotation": up}).tag_faces_floor_when_upright
    assert tps.tag_mount_from_table(2, {**table, "rotation": down}).tag_faces_floor_when_upright
    forced = tps.tag_mount_from_table(3, {**table, "rotation": up, "upside_down": True})
    assert forced.tag_faces_floor_when_upright


def test_candidates_reject_mismatched_shapes() -> None:
    def build(
        n_solutions: np.ndarray, t_up_body: np.ndarray, size_px: np.ndarray
    ) -> tps.Candidates:
        return tps.Candidates(
            stamp_ns=np.zeros(2, np.int64),
            frame_index=np.zeros(2, np.int64),
            tag_id=np.zeros(2, np.int64),
            upside_down=np.zeros(2, bool),
            n_solutions=n_solutions,
            t_up_body=t_up_body,
            tilt_from_rest_rad=np.zeros((2, 2)),
            reprojection_error_px=np.zeros((2, 2)),
            size_px=size_px,
        )

    poses = np.broadcast_to(np.eye(4), (2, 2, 4, 4)).copy()
    build(np.array([1, 2]), poses, np.zeros(2))
    with pytest.raises(ValueError, match="size_px"):
        build(np.array([1, 2]), poses, np.zeros(3))
    with pytest.raises(ValueError, match="n_solutions"):
        build(np.array([1, 3]), poses, np.zeros(2))
    unequal = poses.copy()
    unequal[0, 1] = 2.0 * np.eye(4)
    with pytest.raises(ValueError, match="slot 1"):
        build(np.array([1, 2]), unequal, np.zeros(2))


def test_knots_reject_decreasing_time_and_bad_shapes() -> None:
    kind = np.full((3, 1), tps.KnotKind.POSITION, dtype=np.int8)
    good = dict(t0_ns=0, kind=kind, z=np.zeros((3, 1)), variance=np.ones((3, 1)))
    tps.Knots(t_since_t0_s=np.array([0.0, 1.0, 1.0]), **good)
    with pytest.raises(ValueError, match="non-decreasing"):
        tps.Knots(t_since_t0_s=np.array([0.0, 1.0, 0.5]), **good)
    with pytest.raises(ValueError, match="variance"):
        tps.Knots(t_since_t0_s=np.arange(3.0), **{**good, "variance": np.ones((3, 2))})
    with pytest.raises(ValueError, match="int8"):
        tps.Knots(t_since_t0_s=np.arange(3.0), **{**good, "kind": kind.astype(np.int64)})


def test_up_from_field_flags_camera_below_the_field() -> None:
    above = np.eye(4)
    above[2, 3] = 1.0
    below = np.eye(4)
    below[2, 3] = -1.0
    stamps = np.zeros(1, np.int64)
    t_up, flipped = tps.up_from_field(sysid_io.FieldFromCamera(stamps, above[None]))
    assert not flipped
    np.testing.assert_allclose(t_up, np.eye(4))
    frame = tps.up_from_field(sysid_io.FieldFromCamera(stamps, below[None]))
    assert frame.flipped
    np.testing.assert_allclose(frame.t_up_field, np.diag([1.0, -1.0, -1.0, 1.0]))


def test_angle_branch_after_dropout_mid_spin() -> None:
    rate = 8.0
    t = np.arange(0.0, 3.0, 1.0 / 60.0)
    t = t[(t < 1.5) | (t > 1.8)]
    rng = np.random.default_rng(0)
    truth = rate * t
    knots = tps.Knots(
        t_since_t0_s=t,
        t0_ns=0,
        kind=np.full((t.size, 1), tps.KnotKind.POSITION, dtype=np.int8),
        z=np.asarray(tps.wrap_angle(truth + rng.normal(0.0, 0.01, t.size)))[:, None],
        variance=np.full((t.size, 1), 0.01**2),
    )
    base = tps.FilterOptions(angular=(True,), prior_std=((math.pi, 40.0, 1000.0),))
    q = np.array([[1e6]])
    after = t > 1.9

    def worst_error(options: tps.FilterOptions) -> float:
        run = tps.run_smoother(knots, q, options)
        return float(np.max(np.abs(run.m_filt[0, after, 0, 0] - truth[after])))

    assert worst_error(base) < 0.1
    assert worst_error(replace(base, branch_gap_s=math.inf)) > math.pi


def test_recheck_readmits_fix_at_reversal() -> None:
    t = np.arange(0.0, 2.0, 1.0 / 60.0)
    speed = np.where(t < 1.0, 1.0, -1.0)
    x = np.cumsum(speed / 60.0)
    outlier = 30
    x[outlier] += 10.0
    knots = tps.Knots(
        t_since_t0_s=t,
        t0_ns=0,
        kind=np.full((t.size, 1), tps.KnotKind.POSITION, dtype=np.int8),
        z=x[:, None],
        variance=np.full((t.size, 1), 0.003**2),
    )
    options = tps.FilterOptions(
        angular=(False,), prior_std=((1.0, 5.0, 50.0),), max_consecutive_rejects=100
    )
    q = np.array([[3000.0]])
    forward = tps.run_smoother(knots, q, options)
    run, exempt = tps.smooth_with_recheck(knots, q, options)
    rejected_forward = set(np.flatnonzero(forward.outcome[0] == tps.Outcome.REJECTED))
    rejected_after = set(np.flatnonzero(run.outcome[0] == tps.Outcome.REJECTED))
    assert len(rejected_forward - {outlier}) >= 2
    assert rejected_after == {outlier}
    assert exempt.sum() >= 2


def test_interpolate_smoothed_is_exact_at_knots() -> None:
    t = np.arange(0.0, 2.0, 0.05)
    knots = tps.Knots(
        t_since_t0_s=t,
        t0_ns=1_000,
        kind=np.full((t.size, 1), tps.KnotKind.POSITION, dtype=np.int8),
        z=np.sin(t)[:, None],
        variance=np.full((t.size, 1), 0.01**2),
    )
    options = tps.FilterOptions(angular=(False,), prior_std=((1.0, 5.0, 50.0),))
    q = np.array([[10.0]])
    run = tps.run_smoother(knots, q, options)
    grid_ns = 1_000 + np.round(t * 1e9).astype(np.int64)
    mean, cov = tps.interpolate_smoothed(run, knots, q, grid_ns)
    np.testing.assert_allclose(mean, run.m_smooth[0], atol=1e-6)
    np.testing.assert_allclose(cov, run.p_smooth[0], atol=1e-6)
    clamped, _ = tps.interpolate_smoothed(run, knots, q, np.array([grid_ns[-1] + 10**9]))
    np.testing.assert_allclose(clamped[0], run.m_smooth[0, -1], atol=1e-6)


def test_tilt_smoother_does_not_alias_to_full_turn_per_frame() -> None:
    options = tps._filter_options(tps._TILT_OPTIONS, tps.SmootherOptions())
    assert not any(options.angular)
    t = np.arange(0.0, 1.0, 1.0 / 60.0)
    z = np.where(t < 0.5, 0.0, 4.0)
    knots = tps.Knots(
        t_since_t0_s=t,
        t0_ns=0,
        kind=np.full((t.size, 2), tps.KnotKind.POSITION, dtype=np.int8),
        z=np.stack([z, z], axis=1),
        variance=np.full((t.size, 2), 0.01**2),
    )
    run = tps.run_smoother(knots, np.full((1, 2), 1e6), replace(options, gate=False))
    assert run.m_filt[0, -1, 0, 0] == pytest.approx(4.0, abs=0.1)


def test_unwrap_onto_keeps_angle_within_half_turn_of_reference() -> None:
    reference = np.array([0.3, 7.0, -12.5])
    angle = reference + np.array([0.2, -0.4, 0.1]) + 2.0 * math.pi * np.array([3, -2, 5])
    out = tps.unwrap_onto(angle, reference)
    np.testing.assert_allclose(out, reference + np.array([0.2, -0.4, 0.1]))


def test_reported_tilt_is_wrapped() -> None:
    t = np.arange(0.0, 1.0, 0.05)

    def fit(axes: int, value: float) -> tps._AxisFit:
        knots = tps.Knots(
            t_since_t0_s=t,
            t0_ns=0,
            kind=np.full((t.size, axes), tps.KnotKind.POSITION, dtype=np.int8),
            z=np.full((t.size, axes), value),
            variance=np.full((t.size, axes), 0.01**2),
        )
        options = tps.FilterOptions(
            angular=(False,) * axes, prior_std=((10.0, 10.0, 100.0),) * axes
        )
        q = np.full((1, axes), 10.0)
        return tps._AxisFit(
            knots, np.arange(t.size), tps.run_smoother(knots, q, options), q, np.zeros(t.size, bool)
        )

    turn_off = 2.0 * math.pi + 0.3
    grid = tps._grid_frame(
        fit(3, 0.0), fit(2, turn_off), np.round(t[5:15] * 1e9).astype(np.int64), 0.0
    )
    np.testing.assert_allclose(grid["pitch"], 0.3, atol=0.05)
    np.testing.assert_allclose(grid["roll"], 0.3, atol=0.05)


def test_yolo_fix_outcomes_align_with_fix_rows(
    session: Session, smoothed: tps.SessionSmoothing, monkeypatch: pytest.MonkeyPatch
) -> None:
    grid = smoothed.smoothed_grid_df
    pick = np.array([300, 700, 1200, 1700, 2200])
    stamps = grid["stamp_ns"].to_numpy(np.int64)[pick]
    xs = grid["x"].to_numpy()[pick].copy()
    ys = grid["y"].to_numpy()[pick]
    outlier = 2
    xs[outlier] += 8.0
    fixes = pd.DataFrame({"stamp_ns": stamps, "x": xs, "y": ys, "yaw": np.nan, "conf": 0.9})
    shuffled = fixes.iloc[[3, 0, 4, 2, 1]].reset_index(drop=True)
    calibration = SimpleNamespace(
        sigma_along_m=0.02, sigma_across_m=0.02, sigma_yaw_rad=0.1, as_dict=lambda: {}
    )
    monkeypatch.setattr(tps.yolo_field_fixes, "calibrate", lambda *a, **k: calibration)
    monkeypatch.setattr(tps.yolo_field_fixes, "select_fixes", lambda *a, **k: (shuffled, {}))
    result = tps.smooth_session(
        parsed_frames(session),
        field_series(session),
        mounts(),
        yolo=pd.DataFrame({"stamp_ns": stamps}),
    )
    table = result.yolo_fixes
    assert table is not None
    assert table["stamp_ns"].is_monotonic_increasing
    rejected = table.loc[table["rejected"], "stamp_ns"].to_numpy()
    np.testing.assert_array_equal(rejected, [stamps[outlier]])
