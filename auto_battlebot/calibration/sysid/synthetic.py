"""A simulated hand-driven sysid session, written in the recorder's formats.

Stands in for a recorded session until one exists: tests replay it, and
``playground/calibration/sysid_coverage.py --synthetic`` writes it to an MCAP for the page.

- Robot: ``plant.Plant`` with invented "true" parameters behind the batched firmware mixer, so
  the flip switch down runs the same yaw loop the firmware does. Wall contact clamps the pose
  and kills most of the speed.
- Driver: a simple one that turns toward open floor, then does held steps, arcs, spins, creeps
  and snaps at amplitudes the 1.52 m box allows, and lets go when a rail comes up.
- Telemetry: every ``/robot/espnow_telemetry`` schema field at 100 Hz on a robot clock with an
  offset, plus receive jitter on the host clock and an occasional dropped frame.
- Tags: ``/apriltag/robot_tags`` at 60 Hz from a fixed camera, one IPPE solution per detection,
  pose noise of a few millimetres, frames dropped more often the faster the robot spins.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from auto_battlebot.calibration.sysid.profile import SysidProfile
from auto_battlebot.calibration.sysid.telemetry import ESPNOW_TELEMETRY_TOPIC
from auto_battlebot.control.firmware_mixer import (
    BatchedMixer,
    LoopState,
    gyro_from_w,
    heading_from_theta,
    motor_to_plant,
)
from auto_battlebot.control.plant import Plant, PlantParams, PlantState
from auto_battlebot.recording import mcap_write
from auto_battlebot.recording.sysid_io import ROBOT_TAGS_TOPIC

T0_NS = 1_760_000_000_000_000_000
DT = 0.01
FRAME_DT = 1.0 / 60.0
ROBOT_CLOCK_OFFSET_MS = 123_456
TAG_SIZE_M = 0.064
CAMERA = {"fx": 700.0, "fy": 700.0, "cx": 640.0, "cy": 360.0, "width": 1280, "height": 720}
ROBOT_RADIUS_M = 0.08

TRUTH = PlantParams(
    dz_lin_fwd=0.062,
    dz_lin_rev=0.051,
    dz_ang_l=0.071,
    dz_ang_r=0.056,
    k_fwd=5.21,
    k_rev=4.62,
    k_ang=55.3,
    tau_lin_a=0.061,
    tau_lin_d=0.094,
    tau_ang_a=0.052,
    tau_ang_d=0.087,
    delay_s=0.050,  # a whole number of 10 ms steps, so the plant's held delay is exact
    c_sb=0.24,
    c_ad=0.15,
    c_drift=0.0,
    c_drift_bias=0.8,
)


def camera_pose() -> np.ndarray:
    """T_field_camera: a tripod 1.6 m back and 1.8 m up, looking at the box center."""
    cam = np.array([0.0, -1.6, 1.8])
    forward = -cam / np.linalg.norm(cam)
    right = np.cross(forward, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    t = np.eye(4)
    t[:3, :3] = np.stack([right, down, forward], axis=1)
    t[:3, 3] = cam
    return t


@dataclass
class Session:
    telemetry: list[dict[str, Any]] = field(default_factory=list)  # with host_receive_ns
    frames: list[tuple[int, dict[str, Any]]] = field(default_factory=list)  # (log ns, payload)
    truth: list[tuple[float, float, float, float]] = field(default_factory=list)  # t, x, y, yaw
    contacts: int = 0


class _Driver:
    """Hands on the sticks. Returns (throttle, steer) fractions per 10 ms tick."""

    def __init__(self, rng: np.random.Generator, half: float) -> None:
        self.rng = rng
        self.half = half
        self.queue: list[tuple[float, float]] = []

    def _hold(self, seconds: float, thr: float, steer: float) -> None:
        self.queue += [(thr, steer)] * max(1, int(round(seconds / DT)))

    def _ramp(self, seconds: float, thr1: float, steer1: float) -> None:
        n = max(1, int(round(seconds / DT)))
        self.queue += [(thr1 * (i + 1) / n, steer1 * (i + 1) / n) for i in range(n)]

    def next(self, x: float, y: float, theta: float, v: float) -> tuple[float, float]:
        margin = self.half - max(abs(x), abs(y))
        heading_out = math.cos(theta) * x + math.sin(theta) * y
        if margin < 0.3 and v * heading_out > 0.2 and self.queue and self.queue[0][0] != 0.0:
            self.queue = []
            self._hold(0.12, 0.0, 0.0)
        if not self.queue:
            self._plan(x, y, theta)
        return self.queue.pop(0)

    def _plan(self, x: float, y: float, theta: float) -> None:
        r = self.rng
        err = math.atan2(-y, -x) - theta
        err = math.atan2(math.sin(err), math.cos(err))
        if max(abs(x), abs(y)) > self.half - 0.35 and abs(err) > 0.5:
            amp = 0.25 + 0.1 * r.random()
            self._hold(0.2 + 0.2 * r.random(), 0.0, 0.0)
            self._hold(
                max(0.03, abs(err) / (55.0 * (amp - 0.06) / 0.94)), 0.0, math.copysign(amp, err)
            )
            self._hold(0.6, 0.0, 0.0)
            return
        pick = r.random()
        if pick < 0.3:
            amp = 0.12 + 0.18 * r.random()
            self._hold(0.3, 0.0, 0.0)
            self._hold(0.25 + 0.35 * r.random(), amp if abs(err) < 1.2 else -amp, 0.0)
            self._hold(0.8, 0.0, 0.0)
        elif pick < 0.5:
            amp = (0.15 + 0.5 * r.random()) * (1 if r.random() < 0.5 else -1)
            self._hold(0.3, 0.0, 0.0)
            self._hold(0.6 + 0.4 * r.random(), 0.0, amp)
            self._hold(0.8, 0.0, 0.0)
        elif pick < 0.65:
            thr = 0.15 + 0.1 * r.random()
            self._hold(0.3, 0.0, 0.0)
            self._hold(0.7, thr, (0.3 + 0.3 * r.random()) * (1 if r.random() < 0.5 else -1))
            self._hold(0.8, 0.0, 0.0)
        elif pick < 0.8:
            ch = r.integers(2)
            sign = 1 if r.random() < 0.5 else -1
            self._hold(0.3, 0.0, 0.0)
            self._ramp(2.0, sign * 0.18 if ch == 0 else 0.0, sign * 0.18 if ch == 1 else 0.0)
            self._hold(0.8, 0.0, 0.0)
        else:
            for _ in range(3):
                self._hold(0.12, 0.45 if abs(err) < 1.2 else -0.45, 0.0)
                self._hold(0.6, 0.0, 0.0)


def _tag_payload(
    stamp_ns: int, t_field_body: np.ndarray, mount: Any, rng: np.random.Generator
) -> dict[str, Any] | None:
    t_cam_tag = np.linalg.inv(camera_pose()) @ t_field_body @ mount.t_body_tag
    r, t = t_cam_tag[:3, :3], t_cam_tag[:3, 3]
    s = TAG_SIZE_M / 2.0
    corners_tag = np.array([[-s, s, 0.0], [s, s, 0.0], [s, -s, 0.0], [-s, -s, 0.0]])
    pts = (r @ corners_tag.T).T + t
    if np.any(pts[:, 2] <= 0.0):
        return None
    u = CAMERA["fx"] * pts[:, 0] / pts[:, 2] + CAMERA["cx"]
    v = CAMERA["fy"] * pts[:, 1] / pts[:, 2] + CAMERA["cy"]
    return {
        "image_stamp_ns": stamp_ns,
        "frame_id": "camera",
        "camera": CAMERA,
        "tag_size_m": TAG_SIZE_M,
        "roi": None,
        "detections": [
            {
                "id": mount.tag_id,
                "corners": np.stack([u, v], axis=1).round(2).tolist(),
                "decision_margin": 60.0,
                "solutions": [
                    {
                        "rvec": Rotation.from_matrix(r).as_rotvec().tolist(),
                        "tvec": t.tolist(),
                        "reprojection_error_px": float(0.2 + 0.2 * rng.random()),
                    }
                ],
            }
        ],
    }


def simulate_session(
    profile: SysidProfile,
    seconds: float = 240.0,
    seed: int = 1,
    loop_on_from_s: float | None = None,
    truth: PlantParams = TRUTH,
    receive_jitter: bool = True,
) -> Session:
    """Drive for ``seconds``. The flip switch is middle (loop off) until ``loop_on_from_s``."""
    rng = np.random.default_rng(seed)
    upright = [m for m in profile.mounts.values() if not m.upside_down][0]
    half = profile.half_extent_m
    limit = half - ROBOT_RADIUS_M
    plant = Plant(truth)
    plant.reset(PlantState(0.0, -0.3, math.pi / 2, 0.0, 0.0))
    mixer = BatchedMixer(LoopState.idle(1))
    driver = _Driver(rng, half)
    out = Session()
    seq = 0
    last_rx = 0
    next_frame = 0.0
    pitch = Rotation.from_euler("y", profile.rest_pitch_rad).as_matrix()
    for k in range(int(round(seconds / DT))):
        t = k * DT
        st = plant.state
        x, y, th, v, w = (float(np.asarray(a)) for a in (st.x, st.y, st.theta, st.v, st.w))
        thr, steer = driver.next(x, y, th, v)
        a_pct, b_pct = -thr * 100.0, -steer * 100.0
        auto = loop_on_from_s is not None and t >= loop_on_from_s
        heading = heading_from_theta(np.array([th]))
        gyro = gyro_from_w(np.array([w])) + rng.normal(0.0, 0.5)
        left, right, _s = mixer.step(
            np.array([a_pct]), np.array([b_pct]), np.array([auto]), heading, gyro
        )
        u_lin, u_ang = motor_to_plant(left[0], right[0])
        plant.push_command(t, float(u_lin), float(u_ang))
        loop = mixer.s
        seq += 1
        # Serial delivery keeps the order; the radio loses an occasional frame.
        late = 0.004 + rng.exponential(0.002) if receive_jitter else 0.0
        receive_ns = max(T0_NS + int(round((t + late) * 1e9)), last_rx + 1000)
        if rng.random() > 0.002:
            last_rx = receive_ns
            out.telemetry.append(
                {
                    "host_receive_ns": receive_ns,
                    "timestamp_ms": ROBOT_CLOCK_OFFSET_MS + int(round(t * 1000)),
                    "loop_us": 900,
                    "a_percent": a_pct,
                    "b_percent": b_pct,
                    "left_cmd": float(left[0]),
                    "right_cmd": float(right[0]),
                    "accel_x": 0.0,
                    "accel_y": 0.0,
                    "accel_z": 9.8,
                    "orientation_x": round(float(heading[0]) * 16.0) / 16.0,
                    "orientation_y": math.degrees(profile.rest_pitch_rad),
                    "orientation_z": 0.0,
                    "pid_setpoint": float(loop.setpoint[0]),
                    "pid_output": float(_s[0]),
                    "vbat": 15.6,
                    "ibat": 2.0,
                    "yaw_rate": float(gyro[0]),
                    "yaw_rate_cmd": float(loop.rate_command[0]) if auto else None,
                    "radio_connected": True,
                    "armed": True,
                    "autonomy_enabled": False,
                    "button_state": False,
                    "flip_switch": 0 if auto else 1,
                    "is_upside_down": False,
                    "espnow_active": False,
                    "espnow_channel": 1,
                    "espnow_age_ms": None,
                    "espnow_seq": 0,
                    "jetson_tx_us": 0,
                    "espnow_lq": 100,
                    "telemetry_seq": seq,
                    "round_trip_us": None,
                }
            )
        # The camera runs at 60 Hz; a frame due within this tick is stamped at the tick, so its
        # pose is the state it carries.
        while next_frame <= t + 1e-9:
            drop = 0.03 + 0.55 * min(1.0, abs(w) / 45.0) ** 2
            if rng.random() > drop:
                body = np.eye(4)
                yaw = th + rng.normal(0.0, 0.008)
                body[:3, :3] = Rotation.from_euler("z", yaw).as_matrix() @ pitch
                body[:3, 3] = [x + rng.normal(0.0, 0.002), y + rng.normal(0.0, 0.002), 0.025]
                stamp = T0_NS + int(round(t * 1e9))
                payload = _tag_payload(stamp, body, upright, rng)
                if payload is not None:
                    out.frames.append((stamp + 30_000_000, payload))
            else:
                stamp = T0_NS + int(round(t * 1e9))
                out.frames.append(
                    (
                        stamp + 30_000_000,
                        {
                            "image_stamp_ns": stamp,
                            "frame_id": "camera",
                            "camera": CAMERA,
                            "tag_size_m": TAG_SIZE_M,
                            "roi": None,
                            "detections": [],
                        },
                    )
                )
            next_frame += FRAME_DT
        out.truth.append((t, x, y, th))
        plant.step(DT)
        s2 = plant.state
        nx, ny = float(np.asarray(s2.x)), float(np.asarray(s2.y))
        if abs(nx) > limit or abs(ny) > limit:
            out.contacts += 1
            plant.state = PlantState(
                float(np.clip(nx, -limit, limit)),
                float(np.clip(ny, -limit, limit)),
                float(np.asarray(s2.theta)),
                -0.15 * float(np.asarray(s2.v)),
                0.4 * float(np.asarray(s2.w)),
            )
    return out


def write_mcap(session: Session, path: Path | str, profile_name: str) -> Path:
    path = Path(path)
    cam = camera_pose()
    with mcap_write.McapWriter(path, active_profile=profile_name, allow_overwrite=True) as writer:
        events: list[tuple[int, int, Any]] = []
        for row in session.telemetry:
            events.append((row["host_receive_ns"], 0, row))
        for log_ns, payload in session.frames:
            events.append((log_ns, 1, payload))
        for k in range(0, len(session.frames), 30):
            events.append((session.frames[k][0], 2, None))
        events.sort(key=lambda e: (e[0], e[1]))
        for log_ns, kind, payload in events:
            if kind == 0:
                writer.log_json(ESPNOW_TELEMETRY_TOPIC, payload, log_ns)
            elif kind == 1:
                writer.log_json(ROBOT_TAGS_TOPIC, payload, log_ns)
            else:
                writer.log(
                    "/tf",
                    mcap_write.fg.FrameTransforms(
                        transforms=[
                            mcap_write.frame_transform(log_ns, "field", "camera_world", np.eye(4)),
                            mcap_write.frame_transform(log_ns, "camera_world", "camera", cam),
                        ]
                    ),
                    log_ns,
                )
    return path
