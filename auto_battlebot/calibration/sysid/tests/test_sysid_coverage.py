"""Live sysid coverage monitor: firmware mirror, telemetry, segmenter, fit, session, WS client."""

from __future__ import annotations

import ast
import dataclasses
import json
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from websockets.sync.server import serve

from auto_battlebot.calibration.sysid import model, synthetic, telemetry
from auto_battlebot.calibration.sysid.coverage import CoverageSession, WindowStatus, _pitched
from auto_battlebot.calibration.sysid.maneuvers import EventKind, ManeuverSegmenter, edges_of
from auto_battlebot.calibration.sysid.pose_stream import TagPose
from auto_battlebot.calibration.sysid.profile import load_profile
from auto_battlebot.calibration.sysid.sources import Dispatcher, live, replay
from auto_battlebot.calibration.sysid.worker import InlineEngine
from auto_battlebot.control.firmware_mixer import BatchedMixer, LoopState
from auto_battlebot.mujoco_sim.firmware import FirmwareMixer
from auto_battlebot.paths import REPO_ROOT
from auto_battlebot.recording import mcap_write
from auto_battlebot.recording.foxglove_ws import FoxgloveSubscriber
from auto_battlebot.recording.mcap_io import decode_json
from auto_battlebot.recording.sysid_io import parse_robot_tags

HEADING_QUANTIZATION_TOL = 0.05
THROTTLE_LEVELS = (0.0, -30.0, 40.0, 3.0)
TURN_LEVELS = (0.0, 0.5, 25.0, -60.0)
HOLD_M_AT_500MS_LIMIT = 0.03
NOSE_LIFT_FRAMES = int(0.25 * 60)


@pytest.fixture(scope="module")
def profile() -> Any:
    return load_profile()


@pytest.mark.parametrize("auto", [False, True])
def test_batched_mixer_matches_scalar_mirror(auto: bool) -> None:
    rng = np.random.default_rng(3)
    n = 600
    a = np.repeat(rng.choice(THROTTLE_LEVELS, n // 20), 20)
    b = np.repeat(rng.choice(TURN_LEVELS, n // 20), 20)
    heading = np.cumsum(rng.normal(0.0, 2.0, n)) % 360.0
    rate = rng.normal(0.0, 80.0, n)
    scalar = FirmwareMixer(auto_steer=auto)
    batched = BatchedMixer(LoopState.idle(1))
    for k in range(n):
        left_s, right_s = scalar.step(a[k], b[k], heading[k], 0.01, yaw_rate_dps=rate[k])
        left_b, right_b, _ = batched.step(
            a[k : k + 1], b[k : k + 1], np.array([auto]), heading[k : k + 1], rate[k : k + 1]
        )
        assert left_b[0] == pytest.approx(left_s, abs=1e-9)
        assert right_b[0] == pytest.approx(right_s, abs=1e-9)


def test_mirror_reproduces_recorded_pid_output(profile: Any) -> None:
    sim = synthetic.simulate_session(profile, seconds=20.0, seed=4, loop_on_from_s=5.0)
    mixer = FirmwareMixer(auto_steer=False)
    worst = 0.0
    for row in sim.telemetry:
        mixer.auto_steer = row["flip_switch"] == telemetry.FLIP_SWITCH_DOWN
        mixer.step(
            row["a_percent"],
            row["b_percent"],
            row["orientation_x"],
            0.01,
            yaw_rate_dps=row["yaw_rate"],
        )
        worst = max(worst, abs(mixer.pid_output - row["pid_output"]))
    assert worst < HEADING_QUANTIZATION_TOL


def test_telemetry_columns_follow_the_schema(tmp_path: Path) -> None:
    schema = json.loads(mcap_write.ESPNOW_TELEMETRY_JSON_SCHEMA)["properties"]
    assert [name for name, _ in telemetry.TELEMETRY_COLUMNS] == list(schema)
    path = tmp_path / "t.mcap"
    rows = [{"host_receive_ns": 10_000 + i, "timestamp_ms": i, "a_percent": None} for i in range(3)]
    with mcap_write.McapWriter(path) as writer:
        for row in rows:
            writer.log_json(telemetry.ESPNOW_TELEMETRY_TOPIC, row, row["host_receive_ns"])
    df = telemetry.load_telemetry(path)
    assert len(df) == 3
    assert df["a_percent"].isna().all()
    assert df["telemetry_seq"].isna().all()


def test_segmenter_labels_snap_hold_release_and_creep() -> None:
    seg = ManeuverSegmenter(0.01)
    trace = [(0.0, 0.0)] * 60 + [(0.5, 0.0)] * 80 + [(0.0, 0.0)] * 80
    trace += [(0.0, 0.18 * k / 200) for k in range(200)] + [(0.0, 0.0)] * 60
    events = []
    for thr, steer in trace:
        events += seg.push(thr, steer, False)
    kinds = [(e.kind, e.text) for e in events if e.kind != EventKind.PLATEAU]
    assert ("edge", "Throttle snap to 50% forward") in kinds
    assert ("hold", "Forward hold 50%, 0.8 s") in kinds
    assert ("edge", "Throttle snap release from 50% forward") in kinds
    assert any(k == "creep" and "steer creep left" in t for k, t in kinds)
    snap = next(e for e in events if e.text == "Throttle snap to 50% forward")
    assert snap.edge_cell == (0, 1)


def test_fit_recovers_truth_within_reported_sigma() -> None:
    truth = synthetic.TRUTH.to_vector(model.NAMES)
    rng = np.random.default_rng(0)
    subs, owners = [], []
    for i, motion in enumerate(model.CATALOG):
        s, _travel = model.expected_subwindows(truth, motion, False, (0.006, 0.002, 0.01))
        for sw in s:
            sw.tag_x = sw.tag_x + rng.normal(0, 0.005, sw.tag_x.shape)
            sw.tag_y = sw.tag_y + rng.normal(0, 0.005, sw.tag_y.shape)
            sw.tag_yaw = sw.tag_yaw + rng.normal(0, 0.02, sw.tag_yaw.shape)
            sw.gyro_ccw_radps = sw.gyro_ccw_radps + rng.normal(0, 0.1, sw.gyro_ccw_radps.shape)
        subs += s
        owners += [i] * len(s)
    batch = model.stack(subs, owners, 0.3)
    result = model.fit(model.PRIOR_VECTOR.copy(), batch, max_nfev=15)
    sd = np.sqrt(np.diag(model.covariance(result.fisher, result.residual_variance)))
    z = (result.theta - truth) / sd
    assert np.max(np.abs(z)) < 4.0, dict(zip(model.NAMES, np.round(z, 2)))


def _feed(session: CoverageSession, sim: synthetic.Session) -> None:
    session.feed_field(synthetic.camera_pose())
    events = [(r["host_receive_ns"], 0, r) for r in sim.telemetry]
    events += [(log, 1, p) for log, p in sim.frames]
    events.sort(key=lambda e: (e[0], e[1]))
    last = 0
    for log, kind, payload in events:
        if kind == 0:
            session.feed_telemetry(payload, log)
        else:
            session.feed_tags(parse_robot_tags(payload, log))
        if log - last > 100_000_000:
            session.advance()
            last = log
    session.advance()


def test_session_gates_and_converges(profile: Any) -> None:
    sim = synthetic.simulate_session(profile, seconds=75.0, seed=2, loop_on_from_s=40.0)
    session = CoverageSession(profile, InlineEngine())
    _feed(session, sim)
    snap = session.snapshot()
    w = snap["windows"]
    assert w["used"] >= 15 and w["holdout"] >= 3
    assert w["wall"] > 0, "synthetic box is small and the driver hits it"
    assert w["jetson"] == 0 and not snap["link"]["jetson_seen"]
    assert snap["refits"] >= 3
    assert snap["overall"] > 0.7
    assert any(e["status"] == "ok" for e in snap["events"])
    assert any("yaw loop" in r for r in snap["edge_rows"])
    hold_m_at_500ms = snap["history"][-1]["hold"]
    assert hold_m_at_500ms < HOLD_M_AT_500MS_LIMIT
    json.dumps(snap, allow_nan=False)


def _poses(pitch_deg: np.ndarray) -> list[TagPose]:
    return [
        TagPose(
            stamp_ns=i * 16_666_667,
            x=0.0,
            y=0.0,
            yaw=0.0,
            pitch_from_rest_rad=float(np.radians(p)),
            sigma_along=0.01,
            sigma_across=0.01,
            sigma_yaw=0.01,
            upside_down=False,
        )
        for i, p in enumerate(pitch_deg)
    ]


def test_pitch_gate_ignores_single_frame_spikes() -> None:
    pitch = np.random.default_rng(0).normal(0.0, 1.5, 90)
    pitch[[20, 50, 51, 70]] = [-5.1, 7.8, 4.8, -5.8]
    assert not _pitched(_poses(pitch))
    pitch[30 : 30 + NOSE_LIFT_FRAMES] = 6.0
    assert _pitched(_poses(pitch))
    assert not _pitched([])


def test_jetson_command_flag_drops_windows(profile: Any) -> None:
    sim = synthetic.simulate_session(profile, seconds=12.0, seed=5)
    for row in sim.telemetry[300:600]:
        row["espnow_active"] = True
    session = CoverageSession(profile, InlineEngine())
    _feed(session, sim)
    snap = session.snapshot()
    assert snap["link"]["jetson_seen"]
    assert snap["windows"]["jetson"] >= 2


def test_replay_from_mcap_matches_direct_feed(profile: Any, tmp_path: Path) -> None:
    sim = synthetic.simulate_session(profile, seconds=20.0, seed=6)
    path = synthetic.write_mcap(sim, tmp_path / "s.mcap", profile.name)
    session = CoverageSession(profile, InlineEngine())
    dispatch = Dispatcher(session)
    for topic, log, data in replay(path):
        dispatch(topic, log, data)
    session.advance()
    snap = session.snapshot()
    assert snap["link"]["field"]
    assert snap["windows"]["used"] + snap["windows"]["holdout"] >= 3


def test_live_source_subscribes_only_to_telemetry_topics_and_sends_nothing_else() -> None:
    received: list[Any] = []

    def handler(ws: Any) -> None:
        channels = [
            {
                "id": i,
                "topic": topic,
                "encoding": "json",
                "schemaName": "x",
                "schema": "{}",
            }
            for i, topic in enumerate(
                ["/control/command", "/robot/espnow_telemetry", "/remote/command"], start=1
            )
        ]
        ws.send(json.dumps({"op": "advertise", "channels": channels}))
        try:
            for message in ws:
                received.append(json.loads(message))
        except Exception:
            pass

    with serve(handler, "127.0.0.1", 0) as server:
        port = server.socket.getsockname()[1]
        threading.Thread(target=server.serve_forever, daemon=True).start()
        sub, _messages = live(f"ws://127.0.0.1:{port}")
        threading.Thread(target=lambda: next(_messages, None), daemon=True).start()
        deadline = time.monotonic() + 3.0
        while not received and time.monotonic() < deadline:
            time.sleep(0.02)
        sub.close()
        server.shutdown()
    assert [m["op"] for m in received] == ["subscribe"]
    assert [s["channelId"] for s in received[0]["subscriptions"]] == [2]


def test_subscriber_only_subscribes() -> None:
    """The live client sends subscribe and nothing else, and decodes message frames."""
    received: list[Any] = []
    payload = json.dumps({"timestamp_ms": 7}).encode()

    def handler(ws: Any) -> None:
        ws.send(
            json.dumps(
                {
                    "op": "advertise",
                    "channels": [
                        {
                            "id": 4,
                            "topic": "/robot/espnow_telemetry",
                            "encoding": "json",
                            "schemaName": "auto_battlebot.EspNowTelemetry",
                            "schema": "{}",
                        },
                        {
                            "id": 5,
                            "topic": "/control/command",
                            "encoding": "json",
                            "schemaName": "x",
                            "schema": "{}",
                        },
                    ],
                }
            )
        )
        msg = json.loads(ws.recv())
        received.append(msg)
        sub_id = msg["subscriptions"][0]["id"]
        header = bytes([1]) + sub_id.to_bytes(4, "little") + (123).to_bytes(8, "little")
        ws.send(header + payload)
        try:
            for extra in ws:
                received.append(extra)
        except Exception:
            pass

    with serve(handler, "127.0.0.1", 0) as server:
        port = server.socket.getsockname()[1]
        threading.Thread(target=server.serve_forever, daemon=True).start()
        sub = FoxgloveSubscriber(f"ws://127.0.0.1:{port}", ["/robot/espnow_telemetry"])
        topic, log, data = next(sub.messages())
        sub.close()
        server.shutdown()
    assert (topic, log) == ("/robot/espnow_telemetry", 123)
    assert decode_json(data) == {"timestamp_ms": 7}
    assert received[0]["op"] == "subscribe"
    assert [s["channelId"] for s in received[0]["subscriptions"]] == [4]
    assert len(received) == 1


def _ring_delivery(
    rows: list[dict[str, Any]], poll_s: float, outage_s: tuple[float, float]
) -> list[dict[str, Any]]:
    """Synthetic telemetry as the Jetson ring poller delivers it, bursting after ``outage_s``."""
    t0 = rows[0]["host_receive_ns"]
    poll_ns = int(poll_s * 1e9)
    lo, hi = (t0 + int(s * 1e9) for s in outage_s)
    batches: dict[int, list[dict[str, Any]]] = {}
    for r in rows:
        k = -(-(r["host_receive_ns"] - t0) // poll_ns)
        at = t0 + k * poll_ns
        if lo <= at < hi:
            at = t0 + -(-(hi - t0) // poll_ns) * poll_ns
        d = {
            key: v for key, v in r.items() if not key.startswith("espnow") and key != "jetson_tx_us"
        }
        d["seq"] = d.pop("telemetry_seq")
        d["wifi_clients"] = 0
        batches.setdefault(at, []).append(d)
    out = []
    for at in sorted(batches):
        batch = batches[at]
        for i, d in enumerate(batch):
            d["host_receive_ns"] = at + 3_000_000
            d["backfill"] = i + 1 < len(batch)
            out.append(telemetry.from_diagnostics(d))
    return out


def test_ring_backfill_matches_live_delivery(profile: Any) -> None:
    sim = synthetic.simulate_session(profile, seconds=60.0, seed=3, loop_on_from_s=30.0)
    direct = CoverageSession(profile, InlineEngine())
    _feed(direct, sim)

    sim.telemetry = _ring_delivery(sim.telemetry, poll_s=0.1, outage_s=(20.0, 23.0))
    assert sum(r["backfill"] for r in sim.telemetry) > 0.8 * len(sim.telemetry)
    ring = CoverageSession(profile, InlineEngine())
    _feed(ring, sim)

    a, b = direct.snapshot(), ring.snapshot()
    assert ring.seq_drops == direct.seq_drops
    assert b["windows"]["link"] == a["windows"]["link"]
    assert abs(b["windows"]["used"] - a["windows"]["used"]) <= 1, (a["windows"], b["windows"])
    assert ring.clock.status is not None
    assert ring.clock.status.residual_p50_ms < 15.0, ring.clock.status


def test_from_diagnostics_renames_seq() -> None:
    assert telemetry.from_diagnostics({"seq": 7, "timestamp_ms": 1})["telemetry_seq"] == 7
    assert "telemetry_seq" not in telemetry.from_diagnostics({"seq": None, "timestamp_ms": 1})
    row = {"telemetry_seq": 3, "seq": 9}
    assert telemetry.from_diagnostics(row) is row


def test_sysid_package_never_sends_commands() -> None:
    banned = (
        "serial",
        "socket",
        "auto_battlebot.calibration.jig",
        "auto_battlebot.mujoco_sim",
        "auto_battlebot.rl",
    )
    package = REPO_ROOT / "auto_battlebot" / "calibration" / "sysid"
    offenders = []
    for path in package.glob("*.py"):
        for node in ast.walk(ast.parse(path.read_text())):
            names = (
                [a.name for a in node.names]
                if isinstance(node, ast.Import)
                else [node.module or ""]
                if isinstance(node, ast.ImportFrom)
                else []
            )
            offenders += [
                f"{path.name}: {n}"
                for n in names
                if any(n == b or n.startswith(b + ".") for b in banned)
            ]
    assert not offenders


def _plain_row(robot_ms: int, host_ns: int, **extra: Any) -> dict[str, Any]:
    return {"timestamp_ms": robot_ms, "host_receive_ns": host_ns, **extra}


def test_session_survives_robot_reboot(profile: Any) -> None:
    session = CoverageSession(profile, InlineEngine())
    host0 = 10_000_000_000
    for i in range(200):
        session.feed_telemetry(_plain_row(500_000 + 10 * i, host0 + i * 10_000_000), 0)
    host1 = host0 + 3_000_000_000
    for i in range(200):
        session.feed_telemetry(_plain_row(20 + 10 * i, host1 + i * 10_000_000), 0)
    robot_ms = [t.robot_ms for t in session._tele]
    assert robot_ms == sorted(robot_ms)
    assert len(robot_ms) == 200
    expected = host1 + 199 * 10_000_000
    mapped = int(session.clock.to_host_ns(20 + 10 * 199))
    assert abs(mapped - expected) < 5_000_000


def test_backfill_does_not_push_clock(profile: Any) -> None:
    session = CoverageSession(profile, InlineEngine())
    pushed: list[int] = []
    push = session.clock.push

    def spy(robot_ms: int, host_ns: int) -> None:
        pushed.append(robot_ms)
        push(robot_ms, host_ns)

    session.clock.push = spy  # type: ignore[method-assign]
    host0 = 10_000_000_000
    session.feed_telemetry(_plain_row(1000, host0), 0)
    for i in range(1, 6):
        late = host0 + i * 10_000_000 + 400_000_000
        session.feed_telemetry(_plain_row(1000 + 10 * i, late, backfill=True), 0)
    session.feed_telemetry(_plain_row(1060, host0 + 60_000_000), 0)
    assert pushed == [1000, 1060]
    assert len(session._tele) == 7


def test_gyro_interpolated_commands_held(profile: Any) -> None:
    session = CoverageSession(profile, InlineEngine())
    host0 = 10_000_000_000
    session.feed_telemetry(_plain_row(0, host0, yaw_rate=0.0, a_percent=0.0), 0)
    session.feed_telemetry(_plain_row(20, host0 + 20_000_000, yaw_rate=100.0, a_percent=50.0), 0)
    row = session._sample(host0 + 10_000_000)
    assert row is not None
    assert row.gyro_cw_dps == pytest.approx(50.0, abs=1.0)
    assert row.a == 0.0


def test_windows_closed_during_refit_count_toward_fisher_new(profile: Any) -> None:
    class Recording(InlineEngine):
        replies: list[Any] = []

        def poll(self) -> Any:
            reply = super().poll()
            if reply is not None:
                self.replies.append(reply)
            return reply

    engine = Recording()
    engine.replies = []
    session = CoverageSession(profile, engine)
    _feed(session, synthetic.simulate_session(profile, seconds=40.0, seed=2))
    assert engine.replies
    used = [w.id for w in session.windows if w.status == WindowStatus.USED]
    stale = dataclasses.replace(engine.replies[-1], upto=used[0])
    session._apply_refit(stale)
    assert np.trace(session.fisher_since_refit) > 0.0
    current = dataclasses.replace(engine.replies[-1], upto=used[-1])
    session._apply_refit(current)
    assert np.trace(session.fisher_since_refit) == 0.0


def test_arc_reported_once_from_throttle_side() -> None:
    seg = ManeuverSegmenter(0.01)
    trace = [(0.0, 0.0)] * 60 + [(0.5, 0.5)] * 150 + [(0.0, 0.0)] * 80
    events = []
    for thr, steer in trace:
        events += seg.push(thr, steer, False)
    motions = [e.kind for e in events if e.kind in (EventKind.HOLD, EventKind.ARC, EventKind.SPIN)]
    assert motions == [EventKind.ARC]


def test_offline_edges_include_final_hold() -> None:
    throttle = np.array([0.0] * 60 + [0.5] * 20)
    steer = np.zeros_like(throttle)
    edges = edges_of(throttle, steer, np.zeros(len(throttle), dtype=bool), 0.01)
    assert [e.text for e in edges] == ["Throttle snap to 50% forward"]


def test_live_client_sources_only_name_subscribe_ops() -> None:
    import re

    root = REPO_ROOT
    files = [
        root / "auto_battlebot/recording/foxglove_ws.py",
        root / "auto_battlebot/calibration/sysid/sources.py",
        root / "auto_battlebot/calibration/sysid/sysid_coverage.py",
    ]
    ops = {op for path in files for op in re.findall(r'"op":\s*"(\w+)"', path.read_text())}
    assert ops == {"subscribe"}
