"""Live sysid coverage monitor: firmware mirror, telemetry, segmenter, fit, session, WS client."""

from __future__ import annotations

import json
import threading
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from websockets.sync.server import serve

from auto_battlebot.calibration.sysid import model, synthetic, telemetry
from auto_battlebot.calibration.sysid.coverage import CoverageSession, _pitched
from auto_battlebot.calibration.sysid.maneuvers import ManeuverSegmenter
from auto_battlebot.calibration.sysid.pose_stream import TagPose
from auto_battlebot.calibration.sysid.profile import load_profile, load_profile_toml
from auto_battlebot.calibration.sysid.sources import Dispatcher, replay
from auto_battlebot.calibration.sysid.worker import InlineEngine
from auto_battlebot.control.firmware_mixer import BatchedMixer, LoopState
from auto_battlebot.mujoco_sim.firmware import FirmwareMixer
from auto_battlebot.recording import mcap_write
from auto_battlebot.recording.foxglove_ws import FoxgloveSubscriber
from auto_battlebot.recording.mcap_io import decode_json
from auto_battlebot.recording.sysid_io import parse_robot_tags


@pytest.fixture(scope="module")
def profile() -> Any:
    return load_profile()


# -- firmware mirror ---------------------------------------------------------------------


@pytest.mark.parametrize("auto", [False, True])
def test_batched_mixer_matches_scalar_mirror(auto: bool) -> None:
    rng = np.random.default_rng(3)
    n = 600
    # Held stick levels with snaps, idles and releases, plus a heading that turns.
    a = np.repeat(rng.choice([0.0, -30.0, 40.0, 3.0], n // 20), 20)
    b = np.repeat(rng.choice([0.0, 0.5, 25.0, -60.0], n // 20), 20)
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
    """The scalar mirror, run on a session's recorded sticks, heading and gyro, gives back the
    steer output the robot reported. On a real recording this pins the mirror to the firmware."""
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
    # The reported heading is quantized to 1/16 degree; the heading hold sees that.
    assert worst < 1.0


# -- telemetry and profile ---------------------------------------------------------------


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
    assert df["telemetry_seq"].isna().all()  # nullable, absent in this payload


def test_profile_extends_chain() -> None:
    data = load_profile_toml("mr_stabs_mk2_sysid_zed_box")
    assert data["navigation"]["type"] == "NoopNavigation"
    assert data["runner"]["autonomy_enabled_by_default"] is False
    prof = load_profile()
    assert prof.field_size_x_m == pytest.approx(1.52)
    assert any(m.upside_down for m in prof.mounts.values())


# -- segmenter ---------------------------------------------------------------------------


def test_segmenter_labels_snap_hold_release_and_creep() -> None:
    seg = ManeuverSegmenter(0.01)
    trace = [(0.0, 0.0)] * 60 + [(0.5, 0.0)] * 80 + [(0.0, 0.0)] * 80
    trace += [(0.0, 0.18 * k / 200) for k in range(200)] + [(0.0, 0.0)] * 60
    events = []
    for thr, steer in trace:
        events += seg.push(thr, steer, False)
    kinds = [(e.kind, e.text) for e in events if e.kind != "plateau"]
    assert ("edge", "Throttle snap to 50% forward") in kinds
    assert ("hold", "Forward hold 50%, 0.8 s") in kinds
    assert ("edge", "Throttle snap release from 50% forward") in kinds
    assert any(k == "creep" and "steer creep left" in t for k, t in kinds)
    snap = next(e for e in events if e.text == "Throttle snap to 50% forward")
    assert snap.edge_cell == (0, 1)


# -- fit ---------------------------------------------------------------------------------


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
            sw.gyro = sw.gyro + rng.normal(0, 0.1, sw.gyro.shape)
        subs += s
        owners += [i] * len(s)
    batch = model.stack(subs, owners, 0.3)
    result = model.fit(model.PRIOR_VECTOR.copy(), batch, max_nfev=15)
    sd = np.sqrt(np.diag(model.covariance(result.fisher, result.scale)))
    z = (result.theta - truth) / sd
    assert np.max(np.abs(z)) < 4.0, dict(zip(model.NAMES, np.round(z, 2)))


# -- session -----------------------------------------------------------------------------


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
    assert w["wall"] > 0  # the box is small and the driver hits it
    assert w["jetson"] == 0 and not snap["link"]["jetson_seen"]
    assert snap["refits"] >= 3
    assert snap["overall"] > 0.7
    assert any(e["status"] == "ok" for e in snap["events"])
    assert any("yaw loop" in r for r in snap["edge_rows"])
    assert snap["history"][-1]["hold"] < 0.03  # metres at 500 ms, closed loop
    json.dumps(snap, allow_nan=False)


def _poses(pitch_deg: np.ndarray) -> list[TagPose]:
    return [
        TagPose(
            stamp_ns=i * 16_666_667,
            x=0.0,
            y=0.0,
            yaw=0.0,
            pitch=float(np.radians(p)),
            sigma_along=0.01,
            sigma_across=0.01,
            sigma_yaw=0.01,
            upside_down=False,
        )
        for i, p in enumerate(pitch_deg)
    ]


def test_pitch_gate_ignores_single_frame_spikes() -> None:
    # The 2026-10-14 23-11-27 flat spin: 1.7 deg frame noise with lone spikes up to 7.8 deg.
    pitch = np.random.default_rng(0).normal(0.0, 1.5, 90)
    pitch[[20, 50, 51, 70]] = [-5.1, 7.8, 4.8, -5.8]
    assert not _pitched(_poses(pitch))
    pitch[30:45] = 6.0  # a quarter-second nose lift
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


# -- websocket client --------------------------------------------------------------------


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
    """Synthetic telemetry as the Jetson's ring poller publishes it on /robot/esp32_diagnostics.

    Each row lands at the first poll after it was produced, all rows of one poll stamped with that
    poll's arrival, and every row but a caught-up poll's newest marked backfill. No poll gets
    through during ``outage_s`` (session seconds), so that stretch arrives in one burst after it.
    """
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
    # The 3 s outage loses nothing: same frame drops, same link gaps, same windows used.
    assert ring.seq_drops == direct.seq_drops
    assert b["windows"]["link"] == a["windows"]["link"]
    assert abs(b["windows"]["used"] - a["windows"]["used"]) <= 1, (a["windows"], b["windows"])
    # Only the newest row of each poll reaches the clock, so its fit stays at radio latency
    # instead of the 100 ms a poll's oldest row waits.
    assert ring.clock.status is not None
    assert ring.clock.status.residual_p50_ms < 15.0, ring.clock.status


def test_from_diagnostics_renames_seq() -> None:
    assert telemetry.from_diagnostics({"seq": 7, "timestamp_ms": 1})["telemetry_seq"] == 7
    assert "telemetry_seq" not in telemetry.from_diagnostics({"seq": None, "timestamp_ms": 1})
    row = {"telemetry_seq": 3, "seq": 9}
    assert telemetry.from_diagnostics(row) is row
