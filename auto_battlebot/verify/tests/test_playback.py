"""Tests for the playback regression tiers in auto_battlebot.verify.playback."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest

from auto_battlebot.compat import tomllib
from auto_battlebot.recording import mcap_io, mcap_write
from auto_battlebot.verify import playback
from auto_battlebot.verify.cli import parse_args
from auto_battlebot.verify.suites import playback_group

pytestmark = pytest.mark.jetson

FRAME_NS = 33_333_333
FRAMES = 60


def diag(sections: dict[str, dict[str, Any]]) -> bytes:
    return json.dumps(sections).encode()


def section(values: dict[str, Any], level: int = 0, held: bool = False, age: float = 1.0) -> dict:
    return {"level": level, "message": "", "age_ms": age, "held": held, "values": values}


def robot_scene(robots: dict[str, tuple[float, float]]) -> Any:
    entities = []
    for index, (name, (x, y)) in enumerate(sorted(robots.items())):
        pose = mcap_io.Pose(position=(x, y, 0.0))
        color = mcap_io.Color()
        entities.append(
            mcap_io.SceneEntity(
                id=f"robot_bounds/{index}",
                frame_id="field",
                stamp_ns=0,
                cubes=[mcap_io.CubePrimitive(pose=pose, size=(0.2, 0.2, 0.1), color=color)],
            )
        )
        entities.append(
            mcap_io.SceneEntity(
                id=f"robot_labels/{index}",
                frame_id="field",
                stamp_ns=0,
                texts=[
                    mcap_io.TextPrimitive(
                        pose=pose,
                        billboard=True,
                        font_size=12.0,
                        scale_invariant=True,
                        color=color,
                        text=f"{name} (label)",
                    )
                ],
            )
        )
    return mcap_write.scene_update(mcap_io.SceneUpdate(entities=entities))


def write_clip(path: Path, linear: float = 0.5, dropped_frame: int | None = None) -> Path:
    """A clip MCAP: our robot every frame with one 1 m jump, the opponent on even frames."""
    with mcap_write.McapWriter(path, active_profile="unit_test") as writer:
        for i in range(FRAMES):
            t = (i + 1) * FRAME_NS
            if i == dropped_frame:
                continue
            writer.log_json(
                "/camera/frame_meta", {"image_stamp_ns": str(t), "video_frame_index": i}, t
            )
            our_x = 0.01 * i + (1.0 if i >= 30 else 0.0)
            robots = {"our_robot_1": (our_x, 0.0)}
            if i % 2 == 0:
                robots["their_robot_1"] = (1.0, 1.0)
            writer.log("/robot_markers", robot_scene(robots), t)
            writer.log_diagnostics(
                "playback_transmitter",
                {"process": section({"linear_out": linear, "angular_out": 0.0}, age=i)},
                t,
            )
            writer.log_diagnostics(
                "runner",
                {
                    "pipeline": section({"latency_ms": float(i)}),
                    "runner": section({"rate": 30.0 + i, "uptime_s": i}),
                    "tick": section({"elapsed_ms": 4.0 + i}),
                },
                t,
            )
            monitor = {"camera_frames_link": section({"total": i}, level=2 if i == 5 else 0)}
            monitor["relay_publish_link"] = section({"total": i * 7}, level=2)
            writer.log_diagnostics("performance_monitor", monitor, t)
            writer.log_json("/status/sticks", {"linear": float(i % 3)}, t, schema=("s", "{}"))
            writer.log_diagnostics("health", {"disk": section({"free_gb": 100.0 - i})}, t)
    return path


def test_canonical_diagnostics_drops_every_wall_time_field() -> None:
    data = diag(
        {
            "kept": section({"x": 1.0, "rate": 30.0, "uptime_s": 4, "publish.elapsed_ms": 2.0}),
            "stale": section({"x": 2.0}, held=True),
            "relay_publish_link": section({"total": 9}),
            "relay": section({"viz_dropped": 3, "y": 5}),
        }
    )
    assert json.loads(playback.canonical_diagnostics(data)) == {
        "kept": {"level": 0, "message": "", "values": {"x": 1.0}},
        "relay": {"level": 0, "message": "", "values": {"y": 5}},
    }
    later = diag(
        {
            "kept": section({"x": 1.0, "rate": 31.0, "uptime_s": 5}, age=99.0),
            "relay": section({"viz_dropped": 8, "y": 5}),
        }
    )
    assert playback.canonical_diagnostics(later) == playback.canonical_diagnostics(data)


def test_topic_hashes_skip_wall_time_topics_and_catch_payload_changes() -> None:
    base = [
        ("/robot_markers", b"a"),
        ("/diagnostics/kalman", diag({"k": section({"v": 1})})),
        ("/log", b"wall"),
        ("/status/system", b"10 Hz"),
        ("/diagnostics/health", b"{}"),
        ("/diagnostics/foxglove_publisher", b"{}"),
    ]
    hashes = playback.topic_hashes(base)
    assert sorted(hashes) == ["/diagnostics/kalman", "/robot_markers"]
    noisy = [(t, b"other" if t in ("/log", "/status/system") else d) for t, d in base]
    assert playback.topic_hashes(noisy) == hashes
    changed = [
        (t, diag({"k": section({"v": 2})}) if t == "/diagnostics/kalman" else d) for t, d in base
    ]
    assert playback.topic_hashes(changed)["/diagnostics/kalman"] != hashes["/diagnostics/kalman"]
    split = [("/robot_markers", b"ab"), ("/robot_markers", b"")]
    joined = [("/robot_markers", b"a"), ("/robot_markers", b"b")]
    assert playback.topic_hashes(split) != playback.topic_hashes(joined)


def test_digest_reads_tracks_commands_liveness_and_drops(tmp_path: Path) -> None:
    got = playback.digest_mcap(write_clip(tmp_path / "a.mcap", dropped_frame=10), "realtime")
    metrics = got.metrics
    assert metrics["frames"] == FRAMES - 1
    assert metrics["our_coverage"] == 1.0
    assert metrics["opponent_coverage"] == pytest.approx(29 / 59)
    minutes = (FRAMES - 1) / playback.CAMERA_FPS / 60.0
    assert metrics["our_jumps_per_min"] == pytest.approx(1 / minutes)
    assert metrics["opponent_jumps_per_min"] == 0.0
    assert metrics["linear_mean_abs"] == pytest.approx(0.5)
    assert metrics["moving_fraction"] == 1.0
    assert metrics["frame_drop_rate"] == pytest.approx(1 / FRAMES)
    latencies = [float(i) for i in range(FRAMES) if i != 10]
    assert metrics["latency_p95_ms"] == pytest.approx(playback.percentile(latencies, 0.95))
    assert got.red_checks == ["camera_frames_link", "relay_publish_link"]
    lockstep = playback.digest_mcap(tmp_path / "a.mcap", "lockstep")
    assert lockstep.red_checks == ["camera_frames_link"]


def test_two_writes_of_one_clip_hash_identical_and_a_command_change_does_not(
    tmp_path: Path,
) -> None:
    a = playback.digest_mcap(write_clip(tmp_path / "a.mcap"), "lockstep")
    b = playback.digest_mcap(write_clip(tmp_path / "b.mcap"), "lockstep")
    c = playback.digest_mcap(write_clip(tmp_path / "c.mcap", linear=0.6), "lockstep")
    assert a.hashes == b.hashes
    assert [t for t in a.hashes if a.hashes[t] != c.hashes[t]] == [
        "/diagnostics/playback_transmitter"
    ]


def digest(hashes: dict[str, str], **metrics: float) -> playback.Digest:
    base = {name: 0.5 for name in playback.CROSS_CLASS_TOLERANCE}
    base.update(metrics)
    return playback.Digest(hashes=hashes, metrics=base, red_checks=[])


def baselines_with(machine: str, clip: str, got: playback.Digest) -> dict[str, Any]:
    return {"lockstep": {machine: {clip: json.loads(json.dumps(got.__dict__))}}, "realtime": {}}


def test_compare_lockstep_fails_on_any_hash_change_on_the_same_machine_class() -> None:
    stored = digest({"/a": "1", "/b": "2"})
    baselines = baselines_with("m", "clip", stored)
    assert playback.compare_lockstep("clip", stored, baselines, "m") == ([], [])
    problems, _ = playback.compare_lockstep(
        "clip", digest({"/a": "1", "/b": "3"}, moving_fraction=0.5), baselines, "m"
    )
    assert problems == ["clip: 1 topics differ: /b"]
    problems, _ = playback.compare_lockstep("clip", digest({"/a": "1"}), baselines, "m")
    assert problems == ["clip: 1 topics differ: /b"]


def test_compare_lockstep_falls_back_to_metric_tolerance_on_another_machine_class() -> None:
    baselines = baselines_with("x86", "clip", digest({"/a": "1"}))
    near = digest({"/a": "other"}, opponent_coverage=0.53)
    problems, notes = playback.compare_lockstep("clip", near, baselines, "jetson")
    assert problems == []
    assert notes == ["clip: no jetson baseline; metrics checked against x86"]
    far = digest({"/a": "other"}, opponent_coverage=0.4)
    problems, _ = playback.compare_lockstep("clip", far, baselines, "jetson")
    assert len(problems) == 1 and problems[0].startswith("clip: opponent_coverage")
    problems, _ = playback.compare_lockstep("unknown", far, baselines, "jetson")
    assert problems == ["unknown: no lockstep baseline for any machine class"]


def realtime_digest(latency: float, drop: float, red: list[str] | None = None) -> playback.Digest:
    return playback.Digest(
        hashes={},
        metrics={"latency_p95_ms": latency, "frame_drop_rate": drop},
        red_checks=red or [],
    )


def test_realtime_bounds_widen_by_the_aa_spread_with_a_floor() -> None:
    runs = [realtime_digest(20.0, 0.0), realtime_digest(30.0, 0.01, ["imu_link"])]
    bounds = playback.realtime_bounds(runs)
    assert bounds["metrics"] == {"latency_p95_ms": [20.0, 30.0], "frame_drop_rate": [0.0, 0.01]}
    baselines = {"lockstep": {}, "realtime": {"m": {"clip": bounds}}}
    assert playback.compare_realtime("clip", realtime_digest(39.9, 0.03), baselines, "m") == []
    assert (
        playback.compare_realtime("clip", realtime_digest(10.5, 0.0, ["imu_link"]), baselines, "m")
        == []
    )
    slow = playback.compare_realtime("clip", realtime_digest(40.5, 0.0), baselines, "m")
    assert len(slow) == 1 and "latency_p95_ms" in slow[0]
    dropping = playback.compare_realtime("clip", realtime_digest(25.0, 0.05), baselines, "m")
    assert len(dropping) == 1 and "frame_drop_rate" in dropping[0]
    red = playback.compare_realtime(
        "clip", realtime_digest(25.0, 0.0, ["crsf_link"]), baselines, "m"
    )
    assert red == ["clip: new red checks ['crsf_link']"]
    assert playback.compare_realtime("clip", realtime_digest(25.0, 0.0), baselines, "other") == [
        "clip: no realtime baseline for other"
    ]


def clip(name: str = "clip") -> playback.Clip:
    return playback.Clip(name, "rec", "mrs_buff_mk3", 10, 0, Path("/x.svo2"))


def fake_runs(monkeypatch: pytest.MonkeyPatch, results: list[playback.Digest]) -> list[Path]:
    calls: list[Path] = []
    queue = list(results)

    def run_clip(c: playback.Clip, tier: str, out_dir: Path) -> Path:
        calls.append(out_dir)
        return out_dir

    monkeypatch.setattr(playback, "run_clip", run_clip)
    monkeypatch.setattr(playback, "digest_mcap", lambda path, tier: queue.pop(0))
    return calls


def test_lockstep_rerun_passes_only_when_the_rerun_matches(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    good = digest({"/a": "1"})
    bad = digest({"/a": "2"})
    baselines = baselines_with("m", "clip", good)

    calls = fake_runs(monkeypatch, [good])
    _, problems, notes = playback.lockstep_with_retry(clip(), tmp_path, baselines, "m")
    assert (problems, notes, len(calls)) == ([], [], 1)

    calls = fake_runs(monkeypatch, [bad, good])
    _, problems, notes = playback.lockstep_with_retry(clip(), tmp_path, baselines, "m")
    assert problems == [] and len(calls) == 2
    assert notes[0].startswith("clip: first run differed and the rerun matched")

    calls = fake_runs(monkeypatch, [bad, bad])
    _, problems, _ = playback.lockstep_with_retry(clip(), tmp_path, baselines, "m")
    assert problems[-1] == "clip: the rerun differs too" and len(calls) == 2

    calls = fake_runs(monkeypatch, [bad])
    _, problems, _ = playback.lockstep_with_retry(clip(), tmp_path, baselines, "other")
    assert len(calls) == 1


def test_clip_profile_extends_the_tier_profile_and_limits_frames(tmp_path: Path) -> None:
    c = playback.Clip("clip", "rec", "mrs_buff_mk3", 1800, 3000, Path("/data/rec.svo2"))
    lockstep = tomllib.loads(playback.clip_profile_text(c, "lockstep", tmp_path))
    assert lockstep["extends"] == "playback/mrs_buff_mk3_lockstep"
    assert lockstep["rgbd_camera"] == {
        "svo_file_path": "/data/rec.svo2",
        "svo_start_frame": 3000,
        "svo_max_frames": 1800,
    }
    assert lockstep["mcap"] == {"enable": True, "recordings_dir": str(tmp_path)}
    realtime = tomllib.loads(playback.clip_profile_text(c, "realtime", tmp_path))
    assert realtime["extends"] == "playback/mrs_buff_mk3_playback"


def test_load_clips_takes_start_frames_from_the_catalog(tmp_path: Path) -> None:
    catalog = tmp_path / "recordings.toml"
    catalog.write_text(
        '[recordings.a]\npath = "data/a.svo2"\nstart_frame = 700\n'
        '[recordings.b]\npath = "/abs/b.svo2"\n'
    )
    clips_file = tmp_path / "clips.toml"
    clips_file.write_text(
        '[clips.one]\nrecording = "a"\nrobot = "r"\nframes = 10\n'
        '[clips.two]\nrecording = "b"\nrobot = "r"\nframes = 20\nstart_frame = 5\n'
    )
    one, two = playback.load_clips(clips_file, catalog)
    assert (one.start_frame, one.svo_path) == (700, playback.REPO_ROOT / "data/a.svo2")
    assert (two.start_frame, two.svo_path) == (5, Path("/abs/b.svo2"))
    clips_file.write_text('[clips.bad]\nrecording = "missing"\nrobot = "r"\nframes = 1\n')
    with pytest.raises(ValueError, match="unknown recording 'missing'"):
        playback.load_clips(clips_file, catalog)


def test_every_committed_clip_resolves_and_has_a_lockstep_baseline() -> None:
    clips = playback.load_clips()
    assert len(clips) >= 3
    for c in clips:
        assert (playback.REPO_ROOT / "config" / f"{c.profile('lockstep')}.toml").exists()
        assert (playback.REPO_ROOT / "config" / f"{c.profile('realtime')}.toml").exists()
    stored = playback.load_baselines()["lockstep"]
    assert stored, "no lockstep baseline recorded"
    for machine, entries in stored.items():
        assert sorted(entries) == sorted(c.name for c in clips), machine


def test_prune_keeps_the_newest_runs(tmp_path: Path) -> None:
    for name in ["2026-01-01_a", "2026-01-02_a", "2026-01-03_a"]:
        (tmp_path / name).mkdir()
    playback.prune_runs(tmp_path, keep=2)
    assert sorted(p.name for p in tmp_path.iterdir()) == ["2026-01-02_a", "2026-01-03_a"]


def test_verify_cli_adds_playback_groups(tmp_path: Path) -> None:
    args = parse_args(["--playback", "--realtime"])
    assert args.playback and args.realtime
    group = playback_group(tmp_path, "lockstep")
    assert [step.name for step in group.steps] == [
        "release configure",
        "app build",
        "playback lockstep",
    ]
    assert group.steps[-1].command[1:] == [
        "-m",
        "auto_battlebot.verify.playback",
        "check",
        "--tier",
        "lockstep",
    ]
