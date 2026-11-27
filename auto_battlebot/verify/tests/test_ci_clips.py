from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path

import pytest

from auto_battlebot.paths import REPO_ROOT
from auto_battlebot.verify import ci_clips, ci_engines
from auto_battlebot.verify.playback import load_clips

pytestmark = pytest.mark.jetson


def test_committed_manifest_covers_every_clip_recording() -> None:
    entries = json.loads(ci_clips.MANIFEST.read_text())
    for clip in load_clips():
        assert str(clip.svo_path.relative_to(REPO_ROOT)) in entries
    assert all(len(entry["sha256"]) == 64 and entry["bytes"] > 0 for entry in entries.values())


def test_hash_cache_reuses_a_digest_until_the_file_changes(tmp_path: Path) -> None:
    file = tmp_path / "clip.svo2"
    file.write_bytes(b"one")
    cache = ci_clips.HashCache(tmp_path / "cache.json")
    first = cache.sha256(file)
    key = str(file.resolve())
    cache.entries[key][2] = "stale"
    assert cache.sha256(file) == "stale"
    file.write_bytes(b"two!")
    assert cache.sha256(file) == ci_clips.sha256_file(file) != first


def test_hash_cache_round_trips_through_its_file(tmp_path: Path) -> None:
    file = tmp_path / "clip.svo2"
    file.write_bytes(b"one")
    cache = ci_clips.HashCache(tmp_path / "cache.json")
    cache.sha256(file)
    cache.save()
    assert ci_clips.HashCache(tmp_path / "cache.json").entries == cache.entries


def test_problems_name_missing_resized_and_changed_files(tmp_path: Path) -> None:
    for name, body in {"ok": b"same", "resized": b"long body", "changed": b"abcd"}.items():
        (tmp_path / name).write_bytes(body)
    manifest = tmp_path / "manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "ok": {"bytes": 4, "sha256": ci_clips.sha256_file(tmp_path / "ok")},
                "resized": {"bytes": 4, "sha256": "x"},
                "changed": {"bytes": 4, "sha256": "0" * 64},
                "missing": {"bytes": 1, "sha256": "x"},
            }
        )
    )
    found = ci_clips.problems(tmp_path, manifest, ci_clips.HashCache(tmp_path / "c.json"))
    assert sorted(found) == ["missing missing", "sha256 differs changed", "size differs resized"]


def test_sync_writes_each_file_into_the_ci_data_dir_as_ci(tmp_path: Path) -> None:
    manifest = tmp_path / "manifest.json"
    manifest.write_text(json.dumps({"data/svo/a/b.svo2": {}, "data/models/m.onnx": {}}))
    commands = ci_clips.sync_commands("ben@box", manifest)
    destinations = [command[-1] for command in commands]
    assert destinations == [
        "ben@box:/home/ci/ci-data/svo/a/b.svo2",
        "ben@box:/home/ci/ci-data/models/m.onnx",
    ]
    assert all("--rsync-path=sudo -u ci rsync" in command for command in commands)


def target(tmp_path: Path) -> ci_engines.EngineTarget:
    onnx = tmp_path / "data" / "models" / "m.onnx"
    onnx.parent.mkdir(parents=True)
    onnx.write_bytes(b"onnx")
    return ci_engines.EngineTarget(
        onnx.parent / "m_aarch64_sm87.engine", (onnx,), ci_engines.YOLO_CLI
    )


def test_source_for_prefers_a_deeplab_checkpoint_then_onnx(tmp_path: Path) -> None:
    models = tmp_path / "data" / "models"
    models.mkdir(parents=True)
    for name in ("field.pth", "field.toml", "yolo.onnx"):
        (models / name).write_bytes(b"x")
    field = ci_engines.source_for(
        Path("data/models/field_aarch64_sm87.engine"), "aarch64_sm87", tmp_path
    )
    yolo = ci_engines.source_for(
        Path("data/models/yolo_aarch64_sm87.engine"), "aarch64_sm87", tmp_path
    )
    assert field is not None and field.cli == ci_engines.DEEPLAB_CLI
    assert [p.name for p in field.sources] == ["field.pth", "field.toml"]
    assert yolo is not None and yolo.cli == ci_engines.YOLO_CLI
    assert (
        ci_engines.source_for(Path("data/models/yolo_x86_64_sm89.engine"), "aarch64_sm87", tmp_path)
        is None
    )
    assert (
        ci_engines.source_for(
            Path("data/models/none_aarch64_sm87.engine"), "aarch64_sm87", tmp_path
        )
        is None
    )


def test_engine_candidates_are_found_in_any_section() -> None:
    profile = {
        "a": {"engine": {"candidates": ["x"]}},
        "b": {"c": {"engine": {"candidates": ["y"]}}},
        "d": 1,
    }
    assert list(ci_engines.engine_candidates(profile)) == ["x", "y"]


def patch_box(monkeypatch: pytest.MonkeyPatch, target: ci_engines.EngineTarget) -> None:
    monkeypatch.setattr(ci_engines, "required_engines", lambda tag, root: ([target], []))
    monkeypatch.setattr(ci_engines, "platform_tag", lambda: "aarch64_sm87")
    monkeypatch.setattr(ci_engines, "tensorrt_tag", lambda: "trt10.3")


def test_cache_hit_links_without_building(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    engine = target(tmp_path)
    patch_box(monkeypatch, engine)
    cached = ci_engines.cache_dir(engine, tmp_path / "cache", "trt10.3") / engine.engine.name
    cached.parent.mkdir(parents=True)
    cached.write_bytes(b"engine")

    def no_build(*args: object, **kwargs: object) -> None:
        raise AssertionError("built on a cache hit")

    monkeypatch.setattr(ci_engines.subprocess, "run", no_build)
    assert ci_engines.build(tmp_path / "cache", tmp_path) == 0
    assert engine.engine.is_symlink() and engine.engine.resolve() == cached.resolve()


def test_cache_miss_builds_into_the_cache_then_links(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    engine = target(tmp_path)
    patch_box(monkeypatch, engine)
    engine.engine.write_bytes(b"stale engine from another TensorRT")
    commands = []

    def fake_build(command: list[str], cwd: Path) -> subprocess.CompletedProcess[str]:
        commands.append(command)
        out = Path(command[command.index("-o") + 1])
        out.with_name(f"{out.stem}_aarch64_sm87.engine").write_bytes(b"engine")
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(ci_engines.subprocess, "run", fake_build)
    assert ci_engines.build(tmp_path / "cache", tmp_path) == 0
    assert len(commands) == 1 and commands[0][1].endswith(ci_engines.YOLO_CLI)
    timing = commands[0][commands[0].index("--timing-cache") + 1]
    assert timing == str(tmp_path / "cache" / "timing_trt10.3.cache")
    assert engine.engine.is_symlink()
    assert engine.engine.resolve().parent.name.endswith("_trt10.3")


def test_a_changed_source_gets_a_new_cache_entry(tmp_path: Path) -> None:
    engine = target(tmp_path)
    before = ci_engines.cache_dir(engine, tmp_path, "trt10.3")
    engine.sources[0].write_bytes(b"retrained")
    assert ci_engines.cache_dir(engine, tmp_path, "trt10.3") != before
    assert ci_engines.cache_dir(engine, tmp_path, "trt10.16") != ci_engines.cache_dir(
        engine, tmp_path, "trt10.3"
    )


def test_relinking_the_same_engine_keeps_the_link(tmp_path: Path) -> None:
    cached = tmp_path / "cached.engine"
    cached.write_bytes(b"e")
    engine = tmp_path / "m.engine"
    ci_engines.link(engine, cached)
    inode = os.lstat(engine).st_ino
    ci_engines.link(engine, cached)
    assert os.lstat(engine).st_ino == inode
