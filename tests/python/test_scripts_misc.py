from __future__ import annotations

import importlib.util
import ipaddress
import json
import re
import subprocess
import sys
from pathlib import Path
from types import ModuleType

import pytest

from auto_battlebot.recording import mcap_io, mcap_write

REPO = Path(__file__).resolve().parents[2]


def _load(name: str) -> ModuleType:
    spec = importlib.util.spec_from_file_location(
        f"scripts_{name}", REPO / "scripts" / f"{name}.py"
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def combine() -> ModuleType:
    return _load("combine_mcap_svo")


def _write_recording(path: Path, log_lines: list[str], frame_meta_paths: list[str]) -> None:
    with mcap_write.McapWriter(path, active_profile="unit_test") as writer:
        for i, line in enumerate(log_lines):
            entry = mcap_io.LogMessage(
                1_000 + i, mcap_io.LogLevel.INFO, "auto_battlebot", line, "main.cpp", 1
            )
            writer.log("/log", mcap_write.log_message(entry), 1_000 + i)
        for i, svo_path in enumerate(frame_meta_paths):
            writer.log_json(
                "/camera/frame_meta",
                {"image_stamp_ns": "1", "video_frame_index": i, "svo_path": svo_path},
                2_000 + i,
            )


def test_svo_path_regex_matches_cpp_log_lines(combine: ModuleType) -> None:
    resolved = "Resolved SVO path: /data/svo/a.svo2"
    started = "SVO recording started: /data/svo/b.svo2"
    assert combine.SVO_PATH_REGEX.search(resolved).group(1) == "/data/svo/a.svo2"
    assert combine.SVO_PATH_REGEX.search(started).group(1) == "/data/svo/b.svo2"


def test_extract_svo_paths_reads_log_and_frame_meta(combine: ModuleType, tmp_path: Path) -> None:
    recording = tmp_path / "rotated.mcap"
    _write_recording(
        recording,
        ["SVO recording started: /jetson/2026-08-28T20-57-35.svo2"],
        [
            "/jetson/2026-08-28T20-57-35.svo2",
            "/jetson/2026-08-28T21-03-09.svo2",
            "/jetson/2026-08-28T21-03-09.svo2",
        ],
    )
    paths = combine.extract_svo_paths(recording)
    assert paths == [
        Path("/jetson/2026-08-28T20-57-35.svo2"),
        Path("/jetson/2026-08-28T21-03-09.svo2"),
    ]


def test_extract_svo_paths_finds_rotation_missing_from_log(
    combine: ModuleType, tmp_path: Path
) -> None:
    recording = tmp_path / "rotated.mcap"
    _write_recording(recording, ["SVO recording started: /j/first.svo2"], ["/j/second.svo2"])
    assert Path("/j/second.svo2") in combine.extract_svo_paths(recording)


def test_resolve_ignores_zero_byte_svo(combine: ModuleType, tmp_path: Path) -> None:
    empty_dir = tmp_path / "a" / "2026-08-28"
    real_dir = tmp_path / "b"
    empty_dir.mkdir(parents=True)
    real_dir.mkdir()
    (empty_dir / "x.svo2").write_bytes(b"")
    (real_dir / "x.svo2").write_bytes(b"data")
    found = combine.resolve_svo_path(Path("/gone/x.svo2"), [tmp_path / "a", tmp_path / "b"])
    assert found == (real_dir / "x.svo2").resolve()


def test_resolve_earlier_search_dir_and_shallow_win(combine: ModuleType, tmp_path: Path) -> None:
    first = tmp_path / "first"
    second = tmp_path / "second"
    (first / "2026-08-23").mkdir(parents=True)
    (second).mkdir()
    (first / "2026-08-23" / "y.svo2").write_bytes(b"deep")
    (second / "y.svo2").write_bytes(b"later")
    found = combine.resolve_svo_path(Path("/gone/y.svo2"), [first, second])
    assert found == (first / "2026-08-23" / "y.svo2").resolve()

    (first / "y.svo2").write_bytes(b"shallow")
    found = combine.resolve_svo_path(Path("/gone/y.svo2"), [first, second])
    assert found == (first / "y.svo2").resolve()


def test_svo_time_range_comes_from_frame_stamps(
    combine: ModuleType, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(combine.svo2, "read_frame_stamps", lambda _p, _t: [30, 10, 20])
    assert combine.read_svo_time_range(Path("x.svo2"), "/t") == (10, 30)


@pytest.fixture(scope="module")
def convert() -> ModuleType:
    return _load("convert_ros1_mcap")


def test_legacy_diagnostic_values_become_typed(convert: ModuleType) -> None:
    assert convert.coerce_value("1") == 1
    assert isinstance(convert.coerce_value("1"), int)
    assert convert.coerce_value("12.500000") == 12.5
    assert convert.coerce_value("hello") == "hello"
    assert convert.coerce_value(7) == 7
    assert convert.coerce_value("nan") != convert.coerce_value("nan")


def test_engine_filename_carries_platform_tag(monkeypatch: pytest.MonkeyPatch) -> None:
    sync_models = _load("sync_models")
    monkeypatch.setattr(sync_models, "_get_compute_capability", lambda: (8, 7))
    monkeypatch.setattr(sync_models.platform, "machine", lambda: "aarch64")
    tagged = sync_models.engine_path_with_platform_tag(Path("models/model.engine"))
    assert tagged == Path("models/model_aarch64_sm87.engine")
    monkeypatch.setattr(sync_models, "_get_compute_capability", lambda: (8, 9))
    assert sync_models.engine_path_with_platform_tag(Path("m.engine")).name != tagged.name


def test_nhrl_source_videos_skip_annotated(tmp_path: Path) -> None:
    nhrl = _load("nhrl_to_recordings")
    (tmp_path / "annotated").mkdir()
    (tmp_path / "annotated" / "overlay.mp4").write_bytes(b"x")
    (tmp_path / "fight.mp4").write_bytes(b"x")
    assert nhrl.source_videos(tmp_path) == [tmp_path / "fight.mp4"]


def test_latency_window_starts_after_field_init() -> None:
    report = _load("mcap_latency_report")
    stage = report.FIELD_INIT_STAGE
    stage_t_ns = {stage: [100, 200, 300], "runner.tick": [50, 100, 150, 200]}
    stage_ms = {stage: [1.0, 2.0, 3.0], "runner.tick": [900.0, 800.0, 7.0, 8.0]}
    t0, rate_hz, note = report._window_after_field_init(
        Path("x.mcap"), stage_t_ns, stage_ms, [50, 150], [30.0, 31.0], first_ns=0
    )
    assert t0 == 100
    assert stage_t_ns["runner.tick"] == [150, 200]
    assert stage_ms["runner.tick"] == [7.0, 8.0]
    assert stage_t_ns[stage] == [200, 300]
    assert rate_hz == [31.0]
    assert note is not None


def _run_slop(tmp_path: Path, file: Path, content: str) -> subprocess.CompletedProcess[str]:
    file.parent.mkdir(parents=True, exist_ok=True)
    file.write_text(content)
    payload = json.dumps({"tool_input": {"file_path": str(file)}})
    return subprocess.run(
        ["bash", str(REPO / "scripts" / "no_ai_slop_check.sh")],
        input=payload,
        capture_output=True,
        text=True,
        check=False,
    )


def test_slop_check_flags_prose_with_line_numbers(tmp_path: Path) -> None:
    for name in ("note.md", "note.txt", "note.rst"):
        result = _run_slop(tmp_path, tmp_path / name, "fine\nwe leverage this\n")
        assert result.returncode == 2, name
        assert "2:we leverage this" in result.stderr, name


def test_slop_check_skips_claude_md_skills_and_data(tmp_path: Path) -> None:
    banned = "we leverage this\n"
    for rel in ("CLAUDE.md", ".claude/skills/x/SKILL.md", "data/notes.md"):
        assert _run_slop(tmp_path, tmp_path / rel, banned).returncode == 0, rel


def test_retimed_originals_overfetch_by_max_offset(combine: ModuleType, tmp_path: Path) -> None:
    recording = tmp_path / "orig.mcap"
    with mcap_write.McapWriter(recording, active_profile="unit_test") as writer:
        for log_time in (1_000, 1_100, 1_500):
            entry = mcap_io.LogMessage(
                log_time, mcap_io.LogLevel.INFO, "n", f"m{log_time}", "f.cpp", 1
            )
            writer.log("/log", mcap_write.log_message(entry), log_time)
    retimer = combine.HeaderStampRetimer([(1_000, 900), (1_500, 1_400)])
    buffered = combine._load_retimed_originals(recording, retimer, start_ns=800, end_ns=950)
    assert [item[0] for item in buffered] == sorted(item[0] for item in buffered)
    assert 900 in [item[0] for item in buffered]
    assert all(800 <= item[0] <= 950 for item in buffered)


def test_duplicate_svo_references_process_once(
    combine: ModuleType,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    caplog: pytest.LogCaptureFixture,
) -> None:
    svo = tmp_path / "same.svo2"
    svo.write_bytes(b"x")
    monkeypatch.setattr(
        combine, "extract_svo_paths", lambda _p: [Path("/a/same.svo2"), Path("/b/same.svo2")]
    )
    monkeypatch.setattr(combine, "get_input_profile", lambda _p: "foxglove")
    monkeypatch.setattr(combine, "resolve_svo_path", lambda _ref, _dirs: svo)
    with caplog.at_level("INFO"):
        written, missing = combine._combine_svos(
            tmp_path / "in.mcap",
            tmp_path,
            [tmp_path],
            overwrite=False,
            dry_run=True,
            ffmpeg_bin="ffmpeg",
            ignore_missing_svo=False,
        )
    assert (written, missing) == (0, 0)
    assert sum("DRY-RUN: would combine" in r.message for r in caplog.records) == 1


@pytest.mark.parametrize("script", ["run_playback.sh", "shell.sh"])
def test_docker_scripts_parse_no_display_before_sourcing_common(script: str) -> None:
    lines = (REPO / "scripts" / "docker" / script).read_text().splitlines()
    parse = next(i for i, line in enumerate(lines) if "AUTO_BATTLEBOT_NO_DISPLAY=1" in line)
    source = next(
        i for i, line in enumerate(lines) if line.startswith("source ") and "docker_common" in line
    )
    assert parse < source


def _function_body(path: Path, name: str) -> str:
    text = path.read_text()
    start = text.index(f"{name}() {{")
    return text[start : text.index("\n}\n", start)]


def test_installer_and_with_display_agree_on_desktop_detection() -> None:
    installer = _function_body(REPO / "service" / "build_and_install_service.sh", "boots_a_desktop")
    launcher = _function_body(REPO / "service" / "with_display.sh", "boots_a_desktop")
    assert installer == launcher


def test_dashboard_subnet_matches_between_installer_and_host_header() -> None:
    script = (REPO / "install" / "install_dashboard_network.sh").read_text()
    header = (REPO / "include" / "host" / "dashboard_network.hpp").read_text()
    address = re.search(r'local address="([0-9.]+/[0-9]+)"', script)
    net = re.search(r"kDashboardCableNet = (0x[0-9A-Fa-f]+);", header)
    mask = re.search(r"kDashboardCableMask = (0x[0-9A-Fa-f]+);", header)
    assert address and net and mask
    interface = ipaddress.ip_interface(address.group(1))
    header_net = ipaddress.ip_network(
        (int(net.group(1), 16), bin(int(mask.group(1), 16)).count("1"))
    )
    assert interface.network == header_net


def test_clang_tidy_step_promotes_warnings_to_errors() -> None:
    lint = (REPO / "scripts" / "lint").read_text()
    call = lint[lint.index('"$RUN_CLANG_TIDY" -p') :]
    assert "-warnings-as-errors='*'" in call.splitlines()[1]


def test_slop_check_reports_line_numbers_for_source_comments(tmp_path: Path) -> None:
    result = _run_slop(tmp_path, tmp_path / "tool.py", "x = 1\n# we leverage this\n")
    assert result.returncode == 2
    assert "2:# we leverage this" in result.stderr


def test_sync_models_never_overwrites_existing_files(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    sync_models = _load("sync_models")

    class Remote:
        def __init__(self, path: str) -> None:
            self.path = path
            self.id = path

    downloads: list[str] = []

    def fake_download(id: str, output: str, **_kwargs: object) -> None:
        downloads.append(id)
        Path(output).write_bytes(b"remote")

    monkeypatch.setattr(
        sync_models,
        "list_remote_files",
        lambda _url, _dir: [Remote("keep.engine"), Remote("new.engine")],
    )
    monkeypatch.setattr(sync_models.gdown, "download", fake_download)
    monkeypatch.setattr(sync_models, "get_models_dir", lambda: tmp_path)
    monkeypatch.setattr(sync_models, "print_platform_tag_matches", lambda _dir: None)
    (tmp_path / "keep.engine").write_bytes(b"local")
    sync_models.sync_models_cmd(tmp_path, "url")
    assert (tmp_path / "keep.engine").read_bytes() == b"local"
    assert (tmp_path / "new.engine").read_bytes() == b"remote"
    assert downloads == ["new.engine"]


def test_split_diagnostics_flags_a_module_with_more_messages_than_source_arrays(
    convert: ModuleType,
) -> None:
    span = convert.TopicSpan(count=5, first=10, last=90)
    prefix = convert.mcap_io.DIAGNOSTICS_TOPIC_PREFIX
    duplicated = {
        prefix + "a": convert.TopicSpan(count=6, first=10, last=90),
        prefix + "b": convert.TopicSpan(count=2, first=20, last=80),
    }
    problems = convert._split_diagnostics_problems("/diagnostics", span, duplicated)
    assert problems == ["/diagnostics: 5 arrays in, 6 on one module out"]


def test_split_diagnostics_accepts_modules_that_cover_the_source_span(convert: ModuleType) -> None:
    span = convert.TopicSpan(count=5, first=10, last=90)
    prefix = convert.mcap_io.DIAGNOSTICS_TOPIC_PREFIX
    split = {
        prefix + "a": convert.TopicSpan(count=5, first=10, last=70),
        prefix + "b": convert.TopicSpan(count=3, first=20, last=90),
    }
    assert convert._split_diagnostics_problems("/diagnostics", span, split) == []
    narrower = {prefix + "a": convert.TopicSpan(count=5, first=10, last=70)}
    assert len(convert._split_diagnostics_problems("/diagnostics", span, narrower)) == 1


def test_nhrl_set_clips_chain_without_overlapping_stamps() -> None:
    nhrl = _load("nhrl_to_recordings")
    clips = [(900, 30.0), (1800, 60.0), (450, 25.0)]
    start = nhrl.SET_EPOCH_STAMP_NS
    for frames, fps in clips:
        end = start + int(frames / fps * 1e9)
        following = nhrl.next_start_stamp_ns(start, frames, fps)
        assert following > end
        start = following
