"""Tests for reach discovery, the registry, proof ids, lint rules, draft, and the map builder."""

from __future__ import annotations

import json
import runpy
import shutil
import subprocess
from pathlib import Path
from typing import Any

import pytest

from auto_battlebot.features import (
    check,
    draft,
    feature_map,
    proofs,
    reach,
    registry,
    serve,
    staleness,
)
from auto_battlebot.paths import REPO_ROOT
from auto_battlebot.verify.run_store import ProofResult, Run

PROTOCOL = """
struct RemoteTopics {
    static constexpr auto status = std::tuple{
        StatusTopic<SystemStatusMessage>{"/status/system", Latch::NO, 10.0},
    };
    static constexpr auto commands = std::tuple{
        CommandTopic<SetAutonomyCommand>{"/command/set_autonomy"},
    };
};
"""
VIZ = """
inline constexpr auto kVizTopics = std::array{
    VizTopic{VizTopicId::TF, "/tf", "protobuf", false, TopicRate::PER_FRAME, true},
    VizTopic{VizTopicId::LOG, "/log", "protobuf", false, TopicRate::EVENT, true},
};
"""
CHANNELS = """# CRSF channel convention

Sticks:

| Control | Stick | Function | Output |
| ------- | ----- | -------- | ------ |
| Ele | right stick vertical | drive forward/back | ch0 |

Switches:

| Switch | Function | Detail |
| ------ | -------- | ------ |
| SA | enable autonomy | SA down |
"""
PERFORMANCE = "enum class PerformanceCheck {\n    CAMERA_FRAMES_LINK,\n    SLEW_BINDING,\n};\n"
LINKS = "enum class MonitorLink {\n    CAMERA_FRAMES,\n    TRANSMIT,\n};\n"


def write(root: Path, rel: str, text: str = "") -> Path:
    path = root / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    return path


@pytest.fixture
def tree(tmp_path: Path) -> Path:
    write(tmp_path, reach.PROTOCOL_HEADER, PROTOCOL)
    write(tmp_path, reach.VIZ_TOPICS_HEADER, VIZ)
    write(tmp_path, reach.CHANNEL_CONVENTION, CHANNELS)
    write(tmp_path, proofs.PERFORMANCE_HEADER, PERFORMANCE)
    write(tmp_path, proofs.LINK_HEADER, LINKS)
    write(
        tmp_path,
        "src/transmitter/config.cpp",
        "REGISTER_CONFIG(TransmitterConfiguration, NoopTransmitterConfiguration,"
        ' "NoopTransmitter")\n'
        "REGISTER_CONFIG(TransmitterConfiguration, EspNowTransmitterConfiguration,\n"
        '                "EspNowTransmitter")\n',
    )
    write(tmp_path, "src/transmitter/espnow.cpp", "SetAutonomyCommand handled here\n")
    write(tmp_path, "auto_battlebot/__init__.py")
    write(tmp_path, "auto_battlebot/tool.py", "def main():\n    return 0\n")
    write(tmp_path, "auto_battlebot/lib.py", "def helper():\n    return 0\n")
    write(tmp_path, "auto_battlebot/wrapped.py", "def main():\n    return 0\n")
    write(
        tmp_path,
        "training/run_wrapped.py",
        "import sys\n\nfrom auto_battlebot.wrapped import main\n\nsys.exit(main())\n",
    )
    write(
        tmp_path,
        "auto_battlebot/guarded.py",
        'def main():\n    return 0\n\n\nif __name__ == "__main__":\n    main()\n',
    )
    write(tmp_path, "training/also_guarded.py", "from auto_battlebot.guarded import main\n")
    write(tmp_path, "auto_battlebot/tests/test_tool.py", "def main():\n    pass\n")
    write(tmp_path, "scripts/build.sh", "#!/bin/bash\n")
    write(tmp_path, "scripts/README.md", "docs\n")
    write(
        tmp_path,
        "web/src/components/Battery.svelte",
        "<script>import { status } from '../lib/status.svelte';</script>\n",
    )
    write(tmp_path, "web/src/components/Logo.svelte", "<svg></svg>\n")
    write(tmp_path, "config/robot_zed_box.toml", "")
    write(tmp_path, "config/_common.toml", "")
    write(tmp_path, "config/profiles.toml", "")
    write(tmp_path, "service/app.service", "")
    write(
        tmp_path,
        "tests/test_espnow.cpp",
        "TEST(EspNow, SubstitutesUnderSa) {}\nTEST_F(Fixture, Case) {}\n",
    )
    write(
        tmp_path,
        "firmware/mr_stabs_mk2/test/test_pid/test_main.cpp",
        "TEST(Pid, ProportionalOnError) {}\n",
    )
    write(
        tmp_path,
        "web/src/lib/status.test.ts",
        'describe("x", () => {\n  it("renders the battery", () => {});\n});\n',
    )
    write(
        tmp_path,
        "auto_battlebot/tests/test_lib.py",
        "def test_helper():\n    pass\n\n"
        "class TestGroup:\n    def test_inner(self):\n        pass\n",
    )
    write(
        tmp_path,
        proofs.PLAYBACK_CLIPS,
        '[clips.clyde]\nrecording = "r"\nrobot = "b"\nframes = 10\n',
    )
    write(tmp_path, proofs.HIL_CHECKLIST, '[[steps]]\nid = "flip_sa"\nprompt = "p"\nexpect = "e"\n')
    return tmp_path


def ids(points: list[reach.ReachPoint]) -> set[str]:
    return {p.id for p in points}


def test_discover_reads_every_source(tree: Path) -> None:
    found = ids(reach.discover(tree))
    assert found == {
        "status:/status/system",
        "command:/command/set_autonomy",
        "type:NoopTransmitter",
        "type:EspNowTransmitter",
        "cli:auto_battlebot.tool",
        "cli:training/run_wrapped.py",
        "cli:auto_battlebot.guarded",
        "cli:scripts/build.sh",
        "channel:Ele",
        "channel:SA",
        "readout:Battery",
        "profile:robot_zed_box",
        "unit:app",
        "topic:/tf",
        "topic:/log",
    }


def test_multiline_register_config_keeps_its_line(tree: Path) -> None:
    types = {t.name: t for t in reach.factory_types(tree)}
    assert types["EspNowTransmitter"].line == 2
    assert types["EspNowTransmitter"].base == "TransmitterConfiguration"


def test_registry_reports_shape_errors() -> None:
    reg = registry.parse(
        'enforce = "yes"\nbogus = 1\n[features.a]\nname = "A"\n'
        'status = "done"\nproofs = "x"\nextra = 1\n'
    )
    joined = "\n".join(reg.errors)
    assert "enforce must be true or false" in joined
    assert "unknown top-level keys ['bogus']" in joined
    assert "status must be one of" in joined
    assert "features.a.proofs must be a list of strings" in joined
    assert "unknown fields ['extra']" in joined


def test_variants_claim_their_types() -> None:
    reg = registry.parse(
        '[features.tx]\nname = "Transmit"\nvariants = { EspNowTransmitter = ["gtest:A.B"] }\n'
    )
    feature = reg.features["tx"]
    assert feature.claims() == ["type:EspNowTransmitter"]
    assert feature.all_proofs() == ["gtest:A.B"]


def test_resolver_checks_each_runner(tree: Path) -> None:
    resolver = proofs.Resolver(tree)
    good = [
        "gtest:EspNow.SubstitutesUnderSa",
        "gtest:Fixture.Case",
        "pytest:auto_battlebot/tests/test_lib.py::test_helper",
        "pytest:auto_battlebot/tests/test_lib.py::TestGroup::test_inner",
        "pytest:auto_battlebot/tests/test_lib.py::test_helper[param]",
        "vitest:web/src/lib/status.test.ts::renders the battery",
        "native:test_pid::Pid.ProportionalOnError",
        "playback:clyde:opponent_coverage",
        "playback:clyde:hashes",
        "mcap:slew_binding",
        "mcap:topic_rate:/tf",
        "mcap:topic_rate:/status/system",
        "hil:flip_sa",
    ]
    assert resolver.problems(good) == []
    bad = [
        "gtest:EspNow.Missing",
        "pytest:auto_battlebot/tests/test_lib.py::test_missing",
        "pytest:nowhere.py::test_x",
        "native:test_pid::Pid.Missing",
        "playback:sphinx:hashes",
        "mcap:not_a_check",
        "hil:flip_sb",
        "sim:attack:reaches_target",
        "nonsense",
        "unknown:x",
    ]
    assert len(resolver.problems(bad)) == len(bad)
    assert "smoke tier has no scenarios" in resolver.problem("sim:attack:reaches_target")


def test_check_reports_unclaimed_and_reference_errors(tree: Path) -> None:
    reg = registry.parse(
        "\n".join(
            [
                "[features.tx]",
                'name = "Transmit"',
                'reach = ["command:/command/set_autonomy", "command:/command/missing",'
                ' "type:NoopTransmitter"]',
                "variants = { EspNowTransmitter = [], Bogus = [] }",
                'code = ["src/transmitter/", "src/missing.cpp"]',
                'proofs = ["gtest:EspNow.Missing"]',
                'links = ["transmit", "warp_drive"]',
                'docs = ["docs/missing.md"]',
                "[features.dup]",
                'name = "Dup"',
                'reach = ["command:/command/set_autonomy"]',
                'proofs = ["hil:flip_sa"]',
                'status = "reviewed"',
                "[ignored]",
                '"channel:Ele" = "no function"',
                '"channel:Nope" = "gone"',
            ]
        )
    )
    report = check.check(tree, reg)
    errors = "\n".join(report.errors)
    for expected in (
        "command:/command/missing is not a reach point",
        "list type:NoopTransmitter under variants instead",
        "Bogus is not a REGISTER_CONFIG type",
        "src/missing.cpp does not exist",
        "gtest:EspNow.Missing: no such gtest proof",
        "warp_drive is not a MonitorLink",
        "docs/missing.md does not exist",
        "command:/command/set_autonomy is claimed by 2 features",
        "ignored: channel:Nope is not a reach point",
    ):
        assert expected in errors
    unclaimed = {p.id for p in report.unclaimed}
    assert "channel:SA" in unclaimed
    assert "channel:Ele" not in unclaimed
    assert "type:EspNowTransmitter" not in unclaimed
    assert report.failed()


def test_gaps_fail_only_when_enforced(tree: Path) -> None:
    lenient = check.check(tree, registry.parse("enforce = false\n"))
    assert lenient.gaps and not lenient.errors and not lenient.failed()
    strict = check.check(tree, registry.parse("enforce = true\n"))
    assert strict.failed()
    assert all(line.startswith("error:") for line in strict.lines())


def test_draft_claims_everything_and_stays_proofless(tree: Path) -> None:
    write(tree, registry.FEATURES_FILE, "enforce = false\n")
    current = registry.load(tree)
    report = check.check(tree, current)
    written = draft.draft(tree, current, report.unclaimed)
    assert written == len(report.unclaimed)
    after = registry.load(tree)
    assert after.errors == []
    rechecked = check.check(tree, after)
    assert rechecked.unclaimed == [] and rechecked.errors == []
    assert any("has no entry-point proof" in gap for gap in rechecked.gaps)
    assert after.features["type_espnowtransmitter"].variants == {"EspNowTransmitter": []}
    assert after.features["command_command_set_autonomy"].code == ["src/transmitter/espnow.cpp"]
    assert draft.draft(tree, after, []) == 0


def test_the_repo_registry_has_no_errors() -> None:
    report = check.check(REPO_ROOT, registry.load(REPO_ROOT))
    assert report.errors == []


def run(stamp: str, commit: str, results: dict[str, str], diff: str = "") -> Run:
    return Run(stamp, commit, diff, "t", [ProofResult(k, v, 1.0) for k, v in results.items()])


class FakeChanges(staleness.ChangedFiles):
    def __init__(self, changed: dict[str, list[str] | None]) -> None:
        self.changed = changed

    def since(self, commit: str) -> list[str] | None:
        return self.changed.get(commit, [])


def test_pass_goes_stale_only_when_its_scope_changes() -> None:
    runs = [
        run("1", "aaa", {"gtest:A.B": "passed", "gtest:C.D": "passed", "gtest:E.F": "failed"}),
    ]
    scopes = {"gtest:A.B": ["src/a/"], "gtest:C.D": ["src/c.cpp"]}
    states = staleness.proof_states(
        runs, scopes, FakeChanges({"aaa": ["src/a/x.cpp"]}), ("bbb", "")
    )
    assert states["gtest:A.B"].state == "stale"
    assert states["gtest:A.B"].changed == ["src/a/x.cpp"]
    assert states["gtest:C.D"].state == "passing"
    assert states["gtest:E.F"].state == "failed"
    same = staleness.proof_states(runs, scopes, FakeChanges({"aaa": ["src/a/x.cpp"]}), ("aaa", ""))
    assert same["gtest:A.B"].state == "passing"
    unknown = staleness.proof_states(runs, scopes, FakeChanges({"aaa": None}), ("bbb", ""))
    assert unknown["gtest:C.D"].state == "stale"


def test_flakes_count_commits_with_both_outcomes() -> None:
    runs = [
        run("1", "aaa", {"gtest:A.B": "passed"}),
        run("2", "aaa", {"gtest:A.B": "failed"}),
        run("3", "bbb", {"gtest:A.B": "passed"}),
    ]
    state = staleness.proof_states(runs, {}, FakeChanges({}), ("bbb", ""))["gtest:A.B"]
    assert state.flakes == 1
    assert state.last_pass_commit == "bbb"
    assert state.state == "passing"


def git(root: Path, *args: str) -> str:
    return subprocess.run(
        ["git", *args], cwd=root, check=True, capture_output=True, text=True
    ).stdout.strip()


def test_changed_files_follow_git(tmp_path: Path) -> None:
    git(tmp_path, "init", "-q")
    git(tmp_path, "config", "user.email", "t@t")
    git(tmp_path, "config", "user.name", "t")
    write(tmp_path, "src/a.cpp", "1")
    git(tmp_path, "add", ".")
    git(tmp_path, "commit", "-qm", "one")
    first = git(tmp_path, "rev-parse", "HEAD")
    write(tmp_path, "src/a.cpp", "2")
    write(tmp_path, "src/new.cpp", "3")
    changes = staleness.ChangedFiles(tmp_path)
    assert changes.since(first) == ["src/a.cpp", "src/new.cpp"]
    assert changes.since("0" * 40) is None


def test_feature_state_ranks_failures_first() -> None:
    entry = lambda s: {"state": s}  # noqa: E731
    assert feature_map.feature_state([]) == "gap"
    assert feature_map.feature_state([entry("passing"), entry("failed")]) == "failed"
    assert feature_map.feature_state([entry("passing"), entry("unrun")]) == "unrun"
    assert feature_map.feature_state([entry("passing")]) == "passing"


def test_stage_comes_from_code_and_channels() -> None:
    nav = registry.Feature(key="n", name="n", code=["src/navigation/x.cpp", "src/navigation/"])
    assert feature_map.stage_of(nav, {}) == "nav"
    switch = registry.Feature(key="s", name="s", reach=["channel:SB"])
    assert feature_map.stage_of(switch, {}) == "robot"
    cli = registry.Feature(key="c", name="c", code=["auto_battlebot/rl/"])
    assert feature_map.stage_of(cli, {}) == "platform"
    assert feature_map.stages_of(cli, {}) == []
    autonomy = registry.Feature(
        key="a", name="a", code=["src/transmitter/", "src/target_selector/"], reach=["channel:SA"]
    )
    assert feature_map.stages_of(autonomy, {}) == ["target", "transmit", "robot"]


def test_module_of_keeps_directory_entries_whole() -> None:
    assert feature_map.module_of("src/transmitter/") == "src/transmitter/"
    assert feature_map.module_of("src/transmitter/espnow.cpp") == "src/transmitter/"
    assert feature_map.module_of("include/transmitter/espnow.hpp") == "src/transmitter/"
    assert feature_map.module_of("src/runner.cpp") == "src/runner"
    assert feature_map.module_of("include/runner.hpp") == "src/runner"
    assert feature_map.module_of("scripts/verify") == "scripts/"
    assert feature_map.is_test_path("tests/test_runner.cpp")
    assert feature_map.is_test_path("auto_battlebot/rl/tests/test_env.py")
    assert not feature_map.is_test_path("auto_battlebot/rl/env.py")


def test_shared_modules_collapse_into_infrastructure() -> None:
    def feature(key: str, touches: list[str], code: list[str], proof_modules: list[str]) -> dict:
        proof = {"modules": proof_modules}
        return {
            "key": key,
            "touches": touches,
            "code": code,
            "proofs": [proof],
            "supporting": [],
            "variants": [],
        }

    features = [
        feature("a", [], [], ["src/config/", "src/nav/"]),
        feature("b", [], [], ["src/config/"]),
        feature("c", [], ["src/config/"], ["src/logging/"]),
        feature("d", [], [], ["src/logging/", "src/config/"]),
        feature("p1", [], ["auto_battlebot/rl/"], []),
        feature("p2", [], ["auto_battlebot/eval/"], []),
        feature("p3", [], ["auto_battlebot/eval/"], []),
        feature("p4", [], ["auto_battlebot/nav/"], []),
    ]
    index = feature_map.module_index(features)
    assert index["infrastructure"] == ["src/config/"]
    assert index["modules"]["src/nav/"] == ["a"]
    assert index["modules"]["src/logging/"] == ["c", "d"]


def test_a_proof_covering_most_of_a_root_collapses_to_it() -> None:
    def feature(key: str, modules: list[str]) -> dict:
        proof = {"modules": modules}
        return {"key": key, "proofs": [proof], "supporting": [], "variants": []}

    every = ["src/a/", "src/b/", "src/c/", "src/d/"]
    features = [feature("unit", ["src/a/"]), feature("lockstep", [*every[:2], "web/x/"])]
    features.append(feature("rest", every[3:]))
    feature_map.collapse_wide_proofs(features)
    assert features[0]["proofs"][0]["modules"] == ["src/a/"]
    assert features[1]["proofs"][0]["modules"] == ["src/*", "web/x/"]


def test_build_map_joins_runs_and_marks_the_diff(tree: Path) -> None:
    write(
        tree,
        registry.FEATURES_FILE,
        '[features.tx]\nname = "Transmit"\nreach = ["command:/command/set_autonomy"]\n'
        'code = ["src/transmitter/"]\n'
        'proofs = ["gtest:EspNow.SubstitutesUnderSa", "hil:flip_sa"]\n',
    )
    write(tree, ".gitignore", "data/\n")
    git(tree, "init", "-q")
    git(tree, "config", "user.email", "t@t")
    git(tree, "config", "user.name", "t")
    git(tree, "add", ".")
    git(tree, "commit", "-qm", "base")
    commit = git(tree, "rev-parse", "--short=12", "HEAD")
    runs_dir = tree / "data/verify/runs"
    runs_dir.mkdir(parents=True)
    result = {"id": "gtest:EspNow.SubstitutesUnderSa", "status": "passed", "seconds": 0.5}
    payload = {"stamp": "s", "commit": commit, "diff_hash": "", "tier": "t", "results": [result]}
    (runs_dir / f"s_{commit}.json").write_text(json.dumps(payload))
    clean = feature_map.build_map(tree)
    tx = next(f for f in clean["features"] if f["key"] == "tx")
    states = {p["id"]: p["state"] for p in tx["proofs"]}
    assert states == {"gtest:EspNow.SubstitutesUnderSa": "passing", "hil:flip_sa": "unrun"}
    assert not tx["in_diff"]
    write(tree, "src/transmitter/espnow.cpp", "edited\n")
    edited = feature_map.build_map(tree)
    tx = next(f for f in edited["features"] if f["key"] == "tx")
    assert tx["in_diff"]
    assert tx["proofs"][0]["state"] == "stale"
    assert tx["proofs"][0]["changed"] == ["src/transmitter/espnow.cpp"]
    assert tx["proofs"][0]["modules"] == ["src/transmitter/"]
    assert tx["stages"] == ["transmit"]
    assert edited["diff"]["features"] == ["tx"]


DUMPS = {
    "tools/viz_topic_dump.cpp": REPO_ROOT / "build/viz_topic_dump",
    "tools/remote_protocol_dump.cpp": REPO_ROOT / "build/remote_protocol_dump",
    "tools/config_schema_dump.cpp": REPO_ROOT / "build/config_schema_dump",
}


def dump(source: str) -> Any:
    binary = DUMPS[source]
    if not binary.exists():
        pytest.skip(f"{binary} is not built")
    out = subprocess.run([str(binary)], capture_output=True, text=True, check=True).stdout
    return json.loads(out)


def test_header_parse_matches_the_dump_binaries() -> None:
    viz = dump("tools/viz_topic_dump.cpp")
    parsed = [
        (r.topic, r.encoding, r.latch, r.rate, r.recorded) for r in reach.viz_topic_rows(REPO_ROOT)
    ]
    dumped = [
        (t["topic"], t["encoding"], t["latch"], t["rate"], t["recorded"]) for t in viz["topics"]
    ]
    assert parsed == dumped
    remote = dump("tools/remote_protocol_dump.cpp")
    dumped_remote = {f"status:{s['topic']}" for s in remote["status"]}
    dumped_remote |= {f"command:{c['topic']}" for c in remote["commands"]}
    assert ids(reach.remote_topics(REPO_ROOT)) == dumped_remote


def test_config_schema_dump_lists_every_type_and_resolves_every_profile() -> None:
    schema = dump("tools/config_schema_dump.cpp")
    dumped = {t for section in schema["sections"].values() for t in section["types"]}
    assert dumped == {t.name for t in reach.factory_types(REPO_ROOT)}
    espnow = schema["sections"]["transmitter"]["types"]["EspNowTransmitter"]
    assert espnow["error"] is None and espnow["fields"]
    profiles = {p.id.partition(":")[2] for p in reach.profiles(REPO_ROOT)}
    assert profiles <= set(schema["profiles"])
    for name in profiles:
        assert schema["profiles"][name]["error"] is None
        assert "transmitter" in schema["profiles"][name]["types"]
    variant = feature_map.variant_config(schema, "EspNowTransmitter")
    assert variant["section"] == "transmitter"
    assert "mr_stabs_mk2_aer_zed_box" in variant["profiles"]


def test_package_main_runs_the_cli(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("sys.argv", ["features", "check", "--quiet"])
    with pytest.raises(SystemExit) as exit_info:
        runpy.run_path(str(REPO_ROOT / "auto_battlebot/features/__main__.py"), run_name="__main__")
    assert exit_info.value.code == 0


@pytest.mark.skipif(shutil.which("git") is None, reason="needs git")
def test_cli_check_prints_a_summary(capsys: pytest.CaptureFixture[str]) -> None:
    from auto_battlebot.features import cli

    assert cli.main(["check", "--quiet"]) == 0
    assert capsys.readouterr().out.startswith("features: 0 errors")


def test_scripts_wrapper_runs_registry_commands_and_the_server_help() -> None:
    script = str(REPO_ROOT / "scripts/feature_map")
    checked = subprocess.run(
        [script, "check", "--quiet"], capture_output=True, text=True, check=False
    )
    assert checked.returncode == 0, checked.stderr
    assert checked.stdout.startswith("features: 0 errors")
    served = subprocess.run([script, "--help"], capture_output=True, text=True, check=True)
    assert "--port" in served.stdout
    assert "--host" in served.stdout


def test_default_host_is_the_tailscale_ip(monkeypatch: pytest.MonkeyPatch) -> None:
    def fake_run(cmd: list[str], **_: Any) -> subprocess.CompletedProcess[str]:
        assert cmd == ["tailscale", "ip", "-4"]
        return subprocess.CompletedProcess(cmd, 0, "100.81.122.93\n", "")

    monkeypatch.setattr(serve.subprocess, "run", fake_run)
    assert serve.default_host() == "100.81.122.93"


@pytest.mark.parametrize("failure", ["missing", "down"])
def test_default_host_falls_back_to_loopback(monkeypatch: pytest.MonkeyPatch, failure: str) -> None:
    def fake_run(cmd: list[str], **_: Any) -> subprocess.CompletedProcess[str]:
        if failure == "missing":
            raise FileNotFoundError(cmd[0])
        return subprocess.CompletedProcess(cmd, 1, "", "Tailscale is stopped.")

    monkeypatch.setattr(serve.subprocess, "run", fake_run)
    assert serve.default_host() == serve.LOOPBACK
