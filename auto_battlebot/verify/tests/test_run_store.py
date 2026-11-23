"""Tests for the run store, report parsers, gcda reader, --only planning, coverage, and serve."""

from __future__ import annotations

import json
import struct
import threading
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest

from auto_battlebot.features import serve
from auto_battlebot.verify import coverage, gcda, only, run_store, suites
from auto_battlebot.verify.run_store import ProofResult
from auto_battlebot.verify.selection import Selection, gtest_filter, select


def test_gtest_json_strips_parameters_and_reads_skips(tmp_path: Path) -> None:
    report = {
        "testsuites": [
            {
                "name": "EspNow",
                "testsuite": [
                    {"name": "Sends", "classname": "EspNow", "time": "0.002s"},
                    {"name": "Drops", "classname": "EspNow", "failures": [{"failure": "x"}]},
                    {"name": "Skips", "classname": "EspNow", "result": "SKIPPED"},
                ],
            },
            {
                "name": "Formats/H264",
                "testsuite": [
                    {"name": "Flush/0", "classname": "Formats/H264", "time": "0.1s"},
                    {"name": "Flush/1", "classname": "Formats/H264", "failures": [{}]},
                ],
            },
        ]
    }
    path = tmp_path / "gtest.json"
    path.write_text(json.dumps(report))
    results = {r.id: r.status for r in run_store.gtest_results(path)}
    assert results == {
        "gtest:EspNow.Sends": "passed",
        "gtest:EspNow.Drops": "failed",
        "gtest:EspNow.Skips": "skipped",
        "gtest:H264.Flush": "failed",
    }


def test_junit_maps_classnames_back_to_files(tmp_path: Path) -> None:
    (tmp_path / "pkg/tests").mkdir(parents=True)
    (tmp_path / "pkg/tests/test_x.py").write_text("")
    xml = """<testsuites><testsuite>
      <testcase classname="pkg.tests.test_x" name="test_a" time="0.5"/>
      <testcase classname="pkg.tests.test_x.TestGroup" name="test_b[1]" time="0.1">
        <failure/>
      </testcase>
      <testcase classname="pkg.tests.test_x" name="test_c"><skipped/></testcase>
      <testcase classname="nowhere" name="test_d"/>
    </testsuite></testsuites>"""
    path = tmp_path / "pytest.xml"
    path.write_text(xml)
    results = {r.id: r.status for r in run_store.junit_results(path, tmp_path)}
    assert results == {
        "pytest:pkg/tests/test_x.py::test_a": "passed",
        "pytest:pkg/tests/test_x.py::TestGroup::test_b": "failed",
        "pytest:pkg/tests/test_x.py::test_c": "skipped",
    }


def test_vitest_and_pio_reports(tmp_path: Path) -> None:
    vitest = {
        "testResults": [
            {
                "name": str(tmp_path / "web/src/lib/a.test.ts"),
                "assertionResults": [
                    {"title": "renders", "status": "passed", "duration": 25.0},
                    {"title": "breaks", "status": "failed", "duration": None},
                    {"title": "later", "status": "todo"},
                ],
            }
        ]
    }
    (tmp_path / "v.json").write_text(json.dumps(vitest))
    results = {r.id: r.status for r in run_store.vitest_results(tmp_path / "v.json", tmp_path)}
    assert results == {
        "vitest:web/src/lib/a.test.ts::renders": "passed",
        "vitest:web/src/lib/a.test.ts::breaks": "failed",
        "vitest:web/src/lib/a.test.ts::later": "skipped",
    }
    pio = {
        "test_suites": [
            {
                "test_name": "test_pid",
                "test_cases": [
                    {"name": "Pid.Holds", "status": "PASSED", "duration": 0},
                    {"name": "Pid.Breaks", "status": "ERRORED", "duration": 0.1},
                ],
            }
        ]
    }
    (tmp_path / "p.json").write_text(json.dumps(pio))
    results = {r.id: r.status for r in run_store.pio_results(tmp_path / "p.json")}
    assert results == {
        "native:test_pid::Pid.Holds": "passed",
        "native:test_pid::Pid.Breaks": "failed",
    }
    assert run_store.read_report("gtest", tmp_path / "missing.json", tmp_path) == []
    (tmp_path / "bad.json").write_text("{")
    assert run_store.read_report("vitest", tmp_path / "bad.json", tmp_path) == []


def test_merge_lets_a_failure_win() -> None:
    merged = run_store.merge(
        [
            ProofResult("a", "passed", 1.0),
            ProofResult("a", "failed", 2.0),
            ProofResult("b", "skipped", 0),
        ]
    )
    assert {r.id: (r.status, r.seconds) for r in merged} == {
        "a": ("failed", 3.0),
        "b": ("skipped", 0),
    }


def test_write_load_and_prune(tmp_path: Path) -> None:
    for i in range(4):
        run_store.write_run(
            tmp_path, f"t{i}", [ProofResult("gtest:A.B", "passed", 0.1)], "abc", "", keep=3
        )
    runs = run_store.load_runs(tmp_path)
    assert [r.tier for r in runs] == ["t1", "t2", "t3"]
    assert runs[-1].by_id()["gtest:A.B"].status == "passed"
    names = [p.name for p in (tmp_path / run_store.RUNS_DIR).glob("*.json")]
    assert all("_abc" in name for name in names)
    with pytest.raises(ValueError):
        run_store.write_run(tmp_path, "t", [ProofResult("x", "maybe", 0)], "abc", "")


def test_suite_steps_write_their_reports(tmp_path: Path) -> None:
    selection = Selection()
    selection.scopes["gtest"].add("EspNow.Sends")
    selection.scopes["firmware"].add("test_pid")
    gtest = suites.gtest_group(tmp_path, selection, tmp_path)
    assert f"--gtest_output=json:{tmp_path / 'gtest.json'}" in gtest.steps[-1].command
    assert gtest.steps[-1].report == suites.Report("gtest", tmp_path / "gtest.json")
    firmware = suites.firmware_group(tmp_path, selection, tmp_path)
    assert firmware.steps[0].command[-4:] == [
        "-f",
        "test_pid",
        "--json-output-path",
        str(tmp_path / "firmware.json"),
    ]
    assert suites.gtest_group(tmp_path, selection).steps[-1].report is None


def test_gtest_filter_takes_cases_or_suites() -> None:
    assert gtest_filter(["EspNow.Sends"]) == "EspNow.Sends:*/EspNow.Sends/*:EspNow/*.Sends"
    assert gtest_filter(["EspNow"]) == "EspNow.*:*/EspNow.*:EspNow/*"


def test_only_plans_each_runner() -> None:
    plan = only.plan_only(
        [
            "gtest:EspNow.Sends",
            "pytest:a/tests/test_x.py::test_a",
            "native:test_pid::Pid.Holds",
            "vitest:tools/feature_map/src/lib/graph.test.ts::groups",
            "vitest:web/src/lib/connection.test.ts::retries",
            "playback:sphinx:hashes",
            "playback:clyde:frames",
            "hil:flip_sa",
            "garbage",
        ]
    )
    scopes = plan.selection.scopes
    assert scopes["gtest"].items == {"EspNow.Sends"}
    assert scopes["pytest"].items == {"a/tests/test_x.py::test_a"}
    assert scopes["firmware"].items == {"test_pid"}
    assert scopes["tools"].items == {"feature_map"}
    assert scopes["web"].everything
    assert plan.playback == ("lockstep",)
    assert plan.clips == ["clyde", "sphinx"]
    assert len(plan.problems) == 2
    every = only.plan_only(["playback:clyde:frames", "mcap:topic_rate:/tf"])
    assert every.clips == [] and every.playback == ("lockstep",)


def test_tool_paths_select_the_tools_suite(tmp_path: Path) -> None:
    (tmp_path / "tools/feature_map").mkdir(parents=True)
    (tmp_path / "tools/feature_map/package.json").write_text("{}")
    (tmp_path / "tools/feature_map/src").mkdir()
    selection = select(["tools/feature_map/src/App.svelte", "tools/other.cpp"], tmp_path)
    assert selection.scopes["tools"].items == {"feature_map"}


def gcda_bytes(functions: list[tuple[int, list[int] | None]]) -> bytes:
    out = struct.pack("<IIII", gcda.GCDA_MAGIC, 0, 0, 0)
    out += struct.pack("<Ii", 0xA1000000, 8) + struct.pack("<II", 1, 0)
    for ident, counts in functions:
        out += struct.pack("<Ii", gcda.FUNCTION_TAG, 12) + struct.pack("<III", ident, 0, 0)
        if counts is None:
            out += struct.pack("<Ii", gcda.ARC_COUNTER_TAG, -16)
        else:
            out += struct.pack("<Ii", gcda.ARC_COUNTER_TAG, 8 * len(counts))
            out += struct.pack(f"<{len(counts)}q", *counts)
    return out


class FakeIndex(gcda.FunctionIndex):
    def __init__(self, functions: dict[int, str]) -> None:
        self.mapping = functions

    def functions(self, gcno: Path) -> dict[int, str]:
        del gcno
        return self.mapping


def test_gcda_blocks_and_moved_functions() -> None:
    blocks = gcda.function_blocks(gcda_bytes([(7, None), (9, [0, 3])]))
    assert blocks[7] == b""
    assert struct.unpack("<2q", blocks[9]) == (0, 3)
    with pytest.raises(ValueError):
        gcda.function_blocks(b"x" * 16)
    obj = Path("/b/CMakeFiles/t.dir/src/a.cpp.gcda")
    base = {obj: gcda.function_blocks(gcda_bytes([(7, None), (9, [1, 1])]))}
    run = {obj: gcda.function_blocks(gcda_bytes([(7, [2, 0]), (9, [1, 1])]))}
    index = FakeIndex({7: "include/a.hpp", 9: "src/a.cpp"})
    assert gcda.executed_files(run, base, index) == {"include/a.hpp"}
    assert gcda.executed_files(base, base, index) == set()


def test_first_party_skips_system_and_vendored_files(tmp_path: Path) -> None:
    root = tmp_path.resolve()
    assert gcda.first_party(f"{root}/src/a.cpp", root) == "src/a.cpp"
    assert gcda.first_party(f"{root}/third_party/x.h", root) is None
    assert gcda.first_party(f"{root}/build-test/gen.cpp", root) is None
    assert gcda.first_party("/usr/include/c++/13/vector", root) is None


def test_gcno_function_lines_parse() -> None:
    line = (
        "x.gcno:  01000000: 109:FUNCTION ident=1364491206, lineno_checksum=0x7c, "
        "cfg_checksum=0xa4, `_ZSt8_Destroy' /home/u/repo/src/a.cpp:146:5-153:5\n"
    )
    match = gcda.GCNO_FUNCTION.search(line)
    assert match is not None
    assert match.group(1) == "1364491206" and match.group(2) == "/home/u/repo/src/a.cpp"


def test_coverage_contexts_become_pytest_ids() -> None:
    assert (
        coverage.context_proof("pkg/tests/test_x.py::test_a|run")
        == "pytest:pkg/tests/test_x.py::test_a"
    )
    assert coverage.context_proof("pkg/tests/test_x.py::T::test_b[1]|setup") == (
        "pytest:pkg/tests/test_x.py::T::test_b"
    )
    assert coverage.context_proof("") is None


def test_bundle_staleness_follows_the_source(tmp_path: Path) -> None:
    static = tmp_path / "static"
    tool = tmp_path / serve.TOOL_DIR / "src"
    tool.mkdir(parents=True)
    (tool / "App.svelte").write_text("a")
    assert serve.bundle_is_stale(tmp_path, static)
    static.mkdir()
    (static / "index.html").write_text("built")
    assert not serve.bundle_is_stale(tmp_path, static)
    later = (static / "index.html").stat().st_mtime + 10
    import os

    os.utime(tool / "App.svelte", (later, later))
    assert serve.bundle_is_stale(tmp_path, static)


def test_server_serves_the_map_and_the_bundle(tmp_path: Path) -> None:
    static = tmp_path / "static"
    static.mkdir()
    (static / "index.html").write_text("<html>map</html>")
    state = serve.MapState(tmp_path)
    state.payload = b'{"features": []}'
    server = ThreadingHTTPServer(("127.0.0.1", 0), serve.make_handler(state, static))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        base = f"http://127.0.0.1:{server.server_address[1]}"
        with urllib.request.urlopen(f"{base}/api/map") as response:
            assert json.loads(response.read()) == {"features": []}
        with urllib.request.urlopen(f"{base}/some/route") as response:
            assert response.read() == b"<html>map</html>"
        with urllib.request.urlopen(f"{base}/../../etc/passwd") as response:
            assert response.read() == b"<html>map</html>"
    finally:
        server.shutdown()
        server.server_close()


def test_wait_for_wakes_on_a_new_version(tmp_path: Path) -> None:
    state = serve.MapState(tmp_path)
    assert state.wait_for(0, 0.01) == 0
    with state.changed:
        state.version = 1
        state.changed.notify_all()
    assert state.wait_for(0, 0.01) == 1
