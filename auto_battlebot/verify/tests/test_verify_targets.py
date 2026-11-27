from __future__ import annotations

from pathlib import Path

import pytest

from auto_battlebot.verify import cli, suites
from auto_battlebot.verify.selection import Scope, Selection, everything


def names(groups: list[suites.Group]) -> list[str]:
    return [group.name for group in groups]


def step_names(group: suites.Group) -> list[str]:
    return [step.name for step in group.steps]


@pytest.mark.parametrize(
    ("fast", "target", "nightly", "gpu_ok", "expected"),
    [
        (True, None, False, True, ["-m", "not slow"]),
        (False, None, False, True, []),
        (True, "jetson", False, True, ["-m", "jetson and not slow and not gpu"]),
        (True, "x86", False, True, ["-m", "not slow and not gpu"]),
        (False, "x86", True, True, []),
        (False, "x86", True, False, ["-m", "not gpu"]),
        (False, "jetson", True, True, ["-m", "jetson"]),
    ],
)
def test_pytest_markers_split_by_target_and_tier(
    fast: bool, target: str | None, nightly: bool, gpu_ok: bool, expected: list[str]
) -> None:
    assert suites.pytest_markers(fast, target, nightly, gpu_ok) == expected


def test_jetson_plan_runs_gtest_then_checks_protocol_against_its_build(tmp_path: Path) -> None:
    groups = suites.plan(tmp_path, everything(), fast=True, target="jetson")
    gtest = groups[0]
    assert step_names(gtest)[-3:] == ["gtest run", "protocol dump build", "lint protocol.ts"]
    lint = gtest.steps[-1].command
    assert "--only=protocol" in lint and "--dry-run" in lint
    assert f"--protocol-dump={tmp_path / 'build-test' / 'remote_protocol_dump'}" in lint


def test_local_plan_leaves_protocol_to_lint(tmp_path: Path) -> None:
    gtest = suites.plan(tmp_path, everything())[0]
    assert "lint protocol.ts" not in step_names(gtest)


def test_jetson_nightly_adds_release_and_asan_builds(tmp_path: Path) -> None:
    push = names(suites.plan(tmp_path, everything(), target="jetson"))
    nightly = names(suites.plan(tmp_path, everything(), target="jetson", nightly=True))
    assert nightly == [*push, "release", "gtest asan"]
    asan = suites.asan_group(tmp_path, everything())
    assert "-DAUTO_BATTLEBOT_SANITIZER=address" in asan.steps[0].command
    assert str(tmp_path / "build-asan") in asan.steps[0].command


def test_lint_applies_locally_checks_on_x86_and_skips_on_jetson(tmp_path: Path) -> None:
    local = suites.lint_step(tmp_path)
    x86 = suites.lint_step(tmp_path, "x86")
    x86_nightly = suites.lint_step(tmp_path, "x86", nightly=True)
    assert local is not None and "--dry-run" not in local.command
    assert x86 is not None and x86_nightly is not None
    assert {"--dry-run", "--strict", "--skip=protocol", "--quick"} <= set(x86.command)
    assert "--quick" not in x86_nightly.command and "--skip=clang-tidy" in x86_nightly.command
    assert suites.lint_step(tmp_path, "jetson") is None


def test_playback_configures_a_missing_release_build(tmp_path: Path) -> None:
    fresh = suites.playback_group(tmp_path, "lockstep")
    assert step_names(fresh)[0] == "release configure"
    (tmp_path / "build").mkdir()
    (tmp_path / "build" / "Makefile").write_text("")
    assert step_names(suites.playback_group(tmp_path, "lockstep"))[0] == "app build"


def test_sequential_run_finishes_each_group_before_the_next(tmp_path: Path) -> None:
    out = tmp_path / "order"
    slow = suites.Group(
        "slow", [suites.Step("a", ["sh", "-c", f"sleep 0.3; echo a >> {out}"], tmp_path)]
    )
    quick = suites.Group("quick", [suites.Step("b", ["sh", "-c", f"echo b >> {out}"], tmp_path)])
    suites.run([slow, quick], tmp_path / "logs", parallel=False)
    assert out.read_text().split() == ["a", "b"]


def test_target_restricts_the_listed_suites(capsys: pytest.CaptureFixture[str]) -> None:
    assert cli.main(["--list", "--target", "jetson", "--nightly"]) == 0
    jetson = capsys.readouterr().out.splitlines()
    assert jetson == ["gtest: all", "pytest: all", "playback realtime: every clip"]
    assert cli.main(["--list", "--target", "x86"]) == 0
    x86 = capsys.readouterr().out.splitlines()
    assert x86 == ["pytest: all", "synthetic: all", "firmware: all", "web: all"]


def test_nightly_needs_a_target() -> None:
    assert cli.main(["--nightly"]) == 2


def write_test(root: Path, rel: str, body: str) -> None:
    path = root / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(body)


def test_jetson_pytest_names_only_marked_files_so_x86_imports_never_load(tmp_path: Path) -> None:
    write_test(
        tmp_path,
        "auto_battlebot/tests/test_trt.py",
        "import pytest\npytestmark = pytest.mark.jetson\n",
    )
    write_test(tmp_path, "auto_battlebot/tests/test_sim.py", "import mujoco\n")
    command = (
        suites.pytest_group(tmp_path, everything(), fast=True, target="jetson").steps[0].command
    )
    assert command[-1] == "auto_battlebot/tests/test_trt.py"
    assert "auto_battlebot/tests/test_sim.py" not in command


def test_jetson_skips_pytest_when_no_changed_test_is_marked(tmp_path: Path) -> None:
    write_test(
        tmp_path,
        "auto_battlebot/tests/test_trt.py",
        "import pytest\npytestmark = pytest.mark.jetson\n",
    )
    scopes = {name: Scope(run=False) for name in ("gtest", "synthetic", "firmware", "web")}
    scopes["pytest"] = Scope(run=True, items={"auto_battlebot/tests/test_sim.py"})
    assert names(suites.plan(tmp_path, Selection(scopes), target="jetson")) == []
    scopes["pytest"] = Scope(run=True, items={"auto_battlebot/tests/test_trt.py"})
    assert names(suites.plan(tmp_path, Selection(scopes), target="jetson")) == ["pytest"]


def test_gtest_runs_with_a_private_tmpdir_inside_its_build(tmp_path: Path) -> None:
    run = next(
        step
        for step in suites.gtest_group(tmp_path, everything()).steps
        if step.name == "gtest run"
    )
    assert run.env["TMPDIR"] == str(tmp_path / "build-test" / "tmp")
    assert (tmp_path / "build-test" / "tmp").is_dir()


def test_tail_names_every_failed_gtest_case_even_far_above_the_tail(tmp_path: Path) -> None:
    log = tmp_path / "gtest_run.log"
    lines = [
        "[ RUN      ] A.one",
        "[  FAILED  ] A.one (3 ms)",
        *[f"noise {i}" for i in range(200)],
        "[  FAILED  ] B.two (1 ms)",
        "[  FAILED  ] 2 tests, listed below:",
        "[  FAILED  ] A.one",
        "[  FAILED  ] B.two",
    ]
    log.write_text("\n".join(lines))
    out = suites.tail(log, lines=3).splitlines()
    assert out[:2] == ["failed: A.one", "failed: B.two"]
    assert out[2:] == lines[-3:]


def test_tail_of_a_passing_style_log_has_no_failed_lines(tmp_path: Path) -> None:
    log = tmp_path / "x.log"
    log.write_text("a\nb\nc")
    assert suites.tail(log, lines=2) == "b\nc"
