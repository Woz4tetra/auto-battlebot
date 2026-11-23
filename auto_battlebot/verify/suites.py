"""Turn a selection into shell steps and run them, one log per step."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import time
from collections.abc import Callable, Sequence
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path

from auto_battlebot.verify.selection import Selection, gtest_filter, node_tools

PYTEST_ARGS = ("-q", "--no-header", "-p", "no:cacheprovider")
PYTEST_WORKERS = 8
PYTEST_THREADS_PER_WORKER = "4"
PYTEST_THREAD_ENV = {
    name: PYTEST_THREADS_PER_WORKER
    for name in (
        "OMP_NUM_THREADS",
        "MKL_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "NUMEXPR_NUM_THREADS",
    )
}
PYTEST_FAST_ARGS = ("-m", "not slow")
GTEST_BUILD_DIR = "build-test"
GTEST_TARGET = "auto_battlebot_test"
FIRMWARE_DIR = "firmware/mr_stabs_mk2"
DEFAULT_JOBS = 6
MEMORY_CAP = "16G"
FAILURE_TAIL_LINES = 40


@dataclass(frozen=True)
class Report:
    """A machine-readable result file a step writes, and which parser reads it."""

    kind: str
    path: Path


@dataclass
class Step:
    """One command in a suite."""

    name: str
    command: list[str]
    cwd: Path
    env: dict[str, str] = field(default_factory=dict)
    report: Report | None = None


@dataclass
class Group:
    """Steps that run in order; groups run side by side."""

    name: str
    steps: list[Step]


@dataclass
class StepResult:
    name: str
    ok: bool
    seconds: float
    log: Path
    command: list[str]
    report: Report | None = None


def build_jobs() -> int:
    return int(os.environ.get("VERIFY_JOBS", min(DEFAULT_JOBS, os.cpu_count() or 1)))


def memory_cap_prefix() -> list[str]:
    """A systemd-run scope that caps build memory, or nothing when user systemd is absent."""
    if shutil.which("systemd-run") is None:
        return []
    probe = subprocess.run(
        ["systemd-run", "--user", "--scope", "-q", "true"], capture_output=True, check=False
    )
    if probe.returncode != 0:
        return []
    return [
        "systemd-run",
        "--user",
        "--scope",
        "-q",
        "-p",
        f"MemoryMax={MEMORY_CAP}",
        "-p",
        "MemorySwapMax=0",
        "nice",
        "-n",
        "10",
    ]


def python_bin(root: Path) -> str:
    venv = root / "venv" / "bin" / "python"
    return str(venv) if venv.exists() else sys.executable


def pio_bin(root: Path) -> str:
    for candidate in (
        shutil.which("pio"),
        str(Path.home() / ".platformio" / "penv" / "bin" / "pio"),
        str(root / "venv" / "bin" / "pio"),
    ):
        if candidate and Path(candidate).exists():
            return candidate
    return "pio"


def gtest_group(root: Path, selection: Selection, reports: Path | None = None) -> Group:
    build = root / GTEST_BUILD_DIR
    steps = []
    if not (build / "Makefile").exists():
        steps.append(
            Step(
                "gtest configure",
                [
                    "cmake",
                    "-S",
                    str(root),
                    "-B",
                    str(build),
                    "-DCMAKE_BUILD_TYPE=Debug",
                    "-DBUILD_TESTING=ON",
                    "-DBUILD_DEBUG=ON",
                ],
                root,
            )
        )
    steps.append(
        Step(
            "gtest build",
            [*memory_cap_prefix(), "make", "-C", str(build), f"-j{build_jobs()}", GTEST_TARGET],
            root,
        )
    )
    command = [str(build / GTEST_TARGET), "--gtest_brief=1"]
    scope = selection.scopes["gtest"]
    if not scope.everything:
        command.append(f"--gtest_filter={gtest_filter(scope.items)}")
    report = Report("gtest", reports / "gtest.json") if reports else None
    if report:
        command.append(f"--gtest_output=json:{report.path}")
    steps.append(Step("gtest run", command, build, report=report))
    return Group("gtest", steps)


def pytest_workers(targets: list[str]) -> list[str]:
    """xdist arguments: whole files per worker, and no workers for a single file."""
    if len(targets) == 1:
        return []
    return ["-n", str(PYTEST_WORKERS), "--dist", "loadfile"]


def pytest_group(
    root: Path, selection: Selection, fast: bool = False, reports: Path | None = None
) -> Group:
    scope = selection.scopes["pytest"]
    targets = [] if scope.everything else sorted(scope.items)
    command = [
        python_bin(root),
        "-m",
        "pytest",
        *PYTEST_ARGS,
        *pytest_workers(targets),
        *(PYTEST_FAST_ARGS if fast else ()),
        *targets,
    ]
    report = Report("junit", reports / "pytest.xml") if reports else None
    if report:
        command.append(f"--junitxml={report.path}")
    env = PYTEST_THREAD_ENV if pytest_workers(targets) else {}
    return Group("pytest", [Step("pytest", command, root, env, report)])


def synthetic_group(root: Path, reports: Path | None = None) -> Group:
    command = [python_bin(root), "-m", "pytest", *PYTEST_ARGS, "training/synthetic"]
    report = Report("junit", reports / "synthetic.xml") if reports else None
    if report:
        command.append(f"--junitxml={report.path}")
    return Group("synthetic", [Step("pytest synthetic", command, root, report=report)])


def firmware_group(
    root: Path, selection: Selection | None = None, reports: Path | None = None
) -> Group:
    command = [pio_bin(root), "test", "-e", "native"]
    scope = selection.scopes["firmware"] if selection else None
    if scope is not None and not scope.everything:
        for suite in sorted(scope.items):
            command += ["-f", suite]
    report = Report("pio", reports / "firmware.json") if reports else None
    if report:
        command += ["--json-output-path", str(report.path)]
    return Group("firmware", [Step("firmware native", command, root / FIRMWARE_DIR, report=report)])


def web_group(root: Path, reports: Path | None = None) -> Group:
    web = root / "web"
    command = ["npm", "test"]
    report = Report("vitest", reports / "vitest.json") if reports else None
    if report:
        command += [
            "--",
            "--reporter=default",
            "--reporter=json",
            f"--outputFile.json={report.path}",
        ]
    return Group(
        "web",
        [
            Step("web check", ["npm", "run", "check"], web),
            Step("web test", command, web, report=report),
        ],
    )


def tools_group(root: Path, selection: Selection, reports: Path | None = None) -> Group:
    """svelte-check and vitest for each TypeScript tool under tools/."""
    scope = selection.scopes["tools"]
    names = node_tools(root) if scope.everything else sorted(scope.items)
    steps = []
    for name in names:
        tool = root / "tools" / name
        command = ["npm", "test"]
        report = Report("vitest", reports / f"vitest_{name}.json") if reports else None
        if report:
            command += [
                "--",
                "--reporter=default",
                "--reporter=json",
                f"--outputFile.json={report.path}",
            ]
        steps.append(Step(f"{name} check", ["npm", "run", "check"], tool))
        steps.append(Step(f"{name} test", command, tool, report=report))
    return Group("tools", steps)


def playback_group(
    root: Path, tier: str, reports: Path | None = None, clips: Sequence[str] = ()
) -> Group:
    """Build the app, then replay the clips for one playback tier."""
    build = Step(
        "app build",
        [
            *memory_cap_prefix(),
            "cmake",
            "--build",
            str(root / "build"),
            "--target",
            "auto_battlebot",
            f"-j{build_jobs()}",
        ],
        root,
    )
    command = [python_bin(root), "-m", "auto_battlebot.verify.playback", "check", "--tier", tier]
    for clip in clips:
        command += ["--clip", clip]
    report = Report("proofs", reports / f"playback_{tier}.json") if reports else None
    if report:
        command += ["--results", str(report.path)]
    return Group(
        f"playback {tier}", [build, Step(f"playback {tier}", command, root, report=report)]
    )


def lint_step(root: Path) -> Step:
    return Step("lint --quick", [str(root / "scripts" / "lint"), "--quick"], root)


def plan(
    root: Path, selection: Selection, fast: bool = False, reports: Path | None = None
) -> list[Group]:
    """The suite groups a selection runs, in a fixed order."""
    builders: dict[str, Callable[[], Group]] = {
        "gtest": lambda: gtest_group(root, selection, reports),
        "pytest": lambda: pytest_group(root, selection, fast, reports),
        "synthetic": lambda: synthetic_group(root, reports),
        "firmware": lambda: firmware_group(root, selection, reports),
        "web": lambda: web_group(root, reports),
        "tools": lambda: tools_group(root, selection, reports),
    }
    return [builders[name]() for name, scope in selection.scopes.items() if scope.run]


def log_name(step: Step) -> str:
    return step.name.replace(" ", "_").replace("-", "") + ".log"


def run_step(step: Step, log_dir: Path) -> StepResult:
    log = log_dir / log_name(step)
    if step.report:
        step.report.path.unlink(missing_ok=True)
    start = time.monotonic()
    env = {**os.environ, **step.env}
    with log.open("w") as out:
        out.write("$ " + " ".join(step.command) + f"\n# cwd {step.cwd}\n")
        out.flush()
        try:
            code = subprocess.run(
                step.command, cwd=step.cwd, env=env, stdout=out, stderr=subprocess.STDOUT
            ).returncode
        except FileNotFoundError as error:
            out.write(f"{error}\n")
            code = 127
    return StepResult(
        step.name, code == 0, time.monotonic() - start, log, step.command, step.report
    )


def run_group(group: Group, log_dir: Path) -> list[StepResult]:
    results = []
    for step in group.steps:
        result = run_step(step, log_dir)
        results.append(result)
        if not result.ok:
            break
    return results


def run(
    groups: Sequence[Group],
    log_dir: Path,
    before: Sequence[Step] = (),
    report: Callable[[StepResult], None] = lambda _: None,
) -> list[StepResult]:
    """Run the `before` steps in order, then every group in parallel."""
    log_dir.mkdir(parents=True, exist_ok=True)
    results = []
    for step in before:
        result = run_step(step, log_dir)
        report(result)
        results.append(result)
    with ThreadPoolExecutor(max_workers=max(1, len(groups))) as pool:
        for group_results in pool.map(lambda g: run_group(g, log_dir), groups):
            for result in group_results:
                report(result)
            results.extend(group_results)
    return results


def tail(path: Path, lines: int = FAILURE_TAIL_LINES) -> str:
    text = path.read_text(errors="replace").splitlines()
    return "\n".join(text[-lines:])
