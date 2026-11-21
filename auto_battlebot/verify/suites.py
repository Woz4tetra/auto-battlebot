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

from auto_battlebot.verify.selection import Selection, gtest_filter

PYTEST_ARGS = ("-q", "--no-header", "-p", "no:cacheprovider")
GTEST_BUILD_DIR = "build-test"
GTEST_TARGET = "auto_battlebot_test"
FIRMWARE_DIR = "firmware/mr_stabs_mk2"
DEFAULT_JOBS = 6
MEMORY_CAP = "16G"
FAILURE_TAIL_LINES = 40


@dataclass
class Step:
    """One command in a suite."""

    name: str
    command: list[str]
    cwd: Path
    env: dict[str, str] = field(default_factory=dict)


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


def gtest_group(root: Path, selection: Selection) -> Group:
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
    steps.append(Step("gtest run", command, build))
    return Group("gtest", steps)


def pytest_group(root: Path, selection: Selection) -> Group:
    scope = selection.scopes["pytest"]
    targets = [] if scope.everything else sorted(scope.items)
    command = [python_bin(root), "-m", "pytest", *PYTEST_ARGS, *targets]
    return Group("pytest", [Step("pytest", command, root)])


def synthetic_group(root: Path) -> Group:
    command = [python_bin(root), "-m", "pytest", *PYTEST_ARGS, "training/synthetic"]
    return Group("synthetic", [Step("pytest synthetic", command, root)])


def firmware_group(root: Path) -> Group:
    command = [pio_bin(root), "test", "-e", "native"]
    return Group("firmware", [Step("firmware native", command, root / FIRMWARE_DIR)])


def web_group(root: Path) -> Group:
    web = root / "web"
    return Group(
        "web",
        [
            Step("web check", ["npm", "run", "check"], web),
            Step("web test", ["npm", "test"], web),
        ],
    )


def lint_step(root: Path) -> Step:
    return Step("lint --quick", [str(root / "scripts" / "lint"), "--quick"], root)


def plan(root: Path, selection: Selection) -> list[Group]:
    """The suite groups a selection runs, in a fixed order."""
    builders: dict[str, Callable[[], Group]] = {
        "gtest": lambda: gtest_group(root, selection),
        "pytest": lambda: pytest_group(root, selection),
        "synthetic": lambda: synthetic_group(root),
        "firmware": lambda: firmware_group(root),
        "web": lambda: web_group(root),
    }
    return [builders[name]() for name, scope in selection.scopes.items() if scope.run]


def log_name(step: Step) -> str:
    return step.name.replace(" ", "_").replace("-", "") + ".log"


def run_step(step: Step, log_dir: Path) -> StepResult:
    log = log_dir / log_name(step)
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
    return StepResult(step.name, code == 0, time.monotonic() - start, log, step.command)


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
