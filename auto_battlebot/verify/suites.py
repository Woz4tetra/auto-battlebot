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

from auto_battlebot.gpu_queue import gpu_busy_processes
from auto_battlebot.verify.selection import PythonIndex, Selection, gtest_filter

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
PROTOCOL_DUMP_TARGET = "remote_protocol_dump"
RELEASE_BUILD_DIR = "build"
ASAN_BUILD_DIR = "build-asan"
TARGETS = ("jetson", "x86")
JETSON_MARK = "pytest.mark.jetson"
TARGET_SUITES = {
    "jetson": ("gtest", "pytest"),
    "x86": ("pytest", "synthetic", "firmware", "web"),
}
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


def make_step(name: str, build: Path, target: str, root: Path) -> Step:
    command = [*memory_cap_prefix(), "make", "-C", str(build), f"-j{build_jobs()}", target]
    return Step(name, command, root)


def gtest_group(
    root: Path,
    selection: Selection,
    protocol_check: bool = False,
    build_dir: str = GTEST_BUILD_DIR,
    label: str = "gtest",
    extra_cmake: tuple[str, ...] = (),
) -> Group:
    """Configure if needed, build and run gtest; optionally check protocol.ts against the build."""
    build = root / build_dir
    steps = []
    if not (build / "Makefile").exists():
        steps.append(
            Step(
                f"{label} configure",
                [
                    "cmake",
                    "-S",
                    str(root),
                    "-B",
                    str(build),
                    "-DCMAKE_BUILD_TYPE=Debug",
                    "-DBUILD_TESTING=ON",
                    "-DBUILD_DEBUG=ON",
                    *extra_cmake,
                ],
                root,
            )
        )
    steps.append(make_step(f"{label} build", build, GTEST_TARGET, root))
    command = [str(build / GTEST_TARGET), "--gtest_brief=1"]
    scope = selection.scopes["gtest"]
    if not scope.everything:
        command.append(f"--gtest_filter={gtest_filter(scope.items)}")
    scratch = build / "tmp"
    scratch.mkdir(parents=True, exist_ok=True)
    steps.append(Step(f"{label} run", command, build, {"TMPDIR": str(scratch)}))
    if protocol_check:
        steps.append(make_step("protocol dump build", build, PROTOCOL_DUMP_TARGET, root))
        steps.append(
            Step(
                "lint protocol.ts",
                [
                    str(root / "scripts" / "lint"),
                    "--dry-run",
                    "--strict",
                    "--only=protocol",
                    f"--protocol-dump={build / PROTOCOL_DUMP_TARGET}",
                ],
                root,
            )
        )
    return Group(label, steps)


def asan_group(root: Path, selection: Selection) -> Group:
    return gtest_group(
        root,
        selection,
        build_dir=ASAN_BUILD_DIR,
        label="gtest asan",
        extra_cmake=("-DAUTO_BATTLEBOT_SANITIZER=address",),
    )


def pytest_workers(targets: list[str]) -> list[str]:
    """xdist arguments: whole files per worker, and no workers for a single file."""
    if len(targets) == 1:
        return []
    return ["-n", str(PYTEST_WORKERS), "--dist", "loadfile"]


def pytest_markers(
    fast: bool, target: str | None = None, nightly: bool = False, gpu_ok: bool = True
) -> list[str]:
    """The `-m` expression for a run: the Jetson takes `jetson` tests, CI skips `gpu` on push."""
    terms = []
    if target == "jetson":
        terms.append("jetson")
    if fast and not nightly:
        terms.append("not slow")
    if target is not None and not (nightly and gpu_ok):
        terms.append("not gpu")
    return ["-m", " and ".join(terms)] if terms else []


def jetson_test_files(root: Path) -> list[str]:
    """Test modules that carry the jetson marker, found by text so nothing x86-only is imported."""
    return [
        test
        for test in PythonIndex(root).test_files()
        if JETSON_MARK in (root / test).read_text(errors="replace")
    ]


def pytest_targets(root: Path, selection: Selection, target: str | None) -> list[str] | None:
    """Explicit test files to run, or None for the whole testpaths tree."""
    scope = selection.scopes["pytest"]
    chosen = None if scope.everything else set(scope.items)
    if target == "jetson":
        marked = set(jetson_test_files(root))
        chosen = marked if chosen is None else chosen & marked
    return None if chosen is None else sorted(chosen)


def pytest_group(
    root: Path,
    selection: Selection,
    fast: bool = False,
    target: str | None = None,
    nightly: bool = False,
    gpu_ok: bool = True,
) -> Group:
    targets = pytest_targets(root, selection, target) or []
    command = [
        python_bin(root),
        "-m",
        "pytest",
        *PYTEST_ARGS,
        *pytest_workers(targets),
        *pytest_markers(fast, target, nightly, gpu_ok),
        *targets,
    ]
    env = PYTEST_THREAD_ENV if pytest_workers(targets) else {}
    return Group("pytest", [Step("pytest", command, root, env)])


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


def release_configure_step(root: Path) -> Step | None:
    build = root / RELEASE_BUILD_DIR
    if (build / "Makefile").exists():
        return None
    command = ["cmake", "-S", str(root), "-B", str(build), "-DCMAKE_BUILD_TYPE=Release"]
    return Step("release configure", command, root)


def release_group(root: Path) -> Group:
    """Configure build/ if needed and build every release target."""
    configure = release_configure_step(root)
    build = make_step("release build", root / RELEASE_BUILD_DIR, "all", root)
    return Group("release", [*([configure] if configure else []), build])


def playback_group(root: Path, tier: str) -> Group:
    """Build the app, then replay the clips for one playback tier."""
    configure = release_configure_step(root)
    build = Step(
        "app build",
        [
            *memory_cap_prefix(),
            "cmake",
            "--build",
            str(root / RELEASE_BUILD_DIR),
            "--target",
            "auto_battlebot",
            f"-j{build_jobs()}",
        ],
        root,
    )
    command = [python_bin(root), "-m", "auto_battlebot.verify.playback", "check", "--tier", tier]
    steps = [*([configure] if configure else []), build, Step(f"playback {tier}", command, root)]
    return Group(f"playback {tier}", steps)


def lint_step(root: Path, target: str | None = None, nightly: bool = False) -> Step | None:
    """Lint before the suites: apply locally, check-only in CI, none on the Jetson."""
    lint = str(root / "scripts" / "lint")
    if target is None:
        return Step("lint --quick", [lint, "--quick"], root)
    if target == "jetson":
        return None
    depth = ["--skip=clang-tidy"] if nightly else ["--quick"]
    return Step("lint", [lint, "--dry-run", "--strict", "--skip=protocol", *depth], root)


def gpu_idle() -> bool:
    """No other process holds real GPU memory, per the GPU queue's own check."""
    return not gpu_busy_processes(ignore_pids=set())


def plan(
    root: Path,
    selection: Selection,
    fast: bool = False,
    target: str | None = None,
    nightly: bool = False,
    gpu_ok: bool = True,
) -> list[Group]:
    """The suite groups a selection runs, in a fixed order."""
    jetson = target == "jetson"
    builders: dict[str, Callable[[], Group]] = {
        "gtest": lambda: gtest_group(root, selection, protocol_check=jetson),
        "pytest": lambda: pytest_group(root, selection, fast, target, nightly, gpu_ok),
        "synthetic": lambda: synthetic_group(root),
        "firmware": lambda: firmware_group(root),
        "web": lambda: web_group(root),
    }
    if target == "jetson" and pytest_targets(root, selection, target) == []:
        selection.scopes["pytest"].run = False
    groups = [builders[name]() for name, scope in selection.scopes.items() if scope.run]
    if jetson and nightly:
        groups += [release_group(root), asan_group(root, selection)]
    return groups


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
    parallel: bool = True,
) -> list[StepResult]:
    """Run the `before` steps in order, then every group, side by side unless `parallel` is off."""
    log_dir.mkdir(parents=True, exist_ok=True)
    results = []
    for step in before:
        result = run_step(step, log_dir)
        report(result)
        results.append(result)
    workers = max(1, len(groups)) if parallel else 1
    with ThreadPoolExecutor(max_workers=workers) as pool:
        for group_results in pool.map(lambda g: run_group(g, log_dir), groups):
            for result in group_results:
                report(result)
            results.extend(group_results)
    return results


GTEST_FAILED = "[  FAILED  ] "


def failed_gtest_names(text: list[str]) -> list[str]:
    """Each failing gtest case once, in the order gtest reported it."""
    names: dict[str, None] = {}
    for line in text:
        if line.startswith(GTEST_FAILED) and not line[len(GTEST_FAILED) :][:1].isdigit():
            names[line[len(GTEST_FAILED) :].split(" ", 1)[0]] = None
    return list(names)


def tail(path: Path, lines: int = FAILURE_TAIL_LINES) -> str:
    """A failing step's last lines, led by the gtest cases that failed, which the tail can miss."""
    text = path.read_text(errors="replace").splitlines()
    failed = [f"failed: {name}" for name in failed_gtest_names(text)]
    return "\n".join([*failed, *text[-lines:]])
