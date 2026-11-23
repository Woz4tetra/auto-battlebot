"""Proof ids: `<runner>:<locator>`, and the listing each runner checks existence against."""

from __future__ import annotations

import ast
import re
from collections.abc import Callable, Iterable
from dataclasses import dataclass
from pathlib import Path

from auto_battlebot.compat import tomllib
from auto_battlebot.features import reach
from auto_battlebot.verify.playback import CLIP_METRICS

RUNNERS = ("gtest", "pytest", "vitest", "native", "playback", "sim", "hil", "mcap")
GTEST_DIR = "tests"
FIRMWARE_TEST_DIR = "firmware/mr_stabs_mk2/test"
VITEST_ROOTS = ("web/src", "tools")
PERFORMANCE_HEADER = "include/health/performance_monitor.hpp"
LINK_HEADER = "include/health/link_report.hpp"
PLAYBACK_CLIPS = "auto_battlebot/verify/playback_clips.toml"
HIL_CHECKLIST = "auto_battlebot/verify/hil_checklist.toml"
PLAYBACK_METRICS = ("hashes", *CLIP_METRICS)
TOPIC_RATE_PREFIX = "topic_rate:"

GTEST_DECL = re.compile(r"^\s*(?:TEST|TEST_F|TEST_P|TYPED_TEST)\(\s*(\w+)\s*,\s*(\w+)\s*\)", re.M)
VITEST_DECL = re.compile(r"""^\s*(?:it|test)(?:\.\w+)?\(\s*(["'`])(.+?)\1""", re.M)
ENUM_BODY = re.compile(r"enum class (\w+) \{([^}]*)\}", re.S)


@dataclass(frozen=True)
class ProofId:
    """A parsed proof id."""

    runner: str
    locator: str

    def __str__(self) -> str:
        return f"{self.runner}:{self.locator}"


def parse(proof: str) -> ProofId | None:
    runner, sep, locator = proof.partition(":")
    if not sep or runner not in RUNNERS or not locator:
        return None
    return ProofId(runner, locator)


def gtest_ids(root: Path) -> set[str]:
    ids = set()
    for path in (root / GTEST_DIR).rglob("*.cpp"):
        for m in GTEST_DECL.finditer(path.read_text(errors="replace")):
            ids.add(f"{m.group(1)}.{m.group(2)}")
    return ids


def python_test_names(path: Path) -> set[str]:
    try:
        tree = ast.parse(path.read_text(errors="replace"))
    except SyntaxError:
        return set()
    names = set()
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name.startswith(
            "test"
        ):
            names.add(node.name)
        elif isinstance(node, ast.ClassDef) and node.name.startswith("Test"):
            for item in node.body:
                if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)) and (
                    item.name.startswith("test")
                ):
                    names.add(f"{node.name}::{item.name}")
    return names


def pytest_exists(root: Path, locator: str) -> bool:
    path, sep, name = locator.partition("::")
    if not sep:
        return False
    file = root / path
    if not file.is_file() or file.suffix != ".py":
        return False
    return name.split("[", 1)[0] in python_test_names(file)


def vitest_ids(root: Path) -> set[str]:
    ids = set()
    for base in VITEST_ROOTS:
        for path in (root / base).rglob("*.test.ts"):
            if "node_modules" in path.parts:
                continue
            rel = str(path.relative_to(root))
            for m in VITEST_DECL.finditer(path.read_text(errors="replace")):
                ids.add(f"{rel}::{m.group(2)}")
    return ids


def native_ids(root: Path) -> set[str]:
    ids = set()
    for suite in sorted((root / FIRMWARE_TEST_DIR).iterdir()):
        if not suite.is_dir():
            continue
        for path in suite.rglob("*.cpp"):
            for m in GTEST_DECL.finditer(path.read_text(errors="replace")):
                ids.add(f"{suite.name}::{m.group(1)}.{m.group(2)}")
    return ids


def enum_values(root: Path, header: str, enum: str) -> list[str]:
    for m in ENUM_BODY.finditer((root / header).read_text()):
        if m.group(1) == enum:
            return [v.strip().lower() for v in m.group(2).split(",") if v.strip()]
    return []


def link_names(root: Path) -> set[str]:
    return set(enum_values(root, LINK_HEADER, "MonitorLink"))


def mcap_ids(root: Path) -> set[str]:
    ids = set(enum_values(root, PERFORMANCE_HEADER, "PerformanceCheck"))
    topics = [row.topic for row in reach.viz_topic_rows(root)]
    topics += [p.id.partition(":")[2] for p in reach.remote_topics(root) if p.kind == "status"]
    ids |= {TOPIC_RATE_PREFIX + topic for topic in topics}
    return ids


def playback_clip_names(root: Path) -> set[str]:
    path = root / PLAYBACK_CLIPS
    if not path.exists():
        return set()
    return set(tomllib.loads(path.read_text()).get("clips", {}))


def playback_ids(root: Path) -> set[str]:
    return {f"{clip}:{metric}" for clip in playback_clip_names(root) for metric in PLAYBACK_METRICS}


def hil_ids(root: Path) -> set[str]:
    path = root / HIL_CHECKLIST
    if not path.exists():
        return set()
    return {step["id"] for step in tomllib.loads(path.read_text()).get("steps", [])}


def sim_ids(root: Path) -> set[str]:
    del root
    return set()


Listing = Callable[[Path], set[str]]
LISTINGS: dict[str, Listing] = {
    "gtest": gtest_ids,
    "vitest": vitest_ids,
    "native": native_ids,
    "playback": playback_ids,
    "sim": sim_ids,
    "hil": hil_ids,
    "mcap": mcap_ids,
}


class Resolver:
    """Checks proof ids against each runner's listing, reading each listing once."""

    def __init__(self, root: Path) -> None:
        self.root = root
        self._listings: dict[str, set[str]] = {}

    def listing(self, runner: str) -> set[str]:
        if runner not in self._listings:
            self._listings[runner] = LISTINGS[runner](self.root)
        return self._listings[runner]

    def problem(self, proof: str) -> str | None:
        """Why a proof id does not resolve, or None when it does."""
        parsed = parse(proof)
        if parsed is None:
            return f"{proof!r} is not <runner>:<locator> with a runner in {list(RUNNERS)}"
        if parsed.runner == "pytest":
            ok = pytest_exists(self.root, parsed.locator)
        else:
            ok = parsed.locator in self.listing(parsed.runner)
        if ok:
            return None
        if parsed.runner == "sim":
            return f"{proof}: the sim smoke tier has no scenarios yet"
        return f"{proof}: no such {parsed.runner} proof"

    def problems(self, proofs: Iterable[str]) -> list[str]:
        return [p for p in (self.problem(proof) for proof in proofs) if p]
