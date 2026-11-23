"""Map changed files to the suites and test cases that cover them."""

from __future__ import annotations

import ast
import os
import re
from collections.abc import Iterable
from dataclasses import dataclass, field
from pathlib import Path

SUITES = ("gtest", "pytest", "synthetic", "firmware", "web")
GTEST_DECL = re.compile(
    r"^\s*(?:TEST|TEST_F|TEST_P|TYPED_TEST|TYPED_TEST_P)\(\s*(\w+)\s*,", re.MULTILINE
)
GTEST_DEPFILE_DIR = Path("build-test/CMakeFiles/auto_battlebot_test.dir")
GTEST_EVERYTHING = ("CMakeLists.txt", "cmake/", "config/", "third_party/")
CPP_SUFFIXES = {".cpp", ".hpp", ".h", ".c", ".cu", ".cuh"}
FIRMWARE_NATIVE = ("firmware/mr_stabs_mk2/", "firmware/shared/")
WEB = ("web/", "include/remote/")
SYNTHETIC = "training/synthetic/"
PYTEST_ROOTS = (
    "auto_battlebot",
    "scripts",
    "simulation",
    "training",
    "tests",
    "firmware",
    "hardware",
    "install",
    "service",
)
PYTEST_EVERYTHING = ("conftest.py", "pyproject.toml")
FIRST_PARTY = ("auto_battlebot",)
IGNORED_PARTS = {"node_modules", ".pio", "__pycache__"}
PRUNED_DIRS = IGNORED_PARTS | {"venv", "data", "runs"}
CODE_SUFFIXES = CPP_SUFFIXES | {".py", ".sh", ".ts", ".svelte"}


@dataclass
class Scope:
    """What one suite runs: nothing, everything, or a set of items."""

    run: bool = False
    everything: bool = False
    items: set[str] = field(default_factory=set)

    def add(self, item: str) -> None:
        self.run = True
        self.items.add(item)

    def add_everything(self) -> None:
        self.run = True
        self.everything = True


@dataclass
class Selection:
    """Per-suite scopes plus the code files no suite covers."""

    scopes: dict[str, Scope] = field(default_factory=lambda: {s: Scope() for s in SUITES})
    uncovered: list[str] = field(default_factory=list)

    def any(self) -> bool:
        return any(scope.run for scope in self.scopes.values())


def everything() -> Selection:
    """A selection that runs every suite in full."""
    selection = Selection()
    for scope in selection.scopes.values():
        scope.add_everything()
    return selection


def gtest_suites_in(text: str) -> set[str]:
    """Test suite names declared in a gtest source."""
    return set(GTEST_DECL.findall(text))


def gtest_filter(suites: Iterable[str]) -> str:
    """A --gtest_filter that matches plain, parameterized, and typed tests of each suite."""
    patterns = []
    for suite in sorted(suites):
        patterns += [f"{suite}.*", f"*/{suite}.*", f"{suite}/*"]
    return ":".join(patterns)


def parse_depfile(text: str, root: Path) -> set[str]:
    """Repo-relative paths a make depfile lists, dropping the target and outside files."""
    tokens = text.replace("\\\n", " ").split()
    prefix = str(root.resolve()) + "/"
    deps = set()
    for token in tokens[1:]:
        if token.startswith(prefix):
            deps.add(token[len(prefix) :])
    return deps


class GtestIndex:
    """Which test sources include which repo files, from the build-test depfiles."""

    def __init__(self, root: Path) -> None:
        self.root = root
        self.includers: dict[str, set[str]] = {}
        self.available = False
        depdir = root / GTEST_DEPFILE_DIR
        if not depdir.is_dir():
            return
        for depfile in depdir.rglob("*.o.d"):
            source = str(depfile.relative_to(depdir)).removesuffix(".o.d")
            if not source.startswith("tests/"):
                continue
            self.available = True
            for dep in parse_depfile(depfile.read_text(errors="replace"), root):
                self.includers.setdefault(dep, set()).add(source)

    def tests_including(self, paths: Iterable[str]) -> set[str]:
        found: set[str] = set()
        for path in paths:
            found |= self.includers.get(path, set())
        return found

    def suites_of(self, sources: Iterable[str]) -> set[str]:
        suites: set[str] = set()
        for source in sources:
            path = self.root / source
            if path.is_file():
                suites |= gtest_suites_in(path.read_text(errors="replace"))
        return suites


def headers_for_source(root: Path, path: str) -> list[str]:
    """Headers a src/ file implements: its same-stem header, else its module's headers."""
    relative = Path(path).relative_to("src")
    for suffix in (".hpp", ".h"):
        twin = Path("include") / relative.with_suffix(suffix)
        if (root / twin).is_file():
            return [str(twin)]
    module = root / "include" / relative.parts[0]
    if len(relative.parts) > 1 and module.is_dir():
        return [str(p.relative_to(root)) for p in module.rglob("*") if p.suffix in CPP_SUFFIXES]
    return []


class PythonIndex:
    """First-party import closure of each pytest file, plus the files it names in strings."""

    def __init__(self, root: Path) -> None:
        self.root = root
        self.closures: dict[str, set[str]] = {}
        self.strings: dict[str, set[str]] = {}

    def module_file(self, dotted: str) -> str | None:
        base = self.root / dotted.replace(".", "/")
        for candidate in (base.with_suffix(".py"), base / "__init__.py"):
            if candidate.is_file():
                return str(candidate.relative_to(self.root))
        return None

    def _package_parents(self, dotted: str) -> list[str]:
        parts = dotted.split(".")
        found = []
        for end in range(1, len(parts)):
            init = self.module_file(".".join(parts[:end]))
            if init is not None:
                found.append(init)
        return found

    def _resolve(self, dotted: str) -> list[str]:
        if not dotted.startswith(FIRST_PARTY):
            return []
        found = self._package_parents(dotted)
        own = self.module_file(dotted)
        if own is not None:
            found.append(own)
        return found

    def _tree(self, path: str) -> ast.Module | None:
        try:
            return ast.parse((self.root / path).read_text(errors="replace"))
        except (OSError, SyntaxError, ValueError):
            return None

    def direct_imports(self, path: str) -> set[str]:
        tree = self._tree(path)
        if tree is None:
            return set()
        package = ".".join(Path(path).with_suffix("").parts[:-1])
        found: set[str] = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    found.update(self._resolve(alias.name))
            elif isinstance(node, ast.ImportFrom):
                if node.level:
                    anchor = package.split(".")[: len(package.split(".")) - node.level + 1]
                    module = ".".join(anchor + ([node.module] if node.module else []))
                else:
                    module = node.module or ""
                found.update(self._resolve(module))
                for alias in node.names:
                    found.update(self._resolve(f"{module}.{alias.name}"))
        found.discard(path)
        return found

    def closure(self, path: str) -> set[str]:
        if path in self.closures:
            return self.closures[path]
        seen = {path}
        stack = [path]
        while stack:
            for dep in self.direct_imports(stack.pop()):
                if dep not in seen:
                    seen.add(dep)
                    stack.append(dep)
        self.closures[path] = seen
        return seen

    def strings_in(self, path: str) -> set[str]:
        if path not in self.strings:
            tree = self._tree(path)
            self.strings[path] = (
                set()
                if tree is None
                else {
                    node.value
                    for node in ast.walk(tree)
                    if isinstance(node, ast.Constant) and isinstance(node.value, str)
                }
            )
        return self.strings[path]

    def test_files(self) -> list[str]:
        found = []
        for top in PYTEST_ROOTS:
            for dirpath, dirnames, filenames in os.walk(self.root / top):
                dirnames[:] = [d for d in dirnames if not pruned(d)]
                rel = Path(dirpath).relative_to(self.root)
                found += [str(rel / f) for f in filenames if is_pytest_file(str(rel / f))]
        return sorted(found)

    def tests_covering(self, path: str) -> set[str]:
        name = Path(path).name
        dotted = str(Path(path).with_suffix("")).replace("/", ".")
        found = set()
        for test in self.test_files():
            if test == path:
                found.add(test)
                continue
            strings = self.strings_in(test)
            named = path in strings or name in strings or dotted in strings
            if not named:
                named = any(s.endswith("/" + name) for s in strings)
            if named or path in self._deps(test, strings):
                found.add(test)
        return found

    def _deps(self, test: str, strings: set[str]) -> set[str]:
        deps = set(self.closure(test))
        for value in strings:
            if value.endswith(".py") and (self.root / value).is_file():
                deps |= self.closure(value)
        return deps


def pruned(name: str) -> bool:
    """A directory the pytest walk never enters."""
    return name in PRUNED_DIRS or name.startswith((".", "build"))


def is_pytest_file(path: str) -> bool:
    """A test module the main pytest suite collects: <root>/**/tests/test_*.py."""
    p = Path(path)
    return (
        p.parent.name == "tests"
        and p.name.startswith("test_")
        and p.suffix == ".py"
        and p.parts[0] in PYTEST_ROOTS
        and not path.startswith(SYNTHETIC)
    )


def is_ignored(path: str) -> bool:
    return bool(IGNORED_PARTS.intersection(Path(path).parts))


def _gtest_headers(path: str, root: Path) -> list[str] | None:
    if path.startswith("src/"):
        return headers_for_source(root, path)
    if path.startswith("include/") or path.startswith("tests/"):
        return [path]
    return None


def _select_gtest(path: str, root: Path, index: GtestIndex, scope: Scope) -> bool:
    if path.startswith(GTEST_EVERYTHING):
        scope.add_everything()
        return True
    if Path(path).suffix not in CPP_SUFFIXES or is_pytest_file(path):
        return False
    if path.startswith("tests/") and path.endswith(".cpp"):
        for suite in index.suites_of([path]):
            scope.add(suite)
        return True
    headers = _gtest_headers(path, root)
    if headers is None:
        return False
    tests = index.tests_including(headers) if index.available else set()
    if not tests:
        scope.add_everything()
        return True
    for suite in index.suites_of(tests):
        scope.add(suite)
    return True


def select(paths: Iterable[str], root: Path) -> Selection:
    """Suites and cases affected by a list of repo-relative paths."""
    selection = Selection()
    scopes = selection.scopes
    gtest_index = GtestIndex(root)
    python_index = PythonIndex(root)
    for path in sorted(set(paths)):
        if is_ignored(path):
            continue
        covered = _select_gtest(path, root, gtest_index, scopes["gtest"])
        if path.startswith(FIRMWARE_NATIVE):
            scopes["firmware"].add_everything()
            covered = True
        if path.startswith(WEB):
            scopes["web"].add_everything()
            covered = covered or path.startswith("web/")
        if path.startswith(SYNTHETIC):
            scopes["synthetic"].add_everything()
            covered = True
        if path in PYTEST_EVERYTHING:
            scopes["pytest"].add_everything()
            covered = True
        else:
            tests = python_index.tests_covering(path)
            for test in tests:
                scopes["pytest"].add(test)
            covered = covered or bool(tests)
        if not covered and Path(path).suffix in CODE_SUFFIXES and (root / path).exists():
            selection.uncovered.append(path)
    return selection
